//==============================================================================
// o6_csr.sv - CSR 寄存器堆、陷入、中断仲裁、计数器
//
// 三个入口：
//   1) 执行口 (ROB 头的 FU_CSR 指令，组合返回)：CSR 读改写、系统指令合法性检查、
//      MRET/SRET 的状态切换 (在执行时完成；指令已启动就必须提交，中断不会插进来)
//   2) 陷入口 (commit_ctrl)：写 xepc/xcause/xtval/xstatus/priv，输出陷入向量
//   3) 提交口：minstret / fflags / FS 脏位
//   4) 调试口：进入调试模式 (commit_ctrl)；DRET 在执行口完成退出
//
// 调试模式 (Debug 0.13.2)：dcsr / dpc / dscratch0-1 只在调试模式可访问；调试模式下按 M 态运行，
//   不响应中断，MPRV 不生效 (mprven = 0)；单步 (dcsr.step) 且 stepie = 0 时不响应中断。
//   stopcount / stoptime 恒 0 (计数器与 mtime 照常走)。
//
// H 扩展 (RVA23 R5，HAS_H = HAS_MMU)：V 位 + HS / VS 级 CSR；各 CSR 的可写掩码、访问权限 (illegal / virtual 的判定次序)、
//   陷入路由与写入字段、中断三组仲裁、xRET 都按 docs/15 (Spike fdc1ffa 的实际代码) 实现。要点：
//   - V = 1 时 S 级地址 sstatus / sie / stvec / sscratch / sepc / scause / stval / sip / satp / stimecmp 改访问 vs* (权限按原地址)；
//     senvcfg / scounteren / sstateen / scountovf 不重定向
//   - 每次陷入 HS / M 都写 htval / htinst (mtval2 / mtinst) = tval2 / tinst (通常 0)；陷入 VS 不动这四个
//   - 中断按 M 组 > HS 组 > VS 组选，组内 MEI MSI MTI SEI SSI STI SGEI VSEI VSSI VSTI LCOFI
//   - 翻译上下文 (取指 / 普通访存 / HLV-HSV) 在这里算好输出 (xctx_t)
//
// 与 Spike 对齐的约定 (逐条在对拍中验证，差异记录在 docs/04_cosim.md)：
//   - mtvec/stvec/vstvec 写入时清 bit1 (MODE 只能 0/1)
//   - mepc/sepc/vsepc 写入时清 bit0 (有 C 扩展)
//   - MPP 写 2 (保留) 视为 U
//   - 写 minstret 后，该指令提交时的 +1 不再生效 (写 val-1，提交时 +1)
//   - satp 写入不支持的 MODE 时整个写入被忽略 (V = 1 经 satp 写 vsatp 也一样)；直接写 vsatp / hgatp 时 MODE 保持、其余照写
//   - mhpmcounter3..6 计数 (对拍时把读值注入 Spike)，7..31 恒 0；写 const 计数器被忽略
//==============================================================================
`default_nettype none
module o6_csr
  import o6_pkg::*;
  import o6_isa_pkg::*;
#(
  parameter bit HAS_FPU = 1'b0,
  parameter bit HAS_MMU = 1'b0
) (
  input  wire              clk_i,
  input  wire              rst_ni,

  //---------------- 执行口
  input  wire              ex_req_i,
  input  var uop_t         ex_uop_i,
  input  wire  [63:0]      ex_wdata_i,
  output logic             ex_done_o,
  output logic [63:0]      ex_rdata_o,
  output logic             ex_excp_o,
  output logic [5:0]       ex_cause_o,      // 非法指令 (2) 或 virtual instruction (22)
  output logic [63:0]      ex_tval_o,

  //---------------- 陷入口
  input  wire              trap_i,
  input  wire              trap_irq_i,
  input  wire  [5:0]       trap_cause_i,
  input  wire  [63:0]      trap_epc_i,
  input  wire  [63:0]      trap_tval_i,
  input  wire  [63:0]      trap_tval2_i,    // H：guest page fault 的 GPA >> 2 (其它陷入 0)
  input  wire  [63:0]      trap_tinst_i,    // H：0 / 0x3000 (VS-stage 读 PTE 引起的 guest page fault)
  input  wire              trap_hv_i,       // 出错的是 HLV / HSV (有效 V = 1，GVA 用)
  output logic [63:0]      trap_vec_o,      // 组合：按当前状态算出的陷入入口

  //---------------- 提交口
  input  wire  [2:0]       cm_cnt_i,        // 本拍提交条数
  input  wire  [4:0]       cm_fflags_i,
  input  wire              cm_fp_dirty_i,

  //---------------- 调试
  input  wire              dbg_enter_i,     // 进入调试模式 (commit_ctrl)
  input  wire  [2:0]       dbg_cause_i,     // dcsr.cause：1 ebreak，3 haltreq，4 step
  input  wire  [63:0]      dbg_pc_i,        // 写入 dpc
  output logic             dbg_mode_o,
  output logic [63:0]      dpc_o,
  output logic             dbg_step_o,      // dcsr.step 且不在调试模式
  output logic             ebreak_dbg_o,    // 当前特权级的 ebreak 进调试模式

  //---------------- 中断
  input  wire              mtip_i,
  input  wire              msip_i,
  input  wire              meip_i,
  input  wire              seip_i,
  input  wire  [63:0]      mtime_i,
  output logic             irq_pending_o,
  output logic [5:0]       irq_cause_o,

  //---------------- 状态输出
  output priv_e            priv_o,
  output logic             virt_o,          // H：V 位
  output logic             hlv_ok_o,        // HLV / HSV 可执行 (有 H、V = 0、S 及以上或 hstatus.HU)
  output logic             hlv_vi_o,        // 不可执行时报 virtual instruction (V = 1；HU 无关)
  output logic [1:0]       fs_o,            // 有效 FS：V = 1 时 vsstatus.FS = Off 也算 Off (译码判非法)
  output logic [1:0]       vs_o,            // 有效 VS (同上)
  input  wire              vs_dirty_i,      // 向量单元收下会改向量状态的指令：VS 置 Dirty
  input  wire  [4:0]       vfflags_i,       // 向量单元浮点运算的异常标志 (执行时随到随并；非零时 FS 置 Dirty)
  output logic [2:0]       frm_o,
  output xctx_t            ictx_o,          // 取指的翻译上下文
  output xctx_t            dctx_o,          // 普通数据访存 (含 MPRV / MPV)
  output xctx_t            hctx_o,          // HLV / HSV (V = 1，特权级 = hstatus.SPVP)
  output logic [63:0]      satp_o,
  output logic [63:0]      vsatp_o,
  output logic [63:0]      hgatp_o,
  output logic             satp_wr_o,       // satp / vsatp / hgatp 被写 (清 TLB)
  output logic [7:0]       pmpcfg_o  [8],
  output logic [53:0]      pmpaddr_o [8],
  output logic [63:0]      mepc_o,
  output logic [63:0]      sepc_o,          // SRET 的返回地址 (执行 SRET 时锁存：V = 1 取 vsepc，否则 sepc)
  output logic             pbmte_o,         // menvcfg.PBMTE (Svpbmt，V = 0 的页表与 G-stage)
  output logic             vpbmte_o,        // henvcfg.PBMTE 读值 (VS-stage)
  output logic [2:0]       cbo_ok_o,        // Zicbom / Zicboz 在当前特权级可执行：{cbo.zero, cbo.clean / flush, cbo.inval} (译码判非法用)
  output logic [2:0]       cbo_vi_o,        // 不可执行时是 virtual instruction (否则非法指令)

  // 性能事件 (mhpmcounter3..6 的事件源)
  input  wire  [7:0]       hpm_event_i      // 事件在 CSR 里寄存一拍再计数 (来自核内各处的组合信号)
);
  localparam bit HAS_H = HAS_MMU;

  //--------------------------------------------------------------------------
  // 寄存器
  //--------------------------------------------------------------------------
  priv_e        priv;
  logic         virt;          // H：V 位
  // mstatus 字段
  logic         st_sie, st_mie, st_spie, st_mpie, st_spp, st_mprv, st_sum, st_mxr, st_tvm, st_tw, st_tsr;
  logic         st_mpv, st_gva;
  logic [1:0]   st_mpp, st_fs, st_vs;
  logic [63:0]  medeleg, mideleg, mie, mip_sw;   // mip_sw：软件可写的 SSIP/VSSIP/STIP/SEIP/LCOFIP
  logic [63:0]  mtvec, stvec, mscratch, sscratch, mepc, sepc, mcause, scause, mtval, stval, satp;
  logic [63:0]  sret_pc;
  logic [31:0]  mcounteren, scounteren;
  logic [31:0]  mcountinhibit;
  logic         mci_ir_eff, mci_pend;   // IR 禁止位的计数效果：写它的那条指令按旧值计 (与 Spike 一致)
  logic [63:0]  mcycle, minstret;
  logic [7:0]   hpm_ev_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) hpm_ev_q <= '0;
    else         hpm_ev_q <= hpm_event_i;
  // 提交数来自核里寄存的提交包 (本拍末才加进 minstret)；读时补上
  wire  [2:0]   cm_cnt_eff = mci_ir_eff ? 3'd0 : cm_cnt_i;
  logic [63:0]  hpmcnt [4];
  logic [7:0]   hpmevt [4];
  // Sscofpmf (RVA23 R3)：mhpmevent 的溢出位与按特权级的抑制位 (H：再加 VSINH / VUINH)
  logic [3:0]   hpm_of, hpm_minh, hpm_sinh, hpm_uinh, hpm_vsinh, hpm_vuinh;
  logic [4:0]   fflags;
  logic [2:0]   frm;
  logic         menvcfg_fiom, senvcfg_fiom;
  // Zicbom / Zicboz (RVA23 R2)：xenvcfg 的 CBZE (7) / CBCFE (6) / CBIE (5:4，保留值 2 写入当 0，与 Spike 一致)
  logic         menvcfg_cbze, menvcfg_cbcfe, senvcfg_cbze, senvcfg_cbcfe;
  logic [1:0]   menvcfg_cbie, senvcfg_cbie;
  // Sstc (RVA23 R3)：menvcfg.STCE 打开后 mip.STIP 由 mtime >= stimecmp 产生 (寄存一拍)，软件写不进
  logic         menvcfg_stce;
  logic         menvcfg_pbmte;      // Svpbmt (RVA23 R4)
  logic [1:0]   menvcfg_pmm, senvcfg_pmm;   // Smnpm / Ssnpm：0 关，2 = PMLEN 7，3 = PMLEN 16 (写 1 保留值当 0，同 Spike)
  logic [63:0]  stimecmp;
  logic         stip_hw;
  // Smstateen / Ssstateen (RVA23 R3)：mstateen0 可写 SE0 (63) / ENVCFG (62) / PRIV113 (56，H) / C (0)，mstateen1..3 只有 SE (63)；
  // sstateen0 只有 C (0)，读写都按 mstateen0 (V = 1 再按 hstateen0) 屏蔽；sstateen1..3 恒 0
  logic         m0_se, m0_envcfg, m0_p113, m0_c, s0_c;
  logic [3:1]   m_se;
  logic [7:0]   pmpcfg [8];
  logic [53:0]  pmpaddr [8];
  logic [13:0]  hcontext;      // 调试上下文：mcontext (0x7A8) 是它的别名 (Spike：有 H 时 14 位，否则 13 位)
  logic [31:0]  scontext;
  // H 扩展
  logic         hs_gva, hs_spv, hs_spvp, hs_hu, hs_vtvm, hs_vtw, hs_vtsr;
  logic [1:0]   hs_hupmm;
  logic [63:0]  hedeleg, hideleg;
  logic         hvip_vstip, hvip_vseip;
  logic [63:0]  htval, htinst, mtval2, mtinst, htimedelta;
  logic [31:0]  hcounteren;
  logic         henvcfg_fiom, henvcfg_cbze, henvcfg_cbcfe, henvcfg_pbmte, henvcfg_stce;
  logic [1:0]   henvcfg_cbie, henvcfg_pmm;
  logic         h0_se, h0_envcfg, h0_c;
  logic [3:1]   h_se;
  logic [3:0]   hgatp_mode;
  logic [13:0]  hgatp_vmid;
  logic [43:2]  hgatp_ppn;
  // vsstatus 字段 + VS CSR
  logic         vs_sie, vs_spie, vs_spp, vs_sum, vs_mxr;
  logic [1:0]   vs_fs, vs_vs;
  logic [63:0]  vstvec, vsscratch, vsepc, vscause, vstval, vsatp, vstimecmp;
  logic         vstip_hw;
  logic [63:0]  mt_hd;         // mtime + htimedelta (寄存一拍：V = 1 的 time 读值与 VSTIP 比较)
  // 调试
  logic         dbg_mode;
  logic         d_ebreakm, d_ebreaks, d_ebreaku, d_ebreakvs, d_ebreakvu, d_stepie, d_step, d_v;
  logic [2:0]   d_cause;
  logic [1:0]   d_prv;
  logic [63:0]  dpc, dscratch0, dscratch1;
  wire  [63:0]  dcsr = {32'b0, 4'd4 /*xdebugver*/, 10'b0, d_ebreakvs, d_ebreakvu, d_ebreakm, 1'b0, d_ebreaks, d_ebreaku,
                        d_stepie, 1'b0 /*stopcount*/, 1'b0 /*stoptime*/, d_cause, d_v, 1'b0 /*mprven*/,
                        1'b0 /*nmip*/, d_step, d_prv};

`ifdef O6_NO_VEC
  localparam logic HAS_VEC = 1'b0;
`else
  localparam logic HAS_VEC = 1'b1;
`endif
  localparam logic [63:0] MISA = {2'b10, 36'b0,
                                  1'b0, 1'b0, 1'b0, 1'b0, HAS_VEC /*V*/, 1'b1 /*U*/, 1'b0, 1'b1 /*S*/, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
                                  1'b1 /*M*/, 1'b0, 1'b0, 1'b0, 1'b1 /*I*/, HAS_H /*H*/, 1'b0, HAS_FPU /*F*/, 1'b0, HAS_FPU /*D*/,
                                  1'b1 /*C*/, 1'b1 /*B：Zba+Zbb+Zbs 齐备 (B 扩展 1.0)*/, 1'b1 /*A*/};

  // medeleg：H 时再加 VS ecall (10) 与 guest page fault / virtual instruction (20..23)
  localparam logic [63:0] MEDELEG_MASK = (HAS_H ? 64'h0000_0000_00F8_B7FE : 64'h0000_0000_0008_B3FE) &
                                         (HAS_MMU ? 64'hFFFF_FFFF_FFFF_FFFF : ~64'h0000_0000_0000_B000);
  localparam logic [63:0] HEDELEG_MASK = 64'h0000_0000_0008_B1FF;   // bit 0..8, 12, 13, 15, 19 (Spike)
  // bit 13 = LCOFIP (Sscofpmf 计数器溢出中断)：可委托、可使能、M / S (委托后) 软件可写，计数器溢出时硬件置位
  localparam logic [63:0] MIDELEG_MASK = 64'h2222;
  localparam logic [63:0] MIDELEG_RO1  = HAS_H ? 64'h1444 : 64'h0;  // VSSI / VSTI / VSEI / SGEI：mideleg 读恒 1
  localparam logic [63:0] HS_IRQ       = 64'h1444;
  localparam logic [63:0] VS_IRQ       = 64'h0444;
  localparam logic [63:0] MIE_MASK     = HAS_H ? 64'h3EEE : 64'h2AAA;
  localparam logic [63:0] MIP_SW_MASK  = HAS_H ? 64'h2226 : 64'h2222;    // + VSSIP (STIP 在 STCE = 1 时不可写，见写入处)
  localparam logic [63:0] SIP_SW_MASK  = 64'h2002;    // S 态可写 SSIP / LCOFIP

  wire        stce_h = menvcfg_stce & henvcfg_stce;                 // henvcfg.STCE 读值
  wire        vstip  = hvip_vstip | (stce_h & vstip_hw);
  wire [63:0] mip = (mip_sw & ~{58'b0, menvcfg_stce, 5'b0}) |
                    {52'b0, meip_i, hvip_vseip, (mip_sw[9] | seip_i), 1'b0, mtip_i, vstip, menvcfg_stce & stip_hw, 1'b0, msip_i, 3'b0};
  // 注：SEIP 读值 = 软件位 | 外部线；写只改软件位。STCE = 1 时 STIP 只来自 stimecmp 比较
  wire [63:0] mideleg_rd = mideleg | MIDELEG_RO1;

  // 读 mstatus / sstatus 时 FS 要算上提交包里还没落地的"浮点状态变脏"
  wire [1:0]  fs_rd = cm_fp_dirty_i ? 2'b11 : st_fs;
  wire        sd = (fs_rd == 2'b11) || (st_vs == 2'b11);
  wire [63:0] mstatus = {sd, 23'b0, st_mpv, st_gva, 2'b00, 2'b10 /*SXL*/, 2'b10 /*UXL*/, 9'b0,
                         st_tsr, st_tw, st_tvm, st_mxr, st_sum, st_mprv, 2'b00 /*XS*/, fs_rd,
                         st_mpp, st_vs, st_spp, st_mpie, 1'b0 /*UBE*/, st_spie, 1'b0, st_mie, 1'b0, st_sie, 1'b0};
  localparam logic [63:0] SSTATUS_MASK = 64'h8000_0003_000D_E762;
  wire [63:0] sstatus = mstatus & SSTATUS_MASK;
  wire [1:0]  vfs_rd  = (cm_fp_dirty_i && virt) ? 2'b11 : vs_fs;
  wire        vsd     = (vfs_rd == 2'b11) || (vs_vs == 2'b11);
  wire [63:0] vsstatus = {vsd, 29'b0, 2'b10 /*UXL*/, 12'b0, vs_mxr, vs_sum, 1'b0, 2'b00, vfs_rd, 2'b00, vs_vs, vs_spp,
                          2'b00, vs_spie, 3'b000, vs_sie, 1'b0};
  wire [63:0] hstatus = {14'b0, hs_hupmm, 14'b0, 2'b10 /*VSXL*/, 9'b0, hs_vtsr, hs_vtw, hs_vtvm, 2'b0, 6'b0 /*VGEIN*/,
                         2'b0, hs_hu, hs_spvp, hs_spv, hs_gva, 1'b0 /*VSBE*/, 5'b0};
  wire [63:0] hgatp   = {hgatp_mode, 2'b00, hgatp_vmid, hgatp_ppn, 2'b00};
  wire [63:0] henvcfg = {stce_h, menvcfg_pbmte & henvcfg_pbmte, 28'b0, henvcfg_pmm, 24'b0,
                         henvcfg_cbze, henvcfg_cbcfe, henvcfg_cbie, 3'b0, henvcfg_fiom};

  //--------------------------------------------------------------------------
  // 执行口：读 + 合法性
  //--------------------------------------------------------------------------
  wire [11:0] ca   = ex_uop_i.csr_addr;
  wire [4:0]  op   = ex_uop_i.op;
  wire        is_csr = (op[2:0] != 3'b000) && (op[4:3] == 2'b00);   // CSR_RW..CSR_RCI
  // 是否写：RW/RWI 总写；RS/RC 在 rs1 (或 zimm) 非零时写
  wire        do_wr  = (op[1:0] == 2'b01) || (ex_uop_i.lrs1 != 5'd0);
  // V = 1 时 S 级地址重定向到 vs* (权限仍按原地址判)
  logic [11:0] ra;
  always_comb begin
    ra = ca;
    if (HAS_H && virt)
      unique case (ca)
        CSR_SSTATUS, CSR_SIE, CSR_STVEC, CSR_SSCRATCH, CSR_SEPC, CSR_SCAUSE, CSR_STVAL, CSR_SIP, CSR_SATP, CSR_STIMECMP:
          ra = {4'h2, ca[7:0]};
        default: ;
      endcase
  end

  logic        exists;
  logic [63:0] rval;

  always_comb begin
    exists = 1'b1;
    rval   = 64'd0;
    // priority：mhpmcounter / hpmcounter / mhpmevent 的范围项与其中的具体地址 (cycle、mcycle、0xB01 等) 重叠，
    // 具体项必须优先 (0xB01 / 0x321 / 0x322 靠它报不存在)。写成 unique 时综合按并行 case 处理会出错 (Verilator 查出)
    priority casez (ra)
      CSR_FFLAGS:     begin rval = {59'b0, fflags | cm_fflags_i};       exists = HAS_FPU; end
      CSR_FRM:        begin rval = {61'b0, frm};          exists = HAS_FPU; end
      CSR_FCSR:       begin rval = {56'b0, frm, fflags | cm_fflags_i};  exists = HAS_FPU; end
      CSR_CYCLE:      rval = mcycle;
      CSR_TIME:       rval = virt ? mt_hd : mtime_i;
      CSR_INSTRET:    rval = minstret + 64'(cm_cnt_eff);
      12'hC03, 12'hC04, 12'hC05, 12'hC06:
                      rval = hpmcnt[ra[1:0] - 2'd3];
      12'b1100_000?_????:  // hpmcounter7..31
                      rval = 64'd0;
      CSR_SSTATUS:    rval = sstatus;
      CSR_SIE:        rval = mie & mideleg;
      CSR_STVEC:      rval = stvec;
      CSR_SCOUNTEREN: rval = {32'b0, scounteren};
      CSR_SENVCFG:    rval = {30'b0, senvcfg_pmm, 24'b0, senvcfg_cbze, senvcfg_cbcfe, senvcfg_cbie, 3'b0, senvcfg_fiom};
      CSR_SSCRATCH:   rval = sscratch;
      CSR_SEPC:       rval = sepc;
      CSR_SCAUSE:     rval = scause;
      CSR_STVAL:      rval = stval;
      CSR_SIP:        rval = mip & mideleg;
      CSR_SATP:       rval = satp;
      CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID, CSR_MHARTID, CSR_MCONFIGPTR:
                      rval = 64'd0;
      CSR_MSTATUS:    rval = mstatus;
      CSR_MISA:       rval = MISA;
      CSR_MEDELEG:    rval = medeleg;
      CSR_MIDELEG:    rval = mideleg_rd;
      CSR_MIE:        rval = mie;
      CSR_MTVEC:      rval = mtvec;
      CSR_MCOUNTEREN: rval = {32'b0, mcounteren};
      CSR_MENVCFG:    rval = {menvcfg_stce, menvcfg_pbmte, 28'b0, menvcfg_pmm, 24'b0, menvcfg_cbze, menvcfg_cbcfe, menvcfg_cbie, 3'b0, menvcfg_fiom};
      // RVA23 R3
      CSR_STIMECMP:   rval = stimecmp;                                   // Sstc stimecmp
      12'hDA0:        begin                                              // Sscofpmf scountovf (只读)
                        rval = {57'b0, hpm_of, 3'b0};
                        if (priv != PRV_M) rval = rval & {32'b0, mcounteren};
                        if (virt)          rval = rval & {32'b0, hcounteren};
                      end
      12'h30C:        rval = {m0_se, m0_envcfg, 5'b0, m0_p113 & HAS_H, 55'b0, m0_c};   // mstateen0
      12'h30D, 12'h30E, 12'h30F: rval = {m_se[ra[1:0]], 63'b0};          // mstateen1..3
      12'h10C:        rval = {63'b0, s0_c & m0_c & (!virt || h0_c)};    // sstateen0
      12'h10D, 12'h10E, 12'h10F: rval = 64'd0;                           // sstateen1..3
      // H：HS 级
      CSR_HSTATUS:    begin rval = hstatus;                     exists = HAS_H; end
      CSR_HEDELEG:    begin rval = hedeleg;                     exists = HAS_H; end
      CSR_HIDELEG:    begin rval = hideleg;                     exists = HAS_H; end
      CSR_HIE:        begin rval = mie & HS_IRQ;                exists = HAS_H; end
      CSR_HTIMEDELTA: begin rval = htimedelta;                  exists = HAS_H; end
      CSR_HCOUNTEREN: begin rval = {32'b0, hcounteren};         exists = HAS_H; end
      CSR_HGEIE:      begin rval = 64'd0;                       exists = HAS_H; end
      CSR_HENVCFG:    begin rval = henvcfg;                     exists = HAS_H; end
      12'h60C:        begin rval = {h0_se & m0_se, h0_envcfg & m0_envcfg, 61'b0, h0_c & m0_c}; exists = HAS_H; end
      12'h60D, 12'h60E, 12'h60F:
                      begin rval = {h_se[ra[1:0]] & m_se[ra[1:0]], 63'b0}; exists = HAS_H; end
      CSR_HTVAL:      begin rval = htval;                       exists = HAS_H; end
      CSR_HIP:        begin rval = mip & HS_IRQ;                exists = HAS_H; end
      CSR_HVIP:       begin rval = {53'b0, hvip_vseip, 3'b0, hvip_vstip, 3'b0, mip_sw[2], 2'b0}; exists = HAS_H; end
      CSR_HTINST:     begin rval = htinst;                      exists = HAS_H; end
      CSR_HGATP:      begin rval = hgatp;                       exists = HAS_H; end
      CSR_HCONTEXT:   begin rval = {50'b0, hcontext};           exists = HAS_H; end
      CSR_HGEIP:      begin rval = 64'd0;                       exists = HAS_H; end
      // H：VS 级
      CSR_VSSTATUS:   begin rval = vsstatus;                    exists = HAS_H; end
      CSR_VSIE:       begin rval = (mie & hideleg & VS_IRQ) >> 1; exists = HAS_H; end
      CSR_VSTVEC:     begin rval = vstvec;                      exists = HAS_H; end
      CSR_VSSCRATCH:  begin rval = vsscratch;                   exists = HAS_H; end
      CSR_VSEPC:      begin rval = vsepc;                       exists = HAS_H; end
      CSR_VSCAUSE:    begin rval = vscause;                     exists = HAS_H; end
      CSR_VSTVAL:     begin rval = vstval;                      exists = HAS_H; end
      CSR_VSIP:       begin rval = (mip & hideleg & VS_IRQ) >> 1; exists = HAS_H; end
      CSR_VSTIMECMP:  begin rval = vstimecmp;                   exists = HAS_H; end
      CSR_VSATP:      begin rval = vsatp;                       exists = HAS_H; end
      CSR_MTINST:     begin rval = mtinst;                      exists = HAS_H; end
      CSR_MTVAL2:     begin rval = mtval2;                      exists = HAS_H; end
      CSR_MCOUNTINHIBIT: rval = {32'b0, mcountinhibit};
      CSR_MSCRATCH:   rval = mscratch;
      CSR_MEPC:       rval = mepc;
      CSR_MCAUSE:     rval = mcause;
      CSR_MTVAL:      rval = mtval;
      CSR_MIP:        rval = mip;
      CSR_PMPCFG0:    rval = {pmpcfg[7], pmpcfg[6], pmpcfg[5], pmpcfg[4], pmpcfg[3], pmpcfg[2], pmpcfg[1], pmpcfg[0]};
      // pmpcfg2..14 (RV64 只有偶数号)：第 8 项以后未实现，读 0
      12'h3A2, 12'h3A4, 12'h3A6, 12'h3A8, 12'h3AA, 12'h3AC, 12'h3AE: rval = 64'd0;
      // pmpaddr0..15 (8..15 恒 0)；粒度 64 B 的读值：NAPOT 低 G-1 位读 1，OFF / TOR 低 G 位读 0 (存储值不变)
      12'h3B?:        rval = (ra[3] == 1'b0) ?
                             (pmpcfg[ra[2:0]][4] ? {10'b0, pmpaddr[ra[2:0]] | 54'((1 << (PMP_G - 1)) - 1)}
                                                 : {10'b0, pmpaddr[ra[2:0]] & ~54'((1 << PMP_G) - 1)}) : 64'd0;
      12'h3C?, 12'h3D?, 12'h3E?: rval = 64'd0;                                    // pmpaddr16..63 未实现
      CSR_MCYCLE:     rval = mcycle;
      CSR_MINSTRET:   rval = minstret + 64'(cm_cnt_eff);
      12'hB01, 12'h321, 12'h322: exists = 1'b0;   // 落在下面范围里但不存在的地址
      // 触发器：0 个 (tselect 写入被忽略、tdata/tinfo 恒 0；tcontrol 不存在)
      12'h7A0, 12'h7A1, 12'h7A2, 12'h7A3, 12'h7A4: rval = 64'd0;
      12'h7A8:        rval = HAS_H ? {50'b0, hcontext} : {51'b0, hcontext[12:0]};
      12'h5A8:        rval = {32'b0, scontext};
      CSR_DCSR:       begin rval = dcsr;      exists = dbg_mode; end
      CSR_DPC:        begin rval = dpc;       exists = dbg_mode; end
      CSR_DSCRATCH0:  begin rval = dscratch0; exists = dbg_mode; end
      CSR_DSCRATCH1:  begin rval = dscratch1; exists = dbg_mode; end
      12'hB03, 12'hB04, 12'hB05, 12'hB06:
                      rval = hpmcnt[ra[1:0] - 2'd3];
      12'b1011_000?_????: rval = 64'd0;           // mhpmcounter7..31
      12'h323, 12'h324, 12'h325, 12'h326: begin
                        logic [1:0] k;
                        k = ra[1:0] - 2'd3;
                        // {OF, MINH, SINH, UINH, VSINH, VUINH, ..., 事件号}
                        rval = {hpm_of[k], hpm_minh[k], hpm_sinh[k], hpm_uinh[k], hpm_vsinh[k], hpm_vuinh[k], 50'b0, hpmevt[k]};
                      end
      12'b0011_001?_????: rval = 64'd0;           // mhpmevent7..31
      default:        exists = 1'b0;
    endcase
  end

  // 合法性：0 = 合法，1 = 非法指令，2 = virtual instruction。判定次序同 Spike (docs/15 §3、§4)
  localparam logic [1:0] OK = 2'd0, ILL = 2'd1, VIR = 2'd2;
  // 计数器 (cycle / time / instret / hpmcounterN 的用户态映像)：mcounteren -> hcounteren -> scounteren
  function automatic logic [1:0] ctr_chk(input logic [4:0] n, input priv_e p, input logic v,
                                         input logic [31:0] mce, input logic [31:0] hce, input logic [31:0] sce);
    if (p != PRV_M && !mce[n]) return ILL;
    if (v && !hce[n])          return VIR;
    if (p == PRV_U && !sce[n]) return v ? VIR : ILL;
    return OK;
  endfunction
  // 有效特权等级：M 3、HS 2、VS 1、U / VU 0 (Spike csr_t::verify_permissions)
  wire [1:0] pe = (priv == PRV_M) ? 2'd3 : (priv == PRV_S) ? (virt ? 2'd1 : 2'd2) : 2'd0;

  logic [1:0]  chk;
  logic [31:0] raw;
  always_comb begin
    logic [1:0] gen;
    chk = OK;
    raw = 32'd0;
    // 通用检查：不存在 / 写只读 -> 非法 (V = 1 也是)；特权不够：V = 1 且 CSR 级 <= HS -> virtual，否则非法
    gen = OK;
    if (!exists)                               gen = ILL;
    else if (do_wr && ca[11:10] == 2'b11)      gen = ILL;
    else if (pe < ca[9:8])                     gen = (virt && ca[9:8] <= 2'd2) ? VIR : ILL;
    if (is_csr) begin
      raw = {ca, ex_uop_i.lrs1, op[2:0], ex_uop_i.lrd, OPC_SYSTEM};
      chk = gen;
      if (ca[11:5] == 7'b1100_000) begin                             // cycle / time / instret / hpmcounterN
        if (gen == OK) chk = ctr_chk(ca[4:0], priv, virt, mcounteren, hcounteren, scounteren);
      end else if (ca == CSR_STIMECMP || ca == CSR_VSTIMECMP) begin
        if (priv != PRV_M && !menvcfg_stce)                         chk = ILL;
        else if (ctr_chk(5'd1, priv, virt, mcounteren, hcounteren, scounteren) != OK)
                                                                    chk = ctr_chk(5'd1, priv, virt, mcounteren, hcounteren, scounteren);
        else if (virt && !stce_h)                                   chk = VIR;
        else                                                        chk = gen;
      end else if (ca == 12'h60C || ca == 12'h60D || ca == 12'h60E || ca == 12'h60F) begin   // hstateenN
        if (HAS_H && priv != PRV_M && !((ca[1:0] == 2'd0) ? m0_se : m_se[ca[1:0]])) chk = ILL;
      end else if (ca == 12'h10C || ca == 12'h10D || ca == 12'h10E || ca == 12'h10F) begin   // sstateenN
        if (priv != PRV_M && !((ca[1:0] == 2'd0) ? m0_se : m_se[ca[1:0]])) chk = ILL;
        else if (gen != OK)                                         chk = gen;
        else if (virt && !((ca[1:0] == 2'd0) ? h0_se : h_se[ca[1:0]])) chk = VIR;
      end else if (ca == CSR_HENVCFG) begin
        if (HAS_H && priv != PRV_M && !m0_envcfg)                   chk = ILL;
      end else if (ca == CSR_SENVCFG) begin
        if (priv != PRV_M && !m0_envcfg)                            chk = ILL;
        else if (virt && !h0_envcfg)                                chk = VIR;
      end else if (ca == CSR_SATP) begin
        if (gen == OK && !virt && priv == PRV_S && st_tvm)          chk = ILL;
        if (gen == OK && virt && hs_vtvm)                           chk = VIR;
      end else if (ca == CSR_HGATP) begin
        if (gen == OK && !virt && priv == PRV_S && st_tvm)          chk = ILL;
      end else if (ca == CSR_FFLAGS || ca == CSR_FRM || ca == CSR_FCSR) begin
        if (gen == OK && (st_fs == 2'b00 || (virt && vs_fs == 2'b00))) chk = ILL;
      end
    end else begin
      unique case (op)
        SYS_MRET: begin
          raw = 32'h3020_0073;
          chk = (priv != PRV_M) ? ILL : OK;
        end
        SYS_SRET: begin
          raw = 32'h1020_0073;
          if (virt)                          chk = (priv == PRV_U || hs_vtsr) ? VIR : OK;
          else                               chk = (priv == PRV_U || (priv == PRV_S && st_tsr)) ? ILL : OK;
        end
        SYS_DRET: begin
          raw = 32'h7b20_0073;
          chk = !dbg_mode ? ILL : OK;
        end
        SYS_WFI: begin
          raw = 32'h1050_0073;
          if (st_tw)                         chk = (priv != PRV_M) ? ILL : OK;
          else if (virt)                     chk = (priv == PRV_U || hs_vtw) ? VIR : OK;
          else                               chk = (priv == PRV_U) ? ILL : OK;
        end
        SYS_WRS: begin                         // Zawrs wrs.nto：TW = 1 时非 M 态非法；V = 1 且 VTW -> virtual (U / VU 在 TW = VTW = 0 时合法)
          raw = 32'h00d0_0073;
          if (st_tw)                         chk = (priv != PRV_M) ? ILL : OK;
          else if (virt && hs_vtw)           chk = VIR;
        end
        SYS_SFENCE: begin                      // sfence.vma / sinval.vma (Svinval)：tval 用原始编码
          raw = ex_uop_i.raw;
          if (virt)                          chk = (priv == PRV_U || hs_vtvm) ? VIR : OK;
          else                               chk = (priv == PRV_U || (priv == PRV_S && st_tvm)) ? ILL : OK;
        end
        SYS_INVF: begin                        // Svinval sfence.w.inval / sfence.inval.ir：只要求 S 及以上 (VU -> virtual)
          raw = ex_uop_i.raw;
          if (priv == PRV_U)                 chk = virt ? VIR : ILL;
        end
        SYS_HFENCEV: begin                     // hfence.vvma / hinval.vvma：V = 1 -> virtual；U -> 非法 (HU 无关)
          raw = ex_uop_i.raw;
          if (!HAS_H)                        chk = ILL;
          else if (virt)                     chk = VIR;
          else if (priv == PRV_U)            chk = ILL;
        end
        SYS_HFENCEG: begin                     // hfence.gvma / hinval.gvma：再加 TVM = 1 时要 M
          raw = ex_uop_i.raw;
          if (!HAS_H)                        chk = ILL;
          else if (virt)                     chk = VIR;
          else if (priv == PRV_U || (priv == PRV_S && st_tvm)) chk = ILL;
        end
        default: chk = OK;   // FENCE / FENCE.I
      endcase
    end
  end
  wire ill = (chk != OK);

  assign ex_done_o  = ex_req_i;
  assign ex_rdata_o = rval;
  assign ex_excp_o  = ill;
  assign ex_cause_o = (chk == VIR) ? EXC_VIRT_INST : EXC_ILLEGAL_INST;
  assign ex_tval_o  = {32'b0, raw};

  // 写入值
  //   mip / sip 的读改写 (csrrs / csrrc) 里 SEIP 只用软件位 (特权规范 3.1.9：忽略外部中断线)；
  //   否则外部 SEIP 挂着时一次 csrs mip 就把它锁进软件位，设备清了中断也清不掉 (#30，UVM o6_tmr_test 查出)
  logic [63:0] wv, rval_rmw;
  always_comb begin
    rval_rmw = rval;
    if (ra == CSR_MIP || ra == CSR_SIP) rval_rmw[9] = mip_sw[9] & ((ra == CSR_MIP) | mideleg[9]);
    unique case (op[1:0])
      2'b01:   wv = ex_wdata_i;                 // RW
      2'b10:   wv = rval_rmw | ex_wdata_i;      // RS
      2'b11:   wv = rval_rmw & ~ex_wdata_i;     // RC
      default: wv = rval_rmw;
    endcase
  end
  wire wr = ex_req_i && is_csr && do_wr && !ill;

  //--------------------------------------------------------------------------
  // 中断仲裁
  //--------------------------------------------------------------------------
  // 仲裁结果寄存一拍 (别让中断判定串进提交级 -> LSU -> D$ 的路径)。
  // 寄存值落后一拍：上一拍执行过 CSR/系统指令、陷入或进调试模式 (可能关掉了使能或换了特权级)，
  // 本拍就不报中断，等下一拍按新状态重新算
  logic       irq_c, irq_q, irq_blk_q;
  logic [5:0] irq_cause_c;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      irq_q <= 1'b0; irq_cause_o <= '0; irq_blk_q <= 1'b1;
    end else begin
      irq_q <= irq_c; irq_cause_o <= irq_cause_c;
      irq_blk_q <= ex_req_i || trap_i || dbg_enter_i;
    end
  end
  assign irq_pending_o = irq_q && !irq_blk_q;
  // 组内优先级：MEI MSI MTI SEI SSI STI SGEI VSEI VSSI VSTI LCOFI (与 Spike 一致)
  function automatic logic [5:0] irq_pick(input logic [63:0] e);
    if (e[11]) return 6'd11;
    if (e[3])  return 6'd3;
    if (e[7])  return 6'd7;
    if (e[9])  return 6'd9;
    if (e[1])  return 6'd1;
    if (e[5])  return 6'd5;
    if (e[12]) return 6'd12;
    if (e[10]) return 6'd10;
    if (e[2])  return 6'd2;
    if (e[6])  return 6'd6;
    return 6'd13;
  endfunction
  always_comb begin
    logic [63:0] pend, m_en, h_en, v_en;
    pend = mip & mie;
    // 三组 (先 M 再 HS 再 VS，前一组有就不看后一组)：
    //   M：未委托，priv < M 或 MIE；HS：mideleg 且非 hideleg，V = 1 或 U 或 (HS 且 SIE)，M 态不取；
    //   VS：hideleg，仅 V = 1，且 VU 或 (VS 且 vsstatus.SIE)
    m_en = pend & ~mideleg_rd & {64{(priv != PRV_M) || st_mie}};
    h_en = pend &  mideleg_rd & ~hideleg & {64{virt || (priv == PRV_U) || (priv == PRV_S && st_sie)}};
    v_en = pend &  hideleg & {64{virt && ((priv == PRV_U) || (priv == PRV_S && vs_sie))}};
    // 调试模式、或单步且 stepie = 0：不响应中断
    if (dbg_mode || (d_step && !d_stepie)) begin m_en = '0; h_en = '0; v_en = '0; end
    irq_c       = |(m_en | h_en | v_en);
    irq_cause_c = (|m_en) ? irq_pick(m_en) : (|h_en) ? irq_pick(h_en) : irq_pick(v_en);
  end

  //--------------------------------------------------------------------------
  // 陷入目标
  //--------------------------------------------------------------------------
  // VS：V = 1 且 priv <= S，异常要 medeleg & hedeleg、中断要 hideleg；HS：priv <= S 且 medeleg / mideleg；其余 M
  wire trap_to_vs = HAS_H && virt && (priv != PRV_M) &&
                    (trap_irq_i ? hideleg[trap_cause_i] : (medeleg[trap_cause_i] && hedeleg[trap_cause_i]));
  wire trap_to_s  = !trap_to_vs && (priv != PRV_M) &&
                    (trap_irq_i ? mideleg_rd[trap_cause_i] : medeleg[trap_cause_i]);
  // 送 VS 的中断 cause 转换：VSSI 2 -> 1、VSTI 6 -> 5、VSEI 10 -> 9
  wire [5:0] vs_cause = (trap_irq_i && trap_cause_i <= 6'd10) ? trap_cause_i - 6'd1 : trap_cause_i;
  // 三个目标的入口各自先算好 (向量模式的加法与委托判断并行)，最后只剩一级三选一：
  // 串行写法 (委托 -> 选 tvec -> 64 位加) 在 h1 块级综合 in 组 −35 ps (2026-10-30)
  function automatic logic [63:0] tvec_ent(input logic [63:0] tv, input logic [5:0] c, input logic irq);
    return (irq && tv[0]) ? {tv[63:2], 2'b00} + {56'b0, c, 2'b00} : {tv[63:2], 2'b00};
  endfunction
  wire [63:0] ent_m  = tvec_ent(mtvec,  trap_cause_i, trap_irq_i);
  wire [63:0] ent_s  = tvec_ent(stvec,  trap_cause_i, trap_irq_i);
  wire [63:0] ent_vs = tvec_ent(vstvec, vs_cause,     trap_irq_i);
  assign trap_vec_o = trap_to_vs ? ent_vs : trap_to_s ? ent_s : ent_m;
  // GVA：tval 写入的是 guest 虚地址 —— guest page fault 恒 1；取指类 / breakpoint 看 V；访存类看有效 V (HLV / HSV、MPRV + MPV)
  logic trap_gva;
  always_comb begin
    trap_gva = 1'b0;
    if (HAS_H && !trap_irq_i)
      unique case (trap_cause_i)
        EXC_INST_GPAGE, EXC_LOAD_GPAGE, EXC_STORE_GPAGE:          trap_gva = 1'b1;
        EXC_INST_MISALIGN, EXC_INST_ACCESS, EXC_INST_PAGE, EXC_BREAKPOINT:
                                                                  trap_gva = virt;
        EXC_LOAD_MISALIGN, EXC_LOAD_ACCESS, EXC_STORE_MISALIGN, EXC_STORE_ACCESS, EXC_LOAD_PAGE, EXC_STORE_PAGE:
                                                                  trap_gva = trap_hv_i || dctx_o.v;
        default: ;
      endcase
  end

  //--------------------------------------------------------------------------
  // 翻译上下文
  //--------------------------------------------------------------------------
  wire        mprv_e = st_mprv && !dbg_mode;              // mprven = 0：调试模式下 MPRV 不生效
  priv_e      dpv;
  assign      dpv    = mprv_e ? priv_e'(st_mpp) : priv;
  wire        dvv    = mprv_e ? (st_mpv && priv_e'(st_mpp) != PRV_M) : virt;
  always_comb begin
    logic [63:0] da;
    // 取指
    ictx_o      = '0;
    ictx_o.v    = virt;
    ictx_o.priv = priv;
    ictx_o.xl   = HAS_MMU && priv != PRV_M && (virt ? (vsatp[63] || hgatp_mode != 4'd0) : satp[63]);
    ictx_o.cn   = virt ? vsatp[63] : satp[63];
    ictx_o.vam  = virt ? vsatp[61:60] : satp[61:60];
    // 普通数据访存
    da          = dvv ? vsatp : satp;
    dctx_o      = '0;
    dctx_o.v    = dvv;
    dctx_o.priv = dpv;
    dctx_o.xl   = HAS_MMU && dpv != PRV_M && (da[63] || (dvv && hgatp_mode != 4'd0));
    dctx_o.cn   = da[63];
    dctx_o.gmxr = st_mxr;
    dctx_o.sum  = dvv ? vs_sum : st_sum;
    dctx_o.mxr  = st_mxr | (dvv & vs_mxr);
    dctx_o.vam  = da[61:60];
    dctx_o.zx   = (dpv == PRV_M) || !da[63];
    // 指针掩码 (Spike get_pmlen)：M 不掩 (无 Smmpm)；MXR 不掩；HS 用 menvcfg，VS 用 henvcfg，U / VU 用 senvcfg
    if (dpv == PRV_M || st_mxr || (dvv && vs_mxr)) dctx_o.pmm = 2'b00;
    else if (dpv == PRV_S)                         dctx_o.pmm = dvv ? henvcfg_pmm : menvcfg_pmm;
    else                                           dctx_o.pmm = senvcfg_pmm;
    // HLV / HSV：V = 1，特权级 hstatus.SPVP；U 态发出且 SPVP = 0 时用 hstatus.HUPMM
    hctx_o      = '0;
    hctx_o.v    = 1'b1;
    hctx_o.priv = hs_spvp ? PRV_S : PRV_U;
    hctx_o.xl   = HAS_MMU && (vsatp[63] || hgatp_mode != 4'd0);
    hctx_o.cn   = vsatp[63];
    hctx_o.gmxr = st_mxr;
    hctx_o.sum  = vs_sum;
    hctx_o.mxr  = st_mxr | vs_mxr;
    hctx_o.vam  = vsatp[61:60];
    hctx_o.zx   = !vsatp[63];
    if (st_mxr || vs_mxr)                          hctx_o.pmm = 2'b00;
    else if (hs_spvp)                              hctx_o.pmm = henvcfg_pmm;
    else if (priv == PRV_U)                        hctx_o.pmm = hs_hupmm;
    else                                           hctx_o.pmm = senvcfg_pmm;
  end

  //--------------------------------------------------------------------------
  // 状态更新
  //--------------------------------------------------------------------------
  function automatic logic [1:0] legal_mpp(input logic [1:0] v);
    return (v == 2'b10) ? 2'b00 : v;
  endfunction
  function automatic logic atp_ok(input logic [3:0] m);
    return (m == SATP_BARE) || (HAS_MMU && (m == SATP_SV39 || m == SATP_SV48 || m == SATP_SV57));
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      priv <= PRV_M; virt <= 1'b0;
      st_sie <= 1'b0; st_mie <= 1'b0; st_spie <= 1'b0; st_mpie <= 1'b0; st_spp <= 1'b0;
      st_mprv <= 1'b0; st_sum <= 1'b0; st_mxr <= 1'b0; st_tvm <= 1'b0; st_tw <= 1'b0; st_tsr <= 1'b0;
      st_mpv <= 1'b0; st_gva <= 1'b0;
      st_mpp <= 2'b00; st_fs <= 2'b00; st_vs <= 2'b00;
      medeleg <= '0; mideleg <= '0; mie <= '0; mip_sw <= '0;
      mtvec <= '0; stvec <= '0; mscratch <= '0; sscratch <= '0;
      mepc <= '0; sepc <= '0; mcause <= '0; scause <= '0; mtval <= '0; stval <= '0; satp <= '0; sret_pc <= '0;
      mcounteren <= '0; scounteren <= '0; mcountinhibit <= '0; mci_ir_eff <= 1'b0; mci_pend <= 1'b0;
      mcycle <= '0; minstret <= '0;
      for (int i = 0; i < 4; i++) begin hpmcnt[i] <= '0; hpmevt[i] <= '0; end
      hpm_of <= '0; hpm_minh <= '0; hpm_sinh <= '0; hpm_uinh <= '0; hpm_vsinh <= '0; hpm_vuinh <= '0;
      fflags <= '0; frm <= '0;
      menvcfg_fiom <= 1'b0; senvcfg_fiom <= 1'b0;
      menvcfg_cbze <= 1'b0; menvcfg_cbcfe <= 1'b0; menvcfg_cbie <= '0;
      senvcfg_cbze <= 1'b0; senvcfg_cbcfe <= 1'b0; senvcfg_cbie <= '0;
      menvcfg_pmm <= '0; senvcfg_pmm <= '0;
      menvcfg_stce <= 1'b0; stimecmp <= '0; stip_hw <= 1'b0; menvcfg_pbmte <= 1'b0;
      m0_se <= 1'b0; m0_envcfg <= 1'b0; m0_p113 <= 1'b0; m0_c <= 1'b0; s0_c <= 1'b0; m_se <= '0;
      for (int i = 0; i < 8; i++) begin pmpcfg[i] <= '0; pmpaddr[i] <= '0; end
      hcontext <= '0; scontext <= '0;
      hs_gva <= 1'b0; hs_spv <= 1'b0; hs_spvp <= 1'b0; hs_hu <= 1'b0; hs_vtvm <= 1'b0; hs_vtw <= 1'b0; hs_vtsr <= 1'b0;
      hs_hupmm <= '0; hedeleg <= '0; hideleg <= '0; hvip_vstip <= 1'b0; hvip_vseip <= 1'b0;
      htval <= '0; htinst <= '0; mtval2 <= '0; mtinst <= '0; htimedelta <= '0; hcounteren <= '0;
      henvcfg_fiom <= 1'b0; henvcfg_cbze <= 1'b0; henvcfg_cbcfe <= 1'b0; henvcfg_pbmte <= 1'b0; henvcfg_stce <= 1'b0;
      henvcfg_cbie <= '0; henvcfg_pmm <= '0;
      h0_se <= 1'b0; h0_envcfg <= 1'b0; h0_c <= 1'b0; h_se <= '0;
      hgatp_mode <= '0; hgatp_vmid <= '0; hgatp_ppn <= '0;
      vs_sie <= 1'b0; vs_spie <= 1'b0; vs_spp <= 1'b0; vs_sum <= 1'b0; vs_mxr <= 1'b0; vs_fs <= '0; vs_vs <= '0;
      vstvec <= '0; vsscratch <= '0; vsepc <= '0; vscause <= '0; vstval <= '0; vsatp <= '0; vstimecmp <= '0; vstip_hw <= 1'b0; mt_hd <= '0;
      dbg_mode <= 1'b0; d_ebreakm <= 1'b0; d_ebreaks <= 1'b0; d_ebreaku <= 1'b0; d_ebreakvs <= 1'b0; d_ebreakvu <= 1'b0;
      d_stepie <= 1'b0; d_v <= 1'b0;
      d_step <= 1'b0; d_cause <= '0; d_prv <= 2'b11; dpc <= '0; dscratch0 <= '0; dscratch1 <= '0;
    end else begin
      // 计数器
      if (!mcountinhibit[0]) mcycle <= mcycle + 1'b1;
      // 提交级 (ROB 头 -> 提交条数) 到 64 位加法不在一拍里：先寄存条数，下一拍累加；读时补上 (见 rval)
      minstret <= minstret + 64'(cm_cnt_eff);
      if (mci_pend && cm_cnt_i != 3'd0) begin mci_pend <= 1'b0; mci_ir_eff <= mcountinhibit[2]; end
      // hpm 计数：Sscofpmf 的 xINH 按当前特权级 (含 V) 抑制；从全 1 回绕时 OF 由 0 变 1 并置 LCOFIP (OF 已为 1 不再报)
      for (int i = 0; i < 4; i++) begin
        logic inh;
        inh = (priv == PRV_M && hpm_minh[i]) ||
              (priv == PRV_S && (virt ? hpm_vsinh[i] : hpm_sinh[i])) ||
              (priv == PRV_U && (virt ? hpm_vuinh[i] : hpm_uinh[i]));
        if (!mcountinhibit[3+i] && !inh && hpmevt[i] != 8'd0 && hpmevt[i] <= 8'd8 && hpm_ev_q[hpmevt[i]-1]) begin
          hpmcnt[i] <= hpmcnt[i] + 1'b1;
          if (hpmcnt[i] == '1 && !hpm_of[i]) begin hpm_of[i] <= 1'b1; mip_sw[13] <= 1'b1; end
        end
      end
      // Sstc：STIP / VSTIP 比较寄存一拍 (mtime 每拍变，64 位比较不进中断仲裁的路径)
      stip_hw  <= mtime_i >= stimecmp;
      mt_hd    <= mtime_i + htimedelta;
      vstip_hw <= mt_hd >= vstimecmp;
      // 浮点 / 向量状态 (V = 1 时 vsstatus 的同时置 Dirty)
      if (|(cm_fflags_i | vfflags_i)) fflags <= fflags | cm_fflags_i | vfflags_i;
      if (cm_fp_dirty_i || (|vfflags_i)) begin st_fs <= 2'b11; if (virt) vs_fs <= 2'b11; end

      // 陷入
      if (trap_i) begin
        if (trap_to_vs) begin
          vsepc   <= trap_epc_i;
          vscause <= {trap_irq_i, 57'b0, vs_cause};
          vstval  <= trap_tval_i;
          vs_spie <= vs_sie;
          vs_sie  <= 1'b0;
          vs_spp  <= (priv != PRV_U);
          priv    <= PRV_S;
        end else if (trap_to_s) begin
          sepc    <= trap_epc_i;
          scause  <= {trap_irq_i, 57'b0, trap_cause_i};
          stval   <= trap_tval_i;
          st_spie <= st_sie;
          st_sie  <= 1'b0;
          st_spp  <= (priv != PRV_U);
          priv    <= PRV_S;
          if (HAS_H) begin
            htval  <= trap_tval2_i;
            htinst <= trap_tinst_i;
            hs_spv <= virt;
            if (virt) hs_spvp <= (priv != PRV_U);
            hs_gva <= trap_gva;
            virt   <= 1'b0;
          end
        end else begin
          mepc    <= trap_epc_i;
          mcause  <= {trap_irq_i, 57'b0, trap_cause_i};
          mtval   <= trap_tval_i;
          st_mpie <= st_mie;
          st_mie  <= 1'b0;
          st_mpp  <= priv;
          priv    <= PRV_M;
          if (HAS_H) begin
            mtval2 <= trap_tval2_i;
            mtinst <= trap_tinst_i;
            st_mpv <= virt;
            st_gva <= trap_gva;
            virt   <= 1'b0;
          end
        end
      end

      // 调试模式进入 (不动 mepc/mcause 等)
      if (dbg_enter_i) begin
        dbg_mode <= 1'b1;
        dpc      <= dbg_pc_i;
        d_cause  <= dbg_cause_i;
        d_prv    <= priv;
        d_v      <= virt;
        priv     <= PRV_M;
        virt     <= 1'b0;
      end
      // DRET：回到 dcsr.prv (/ dcsr.v)
      if (ex_req_i && !ill && !is_csr && op == SYS_DRET) begin
        dbg_mode <= 1'b0;
        priv     <= priv_e'(d_prv);
        virt     <= HAS_H && d_v && priv_e'(d_prv) != PRV_M;
        if (priv_e'(d_prv) != PRV_M) st_mprv <= 1'b0;
      end

      // 执行口：xRET
      if (ex_req_i && !ill && !is_csr && op == SYS_MRET) begin
        priv    <= priv_e'(st_mpp);
        virt    <= HAS_H && st_mpv && priv_e'(st_mpp) != PRV_M;
        st_mpv  <= 1'b0;
        st_mie  <= st_mpie;
        st_mpie <= 1'b1;
        st_mpp  <= 2'b00;
        if (priv_e'(st_mpp) != PRV_M) st_mprv <= 1'b0;
      end
      if (ex_req_i && !ill && !is_csr && op == SYS_SRET) begin
        if (virt) begin                      // VS 执行：vsstatus，V 保持 1
          priv    <= vs_spp ? PRV_S : PRV_U;
          vs_sie  <= vs_spie;
          vs_spie <= 1'b1;
          vs_spp  <= 1'b0;
          sret_pc <= vsepc;
        end else begin                       // HS / M 执行：目标 V = hstatus.SPV
          priv    <= st_spp ? PRV_S : PRV_U;
          virt    <= HAS_H && hs_spv;
          hs_spv  <= 1'b0;
          st_sie  <= st_spie;
          st_spie <= 1'b1;
          st_spp  <= 1'b0;
          st_mprv <= 1'b0;
          sret_pc <= sepc;
        end
      end

      if (vs_dirty_i) begin st_vs <= 2'b11; if (virt) vs_vs <= 2'b11; end
      // 执行口：CSR 写 (按重定向后的地址 ra)
      if (wr) begin
        unique casez (ra)
          CSR_FFLAGS: begin fflags <= wv[4:0]; st_fs <= 2'b11; if (virt) vs_fs <= 2'b11; end
          CSR_FRM:    begin frm <= wv[2:0];    st_fs <= 2'b11; if (virt) vs_fs <= 2'b11; end
          CSR_FCSR:   begin fflags <= wv[4:0]; frm <= wv[7:5]; st_fs <= 2'b11; if (virt) vs_fs <= 2'b11; end
          CSR_SSTATUS: begin
            st_sie <= wv[1]; st_spie <= wv[5]; st_spp <= wv[8];
            st_fs <= HAS_FPU ? wv[14:13] : 2'b00; st_sum <= wv[18]; st_mxr <= wv[19];
            st_vs <= wv[10:9];
          end
          CSR_MSTATUS: begin
            st_sie <= wv[1]; st_mie <= wv[3]; st_spie <= wv[5]; st_mpie <= wv[7]; st_spp <= wv[8];
            st_mpp <= legal_mpp(wv[12:11]); st_fs <= HAS_FPU ? wv[14:13] : 2'b00; st_vs <= wv[10:9];
            st_mprv <= wv[17]; st_sum <= wv[18]; st_mxr <= wv[19];
            st_tvm <= wv[20]; st_tw <= wv[21]; st_tsr <= wv[22];
            if (HAS_H) begin st_gva <= wv[38]; st_mpv <= wv[39]; end
          end
          CSR_SIE:        mie <= (mie & ~mideleg) | (wv & mideleg & MIE_MASK);
          CSR_STVEC:      stvec <= wv & ~64'd2;
          CSR_SCOUNTEREN: scounteren <= wv[31:0];
          CSR_SENVCFG:    begin
            senvcfg_fiom <= wv[0]; senvcfg_cbze <= wv[7]; senvcfg_cbcfe <= wv[6];
            senvcfg_cbie <= (wv[5:4] == 2'b10) ? 2'b00 : wv[5:4];
            senvcfg_pmm  <= (wv[33:32] == 2'b01) ? 2'b00 : wv[33:32];
          end
          CSR_SSCRATCH:   sscratch <= wv;
          CSR_SEPC:       sepc <= wv & ~64'd1;
          CSR_SCAUSE:     scause <= wv;
          CSR_STVAL:      stval <= wv;
          CSR_SIP:        mip_sw <= (mip_sw & ~(mideleg & SIP_SW_MASK)) | (wv & mideleg & SIP_SW_MASK);
          CSR_SATP: begin
            if (wv[63:60] == SATP_BARE)        satp <= {4'b0, wv[59:44], wv[43:0]};   // Bare 也保留 ASID/PPN (与 Spike 一致)
            else if (atp_ok(wv[63:60]))        satp <= {wv[63:60], wv[59:44], wv[43:0]};
          end
          CSR_MEDELEG:    medeleg <= wv & MEDELEG_MASK;
          CSR_MIDELEG:    mideleg <= wv & MIDELEG_MASK;
          CSR_MIE:        mie <= wv & MIE_MASK;
          CSR_MTVEC:      mtvec <= wv & ~64'd2;
          CSR_MCOUNTEREN: mcounteren <= wv[31:0];
          CSR_MENVCFG:    begin
            menvcfg_pmm  <= (wv[33:32] == 2'b01) ? 2'b00 : wv[33:32];
            menvcfg_fiom <= wv[0]; menvcfg_stce <= wv[63]; menvcfg_pbmte <= wv[62]; menvcfg_cbze <= wv[7]; menvcfg_cbcfe <= wv[6];
            menvcfg_cbie <= (wv[5:4] == 2'b10) ? 2'b00 : wv[5:4];
          end
          // RVA23 R3
          CSR_STIMECMP:   stimecmp <= wv;
          12'h30C:        begin m0_se <= wv[63]; m0_envcfg <= wv[62]; m0_p113 <= wv[56] & HAS_H; m0_c <= wv[0]; end
          12'h30D, 12'h30E, 12'h30F: m_se[ra[1:0]] <= wv[63];
          12'h10C:        if (m0_c && (!virt || h0_c)) s0_c <= wv[0];      // 写掩码按 mstateen0 (V = 1 再按 hstateen0) 屏蔽
          // H：HS 级
          CSR_HSTATUS: begin
            hs_gva <= wv[6]; hs_spv <= wv[7]; hs_spvp <= wv[8]; hs_hu <= wv[9];
            hs_vtvm <= wv[20]; hs_vtw <= wv[21]; hs_vtsr <= wv[22];
            hs_hupmm <= (wv[49:48] == 2'b01) ? 2'b00 : wv[49:48];
          end
          CSR_HEDELEG:    hedeleg <= wv & HEDELEG_MASK;
          CSR_HIDELEG:    hideleg <= wv & VS_IRQ;
          CSR_HIE:        mie <= (mie & ~HS_IRQ) | (wv & HS_IRQ);
          CSR_HTIMEDELTA: htimedelta <= wv;
          CSR_HCOUNTEREN: hcounteren <= wv[31:0];
          CSR_HENVCFG: begin
            henvcfg_fiom <= wv[0]; henvcfg_cbze <= wv[7]; henvcfg_cbcfe <= wv[6];
            henvcfg_cbie <= (wv[5:4] == 2'b10) ? 2'b00 : wv[5:4];
            henvcfg_pmm  <= (wv[33:32] == 2'b01) ? 2'b00 : wv[33:32];
            if (menvcfg_pbmte) henvcfg_pbmte <= wv[62];                     // menvcfg 对应位 = 1 时才可写 (存储值保留)
            if (menvcfg_stce)  henvcfg_stce  <= wv[63];
          end
          12'h60C: begin                                                     // hstateen0：写掩码再按 mstateen0
            if (m0_se)     h0_se     <= wv[63];
            if (m0_envcfg) h0_envcfg <= wv[62];
            if (m0_c)      h0_c      <= wv[0];
          end
          12'h60D, 12'h60E, 12'h60F: if (m_se[ra[1:0]]) h_se[ra[1:0]] <= wv[63];
          CSR_HTVAL:      htval <= wv;
          CSR_HIP:        mip_sw[2] <= wv[2];
          CSR_HVIP:       begin mip_sw[2] <= wv[2]; hvip_vstip <= wv[6]; hvip_vseip <= wv[10]; end
          CSR_HTINST:     htinst <= wv;
          CSR_HGATP: begin                                                   // 非法 MODE：MODE 保持，VMID / PPN 照写
            if (wv[63:60] == SATP_BARE || (HAS_MMU && (wv[63:60] == 4'd8 || wv[63:60] == 4'd9 || wv[63:60] == 4'd10)))
              hgatp_mode <= wv[63:60];
            hgatp_vmid <= wv[57:44];
            hgatp_ppn  <= wv[43:2];
          end
          CSR_HCONTEXT:   hcontext <= wv[13:0];
          // H：VS 级
          CSR_VSSTATUS: begin
            vs_sie <= wv[1]; vs_spie <= wv[5]; vs_spp <= wv[8]; vs_vs <= wv[10:9];
            vs_fs <= HAS_FPU ? wv[14:13] : 2'b00; vs_sum <= wv[18]; vs_mxr <= wv[19];
          end
          CSR_VSIE:       mie <= (mie & ~hideleg) | ((wv << 1) & hideleg & VS_IRQ);
          CSR_VSTVEC:     vstvec <= wv & ~64'd2;
          CSR_VSSCRATCH:  vsscratch <= wv;
          CSR_VSEPC:      vsepc <= wv & ~64'd1;
          CSR_VSCAUSE:    vscause <= wv;
          CSR_VSTVAL:     vstval <= wv;
          CSR_VSIP:       if (hideleg[2]) mip_sw[2] <= wv[1];
          CSR_VSTIMECMP:  vstimecmp <= wv;
          CSR_VSATP: begin
            if (ca == CSR_SATP) begin                                        // V = 1 经 satp 地址：非法 MODE 整笔忽略
              if (atp_ok(wv[63:60])) vsatp <= {wv[63:60], wv[59:44], wv[43:0]};
            end else begin                                                   // 直接写 vsatp：非法 MODE 时 MODE 保持
              vsatp <= {atp_ok(wv[63:60]) ? wv[63:60] : vsatp[63:60], wv[59:44], wv[43:0]};
            end
          end
          CSR_MTINST:     mtinst <= wv;
          CSR_MTVAL2:     mtval2 <= wv;
          CSR_MCOUNTINHIBIT: begin mcountinhibit <= wv[31:0] & 32'hFFFF_FFFD; mci_pend <= 1'b1; end   // TM 位只读 0
          CSR_MSCRATCH:   mscratch <= wv;
          CSR_MEPC:       mepc <= wv & ~64'd1;
          CSR_MCAUSE:     mcause <= wv;
          CSR_MTVAL:      mtval <= wv;
          CSR_MIP: begin                       // STCE = 1 时 STIP 不可写 (保持)，与 Spike 的 mip 写掩码一致
            logic [63:0] m;
            m = MIP_SW_MASK & ~{58'b0, menvcfg_stce, 5'b0};
            mip_sw <= (mip_sw & ~m) | (wv & m);
          end
          CSR_PMPCFG0: begin
            for (int i = 0; i < 8; i++)
              if (!pmpcfg[i][7]) begin                                 // L 位锁定后不可改
                logic [7:0] c;
                c = wv[8*i +: 8] & 8'h9F;
                if (c[1] && !c[0]) c[1] = 1'b0;                        // R=0 W=1 是保留组合：清 W
                if (c[4:3] == 2'b10) c[4:3] = 2'b11;                   // 粒度 64 B：NA4 不可选，按 NAPOT (同 Spike)
                pmpcfg[i] <= c;
              end
          end
          12'h3B?: begin
            // pmpaddr[i] 可写：本项未锁定，且第 i+1 项不是「锁定的 TOR」(它以 pmpaddr[i] 为下界，规范要求一并锁住) (#33)
            if (ra[3] == 1'b0 && !pmpcfg[ra[2:0]][7] &&
                !(ra[2:0] != 3'd7 && pmpcfg[ra[2:0] + 3'd1][7] && pmpcfg[ra[2:0] + 3'd1][4:3] == 2'b01))
              pmpaddr[ra[2:0]] <= wv[53:0];
          end
          CSR_MCYCLE:     mcycle <= wv;
          CSR_MINSTRET:   minstret <= mci_ir_eff ? wv : wv - 1'b1;
          12'hB03, 12'hB04, 12'hB05, 12'hB06: hpmcnt[ra[1:0] - 2'd3] <= wv;
          12'h323, 12'h324, 12'h325, 12'h326: begin
            hpmevt[ra[1:0] - 2'd3]   <= wv[7:0];
            hpm_of[ra[1:0] - 2'd3]   <= wv[63];
            hpm_minh[ra[1:0] - 2'd3] <= wv[62];
            hpm_sinh[ra[1:0] - 2'd3] <= wv[61];
            hpm_uinh[ra[1:0] - 2'd3] <= wv[60];
            if (HAS_H) begin hpm_vsinh[ra[1:0] - 2'd3] <= wv[59]; hpm_vuinh[ra[1:0] - 2'd3] <= wv[58]; end
          end
          12'h7A8: hcontext <= HAS_H ? wv[13:0] : {1'b0, wv[12:0]};
          12'h5A8: scontext <= wv[31:0];
          CSR_DCSR: begin
            d_ebreakm <= wv[15]; d_ebreaks <= wv[13]; d_ebreaku <= wv[12];
            if (HAS_H) begin d_ebreakvs <= wv[17]; d_ebreakvu <= wv[16]; d_v <= wv[5]; end
            d_stepie <= wv[11]; d_step <= wv[2];
            d_prv <= (wv[1:0] == 2'b10) ? d_prv : wv[1:0];      // 2 (H) 不存在：保持
          end
          CSR_DPC:       dpc <= wv & ~64'd1;
          CSR_DSCRATCH0: dscratch0 <= wv;
          CSR_DSCRATCH1: dscratch1 <= wv;
          default: ;
        endcase
      end
    end
  end

  //--------------------------------------------------------------------------
  // 输出
  //--------------------------------------------------------------------------
  assign priv_o = priv;
  assign virt_o = virt;
  assign hlv_ok_o = HAS_H && !virt && (priv != PRV_U || hs_hu);
  assign hlv_vi_o = HAS_H && virt;
  // 有效 FS / VS：V = 1 时 vsstatus 的为 Off 也按 Off (Spike sstatus->enabled 要求两者都非 Off；结果是非法指令)
  assign fs_o   = (virt && vs_fs == 2'b00) ? 2'b00 : st_fs;
  assign vs_o   = (virt && vs_vs == 2'b00) ? 2'b00 : st_vs;
  assign frm_o  = frm;
  assign satp_o  = satp;
  assign vsatp_o = vsatp;
  assign hgatp_o = hgatp;
  assign satp_wr_o = wr && (ra == CSR_SATP || ra == CSR_VSATP || ra == CSR_HGATP);
  always_comb for (int i = 0; i < 8; i++) begin pmpcfg_o[i] = pmpcfg[i]; pmpaddr_o[i] = pmpaddr[i]; end
  assign mepc_o = mepc;
  assign sepc_o = sret_pc;
  assign pbmte_o  = menvcfg_pbmte;
  assign vpbmte_o = menvcfg_pbmte & henvcfg_pbmte;
  // 与 Spike 的 require_envcfg 一致：M 态总可执行；否则 menvcfg 位 = 0 -> 非法；V = 0 的 U 态 senvcfg 位 = 0 -> 非法；
  // V = 1 时 henvcfg 位 = 0 或 (VU 且 senvcfg 位 = 0) -> virtual
  always_comb begin
    logic [2:0] me, se, he;
    me = {menvcfg_cbze, menvcfg_cbcfe, menvcfg_cbie != 2'b00};
    se = {senvcfg_cbze, senvcfg_cbcfe, senvcfg_cbie != 2'b00};
    he = {henvcfg_cbze, henvcfg_cbcfe, henvcfg_cbie != 2'b00};
    for (int k = 0; k < 3; k++) begin
      cbo_ok_o[k] = 1'b1; cbo_vi_o[k] = 1'b0;
      if (priv != PRV_M) begin
        if (!me[k])                                            cbo_ok_o[k] = 1'b0;
        else if (!virt && priv == PRV_U && !se[k])             cbo_ok_o[k] = 1'b0;
        else if (virt && (!he[k] || (priv == PRV_U && !se[k]))) begin cbo_ok_o[k] = 1'b0; cbo_vi_o[k] = 1'b1; end
      end
    end
  end
  assign dbg_mode_o   = dbg_mode;
  assign dpc_o        = dpc;
  assign dbg_step_o   = d_step && !dbg_mode;
  assign ebreak_dbg_o = (priv == PRV_M) ? d_ebreakm :
                        (priv == PRV_S) ? (virt ? d_ebreakvs : d_ebreaks) : (virt ? d_ebreakvu : d_ebreaku);
endmodule
