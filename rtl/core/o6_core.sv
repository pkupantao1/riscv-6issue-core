//==============================================================================
// o6_core.sv - OOO6 核顶层 (M1：整数核 + LSU，理想 I/D 存储接口)
//
// 唤醒口 (整数，N_IWAKE 个)：
//   0..3  P0..P3 发射当拍 (1 拍运算)；P2 / P3 在 MUL 处于 EX1 时改为 MUL 的唤醒
//   4     W4 长延迟共享口 (DIV / CSR / F2I)
//   5..   LSU 各 load 流水 (L2 级)
// 浮点唤醒 (N_FWAKE 个)：0..2 浮点簇 (两口 + I2F)，3.. LSU 各 load 流水
// 完成口 (N_CMPL 个)：0..3 P0..P3，4 W4，5.. LSU 各流水 (load × N_LDP + STA)，其后浮点簇 3 个
// 发射队列：A (P0/P1)、B (P2/P3)、M (load × N_LDP)、F (浮点 2)、S (STA 1)、D (整数 STD × N_STDP)
//==============================================================================
`default_nettype none
module o6_core
  import o6_pkg::*;
  import o6_isa_pkg::*;
#(
  parameter bit HAS_FPU = 1'b0,
  parameter bit HAS_MMU = 1'b0,
  parameter bit FE_M2   = 1'b1       // 1 = M2 前端 (uBTB + 方向表，三级取指)；0 = M1 简化前端 (取指环路过长，仅作参考)
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 下游存储 (64 B 行)：I$ 与 D$ 共用，写回请求没有响应
  output logic                mem_req_valid_o,
  input  wire                 mem_req_ready_i,
  output logic                mem_req_we_o,
  output logic [PA_W-1:6]     mem_req_addr_o,
  output logic [511:0]        mem_req_data_o,
  output logic [3:0]          mem_req_id_o,       // 15 = I$，0..14 = D$ 的 MSHR 号
  input  wire                 mem_resp_valid_i,
  output logic                mem_resp_ready_o,
  input  wire  [3:0]          mem_resp_id_i,
  input  wire  [511:0]        mem_resp_data_i,

  // MMIO
  output logic                mmio_req_o,
  output logic                mmio_we_o,
  output logic [PA_W-1:0]     mmio_addr_o,
  output logic [1:0]          mmio_size_o,
  output logic [63:0]         mmio_wdata_o,
  output logic [7:0]          mmio_wmask_o,
  input  wire                 mmio_ack_i,
  input  wire  [63:0]         mmio_rdata_i,

  // 中断与时间
  input  wire                 mtip_i,
  input  wire                 msip_i,
  input  wire                 meip_i,
  input  wire                 seip_i,
  input  wire  [63:0]         mtime_i,

  // 调试
  input  wire                 dbg_haltreq_i,      // 调试模块的停机请求 (电平)

  // MBIST (L1 缓存)
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  //==========================================================================
  // 全局广播
  //==========================================================================
  br_update_t  br_upd;
  logic        flush;
  wakeup_t [N_IWAKE-1:0] iwake;        // 给发射队列：load 在 L1 投机唤醒
  wakeup_t [N_IWAKE_C-1:0] iwake_c;    // 给重命名 (busy 表 / 队列中的 uop)：只含确认过的唤醒 (末尾 N_ALU 个是补发的)
  wakeup_t [N_CWAKE-1:0] cwake;        // 给 IQ 入队口的确认唤醒：load 在 L2 的 + 整数口补发的
  wakeup_t [N_ALU-1:0]   awk_q;        // 带 poia 发射的整数口唤醒，寄存一拍等判定后补发
  wakeup_t [N_FWAKE-1:0] fwake;
  wakeup_t [N_LDP-1:0]   lsu_cwake;    // load 在 L2 的确认唤醒
  wakeup_t [N_LDP-1:0]   lsu_ewake;    // load 在 L1 的投机唤醒 (只给 IQ 入队口)
  lwake_t  [N_LDP-1:0]   lsu_lwake;    // load -> load 快速唤醒 (只给 IQ_M)
  logic    [N_LDP-1:0]   ld_fail, ld_fail_q;   // load 投机唤醒失败 (本拍 / 寄存)
  // IQ 用定制宏 AGESEL / WKCAM (32 项、端口 <= 2 的队列；见 o6_iq USE_CAM)
`ifdef O6_IQ_CAM
  localparam bit IQ_CAM = 1'b1;
`else
  localparam bit IQ_CAM = 1'b0;
`endif
  wb_t     [N_IWB-1:0] iwb;
  preg_t   [N_IWB-1:0] iwb_pn;           // 各写回口下一拍的目的号 (整数执行单元提前比较旁路用)
  preg_t   [N_ALU:0]   exu_wb_pn;
  preg_t   [N_LDP-1:0] lsu_iwb_pn;
  cmpl_t   [N_CMPL-1:0] cmpl;

  //==========================================================================
  // CSR 状态
  //==========================================================================
  priv_e       priv;
  logic [1:0]  fs, vs;
  logic        vs_dirty;
  logic [2:0]  frm;

  //==========================================================================
  // 前端
  //==========================================================================
  logic        redirect;
  logic [63:0] redirect_pc;
  logic [FTQ_W-1:0] redirect_ftq;
  logic [63:0] cm_redirect_pc;
  logic        cm_ftq_v;
  logic [FTQ_W-1:0] cm_ftq_idx;
  // 提交包 (见"提交包"一节)
  logic [W-1:0]     cmq_rd_valid, cmq_rd_fp, cmq_is_br;
  logic [4:0]       cmq_lrd [W];
  preg_t [W-1:0]    cmq_pdst, cmq_old_pdst;
  logic [W-1:0]     cmq_mvel;
  logic [2:0]       cmq_cnt, cmq_ld_cnt, cmq_st_cnt;
  logic             cmq_ftq_v;
  logic [FTQ_W-1:0] cmq_ftq_idx;
  logic [4:0]       cmq_fflags;
  logic             cmq_fp_dirty;
  logic [FTQ_W-1:0] ftq_ridx [N_FTQ_RD];      // [0..N_BRU-1] 分支单元，[N_BRU] 提交控制
  logic [63:0] ftq_pc [N_FTQ_RD], ftq_tgt [N_FTQ_RD];
  logic [W-1:0] ib_valid;
  fetch_inst_t [W-1:0] ib_inst;
  logic [2:0]  ib_pop;

  // MMU 公共信号
  logic             tlb_flush, satp_wr, pbmte, vpbmte;
  logic [63:0]      satp, vsatp, hgatp;
  // 翻译上下文 (o6_csr 算好)：取指 / 普通数据访存 (含 MPRV、MPV) / HLV-HSV (H：V = 1，特权级 hstatus.SPVP)
  xctx_t            ictx, dctx, hctx;
  logic             virt, hlv_ok, hlv_vi;
  logic [7:0]       pmpcfg  [8];
  logic [53:0]      pmpaddr [8];
  pmp_dec_t         pmpd_c  [8], pmpd [8];     // PMP 预译码，寄存一拍再给各检查点 (见 o6_pmp.sv)
  logic             ptw_ireq, ptw_dreq, ptw_ifill, ptw_dfill, ptw_busy;
  logic             ptw_iv, ptw_dv, ptw_fv;     // H：请求 / 填写的 V
  logic [63:0]      fe_gx, vm_gx;               // H：取指 / 向量访存 guest page fault 的 GPA
  logic [VPN_W-1:0] ptw_ivpn, ptw_dvpn, ptw_fvpn;
  logic [2:0]       ptw_flvl;
  logic [1:0]       ptw_ffault, ptw_fpbmt;
  logic             ptw_fnap;
  logic [3:0]       ptw_fgperm;                 // H：G-stage 权限 / GPA / 隐式 guest page fault
  logic [56:0]      ptw_fgpa;
  logic             ptw_fgimp;
  logic [PA_W-13:0] ptw_fppn;
  logic [7:0]       ptw_fperm;
  logic             ptw_mreq, ptw_mgnt, ptw_mvalid;
  logic [PA_W-1:0]  ptw_maddr;
  logic [63:0]      ptw_mdata;

  logic        ic_req, ic_ready, ic_resp_v, ic_kill;
  // 调试状态 (CSR 给出)
  logic        dbg_mode, dbg_step, ebreak_dbg, dbg_enter;
  logic [2:0]  dbg_cause;
  logic [63:0] dpc;
  logic        step_pend;        // 单步：一条指令已提交 (或已陷入)，下一条之前进调试模式

  // store 预取 (LSU STA -> D$)
  logic            dc_pf_req;
  logic [PA_W-1:0] dc_pf_paddr;
  // 访存违例 (LSU -> ROB 重放；位置给前端训练访存相关预测)
  logic [N_STA-1:0]     viol_v;   rob_ptr_t [N_STA-1:0] viol_rob;   // 各 STA 流水的违例 (给 ROB)
  logic                 viol_any;                                     // 有违例 (前端训练访存相关预测)
  logic [FTQ_W-1:0]     viol_ftq;
  logic [FTQ_OFF_W-1:0] viol_off;

  // 分支解析 (M2 前端训练)
  logic [N_BRU-1:0]     bres_v, bres_taken, bres_mis, bres_ind;
  logic                 redirect_call, redirect_ret;
  logic [63:0]          redirect_link;
  logic [FTQ_W-1:0]     bres_ftq [N_BRU];
  logic [FTQ_OFF_W-1:0] bres_off [N_BRU];
  logic [63:0]          bres_tgt [N_BRU];
  logic [63:0] ic_addr;
  logic [255:0] ic_resp_d;
  logic         fe_bd, fe_bf;          // 前端 (TAGE SRAM) 的 MBIST

  if (FE_M2) begin : g_fe2
  o6_frontend u_fe (
      .clk_i, .rst_ni,
      .imem_req_o(ic_req), .imem_ready_i(ic_ready), .imem_addr_o(ic_addr),
      .imem_resp_valid_i(ic_resp_v), .imem_data_i(ic_resp_d), .imem_kill_o(ic_kill),
      .be_redirect_i(redirect), .be_redirect_pc_i(redirect_pc), .be_redirect_ftq_i(redirect_ftq),
      .be_redirect_call_i(redirect_call), .be_redirect_ret_i(redirect_ret), .be_redirect_link_i(redirect_link),
      .cm_flush_i(flush), .cm_redirect_pc_i(cm_redirect_pc), .cm_flush_ftq_i(ftq_ridx[N_BRU]),
      .mdp_set_i(viol_any), .mdp_ftq_i(viol_ftq), .mdp_off_i(viol_off),
      .xlate_i(ictx.xl), .priv_i(priv), .dbg_mode_i(dbg_mode), .tlb_flush_i(tlb_flush), .pmpd_i(pmpd),
      .ptw_ireq_o(ptw_ireq), .ptw_ivpn_o(ptw_ivpn), .ptw_iv_o(ptw_iv), .ifill_i(ptw_ifill), .fill_v_i(ptw_fv), .xv_i(ictx.v),
      .cn_i(ictx.cn), .fgx_o(fe_gx), .fill_gperm_i(ptw_fgperm), .fill_gpa_i(ptw_fgpa), .fill_gimp_i(ptw_fgimp),
      .fill_vpn_i(ptw_fvpn), .fill_level_i(ptw_flvl), .fill_nap_i(ptw_fnap), .fill_ppn_i(ptw_fppn), .fill_perm_i(ptw_fperm),
      .fill_fault_i(ptw_ffault), .vam_i(ictx.vam),
        .bres_valid_i(bres_v), .bres_ftq_i(bres_ftq), .bres_off_i(bres_off), .bres_taken_i(bres_taken),
        .bres_mispred_i(bres_mis), .bres_target_i(bres_tgt), .bres_ind_i(bres_ind),
      .cm_ftq_v_i(cmq_ftq_v), .cm_ftq_idx_i(cmq_ftq_idx),
      .ftq_ridx_i(ftq_ridx), .ftq_pc_o(ftq_pc), .ftq_tgt_o(ftq_tgt),
      .ib_valid_o(ib_valid), .ib_inst_o(ib_inst), .ib_pop_i(ib_pop),
      .bist_start_i, .bist_done_o(fe_bd), .bist_fail_o(fe_bf)
    );
  end else begin : g_fe1
    assign fe_bd = 1'b1; assign fe_bf = 1'b0;    // M1 前端没有 SRAM
  o6_frontend_m1 u_fe (
      .clk_i, .rst_ni,
      .imem_req_o(ic_req), .imem_ready_i(ic_ready), .imem_addr_o(ic_addr),
      .imem_resp_valid_i(ic_resp_v), .imem_data_i(ic_resp_d), .imem_kill_o(ic_kill),
      .be_redirect_i(redirect), .be_redirect_pc_i(redirect_pc), .be_redirect_ftq_i(redirect_ftq),
      .cm_flush_i(flush), .cm_redirect_pc_i(cm_redirect_pc),
      .xlate_i(ictx.xl), .priv_i(priv), .dbg_mode_i(dbg_mode), .tlb_flush_i(tlb_flush), .pmpd_i(pmpd),
      .ptw_ireq_o(ptw_ireq), .ptw_ivpn_o(ptw_ivpn), .ptw_iv_o(ptw_iv), .ifill_i(ptw_ifill), .fill_v_i(ptw_fv), .xv_i(ictx.v),
      .cn_i(ictx.cn), .fgx_o(fe_gx), .fill_gperm_i(ptw_fgperm), .fill_gpa_i(ptw_fgpa), .fill_gimp_i(ptw_fgimp),
      .fill_vpn_i(ptw_fvpn), .fill_level_i(ptw_flvl), .fill_nap_i(ptw_fnap), .fill_ppn_i(ptw_fppn), .fill_perm_i(ptw_fperm),
      .fill_fault_i(ptw_ffault), .vam_i(ictx.vam),
      .cm_ftq_v_i(cmq_ftq_v), .cm_ftq_idx_i(cmq_ftq_idx),
      .ftq_ridx_i(ftq_ridx), .ftq_pc_o(ftq_pc), .ftq_tgt_o(ftq_tgt),
      .ib_valid_o(ib_valid), .ib_inst_o(ib_inst), .ib_pop_i(ib_pop)
    );
  end

  //==========================================================================
  // 译码 / 重命名 / 分派
  //==========================================================================
  logic [W-1:0] de_valid;   uop_t [W-1:0] de_uop;   logic rn_ready;
  logic [W-1:0] rn_valid;   uop_t [W-1:0] rn_uop;   logic ds_ready;   ds_need_t rn_need;

  logic [2:0] cbo_ok, cbo_vi;   // Zicbom / Zicboz 当前特权级可执行 / 不可执行时报 virtual (o6_csr -> 译码)
  o6_decode_stage u_de (
    .clk_i, .rst_ni, .flush_i(flush || br_upd.kill),
    .ib_valid_i(ib_valid), .ib_inst_i(ib_inst), .ib_pop_o(ib_pop),
    .priv_i(priv), .fs_i(fs), .vs_i(vs), .frm_i(frm), .cbo_ok_i(cbo_ok), .cbo_vi_i(cbo_vi),
    .virt_i(virt), .hlv_ok_i(hlv_ok), .hlv_vi_i(hlv_vi),
    .out_valid_o(de_valid), .out_uop_o(de_uop), .out_ready_i(rn_ready)
  );

  // 提交 -> 重命名
  logic [W-1:0] cm_valid, cm_rd_valid, cm_rd_fp, cm_is_ld, cm_is_st, cm_is_br;
  logic [4:0]   cm_lrd [W];
  preg_t [W-1:0] cm_pdst, cm_old_pdst;
  logic [W-1:0]  cm_mvel;
  logic [FTQ_W-1:0] cm_ftq [W];
  logic [4:0]   cm_fflags;
  logic         cm_fp_dirty;

  logic [6:0]  iq_free [N_IQ];        // 各发射队列空位 (7 位：IQ 可大于 63 项) (重命名的 ALU 分流、分派的资源判定)
  o6_rename #(.N_IWAKE(N_IWAKE_C), .N_FWAKE(N_FWAKE)) u_rn (
    .clk_i, .rst_ni,
    .in_valid_i(de_valid), .in_uop_i(de_uop), .in_ready_o(rn_ready),
    .out_valid_o(rn_valid), .out_uop_o(rn_uop), .out_need_o(rn_need), .out_ready_i(ds_ready),
    .iq_free_i(iq_free),
    .br_upd_i(br_upd), .flush_i(flush), .iwake_i(iwake_c), .fwake_i(fwake),
    .cm_valid_i(cmq_rd_valid), .cm_is_fp_i(cmq_rd_fp), .cm_lrd_i(cmq_lrd),
    .cm_pdst_i(cmq_pdst), .cm_old_pdst_i(cmq_old_pdst), .cm_mvel_i(cmq_mvel)
  );

  logic        rob_ready, rob_fire;
  rob_ptr_t    rob_ptr;
  logic [W-1:0] rob_dvalid;
  uop_t [W-1:0] rob_duop;
  logic [63:0] rob_dtval [W];
  lq_ptr_t     lq_tail;  sq_ptr_t sq_tail;
  logic [LQ_W:0] lq_free; logic [SQ_W:0] sq_free;
  logic [W-1:0] lq_alloc, sq_alloc;
  uop_t [W-1:0] lsu_auop;
  logic [IQ_ENQ_MAX-1:0] iq_enq_v [N_IQ];
  uop_t [IQ_ENQ_MAX-1:0] iq_enq_u [N_IQ];

  o6_dispatch u_ds (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .in_valid_i(rn_valid), .in_uop_i(rn_uop), .in_need_i(rn_need), .in_ready_o(ds_ready),
    .rob_ready_i(rob_ready), .rob_ptr_i(rob_ptr), .rob_fire_o(rob_fire),
    .rob_valid_o(rob_dvalid), .rob_uop_o(rob_duop), .rob_tval_o(rob_dtval),
    .lq_tail_i(lq_tail), .sq_tail_i(sq_tail), .lq_free_i(lq_free), .sq_free_i(sq_free),
    .lq_alloc_o(lq_alloc), .sq_alloc_o(sq_alloc), .lsu_uop_o(lsu_auop),
    .iq_free_i(iq_free), .iq_enq_valid_o(iq_enq_v), .iq_enq_uop_o(iq_enq_u)
  );

  //==========================================================================
  // ROB 与提交
  //==========================================================================
  logic [N_STD-1:0] std_done;   rob_ptr_t [N_STD-1:0] std_rob;    // [0..N_STDP-1] 整数 STD 口，[N_STDP] 浮点 STD
  logic        head_start, head_start_exu, head_start_lsu;
  logic        vec_req, vec_rsp, vec_excp, vu_idle;
  preg_t       xfp_ra;            // P3 读浮点 PRF (向量指令的标量浮点操作数)
  logic [63:0] xfp_rd;
  logic [4:0]  vu_ff;             // 向量单元的浮点异常标志
  logic [31:0] vec_inst;
  logic [63:0] vec_a, vec_b, vec_rdata, vec_tval;
  logic [5:0]  vec_cause;
  logic        vm_req, vm_ack, vm_we, vm_done, vm_fault;
  logic [63:0]  vm_va;
  logic [511:0] vm_wdata, vm_rdata;
  logic [63:0]  vm_wmask;
  logic [5:0]  vm_cause;
  rob_ptr_t    rob_head;
  logic        head_go;
  rob_ptr_t    rob_head2;           // ROB 第二老项
  logic        head2_go;            // 头是已完成的向量指令 (第二老的向量指令可提前发射)
  logic        ev_valid, ev_trap, ev_replay, ev_irq, ev_rvc, ev_block;
  logic [5:0]  ev_cause;
  logic [63:0] ev_tval, ev_tval2;
  logic [63:0] cmpl_x2 [N_CMPL];     // H：完成口的 guest page fault GPA (只有 LSU 口)
  logic [63:0] lsu_x2 [LSU_NP];
  logic [63:0] trap_tval2, trap_tinst;
  logic [1:0]  ev_tsel;
  logic [FTQ_W-1:0] ev_ftq_idx;
  logic [FTQ_OFF_W-1:0] ev_ftq_off;
  fu_e         ev_fu;
  logic [4:0]  ev_op;
  logic        irq_pending;
  logic [5:0]  irq_cause;
  logic        rob_empty;

  // 调试：停机请求 / 单步完成借中断通路在 ROB 头进入调试模式 (优先于普通中断)
  logic        rob_irq;
  // 调试停机请求寄存一拍 (DM 的寄存器经核内组合逻辑进 ROB 头 -> head_go -> IQ 选择，太长；晚一拍响应无害)
  logic        haltreq_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) haltreq_q <= 1'b0;
    else         haltreq_q <= dbg_haltreq_i;
  logic [5:0]  rob_irq_cause;
  always_comb begin
    rob_irq = irq_pending; rob_irq_cause = irq_cause;
    if (!dbg_mode && haltreq_q)      begin rob_irq = 1'b1; rob_irq_cause = IRQ_DBG_HALT; end
    else if (!dbg_mode && step_pend) begin rob_irq = 1'b1; rob_irq_cause = IRQ_DBG_STEP; end
  end

  assign head_start = head_start_exu || head_start_lsu;

  // 可能报异常的完成口：W4 (CSR) 与 LSU 各流水
  localparam logic [N_CMPL-1:0] XCPT = N_CMPL'(((1 << (LSU_NP + 1)) - 1) << N_ALU);
  o6_rob #(.N_CMPL(N_CMPL), .N_STD(N_STD), .XCPT_PORTS(XCPT)) u_rob (
    .clk_i, .rst_ni,
    .dis_valid_i(rob_dvalid), .dis_uop_i(rob_duop), .dis_tval_i(rob_dtval),
    .dis_fire_i(rob_fire), .dis_ready_o(rob_ready), .dis_ptr_o(rob_ptr),
    .cmpl_i(cmpl), .cmpl_x2_i(cmpl_x2), .std_done_i(std_done), .std_rob_i(std_rob),
    .head_start_i(head_start), .step_i(dbg_step), .viol_valid_i(viol_v), .viol_rob_i(viol_rob),
    .br_upd_i(br_upd), .flush_i(flush),
    .cm_valid_o(cm_valid), .cm_rd_valid_o(cm_rd_valid), .cm_rd_fp_o(cm_rd_fp),
    .cm_lrd_o(cm_lrd), .cm_pdst_o(cm_pdst), .cm_old_pdst_o(cm_old_pdst), .cm_mvel_o(cm_mvel),
    .cm_is_ld_o(cm_is_ld), .cm_is_st_o(cm_is_st), .cm_is_br_o(cm_is_br),
    .cm_ftq_idx_o(cm_ftq), .cm_fflags_o(cm_fflags), .cm_fp_dirty_o(cm_fp_dirty),
    .ev_valid_o(ev_valid), .ev_trap_o(ev_trap), .ev_replay_o(ev_replay), .ev_irq_o(ev_irq),
    .ev_cause_o(ev_cause), .ev_tval_o(ev_tval), .ev_tval2_o(ev_tval2), .ev_tval_sel_o(ev_tsel),
    .ev_ftq_idx_o(ev_ftq_idx), .ev_ftq_off_o(ev_ftq_off), .ev_is_rvc_o(ev_rvc),
    .ev_fu_o(ev_fu), .ev_op_o(ev_op),
    .irq_pending_i(rob_irq), .irq_cause_i(rob_irq_cause), .ev_block_i(ev_block),
    .head_o(rob_head), .head_go_o(head_go), .head2_o(rob_head2), .head2_go_o(head2_go), .empty_o(rob_empty)
  );

  // 提交统计
  logic [2:0] cm_cnt, cm_ld_cnt, cm_st_cnt;
  always_comb begin
    cm_cnt = '0; cm_ld_cnt = '0; cm_st_cnt = '0;
    cm_ftq_v = 1'b0; cm_ftq_idx = '0;
    for (int c = 0; c < W; c++) begin
      cm_cnt    += cm_valid[c];
      cm_ld_cnt += cm_is_ld[c];
      cm_st_cnt += cm_is_st[c];
      if (cm_valid[c]) begin cm_ftq_v = 1'b1; cm_ftq_idx = cm_ftq[c]; end
    end
  end

  // 提交包：ROB 本拍的提交结果寄存一拍再分发给重命名 (提交 RAT / 空闲表)、LSU、CSR、前端 FTQ。
  // ROB 头 -> 本拍能提交几条 -> 各模块当拍更新 原来串在一拍里。提交控制在事件后多等一拍再发冲刷
  // (o6_commit_ctrl 的 S_EV0)，所以提交包总在冲刷之前落地，各模块的冲刷恢复不用改。
  // 单步调试仍用未寄存的提交数 (保证一步只走一条)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cmq_rd_valid <= '0; cmq_cnt <= '0; cmq_ld_cnt <= '0; cmq_st_cnt <= '0; cmq_ftq_v <= 1'b0;
      cmq_fflags <= '0; cmq_fp_dirty <= 1'b0; cmq_is_br <= '0;
    end else begin
      cmq_rd_valid <= cm_rd_valid; cmq_cnt <= cm_cnt; cmq_ld_cnt <= cm_ld_cnt; cmq_st_cnt <= cm_st_cnt;
      cmq_ftq_v <= cm_ftq_v; cmq_fflags <= cm_fflags; cmq_fp_dirty <= cm_fp_dirty; cmq_is_br <= cm_is_br;
    end
  end
  always_ff @(posedge clk_i) begin
    cmq_rd_fp <= cm_rd_fp; cmq_pdst <= cm_pdst; cmq_old_pdst <= cm_old_pdst; cmq_ftq_idx <= cm_ftq_idx;
    cmq_mvel <= cm_mvel;
    for (int c = 0; c < W; c++) cmq_lrd[c] <= cm_lrd[c];
  end

  // 提交控制
  logic        trap, trap_irq, trap_hv;
  logic [5:0]  trap_cause;
  logic [63:0] trap_epc, trap_tval, trap_vec, mepc, sepc;
  logic        fencei_req, sfence, lr_clear, fencei_done;
  logic        sq_drained;

  o6_commit_ctrl u_cc (
    .clk_i, .rst_ni,
    .ev_valid_i(ev_valid), .ev_trap_i(ev_trap), .ev_replay_i(ev_replay), .ev_irq_i(ev_irq),
    .ev_cause_i(ev_cause), .ev_tval_i(ev_tval), .ev_tval_sel_i(ev_tsel),
    .ev_tval2_i(ev_tval2), .fgx_i(fe_gx), .vgx_i(vm_gx),
    .ev_ftq_idx_i(ev_ftq_idx), .ev_ftq_off_i(ev_ftq_off), .ev_is_rvc_i(ev_rvc),
    .ev_fu_i(ev_fu), .ev_op_i(ev_op), .ev_block_o(ev_block),
    .ftq_ridx_o(ftq_ridx[N_BRU]), .ftq_pc_i(ftq_pc[N_BRU]),
    .trap_o(trap), .trap_irq_o(trap_irq), .trap_cause_o(trap_cause),
    .trap_epc_o(trap_epc), .trap_tval_o(trap_tval), .trap_hv_o(trap_hv), .trap_tval2_o(trap_tval2), .trap_tinst_o(trap_tinst),
    .trap_vec_i(trap_vec),
    .mepc_i(mepc), .sepc_i(sepc),
    .dbg_mode_i(dbg_mode), .ebreak_dbg_i(ebreak_dbg), .dpc_i(dpc), .dbg_enter_o(dbg_enter), .dbg_cause_o(dbg_cause),
    .fencei_req_o(fencei_req), .fencei_done_i(fencei_done), .sfence_o(sfence), .sq_drained_i(sq_drained),
    .ptw_busy_i(ptw_busy),
    .flush_o(flush), .redirect_pc_o(cm_redirect_pc), .lr_clear_o(lr_clear)
  );

  // step_arm：离开调试模式后，等 DRET 自己的冲刷过去才开始计 (DRET 的提交不算单步的那一条)
  logic step_arm;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      step_pend <= 1'b0; step_arm <= 1'b0;
    end else begin
      if (dbg_mode)   step_arm <= 1'b0;
      else if (flush) step_arm <= 1'b1;
      if (dbg_enter || dbg_mode)                                 step_pend <= 1'b0;
      else if (dbg_step && step_arm && (cm_cnt != 3'd0 || trap)) step_pend <= 1'b1;
    end
  end

  //==========================================================================
  // 发射队列
  //==========================================================================
  logic [15:0] fu_ok_int [N_ALU], fu_ok_nx_int [N_ALU];
  logic [15:0] fu_ok_lsu [LSU_NP], fu_ok_nx_lsu [LSU_NP];
  logic [15:0] fu_ok_fp  [2];
  logic [1:0]        iss_va, iss_vb, iss_vc, iss_vf;
  logic [N_LDP-1:0]  iss_vm;
  logic [N_STA-1:0]  iss_vs;
  logic [N_STDP-1:0] iss_vd;
  uop_t [1:0]        iss_ua, iss_ub, iss_uc, iss_uf;
  uop_t [N_LDP-1:0]  iss_um;
  uop_t [N_STA-1:0]  iss_us;
  uop_t [N_STDP-1:0] iss_ud;

  // DC 2018 不支持端口连接里的赋值模式 / 数组切片，用中间变量
  logic [15:0] fu_ok_a [2], fu_ok_b [2], fu_ok_c [2], fu_ok_m [2], fu_ok_n [1], fu_ok_s [N_STA], fu_ok_d [N_STDP];
  logic [15:0] fx_a [2], fx_b [2], fx_c [2], fx_m [2], fx_n [1], fx_s [N_STA];   // 下一拍的 fu_ok (IQ 定制宏的请求预测)
  always_comb begin
    fu_ok_a[0] = fu_ok_int[0]; fu_ok_a[1] = fu_ok_int[1];
    fu_ok_b[0] = fu_ok_int[2]; fu_ok_b[1] = fu_ok_int[3];
    fu_ok_c[0] = fu_ok_int[4]; fu_ok_c[1] = fu_ok_int[5];
    fu_ok_m[0] = fu_ok_lsu[0]; fu_ok_m[1] = fu_ok_lsu[1]; fu_ok_n[0] = fu_ok_lsu[2];
    for (int k = 0; k < N_STA; k++) fu_ok_s[k] = fu_ok_lsu[LSU_KS+k];
    for (int k = 0; k < N_STDP; k++) begin fu_ok_d[k] = '0; fu_ok_d[k][FU_STD] = 1'b1; end
    fx_a[0] = fu_ok_nx_int[0]; fx_a[1] = fu_ok_nx_int[1];
    fx_b[0] = fu_ok_nx_int[2]; fx_b[1] = fu_ok_nx_int[3];
    fx_c[0] = fu_ok_nx_int[4]; fx_c[1] = fu_ok_nx_int[5];
    fx_m[0] = fu_ok_nx_lsu[0]; fx_m[1] = fu_ok_nx_lsu[1]; fx_n[0] = fu_ok_nx_lsu[2];
    for (int k = 0; k < N_STA; k++) fx_s[k] = fu_ok_nx_lsu[LSU_KS+k];
  end

  // 空位计数各队列位宽不同，统一补到 6 位
  logic [$clog2(IQ_A_SIZE+1)-1:0] fc_a;
  logic [$clog2(IQ_B_SIZE+1)-1:0] fc_b;
  logic [$clog2(IQ_M_SIZE+1)-1:0] fc_m;
  logic [$clog2(IQ_N_SIZE+1)-1:0] fc_n;
  logic [$clog2(IQ_F_SIZE+1)-1:0] fc_f;
  logic [$clog2(IQ_S_SIZE+1)-1:0] fc_s;
  logic [$clog2(IQ_D_SIZE+1)-1:0] fc_d;
  logic [$clog2(IQ_C_SIZE+1)-1:0] fc_c;
  always_comb begin
    iq_free[IQ_A] = 7'(fc_a); iq_free[IQ_B] = 7'(fc_b); iq_free[IQ_M] = 7'(fc_m);
    iq_free[IQ_F] = 7'(fc_f); iq_free[IQ_S] = 7'(fc_s); iq_free[IQ_D] = 7'(fc_d); iq_free[IQ_C] = 7'(fc_c);
    iq_free[IQ_N] = 7'(fc_n);
  end

  o6_iq #(.SIZE(IQ_C_SIZE), .NP(2), .PBASE(P_ALU4), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqc (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_C]), .enq_uop_i(iq_enq_u[IQ_C]), .free_cnt_o(fc_c),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_c), .fu_ok_nx_i(fx_c), .iss_valid_o(iss_vc), .iss_uop_o(iss_uc)
  );
  o6_iq #(.SIZE(IQ_A_SIZE), .NP(2), .PBASE(P_ALU0), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqa (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_A]), .enq_uop_i(iq_enq_u[IQ_A]), .free_cnt_o(fc_a),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_a), .fu_ok_nx_i(fx_a), .iss_valid_o(iss_va), .iss_uop_o(iss_ua)
  );
  o6_iq #(.SIZE(IQ_B_SIZE), .NP(2), .PBASE(P_ALU2), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqb (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_B]), .enq_uop_i(iq_enq_u[IQ_B]), .free_cnt_o(fc_b),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_b), .fu_ok_nx_i(fx_b), .iss_valid_o(iss_vb), .iss_uop_o(iss_ub)
  );
  // load 队列拆成两个 (2026-10-17，6 GHz)：IQ_M -> load 口 0 / 1，IQ_N -> load 口 2
  logic [1:0] iss_vm01;
  uop_t [1:0] iss_um01;
  logic [0:0] iss_vn;
  uop_t [0:0] iss_un;
  o6_iq #(.SIZE(IQ_M_SIZE), .NP(2), .PBASE(P_LD0), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqm (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_M]), .enq_uop_i(iq_enq_u[IQ_M]), .free_cnt_o(fc_m),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i(lsu_lwake), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_m), .fu_ok_nx_i(fx_m), .iss_valid_o(iss_vm01), .iss_uop_o(iss_um01)
  );
  o6_iq #(.SIZE(IQ_N_SIZE), .NP(1), .PBASE(P_LD0 + 2), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqn (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_N]), .enq_uop_i(iq_enq_u[IQ_N]), .free_cnt_o(fc_n),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i(lsu_lwake), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_n), .fu_ok_nx_i(fx_n), .iss_valid_o(iss_vn), .iss_uop_o(iss_un)
  );
  always_comb begin
    iss_vm[0] = iss_vm01[0]; iss_um[0] = iss_um01[0];
    iss_vm[1] = iss_vm01[1]; iss_um[1] = iss_um01[1];
    iss_vm[2] = iss_vn[0];   iss_um[2] = iss_un[0];
  end
`ifndef SYNTHESIS
  initial assert (N_LDP == 3) else $fatal(1, "core: IQ_M / IQ_N 的拆分按 3 个 load 口写死");
`endif
  o6_iq #(.SIZE(IQ_F_SIZE), .NP(2), .PBASE(P_FP0), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqf (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_F]), .enq_uop_i(iq_enq_u[IQ_F]), .free_cnt_o(fc_f),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_fp), .fu_ok_nx_i(fu_ok_fp), .iss_valid_o(iss_vf), .iss_uop_o(iss_uf)
  );
  o6_iq #(.SIZE(IQ_S_SIZE), .NP(N_STA), .PBASE(P_STA), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqs (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_S]), .enq_uop_i(iq_enq_u[IQ_S]), .free_cnt_o(fc_s),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_s), .fu_ok_nx_i(fx_s), .iss_valid_o(iss_vs), .iss_uop_o(iss_us)
  );
  o6_iq #(.SIZE(IQ_D_SIZE), .NP(N_STDP), .PBASE(P_STD0), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE), .USE_CAM(IQ_CAM)) u_iqd (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd),
    .enq_valid_i(iq_enq_v[IQ_D]), .enq_uop_i(iq_enq_u[IQ_D]), .free_cnt_o(fc_d),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(lsu_ewake), .lwake_i('0), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(rob_head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok_d), .fu_ok_nx_i(fu_ok_d), .iss_valid_o(iss_vd), .iss_uop_o(iss_ud)
  );

`ifndef SYNTHESIS
  // 仿真观察用：各队列的发射按 [队列][端口] 排好 (tb 的 +debug 打印)
  logic [2:0] iss_v [N_IQ];
  uop_t [2:0] iss_u [N_IQ];
  always_comb begin
    for (int q = 0; q < N_IQ; q++) begin iss_v[q] = '0; iss_u[q] = '0; end
    for (int p = 0; p < 2; p++) begin
      iss_v[IQ_A][p] = iss_va[p]; iss_u[IQ_A][p] = iss_ua[p];
      iss_v[IQ_B][p] = iss_vb[p]; iss_u[IQ_B][p] = iss_ub[p];
      iss_v[IQ_C][p] = iss_vc[p]; iss_u[IQ_C][p] = iss_uc[p];
      iss_v[IQ_F][p] = iss_vf[p]; iss_u[IQ_F][p] = iss_uf[p];
    end
    for (int p = 0; p < 2; p++)  begin iss_v[IQ_M][p] = iss_vm01[p]; iss_u[IQ_M][p] = iss_um01[p]; end
    iss_v[IQ_N][0] = iss_vn[0]; iss_u[IQ_N][0] = iss_un[0];
    for (int p = 0; p < N_STDP; p++) begin iss_v[IQ_D][p] = iss_vd[p]; iss_u[IQ_D][p] = iss_ud[p]; end
    for (int p = 0; p < N_STA; p++) begin iss_v[IQ_S][p] = iss_vs[p]; iss_u[IQ_S][p] = iss_us[p]; end
  end
`endif

  //==========================================================================
  // 整数 PRF：读口 [0..7] P0..P3，[8..8+LSU_NP-1] LSU 各流水基址，其后 STD 口
  //==========================================================================
  localparam int RD_LSU = 2 * N_ALU;
  localparam int RD_STD = 2 * N_ALU + LSU_NP;
  preg_t [N_IRD-1:0] prf_ra;
  logic [63:0]       prf_rd [N_IRD];
  logic [N_IWB-1:0]  prf_we;
  preg_t [N_IWB-1:0] prf_wa;
  logic [63:0]       prf_wd [N_IWB];
  preg_t [2*N_ALU-1:0] exu_ra;
  logic [63:0]       exu_rd [2*N_ALU];
  preg_t [LSU_NP-1:0] lsu_ra;
  logic [63:0]       lsu_rd [LSU_NP];
  preg_t [N_STDP-1:0] stdu_ra;
  logic [63:0]       stdu_rd [N_STDP];

  always_comb begin
    for (int r = 0; r < 2 * N_ALU; r++) begin prf_ra[r] = exu_ra[r]; exu_rd[r] = prf_rd[r]; end
    for (int r = 0; r < LSU_NP; r++) begin prf_ra[RD_LSU+r] = lsu_ra[r]; lsu_rd[r] = prf_rd[RD_LSU+r]; end
    for (int r = 0; r < N_STDP; r++) begin prf_ra[RD_STD+r] = stdu_ra[r]; stdu_rd[r] = prf_rd[RD_STD+r]; end
    for (int w = 0; w < N_IWB; w++) begin
      prf_we[w] = iwb[w].valid && iwb[w].wen;
      prf_wa[w] = iwb[w].pdst;
      prf_wd[w] = iwb[w].data;
    end
  end

  o6_prf #(.N(N_IPREG), .NR(N_IRD), .NW(N_IWB), .ZERO_P0(1'b1)) u_iprf (
    .clk_i, .raddr_i(prf_ra), .rdata_o(prf_rd), .wen_i(prf_we), .waddr_i(prf_wa), .wdata_i(prf_wd)
  );

  //==========================================================================
  // 整数执行簇
  //==========================================================================
  wb_t  [N_ALU-1:0] exu_wb;
  wb_t        exu_w6;
  wakeup_t    wake_mul, wake_mul1, wake_w6;
  cmpl_t [N_ALU-1:0] exu_cmpl;
  cmpl_t      w6_cmpl;
  logic       csr_req; uop_t csr_uop; logic [63:0] csr_wd, csr_rs2;
  logic       csr_done, csr_excp; logic [63:0] csr_rdata, csr_tval; logic [5:0] csr_cause;

  // 浮点簇往来
  logic        f2i_v, i2f_v, i2f_rdy, fstd_v;
  uop_t        f2i_u, i2f_u;
  logic [63:0] f2i_d, i2f_d, fstd_d;
  logic [4:0]  f2i_ff;
  logic        f2i_occ;
  wakeup_t     f2i_wk;
  sq_ptr_t     fstd_sq;
  rob_ptr_t    fstd_rob;
  wakeup_t [2:0] fpu_wake;
  cmpl_t   [2:0] fpu_cmpl;

  logic [FTQ_W-1:0] exu_ftq_ridx [N_BRU];
  logic [63:0]      exu_ftq_pc [N_BRU], exu_ftq_tgt [N_BRU];
  always_comb begin
    for (int k = 0; k < N_BRU; k++) begin
      ftq_ridx[k]    = exu_ftq_ridx[k];
      exu_ftq_pc[k]  = ftq_pc[k];
      exu_ftq_tgt[k] = ftq_tgt[k];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) ld_fail_q <= '0;
    else         ld_fail_q <= ld_fail;

  // 整数口带 sn 的发射当拍唤醒：扣下一拍，判定没失败再作为确认唤醒补发。
  // 本拍的 sc 判定失败则这个唤醒本身作废；被分支杀掉的 (本拍或下一拍) 也不补发：
  // 被杀指令的 pdst 可能已经退回 free list 并重新分配置 busy，晚一拍的补发会把新主人的 busy 清掉
  uop_t     [N_ALU-1:0] iss_ui;          // 各整数口本拍发射的 uop (取 br_mask)
  br_mask_t [N_ALU-1:0] awk_bm;
  always_comb begin
    iss_ui[0] = iss_ua[0]; iss_ui[1] = iss_ua[1]; iss_ui[2] = iss_ub[0];
    iss_ui[3] = iss_ub[1]; iss_ui[4] = iss_uc[0]; iss_ui[5] = iss_uc[1];
  end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin
      for (int k = 0; k < N_ALU; k++) awk_q[k].valid <= 1'b0;
    end else begin
      for (int k = 0; k < N_ALU; k++)
        awk_q[k].valid <= iwake[k].valid && (|iwake[k].sn) && !(|(iwake[k].sc & ld_fail)) && !flush &&
                          !br_killed(iss_ui[k].br_mask, br_upd);
    end
  always_ff @(posedge clk_i)
    for (int k = 0; k < N_ALU; k++) begin
      awk_q[k].is_fp <= 1'b0; awk_q[k].pdst <= iwake[k].pdst;
      awk_q[k].sa <= '0; awk_q[k].sn <= iwake[k].sn; awk_q[k].sc <= '0;
      awk_bm[k] <= iss_ui[k].br_mask & ~br_upd.resolve_mask;
    end

  o6_int_exu u_iexu (
    .clk_i, .rst_ni, .flush_i(flush), .ld_fail_i(ld_fail), .ld_fail_q_i(ld_fail_q),
    .iss_valid_i({iss_vc, iss_vb, iss_va}), .iss_uop_i({iss_uc, iss_ub, iss_ua}), .fu_ok_o(fu_ok_int), .fu_ok_nx_o(fu_ok_nx_int),
    .prf_raddr_o(exu_ra), .prf_rdata_i(exu_rd),
    .fprf_raddr_o(xfp_ra), .fprf_rdata_i(xfp_rd), .vu_idle_i(vu_idle),
    .wb_i(iwb), .wb_p_o(exu_wb), .wb_w6_o(exu_w6), .wb_pn_o(exu_wb_pn), .wb_pn_i(iwb_pn),
    .wake_mul_o(wake_mul), .wake_mul1_o(wake_mul1), .wake_w6_o(wake_w6),
    .cmpl_p_o(exu_cmpl), .cmpl_w6_o(w6_cmpl),
    .br_upd_o(br_upd), .br_upd_i(br_upd),
    .redirect_o(redirect), .redirect_pc_o(redirect_pc), .redirect_ftq_o(redirect_ftq),
    .redirect_call_o(redirect_call), .redirect_ret_o(redirect_ret), .redirect_link_o(redirect_link),
    .bres_valid_o(bres_v), .bres_ftq_o(bres_ftq), .bres_off_o(bres_off), .bres_taken_o(bres_taken),
    .bres_mispred_o(bres_mis), .bres_target_o(bres_tgt), .bres_ind_o(bres_ind),
    .ftq_ridx_o(exu_ftq_ridx), .ftq_pc_i(exu_ftq_pc), .ftq_tgt_i(exu_ftq_tgt),
    .csr_req_o(csr_req), .csr_uop_o(csr_uop), .csr_wdata_o(csr_wd), .csr_rs2_o(csr_rs2),
    .csr_done_i(csr_done), .csr_rdata_i(csr_rdata), .csr_excp_i(csr_excp),
    .csr_cause_i(csr_cause), .csr_tval_i(csr_tval),
    .f2i_valid_i(f2i_v), .f2i_uop_i(f2i_u), .f2i_data_i(f2i_d), .f2i_fflags_i(f2i_ff),
    .f2i_occ_i(f2i_occ), .f2i_wk_i(f2i_wk),
    .i2f_valid_o(i2f_v), .i2f_uop_o(i2f_u), .i2f_data_o(i2f_d), .i2f_ready_i(i2f_rdy),
    .vec_req_o(vec_req), .vec_inst_o(vec_inst), .vec_a_o(vec_a), .vec_b_o(vec_b),
    .vec_rsp_i(vec_rsp), .vec_rdata_i(vec_rdata), .vec_excp_i(vec_excp), .vec_cause_i(vec_cause),
    // 向量访存异常的 tval 报指针掩码后的地址 (同 Spike：transformed_vaddr)；非法指令 (cause 2) 的 tval 是编码
    .vec_tval_i((vec_cause == 6'd2) ? vec_tval : pm_x(vec_tval, dctx.pmm, dctx.zx)),
    .ev_block_i(ev_block), .head_start_o(head_start_exu)
  );

`ifndef SYNTHESIS
  // 提前发射的向量指令到 EX 发请求时，它前面的头必须已提交 (它自己是头)
  always_ff @(posedge clk_i)
    if (rst_ni && vec_req) assert (u_iexu.ex_u[3].rob_idx == rob_head) else $error("core: 向量请求时不是 ROB 头");
`endif

  //==========================================================================
  // 向量单元 (RVV 1.0，docs/11_vector_uarch.md)
  //==========================================================================
`ifdef O6_NO_VEC
  o6_vu_stub u_vu (                  // FPGA 不搭载 RVV：向量指令一律非法指令异常
`else
  o6_vu u_vu (
`endif
    .clk_i, .rst_ni,
    .req_v_i(vec_req), .req_inst_i(vec_inst), .req_a_i(vec_a), .req_b_i(vec_b),
    .rsp_v_o(vec_rsp), .rsp_data_o(vec_rdata), .rsp_excp_o(vec_excp), .rsp_cause_o(vec_cause),
    .rsp_tval_o(vec_tval),
    .vm_req_o(vm_req), .vm_ack_i(vm_ack), .vm_we_o(vm_we), .vm_va_o(vm_va), .vm_wdata_o(vm_wdata), .vm_wmask_o(vm_wmask),
    .vm_done_i(vm_done), .vm_rdata_i(vm_rdata), .vm_fault_i(vm_fault), .vm_cause_i(vm_cause),
    .frm_i(frm), .fflags_o(vu_ff),
    .vs_dirty_o(vs_dirty), .idle_o(vu_idle)
  );

  //==========================================================================
  // 整数 STD 口
  //==========================================================================
  logic [N_STDP-1:0]    stdu_v;
  sq_ptr_t [N_STDP-1:0] stdu_sq;
  rob_ptr_t [N_STDP-1:0] stdu_rob;
  logic [63:0]          stdu_d [N_STDP];
  o6_stdu u_stdu (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd), .ld_fail_i(ld_fail), .ld_fail_q_i(ld_fail_q),
    .iss_valid_i(iss_vd), .iss_uop_i(iss_ud),
    .prf_raddr_o(stdu_ra), .prf_rdata_i(stdu_rd), .wb_i(iwb),
    .std_valid_o(stdu_v), .std_sq_o(stdu_sq), .std_rob_o(stdu_rob), .std_data_o(stdu_d)
  );

  // 发射当拍的 1 拍运算唤醒
  function automatic wakeup_t iss_wake(input logic v, input uop_t u, input logic alu_only);
    wakeup_t w;
    w.valid = v && u.rd_class == RC_INT && (u.fu == FU_ALU || (!alu_only && u.fu == FU_BRU));
    w.is_fp = 1'b0;
    w.pdst  = u.pdst;
    w.sa    = '0;
    w.sn    = u.poia;            // 源的 load 判定在下一拍：消费者记下，下一拍判定
    w.sc    = u.pois;            // 源的 load 判定在本拍：失败则这个唤醒作废
    return w;
  endfunction

  //==========================================================================
  // LSU
  //==========================================================================
  wb_t     [N_LDP-1:0] lsu_iwb, lsu_fwb;
  wakeup_t [N_LDP-1:0] lsu_iwake, lsu_fwake;
  cmpl_t   [LSU_NP-1:0] lsu_cmpl;

  logic [N_LDP-1:0] dc_ld_req, dc_ld_kill, dc_ld_hit, dc_ld_nack;
  logic [PA_W-1:0]  dc_ld_addr [N_LDP], dc_ld_paddr [N_LDP];
  logic [63:0]      dc_ld_data [N_LDP];
  logic [MSHR_W-1:0] dc_ld_mshr [N_LDP];
  logic             dc_refill;
  logic [MSHR_W-1:0] dc_refill_mshr;
  logic             dc_st_req, dc_st_amo, dc_st_done;
  logic [PA_W-1:0]  dc_st_paddr, dc_st_next_paddr, dc_st_lk_paddr;
  logic [63:0]      dc_st_data, dc_st_rdata;
  logic             dc_st_line;
  logic [511:0]     dc_st_ldata, dc_st_lrdata;
  logic [63:0]      dc_st_lmask;
  logic [7:0]       dc_st_mask;
  logic [4:0]       dc_st_amo_op;
  logic [1:0]       dc_st_size;

  // STD：[0..N_STDP-1] 整数 STD 口，[N_STDP] 浮点 STD
  logic [N_STD-1:0]    lsu_std_v;
  sq_ptr_t [N_STD-1:0] lsu_std_sq;
  logic [63:0]         lsu_std_d [N_STD];
  // LSU 的发射输入：[0..N_LDP-1] load (IQ_M)，[LSU_KS] STA (IQ_S)
  logic [LSU_NP-1:0]   lsu_iss_v;
  uop_t [LSU_NP-1:0]   lsu_iss_u;
  always_comb begin
    for (int k = 0; k < N_STDP; k++) begin
      lsu_std_v[k] = stdu_v[k]; lsu_std_sq[k] = stdu_sq[k]; lsu_std_d[k] = stdu_d[k];
    end
    lsu_std_v[N_STDP] = fstd_v; lsu_std_sq[N_STDP] = fstd_sq; lsu_std_d[N_STDP] = fstd_d;
    for (int k = 0; k < N_LDP; k++) begin lsu_iss_v[k] = iss_vm[k]; lsu_iss_u[k] = iss_um[k]; end
    for (int k = 0; k < N_STA; k++) begin lsu_iss_v[LSU_KS+k] = iss_vs[k]; lsu_iss_u[LSU_KS+k] = iss_us[k]; end
  end

  o6_lsu u_lsu (
    .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd), .dbg_mode_i(dbg_mode),
    .lq_alloc_i(lq_alloc), .sq_alloc_i(sq_alloc), .alloc_uop_i(lsu_auop),
    .lq_tail_o(lq_tail), .sq_tail_o(sq_tail), .lq_free_o(lq_free), .sq_free_o(sq_free),
    .iss_valid_i(lsu_iss_v), .iss_uop_i(lsu_iss_u), .fu_ok_o(fu_ok_lsu), .fu_ok_nx_o(fu_ok_nx_lsu),
    .prf_raddr_o(lsu_ra), .prf_rdata_i(lsu_rd), .iwb_i(iwb),
    .std_valid_i(lsu_std_v), .std_sq_i(lsu_std_sq), .std_data_i(lsu_std_d),
    .iwb_o(lsu_iwb), .iwb_pn_o(lsu_iwb_pn), .fwb_o(lsu_fwb), .iwake_o(lsu_iwake), .fwake_o(lsu_fwake), .cmpl_o(lsu_cmpl),
    .cwake_o(lsu_cwake), .ewake_o(lsu_ewake), .lwake_o(lsu_lwake), .ld_fail_o(ld_fail), .ld_fail_q_i(ld_fail_q),
    .viol_valid_o(viol_v), .viol_rob_o(viol_rob), .viol_any_o(viol_any), .viol_ftq_o(viol_ftq), .viol_off_o(viol_off),
    .dc_pf_req_o(dc_pf_req), .dc_pf_paddr_o(dc_pf_paddr),
    .cm_ld_cnt_i(cmq_ld_cnt), .cm_st_cnt_i(cmq_st_cnt),
    .rob_head_i(rob_head), .head_go_i(head_go), .head_start_o(head_start_lsu),
    .dc_ld_req_o(dc_ld_req), .dc_ld_addr_o(dc_ld_addr), .dc_ld_paddr_o(dc_ld_paddr), .dc_ld_kill_o(dc_ld_kill),
    .dc_ld_hit_i(dc_ld_hit), .dc_ld_nack_i(dc_ld_nack), .dc_ld_data_i(dc_ld_data), .dc_ld_mshr_i(dc_ld_mshr),
    .dc_refill_i(dc_refill), .dc_refill_mshr_i(dc_refill_mshr),
    .dc_st_req_o(dc_st_req), .dc_st_paddr_o(dc_st_paddr), .dc_st_next_paddr_o(dc_st_next_paddr), .dc_st_lk_paddr_o(dc_st_lk_paddr), .dc_st_data_o(dc_st_data), .dc_st_mask_o(dc_st_mask),
    .dc_st_amo_o(dc_st_amo), .dc_st_amo_op_o(dc_st_amo_op), .dc_st_size_o(dc_st_size),
    .dc_st_done_i(dc_st_done), .dc_st_rdata_i(dc_st_rdata),
    .dc_st_line_o(dc_st_line), .dc_st_ldata_o(dc_st_ldata), .dc_st_lmask_o(dc_st_lmask), .dc_st_lrdata_i(dc_st_lrdata),
    .mmio_req_o, .mmio_we_o, .mmio_addr_o, .mmio_size_o, .mmio_wdata_o, .mmio_wmask_o,
    .mmio_ack_i, .mmio_rdata_i,
    .sq_drained_o(sq_drained), .lr_clear_i(lr_clear),
    .vm_req_i(vm_req), .vm_ack_o(vm_ack), .vm_we_i(vm_we), .vm_va_i(vm_va), .vm_wdata_i(vm_wdata), .vm_wmask_i(vm_wmask),
    .vm_done_o(vm_done), .vm_rdata_o(vm_rdata), .vm_fault_o(vm_fault), .vm_cause_o(vm_cause),
    .dctx_i(dctx), .hctx_i(hctx), .tlb_flush_i(tlb_flush),
    .pmpd_i(pmpd),
    .ptw_dreq_o(ptw_dreq), .ptw_dvpn_o(ptw_dvpn), .ptw_dv_o(ptw_dv), .dfill_i(ptw_dfill), .fill_v_i(ptw_fv),
    .fill_vpn_i(ptw_fvpn), .fill_level_i(ptw_flvl), .fill_nap_i(ptw_fnap), .fill_ppn_i(ptw_fppn), .fill_perm_i(ptw_fperm),
    .fill_fault_i(ptw_ffault), .fill_gperm_i(ptw_fgperm), .fill_gpa_i(ptw_fgpa), .fill_gimp_i(ptw_fgimp),
    .cmpl_x2_o(lsu_x2), .vm_x2_o(vm_gx),
    .ptw_mem_req_i(ptw_mreq), .ptw_mem_addr_i(ptw_maddr), .ptw_mem_gnt_o(ptw_mgnt),
    .ptw_mem_valid_o(ptw_mvalid), .ptw_mem_data_o(ptw_mdata)
  );

  //==========================================================================
  // PTW
  //==========================================================================
  assign tlb_flush = sfence || satp_wr;

  o6_ptw u_ptw (
    .clk_i, .rst_ni, .flush_i(tlb_flush), .satp_ppn_i(satp[43:0]), .vam_i(satp[61:60]), .pbmte_i(pbmte), .pmpd_i(pmpd),
    .vsatp_ppn_i(vsatp[43:0]), .vvam_i(vsatp[61:60]), .vson_i(vsatp[63]), .vpbmte_i(vpbmte),
    .hgmode_i(hgatp[63:60]), .hgppn_i({hgatp[43:2], 2'b00}),
    .ireq_i(ptw_ireq), .ireq_vpn_i(ptw_ivpn), .ireq_v_i(ptw_iv), .dreq_i(ptw_dreq), .dreq_vpn_i(ptw_dvpn), .dreq_v_i(ptw_dv),
    .busy_o(ptw_busy),
    .mem_req_o(ptw_mreq), .mem_addr_o(ptw_maddr), .mem_gnt_i(ptw_mgnt), .mem_valid_i(ptw_mvalid),
    .mem_data_i(ptw_mdata),
    .ifill_o(ptw_ifill), .dfill_o(ptw_dfill), .fill_v_o(ptw_fv), .fill_vpn_o(ptw_fvpn), .fill_level_o(ptw_flvl), .fill_nap_o(ptw_fnap),
    .fill_ppn_o(ptw_fppn), .fill_perm_o(ptw_fperm), .fill_pbmt_o(ptw_fpbmt), .fill_fault_o(ptw_ffault),
    .fill_gperm_o(ptw_fgperm), .fill_gpa_o(ptw_fgpa), .fill_gimp_o(ptw_fgimp)
  );

  //==========================================================================
  // 汇总：唤醒 / 写回 / 完成 / STD
  //==========================================================================
  always_comb begin
    iwake[0] = iss_wake(iss_va[0], iss_ua[0], 1'b0);
    iwake[1] = iss_wake(iss_va[1], iss_ua[1], 1'b0);
    // P2 = ALU + BRU + MUL，P3 = ALU + MUL + ...：MUL 在 EX1 的那拍本端口不发 1 拍运算，唤醒槽让给 MUL
    iwake[2] = wake_mul.valid  ? wake_mul  : iss_wake(iss_vb[0], iss_ub[0], 1'b0);
    iwake[3] = wake_mul1.valid ? wake_mul1 : iss_wake(iss_vb[1], iss_ub[1], 1'b1);
    iwake[4] = iss_wake(iss_vc[0], iss_uc[0], 1'b1);        // P4 / P5：纯 ALU
    iwake[5] = iss_wake(iss_vc[1], iss_uc[1], 1'b1);
    iwake[IWB_LONG] = wake_w6;
    for (int k = 0; k < N_LDP; k++) iwake[IWB_LD0+k] = lsu_iwake[k];
    // 给重命名的：去掉本拍判定失败的 sc 唤醒，带 sn (下一拍才判定) 的先扣下、下一拍补发 (awk_q)，
    // load 换成 L2 的确认唤醒 (busy 表不能吃投机唤醒)
    for (int k = 0; k < N_ALU + 1; k++) begin
      iwake_c[k]       = iwake[k];
      iwake_c[k].valid = iwake[k].valid && !(|(iwake[k].sc & ld_fail)) && !(|iwake[k].sn);
      iwake_c[k].sa    = '0;
      iwake_c[k].sn    = '0;
      iwake_c[k].sc    = '0;
    end
    for (int k = 0; k < N_LDP; k++) iwake_c[IWB_LD0+k] = lsu_cwake[k];
    for (int k = 0; k < N_ALU; k++) begin
      iwake_c[N_IWAKE+k]       = awk_q[k];
      iwake_c[N_IWAKE+k].valid = awk_q[k].valid && !(|(awk_q[k].sn & ld_fail)) && !br_killed(awk_bm[k], br_upd) && !flush;
      iwake_c[N_IWAKE+k].sn    = '0;
      cwake[N_LDP+k]           = iwake_c[N_IWAKE+k];
    end
    for (int k = 0; k < N_LDP; k++) cwake[k] = lsu_cwake[k];
    for (int k = 0; k < 3; k++)     fwake[k]   = fpu_wake[k];
    for (int k = 0; k < N_LDP; k++) fwake[3+k] = lsu_fwake[k];

    for (int k = 0; k < N_ALU; k++) iwb[k] = exu_wb[k];
    iwb[IWB_LONG] = exu_w6;
    for (int k = 0; k < N_LDP; k++) iwb[IWB_LD0+k] = lsu_iwb[k];
    for (int k = 0; k <= N_ALU; k++) iwb_pn[k] = exu_wb_pn[k];
    for (int k = 0; k < N_LDP; k++)  iwb_pn[IWB_LD0+k] = lsu_iwb_pn[k];

    for (int k = 0; k < N_ALU; k++)  cmpl[k] = exu_cmpl[k];
    cmpl[N_ALU] = w6_cmpl;
    for (int k = 0; k < LSU_NP; k++) cmpl[N_ALU+1+k] = lsu_cmpl[k];
    for (int k = 0; k < N_CMPL; k++) cmpl_x2[k] = '0;
    for (int k = 0; k < LSU_NP; k++) cmpl_x2[N_ALU+1+k] = lsu_x2[k];
    for (int k = 0; k < 3; k++)      cmpl[N_ALU+1+LSU_NP+k] = fpu_cmpl[k];

    for (int k = 0; k < N_STDP; k++) begin std_done[k] = stdu_v[k]; std_rob[k] = stdu_rob[k]; end
    std_done[N_STDP] = fstd_v;  std_rob[N_STDP] = fstd_rob;
  end

  //==========================================================================
  // 浮点簇
  //==========================================================================
  if (HAS_FPU) begin : g_fpu
    o6_fpu_exu u_fpu (
      .clk_i, .rst_ni, .flush_i(flush), .br_upd_i(br_upd), .frm_i(frm),
      .xrd_addr_i(xfp_ra), .xrd_data_o(xfp_rd),
      .iss_valid_i(iss_vf), .iss_uop_i(iss_uf), .fu_ok_o(fu_ok_fp),
      .i2f_valid_i(i2f_v), .i2f_uop_i(i2f_u), .i2f_data_i(i2f_d), .i2f_ready_o(i2f_rdy),
      .fld_wb_i(lsu_fwb),
      .fwb_o(), .fwake_o(fpu_wake), .cmpl_o(fpu_cmpl),
      .f2i_valid_o(f2i_v), .f2i_uop_o(f2i_u), .f2i_data_o(f2i_d), .f2i_fflags_o(f2i_ff),
      .f2i_occ_o(f2i_occ), .f2i_wk_o(f2i_wk),
      .std_valid_o(fstd_v), .std_sq_o(fstd_sq), .std_rob_o(fstd_rob), .std_data_o(fstd_d)
    );
  end else begin : g_nofpu
    always_comb begin
      fu_ok_fp[0] = '0; fu_ok_fp[1] = '0;
      i2f_rdy = 1'b0;
      fpu_wake = '0; fpu_cmpl = '0;
      f2i_v = 1'b0; f2i_u = '0; f2i_d = '0; f2i_ff = '0; f2i_occ = 1'b0; f2i_wk = '0;
      fstd_v = 1'b0; fstd_sq = '0; fstd_rob = '0; fstd_d = '0;
    end
  end

  //==========================================================================
  // L1 缓存与下游仲裁
  //==========================================================================
  // fence.i：等已提交 store 排空 -> D$ 写回全部脏行 -> I$ 全部作废 -> 完成
  typedef enum logic [1:0] {FI_IDLE, FI_DFLUSH, FI_IINV, FI_DONE} fi_e;
  fi_e  fi_st;
  logic dc_flush_done;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) fi_st <= FI_IDLE;
    else unique case (fi_st)
      FI_IDLE:   if (fencei_req && sq_drained) fi_st <= FI_DFLUSH;
      FI_DFLUSH: if (dc_flush_done) fi_st <= FI_IINV;
      FI_IINV:   fi_st <= FI_DONE;
      FI_DONE:   fi_st <= FI_IDLE;       // commit_ctrl 在 done 那拍发冲刷
      default:   fi_st <= FI_IDLE;
    endcase
  end
  assign fencei_done = (fi_st == FI_DONE);

  logic          icm_req_v, icm_req_rdy, icm_resp_v;
  logic [PA_W-1:6] icm_addr;
  logic          dcm_req_v, dcm_req_rdy, dcm_we, dcm_resp_v, dcm_resp_rdy;
  logic [PA_W-1:6] dcm_addr;
  logic [511:0]  dcm_wdata;
  logic [3:0]    dcm_id;

  logic ic_bd, ic_bf, dc_bd, dc_bf;
  assign bist_done_o = ic_bd && dc_bd && fe_bd;
  assign bist_fail_o = ic_bf || dc_bf || fe_bf;

  o6_icache u_ic (
    .clk_i, .rst_ni,
    .req_valid_i(ic_req), .req_ready_o(ic_ready), .req_addr_i(ic_addr[PA_W-1:0]), .kill_i(ic_kill),
    .resp_valid_o(ic_resp_v), .resp_data_o(ic_resp_d),
    .inval_i(fi_st == FI_IINV),
    .mem_req_valid_o(icm_req_v), .mem_req_ready_i(icm_req_rdy), .mem_req_addr_o(icm_addr),
    .mem_resp_valid_i(icm_resp_v), .mem_resp_data_i(mem_resp_data_i),
    .bist_start_i, .bist_done_o(ic_bd), .bist_fail_o(ic_bf)
  );

  o6_dcache u_dc (
    .clk_i, .rst_ni,
    .ld_req_i(dc_ld_req), .ld_addr_i(dc_ld_addr), .ld_paddr_i(dc_ld_paddr), .ld_kill_i(dc_ld_kill),
    .ld_hit_o(dc_ld_hit), .ld_nack_o(dc_ld_nack), .ld_data_o(dc_ld_data), .ld_mshr_o(dc_ld_mshr),
    .refill_o(dc_refill), .refill_mshr_o(dc_refill_mshr),
    .st_req_i(dc_st_req), .st_paddr_i(dc_st_paddr), .st_next_paddr_i(dc_st_next_paddr), .st_lk_paddr_i(dc_st_lk_paddr), .st_data_i(dc_st_data), .st_mask_i(dc_st_mask),
    .st_amo_i(dc_st_amo), .st_amo_op_i(dc_st_amo_op), .st_size_i(dc_st_size),
    .st_done_o(dc_st_done), .st_rdata_o(dc_st_rdata),
    .st_line_i(dc_st_line), .st_ldata_i(dc_st_ldata), .st_lmask_i(dc_st_lmask), .st_lrdata_o(dc_st_lrdata),
    .pf_req_i(dc_pf_req), .pf_paddr_i(dc_pf_paddr),
    .flush_req_i(fi_st == FI_DFLUSH), .flush_done_o(dc_flush_done),
    .mem_req_valid_o(dcm_req_v), .mem_req_ready_i(dcm_req_rdy), .mem_req_we_o(dcm_we),
    .mem_req_addr_o(dcm_addr), .mem_req_data_o(dcm_wdata), .mem_req_id_o(dcm_id),
    .mem_resp_valid_i(dcm_resp_v), .mem_resp_ready_o(dcm_resp_rdy),
    .mem_resp_id_i(mem_resp_id_i), .mem_resp_data_i(mem_resp_data_i),
    .bist_start_i, .bist_done_o(dc_bd), .bist_fail_o(dc_bf)
  );

  // 请求仲裁：两边都有时轮流；但 D$ 写回总是优先。
  //   L2 响应 FIFO 非空时只收写回不收读，而 D$ 有待发写回时不收响应：若仲裁一直挂着 I$ 的读，
  //   写回发不出去、响应取不走、读也进不去 -> 死锁 (SoC 模式 rv64ui-v-sb 等复现)。
  logic arb_last_i;
  wire  dc_wb  = dcm_req_v && dcm_we;
  wire  pick_i = icm_req_v && !dc_wb && (!dcm_req_v || !arb_last_i);
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) arb_last_i <= 1'b0;
    else if (mem_req_valid_o && mem_req_ready_i) arb_last_i <= pick_i;
  end
  assign mem_req_valid_o = icm_req_v || dcm_req_v;
  assign mem_req_we_o    = pick_i ? 1'b0 : dcm_we;
  assign mem_req_addr_o  = pick_i ? icm_addr : dcm_addr;
  assign mem_req_data_o  = dcm_wdata;
  assign mem_req_id_o    = pick_i ? MEM_ID_IC : dcm_id;
  assign icm_req_rdy     = mem_req_ready_i && pick_i;
  assign dcm_req_rdy     = mem_req_ready_i && !pick_i;
  // 响应按请求号分发：15 = I$ (只有一个在途请求，总能接收)，其余 = D$ 的 MSHR
  assign icm_resp_v       = mem_resp_valid_i && mem_resp_id_i == MEM_ID_IC;
  assign dcm_resp_v       = mem_resp_valid_i && mem_resp_id_i != MEM_ID_IC;
  assign mem_resp_ready_o = (mem_resp_id_i == MEM_ID_IC) ? 1'b1 : dcm_resp_rdy;

  //==========================================================================
  // CSR
  //==========================================================================
  // mhpmevent 事件号 (1..8)：1 ROB 空  2 提交了分支  3 分支误预测重定向  4 I$ 缺失 (向下游发读)
  //   5 D$ 缺失 (向下游发读)  6 提交级冲刷  7 分派停顿  8 取指缓冲空
  logic [7:0] hpm_ev;
  assign hpm_ev = {ib_valid == '0, (|rn_valid) && !ds_ready, flush,
                   dcm_req_v && dcm_req_rdy && !dcm_we, icm_req_v && icm_req_rdy,
                   redirect, cm_is_br != '0, rob_empty};

  o6_csr #(.HAS_FPU(HAS_FPU), .HAS_MMU(HAS_MMU)) u_csr (
    .clk_i, .rst_ni,
    .ex_req_i(csr_req), .ex_uop_i(csr_uop), .ex_wdata_i(csr_wd),
    .ex_done_o(csr_done), .ex_rdata_o(csr_rdata), .ex_excp_o(csr_excp),
    .ex_cause_o(csr_cause), .ex_tval_o(csr_tval),
    .trap_i(trap), .trap_irq_i(trap_irq), .trap_cause_i(trap_cause),
    .trap_epc_i(trap_epc), .trap_tval_i(trap_tval), .trap_vec_o(trap_vec),
    .trap_tval2_i(trap_tval2), .trap_tinst_i(trap_tinst), .trap_hv_i(trap_hv),
    .dbg_enter_i(dbg_enter), .dbg_cause_i(dbg_cause), .dbg_pc_i(trap_epc),
    .dbg_mode_o(dbg_mode), .dpc_o(dpc), .dbg_step_o(dbg_step), .ebreak_dbg_o(ebreak_dbg),
    .cm_cnt_i(cmq_cnt), .cm_fflags_i(cmq_fflags), .cm_fp_dirty_i(cmq_fp_dirty),
    .mtip_i, .msip_i, .meip_i, .seip_i, .mtime_i,
    .irq_pending_o(irq_pending), .irq_cause_o(irq_cause),
    .priv_o(priv), .virt_o(virt), .hlv_ok_o(hlv_ok), .hlv_vi_o(hlv_vi),
    .fs_o(fs), .vs_o(vs), .vs_dirty_i(vs_dirty), .vfflags_i(vu_ff), .frm_o(frm),
    .ictx_o(ictx), .dctx_o(dctx), .hctx_o(hctx),
    .satp_o(satp), .vsatp_o(vsatp), .hgatp_o(hgatp), .satp_wr_o(satp_wr), .pbmte_o(pbmte), .vpbmte_o(vpbmte),
    .pmpcfg_o(pmpcfg), .pmpaddr_o(pmpaddr), .mepc_o(mepc), .sepc_o(sepc), .cbo_ok_o(cbo_ok), .cbo_vi_o(cbo_vi),
    .hpm_event_i(hpm_ev)
  );

  o6_pmp_dec #(.N(8)) u_pmp_dec (.cfg_i(pmpcfg), .addr_i(pmpaddr), .dec_o(pmpd_c));
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) for (int i = 0; i < 8; i++) pmpd[i] <= '0;     // 复位时 pmpcfg = 0 (全 OFF)
    else         pmpd <= pmpd_c;

  wire unused = (|bres_v) ^ (|csr_rs2);
endmodule
