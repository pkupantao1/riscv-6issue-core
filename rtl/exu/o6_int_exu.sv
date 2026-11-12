//==============================================================================
// o6_int_exu.sv - 整数执行簇：端口 P0..P3
//
// 每端口流水：IS (发射队列选择) -> RF (读 PRF + 旁路) -> EX (再旁路 + 运算) -> WB (wbreg 写 PRF)
//   wbreg 在它存在的那一拍既写 PRF 又作为旁路源，所以 RF 和 EX 两处都只需比较 wbreg。
//
// 唤醒时机 (消费者可在"唤醒那拍 + 1"被选中)：
//   ALU / BRU (1 拍)  ：发射当拍 (由顶层从发射队列输出组合产生)
//   MUL (3 拍)        ：发射后第 2 拍 (本模块产生，占用所在端口 P2 / P3 的唤醒/写回槽)
//   DIV / CSR / F2I   ：进入 W6 的那一拍
//
// 端口：P0 / P1 = ALU + BRU；P2 = ALU + BRU + MUL；P3 = ALU + MUL + DIV + CSR + I2F；P4 / P5 = ALU
// (整数 STD 2026-10-13 起走专用口，见 o6_stdu)
// P2 / P3 写回冲突：MUL 在 t 发射，则 t+2 不能在同一端口发射 1 拍运算 (fu_ok 屏蔽)。
//
// 分支：P0/P1/P2 在 EX 解析，结果在下一拍以 br_update 广播 (寄存器输出)。
//   同拍多个误预测取最老的；被本拍广播作废的分支不参与解析。
//==============================================================================
`default_nettype none
module o6_int_exu
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  wire  [N_LDP-1:0]    ld_fail_i,         // load 投机唤醒失败 (本拍)：RF 级丢弃带 poia 的 uop (见 o6_iq)
  input  wire  [N_LDP-1:0]    ld_fail_q_i,       // load 投机唤醒失败 (寄存)：RF 级丢弃带 pois 的 uop

  // 发射 (来自 IQ_A: P0/P1，IQ_B: P2/P3)
  input  wire  [N_ALU-1:0]    iss_valid_i,
  input  var uop_t [N_ALU-1:0] iss_uop_i,
  output logic [15:0]         fu_ok_o [N_ALU],
  output logic [15:0]         fu_ok_nx_o [N_ALU],   // 下一拍的 fu_ok：MUL 屏蔽按本拍进 MUL 的算准，其余取本拍值

  // PRF 读口 (每端口 2 个)
  output preg_t [2*N_ALU-1:0] prf_raddr_o,
  input  wire  [63:0]         prf_rdata_i [2*N_ALU],
  // 浮点 PRF 读口 (P3 的 RF 级)：向量指令的标量浮点操作数 (OPFVF 的 f[rs1])。
  // FU_VEC 只在 ROB 头发射，比它老的都已写回，不需要旁路
  output preg_t               fprf_raddr_o,
  input  wire  [63:0]         fprf_rdata_i,
  input  wire                 vu_idle_i,         // 向量单元空闲 (标量 CSR 指令等它：fflags / FS 由 VU 迟到地更新)

  // 写回寄存器 (旁路源)：W0..W6
  input  var wb_t [N_IWB-1:0] wb_i,          // 全部 7 个 (W4/W5 来自 LSU，W0-3/W6 来自本模块的输出回环)
  output wb_t  [N_ALU-1:0]    wb_p_o,        // 各整数口的写回
  // 各写回口下一拍的目的寄存器号 (= 写回寄存器 pdst 的 D 端)：W0..W5 / W6 由本模块给 (wb_pn_o)，
  // load 口由 LSU 给；核顶层拼好送回 wb_pn_i。EX 级旁路的比较因此提前到 RF 级 (见 "EX 级" 处)
  output preg_t [N_ALU:0]     wb_pn_o,
  input  var preg_t [N_IWB-1:0] wb_pn_i,
  output wb_t                 wb_w6_o,       // W6

  // 唤醒 (本模块产生的：P2 上的 MUL、W6)
  output wakeup_t             wake_mul_o,    // P2 的 MUL
  output wakeup_t             wake_mul1_o,   // P3 的 MUL
  output wakeup_t             wake_w6_o,

  // 完成
  output cmpl_t [N_ALU-1:0]   cmpl_p_o,
  output cmpl_t               cmpl_w6_o,

  // 分支
  output br_update_t          br_upd_o,
  input  var br_update_t      br_upd_i,      // 即 br_upd_o (回环)，统一由顶层连
  output logic                redirect_o,
  output logic [63:0]         redirect_pc_o,
  output logic [FTQ_W-1:0]    redirect_ftq_o,
  // 解析结果写回 FTQ (预测器提交时更新用)
  output logic                redirect_call_o,   // 重定向的那条是调用 (前端补做压栈)
  output logic                redirect_ret_o,    // 重定向的那条是返回 (前端补做弹栈)
  output logic [63:0]         redirect_link_o,   // 调用的返回地址
  output logic [N_BRU-1:0]    bres_valid_o,
  output logic [FTQ_W-1:0]    bres_ftq_o [N_BRU],
  output logic [FTQ_OFF_W-1:0] bres_off_o [N_BRU],
  output logic [N_BRU-1:0]    bres_taken_o,
  output logic [N_BRU-1:0]    bres_mispred_o,
  output logic [63:0]         bres_target_o [N_BRU],
  output logic [N_BRU-1:0]    bres_ind_o,        // 非返回的 JALR (前端 ITTAGE 训练)

  // FTQ 读 (P0/P1/P2 的 RF 级)
  output logic [FTQ_W-1:0]    ftq_ridx_o [N_BRU],
  input  wire  [63:0]         ftq_pc_i [N_BRU],      // 块起始 PC
  input  wire  [63:0]         ftq_tgt_i [N_BRU],     // 块预测目标

  // CSR 单元
  output logic                csr_req_o,
  output uop_t                csr_uop_o,
  output logic [63:0]         csr_wdata_o,     // rs1 值 (或 zimm)
  output logic [63:0]         csr_rs2_o,       // SFENCE.VMA 的 asid
  input  wire                 csr_done_i,      // 组合返回 (同拍)
  input  wire  [63:0]         csr_rdata_i,
  input  wire                 csr_excp_i,
  input  wire  [5:0]          csr_cause_i,
  input  wire  [63:0]         csr_tval_i,

  // F2I (浮点 -> 整数) 结果，优先占 W6
  input  wire                 f2i_valid_i,
  input  var uop_t            f2i_uop_i,
  input  wire                 f2i_occ_i,      // W6 被 F2I 路径占用 (早到，含被杀的)
  input  var wakeup_t         f2i_wk_i,
  input  wire  [63:0]         f2i_data_i,
  input  wire  [4:0]          f2i_fflags_i,

  // I2F (整数 -> 浮点) 送浮点簇
  output logic                i2f_valid_o,
  output uop_t                i2f_uop_o,
  output logic [63:0]         i2f_data_o,
  input  wire                 i2f_ready_i,

  // 向量单元 (ROB 头的 FU_VEC，见 o6_vu)：EX 级发请求，回完成后结果走 W6
  output logic                vec_req_o,
  output logic [31:0]         vec_inst_o,
  output logic [63:0]         vec_a_o,
  output logic [63:0]         vec_b_o,
  input  wire                 vec_rsp_i,
  input  wire  [63:0]         vec_rdata_i,
  input  wire                 vec_excp_i,
  input  wire  [5:0]          vec_cause_i,
  input  wire  [63:0]         vec_tval_i,

  // 头部执行状态
  input  wire                 ev_block_i,       // 提交级正在处理事件 (陷入 / 中断 / flush_after)
  output logic                head_start_o      // CSR / 向量指令进入 EX (启动)
);
  //--------------------------------------------------------------------------
  // 旁路
  //--------------------------------------------------------------------------
  function automatic logic [63:0] byp(input preg_t p, input logic [63:0] v, input wb_t [N_IWB-1:0] wb);
    logic [63:0] r;
    r = v;
    for (int k = 0; k < N_IWB; k++)
      if (wb[k].valid && wb[k].wen && wb[k].pdst == p && p != '0) r = wb[k].data;
    return r;
  endfunction

  //--------------------------------------------------------------------------
  // RF 级
  //--------------------------------------------------------------------------
  logic [N_ALU-1:0] rf_v;
  uop_t        rf_u [N_ALU];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) rf_v <= '0;
    else if (flush_i) rf_v <= '0;
    else for (int p = 0; p < N_ALU; p++)
      rf_v[p] <= iss_valid_i[p] && !br_killed(iss_uop_i[p].br_mask, br_upd_i);
  end
  always_ff @(posedge clk_i) begin
    for (int p = 0; p < N_ALU; p++) begin
      uop_t n;
      n = iss_uop_i[p]; n.br_mask = iss_uop_i[p].br_mask & ~br_upd_i.resolve_mask;
      rf_u[p] <= n;
    end
  end

  always_comb begin
    for (int p = 0; p < N_ALU; p++) begin
      prf_raddr_o[2*p]   = rf_u[p].psrc1;
      prf_raddr_o[2*p+1] = rf_u[p].psrc2;
    end
    for (int p = 0; p < N_BRU; p++) ftq_ridx_o[p] = rf_u[p].ftq_idx;
  end

  logic [63:0] rf_a [N_ALU], rf_b [N_ALU];
  always_comb begin
    for (int p = 0; p < N_ALU; p++) begin
      rf_a[p] = (rf_u[p].rs1_class == RC_NONE) ? 64'd0 :
                (p == 3 && rf_u[p].rs1_class == RC_FP) ? fprf_rdata_i : byp(rf_u[p].psrc1, prf_rdata_i[2*p], wb_i);
      rf_b[p] = (rf_u[p].rs2_class == RC_NONE) ? 64'd0 : byp(rf_u[p].psrc2, prf_rdata_i[2*p+1], wb_i);
    end
  end

  //--------------------------------------------------------------------------
  // EX 级
  //--------------------------------------------------------------------------
  logic [N_ALU-1:0] ex_v;
  uop_t        ex_u [N_ALU];
  logic [63:0] ex_a0 [N_ALU], ex_b0 [N_ALU];
  logic [63:0] ex_pc [N_BRU], ex_tgt [N_BRU];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ex_v <= '0;
    else if (flush_i) ex_v <= '0;
    else for (int p = 0; p < N_ALU; p++)
      ex_v[p] <= rf_v[p] && !br_killed(rf_u[p].br_mask, br_upd_i) && !(|(rf_u[p].pois & ld_fail_q_i)) &&
                 !(|(rf_u[p].poia & ld_fail_i));
  end
  always_ff @(posedge clk_i) begin
    for (int p = 0; p < N_ALU; p++) begin
      uop_t n;
      n = rf_u[p]; n.br_mask = rf_u[p].br_mask & ~br_upd_i.resolve_mask;
      ex_u[p]  <= n;
      ex_a0[p] <= rf_a[p];
      ex_b0[p] <= rf_b[p];
    end
    for (int p = 0; p < N_BRU; p++) begin
      ex_pc[p]  <= ftq_pc_i[p] + {rf_u[p].ftq_off, 1'b0};
      ex_tgt[p] <= ftq_tgt_i[p];
    end
  end

  // EX 级旁路 (2026-10-25，6 GHz)：源号与各写回口 "下一拍目的号" 的比较在 RF 级做完寄存 (ex_sa / ex_sb)，
  // EX 级只剩 "匹配位 & 写回有效 & 写使能" 后的独热与或选择。原来 EX 级当拍 "源号与 10 个写回口比较 ->
  // 选数 -> ALU" (b180：ex_u.psrc1 -> 写回数据 201 ps)。与原 byp() 逐拍一致 (下面断言)
  logic [N_IWB-1:0] ex_sa [N_ALU], ex_sb [N_ALU];
  always_ff @(posedge clk_i)
    for (int p = 0; p < N_ALU; p++)
      for (int k = 0; k < N_IWB; k++) begin
        // 只有整数源才旁路 (源类型在 RF 级就并进匹配位：EX 级不再按 rs*_class 选 0 / 浮点值 / 旁路)
        ex_sa[p][k] <= rf_u[p].psrc1 == wb_pn_i[k] && rf_u[p].psrc1 != '0 && rf_u[p].rs1_class == RC_INT;
        ex_sb[p][k] <= rf_u[p].psrc2 == wb_pn_i[k] && rf_u[p].psrc2 != '0 && rf_u[p].rs2_class == RC_INT;
      end
  function automatic logic [63:0] byp_oh(input logic [N_IWB-1:0] sel, input logic [63:0] v,
                                         input wb_t [N_IWB-1:0] wb);
    logic [N_IWB-1:0] h;
    logic [63:0]      r;
    for (int k = 0; k < N_IWB; k++) h[k] = sel[k] && wb[k].valid && wb[k].wen;
    r = (|h) ? '0 : v;
    for (int k = 0; k < N_IWB; k++) r = r | ({64{h[k]}} & wb[k].data);
    return r;
  endfunction
  logic [N_ALU-1:0] ex_live;
  logic [63:0] ex_a [N_ALU], ex_b [N_ALU], ex_imm [N_ALU], ex_op2 [N_ALU], alu_y [N_ALU];
  always_comb begin
    for (int p = 0; p < N_ALU; p++) begin
      ex_live[p] = ex_v[p] && !br_killed(ex_u[p].br_mask, br_upd_i);
      // RF 级已处理：无源时 ex_a0 / ex_b0 = 0、浮点源时 ex_a0 = 浮点值，且匹配位只对整数源有效
      ex_a[p]    = byp_oh(ex_sa[p], ex_a0[p], wb_i);
      ex_b[p]    = byp_oh(ex_sb[p], ex_b0[p], wb_i);
      ex_imm[p]  = {{32{ex_u[p].imm[31]}}, ex_u[p].imm};
      ex_op2[p]  = ex_u[p].use_imm ? ex_imm[p] : ex_b[p];
    end
  end

`ifndef SYNTHESIS
  // 与原逐口比较的旁路一致；同拍不会有两个写回口写同一物理寄存器
  always_ff @(posedge clk_i)
    for (int p = 0; p < N_ALU; p++)
      if (rst_ni && ex_v[p]) begin
        logic [63:0] ra, rb;
        ra = (ex_u[p].rs1_class == RC_NONE) ? 64'd0 :
             (ex_u[p].rs1_class == RC_FP) ? ex_a0[p] : byp(ex_u[p].psrc1, ex_a0[p], wb_i);
        rb = (ex_u[p].rs2_class == RC_NONE) ? 64'd0 : byp(ex_u[p].psrc2, ex_b0[p], wb_i);
        assert (ex_a[p] == ra) else $error("int_exu: 口 %0d 源 1 旁路不一致", p);
        assert (ex_b[p] == rb) else $error("int_exu: 口 %0d 源 2 旁路不一致", p);
      end
`endif

  for (genvar p = 0; p < N_ALU; p++) begin : g_alu
    o6_alu u_alu (.op_i(ex_u[p].op), .is_w_i(ex_u[p].is_w), .auw_i(ex_u[p].auw), .ash_i(ex_u[p].ash),
                  .a_i(ex_a[p]), .b_i(ex_op2[p]), .y_o(alu_y[p]));
  end

  // 分支单元
  logic [63:0] bru_rd [N_BRU], bru_tgt [N_BRU];
  logic        bru_taken [N_BRU], bru_mis [N_BRU];
  for (genvar p = 0; p < N_BRU; p++) begin : g_bru
    o6_bru u_bru (
      .op_i(ex_u[p].op), .pc_i(ex_pc[p]), .is_rvc_i(ex_u[p].is_rvc),
      .a_i(ex_a[p]), .b_i(ex_b[p]), .imm_i(ex_imm[p]),
      .pred_taken_i(ex_u[p].pred_taken), .pred_tgt_i(ex_tgt[p]),
      .rd_o(bru_rd[p]), .taken_o(bru_taken[p]), .target_o(bru_tgt[p]), .mispred_o(bru_mis[p])
    );
  end

  //--------------------------------------------------------------------------
  // MUL (P2、P3 各一个) 与 DIV / CSR (P3)
  //--------------------------------------------------------------------------
  wire   mul_go = ex_live[2] && ex_u[2].fu == FU_MUL;
  logic  mul_v;
  uop_t  mul_u;
  logic [63:0] mul_y;
  logic  mul_s1v;
  uop_t  mul_s1u;
  o6_mul u_mul (
    .clk_i, .rst_ni, .flush_i, .br_upd_i,
    .valid_i(mul_go), .uop_i(ex_u[2]), .a_i(ex_a[2]), .b_i(ex_b[2]),
    .valid_o(mul_v), .uop_o(mul_u), .y_o(mul_y), .s1_v_o(mul_s1v), .s1_u_o(mul_s1u)
  );

  // MUL 四拍 (2026-10-25)：t 发射 -> EX1 t+2 -> EX4 t+5 -> wbreg t+6。
  // 唤醒在 MUL 位于第 1 级寄存器 (EX2，t+3) 这一拍发：消费者 t+4 被选、t+6 进 EX 时从 wbreg 旁路。
  // 这一拍 P2 的 1 拍运算被屏蔽 (block_p2_alu)：它若在 t+3 发射会在 t+6 写 wbreg，与 MUL 撞写回槽；
  // 屏蔽同时空出了它的发射唤醒槽给 MUL 的唤醒。两者都直接取 MUL 的第 1 级寄存器
  assign wake_mul_o = '{valid: mul_s1v && !br_killed(mul_s1u.br_mask, br_upd_i) && mul_s1u.rd_class == RC_INT,
                        is_fp: 1'b0, pdst: mul_s1u.pdst, sa: '0, sn: '0, sc: '0};
  wire block_p2_alu = mul_s1v;

  // P3 的 MUL：时序与 P2 的完全相同 (唤醒占 iwake[3]、写回占 W3)
  wire   mul1_go = ex_live[3] && ex_u[3].fu == FU_MUL;
  logic  mul1_v;
  uop_t  mul1_u;
  logic [63:0] mul1_y;
  logic  mul1_s1v;
  uop_t  mul1_s1u;
  o6_mul u_mul1 (
    .clk_i, .rst_ni, .flush_i, .br_upd_i,
    .valid_i(mul1_go), .uop_i(ex_u[3]), .a_i(ex_a[3]), .b_i(ex_b[3]),
    .valid_o(mul1_v), .uop_o(mul1_u), .y_o(mul1_y), .s1_v_o(mul1_s1v), .s1_u_o(mul1_s1u)
  );
  assign wake_mul1_o = '{valid: mul1_s1v && !br_killed(mul1_s1u.br_mask, br_upd_i) && mul1_s1u.rd_class == RC_INT,
                         is_fp: 1'b0, pdst: mul1_s1u.pdst, sa: '0, sn: '0, sc: '0};
  wire block_p3_alu = mul1_s1v;

  // DIV
  wire   div_busy;
  logic  div_v, div_done;
  uop_t  div_u;
  logic [63:0] div_y;
  logic  w6_take_div;
  wire   div_go = ex_live[3] && ex_u[3].fu == FU_DIV;
  o6_div u_div (
    .clk_i, .rst_ni, .flush_i, .br_upd_i,
    .valid_i(div_go), .uop_i(ex_u[3]), .a_i(ex_a[3]), .b_i(ex_b[3]),
    .busy_o(div_busy), .valid_o(div_v), .done_o(div_done), .uop_o(div_u), .y_o(div_y), .out_ready_i(w6_take_div)
  );

  // CSR：EX 级把请求交给 CSR 单元 (组合完成)，结果锁存等 W6
  //   提交级处理事件期间不发 CSR / 向量请求 (#28)：IQ 在 head_go 时选中头部的 wait_head 指令，要过 RF 才到 EX；
  //   这一两拍里中断到达，ROB 看头项「未启动」就接受中断，而这条指令随后在 EX 照样改 CSR (如 sret 改 SPP / 特权级)，
  //   中断返回后重执行就重复了副作用。事件期间还在流水里的这类指令都比事件年轻，必被冲刷，不发请求即可
  wire   head_ex_ok  = ex_live[3] && !ev_block_i;
  assign csr_req_o   = head_ex_ok && ex_u[3].fu == FU_CSR;
  assign fprf_raddr_o = rf_u[3].psrc1;
  assign csr_uop_o   = ex_u[3];
  assign csr_wdata_o = ex_u[3].op[2] ? ex_imm[3] : ex_a[3];
  assign csr_rs2_o   = ex_b[3];
  assign head_start_o = csr_req_o || vec_req_o;

  logic        csr_hold;
  uop_t        csr_hu;
  logic [63:0] csr_hd;
  logic        csr_hx;
  logic [5:0]  csr_hc;
  logic [63:0] csr_ht;
  logic        w6_take_csr;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) csr_hold <= 1'b0;
    else if (flush_i) csr_hold <= 1'b0;
    else if (csr_req_o && csr_done_i) csr_hold <= 1'b1;
    else if (w6_take_csr) csr_hold <= 1'b0;
  end
  always_ff @(posedge clk_i) begin
    if (csr_req_o && csr_done_i) begin
      csr_hu <= ex_u[3]; csr_hd <= csr_rdata_i;
      csr_hx <= csr_excp_i; csr_hc <= csr_cause_i; csr_ht <= csr_tval_i;
    end
  end

  // 向量：EX 级发请求 (单拍)，等完成；完成后结果锁存等 W6 (与 CSR 相同，只是完成可能晚很多拍)
  assign vec_req_o  = head_ex_ok && ex_u[3].fu == FU_VEC;
  assign vec_inst_o = ex_u[3].raw;
  assign vec_a_o    = ex_a[3];
  assign vec_b_o    = ex_b[3];
  logic        vec_wait, vec_hold;
  logic        vec_fp, vf_take;           // 结果写浮点 rd；本拍经 I2F 口送出
  uop_t        vec_u;
  logic [63:0] vec_hd, vec_ht;
  logic        vec_hx;
  logic [5:0]  vec_hc;
  logic        w6_take_vec;
  logic        vec_dir;                   // 应答那拍 W6 空：直接完成，不进 vec_hold
  // 应答可以与请求同拍 (VU 当拍判定)：那时 vec_u 还没锁存，用 EX 级的 uop
  wire         vec_now = vec_rsp_i && (vec_wait || vec_req_o);
  uop_t        vec_uc;
  assign vec_uc = vec_wait ? vec_u : ex_u[3];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin vec_wait <= 1'b0; vec_hold <= 1'b0; end
    else if (flush_i) begin vec_wait <= 1'b0; vec_hold <= 1'b0; end
    else begin
      if (vec_req_o && !vec_rsp_i) vec_wait <= 1'b1;
      else if (vec_rsp_i) vec_wait <= 1'b0;
      if (vec_now && !vec_dir) vec_hold <= 1'b1;
      else if (w6_take_vec || vf_take) vec_hold <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    if (vec_req_o) vec_u <= ex_u[3];
    if (vec_now) begin
      vec_hd <= vec_rdata_i; vec_hx <= vec_excp_i; vec_hc <= vec_cause_i; vec_ht <= vec_tval_i;
    end
  end

  // I2F (P3 的 EX 级)；向量指令写浮点 rd (vfmv.f.s) 的结果也从这里送去 FPU 的 FMV 队列 (I2F_MV，按 64 位原样写)
  // 不看本拍分支杀：FPU 收下时自己按同一 br_upd 判 (被杀的进 FMV 队列 dead = 1 / UI 槽 live = 0)，
  // 免得 kill_tag 经 vf_take 选择直通到 i2f_* 输出 (6 GHz：p155u57 in 组 −40.7)
  wire ex_i2f  = ex_v[3] && ex_u[3].fu == FU_I2F;
  assign vec_fp  = vec_u.rd_class == RC_FP;
  assign vf_take = vec_hold && vec_fp && !vec_hx && !ex_i2f && i2f_ready_i;
  always_comb begin
    i2f_valid_o = ex_i2f || vf_take;
    i2f_uop_o   = ex_u[3];
    i2f_data_o  = ex_a[3];
    if (vf_take) begin
      i2f_uop_o = vec_u;
      i2f_uop_o.fu = FU_I2F; i2f_uop_o.op = I2F_MV; i2f_uop_o.fp_dbl = 1'b1;
      i2f_data_o = vec_hd;
    end
  end

  //--------------------------------------------------------------------------
  // fu_ok (发射许可)
  //--------------------------------------------------------------------------
  always_comb begin
    fu_ok_o[0] = '0; fu_ok_o[0][FU_ALU] = 1'b1; fu_ok_o[0][FU_BRU] = 1'b1;
    fu_ok_o[1] = fu_ok_o[0];
    // P2 / P3：MUL 两拍后占写回槽，这一拍不发 1 拍运算 (BRU 也写 W2)
    fu_ok_o[2] = '0; fu_ok_o[2][FU_ALU] = !block_p2_alu; fu_ok_o[2][FU_BRU] = !block_p2_alu; fu_ok_o[2][FU_MUL] = 1'b1;
    fu_ok_o[3] = '0; fu_ok_o[3][FU_ALU] = !block_p3_alu; fu_ok_o[3][FU_MUL] = 1'b1;
    fu_ok_o[3][FU_I2F] = i2f_ready_i;
    // DIV：除法器空闲且流水里 (RF/EX) 没有另一条 DIV
    fu_ok_o[3][FU_DIV] = !div_busy && !(rf_v[3] && rf_u[3].fu == FU_DIV) && !(ex_v[3] && ex_u[3].fu == FU_DIV);
    // CSR：上一条 CSR 结果还没写回时不发
    fu_ok_o[3][FU_CSR] = !csr_hold && !(rf_v[3] && rf_u[3].fu == FU_CSR) && !(ex_v[3] && ex_u[3].fu == FU_CSR) &&
                         vu_idle_i;
    // 向量：上一条还没完成 / 结果没写回时不发
    fu_ok_o[3][FU_VEC] = !vec_wait && !vec_hold && !(rf_v[3] && rf_u[3].fu == FU_VEC) && !(ex_v[3] && ex_u[3].fu == FU_VEC);
    // P4 / P5 (IQ_C)：纯 ALU
    for (int p = 4; p < N_ALU; p++) begin fu_ok_o[p] = '0; fu_ok_o[p][FU_ALU] = 1'b1; end
  end
  // 下一拍：block_p2/3_alu = MUL 第 1 级有效 = 本拍进 MUL 的 (分支杀 / 冲刷按保守处理：多屏蔽一拍无害)
  always_comb begin
    for (int p = 0; p < N_ALU; p++) fu_ok_nx_o[p] = fu_ok_o[p];
    fu_ok_nx_o[2][FU_ALU] = !mul_go;  fu_ok_nx_o[2][FU_BRU] = !mul_go;
    fu_ok_nx_o[3][FU_ALU] = !mul1_go;
  end

  //--------------------------------------------------------------------------
  // 写回寄存器 W0..W3
  //--------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int p = 0; p < N_ALU; p++) wb_p_o[p].valid <= 1'b0;
    end else if (flush_i) begin
      for (int p = 0; p < N_ALU; p++) wb_p_o[p].valid <= 1'b0;
    end else begin
      for (int p = 0; p < N_ALU; p++) begin
        logic one;
        one = ex_live[p] && (ex_u[p].fu == FU_ALU || ex_u[p].fu == FU_BRU);
        if (p == 2)      wb_p_o[p].valid <= one || mul_v;
        else if (p == 3) wb_p_o[p].valid <= one || mul1_v;
        else             wb_p_o[p].valid <= one;
      end
    end
  end

  always_ff @(posedge clk_i) begin
    for (int p = 0; p < N_ALU; p++) begin
      wb_t w;
      w.valid = 1'b0;
      w.pdst  = ex_u[p].pdst;
      w.wen   = ex_u[p].rd_class == RC_INT;
      w.data  = (ex_u[p].fu == FU_BRU && p < N_BRU) ? bru_rd[p < N_BRU ? p : 0] : alu_y[p];
      if (p == 2 && mul_v) begin
        w.pdst = mul_u.pdst; w.wen = mul_u.rd_class == RC_INT; w.data = mul_y;
      end
      if (p == 3 && mul1_v) begin
        w.pdst = mul1_u.pdst; w.wen = mul1_u.rd_class == RC_INT; w.data = mul1_y;
      end
      wb_p_o[p].pdst <= w.pdst;
      wb_p_o[p].wen  <= w.wen;
      wb_p_o[p].data <= w.data;
    end
  end
  always_comb
    for (int p = 0; p < N_ALU; p++)
      wb_pn_o[p] = (p == 2 && mul_v) ? mul_u.pdst : (p == 3 && mul1_v) ? mul1_u.pdst : ex_u[p].pdst;

  // 完成 (与 wbreg 同拍，寄存器输出)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int p = 0; p < N_ALU; p++) cmpl_p_o[p].valid <= 1'b0;
    end else if (flush_i) begin
      for (int p = 0; p < N_ALU; p++) cmpl_p_o[p].valid <= 1'b0;
    end else begin
      for (int p = 0; p < N_ALU; p++) begin
        logic one;
        one = ex_live[p] && (ex_u[p].fu == FU_ALU || ex_u[p].fu == FU_BRU);
        cmpl_p_o[p].valid <= (p == 2) ? (one || mul_v) : (p == 3) ? (one || mul1_v) : one;
      end
    end
  end
  always_ff @(posedge clk_i) begin
    for (int p = 0; p < N_ALU; p++) begin
      cmpl_p_o[p].rob_idx     <= (p == 2 && mul_v) ? mul_u.rob_idx : (p == 3 && mul1_v) ? mul1_u.rob_idx : ex_u[p].rob_idx;
      cmpl_p_o[p].excp        <= 1'b0;
      cmpl_p_o[p].cause       <= '0;
      cmpl_p_o[p].tval        <= '0;
      cmpl_p_o[p].flush_after <= 1'b0;
      cmpl_p_o[p].replay      <= 1'b0;
      cmpl_p_o[p].fflags      <= '0;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni && !flush_i)
    assert (!(mul_v && ex_live[2] && (ex_u[2].fu == FU_ALU || ex_u[2].fu == FU_BRU))) else $error("int_exu: P2 写回冲突 (MUL 与 1 拍运算同拍)");
  always_ff @(posedge clk_i) if (rst_ni && !flush_i)
    assert (!(mul1_v && ex_live[3] && ex_u[3].fu == FU_ALU)) else $error("int_exu: P3 写回冲突 (MUL 与 ALU 同拍)");
`endif

  //--------------------------------------------------------------------------
  // W6 仲裁：F2I > DIV > CSR
  //--------------------------------------------------------------------------
  // 占用 / 选择只看不含本拍分支杀的 f2i_occ_i / div_done (除法结果在手就占住 W6，本拍被杀则 CSR / 向量晚一拍)；
  // 有效位 (w6_take_div、f2i_valid_i) 仍含杀判断。这样 kill_tag 不经选择直通 wb_pn_o (6 GHz：p155u58 in 组 −41.8)。
  // f2i_valid_i 蕴含 f2i_occ_i，w6_take_div 蕴含 w6_sel_div，有效时选择与原来相同
  wire   w6_sel_div  = div_done && !f2i_occ_i;
  assign w6_take_div = div_v && !f2i_occ_i;
  assign w6_take_csr = csr_hold && !f2i_occ_i && !div_done;
  assign vec_dir     = vec_now && (vec_uc.rd_class != RC_FP || vec_excp_i) && !f2i_occ_i && !div_done && !csr_hold;
  assign w6_take_vec = vec_dir || (vec_hold && (!vec_fp || vec_hx) && !f2i_occ_i && !div_done && !csr_hold);
  // W6 用的向量结果：直接完成取当拍应答，否则取保持的
  wire [63:0] vec_d = vec_dir ? vec_rdata_i : vec_hd;
  wire        vec_x = vec_dir ? vec_excp_i  : vec_hx;
  wire [5:0]  vec_c = vec_dir ? vec_cause_i : vec_hc;
  wire [63:0] vec_t = vec_dir ? vec_tval_i  : vec_ht;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wb_w6_o.valid <= 1'b0; cmpl_w6_o.valid <= 1'b0;
    end else if (flush_i) begin
      wb_w6_o.valid <= 1'b0; cmpl_w6_o.valid <= 1'b0;
    end else begin
      wb_w6_o.valid   <= f2i_valid_i || w6_take_div || w6_take_csr || w6_take_vec;
      cmpl_w6_o.valid <= f2i_valid_i || w6_take_div || w6_take_csr || w6_take_vec;
    end
  end
  assign wb_pn_o[N_ALU] = f2i_occ_i ? f2i_uop_i.pdst :
                          (w6_sel_div ? div_u.pdst : w6_take_csr ? csr_hu.pdst : vec_dir ? vec_uc.pdst : vec_u.pdst);
  always_ff @(posedge clk_i) begin
    uop_t u;
    logic [63:0] d;
    logic vx;
    u = f2i_occ_i ? f2i_uop_i : (w6_sel_div ? div_u : w6_take_csr ? csr_hu : vec_dir ? vec_uc : vec_u);
    d = f2i_occ_i ? f2i_data_i : (w6_sel_div ? div_y : w6_take_csr ? csr_hd : vec_d);
    vx = w6_take_vec && !f2i_valid_i && !w6_take_div && !w6_take_csr && vec_x;
    wb_w6_o.pdst <= u.pdst;
    wb_w6_o.wen  <= (u.rd_class == RC_INT) && !(w6_take_csr && csr_hx) && !vx;
    wb_w6_o.data <= d;
    cmpl_w6_o.rob_idx     <= u.rob_idx;
    cmpl_w6_o.excp        <= (w6_take_csr && !f2i_valid_i && !w6_take_div && csr_hx) || vx;
    cmpl_w6_o.cause       <= w6_take_csr ? csr_hc : vec_c;
    cmpl_w6_o.tval        <= w6_take_csr ? csr_ht : vec_t;
    cmpl_w6_o.flush_after <= 1'b0;
    cmpl_w6_o.replay      <= 1'b0;
    cmpl_w6_o.fflags      <= f2i_valid_i ? f2i_fflags_i : 5'd0;
  end

  // W6 唤醒：进入 wbreg6 的那一拍 (组合，取本拍选中的结果)
  always_comb begin
    uop_t u;
    // 本拍刚被杀的也照样唤醒 (F2I 早到版本本来就这样)：消费者比被杀分支年轻、同样被杀，pdst 恢复后才可能再分配。
    // 不查 br_killed，kill_tag 就不直通 wake_w6_o (6 GHz：p155u57 in 组 −40.7)
    u = w6_sel_div ? div_u : w6_take_csr ? csr_hu : vec_dir ? vec_uc : vec_u;
    if (f2i_occ_i) begin
      wake_w6_o = f2i_wk_i;
    end else begin
      wake_w6_o.valid = (w6_take_div || (w6_take_csr && !csr_hx) || (w6_take_vec && !vec_x)) && u.rd_class == RC_INT;
      wake_w6_o.is_fp = 1'b0;
      wake_w6_o.pdst  = u.pdst;
      wake_w6_o.sa    = '0;
      wake_w6_o.sn    = '0;
      wake_w6_o.sc    = '0;
    end
  end

  //--------------------------------------------------------------------------
  // 分支解析 -> br_update (寄存器输出)
  //--------------------------------------------------------------------------
  logic [N_BRU-1:0] br_v;
  always_comb begin
    for (int p = 0; p < N_BRU; p++)
      br_v[p] = ex_live[p] && ex_u[p].fu == FU_BRU && ex_u[p].is_br;
  end

  // 本拍误预测里最老的一个 (独热；ROB 序号不会相同)
  logic [N_BRU-1:0] mis, pick;
  always_comb begin
    for (int p = 0; p < N_BRU; p++) mis[p] = br_v[p] && bru_mis[p];
    for (int p = 0; p < N_BRU; p++) begin
      pick[p] = mis[p];
      for (int q = 0; q < N_BRU; q++)
        if (q != p && mis[q] && rob_older(ex_u[q].rob_idx, ex_u[p].rob_idx)) pick[p] = 1'b0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      br_upd_o <= '0;
      redirect_o <= 1'b0;
      bres_valid_o <= '0;
    end else if (flush_i) begin
      br_upd_o <= '0;
      redirect_o <= 1'b0;
      bres_valid_o <= '0;
    end else begin
      br_update_t b;
      b = '0;
      for (int p = 0; p < N_BRU; p++)
        if (br_v[p]) b.resolve_mask[ex_u[p].br_tag] = 1'b1;
      for (int p = 0; p < N_BRU; p++)
        if (pick[p]) begin
          b.kill     = 1'b1;
          b.kill_tag = ex_u[p].br_tag;
          b.kill_rob = ex_u[p].rob_idx;
          b.kill_lq  = ex_u[p].lq_idx;
          b.kill_sq  = ex_u[p].sq_idx;
        end
      br_upd_o      <= b;
      redirect_o    <= |mis;
      bres_valid_o  <= br_v;
    end
  end
  // 重定向目标 / 返回地址：各口寄存，寄存器之后按寄存的 pick 选 (2026-10-26，6 GHz：原来 mispred -> pick ->
  // 64 位三选一 (高扇出) -> 寄存，b180u44 redirect_pc_o 165 ps；输出端多一级选择，在模块间预算之内)
  logic [N_BRU-1:0] pick_q;
  logic [63:0]      lk_q [N_BRU];
  always_comb begin
    logic [N_BRU-1:0] pk;
    pk = (|pick_q) ? pick_q : N_BRU'(1);          // 无误预测时与原写法一样取 0 号口
    redirect_pc_o = '0; redirect_link_o = '0;
    for (int p = 0; p < N_BRU; p++) begin
      redirect_pc_o   = redirect_pc_o   | ({64{pk[p]}} & bres_target_o[p]);
      redirect_link_o = redirect_link_o | ({64{pk[p]}} & lk_q[p]);
    end
  end
  always_ff @(posedge clk_i) begin
    uop_t        ru;
    ru = ex_u[0];
    for (int p = 0; p < N_BRU; p++)
      if (pick[p]) ru = ex_u[p];
    pick_q <= pick;
    for (int p = 0; p < N_BRU; p++) lk_q[p] <= bru_rd[p];
    redirect_call_o <= (ru.op == BRU_JAL || ru.op == BRU_JALR) && (ru.lrd == 5'd1 || ru.lrd == 5'd5);
    redirect_ret_o  <= ru.op == BRU_JALR && ru.lrd == 5'd0 && (ru.lrs1 == 5'd1 || ru.lrs1 == 5'd5);
    redirect_ftq_o  <= ru.ftq_idx;
    for (int p = 0; p < N_BRU; p++) begin
      bres_ftq_o[p]     <= ex_u[p].ftq_idx;
      bres_off_o[p]     <= ex_u[p].ftq_off;
      bres_taken_o[p]   <= bru_taken[p];
      bres_mispred_o[p] <= bru_mis[p];
      bres_target_o[p]  <= bru_tgt[p];
      bres_ind_o[p]     <= ex_u[p].op == BRU_JALR &&
                           !(ex_u[p].lrd == 5'd0 && (ex_u[p].lrs1 == 5'd1 || ex_u[p].lrs1 == 5'd5));
    end
  end
endmodule
