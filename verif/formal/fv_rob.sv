// fv_rob.sv - 形式证明 F6：ROB 头 / 尾 / 有效位不变量 (docs/13 §5.4「ROB 头尾计数」)
//   被测 o6_rob，行数在 tcl 里用 +define+O6_ROB_ROWS=4 缩小 (行数不是 2 的幂的回绕另由 R1 覆盖)；
//   环境约束 = 分派 / 误预测的合法用法：
//     分派：只在有空行时、有效位是非空前缀；误预测：kill_rob 指向窗口里一个有效项、翻转位与它所在行一致
//   性质：
//     ast_ptr   ：头 / 尾行号在范围内，翻转位与行号先后一致 (满 <=> 行号相同且翻转位不同)
//     ast_nz[r] ：窗口 [head, tail) 内的行非空
//     ast_pfx[r]：窗口内非头行的有效位是前缀；头行在 head_col 以下全空、从 head_col 起连续
//     ast_cm    ：提交位是从 head_col 起的连续段，且每条都有效、两半都完成、无异常 / 重执行
//   注：误预测回滚后窗口外的行留着旧有效位 (下次分派整行覆盖)，所以不断言「窗口外全空」
`default_nettype none
module fv_rob
  import o6_pkg::*;
#(parameter int N_CMPL = 4, parameter int N_STD = 2) (
  input  wire                       clk,
  input  wire                       rst_n,
  input  wire  [W-1:0]              dis_valid,
  input  var uop_t [W-1:0]          dis_uop,
  input  wire                       dis_fire,
  input  var cmpl_t [N_CMPL-1:0]    cmpl,
  input  wire  [N_STD-1:0]          std_done,
  input  var rob_ptr_t [N_STD-1:0]  std_rob,
  input  wire                       head_start,
  input  wire                       step,
  input  wire  [N_STA-1:0]          viol_valid,
  input  var rob_ptr_t [N_STA-1:0]  viol_rob,
  input  var br_update_t            bu,
  input  wire                       flush,
  input  wire                       irq_pending,
  input  wire  [5:0]                irq_cause,
  input  wire                       ev_block
);
  localparam int R = ROB_ROWS;
  logic [63:0] dis_tval [W];
  for (genvar c = 0; c < W; c++) begin : g_tv assign dis_tval[c] = '0; end
  logic dis_ready, empty, head2_go, cm_fp_dirty, ev_valid, ev_trap, ev_replay, ev_irq, ev_is_rvc, head_go;
  rob_ptr_t dis_ptr, head, head2;
  logic [W-1:0] cm_valid, cm_rd_valid, cm_rd_fp, cm_mvel, cm_is_ld, cm_is_st, cm_is_br;
  logic [4:0] cm_lrd [W];
  preg_t [W-1:0] cm_pdst, cm_old_pdst;
  logic [FTQ_W-1:0] cm_ftq_idx [W];
  logic [4:0] cm_fflags, ev_op;
  logic [5:0] ev_cause;
  logic [63:0] ev_tval;
  logic [1:0] ev_tval_sel;
  logic [FTQ_W-1:0] ev_ftq_idx;
  logic [FTQ_OFF_W-1:0] ev_ftq_off;
  fu_e ev_fu;

  o6_rob #(.N_CMPL(N_CMPL), .N_STD(N_STD)) dut (
    .clk_i(clk), .rst_ni(rst_n),
    .dis_valid_i(dis_valid), .dis_uop_i(dis_uop), .dis_tval_i(dis_tval), .dis_fire_i(dis_fire),
    .dis_ready_o(dis_ready), .dis_ptr_o(dis_ptr),
    .cmpl_i(cmpl), .cmpl_x2_i('{default: '0}), .std_done_i(std_done), .std_rob_i(std_rob), .head_start_i(head_start), .step_i(step),
    .viol_valid_i(viol_valid), .viol_rob_i(viol_rob), .br_upd_i(bu), .flush_i(flush),
    .cm_valid_o(cm_valid), .cm_rd_valid_o(cm_rd_valid), .cm_rd_fp_o(cm_rd_fp), .cm_lrd_o(cm_lrd),
    .cm_pdst_o(cm_pdst), .cm_old_pdst_o(cm_old_pdst), .cm_mvel_o(cm_mvel), .cm_is_ld_o(cm_is_ld),
    .cm_is_st_o(cm_is_st), .cm_is_br_o(cm_is_br), .cm_ftq_idx_o(cm_ftq_idx), .cm_fflags_o(cm_fflags),
    .cm_fp_dirty_o(cm_fp_dirty),
    .ev_valid_o(ev_valid), .ev_trap_o(ev_trap), .ev_replay_o(ev_replay), .ev_irq_o(ev_irq), .ev_cause_o(ev_cause),
    .ev_tval_o(ev_tval), .ev_tval_sel_o(ev_tval_sel), .ev_ftq_idx_o(ev_ftq_idx), .ev_ftq_off_o(ev_ftq_off),
    .ev_is_rvc_o(ev_is_rvc), .ev_fu_o(ev_fu), .ev_op_o(ev_op),
    .irq_pending_i(irq_pending), .irq_cause_i(irq_cause), .ev_block_i(ev_block),
    .head_o(head), .head_go_o(head_go), .head2_o(head2), .head2_go_o(head2_go), .empty_o(empty));

  wire [ROB_ROW_W-1:0] hr = dut.head_row, tr = dut.tail_row;
  wire hf = dut.head_flag, tf = dut.tail_flag;
  wire full = hr == tr && hf != tf;
  // 行 r 到头行的距离、窗口行数
  function automatic int rdist(input int r);
    return (r - int'(hr) + R) % R;
  endfunction
  wire [7:0] nrows = full ? 8'(R) : 8'((int'(tr) - int'(hr) + R) % R);
  // 窗口内第 r 行的翻转位
  function automatic logic rflag(input int r);
    return (r >= int'(hr)) ? hf : ~hf;
  endfunction

  // ---- 环境 ----
  asm_dis:  assume property (@(posedge clk) dis_fire |-> (dis_ready && dis_valid[0] && (dis_valid & (dis_valid + 1'b1)) == '0));
  asm_kill: assume property (@(posedge clk) bu.kill |-> (bu.kill_rob.row < R && bu.kill_rob.col < W &&
                                                         rdist(bu.kill_rob.row) < nrows &&
                                                         dut.vld[bu.kill_rob.row][bu.kill_rob.col] &&
                                                         bu.kill_rob.flag == rflag(bu.kill_rob.row)));
  // 时序契约：误预测广播那拍，被杀分支自己还没完成 (BRU 的完成与 br_upd 都在 EX 后寄存一拍，完成要再过一拍才写进 ROB)。
  //   去掉这条，Jasper 给出 3 拍反例：分支在被杀那拍提交、同行更后的项被杀，头行空而 head 不前进 -> 死锁。
  //   o6_rob 里有同名仿真断言，回归时逐次核对这条契约
  asm_kill_nd: assume property (@(posedge clk) bu.kill |-> !dut.ent[bu.kill_rob.row][bu.kill_rob.col].done_a);
  // 完成只写有效项 (o6_rob 自带的仿真断言检查的就是环境这条义务)
  for (genvar k = 0; k < N_CMPL; k++) begin : g_ac
    asm_cmpl: assume property (@(posedge clk) (cmpl[k].valid && !bu.kill && !flush) |->
                               (cmpl[k].rob_idx.row < R && cmpl[k].rob_idx.col < W && dut.vld[cmpl[k].rob_idx.row][cmpl[k].rob_idx.col] &&
                                  cmpl[k].rob_idx.flag == rflag(cmpl[k].rob_idx.row) &&   // 翻转位也对 (vld 不含 flag)
                                  rdist(cmpl[k].rob_idx.row) < nrows));                 // 且在窗口内：tail 之后的行 vld 是被杀时留下的旧值
    // 误预测那拍：不被杀的完成 (不比 kill 年轻) 也只写有效项 —— 原来整拍放开，窗口外 / 行内空位的乱指针让 rob_older 成环，
    //   最老异常的全对比较与锦标赛树各选各的 (o6_rob 的 _assert_3，2026-10-30 反例 2 / 7 拍)；比 kill 年轻的 ROB 自己滤掉
    asm_cmpl_w: assume property (@(posedge clk) (cmpl[k].valid && !flush && bu.kill && !rob_older(bu.kill_rob, cmpl[k].rob_idx)) |->
                                 (cmpl[k].rob_idx.row < R && cmpl[k].rob_idx.col < W && dut.vld[cmpl[k].rob_idx.row][cmpl[k].rob_idx.col] &&
                                  cmpl[k].rob_idx.flag == rflag(cmpl[k].rob_idx.row) &&   // 翻转位也对 (vld 不含 flag)
                                  rdist(cmpl[k].rob_idx.row) < nrows));                 // 且在窗口内：tail 之后的行 vld 是被杀时留下的旧值
    // ROB 号总在范围内 (行数缩成 4 后，超范围的行号写 ent 会被截位、落到别的真实项上)
    asm_cmpl_r: assume property (@(posedge clk) cmpl[k].valid |-> (cmpl[k].rob_idx.row < R && cmpl[k].rob_idx.col < W));
  end
  // 一条 uop 只从一个口完成：同拍两个完成口不写同一项 (2026-10-30 _assert_3 反例 6 拍：两口同拍写 (0,0)，一口带异常一口不带，
  //   不带的置 done 后头项越过最老异常提交，回绕后 ox_rob 落到窗口外)
  for (genvar k = 0; k < N_CMPL; k++) begin : g_ad
    for (genvar j = k + 1; j < N_CMPL; j++) begin : g_j
      asm_cmpl_1: assume property (@(posedge clk) (cmpl[k].valid && cmpl[j].valid) |-> cmpl[k].rob_idx != cmpl[j].rob_idx);
    end
  end
  // 一项只完成一次：完成口只写还没完成的项 (同反例：头三项已完成、同拍提交时第 2 项又收到带异常的完成，越过最老异常提交)。
  //   o6_rob 末尾有同名仿真断言，回归时逐次核对
  for (genvar k = 0; k < N_CMPL; k++) begin : g_a1
    asm_cmpl_once: assume property (@(posedge clk) (cmpl[k].valid && !flush && !(bu.kill && rob_older(bu.kill_rob, cmpl[k].rob_idx))) |->
                                    !dut.ent[cmpl[k].rob_idx.row][cmpl[k].rob_idx.col].done_a);
  end
  // 访存违例只报还没提交的 load (o6_rob 开头的仿真断言查的就是 LSU 这条义务；_assert_1 反例 1 拍：viol_rob 原来没有约束)
  for (genvar s = 0; s < N_STA; s++) begin : g_av
    asm_viol: assume property (@(posedge clk) viol_valid[s] |->
                               (viol_rob[s].row < R && viol_rob[s].col < W && rdist(viol_rob[s].row) < nrows &&
                                viol_rob[s].flag == rflag(viol_rob[s].row) && !rob_older(viol_rob[s], dut.head_o)));
  end

  // ---- 性质 ----
  ast_ptr: assert property (@(posedge clk) disable iff (!rst_n)
    hr < R && tr < R && dut.head_col < W && ((hf == tf) ? (tr >= hr) : (tr <= hr)));
  for (genvar r = 0; r < R; r++) begin : g_r
    wire inw = rdist(r) < nrows;
    wire [W-1:0] v = dut.vld[r];
    wire [W-1:0] vh = v >> dut.head_col;
    ast_nz:  assert property (@(posedge clk) disable iff (!rst_n) inw |-> v != '0);
    ast_pfx: assert property (@(posedge clk) disable iff (!rst_n)
      inw |-> ((r == int'(hr)) ? ((v & ((W'(1) << dut.head_col) - 1'b1)) == '0 && (vh & (vh + 1'b1)) == '0)
                               : ((v & (v + 1'b1)) == '0)));
  end
  wire [W-1:0] cmh = cm_valid >> dut.head_col;
  ast_cm: assert property (@(posedge clk) disable iff (!rst_n)
    ((cm_valid & ((W'(1) << dut.head_col) - 1'b1)) == '0) && ((cmh & (cmh + 1'b1)) == '0));
  for (genvar c = 0; c < W; c++) begin : g_c
    ast_cmv: assert property (@(posedge clk) disable iff (!rst_n)
      cm_valid[c] |-> (dut.vld[hr][c] && !empty && dut.ent[hr][c].done_a && dut.ent[hr][c].done_b &&
                       !dut.ent[hr][c].excp && !dut.ent[hr][c].replay));
  end
  // 换行时头行没有剩下的有效项 (行内有效项全部提交才换行；否则跳过的项永远不提交 —— 变异 FR1)
  ast_adv: assert property (@(posedge clk) disable iff (!rst_n)
    (dut.row_done && !flush) |-> ((dut.vld[hr] & ~cm_valid) == '0));
  // 可达性：满、回绕后满、误预测回滚、整行提交换行
  cov_full:  cover property (@(posedge clk) disable iff (!rst_n) full);
  cov_wrap:  cover property (@(posedge clk) disable iff (!rst_n) full && hf);
  cov_kill:  cover property (@(posedge clk) disable iff (!rst_n) bu.kill && !flush && nrows > 1);
  cov_row:   cover property (@(posedge clk) disable iff (!rst_n) dut.row_done && cm_valid[W-1]);
endmodule
