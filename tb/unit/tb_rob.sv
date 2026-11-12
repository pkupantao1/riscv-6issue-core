//==============================================================================
// tb_rob.sv - o6_rob 单元 TB (docs/13 §5.6；指针 / 有效位不变量由形式 F6 证明，这里测提交与事件语义)
//
// 参考模型 (按 o6_rob 头部注释的规格写，逐拍比对)：
//   程序序的行队列；每拍从头行 head_col 起提交连续的可提交项 (两半都完成、无异常、无重执行；flush_after 那条提交后停；
//   单步每拍 1 条)；头项异常 -> 陷入 (上拍刚分派了非法指令候选时晚一拍)；头项 replay -> 重执行；
//   中断：有挂起、头项有效、头项未启动 (含本拍 head_start)、不在处理上一事件 -> 在头项之前进入中断，本拍不提交
// 检查：R1 每拍提交位与模型一致；R2 提交项的 pdst / old_pdst / lrd / rd_valid / is_ld / is_st 与分派时一致；
//   R3 事件 (有无、trap / replay / irq、cause、tval_sel；tval_sel = REG 时 tval 等于该项记下的 tval —— 最老异常寄存器)
// 环境：分派 (1..6 列一行)、完成口随机完成 (带异常 / flush_after / replay)、STD 完成、wait_head 项到头才启动、
//   误预测 (完成与 kill 同拍，满足 F6 的时序契约)、访存违例、中断挂起随机、事件后下一拍 ev_block + flush (同 commit_ctrl)、偶尔单步
//   +cycles=N (默认 100000)；VDEFS="+define+O6_ROB_ROWS=4" 测小 ROB (满 / 回绕频繁)
//==============================================================================
`default_nettype none
module tb_rob;
  import o6_pkg::*;
  localparam int NC = 4, NS = 2, R = ROB_ROWS;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic [W-1:0] dis_valid = '0;  uop_t [W-1:0] dis_uop = '0;  logic [63:0] dis_tval [W];  logic dis_fire = 0;
  logic dis_ready;  rob_ptr_t dis_ptr;
  cmpl_t [NC-1:0] cmpl = '0;  logic [NS-1:0] std_done = '0;  rob_ptr_t [NS-1:0] std_rob = '0;
  logic head_start = 0, step_i = 0;  logic [N_STA-1:0] viol_v = '0;  rob_ptr_t [N_STA-1:0] viol_rob = '0;
  br_update_t bu = '0;  logic flush = 0, irq_pend = 0, ev_block = 0;  logic [5:0] irq_cause = 6'd11;
  logic [W-1:0] cm_valid, cm_rd_valid, cm_rd_fp, cm_mvel, cm_is_ld, cm_is_st, cm_is_br;
  logic [4:0] cm_lrd [W];  preg_t [W-1:0] cm_pdst, cm_old_pdst;  logic [FTQ_W-1:0] cm_ftq [W];
  logic [4:0] cm_fflags, ev_op;  logic cm_fp_dirty, ev_valid, ev_trap, ev_replay, ev_irq, ev_is_rvc, head_go, head2_go, empty;
  logic [5:0] ev_cause;  logic [63:0] ev_tval;  logic [1:0] ev_tval_sel;  logic [FTQ_W-1:0] ev_ftq;
  logic [FTQ_OFF_W-1:0] ev_off;  fu_e ev_fu;  rob_ptr_t head, head2;

  o6_rob #(.N_CMPL(NC), .N_STD(NS)) dut (
    .clk_i(clk), .rst_ni(rst_n),
    .dis_valid_i(dis_valid), .dis_uop_i(dis_uop), .dis_tval_i(dis_tval), .dis_fire_i(dis_fire),
    .dis_ready_o(dis_ready), .dis_ptr_o(dis_ptr),
    .cmpl_i(cmpl), .std_done_i(std_done), .std_rob_i(std_rob), .head_start_i(head_start), .step_i(step_i),
    .viol_valid_i(viol_v), .viol_rob_i(viol_rob), .br_upd_i(bu), .flush_i(flush),
    .cm_valid_o(cm_valid), .cm_rd_valid_o(cm_rd_valid), .cm_rd_fp_o(cm_rd_fp), .cm_lrd_o(cm_lrd),
    .cm_pdst_o(cm_pdst), .cm_old_pdst_o(cm_old_pdst), .cm_mvel_o(cm_mvel), .cm_is_ld_o(cm_is_ld),
    .cm_is_st_o(cm_is_st), .cm_is_br_o(cm_is_br), .cm_ftq_idx_o(cm_ftq), .cm_fflags_o(cm_fflags),
    .cm_fp_dirty_o(cm_fp_dirty),
    .ev_valid_o(ev_valid), .ev_trap_o(ev_trap), .ev_replay_o(ev_replay), .ev_irq_o(ev_irq), .ev_cause_o(ev_cause),
    .ev_tval_o(ev_tval), .ev_tval_sel_o(ev_tval_sel), .ev_ftq_idx_o(ev_ftq), .ev_ftq_off_o(ev_off),
    .ev_is_rvc_o(ev_is_rvc), .ev_fu_o(ev_fu), .ev_op_o(ev_op),
    .irq_pending_i(irq_pend), .irq_cause_i(irq_cause), .ev_block_i(ev_block),
    .head_o(head), .head_go_o(head_go), .head2_o(head2), .head2_go_o(head2_go), .empty_o(empty));

  //--------------------------------------------------------------------------
  // 模型
  //--------------------------------------------------------------------------
  typedef struct {
    bit da, db, excp, fa, rp, started, wh, is_st, is_ld, is_br, none;
    logic [5:0] cause;  logic [63:0] tval;  logic [1:0] tsel;
    uop_t u;  int seq;
  } ent_t;
  typedef struct { int row; bit flag; ent_t e[$]; int col0; } row_t;   // col0：头行已提交到的列
  row_t rows[$];
  int   seq_n, errs, ncyc, ncm, nev, nirq;
  bit   dx_prev;                 // 上拍分派组里有非法指令候选 (o6_rob 的 dx_v)
  bit   ev_q;                    // 上拍有事件 -> 本拍 ev_block + flush
  int   wh_start_c;              // 头部 wait_head 项启动的拍号 (之后几拍完成)
  int   cyc;

  function automatic void err(string s);
    errs++;
    if (errs <= 25) $display("[%0t] tb_rob 错误：%s", $time, s);
  endfunction
  function automatic rob_ptr_t ptr(int ri, int c);
    return '{flag: rows[ri].flag, row: ROB_ROW_W'(rows[ri].row), col: ROB_COL_W'(c)};
  endfunction

  // 覆盖率
  int c_ncm, c_ev;  bit c_kill, c_full, c_step, c_wh, c_irq_lock, c_dx, c_mmio, c_mmio_irq;
  covergroup cg_rob @(posedge clk);
    cp_ncm: coverpoint c_ncm { bins n[] = {[0:W]}; }
    cp_ev:  coverpoint c_ev  { bins none = {0}; bins trap_dec = {1}; bins trap_cm = {2}; bins replay = {3}; bins flush_after = {4}; bins irq = {5}; }
    cp_kill: coverpoint c_kill { bins y = {1}; }
    cp_full: coverpoint c_full { bins y = {1}; }
    cp_step: coverpoint c_step { bins y = {1}; }
    cp_wh:   coverpoint c_wh   { bins y = {1}; }        // wait_head 项在头部启动
    cp_irql: coverpoint c_irq_lock { bins y = {1}; }    // 中断挂起但头项已启动：不接受
    cp_dx:   coverpoint c_dx   { bins y = {1}; }        // 非法指令候选刚分派那拍头项异常不陷入
    cp_mmio: coverpoint c_mmio_irq { bins y = {1}; }    // 头部 MMIO load 已启动、中断挂起：不接受 (#35)
  endgroup
  cg_rob cg = new();

  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 100000;
    errs = 0; seq_n = 0; ncm = 0; nev = 0; nirq = 0; dx_prev = 0; ev_q = 0; wh_start_c = -1;
    for (int c = 0; c < W; c++) dis_tval[c] = '0;
    repeat (3) @(posedge clk);
    rst_n = 1;
    for (cyc = 0; cyc < ncyc; cyc++) begin @(negedge clk); step(); end
    if (errs == 0) $display("=== UNIT PASS === rob %0d 拍，提交 %0d 条、事件 %0d 次 (中断 %0d)", ncyc, ncm, nev, nirq);
    else           $display("=== UNIT FAIL === rob 错误 %0d", errs);
    $finish;
  end

  task automatic step();
    bit h_valid, locked, take_irq, trap, rep, sflush, go;
    int hc, nexp;
    logic [W-1:0] exp_cm;
    ent_t he;
    c_ncm = 0; c_ev = 0; c_kill = 0; c_step = 0; c_wh = 0; c_irq_lock = 0; c_dx = 0; c_mmio = 0; c_mmio_irq = 0;
    c_full = !dis_ready;
    // ---- 环境：事件处理 (上拍事件 -> 本拍 ev_block + flush) ----
    flush = ev_q; ev_block = ev_q; ev_q = 0;
    irq_pend = ($urandom_range(999) < ((cyc / 2000) % 3 == 0 ? 60 : 3));
    irq_cause = 6'($urandom_range(1) ? 11 : 9);
    step_i = $urandom_range(199) == 0;
    #1;   // head_go 组合于 irq_pending / ev_block
    // 头项 (模型)
    h_valid = rows.size() > 0 && rows[0].col0 < rows[0].e.size();
    if (h_valid) he = rows[0].e[rows[0].col0];
    // wait_head 项到头且 head_go：随机启动
    head_start = 0;
    if (!flush && h_valid && he.wh && !he.started && !he.da && head_go && $urandom_range(3) == 0) begin
      head_start = 1; c_wh = 1;
    end
    // MMIO load：ROB 项不是 wait_head，但在头部开始访问设备时同样给 head_start (#35)
    if (!flush && h_valid && he.is_ld && !he.started && !he.da && head_go && $urandom_range(9) == 0) begin
      head_start = 1; c_mmio = 1;
    end
    // ---- 期望的提交 / 事件 (本拍组合，基于模型状态 + 本拍输入) ----
    locked = h_valid && (he.started || head_start);
    take_irq = irq_pend && h_valid && !locked && !ev_block;
    if (irq_pend && h_valid && locked && !ev_block) begin c_irq_lock = 1; if (!he.wh) c_mmio_irq = 1; end
    exp_cm = '0; trap = 0; rep = 0; sflush = 0; nexp = 0;
    go = !ev_block && !take_irq && !flush;
    hc = h_valid ? rows[0].col0 : 0;
    if (rows.size() > 0)
      for (int c = hc; c < rows[0].e.size() && go; c++) begin
        ent_t e;
        e = rows[0].e[c];
        if (!(e.da && e.db)) go = 0;
        else if (e.excp) begin if (c == hc) begin if (!dx_prev) trap = 1; else c_dx = 1; end go = 0; end
        else if (e.rp) begin if (c == hc) rep = 1; go = 0; end
        else begin
          exp_cm[c] = 1; nexp++;
          if (step_i) begin go = 0; c_step = 1; end
          if (e.fa) begin sflush = 1; go = 0; end
        end
      end
    // ---- 本拍的完成 / STD / 违例 / 误预测 / 分派 (拍末生效) ----
    drive_inputs();
    #1;
    // ---- 比对 ----
    if (cm_valid !== exp_cm) err($sformatf("R1 提交位 %b，期望 %b (头行列 %0d，%0d 项)", cm_valid, exp_cm, hc, rows.size() ? rows[0].e.size() : 0));
    for (int c = 0; c < W; c++) if (cm_valid[c] && exp_cm[c]) begin
      ent_t e;
      e = rows[0].e[c];
      if (cm_pdst[c] != e.u.pdst || cm_old_pdst[c] != e.u.old_pdst || cm_lrd[c] != e.u.lrd ||
          cm_rd_valid[c] != (e.u.rd_class != RC_NONE) || cm_is_ld[c] != e.is_ld || cm_is_st[c] != e.is_st)
        err($sformatf("R2 列 %0d (seq %0d) 提交内容不符", c, e.seq));
    end
    begin
      bit ev_e;
      ev_e = take_irq || trap || rep || sflush;
      if (ev_valid !== ev_e || ev_trap !== (take_irq || trap) || ev_irq !== take_irq || ev_replay !== (!take_irq && !trap && rep))
        err($sformatf("R3 事件 v%0d t%0d i%0d r%0d，期望 v%0d t%0d i%0d r%0d", ev_valid, ev_trap, ev_irq, ev_replay,
                      ev_e, take_irq || trap, take_irq, !take_irq && !trap && rep));
      else if (take_irq) begin
        if (ev_cause != irq_cause || ev_tval_sel != 2'd3) err("R3 中断的 cause / tval_sel 不符");
      end else if (trap) begin
        if (ev_cause != he.cause || ev_tval_sel != he.tsel) err($sformatf("R3 陷入 cause %0d sel %0d，期望 %0d %0d", ev_cause, ev_tval_sel, he.cause, he.tsel));
        else if (he.tsel == 2'd0 && ev_tval !== he.tval) err($sformatf("R3 陷入 tval %h，期望 %h (最老异常寄存器)", ev_tval, he.tval));
      end
      if (ev_e) begin ev_q = 1; nev++; if (take_irq) nirq++; end
      c_ev = take_irq ? 5 : trap ? (he.none ? 1 : 2) : rep ? 3 : sflush ? 4 : 0;
    end
    c_ncm = nexp; ncm += nexp;
    // ---- 模型推进 (与 RTL 同序：冲刷 > 提交 + 误预测回滚 / 分派；然后表项字段) ----
    apply();
  endtask

  // 本拍输入：完成口、STD、违例、误预测、分派
  ent_t pend_cm [NC];  int pend_ri [NC], pend_ci [NC];
  bit   do_kill;  int kill_ri, kill_ci, kill_seq;
  task automatic drive_inputs();
    cmpl = '0; std_done = '0; viol_v = '0; bu = '0; dis_fire = 0; dis_valid = '0; do_kill = 0;
    for (int k = 0; k < NC; k++) pend_ri[k] = -1;
    if (flush) return;
    // 完成口：随机挑未完成的项 (wait_head 项要先启动；被消除 / 译码异常项分派即完成)
    for (int k = 0; k < NC; k++) if (rows.size() > 0 && $urandom_range(99) < 60) begin
      int ri, ci;
      ri = $urandom_range(rows.size() - 1);
      ci = (ri == 0) ? $urandom_range(rows[0].col0, rows[0].e.size() - 1) : $urandom_range(rows[ri].e.size() - 1);
      if (ci >= rows[ri].e.size()) continue;
      if (rows[ri].e[ci].da || (rows[ri].e[ci].wh && !(rows[ri].e[ci].started || (ri == 0 && ci == rows[0].col0 && head_start)))) continue;
      begin
        bit dup;
        dup = 0;
        for (int j = 0; j < k; j++) if (pend_ri[j] == ri && pend_ci[j] == ci) dup = 1;
        if (dup) continue;
      end
      pend_ri[k] = ri; pend_ci[k] = ci;
      cmpl[k].valid = 1; cmpl[k].rob_idx = ptr(ri, ci);
      cmpl[k].excp = $urandom_range(199) < 2; cmpl[k].cause = 6'($urandom_range(15)); cmpl[k].tval = {$urandom, $urandom};
      cmpl[k].flush_after = $urandom_range(199) < 3;
      cmpl[k].replay = $urandom_range(199) < 2;
      // 误预测：分支完成同拍广播 kill (被杀分支本拍还未完成，满足时序契约)
      if (!do_kill && rows[ri].e[ci].is_br && $urandom_range(99) < 30 && !cmpl[k].excp) begin
        do_kill = 1; kill_ri = ri; kill_ci = ci; kill_seq = rows[ri].e[ci].seq;
        bu.kill = 1; bu.kill_rob = ptr(ri, ci);
        cmpl[k].flush_after = 0; cmpl[k].replay = 0;
      end
    end
    // STD
    for (int s = 0; s < NS; s++) if (rows.size() > 0 && $urandom_range(99) < 40) begin
      int ri, ci;
      ri = $urandom_range(rows.size() - 1); ci = $urandom_range(rows[ri].e.size() - 1);
      if (rows[ri].e[ci].is_st && !rows[ri].e[ci].db && !(ri == 0 && ci < rows[0].col0)) begin
        std_done[s] = 1; std_rob[s] = ptr(ri, ci);
      end
    end
    // 访存违例：给某个 load 置 replay
    if (rows.size() > 0 && $urandom_range(99) < 2) begin
      int ri, ci;
      ri = $urandom_range(rows.size() - 1); ci = $urandom_range(rows[ri].e.size() - 1);
      if (rows[ri].e[ci].is_ld && !(ri == 0 && ci < rows[0].col0)) begin viol_v[0] = 1; viol_rob[0] = ptr(ri, ci); end
    end
    // 分派一行 (误预测那拍不分派：RTL 本来也不写)
    if (!do_kill && dis_ready && $urandom_range(99) < 55) begin
      int n;
      n = $urandom_range(1, W);
      dis_fire = 1;
      for (int c = 0; c < n; c++) begin
        uop_t u;
        u = '0;
        case ($urandom_range(9))
          0, 1, 2: u.fu = FU_ALU;
          3:       u.fu = FU_LD;
          4:       u.fu = FU_ST;
          5:       begin u.fu = FU_BRU; u.is_br = 1; end
          6:       begin u.fu = FU_CSR; u.wait_head = 1; u.flush_after = $urandom_range(1); end
          7:       if ($urandom_range(3) == 0) begin u.fu = FU_NONE; u.excp = 1; u.cause = $urandom_range(1) ? o6_isa_pkg::EXC_ILLEGAL_INST : o6_isa_pkg::EXC_BREAKPOINT; end else u.fu = FU_ALU;
          8:       begin u.fu = FU_NONE; u.mvel = 1; end
          default: u.fu = FU_MUL;
        endcase
        if ($urandom_range(59) == 0 && u.fu != FU_NONE) begin u.fu = FU_NONE; u.excp = 1; u.cause = o6_isa_pkg::EXC_INST_PAGE; u.tval_hi = $urandom_range(1); end
        u.rd_class = (u.fu == FU_ST || u.fu == FU_BRU && $urandom_range(1)) ? RC_NONE : ($urandom_range(5) == 0 ? RC_FP : RC_INT);
        u.lrd = $urandom_range(31); u.pdst = $urandom; u.old_pdst = $urandom;
        dis_valid[c] = 1; dis_uop[c] = u;
        dis_tval[c] = {$urandom, $urandom};
      end
    end
  endtask

  task automatic apply();
    bit ill;
    ill = 0;
    if (flush) begin rows.delete(); dx_prev = 0; return; end
    // 提交
    if (rows.size() > 0) begin
      for (int c = 0; c < W; c++) if (cm_valid[c]) rows[0].col0 = c + 1;
      if (rows[0].col0 >= rows[0].e.size()) void'(rows.pop_front());
    end
    // 误预测回滚：被杀分支之后的全部丢掉 (同行更后的列、之后的行)
    if (do_kill) begin
      int ri;
      c_kill = 1;
      ri = kill_ri;
      // 提交可能已弹掉头行：按 seq 重新定位
      begin
        int ks;
        ks = -1;
        foreach (rows[i]) foreach (rows[i].e[j]) if (rows[i].e[j].seq == kill_seq) begin ri = i; ks = j; end
        if (ks >= 0) begin
          while (rows.size() > ri + 1) void'(rows.pop_back());
          while (rows[ri].e.size() > ks + 1) void'(rows[ri].e.pop_back());
        end
      end
    end else if (dis_fire) begin
      row_t r;
      int rr;
      rr = dut.tail_row;
      r.row = rr; r.flag = dut.tail_flag; r.col0 = 0;
      for (int c = 0; c < W; c++) if (dis_valid[c]) begin
        ent_t e;
        uop_t u;
        u = dis_uop[c];
        e.u = u; e.seq = seq_n++;
        e.da = (u.fu == FU_NONE); e.db = (u.fu != FU_ST); e.excp = u.excp; e.cause = u.cause; e.fa = u.flush_after;
        e.rp = 0; e.started = 0; e.wh = u.wait_head; e.is_st = (u.fu == FU_ST) || (u.fu == FU_AMO); e.is_ld = (u.fu == FU_LD);
        e.is_br = u.is_br; e.none = (u.fu == FU_NONE);
        e.tsel = !u.excp ? 2'd3 : (u.cause == o6_isa_pkg::EXC_ILLEGAL_INST) ? 2'd0 :
                 (u.cause == o6_isa_pkg::EXC_BREAKPOINT) ? 2'd1 : (u.tval_hi ? 2'd2 : 2'd1);
        e.tval = dis_tval[c];
        if (u.excp && u.cause == o6_isa_pkg::EXC_ILLEGAL_INST) ill = 1;
        r.e.push_back(e);
      end
      rows.push_back(r);
    end
    dx_prev = dis_fire && !do_kill && ill;
    // 表项字段：完成 / STD / 启动 / 违例 (被杀 / 已提交的不再管)
    for (int k = 0; k < NC; k++) if (cmpl[k].valid) set_cm(cmpl[k]);
    for (int s = 0; s < NS; s++) if (std_done[s]) for_ent(std_rob[s], 1);
    if (head_start && rows.size() > 0) rows[0].e[rows[0].col0].started = 1;
    if (viol_v[0]) for_ent(viol_rob[0], 2);
  endtask
  // 定位指针对应的模型项
  function automatic void set_cm(cmpl_t c);
    foreach (rows[i]) if (rows[i].row == int'(c.rob_idx.row) && rows[i].flag == c.rob_idx.flag && int'(c.rob_idx.col) < rows[i].e.size()) begin
      rows[i].e[c.rob_idx.col].da = 1;
      if (c.excp) begin rows[i].e[c.rob_idx.col].excp = 1; rows[i].e[c.rob_idx.col].cause = c.cause; rows[i].e[c.rob_idx.col].tsel = 2'd0;
                        rows[i].e[c.rob_idx.col].tval = c.tval; end
      if (c.flush_after) rows[i].e[c.rob_idx.col].fa = 1;
      if (c.replay) rows[i].e[c.rob_idx.col].rp = 1;
    end
  endfunction
  function automatic void for_ent(rob_ptr_t p, int what);
    foreach (rows[i]) if (rows[i].row == int'(p.row) && rows[i].flag == p.flag && int'(p.col) < rows[i].e.size()) begin
      if (what == 1) rows[i].e[p.col].db = 1; else rows[i].e[p.col].rp = 1;
    end
  endfunction
endmodule
