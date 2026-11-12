//==============================================================================
// tb_freelist.sv - o6_freelist 单元 TB (docs/13 §3 L0、§16 P2「单元 TB」)
//
// 环境模型 = 一个最小的重命名 + ROB：投机 RAT / 提交 RAT、在途指令队列 (有无目的寄存器、是否分支、分支快照)。
//   每拍随机：提交最老 0..W 条 (释放旧映射，free_vld 稀疏)；分支误预测 (恢复到该分支重命名后的 head 快照，
//   丢掉更年轻的)；异常冲刷 (丢掉全部在途)；重命名 0..W 条 (空闲不够就整组停)。冲刷 / 恢复与提交可同拍。
// 检查 (与实现无关)：
//   C1 出队的物理寄存器不在「活跃集」里 (活跃 = 提交 RAT ∪ 在途新映射)：重复分配
//   C2 free_cnt_o == N - |活跃集| (守恒：不丢、不多)
//   C3 出队候选都 < N
// 两个实例：N = 256 (2 的幂，整数堆) 和 N = 192 (非 2 的幂，浮点堆：ptr_add 走「减 N 翻标志」分支)
// 覆盖率：cg_fl (每实例)，目标 100%
//   +cycles=N (默认 200000)  +ntb_random_seed=S
//==============================================================================
`default_nettype none
module fl_env
  import o6_pkg::*;
#(parameter int N = 256, parameter int NARCH = 32) (input wire clk, input wire rst_n, output int errs);
  localparam int AW = $clog2(N);

  preg_t [W-1:0]          deq_preg;
  logic [$clog2(N+1)-1:0] free_cnt;
  logic                   deq_fire = 0;
  logic [2:0]             deq_cnt = 0;
  logic [AW:0]            head;
  logic                   restore = 0;
  logic [AW:0]            restore_head = '0;
  logic [W-1:0]           free_vld = '0;
  preg_t [W-1:0]          free_preg = '0;
  logic [2:0]             cm_cnt = 0;
  logic                   flush = 0;

  o6_freelist #(.N(N), .NARCH(NARCH)) dut (
    .clk_i(clk), .rst_ni(rst_n),
    .deq_preg_o(deq_preg), .free_cnt_o(free_cnt), .deq_fire_i(deq_fire), .deq_cnt_i(deq_cnt),
    .head_o(head), .restore_i(restore), .restore_head_i(restore_head),
    .free_vld_i(free_vld), .free_preg_i(free_preg), .commit_alloc_cnt_i(cm_cnt), .flush_i(flush));

  function automatic logic [AW:0] ptr_add(input logic [AW:0] p, input int k);
    int s;
    s = p[AW-1:0] + k;
    return (s >= N) ? {~p[AW], AW'(s - N)} : {p[AW], AW'(s)};
  endfunction

  typedef struct { bit has_rd; int ard, newp, oldp; bit is_br; logic [AW:0] snap; int srat[32]; } ent_t;
  ent_t rob[$];
  int   srat[32], crat[32];
  bit   live[N];
  int   nlive;

  // 覆盖率采样量 (每拍由激励写)
  int   s_deq, s_enq, s_evt, s_disc, s_free_pre;
  bit   s_sparse, s_stall, s_hwrap, s_twrap;
  logic [AW:0] head_q = '0, tail_q = {1'b0, AW'(N - NARCH)};

  covergroup cg_fl @(posedge clk);
    option.per_instance = 1;
    cp_deq:   coverpoint s_deq  { bins n[] = {[0:W]}; }                        // 本拍出队个数 (fire 时)
    cp_enq:   coverpoint s_enq  { bins n[] = {[0:W]}; }                        // 本拍释放个数
    cp_sparse: coverpoint s_sparse { bins y = {1}; }                           // 释放有效位不连续 (压缩)
    cp_evt:   coverpoint s_evt  { bins none = {0}; bins rest = {1}; bins rest_cm = {2}; bins fl = {3}; bins fl_cm = {4}; bins fl_rest = {5}; }
    cp_disc:  coverpoint s_disc iff (s_evt == 1 || s_evt == 2) { bins zero = {0}; bins few = {[1:W]}; bins many = {[W+1:$]}; }   // 恢复丢弃的分配数
    cp_free:  coverpoint s_free_pre { bins empty = {0}; bins lt_w = {[1:W-1]}; bins mid = {[W:N-NARCH-1]}; bins full = {N-NARCH}; }
    cp_stall: coverpoint s_stall { bins y = {1}; }                             // 空闲不够，整组停
    cp_hwrap: coverpoint s_hwrap { bins y = {1}; }                             // head 回绕
    cp_twrap: coverpoint s_twrap { bins y = {1}; }                             // tail 回绕
    cx_deq_enq: cross cp_deq, cp_enq;                                          // 同拍出队 × 入队
  endgroup
  cg_fl cg = new();

  function automatic void err(string s);
    errs++;
    if (errs <= 20) $display("[%0t] fl_env N=%0d 错误：%s", $time, N, s);
  endfunction

  function automatic void set_live(int p, bit v);
    if (live[p] == v) err($sformatf("p%0d 活跃位已是 %0d", p, v));
    live[p] = v; nlive += v ? 1 : -1;
  endfunction

  int phase_cm, phase_rn;    // 当前阶段的提交 / 重命名倾向 (每几百拍换一次：填满 / 排空 / 平衡)
  int cyc;
  bit trace = $test$plusargs("trace");

  initial begin
    errs = 0;
    for (int a = 0; a < 32; a++) begin srat[a] = a; crat[a] = a; end
    for (int p = 0; p < N; p++) live[p] = (p < NARCH);
    nlive = NARCH;
    @(posedge rst_n);
    forever begin
      int ncm, br_idx, nfree, wcm;
      ent_t e;
      @(negedge clk);
      cyc++;
      // ---- 检查上一拍的结果 (C2 / C3) ----
      // 冲刷 / 恢复后的那一拍候选无效、free_cnt 报 0 (2026-10-26 起的设计)：那拍按实际指针差核对守恒
      if (dut.cand_ok_q) begin
        if (free_cnt != N - nlive) err($sformatf("free_cnt %0d != 期望 %0d", free_cnt, N - nlive));
      end else begin
        int d;
        d = (dut.tail[AW] == head[AW]) ? int'(dut.tail[AW-1:0]) - int'(head[AW-1:0])
                                       : int'(dut.tail[AW-1:0]) + N - int'(head[AW-1:0]);
        if (free_cnt != 0)     err($sformatf("候选无效拍 free_cnt %0d != 0", free_cnt));
        if (d != N - nlive)    err($sformatf("候选无效拍 指针差 %0d != 期望 %0d", d, N - nlive));
      end
      if (trace && N == 256 && errs < 2)
        $display("T c=%0d free=%0d exp=%0d head=%h ah=%h tail=%h | 上拍 fire=%0d n=%0d rest=%0d rh=%h fl=%0d vld=%b cm=%0d rob=%0d",
                 cyc, free_cnt, N - nlive, head, dut.arch_head, dut.tail, deq_fire, deq_cnt, restore, restore_head, flush, free_vld, cm_cnt, rob.size());
      for (int k = 0; k < W && k < free_cnt; k++)
        if (deq_preg[k] >= N) err($sformatf("候选 %0d = p%0d 越界", k, deq_preg[k]));
      // 回绕：上一拍 head / tail 的标志位翻了 (层次引用 dut.tail 只用来采覆盖率)
      s_hwrap = head[AW] != head_q[AW];
      s_twrap = dut.tail[AW] != tail_q[AW];
      head_q = head; tail_q = dut.tail;
      if (cyc % 400 == 1) begin
        randcase 1: begin phase_cm = 10; phase_rn = 90; end    // 填满：ROB 涨、空闲表见底
                 1: begin phase_cm = 90; phase_rn = 10; end    // 排空：空闲表回满
                 2: begin phase_cm = 60; phase_rn = 60; end
        endcase
      end
      s_free_pre = free_cnt;
      // ---- 本拍激励 ----
      deq_fire = 0; deq_cnt = 0; restore = 0; flush = 0; free_vld = '0; cm_cnt = 0;
      s_deq = 0; s_enq = 0; s_evt = 0; s_disc = 0; s_sparse = 0; s_stall = 0;
      // 提交：最老 ncm 条，每条占提交槽 k (有目的寄存器的那几槽释放旧映射 -> 有效位可以不连续)
      ncm = 0;
      if ($urandom_range(99) < phase_cm) ncm = $urandom_range(W < rob.size() ? W : rob.size());
      wcm = 0;
      for (int k = 0; k < ncm; k++) begin
        e = rob.pop_front();
        if (e.has_rd) begin
          free_vld[k] = 1; free_preg[k] = preg_t'(e.oldp);
          set_live(e.oldp, 0);
          crat[e.ard] = e.newp;
          wcm++;
        end
      end
      cm_cnt = 3'(wcm);
      s_enq = wcm;
      begin   // 有效位不连续：最低位到最高位之间有空槽
        int lo, hi;
        lo = -1; hi = -1;
        for (int k = 0; k < W; k++) if (free_vld[k]) begin if (lo < 0) lo = k; hi = k; end
        s_sparse = (lo >= 0) && (hi - lo + 1 > wcm);
      end
      // 冲刷 / 误预测 (误预测只挑这拍没提交的分支)
      br_idx = -1;
      if (rob.size() > 0 && $urandom_range(99) < 6) begin
        automatic int cands[$];      // initial 块里的声明默认是静态的：不加 automatic 会累积上一拍的内容
        foreach (rob[i]) if (rob[i].is_br) cands.push_back(i);
        if (cands.size() > 0) br_idx = cands[$urandom_range(cands.size() - 1)];
      end
      if ($urandom_range(999) < 8) begin
        flush = 1;
        s_evt = br_idx >= 0 ? 5 : (ncm > 0 ? 4 : 3);
        if (br_idx >= 0) begin restore = 1; restore_head = rob[br_idx].snap; end   // 同拍：冲刷优先
        while (rob.size() > 0) begin e = rob.pop_back(); if (e.has_rd) set_live(e.newp, 0); end
        srat = crat;
      end else if (br_idx >= 0) begin
        restore = 1; restore_head = rob[br_idx].snap;
        s_evt = ncm > 0 ? 2 : 1;
        srat = rob[br_idx].srat;
        while (rob.size() > br_idx + 1) begin e = rob.pop_back(); if (e.has_rd) begin set_live(e.newp, 0); s_disc++; end end
      end else if ($urandom_range(99) < phase_rn && rob.size() < 600) begin
        // 重命名一组 g 条：需要的分配数超过空闲数就整组停
        int g, need, k;
        automatic ent_t grp[$];
        g = $urandom_range(1, W);
        need = 0;
        for (int i = 0; i < g; i++) begin
          e.has_rd = $urandom_range(99) < 75;
          e.is_br  = $urandom_range(99) < 20;
          e.ard    = $urandom_range(1, 31);
          need += e.has_rd;
          grp.push_back(e);
        end
        if (need > free_cnt) s_stall = 1;
        else begin
          deq_fire = 1; deq_cnt = 3'(need); s_deq = need;
          k = 0;
          foreach (grp[i]) begin
            if (grp[i].has_rd) begin
              int p;
              p = deq_preg[k];
              if (p >= N) err($sformatf("出队 p%0d 越界", p));
              else if (live[p]) err($sformatf("C1 重复分配：p%0d 仍活跃", p));
              else set_live(p, 1);
              grp[i].newp = p; grp[i].oldp = srat[grp[i].ard];
              srat[grp[i].ard] = p;
              k++;
            end
            grp[i].snap = ptr_add(head, k);        // 分支快照 = 含本条分配之后的 head (o6_rename 的 fl_add_i(head, ipos + alloc))
            grp[i].srat = srat;
            rob.push_back(grp[i]);
          end
        end
      end
    end
  end
endmodule

module tb_freelist;
  logic clk = 0, rst_n = 0;
  int   e256, e192, ncyc;
  always #5 clk = ~clk;
  fl_env #(.N(256)) u256 (.clk, .rst_n, .errs(e256));
  fl_env #(.N(192)) u192 (.clk, .rst_n, .errs(e192));
  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 200000;
    repeat (3) @(posedge clk);
    rst_n = 1;
    repeat (ncyc) @(posedge clk);
    if (e256 + e192 == 0) $display("=== UNIT PASS === freelist %0d 拍", ncyc);
    else                  $display("=== UNIT FAIL === freelist 错误 %0d / %0d", e256, e192);
    $finish;
  end
endmodule
