// fv_freelist.sv - 形式证明 F3：空闲表不重复分配 (docs/13 §5.4「free list 守恒」)
//   符号寄存器法：任取一个物理寄存器 P (自由常量)，环境只跟踪 P 的状态
//     FREE：在队列 [head, tail) 里，位置 pos；INFL：已分配未提交，位置 pos 在 [arch_head, head)；CMT：在提交 RAT 里
//   环境约束 = 合法的重命名 / 提交协议 (不建模 move 消除：每条提交恰好释放一个旧映射)：
//     出队数 <= 空闲数、<= W；提交的分配数 <= 在途数；释放个数 == 提交的分配数；只有 CMT 的 P 才能被释放、一拍最多一次；
//     恢复点在 [本拍提交后的 arch_head, head] 之间
//   性质：
//     ast_deq ：P 被出队时一定是 FREE，且出自它自己的位置 (队列里别处没有 P 的副本 => 不会重复分配)
//     ast_q   ：P 是 FREE 时，队列在 pos 处存的就是 P (没被覆盖)
//     ast_cnt ：tail - arch_head 恒为 N - NARCH (守恒)
//   N 取 16 (非 2 的幂的路径另由单元 TB 的 N = 192 覆盖)
`default_nettype none
module fv_freelist
  import o6_pkg::*;
#(parameter int N = 16, parameter int NARCH = 4) (
  input  wire                 clk,
  input  wire                 rst_n,
  input  wire                 deq_fire,
  input  wire [2:0]           deq_cnt,
  input  wire                 restore,
  input  wire [$clog2(N):0]   restore_head,
  input  wire [W-1:0]         free_vld,
  input  var preg_t [W-1:0]   free_preg,
  input  wire [2:0]           cm_cnt,
  input  wire                 flush,
  input  var preg_t           P
);
  localparam int AW = $clog2(N);
  preg_t [W-1:0]          deq_preg;
  logic [$clog2(N+1)-1:0] free_cnt;
  logic [AW:0]            head;

  o6_freelist #(.N(N), .NARCH(NARCH)) dut (
    .clk_i(clk), .rst_ni(rst_n), .deq_preg_o(deq_preg), .free_cnt_o(free_cnt), .deq_fire_i(deq_fire), .deq_cnt_i(deq_cnt),
    .head_o(head), .restore_i(restore), .restore_head_i(restore_head),
    .free_vld_i(free_vld), .free_preg_i(free_preg), .commit_alloc_cnt_i(cm_cnt), .flush_i(flush));

  function automatic logic [AW:0] padd(input logic [AW:0] p, input int k);
    int s;
    s = p[AW-1:0] + k;
    return (s >= N) ? {~p[AW], AW'(s - N)} : {p[AW], AW'(s)};
  endfunction
  function automatic int pdiff(input logic [AW:0] a, input logic [AW:0] b);   // a - b (环上)
    return (a[AW] == b[AW]) ? int'(a[AW-1:0]) - int'(b[AW-1:0]) : int'(a[AW-1:0]) + N - int'(b[AW-1:0]);
  endfunction

  wire [AW:0] tail = dut.tail, arch = dut.arch_head;
  int  enq_cnt, p_lanes, p_lane;
  always_comb begin
    enq_cnt = 0; p_lanes = 0; p_lane = 0;
    for (int k = 0; k < W; k++) if (free_vld[k]) begin
      if (free_preg[k] == P) begin p_lanes++; p_lane = enq_cnt; end   // P 在压缩后的第几个
      enq_cnt++;
    end
  end

  // ---- P 的状态 ----
  typedef enum logic [1:0] {S_FREE, S_INFL, S_CMT} st_e;
  // 复位后第一拍按 P 现取初值 (复位期间 P 是自由输入，可能与之后的值不同)
  st_e        st, st_q;
  logic [AW:0] pos, pos_q;
  logic        init;
  always_comb begin
    st  = init ? ((P >= NARCH) ? S_FREE : S_CMT) : st_q;
    pos = init ? {1'b0, AW'(P - NARCH)} : pos_q;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      init <= 1'b1; st_q <= S_CMT; pos_q <= '0;
    end else begin
      init <= 1'b0; st_q <= st; pos_q <= pos;
      case (st)
        S_FREE: if (deq_fire && !restore && !flush && pdiff(pos, head) < int'(deq_cnt)) st_q <= S_INFL;
        S_INFL: if (pdiff(pos, arch) < int'(cm_cnt))                st_q <= S_CMT;       // 本拍提交到它
                else if (flush)                               st_q <= S_FREE;      // 冲刷：退回 (head <- arch_n <= pos)
                else if (restore && pdiff(pos, arch) >= pdiff(restore_head, arch))
                                                              st_q <= S_FREE;      // 恢复点之后的分配退回 (距离都从 arch 量，环上才无歧义)
        S_CMT:  if (p_lanes != 0) begin st_q <= S_FREE; pos_q <= padd(tail, p_lane); end
        default: ;
      endcase
    end
  end

  // ---- 环境约束 ----
  asm_P:     assume property (@(posedge clk) disable iff (!rst_n) !init |-> $stable(P));
  asm_Pr:    assume property (@(posedge clk) P < N);
  asm_deq:   assume property (@(posedge clk) deq_fire |-> (deq_cnt <= free_cnt && deq_cnt <= W && !restore && !flush));
  asm_cm:    assume property (@(posedge clk) cm_cnt <= W && int'(cm_cnt) <= pdiff(head, arch));
  asm_enq:   assume property (@(posedge clk) enq_cnt == int'(cm_cnt));
  asm_pfree: assume property (@(posedge clk) p_lanes <= 1 && (p_lanes == 0 || st == S_CMT));
  // 恢复点合法：在 [本拍提交后的 arch, head] 之间 (从 arch 量距离)
  asm_rest:  assume property (@(posedge clk) restore |-> (pdiff(restore_head, arch) >= int'(cm_cnt) && pdiff(restore_head, arch) <= pdiff(head, arch)));

  // ---- 性质 ----
  for (genvar k = 0; k < W; k++) begin : g_deq
    ast_deq: assert property (@(posedge clk) disable iff (!rst_n)
      (deq_fire && k < deq_cnt && deq_preg[k] == P) |-> (st == S_FREE && pos == padd(head, k)));
  end
  ast_q:   assert property (@(posedge clk) disable iff (!rst_n) st == S_FREE |-> dut.q[pos[AW-1:0]] == P);
  ast_winf: assert property (@(posedge clk) disable iff (!rst_n)
    st == S_FREE |-> (pdiff(pos, head) >= 0 && pdiff(pos, head) < pdiff(tail, head)));  // pdiff 是有符号 int：与无符号量比较前都转 int'，否则 -1 变成大数
  ast_wini: assert property (@(posedge clk) disable iff (!rst_n)
    st == S_INFL |-> (pdiff(pos, arch) >= 0 && pdiff(pos, arch) < pdiff(head, arch)));
  // 唯一性引理 (让 ast_deq 可归纳)：窗口 [arch, tail) 里只有 P 自己那一格存着 P；P 在提交 RAT 里时窗口里没有 P
  for (genvar j = 0; j < N; j++) begin : g_uniq
    wire in_win = ((j - int'(arch[AW-1:0]) + N) % N) < pdiff(tail, arch);
    ast_uniq: assert property (@(posedge clk) disable iff (!rst_n)
      (in_win && dut.q[j] == P) |-> (st != S_CMT && pos[AW-1:0] == AW'(j)));
  end
  ast_cnt: assert property (@(posedge clk) disable iff (!rst_n) pdiff(tail, arch) == N - NARCH);
  // 可达性：P 走完一圈 (出队 -> 提交 -> 释放 -> 再出队)，恢复 / 冲刷退回也要可达
  cov_cycle:  cover property (@(posedge clk) disable iff (!rst_n) st == S_CMT ##[1:$] st == S_FREE ##[1:$] st == S_INFL);
  cov_rest:   cover property (@(posedge clk) disable iff (!rst_n) st == S_INFL && restore && !flush ##1 st == S_FREE);
  cov_flush:  cover property (@(posedge clk) disable iff (!rst_n) st == S_INFL && flush && cm_cnt != 0 ##1 st == S_FREE);
  cov_wrap:   cover property (@(posedge clk) disable iff (!rst_n) st == S_FREE && pos[AW] != 1'b0);
endmodule
