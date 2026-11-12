// fv_ptr_order.sv - 形式证明 F1：环形队列指针的先后比较 (o6_pkg::sq_older / lq_older)
//   设队列最老项指针为 base；窗口内的项指针 a = base + ka (ka < N)，边界指针 b = base + kb (kb <= N，b 可以是 tail，
//   与 base 正好差一整圈 = 队列满)。要求：older(a, b) == (ka < kb)。
//   #26 的错误 (标志不同、下标相同时判「不更老」) 正是 ka = 0、kb = N 这一点；变异 M02 (改回 >) 会被证伪。
`default_nettype none
module fv_ptr_order
  import o6_pkg::*;
(
  input  wire                clk,
  input  sq_ptr_t            sq_base,
  input  wire  [SQ_W:0]      sq_ka, sq_kb,
  input  lq_ptr_t            lq_base,
  input  wire  [LQ_W:0]      lq_ka, lq_kb
);
  // 指针加 k：{flag, idx} 当作 (W+1) 位整数相加 (队列大小是 2 的幂，下标回绕时标志翻转)
  function automatic sq_ptr_t sq_add(sq_ptr_t p, logic [SQ_W:0] k);
    logic [SQ_W:0] s;
    s = {p.flag, p.idx} + k;
    return '{flag: s[SQ_W], idx: s[SQ_W-1:0]};
  endfunction
  function automatic lq_ptr_t lq_add(lq_ptr_t p, logic [LQ_W:0] k);
    logic [LQ_W:0] s;
    s = {p.flag, p.idx} + k;
    return '{flag: s[LQ_W], idx: s[LQ_W-1:0]};
  endfunction

  sq_ptr_t sa, sb;
  lq_ptr_t la, lb;
  assign sa = sq_add(sq_base, sq_ka);
  assign sb = sq_add(sq_base, sq_kb);
  assign la = lq_add(lq_base, lq_ka);
  assign lb = lq_add(lq_base, lq_kb);

  // 窗口约束：项在队列里 (ka < N)，边界指针最多差一整圈 (kb <= N)
  asm_sq: assume property (@(posedge clk) sq_ka < SQ_SIZE && sq_kb <= SQ_SIZE);
  asm_lq: assume property (@(posedge clk) lq_ka < LQ_SIZE && lq_kb <= LQ_SIZE);

  ast_sq_older: assert property (@(posedge clk) sq_older(sa, sb) == (sq_ka < sq_kb));
  ast_lq_older: assert property (@(posedge clk) lq_older(la, lb) == (lq_ka < lq_kb));
  // 「正好满」那一点必须可达 (防止约束写错把它排除掉，证明变成空转)
  cov_sq_full: cover property (@(posedge clk) sq_ka == 0 && sq_kb == SQ_SIZE);
  cov_lq_full: cover property (@(posedge clk) lq_ka == 0 && lq_kb == LQ_SIZE);
endmodule
