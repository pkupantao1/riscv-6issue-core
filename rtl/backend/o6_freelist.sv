//==============================================================================
// o6_freelist.sv - 物理寄存器空闲表 (环形队列)
//
// 内容：所有不在 "提交 RAT ∪ 在途已分配" 里的物理寄存器。
// 出队 (重命名分配)：每拍最多 W 个，从 head 连续取。
// 入队 (提交释放旧映射)：每拍最多 W 个，压缩后写到 tail。
//
// 恢复：
//   分支误预测：head <- 该分支重命名时保存的 head 快照 (之后分配的全部退回)
//   异常冲刷  ：head <- arch_head (提交指针，提交时按"提交了几条有目的寄存器的指令"推进)
// 这两种恢复都只动 head，因为被作废的分配恰好是 [快照, head) 这一段。
//
// 初值：物理寄存器 NARCH..N-1 (0..NARCH-1 是复位时 RAT 的恒等映射)。
// 队列容量取 N。move 消除后 (o6_rename) 几个架构寄存器可共用一个物理寄存器，空闲项可多于 N-NARCH，
// 但 p0 永远被 x0 占着，最多 N-1 项，tail 追不上 arch_head。
//==============================================================================
`default_nettype none
module o6_freelist
  import o6_pkg::*;
#(
  parameter int N     = 192,
  parameter int NARCH = 32
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 出队：deq_cnt 条，前 W 个候选总是可见
  output preg_t [W-1:0]       deq_preg_o,
  output logic [$clog2(N+1)-1:0] free_cnt_o,
  output logic [W:0]          free_ge_o,     // free_ge_o[k] = (free_cnt_o >= k)，寄存的温度计码 (rename 判资源用)
  input  wire                 deq_fire_i,
  input  wire [2:0]           deq_cnt_i,

  // 分支快照：snap_head_o 给出 "出队 k 个之后" 的 head，k = 0..W
  output logic [$clog2(N):0]  head_o,        // {flag, idx}
  input  wire                 restore_i,     // 误预测恢复
  input  wire [$clog2(N):0]   restore_head_i,

  // 提交
  input  wire [W-1:0]         free_vld_i,    // 提交释放旧映射
  input  var preg_t [W-1:0]       free_preg_i,
  input  wire [2:0]           commit_alloc_cnt_i,  // 本拍提交的指令里有几条分配过本类寄存器
  input  wire                 flush_i        // 异常冲刷：head <- arch_head
);
  localparam int AW = $clog2(N);

  preg_t          q [N];
  logic [AW:0]    head, tail, arch_head;   // {flag, idx}

  function automatic logic [AW:0] ptr_add(input logic [AW:0] p, input logic [3:0] k);
    logic [AW:0] s;
    s = {1'b0, p[AW-1:0]} + k;
    if (s >= N) return {~p[AW], AW'(s - N)};
    else        return {p[AW], s[AW-1:0]};
  endfunction

  function automatic logic [AW:0] ptr_diff(input logic [AW:0] a, input logic [AW:0] b); // a - b
    if (a[AW] == b[AW]) return {1'b0, a[AW-1:0]} - {1'b0, b[AW-1:0]};
    else                return {1'b0, a[AW-1:0]} + (AW+1)'(N) - {1'b0, b[AW-1:0]};
  endfunction

  // 入队压缩：第 k 个有效释放写到 tail + (前面有效个数)
  logic [3:0] enq_cnt;
  always_comb begin
    enq_cnt = '0;
    for (int k = 0; k < W; k++) enq_cnt += free_vld_i[k];
  end

  // 出队候选寄存 (2026-10-26，6 GHz)：原来按 head 现读 192 选 1，再进 RAT 写 -> 快照 (b180u44 rename
  // u_ffl/head -> deq_preg 110 ps -> u_frat/snap 197 ps)。现在本拍按寄存的 head 读出 2W 个窗口 (空闲数不足 2W 时
  // 窗口尾部可能正被本拍释放写入，用写入数据旁路)，下一拍的候选 = 窗口[本拍出队数 + k]，晚到的出队数只做最后一级选择。
  // 冲刷 / 误预测恢复 (head 跳变) 的下一拍候选无效、空闲数报 0 (rename 那拍反正在冲刷后的空档里)，再下一拍按新 head 读
  preg_t [W-1:0] cand_q;
  logic          cand_ok_q;
  preg_t         win [2*W];
  always_comb begin
    preg_t       comp [W];          // 本拍释放压缩后的第 m 个
    int          m, fc;
    m = 0;
    for (int k = 0; k < W; k++) comp[k] = '0;
    for (int k = 0; k < W; k++) if (free_vld_i[k]) begin comp[m] = free_preg_i[k]; m++; end
    fc = int'(ptr_diff(tail, head));
    for (int j = 0; j < 2 * W; j++) begin
      logic [AW:0] p;
      p = ptr_add(head, 4'(j));
      win[j] = q[p[AW-1:0]];
      if (j >= fc && j - fc < int'(enq_cnt)) win[j] = comp[j - fc];
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cand_ok_q <= 1'b1;
      for (int k = 0; k < W; k++) cand_q[k] <= preg_t'(k + NARCH);
    end else if (flush_i || restore_i) begin
      cand_ok_q <= 1'b0;
    end else begin
      int a;
      a = deq_fire_i ? int'(deq_cnt_i) : 0;
      cand_ok_q <= 1'b1;
      for (int k = 0; k < W; k++) cand_q[k] <= win[a + k];
    end
  end
  assign deq_preg_o = cand_q;
  assign free_cnt_o = cand_ok_q ? ptr_diff(tail, head) : '0;
  assign head_o     = head;

  // 空闲数 >= k 的温度计码寄存 (2026-10-28，6 GHz：p160u59 rename u_ffl/tail -> 空闲数 (指针相减) -> 比较 -> fire
  // -> 快照写使能 173 ps)。下一拍的空闲数：冲刷 / 恢复后为 0 (同 cand_ok)，否则 = (tail + 本拍入队) - (head + 本拍出队)；
  // "出队 / 不出队"两种都用早到的量 (入队数、deq_cnt) 先比好，晚到的 deq_fire 只做最后的二选一
  logic [W:0] ge_q;
  assign free_ge_o = ge_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int k = 0; k <= W; k++) ge_q[k] <= (N - NARCH) >= k;
    end else if (flush_i || restore_i) begin
      ge_q <= {{W{1'b0}}, 1'b1};
    end else begin
      logic [AW:0] d0;
      d0 = ptr_diff(ptr_add(tail, enq_cnt), head);
      for (int k = 0; k <= W; k++)
        ge_q[k] <= deq_fire_i ? (int'(d0) >= k + int'(deq_cnt_i)) : (int'(d0) >= k);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      head      <= '0;
      arch_head <= '0;
      tail      <= {1'b0, AW'(N - NARCH)};
    end else begin
      tail      <= ptr_add(tail, enq_cnt);
      arch_head <= ptr_add(arch_head, {1'b0, commit_alloc_cnt_i});
      if (flush_i)
        head <= ptr_add(arch_head, {1'b0, commit_alloc_cnt_i});
      else if (restore_i)
        head <= restore_head_i;
      else if (deq_fire_i)
        head <= ptr_add(head, {1'b0, deq_cnt_i});
    end
  end

  // 队列存储：复位时写初值 (只在复位期间，综合成带复位值的触发器阵列)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int j = 0; j < N; j++)
        q[j] <= (j < N - NARCH) ? preg_t'(j + NARCH) : '0;
    end else begin
      logic [3:0] off;
      off = '0;
      for (int k = 0; k < W; k++) begin
        if (free_vld_i[k]) begin
          logic [AW:0] p;
          p = ptr_add(tail, off);
          q[p[AW-1:0]] <= free_preg_i[k];
          off = off + 1'b1;
        end
      end
    end
  end

`ifndef SYNTHESIS
  // 不变量：空闲数不超过 N-NARCH；arch_head 不越过 head
  always_ff @(posedge clk_i) if (rst_ni) begin
    // move 消除后几个架构寄存器可共用一个物理寄存器，空闲数可以超过 N - NARCH (至少 p0 被占用)
    assert (ptr_diff(tail, head) <= N - 1)
      else $error("freelist: 空闲数 %0d 超过 %0d (重复释放?)", ptr_diff(tail, head), N - 1);
    assert (ptr_diff(tail, arch_head) <= N - 1)
      else $error("freelist: tail-arch_head 越界");
    assert (!deq_fire_i || deq_cnt_i <= free_cnt_o)
      else $error("freelist: 出队数超过空闲数");
    for (int k = 0; k <= W; k++)
      assert (ge_q[k] == (int'(free_cnt_o) >= k))
        else $error("freelist: 寄存的空闲温度计码 ge[%0d] = %0d 与空闲数 %0d 不符", k, ge_q[k], free_cnt_o);
    // 寄存的候选与按 head 现读一致 (只核对空闲数以内的)
    if (cand_ok_q)
      for (int k = 0; k < W; k++)
        if (k < int'(ptr_diff(tail, head))) begin
          logic [AW:0] p;
          p = ptr_add(head, 4'(k));
          assert (cand_q[k] == q[p[AW-1:0]])
            else $error("freelist: 寄存的出队候选 %0d = %0d 与现读 %0d 不符", k, cand_q[k], q[p[AW-1:0]]);
        end
  end
`endif
endmodule
