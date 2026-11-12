//==============================================================================
// o6_sq_fwd.sv - 一条 load 流水 L1 的 store -> load 转发搜索 (组合)
//
// 每个字节取"比 load 老、地址已知、覆盖该字节"的最年轻 store：
//   与字节无关的部分 (有效、地址已知、比 load 老、同一双字) 先算成 m；
//   每字节的候选 c = m & 该 store 写这个字节 & load 读这个字节；在候选里取最年轻的较老 store。
//   选中的 store 数据可用 (data_ok、不是 AMO、不是 MMIO) -> 转发该字节；否则要等它 (wait_o，wsq_o 给出其中一条)。
// 2026-10-21 从 o6_lsu 原样拆出 (单元 TB tb/unit/tb_lsu_fwd.sv 直接测它)
//
// 2026-10-27 (6 GHz)：选择改成每字节一棵归约树。原来"后缀或 -> 独热 -> 与或取数据 / 与 ~fok 后 64 位或归约
// -> 各字节合并 -> 等待"串在 L1 一拍里 (b180：l1 pa -> l2_fw_wait 187.7 ps，其中本模块 168 ps)。
// 树的每个节点带 {等级 r, 数据不可用 bad, 下标, 数据字节}，两两比较 6 级即得胜出项，不需要独热。
// 环形年龄并进等级：下标 < 分界 (load 的 SQ 尾) 的候选 r = 2，>= 分界的 r = 1，非候选 r = 0；
// 同等级时下标大的较新。于是"分界以下取最大，否则取全体最大"是一次归约，不是两次选择再二选一。
//==============================================================================
`default_nettype none
module o6_sq_fwd
  import o6_pkg::*;
#(
  parameter int NSQ = SQ_SIZE
) (
  input  wire  [NSQ-1:0]    sqv_i,          // 项有效
  input  wire  [NSQ-1:0]    flag_i,         // 项的指针翻转位
  input  wire  [NSQ-1:0]    addr_ok_i,
  input  wire  [PA_W-1:0]   paddr_i [NSQ],
  input  wire  [7:0]        mask_i  [NSQ],  // store 写的字节 (双字内)
  input  wire  [63:0]       sdata_i [NSQ],  // 已按 paddr[2:0] 对齐到双字内的数据
  input  wire  [NSQ-1:0]    fok_i,          // 数据可转发：data_ok && !is_amo && !mmio
  input  wire  [PA_W-1:0]   ld_pa_i,
  input  wire  [7:0]        ld_mask_i,
  input  var sq_ptr_t       ld_sq_i,        // load 分派时的 SQ 尾 (比它老的 store 在这之前)
  output logic [7:0]        hit_o,          // 该字节有较老 store 覆盖
  output logic [63:0]       data_o,         // 转发的字节 (只在 hit 且选中的 store 可转发时有意义，其余为 0)
  output logic              wait_o,         // 有字节的最年轻较老 store 数据不可用：要等
  output logic [SQ_W-1:0]   wsq_o           // 要等的 store 之一 (wait_o 时有意义：编号最小的这种字节选中的那条)
);
  localparam int SW = $clog2(NSQ);

  typedef struct packed {
    logic [1:0]    r;        // 2：分界以下的候选；1：分界及以上的候选；0：非候选
    logic          bad;      // 数据不可转发
    logic [SW-1:0] idx;
    logic [7:0]    d;
  } nd_t;

  // 两个节点里较新的那个：hi 是下标较大的一半，等级不低于 lo 就取 hi
  function automatic nd_t pick2(input nd_t hi, input nd_t lo);
    return (hi.r[1] || (hi.r[0] && !lo.r[1])) ? hi : lo;
  endfunction

  logic [NSQ-1:0] m, below;
  always_comb
    for (int e = 0; e < NSQ; e++) begin
      sq_ptr_t ep;
      ep = '{flag: flag_i[e], idx: SQ_W'(e)};
      // 与字节无关的部分：有效、地址已知、比 load 老、同一双字
      m[e]     = sqv_i[e] && addr_ok_i[e] && sq_older(ep, ld_sq_i) && paddr_i[e][PA_W-1:3] == ld_pa_i[PA_W-1:3];
      below[e] = SQ_W'(e) < ld_sq_i.idx;
    end

  logic [7:0]    bad_b;
  logic [SW-1:0] idx_b [8];
  always_comb begin
    for (int b = 0; b < 8; b++) begin
      nd_t           nd [NSQ];
      logic [NSQ-1:0] c;
      for (int e = 0; e < NSQ; e++) begin
        c[e]      = m[e] && mask_i[e][b] && ld_mask_i[b];
        nd[e].r   = {c[e] && below[e], c[e] && !below[e]};
        nd[e].bad = !fok_i[e];
        nd[e].idx = SW'(e);
        nd[e].d   = sdata_i[e][8*b +: 8];
      end
      // 原地归约：第 i 个结果只覆盖已读过的 nd[i] (i <= 2i)
      for (int w = NSQ / 2; w >= 1; w = w / 2)
        for (int i = 0; i < w; i++) nd[i] = pick2(nd[2*i+1], nd[2*i]);
      hit_o[b]         = |c;
      bad_b[b]         = |c && nd[0].bad;
      idx_b[b]         = nd[0].idx;
      data_o[8*b +: 8] = (|c && !nd[0].bad) ? nd[0].d : 8'd0;
    end
    wait_o = |bad_b;
    wsq_o  = '0;
    for (int b = 7; b >= 0; b--) if (bad_b[b]) wsq_o = idx_b[b];
  end

  initial assert (NSQ == (1 << SW)) else $fatal(1, "o6_sq_fwd: NSQ 须为 2 的幂");
endmodule
