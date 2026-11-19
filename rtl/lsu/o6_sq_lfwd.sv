//==============================================================================
// o6_sq_lfwd.sv - 整行 (128 B) 的 store -> load 按字节转发搜索 (组合)，M3c (docs/11 §9g)
//
// 给共用 LSU 的向量 load 数据 uop 用：本遍读一行 ln，要的字节 ld_mask_i (窗口落在这一行里的部分)。
// 每个字节取 "比 load 老、地址已定、写这个字节" 的最年轻 store：
//   lm_i[e]：SQ 项 e 在行 ln 上写的字节 (调用方按行号算好：标量 store = 本行时按双字位置放 8 位掩码；
//            向量 store = 窗口里还没排空、落在本行的那段)；
//   数据：标量 store 的双字 sd_i[e] (已按 paddr[2:0] 对齐，字节 b 取 sd[8*(b%8)])，
//         向量 store 的整行 vd_i[e] (已按行内偏移放好，字节 b 取 vd[8*b])。
// 选中的 store 数据可用 (fok：data_ok、不是 AMO / MMIO) -> 转发该字节；否则要等 (wait_o，wsq_o 给出其中一条)。
// 选择与 o6_sq_fwd 相同：每字节一棵归约树，节点 {等级 r, 不可用 bad, 下标, 数据字节}，
//   下标 < 分界 (load 的 SQ 尾) 的候选 r = 2，>= 分界的 r = 1，同级取下标大的 (较新)
//==============================================================================
`default_nettype none
module o6_sq_lfwd
  import o6_pkg::*;
#(
  parameter int NSQ = SQ_SIZE
) (
  input  wire  [NSQ-1:0]    sqv_i,
  input  wire  [NSQ-1:0]    flag_i,
  input  wire  [NSQ-1:0]    addr_ok_i,
  input  wire  [CL_B-1:0]   lm_i   [NSQ],   // 项在本行写的字节
  input  wire  [NSQ-1:0]    vms_i,          // 1 = 向量 store (数据取 vd_i)
  input  wire  [CL_W-1:0]   vd_i   [NSQ],
  input  wire  [63:0]       sd_i   [NSQ],
  input  wire  [NSQ-1:0]    fok_i,
  input  wire  [CL_B-1:0]   ld_mask_i,      // load 本遍要的字节
  input  var sq_ptr_t       ld_sq_i,
  output logic [CL_B-1:0]   hit_o,
  output logic [CL_W-1:0]   data_o,
  output logic              wait_o,
  output logic [SQ_W-1:0]   wsq_o
);
  localparam int SW = $clog2(NSQ);

  typedef struct packed {
    logic [1:0]    r;
    logic          bad;
    logic [SW-1:0] idx;
    logic [7:0]    d;
  } nd_t;
  function automatic nd_t pick2(input nd_t hi, input nd_t lo);
    return (hi.r[1] || (hi.r[0] && !lo.r[1])) ? hi : lo;
  endfunction

  logic [NSQ-1:0] m, below;
  always_comb
    for (int e = 0; e < NSQ; e++) begin
      sq_ptr_t ep;
      ep = '{flag: flag_i[e], idx: SQ_W'(e)};
      m[e]     = sqv_i[e] && addr_ok_i[e] && sq_older(ep, ld_sq_i);
      below[e] = SQ_W'(e) < ld_sq_i.idx;
    end

  logic [CL_B-1:0] bad_b;
  logic [SW-1:0]   idx_b [CL_B];
  always_comb begin
    for (int b = 0; b < CL_B; b++) begin
      nd_t            nd [NSQ];
      logic [NSQ-1:0] c;
      for (int e = 0; e < NSQ; e++) begin
        c[e]      = m[e] && lm_i[e][b] && ld_mask_i[b];
        nd[e].r   = {c[e] && below[e], c[e] && !below[e]};
        nd[e].bad = !fok_i[e];
        nd[e].idx = SW'(e);
        nd[e].d   = vms_i[e] ? vd_i[e][8*b +: 8] : sd_i[e][8*(b % 8) +: 8];
      end
      for (int w = NSQ / 2; w >= 1; w = w / 2)
        for (int i = 0; i < w; i++) nd[i] = pick2(nd[2*i+1], nd[2*i]);
      hit_o[b]         = |c;
      bad_b[b]         = |c && nd[0].bad;
      idx_b[b]         = nd[0].idx;
      data_o[8*b +: 8] = (|c && !nd[0].bad) ? nd[0].d : 8'd0;
    end
    wait_o = |bad_b;
    wsq_o  = '0;
    for (int b = CL_B - 1; b >= 0; b--) if (bad_b[b]) wsq_o = idx_b[b];
  end

  initial assert (NSQ == (1 << SW)) else $fatal(1, "o6_sq_lfwd: NSQ 须为 2 的幂");
endmodule
