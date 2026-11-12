//==============================================================================
// o6_dpf.sv - L1 数据预取：按 load PC 的步长预取器 (RPT，Reference Prediction Table)
//
// 训练：load 第一次执行到 L1 (物理地址已知、可缓存、无异常) 时，按 PC[12:1] 查表 (NE 项直接映射 + 标签)：
//   命中：新步长 = 本次地址 - 上次地址；与记录的步长相同则置信 +1，否则置信 -1、降到 0 时换成新步长；
//         记下本次地址
//   不命中：占用该项 (步长 0、置信 0)
// 预取：命中、步长相同、置信 >= 2、步长非 0 时，预取 "本次地址 + 4 个步长" 所在的行；
//   步长小于一行时改为同方向的第 4 行 (DDR 级延迟下提前 2 行不够)；目标跨出 4 KB 页则不发 (物理地址不能跨页外推)；
//   与上一个预取同一行则不重复发。
// 输出寄存一拍送 D$ 的预取口 (与 store 预取共用，那边优先；D$ 只在 MSHR 宽裕时接受，丢了无妨)。
// 时序：训练输入寄存一拍再查表，查表 / 更新 / 算目标在同一拍，预取请求再寄存一拍。
//==============================================================================
`default_nettype none
module o6_dpf
  import o6_pkg::*;
#(
  parameter int NE = 32
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 tr_v_i,
  input  wire  [11:0]         tr_pch_i,
  input  wire  [PA_W-1:0]     tr_pa_i,
  output logic                pf_v_o,
  output logic [PA_W-1:0]     pf_pa_o
);
  localparam int IW = $clog2(NE);
  localparam int TW = 12 - IW;
  typedef struct packed {
    logic                v;
    logic [TW-1:0]       tag;
    logic [PA_W-1:0]     last;
    logic signed [15:0]  stride;
    logic [1:0]          conf;
  } ent_t;
  ent_t t [NE];

  // 训练输入寄存
  logic            a_v;
  logic [11:0]     a_pch;
  logic [PA_W-1:0] a_pa;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) a_v <= 1'b0;
    else         a_v <= tr_v_i;
  always_ff @(posedge clk_i) begin a_pch <= tr_pch_i; a_pa <= tr_pa_i; end

  wire [IW-1:0] ai  = a_pch[IW-1:0];
  wire [TW-1:0] atg = a_pch[11:IW];

  ent_t            nxt;
  logic            go;
  logic [PA_W-1:0] tgt;
  logic [PA_W-7:0] last_pf;        // 上一个预取的行号 (去重)
  always_comb begin
    ent_t               e;
    logic signed [16:0] ns;
    logic               hit, same;
    logic [PA_W-1:0]    d;
    e   = t[ai];
    hit = e.v && e.tag == atg;
    ns  = 17'($signed({1'b0, a_pa})) - 17'($signed({1'b0, e.last}));    // 低 17 位的差 (步长 ±64 KB 以内有效)
    same = hit && ns[15:0] == e.stride && e.stride != '0 && ns[16] == ns[15];
    nxt = e;
    if (!hit) begin
      nxt = '{v: 1'b1, tag: atg, last: a_pa, stride: '0, conf: 2'd0};
    end else begin
      nxt.last = a_pa;
      if (same)                   nxt.conf = (e.conf == 2'd3) ? 2'd3 : e.conf + 2'd1;
      else if (e.conf != 2'd0)    nxt.conf = e.conf - 2'd1;
      else begin                  nxt.stride = ns[15:0]; end
    end
    // 目标：4 个步长之后；与本次同一行时改为同方向第 2 行
    d   = PA_W'($signed(e.stride)) <<< 2;       // 先符号扩展到地址宽度再移，大步长不溢出
    tgt = a_pa + d;
    if (e.stride < 16'sd64 && e.stride > -16'sd64) tgt = e.stride[15] ? a_pa - PA_W'(256) : a_pa + PA_W'(256);
    go  = a_v && same && e.conf >= 2'd1 && tgt[PA_W-1:12] == a_pa[PA_W-1:12] && tgt[PA_W-1:6] != last_pf;
`ifdef O6_NO_DPF
    go  = 1'b0;                        // 性能对比实验：关掉预取
`endif
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pf_v_o <= 1'b0; last_pf <= '0;
      for (int i = 0; i < NE; i++) t[i].v <= 1'b0;
    end else begin
      pf_v_o <= go;
      if (go) last_pf <= tgt[PA_W-1:6];
      if (a_v) t[ai] <= nxt;
    end
  end
  always_ff @(posedge clk_i) if (go) pf_pa_o <= {tgt[PA_W-1:6], 6'b0};

`ifndef SYNTHESIS
  longint st_pf = 0;
  always_ff @(posedge clk_i) if (rst_ni && go) st_pf++;
  final $display("[perf] 数据预取 (步长) 发出 %0d 个", st_pf);
`endif
endmodule
