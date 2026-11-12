//==============================================================================
// o6_prf.sv - 物理寄存器堆 (触发器阵列，异步读，多写口)
//
// 写口之间不会写同一项 (同一 pdst 只有一个生产者在途)。
// 读口不做写穿透：同拍正在写的值由外部旁路网络 (wbreg) 提供。
// ASIC 与 FPGA 用同一份 RTL：192x64 的触发器阵列在 40nm 约 0.1 mm²，
// 在 VU7P 上约 12k FF + 读口多路选择器，都可接受；LVT/多 bank RAM 方案留作优化。
//
// RLAT = 2 (2026-10-17，虚拟 2nm 6 GHz)：读出两拍。地址在第 1 拍给出，数据在第 2 拍组合输出。
//   第 1 拍：按地址低位在每组 (N/NG 项) 内选出候选，寄存 (该项本拍正被写时寄存新值 = 写前递)；
//   第 2 拍：按寄存的地址高位从 NG 个候选里选。
//   行为上等于「rdata = mem[上一拍的地址]」(mem 已含上一拍的写)，选择树在中间切开：
//   2nm 布局实测 128 项 12 读口的单拍读出 355 ps (连线 220 ps)，6 GHz 放不下。
//==============================================================================
`default_nettype none
module o6_prf
  import o6_pkg::*;
#(
  parameter int N   = 192,
  parameter int NR  = 10,
  parameter int NW  = 7,
  parameter bit ZERO_P0 = 1'b1,     // 整数堆：p0 恒为 0 (x0 的初始映射)
  parameter int RLAT = 1,           // 读出拍数：1 = 组合读；2 = 地址给出后下一拍出数
  parameter int NG   = 8            // RLAT = 2 时第 1 拍的组数 (N 须是 NG 的整数倍)
) (
  input  wire                clk_i,
  input  var preg_t [NR-1:0] raddr_i,
  output logic [63:0]        rdata_o [NR],
  input  wire  [NW-1:0]      wen_i,
  input  var preg_t [NW-1:0] waddr_i,
  input  wire  [63:0]        wdata_i [NW]
);
  logic [63:0] mem [N];

  if (RLAT == 1) begin : g_r1
    always_comb begin
      for (int r = 0; r < NR; r++)
        rdata_o[r] = (ZERO_P0 && raddr_i[r] == '0) ? 64'd0 : mem[raddr_i[r]];
    end
  end else begin : g_r2
    localparam int GS = N / NG;               // 每组项数
    localparam int LG = $clog2(GS);
    logic [63:0]       cand_q [NR][NG];
    logic [$clog2(NG)-1:0] hi_q [NR];
    logic [NR-1:0]     z_q;
    always_ff @(posedge clk_i) begin
      for (int r = 0; r < NR; r++) begin
        for (int g = 0; g < NG; g++) begin
          logic [63:0] v;
          int unsigned e;
          e = g * GS + int'(raddr_i[r][LG-1:0]);
          v = mem[e];
          for (int w = 0; w < NW; w++)
            if (wen_i[w] && int'(waddr_i[w]) == e) v = wdata_i[w];
          cand_q[r][g] <= v;
        end
        hi_q[r] <= raddr_i[r] >> LG;
        z_q[r]  <= ZERO_P0 && raddr_i[r] == '0;
      end
    end
    always_comb begin
      for (int r = 0; r < NR; r++)
        rdata_o[r] = z_q[r] ? 64'd0 : cand_q[r][hi_q[r]];
    end
  end

  // 仅仿真：第一个时钟沿把整个阵列清零。架构寄存器复位值在规范里是不定的 (硬件不复位)，
  // 但 Spike 取 0；不清零的话未初始化寄存器的 X 会在对拍里变成 0 而掩盖真实差异。
`ifndef SYNTHESIS
  logic sim_zeroed;
  initial sim_zeroed = 1'b0;
  always @(posedge clk_i) sim_zeroed <= 1'b1;
`endif

  always_ff @(posedge clk_i) begin
`ifndef SYNTHESIS
    if (!sim_zeroed) for (int p = 0; p < N; p++) mem[p] <= 64'd0;
`endif
    for (int w = 0; w < NW; w++)
      if (wen_i[w]) mem[waddr_i[w]] <= wdata_i[w];
  end

`ifndef SYNTHESIS
  // 仿真后门：提交对拍时读架构值
  function automatic logic [63:0] peek(input int p);
    return (ZERO_P0 && p == 0) ? 64'd0 : mem[p];
  endfunction
  always_ff @(posedge clk_i) begin
    for (int a = 0; a < NW; a++)
      for (int b = a+1; b < NW; b++)
        assert (!(wen_i[a] && wen_i[b] && waddr_i[a] == waddr_i[b]))
          else $error("prf: 写口 %0d 与 %0d 同拍写 p%0d", a, b, waddr_i[a]);
  end
`endif
endmodule
