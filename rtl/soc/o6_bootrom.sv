//==============================================================================
// o6_bootrom.sv - 启动 ROM (64 KB 地址窗口，内容由 sw/bootrom 生成到 o6_bootrom_img.svh)
//
// 内容：复位入口 (a0 = hartid，a1 = 设备树地址，跳到 DRAM_BASE) + 设备树 (DTB)。
// 综合成组合查表 (常量数组)；ASIC 上会被映射成逻辑 (容量很小，不值得用 ROM 宏)。
//==============================================================================
`default_nettype none
module o6_bootrom (
  input  wire          clk_i,
  input  wire          rst_ni,
  input  wire          req_i,
  input  wire  [15:0]  addr_i,
  output logic         ack_o,
  output logic [63:0]  rdata_o
);
`include "o6_bootrom_img.svh"     // localparam int ROM_WORDS; localparam logic [63:0] ROM [ROM_WORDS]

  // ROM 镜像从 0x1000 开始
  wire [15:0] off = addr_i - 16'h1000;
  wire [15:3] wi  = off[15:3];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ack_o <= 1'b0; rdata_o <= '0;
    end else begin
      ack_o <= req_i && !ack_o;
      if (req_i && !ack_o) rdata_o <= (wi < ROM_WORDS) ? ROM[wi] : 64'd0;
    end
  end
endmodule
