//==============================================================================
// o6_clint.sv - 核心本地中断器 (SiFive 兼容布局，单 hart)
//   0x0000  msip      (32 位，bit0 有效)
//   0x4000  mtimecmp  (64 位)
//   0xBFF8  mtime     (64 位，可写)
// mtime 每 TICK_DIV 个时钟加 1 (设备树 timebase-frequency = f_clk / TICK_DIV)。
// 寄存器访问：单拍响应 (req 当拍算，下一拍 ack)。
//==============================================================================
`default_nettype none
module o6_clint #(
  parameter int TICK_DIV = 10
) (
  input  wire          clk_i,
  input  wire          rst_ni,
  input  wire          req_i,
  input  wire          we_i,
  input  wire  [15:0]  addr_i,        // 偏移
  input  wire  [63:0]  wdata_i,       // 双字对齐
  input  wire  [7:0]   wmask_i,
  output logic         ack_o,
  output logic [63:0]  rdata_o,
  output logic         mtip_o,
  output logic         msip_o,
  output logic [63:0]  mtime_o
);
  logic [63:0] mtime, mtimecmp;
  logic        msip;
  logic [$clog2(TICK_DIV+1)-1:0] div;

  function automatic logic [63:0] merge(input logic [63:0] o, input logic [63:0] n, input logic [7:0] m);
    logic [63:0] r;
    for (int b = 0; b < 8; b++) r[8*b +: 8] = m[b] ? n[8*b +: 8] : o[8*b +: 8];
    return r;
  endfunction

  wire [15:0] dw = {addr_i[15:3], 3'b000};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mtime <= '0; mtimecmp <= '1; msip <= 1'b0; div <= '0; ack_o <= 1'b0; rdata_o <= '0;
    end else begin
      ack_o <= req_i && !ack_o;
      if (div == TICK_DIV - 1) begin div <= '0; mtime <= mtime + 1'b1; end
      else div <= div + 1'b1;
      if (req_i && !ack_o) begin
        unique case (dw)
          16'h0000: begin
            rdata_o <= {63'b0, msip};
            if (we_i && wmask_i[0]) msip <= wdata_i[0];
          end
          16'h4000: begin
            rdata_o <= mtimecmp;
            if (we_i) mtimecmp <= merge(mtimecmp, wdata_i, wmask_i);
          end
          16'hBFF8: begin
            rdata_o <= mtime;
            if (we_i) mtime <= merge(mtime, wdata_i, wmask_i);
          end
          default: rdata_o <= '0;
        endcase
      end
    end
  end

  assign mtip_o  = (mtime >= mtimecmp);
  assign msip_o  = msip;
  assign mtime_o = mtime;
endmodule
