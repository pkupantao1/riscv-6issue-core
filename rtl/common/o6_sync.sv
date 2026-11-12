//==============================================================================
// o6_sync.sv - 单比特电平同步器 (双触发器)
//
// 跨时钟域只允许用 rtl/common 里的同步单元，SpyGlass CDC 按单元名识别。
//==============================================================================
`default_nettype none
module o6_sync #(
  parameter int   STAGES    = 2,
  parameter logic RESET_VAL = 1'b0
) (
  input  wire  clk_i,
  input  wire  rst_ni,
  input  wire  d_i,
  output wire  q_o
);
  (* ASYNC_REG = "TRUE" *) logic [STAGES-1:0] q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) q <= {STAGES{RESET_VAL}};
    else         q <= {q[STAGES-2:0], d_i};
  end
  assign q_o = q[STAGES-1];
endmodule
