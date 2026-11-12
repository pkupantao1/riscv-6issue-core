//==============================================================================
// o6_rst_sync.sv - 复位同步器：异步置位、同步释放
//
// test_mode 下输出直接由 rst_ni 引脚控制 (扫描测试要求异步复位可控)。
//==============================================================================
`default_nettype none
module o6_rst_sync #(
  parameter int STAGES = 2
) (
  input  wire  clk_i,
  input  wire  rst_ni,
  input  wire  test_mode_i,
  output wire  rst_no
);
  logic [STAGES-1:0] q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) q <= '0;
    else         q <= {q[STAGES-2:0], 1'b1};
  end
  assign rst_no = test_mode_i ? rst_ni : q[STAGES-1];
endmodule
