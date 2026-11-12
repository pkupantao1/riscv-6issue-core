//==============================================================================
// o6_prio_enc.sv - 优先级编码：找最低位的 1 (独热输出 + 编码输出)
//==============================================================================
`default_nettype none
module o6_prio_enc #(
  parameter int N = 8
) (
  input  wire [N-1:0]               req_i,
  output logic [N-1:0]              gnt_o,
  output logic [$clog2(N)-1:0]      idx_o,
  output wire                       any_o
);
  always_comb begin
    gnt_o = '0;
    idx_o = '0;
    for (int i = N-1; i >= 0; i--) begin
      if (req_i[i]) begin
        gnt_o = '0;
        gnt_o[i] = 1'b1;
        idx_o = i[$clog2(N)-1:0];
      end
    end
  end
  assign any_o = |req_i;
endmodule
