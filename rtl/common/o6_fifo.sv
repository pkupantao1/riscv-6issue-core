//==============================================================================
// o6_fifo.sv - 同步 FIFO (触发器实现，深度任意)
//==============================================================================
`default_nettype none
module o6_fifo #(
  parameter int DW    = 32,
  parameter int DEPTH = 4
) (
  input  wire           clk_i,
  input  wire           rst_ni,
  input  wire           flush_i,
  input  wire           push_i,
  input  wire [DW-1:0]  wdata_i,
  output wire           full_o,
  input  wire           pop_i,
  output wire [DW-1:0]  rdata_o,
  output wire           empty_o,
  output wire [$clog2(DEPTH+1)-1:0] count_o
);
  localparam int AW = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  logic [DW-1:0] mem [DEPTH];
  logic [AW-1:0] rp, wp;
  logic [$clog2(DEPTH+1)-1:0] cnt;

  wire do_push = push_i && !full_o;
  wire do_pop  = pop_i && !empty_o;

  function automatic logic [AW-1:0] inc(input logic [AW-1:0] p);
    return (p == AW'(DEPTH-1)) ? '0 : p + 1'b1;
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rp <= '0; wp <= '0; cnt <= '0;
    end else if (flush_i) begin
      rp <= '0; wp <= '0; cnt <= '0;
    end else begin
      if (do_push) wp <= inc(wp);
      if (do_pop)  rp <= inc(rp);
      cnt <= cnt + do_push - do_pop;
    end
  end

  always_ff @(posedge clk_i) if (do_push) mem[wp] <= wdata_i;

  assign rdata_o = mem[rp];
  assign full_o  = (cnt == DEPTH);
  assign empty_o = (cnt == 0);
  assign count_o = cnt;
endmodule
