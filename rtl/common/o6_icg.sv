//==============================================================================
// o6_icg.sv - 时钟门控单元 (锁存器型 ICG)
//   ASIC (O6_ASIC)：直接例化库单元 CKLNQD4BWP (E 使能、TE 测试使能、CP 时钟、Q 门控时钟)
//   其它 (仿真 / FPGA)：行为模型 —— 时钟低电平时锁存使能，与时钟相与
//==============================================================================
`default_nettype none
module o6_icg (
  input  wire clk_i,
  input  wire en_i,
  input  wire te_i,
  output wire clk_o
);
`ifdef O6_P2N
  CKLNQD4_P2NULVT u_cg (.TE(te_i), .E(en_i), .CP(clk_i), .Q(clk_o));   // 虚拟 2nm (pdk2n) ULVT 库
`elsif O6_ASIC
  CKLNQD4BWP u_cg (.TE(te_i), .E(en_i), .CP(clk_i), .Q(clk_o));
`else
  logic en_l;
  always_latch if (!clk_i) en_l = en_i | te_i;
  assign clk_o = clk_i & en_l;
`endif
endmodule
`default_nettype wire
