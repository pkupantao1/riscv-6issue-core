//==============================================================================
// o6_cg.sv - 时钟门控单元封装
//
// ASIC (O6_ASIC)：例化 TSMC 40LP 的集成门控单元 CKLNQD4BWP (TE 为测试使能)
// FPGA (O6_FPGA)：不门控，直接输出时钟 (UltraScale+ 上细粒度门控应改用 CE)
// 仿真：锁存器 + 与门的行为模型
//
// 只用于粗粒度门控 (FPU、L2 空闲)；细粒度门控交给 DC 自动插入。
//==============================================================================
`default_nettype none
module o6_cg (
  input  wire  clk_i,
  input  wire  en_i,
  input  wire  test_en_i,
  output wire  clk_o
);
`ifdef O6_P2N
  CKLNQD4_P2NULVT u_icg (.CP(clk_i), .E(en_i), .TE(test_en_i), .Q(clk_o));   // 虚拟 2nm (pdk2n) ULVT 库
`elsif O6_ASIC
  CKLNQD4BWP u_icg (.CP(clk_i), .E(en_i), .TE(test_en_i), .Q(clk_o));
`elsif O6_FPGA
  assign clk_o = clk_i;
  wire unused = en_i ^ test_en_i;
`else
  logic en_latch;
  always_latch begin
    if (!clk_i) en_latch = en_i | test_en_i;
  end
  assign clk_o = clk_i & en_latch;
`endif
endmodule
