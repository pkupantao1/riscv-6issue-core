//==============================================================================
// o6_alu.sv - 整数 ALU (纯组合)：RV64I + B 扩展 (Zba / Zbb / Zbs)
//   Zbb 的 clz / ctz / cpop 不在这里：2026-10-26 挪到 o6_mul (6 GHz 下计数是 ALU 最长的一类)
//   Zba 的 shNadd / *.uw / slli.uw 借用 ADD / SLL，操作数 A 先做预处理：auw 零扩展低 32 位，ash 左移 0..3 位
//==============================================================================
`default_nettype none
module o6_alu
  import o6_pkg::*;
(
  input  wire  [4:0]  op_i,
  input  wire         is_w_i,
  input  wire         auw_i,
  input  wire  [1:0]  ash_i,
  input  wire  [63:0] a_i,
  input  wire  [63:0] b_i,
  output logic [63:0] y_o
);
  wire [63:0] a    = (auw_i ? {32'b0, a_i[31:0]} : a_i) << ash_i;
  wire [5:0]  sh   = is_w_i ? {1'b0, b_i[4:0]} : b_i[5:0];
  wire [63:0] sum  = a + b_i;
  wire [63:0] diff = a_i - b_i;
  wire [31:0] aw   = a_i[31:0];
  wire [63:0] bit1 = 64'd1 << b_i[5:0];          // Zbs 的单位掩码
  wire        lt   = $signed(a_i) < $signed(b_i);
  wire        ltu  = a_i < b_i;


  // 循环移位
  wire [63:0] rol64 = (a_i << sh) | (a_i >> (7'd64 - {1'b0, sh}));
  wire [63:0] ror64 = (a_i >> sh) | (a_i << (7'd64 - {1'b0, sh}));
  wire [31:0] rol32 = (aw << sh[4:0]) | (aw >> (6'd32 - {1'b0, sh[4:0]}));
  wire [31:0] ror32 = (aw >> sh[4:0]) | (aw << (6'd32 - {1'b0, sh[4:0]}));

  logic [63:0] r;
  always_comb begin
    unique case (op_i)
      ALU_ADD:  r = sum;
      ALU_SUB:  r = diff;
      ALU_SLL:  r = a << sh;
      ALU_SLT:  r = {63'b0, lt};
      ALU_SLTU: r = {63'b0, ltu};
      ALU_XOR:  r = a_i ^ b_i;
      ALU_SRL:  r = is_w_i ? {32'b0, aw >> sh[4:0]} : a_i >> sh;
      ALU_SRA:  r = is_w_i ? {32'b0, 32'($signed(aw) >>> sh[4:0])} : 64'($signed(a_i) >>> sh);
      ALU_OR:   r = a_i | b_i;
      ALU_AND:  r = a_i & b_i;
      ALU_LUI:  r = b_i;
      ALU_ANDN: r = a_i & ~b_i;
      ALU_ORN:  r = a_i | ~b_i;
      ALU_XNOR: r = ~(a_i ^ b_i);
      ALU_MAX:  r = lt  ? b_i : a_i;
      ALU_MAXU: r = ltu ? b_i : a_i;
      ALU_MIN:  r = lt  ? a_i : b_i;
      ALU_MINU: r = ltu ? a_i : b_i;
      ALU_SEXTB: r = {{56{a_i[7]}}, a_i[7:0]};
      ALU_SEXTH: r = {{48{a_i[15]}}, a_i[15:0]};
      ALU_ZEXTH: r = {48'b0, a_i[15:0]};
      ALU_ROL:  r = is_w_i ? {32'b0, rol32} : rol64;
      ALU_ROR:  r = is_w_i ? {32'b0, ror32} : ror64;
      ALU_ORCB: for (int k = 0; k < 8; k++) r[8*k +: 8] = {8{|a_i[8*k +: 8]}};
      ALU_REV8: for (int k = 0; k < 8; k++) r[8*k +: 8] = a_i[8*(7-k) +: 8];
      ALU_BCLR: r = a_i & ~bit1;
      ALU_BEXT: r = {63'b0, |(a_i & bit1)};
      ALU_BINV: r = a_i ^ bit1;
      ALU_BSET: r = a_i | bit1;
      ALU_CZEQZ: r = (b_i == 64'd0) ? 64'd0 : a_i;     // Zicond
      ALU_CZNEZ: r = (b_i != 64'd0) ? 64'd0 : a_i;
      default:  r = 64'd0;
    endcase
    y_o = is_w_i ? {{32{r[31]}}, r[31:0]} : r;
  end
endmodule
