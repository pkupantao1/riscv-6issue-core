//==============================================================================
// o6_bru.sv - 分支单元 (纯组合)：条件判定、目标计算、与预测比较
//
// 预测语义：pred_taken = 前端认为本条是取指块里 taken 的控制流，
//   目标 = FTQ 中该块记录的预测目标 pred_tgt。
// 误预测 = 方向错，或 taken 且目标不同。JAL 由前端保证正确，这里只算链接地址。
//==============================================================================
`default_nettype none
module o6_bru
  import o6_pkg::*;
(
  input  wire  [4:0]  op_i,
  input  wire  [63:0] pc_i,
  input  wire         is_rvc_i,
  input  wire  [63:0] a_i,          // rs1
  input  wire  [63:0] b_i,          // rs2
  input  wire  [63:0] imm_i,
  input  wire         pred_taken_i,
  input  wire  [63:0] pred_tgt_i,
  output logic [63:0] rd_o,         // 链接地址 / AUIPC 结果
  output logic        taken_o,
  output logic [63:0] target_o,     // 实际下一 PC
  output logic        mispred_o
);
  function automatic logic add_eq(input logic [63:0] x, input logic [63:0] y, input logic [63:0] z);
    logic [63:0] k;
    k = (x & y) | ((x ^ y) & ~z);
    return (x ^ y ^ z) == {k[62:0], 1'b0};
  endfunction
  // JALR 目标第 0 位清零：期望值第 0 位取实际和的第 0 位 (该位总相等，进位照常)
  wire eq_dir  = add_eq(pc_i, imm_i, pred_tgt_i);
  wire eq_jalr = add_eq(a_i, imm_i, {pred_tgt_i[63:1], a_i[0] ^ imm_i[0]});
  wire [63:0] pc_next = pc_i + (is_rvc_i ? 64'd2 : 64'd4);
  logic cond;
  always_comb begin
    unique case (op_i)
      BRU_BEQ:  cond = (a_i == b_i);
      BRU_BNE:  cond = (a_i != b_i);
      BRU_BLT:  cond = $signed(a_i) <  $signed(b_i);
      BRU_BGE:  cond = $signed(a_i) >= $signed(b_i);
      BRU_BLTU: cond = a_i <  b_i;
      BRU_BGEU: cond = a_i >= b_i;
      default:  cond = 1'b0;
    endcase
  end

  always_comb begin
    rd_o     = pc_next;
    taken_o  = 1'b0;
    target_o = pc_next;
    unique case (op_i)
      BRU_JAL:   begin taken_o = 1'b1; target_o = pc_i + imm_i; end
      BRU_JALR:  begin taken_o = 1'b1; target_o = (a_i + imm_i) & ~64'd1; end
      BRU_AUIPC: begin rd_o = pc_i + imm_i; end
      default: begin
        taken_o  = cond;
        target_o = cond ? pc_i + imm_i : pc_next;
      end
    endcase
    // 误预测不等目标加法：直接目标 pc+imm、JALR 的 rs1+imm 都用无进位比较 (a + b == c) 与预测目标核对，
    // 与条件比较并行 (原来是"加法 -> 64 位比较"串在旁路之后)
    mispred_o = 1'b0;
    unique case (op_i)
      BRU_JAL:   mispred_o = !pred_taken_i || !eq_dir;
      BRU_JALR:  mispred_o = !pred_taken_i || !eq_jalr;
      BRU_AUIPC: mispred_o = 1'b0;
      default:   mispred_o = (cond != pred_taken_i) || (cond && !eq_dir);
    endcase
  end

endmodule
