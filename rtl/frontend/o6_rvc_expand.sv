//==============================================================================
// o6_rvc_expand.sv - RV64C 解压：16 位压缩指令 -> 等价 32 位指令
//
// 纯组合。非压缩指令 (inst[1:0]==11) 原样输出。
// 保留 / 非法编码置 ill_o，此时 inst_o 无意义，由译码器报非法指令 (tval = 原 16 位)。
// HINT 编码 (如 rd=x0 的 C.ADDI/C.LI/C.MV) 是合法的，照常展开成写 x0 的指令。
// 依据：RISC-V Unprivileged ISA 20191213 第 16 章，表 16.5–16.7；另加 Zcb (访存 / 一元 / c.mul) 与 Zcmop (c.mop.n -> 空操作)。
//==============================================================================
`default_nettype none
module o6_rvc_expand
  import o6_isa_pkg::*;
(
  input  wire  [31:0] inst_i,
  output logic [31:0] inst_o,
  output logic        ill_o
);
  wire [15:0] c = inst_i[15:0];
  wire [4:0]  rd    = c[11:7];
  wire [4:0]  rs2   = c[6:2];
  wire [4:0]  rdp   = {2'b01, c[4:2]};   // rd' / rs2'
  wire [4:0]  rs1p  = {2'b01, c[9:7]};   // rs1' / rd'

  // 各类立即数
  wire [9:0]  addi4spn_imm = {c[10:7], c[12:11], c[5], c[6], 2'b00};
  wire [7:0]  ld_imm  = {c[6:5], c[12:10], 3'b000};          // C.LD / C.FLD / C.SD / C.FSD
  wire [6:0]  lw_imm  = {c[5], c[12:10], c[6], 2'b00};       // C.LW / C.SW
  wire [5:0]  imm6    = {c[12], c[6:2]};
  wire [9:0]  addi16sp_imm = {c[12], c[4:3], c[5], c[2], c[6], 4'b0000};
  wire [11:0] j_imm   = {c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], 1'b0};
  wire [8:0]  b_imm   = {c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};
  wire [8:0]  ldsp_imm = {c[4:2], c[12], c[6:5], 3'b000};    // C.LDSP / C.FLDSP
  wire [7:0]  lwsp_imm = {c[3:2], c[12], c[6:4], 2'b00};     // C.LWSP
  wire [8:0]  sdsp_imm = {c[9:7], c[12:10], 3'b000};         // C.SDSP / C.FSDSP
  wire [7:0]  swsp_imm = {c[8:7], c[12:9], 2'b00};           // C.SWSP

  // 指令拼装
  function automatic logic [31:0] i_type(input logic [11:0] imm, input logic [4:0] rs1,
                                         input logic [2:0] f3, input logic [4:0] rd_, input logic [6:0] opc);
    return {imm, rs1, f3, rd_, opc};
  endfunction
  function automatic logic [31:0] s_type(input logic [11:0] imm, input logic [4:0] rs2_, input logic [4:0] rs1,
                                         input logic [2:0] f3, input logic [6:0] opc);
    return {imm[11:5], rs2_, rs1, f3, imm[4:0], opc};
  endfunction
  function automatic logic [31:0] b_type(input logic [12:0] imm, input logic [4:0] rs2_, input logic [4:0] rs1,
                                         input logic [2:0] f3);
    return {imm[12], imm[10:5], rs2_, rs1, f3, imm[4:1], imm[11], OPC_BRANCH};
  endfunction
  function automatic logic [31:0] r_type(input logic [6:0] f7, input logic [4:0] rs2_, input logic [4:0] rs1,
                                         input logic [2:0] f3, input logic [4:0] rd_, input logic [6:0] opc);
    return {f7, rs2_, rs1, f3, rd_, opc};
  endfunction

  always_comb begin
    inst_o = inst_i;
    ill_o  = 1'b0;
    if (inst_i[1:0] != 2'b11) begin
      inst_o = 32'h0;
      unique case ({c[1:0], c[15:13]})
        // ---------------- 象限 0 ----------------
        5'b00_000: begin   // C.ADDI4SPN
          inst_o = i_type({2'b00, addi4spn_imm}, 5'd2, 3'b000, rdp, OPC_OP_IMM);
          ill_o  = (addi4spn_imm == 10'd0);   // 含全零指令
        end
        5'b00_001: inst_o = i_type({4'b0, ld_imm}, rs1p, 3'b011, rdp, OPC_LOAD_FP);   // C.FLD
        5'b00_010: inst_o = i_type({5'b0, lw_imm}, rs1p, 3'b010, rdp, OPC_LOAD);      // C.LW
        5'b00_011: inst_o = i_type({4'b0, ld_imm}, rs1p, 3'b011, rdp, OPC_LOAD);      // C.LD
        5'b00_100: begin   // Zcb 的访存 (2026-10-29)：c.lbu / c.lhu / c.lh / c.sb / c.sh，其余保留
          unique case (c[12:10])
            3'b000: inst_o = i_type({10'b0, c[5], c[6]}, rs1p, 3'b100, rdp, OPC_LOAD);       // C.LBU
            3'b001: inst_o = i_type({10'b0, c[5], 1'b0}, rs1p, c[6] ? 3'b001 : 3'b101, rdp, OPC_LOAD); // C.LH / C.LHU
            3'b010: inst_o = s_type({10'b0, c[5], c[6]}, rdp, rs1p, 3'b000, OPC_STORE);      // C.SB
            3'b011: begin                                                                     // C.SH
              inst_o = s_type({10'b0, c[5], 1'b0}, rdp, rs1p, 3'b001, OPC_STORE);
              ill_o  = c[6];
            end
            default: ill_o = 1'b1;
          endcase
        end
        5'b00_101: inst_o = s_type({4'b0, ld_imm}, rdp, rs1p, 3'b011, OPC_STORE_FP);  // C.FSD
        5'b00_110: inst_o = s_type({5'b0, lw_imm}, rdp, rs1p, 3'b010, OPC_STORE);     // C.SW
        5'b00_111: inst_o = s_type({4'b0, ld_imm}, rdp, rs1p, 3'b011, OPC_STORE);     // C.SD
        // ---------------- 象限 1 ----------------
        5'b01_000: inst_o = i_type({{6{imm6[5]}}, imm6}, rd, 3'b000, rd, OPC_OP_IMM);  // C.ADDI / C.NOP
        5'b01_001: begin   // C.ADDIW
          inst_o = i_type({{6{imm6[5]}}, imm6}, rd, 3'b000, rd, OPC_OP_IMM32);
          ill_o  = (rd == 5'd0);
        end
        5'b01_010: inst_o = i_type({{6{imm6[5]}}, imm6}, 5'd0, 3'b000, rd, OPC_OP_IMM); // C.LI
        5'b01_011: begin
          if (rd == 5'd2) begin   // C.ADDI16SP
            inst_o = i_type({{2{addi16sp_imm[9]}}, addi16sp_imm}, 5'd2, 3'b000, 5'd2, OPC_OP_IMM);
            ill_o  = (addi16sp_imm == 10'd0);
          end else if (imm6 == 6'd0 && !rd[4] && rd[0]) begin   // Zcmop c.mop.n (rd = x1, x3 .. x15、立即数 0)：空操作
            inst_o = 32'h0000_0013;
          end else begin          // C.LUI
            inst_o = {{14{imm6[5]}}, imm6, rd, OPC_LUI};
            ill_o  = (imm6 == 6'd0);
          end
        end
        5'b01_100: begin
          unique case (c[11:10])
            2'b00: inst_o = i_type({6'b000000, imm6}, rs1p, 3'b101, rs1p, OPC_OP_IMM);  // C.SRLI
            2'b01: inst_o = i_type({6'b010000, imm6}, rs1p, 3'b101, rs1p, OPC_OP_IMM);  // C.SRAI
            2'b10: inst_o = i_type({{6{imm6[5]}}, imm6}, rs1p, 3'b111, rs1p, OPC_OP_IMM); // C.ANDI
            2'b11: begin
              unique case ({c[12], c[6:5]})
                3'b0_00: inst_o = r_type(7'b0100000, rdp, rs1p, 3'b000, rs1p, OPC_OP);   // C.SUB
                3'b0_01: inst_o = r_type(7'b0000000, rdp, rs1p, 3'b100, rs1p, OPC_OP);   // C.XOR
                3'b0_10: inst_o = r_type(7'b0000000, rdp, rs1p, 3'b110, rs1p, OPC_OP);   // C.OR
                3'b0_11: inst_o = r_type(7'b0000000, rdp, rs1p, 3'b111, rs1p, OPC_OP);   // C.AND
                3'b1_00: inst_o = r_type(7'b0100000, rdp, rs1p, 3'b000, rs1p, OPC_OP32); // C.SUBW
                3'b1_01: inst_o = r_type(7'b0000000, rdp, rs1p, 3'b000, rs1p, OPC_OP32); // C.ADDW
                3'b1_10: inst_o = r_type(7'b0000001, rdp, rs1p, 3'b000, rs1p, OPC_OP);   // Zcb C.MUL
                3'b1_11: begin                                                         // Zcb 一元运算 (rd' = rs1')
                  unique case (c[4:2])
                    3'b000: inst_o = i_type(12'h0FF, rs1p, 3'b111, rs1p, OPC_OP_IMM);   // C.ZEXT.B = andi 255
                    3'b001: inst_o = i_type(12'h604, rs1p, 3'b001, rs1p, OPC_OP_IMM);   // C.SEXT.B (Zbb)
                    3'b010: inst_o = r_type(7'b0000100, 5'd0, rs1p, 3'b100, rs1p, OPC_OP32); // C.ZEXT.H (Zbb)
                    3'b011: inst_o = i_type(12'h605, rs1p, 3'b001, rs1p, OPC_OP_IMM);   // C.SEXT.H (Zbb)
                    3'b100: inst_o = r_type(7'b0000100, 5'd0, rs1p, 3'b000, rs1p, OPC_OP32); // C.ZEXT.W = add.uw (Zba)
                    3'b101: inst_o = i_type(12'hFFF, rs1p, 3'b100, rs1p, OPC_OP_IMM);   // C.NOT = xori -1
                    default: ill_o = 1'b1;
                  endcase
                end
                default: ill_o = 1'b1;
              endcase
            end
          endcase
        end
        5'b01_101: inst_o = {j_imm[11], j_imm[10:1], j_imm[11], {8{j_imm[11]}}, 5'd0, OPC_JAL}; // C.J
        5'b01_110: inst_o = b_type({{4{b_imm[8]}}, b_imm}, 5'd0, rs1p, 3'b000);  // C.BEQZ
        5'b01_111: inst_o = b_type({{4{b_imm[8]}}, b_imm}, 5'd0, rs1p, 3'b001);  // C.BNEZ
        // ---------------- 象限 2 ----------------
        5'b10_000: inst_o = i_type({6'b000000, imm6}, rd, 3'b001, rd, OPC_OP_IMM);       // C.SLLI
        5'b10_001: inst_o = i_type({3'b0, ldsp_imm}, 5'd2, 3'b011, rd, OPC_LOAD_FP);     // C.FLDSP
        5'b10_010: begin   // C.LWSP
          inst_o = i_type({4'b0, lwsp_imm}, 5'd2, 3'b010, rd, OPC_LOAD);
          ill_o  = (rd == 5'd0);
        end
        5'b10_011: begin   // C.LDSP
          inst_o = i_type({3'b0, ldsp_imm}, 5'd2, 3'b011, rd, OPC_LOAD);
          ill_o  = (rd == 5'd0);
        end
        5'b10_100: begin
          if (!c[12]) begin
            if (rs2 == 5'd0) begin   // C.JR
              inst_o = i_type(12'd0, rd, 3'b000, 5'd0, OPC_JALR);
              ill_o  = (rd == 5'd0);
            end else begin           // C.MV
              inst_o = r_type(7'b0, rs2, 5'd0, 3'b000, rd, OPC_OP);
            end
          end else begin
            if (rs2 == 5'd0) begin
              if (rd == 5'd0) inst_o = 32'h0010_0073;                         // C.EBREAK
              else            inst_o = i_type(12'd0, rd, 3'b000, 5'd1, OPC_JALR); // C.JALR
            end else begin           // C.ADD
              inst_o = r_type(7'b0, rs2, rd, 3'b000, rd, OPC_OP);
            end
          end
        end
        5'b10_101: inst_o = s_type({3'b0, sdsp_imm}, rs2, 5'd2, 3'b011, OPC_STORE_FP);   // C.FSDSP
        5'b10_110: inst_o = s_type({4'b0, swsp_imm}, rs2, 5'd2, 3'b010, OPC_STORE);      // C.SWSP
        5'b10_111: inst_o = s_type({3'b0, sdsp_imm}, rs2, 5'd2, 3'b011, OPC_STORE);      // C.SDSP
        default:   ill_o  = 1'b1;
      endcase
    end
  end
endmodule
