//==============================================================================
// o6_decoder.sv - 单条指令译码 (RV64IMAFD + Zicsr + Zifencei，RVC 已在前端展开)
//
// 纯组合。输出 uop_t 中"译码"部分的字段，重命名 / 分派字段清零。
//
// 合法性分工：
//   这里判：编码是否存在、mstatus.FS=Off 时的浮点指令、舍入模式非法、ECALL/EBREAK
//   CSR 单元 (ROB 头) 判：CSR 地址/权限/只读、MRET/SRET/WFI/SFENCE.VMA 的特权条件
//   原因：后者依赖的 CSR 状态只在 ROB 头才精确，而前者依赖的 priv/FS/frm
//   只会被冲刷流水的指令改变，译码时读到的一定是当前值。
//==============================================================================
`default_nettype none
module o6_decoder
  import o6_pkg::*;
  import o6_isa_pkg::*;
(
  input  var fetch_inst_t fi_i,
  input  var priv_e  priv_i,
  input  wire [1:0] fs_i,       // mstatus.FS
  input  wire [1:0] vs_i,       // mstatus.VS
  input  wire [2:0] frm_i,      // fcsr.frm
  input  wire [2:0] cbo_ok_i,   // Zicbom / Zicboz 当前特权级可执行 {zero, clean / flush, inval} (o6_csr)
  input  wire [2:0] cbo_vi_i,   // 不可执行时报 virtual instruction (V = 1 且 henvcfg 位 = 0)
  input  wire       virt_i,     // H：V 位 (ecall 原因码)
  input  wire       hlv_ok_i,   // H：HLV / HSV 可执行 (o6_csr：有 H、V = 0、且 (S 及以上 或 hstatus.HU))
  input  wire       hlv_vi_i,   // H：不可执行时报 virtual instruction (V = 1)
  output uop_t         uop_o
);
  wire [31:0] i   = fi_i.inst;
  wire [6:0]  opc = i[6:0];
  wire [2:0]  f3  = i[14:12];
  wire [6:0]  f7  = i[31:25];
  wire [4:0]  f5  = i[31:27];
  wire [4:0]  rd  = i[11:7];
  wire [4:0]  rs1 = i[19:15];
  wire [4:0]  rs2 = i[24:20];

  wire [31:0] imm_i = {{20{i[31]}}, i[31:20]};
  wire [31:0] imm_s = {{20{i[31]}}, i[31:25], i[11:7]};
  wire [31:0] imm_b = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0};
  wire [31:0] imm_u = {i[31:12], 12'b0};
  wire [31:0] imm_j = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0};

  // 舍入模式合法：静态 rm 0..4，或 7 且 frm 0..4
  wire rm_ok   = (f3 <= 3'd4) || (f3 == 3'd7 && frm_i <= 3'd4);
  wire fp_on   = (fs_i != 2'b00);
  wire vec_on  = (vs_i != 2'b00);
  wire [5:0] f6 = i[31:26];
  // fmt：00 = S，01 = D，其它 (H/Q) 不支持
  wire fmt_ok  = (i[26:25] == 2'b00) || (i[26:25] == 2'b01);

  logic ill;
  logic vill;      // ill 的原因是 virtual instruction (cause 22)

  function automatic rclass_e int_rc(input logic [4:0] r);
    return (r == 5'd0) ? RC_NONE : RC_INT;
  endfunction

  uop_t u;

  always_comb begin
    u = '0;
    ill = 1'b0; vill = 1'b0;

    u.fu        = FU_NONE;
    u.raw       = fi_i.raw;
    u.is_rvc    = fi_i.is_rvc;
    u.ftq_idx   = fi_i.ftq_idx;
    u.ftq_off   = fi_i.ftq_off;
    u.pred_taken= fi_i.pred_taken; u.ld_wait = fi_i.ld_wait; u.pch = fi_i.pch;
    u.rm        = f3;
    u.fp_dbl    = i[25];
    u.lrs1      = rs1;
    u.lrs2      = rs2;
    u.lrs3      = i[31:27];
    u.rs1_class = (rs1 == 5'd0) ? RC_NONE : RC_INT;
    u.rs2_class = RC_NONE;
    u.rd_class  = RC_NONE;

    unique case (opc)
      //------------------------------------------------------------------ 整数
      OPC_LUI: begin
        u.fu = FU_ALU; u.op = ALU_LUI; u.imm = imm_u; u.use_imm = 1'b1;
        u.rs1_class = RC_NONE; u.lrd = rd; u.rd_class = int_rc(rd);
      end
      OPC_AUIPC: begin
        u.fu = FU_BRU; u.op = BRU_AUIPC; u.imm = imm_u; u.use_imm = 1'b1;
        u.rs1_class = RC_NONE; u.lrd = rd; u.rd_class = int_rc(rd);
      end
      OPC_JAL: begin
        u.fu = FU_BRU; u.op = BRU_JAL; u.imm = imm_j; u.use_imm = 1'b1;
        u.rs1_class = RC_NONE; u.lrd = rd; u.rd_class = int_rc(rd);
      end
      OPC_JALR: begin
        u.fu = FU_BRU; u.op = BRU_JALR; u.imm = imm_i; u.use_imm = 1'b1;
        u.is_br = 1'b1; u.lrd = rd; u.rd_class = int_rc(rd);
        ill = (f3 != 3'b000);
      end
      OPC_BRANCH: begin
        u.fu = FU_BRU; u.op = {2'b00, f3}; u.imm = imm_b;
        u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
        u.is_br = 1'b1;
        ill = (f3 == 3'b010) || (f3 == 3'b011);
      end
      OPC_OP_IMM, OPC_OP_IMM32: begin
        u.fu = FU_ALU; u.imm = imm_i; u.use_imm = 1'b1; u.lrd = rd; u.rd_class = int_rc(rd);
        u.is_w = (opc == OPC_OP_IMM32);
        unique case (f3)
          3'b000: u.op = ALU_ADD;
          3'b010: begin u.op = ALU_SLT;  ill = u.is_w; end
          3'b011: begin u.op = ALU_SLTU; ill = u.is_w; end
          3'b100: begin u.op = ALU_XOR;  ill = u.is_w; end
          3'b110: begin u.op = ALU_OR;   ill = u.is_w; end
          3'b111: begin u.op = ALU_AND;  ill = u.is_w; end
          3'b001: begin
            // RV64I：slli / slliw；Zbb：clz/ctz/cpop(w)、sext.b/h；Zbs：bclri/binvi/bseti；Zba：slli.uw
            if (!u.is_w && i[31:26] == 6'b000000)      u.op = ALU_SLL;
            else if (u.is_w && i[31:25] == 7'b0000000) u.op = ALU_SLL;
            // clz / ctz / cpop(w) 走 MUL 流水 (见 o6_pkg MUL_CLZ)
            else if (i[31:20] == 12'h600) begin u.fu = FU_MUL; u.op = MUL_CLZ;  u.use_imm = 1'b0; end
            else if (i[31:20] == 12'h601) begin u.fu = FU_MUL; u.op = MUL_CTZ;  u.use_imm = 1'b0; end
            else if (i[31:20] == 12'h602) begin u.fu = FU_MUL; u.op = MUL_CPOP; u.use_imm = 1'b0; end
            else if (!u.is_w && i[31:20] == 12'h604) begin u.op = ALU_SEXTB; u.use_imm = 1'b0; end
            else if (!u.is_w && i[31:20] == 12'h605) begin u.op = ALU_SEXTH; u.use_imm = 1'b0; end
            else if (!u.is_w && i[31:26] == 6'b010010) u.op = ALU_BCLR;
            else if (!u.is_w && i[31:26] == 6'b011010) u.op = ALU_BINV;
            else if (!u.is_w && i[31:26] == 6'b001010) u.op = ALU_BSET;
            else if (u.is_w && i[31:26] == 6'b000010)  begin u.op = ALU_SLL; u.auw = 1'b1; u.is_w = 1'b0; end  // slli.uw
            else ill = 1'b1;
          end
          3'b101: begin
            // RV64I：srli/srai(w)；Zbb：rori(w)、orc.b、rev8；Zbs：bexti
            if (u.is_w ? ({i[31], i[29:25]} == 6'b0) : ({i[31], i[29:26]} == 5'b0)) u.op = i[30] ? ALU_SRA : ALU_SRL;
            else if (!u.is_w && i[31:26] == 6'b011000)  u.op = ALU_ROR;
            else if (u.is_w && i[31:25] == 7'b0110000)  u.op = ALU_ROR;
            else if (!u.is_w && i[31:20] == 12'h287)    begin u.op = ALU_ORCB; u.use_imm = 1'b0; end
            else if (!u.is_w && i[31:20] == 12'h6B8)    begin u.op = ALU_REV8; u.use_imm = 1'b0; end
            else if (!u.is_w && i[31:26] == 6'b010010)  u.op = ALU_BEXT;
            else ill = 1'b1;
          end
          default: ill = 1'b1;
        endcase
      end
      OPC_OP, OPC_OP32: begin
        u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
        u.is_w = (opc == OPC_OP32);
        u.lrd = rd; u.rd_class = int_rc(rd);
        if (f7 == 7'b0000001) begin
          if (!f3[2]) begin
            u.fu = FU_MUL; u.op = {3'b0, f3[1:0]};
            ill  = u.is_w && (f3 != 3'b000);       // RV64 只有 MULW
          end else begin
            u.fu = FU_DIV; u.op = {3'b0, f3[1:0]}; // 100 DIV 101 DIVU 110 REM 111 REMU
          end
        end else if (f7 == 7'b0000000 || f7 == 7'b0100000) begin
          u.fu = FU_ALU;
          unique case ({f7[5], f3})
            4'b0_000: u.op = ALU_ADD;
            4'b1_000: u.op = ALU_SUB;
            4'b0_001: u.op = ALU_SLL;
            4'b0_010: begin u.op = ALU_SLT;  ill = u.is_w; end
            4'b0_011: begin u.op = ALU_SLTU; ill = u.is_w; end
            4'b0_100: begin u.op = ALU_XOR;  ill = u.is_w; end
            4'b0_101: u.op = ALU_SRL;
            4'b1_101: u.op = ALU_SRA;
            4'b0_110: begin u.op = ALU_OR;   ill = u.is_w; end
            4'b0_111: begin u.op = ALU_AND;  ill = u.is_w; end
            4'b1_111: begin u.op = ALU_ANDN; ill = u.is_w; end     // Zbb
            4'b1_110: begin u.op = ALU_ORN;  ill = u.is_w; end
            4'b1_100: begin u.op = ALU_XNOR; ill = u.is_w; end
            default:  ill = 1'b1;
          endcase
        end else begin
          // B 扩展的寄存器-寄存器形式
          u.fu = FU_ALU;
          unique casez ({f7, f3})
            {7'b0010000, 3'b010}: begin u.op = ALU_ADD; u.ash = 2'd1; u.auw = u.is_w; u.is_w = 1'b0; end  // sh1add(.uw)
            {7'b0010000, 3'b100}: begin u.op = ALU_ADD; u.ash = 2'd2; u.auw = u.is_w; u.is_w = 1'b0; end
            {7'b0010000, 3'b110}: begin u.op = ALU_ADD; u.ash = 2'd3; u.auw = u.is_w; u.is_w = 1'b0; end
            {7'b0000100, 3'b000}: begin u.op = ALU_ADD; u.auw = 1'b1; ill = !u.is_w; u.is_w = 1'b0; end     // add.uw
            {7'b0000100, 3'b100}: begin                                                                       // zext.h
              u.op = ALU_ZEXTH; ill = !u.is_w || (rs2 != 5'd0); u.is_w = 1'b0; u.rs2_class = RC_NONE;
            end
            {7'b0000101, 3'b110}: begin u.op = ALU_MAX;  ill = u.is_w; end
            {7'b0000101, 3'b111}: begin u.op = ALU_MAXU; ill = u.is_w; end
            {7'b0000101, 3'b100}: begin u.op = ALU_MIN;  ill = u.is_w; end
            {7'b0000101, 3'b101}: begin u.op = ALU_MINU; ill = u.is_w; end
            {7'b0110000, 3'b001}: u.op = ALU_ROL;                                                              // rol(w)
            {7'b0110000, 3'b101}: u.op = ALU_ROR;                                                              // ror(w)
            {7'b0100100, 3'b001}: begin u.op = ALU_BCLR; ill = u.is_w; end
            {7'b0100100, 3'b101}: begin u.op = ALU_BEXT; ill = u.is_w; end
            {7'b0110100, 3'b001}: begin u.op = ALU_BINV; ill = u.is_w; end
            {7'b0010100, 3'b001}: begin u.op = ALU_BSET; ill = u.is_w; end
            {7'b0000111, 3'b101}: begin u.op = ALU_CZEQZ; ill = u.is_w; end                                  // Zicond
            {7'b0000111, 3'b111}: begin u.op = ALU_CZNEZ; ill = u.is_w; end
            default: ill = 1'b1;
          endcase
        end
      end
      //------------------------------------------------------------------ 访存
      OPC_LOAD: begin
        u.fu = FU_LD; u.imm = imm_i; u.use_imm = 1'b1; u.lrd = rd; u.rd_class = int_rc(rd);
        u.mem_sz = f3[1:0]; u.mem_unsigned = f3[2];
        ill = (f3 == 3'b111);
      end
      OPC_STORE: begin
        u.fu = FU_ST; u.imm = imm_s; u.use_imm = 1'b1;
        u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
        u.mem_sz = f3[1:0];
        ill = f3[2];
      end
      OPC_LOAD_FP: begin
        if (vmem_w(f3)) begin
          // 向量 load：rs1 基址，跨步 (mop = 10) 时 rs2 是步长；完整译码与 vtype 相关的合法性在向量单元
          u.fu = FU_VEC; u.wait_head = 1'b1;
          u.rs2_class = (i[27:26] == 2'b10) ? int_rc(rs2) : RC_NONE;
          ill = !vec_on;
        end else begin
          u.fu = FU_LD; u.imm = imm_i; u.use_imm = 1'b1;
          u.lrd = rd; u.rd_class = RC_FP;
          u.mem_sz = f3[1:0]; u.mem_unsigned = 1'b1;
          u.fp_dbl = (f3 == 3'b011);
          ill = !fp_on || !(f3 == 3'b001 || f3 == 3'b010 || f3 == 3'b011);     // FLH (Zfhmin) / FLW / FLD
        end
      end
      OPC_STORE_FP: begin
        if (vmem_w(f3)) begin
          // 向量 store：不经过标量 SQ，更年轻的标量 load 可能已读到旧值 -> 完成后冲刷重取
          u.fu = FU_VEC; u.wait_head = 1'b1; u.flush_after = 1'b1;
          u.rs2_class = (i[27:26] == 2'b10) ? int_rc(rs2) : RC_NONE;
          ill = !vec_on;
        end else begin
          u.fu = FU_ST; u.imm = imm_s; u.use_imm = 1'b1;
          u.rs2_class = RC_FP;
          u.mem_sz = f3[1:0];
          u.fp_dbl = (f3 == 3'b011);
          ill = !fp_on || !(f3 == 3'b001 || f3 == 3'b010 || f3 == 3'b011);     // FSH (Zfhmin) / FSW / FSD
        end
      end
      //------------------------------------------------------------------ 向量 (RVV 1.0)
      // 这里只定 FU_VEC、标量操作数 / 标量结果的寄存器类；完整译码在向量单元 (按原始编码)
      OPC_OP_V: begin
        u.fu = FU_VEC; u.wait_head = 1'b1; u.rs1_class = RC_NONE;
        ill = !vec_on;
        unique case (f3)
          3'b111: begin                                  // vsetvli / vsetivli / vsetvl
            u.lrd = rd; u.rd_class = int_rc(rd);
            if (!i[31])                     u.rs1_class = int_rc(rs1);
            else if (i[31:30] == 2'b11)     u.rs1_class = RC_NONE;          // uimm
            else if (i[31:25] == 7'b1000000) begin u.rs1_class = int_rc(rs1); u.rs2_class = int_rc(rs2); end
            else                            ill = 1'b1;
          end
          3'b100, 3'b110: u.rs1_class = int_rc(rs1);   // OPIVX / OPMVX
          3'b101: begin                                  // OPFVF：标量浮点操作数 (P3 从浮点寄存器堆读)
            u.rs1_class = RC_FP; ill = !vec_on || !fp_on;
          end
          3'b010: if (f6 == 6'b010000) begin             // VWXUNARY0：vmv.x.s / vcpop.m / vfirst.m 写整数 rd
            u.lrd = rd; u.rd_class = int_rc(rd);
          end
          3'b001: begin                                  // OPFVV：要求 FS 打开
            ill = !vec_on || !fp_on;
            if (f6 == 6'b010000) begin u.lrd = rd; u.rd_class = RC_FP; end   // VWFUNARY0：vfmv.f.s 写浮点 rd
          end
          default: ;                                     // OPIVV / OPFVV / OPMVV / OPIVI
        endcase
      end
      // 向量点积 (OP-VE)：vd[0] += vs1 . vs2；整数 funct3 = 000，浮点 funct3 = 001 (要求 FS 打开)
      OPC_OP_VE: begin
        u.fu = FU_VEC; u.wait_head = 1'b1; u.rs1_class = RC_NONE;
        ill = !vec_on || !(f3 == 3'b000 || (f3 == 3'b001 && fp_on));
      end
      OPC_AMO: begin
        u.fu = FU_AMO; u.wait_head = 1'b1; u.lrd = rd; u.rd_class = int_rc(rd);
        u.mem_sz = f3[1:0]; u.aq = i[26]; u.rl = i[25];
        u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
        ill = !(f3 == 3'b010 || f3 == 3'b011);
        unique case (f5)
          5'b00010: begin u.op = AMO_LR; u.rs2_class = RC_NONE; ill = ill || (rs2 != 5'd0); end
          5'b00011: u.op = AMO_SC;
          5'b00001: u.op = AMO_SWAP;
          5'b00000: u.op = AMO_ADD;
          5'b00100: u.op = AMO_XOR;
          5'b01100: u.op = AMO_AND;
          5'b01000: u.op = AMO_OR;
          5'b10000: u.op = AMO_MIN;
          5'b10100: u.op = AMO_MAX;
          5'b11000: u.op = AMO_MINU;
          5'b11100: u.op = AMO_MAXU;
          default:  ill = 1'b1;
        endcase
      end
      OPC_MISC_MEM: begin
        u.fu = FU_CSR; u.wait_head = 1'b1; u.rs1_class = RC_NONE;
        unique case (f3)
          3'b000: u.op = SYS_FENCE;
          3'b001: begin u.op = SYS_FENCEI; u.flush_after = 1'b1; end
          3'b010: begin
            // Zicbom / Zicboz：cbo.inval (0) / clean (1) / flush (2) / zero (4)，rd = 0，基址 rs1 (无偏移)。
            // 按 AMO 走 (ROB 头执行)，完成后冲刷 (更年轻的 load 一定在它之后重新执行，不必按 64 B 做转发 / 违例)
            u.fu = FU_AMO; u.mem_sz = 2'd3; u.rs1_class = int_rc(rs1); u.rd_class = RC_NONE; u.lrd = '0;
            u.imm = '0; u.use_imm = 1'b1; u.flush_after = 1'b1;
            unique case (i[31:20])
              12'h000: begin u.op = AMO_CBOF; ill = !cbo_ok_i[0]; vill = cbo_vi_i[0]; end     // inval：按 flush 做
              12'h001: begin u.op = AMO_CBOC; ill = !cbo_ok_i[1]; vill = cbo_vi_i[1]; end
              12'h002: begin u.op = AMO_CBOF; ill = !cbo_ok_i[1]; vill = cbo_vi_i[1]; end
              12'h004: begin u.op = AMO_CBOZ; ill = !cbo_ok_i[2]; vill = cbo_vi_i[2]; end
              default: ill = 1'b1;
            endcase
            if (rd != 5'd0) begin ill = 1'b1; vill = 1'b0; end
          end
          default: ill = 1'b1;
        endcase
      end
      //------------------------------------------------------------------ 系统
      OPC_SYSTEM: begin
        u.fu = FU_CSR; u.wait_head = 1'b1;
        if (f3 == 3'b000) begin
          u.rs1_class = RC_NONE;
          if (i == 32'h0000_0073) begin
            u.op = SYS_ECALL; u.excp = 1'b1; u.fu = FU_NONE; u.wait_head = 1'b0;
            u.cause = (priv_i == PRV_M) ? EXC_ECALL_M :
                      (priv_i == PRV_S) ? (virt_i ? EXC_ECALL_VS : EXC_ECALL_S) : EXC_ECALL_U;
          end else if (i == 32'h0010_0073) begin
            u.op = SYS_EBREAK; u.excp = 1'b1; u.cause = EXC_BREAKPOINT; u.fu = FU_NONE; u.wait_head = 1'b0;
          end else if (i == 32'h3020_0073) begin
            u.op = SYS_MRET;  u.flush_after = 1'b1;
          end else if (i == 32'h1020_0073) begin
            u.op = SYS_SRET;  u.flush_after = 1'b1;
          end else if (i == 32'h7b20_0073) begin
            u.op = SYS_DRET;  u.flush_after = 1'b1;          // 非调试模式下由 CSR 执行口判非法
          end else if (i == 32'h1050_0073) begin
            u.op = SYS_WFI;
          end else if (i == 32'h00d0_0073) begin
            u.op = SYS_WRS;                                 // Zawrs wrs.nto：立即结束，CSR 执行口按 TW 判非法
          end else if (i == 32'h01d0_0073) begin
            u.fu = FU_ALU; u.op = ALU_ADD; u.wait_head = 1'b0;   // wrs.sto：立即结束 (不受 TW 限制)，空操作
          end else if ((f7 == 7'b0001001 || f7 == 7'b0001011) && rd == 5'd0) begin
            // sfence.vma，或 Svinval 的 sinval.vma (按 sfence.vma 做：整表清 + 冲刷，权限检查相同)
            u.op = SYS_SFENCE; u.flush_after = 1'b1;
            u.rs1_class = (rs1 == 5'd0) ? RC_NONE : RC_INT;
            u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
          end else if (i == 32'h1800_0073 || i == 32'h1810_0073) begin
            u.op = SYS_INVF;                               // Svinval sfence.w.inval / sfence.inval.ir
          end else if ((f7 == 7'b0010001 || f7 == 7'b0010011 || f7 == 7'b0110001 || f7 == 7'b0110011) && rd == 5'd0) begin
            // H：hfence.vvma / hinval.vvma (0010001 / 0010011)、hfence.gvma / hinval.gvma (0110001 / 0110011)：
            //   同 sfence.vma 的流程 (整表清 + 冲刷)，合法性在 CSR 执行口 (V = 1 -> virtual 等)
            u.op = f7[5] ? SYS_HFENCEG : SYS_HFENCEV; u.flush_after = 1'b1;
            u.rs1_class = (rs1 == 5'd0) ? RC_NONE : RC_INT;
            u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
          end else begin
            ill = 1'b1;
          end
        end else if (f3 == 3'b100) begin
          // Zimop：mop.r.n (i[31] = 1、i[29:28] = 00、i[25:22] = 0111) / mop.rr.n (i[31] = 1、i[29:28] = 00、i[25] = 1) 写 rd = 0
          if (i[31] && i[29:28] == 2'b00 && (i[25] || i[25:22] == 4'b0111)) begin
            u.fu = FU_ALU; u.op = ALU_LUI; u.imm = '0; u.use_imm = 1'b1; u.wait_head = 1'b0;
            u.rs1_class = RC_NONE; u.rs2_class = RC_NONE; u.lrd = rd; u.rd_class = int_rc(rd);
          end else if (i[31:28] == 4'b0110 && !i[25] &&
                       (rs2 == 5'd0 || (rs2 == 5'd1 && i[27:26] != 2'b11) || (rs2 == 5'd3 && (i[27:26] == 2'b01 || i[27:26] == 2'b10)))) begin
            // H：HLV.{B,BU,H,HU,W,WU,D} / HLVX.{HU,WU} (f7 = 0110 sz 0，rs2 = 0 有符号 / 1 无符号 / 3 HLVX)：
            //   普通 load 流水，翻译用 hctx (V = 1，特权级 hstatus.SPVP)；HLVX 要执行权限
            u.fu = FU_LD; u.imm = '0; u.use_imm = 1'b1; u.lrd = rd; u.rd_class = int_rc(rd); u.wait_head = 1'b0;
            u.rs1_class = int_rc(rs1); u.rs2_class = RC_NONE;
            u.mem_sz = i[27:26]; u.mem_unsigned = rs2[0];
            u.op = (rs2 == 5'd3) ? LS_HLVX : LS_HV;
            ill = !hlv_ok_i; vill = hlv_vi_i;
          end else if (i[31:28] == 4'b0110 && i[25] && rd == 5'd0) begin
            // H：HSV.{B,H,W,D} (f7 = 0110 sz 1，rs2 = 数据)
            u.fu = FU_ST; u.imm = '0; u.use_imm = 1'b1; u.wait_head = 1'b0;
            u.rs1_class = int_rc(rs1); u.rs2_class = (rs2 == 5'd0) ? RC_NONE : RC_INT;
            u.mem_sz = i[27:26]; u.op = LS_HV;
            ill = !hlv_ok_i; vill = hlv_vi_i;
          end else begin
            ill = 1'b1;
          end
        end else begin
          u.op = {2'b00, f3};       // 001 RW 010 RS 011 RC 101 RWI 110 RSI 111 RCI
          u.csr_addr = i[31:20];
          u.imm = {27'b0, rs1};     // zimm
          u.lrd = rd; u.rd_class = int_rc(rd);
          if (f3[2]) u.rs1_class = RC_NONE;
          // 是否写 CSR：RW 总写；RS/RC 在 rs1 / zimm 非零时写
          u.flush_after = (f3[1:0] == 2'b01) || (rs1 != 5'd0);
          // 向量 CSR 在向量单元：同步执行 (先等它排空)，不影响取指，不必冲刷
          if (i[31:20] == CSR_VSTART || i[31:20] == CSR_VXSAT || i[31:20] == CSR_VXRM || i[31:20] == CSR_VCSR ||
              i[31:20] == CSR_VL || i[31:20] == CSR_VTYPE || i[31:20] == CSR_VLENB) begin
            u.fu = FU_VEC; u.flush_after = 1'b0;
            ill = !vec_on;
          end
        end
      end
      //------------------------------------------------------------------ 浮点
      OPC_MADD, OPC_MSUB, OPC_NMSUB, OPC_NMADD: begin
        u.fu = FU_FMA;
        u.op = (opc == OPC_MADD)  ? FMA_MADD  :
               (opc == OPC_MSUB)  ? FMA_MSUB  :
               (opc == OPC_NMSUB) ? FMA_NMSUB : FMA_NMADD;
        u.rs1_class = RC_FP; u.rs2_class = RC_FP; u.rs3_en = 1'b1;
        u.lrd = rd; u.rd_class = RC_FP;
        ill = !fp_on || !fmt_ok || !rm_ok;
      end
      OPC_OP_FP: begin
        u.lrd = rd; u.rd_class = RC_FP;
        u.rs1_class = RC_FP; u.rs2_class = RC_FP;
        // fmt = H 只有 Zfhmin / Zfbfmin 的转换与 FMV (没有完整 Zfh)
        ill = !fp_on || !(fmt_ok || (i[26:25] == 2'b10 &&
                                     (f7[6:2] == 5'b01000 || f7[6:2] == 5'b11100 || f7[6:2] == 5'b11110)));
        unique case (f7[6:2])
          5'b00000: begin u.fu = FU_FMA;  u.op = FMA_ADD;  ill = ill || !rm_ok; end
          5'b00001: begin u.fu = FU_FMA;  u.op = FMA_SUB;  ill = ill || !rm_ok; end
          5'b00010: begin u.fu = FU_FMA;  u.op = FMA_MUL;  ill = ill || !rm_ok; end
          5'b00011: begin u.fu = FU_FDIV; u.op = FDIV_DIV; ill = ill || !rm_ok; end
          5'b01011: begin
            u.fu = FU_FDIV; u.op = FDIV_SQRT; u.rs2_class = RC_NONE;
            ill = ill || !rm_ok || (rs2 != 5'd0);
          end
          5'b00100: begin
            u.fu = FU_FMISC;
            unique case (f3)
              3'b000: u.op = FMISC_SGNJ;
              3'b001: u.op = FMISC_SGNJN;
              3'b010: u.op = FMISC_SGNJX;
              default: ill = 1'b1;
            endcase
          end
          5'b00101: begin
            u.fu = FU_FMISC;
            unique case (f3)
              3'b000: u.op = FMISC_MIN;
              3'b001: u.op = FMISC_MAX;
              3'b010: u.op = FMISC_MINM;             // Zfa
              3'b011: u.op = FMISC_MAXM;
              default: ill = 1'b1;
            endcase
          end
          5'b01000: begin   // FCVT.<目的>.<源>：fmt 是目的格式，rs2 是源格式
            u.fu = FU_FMISC; u.rs2_class = RC_NONE;
            ill = ill || !rm_ok;
            unique case ({i[26:25], rs2})
              {2'b00, 5'd1}: u.op = FMISC_CVTFF;     // FCVT.S.D
              {2'b01, 5'd0}: u.op = FMISC_CVTFF;     // FCVT.D.S
              {2'b00, 5'd2}: u.op = FMISC_CVTSH;
              {2'b10, 5'd0}: u.op = FMISC_CVTHS;
              {2'b01, 5'd2}: u.op = FMISC_CVTDH;
              {2'b10, 5'd1}: u.op = FMISC_CVTHD;
              {2'b00, 5'd6}: u.op = FMISC_CVTSB;     // FCVT.S.BF16 (Zfbfmin)
              {2'b10, 5'd8}: u.op = FMISC_CVTBS;     // FCVT.BF16.S (Zfbfmin)
              {2'b00, 5'd4}, {2'b01, 5'd4}: u.op = FMISC_ROUND;     // Zfa fround.{s,d} (.h 要 Zfh)
              {2'b00, 5'd5}, {2'b01, 5'd5}: u.op = FMISC_ROUNDNX;
              default:       ill = 1'b1;
            endcase
          end
          5'b10100: begin
            u.fu = FU_F2I; u.lrd = rd; u.rd_class = int_rc(rd);
            unique case (f3)
              3'b010: u.op = F2I_EQ;
              3'b001: u.op = F2I_LT;
              3'b000: u.op = F2I_LE;
              3'b100: u.op = F2I_LEQ;                // Zfa
              3'b101: u.op = F2I_LTQ;
              default: ill = 1'b1;
            endcase
          end
          5'b11100: begin
            u.fu = FU_F2I; u.lrd = rd; u.rd_class = int_rc(rd); u.rs2_class = RC_NONE;
            unique case (f3)
              3'b000: u.op = (i[26:25] == 2'b10) ? F2I_MVH : F2I_MV;
              3'b001: begin u.op = F2I_CLASS; ill = ill || (i[26:25] == 2'b10); end
              default: ill = 1'b1;
            endcase
            ill = ill || (rs2 != 5'd0);
          end
          5'b11000: begin
            u.fu = FU_F2I; u.lrd = rd; u.rd_class = int_rc(rd); u.rs2_class = RC_NONE;
            unique case (rs2)
              5'd0: u.op = F2I_CVTW;
              5'd1: u.op = F2I_CVTWU;
              5'd2: u.op = F2I_CVTL;
              5'd3: u.op = F2I_CVTLU;
              5'd8: begin u.op = F2I_CVTMOD; ill = ill || (i[26:25] != 2'b01) || (f3 != 3'b001); end   // Zfa fcvtmod.w.d (rm 固定 RTZ)
              default: ill = 1'b1;
            endcase
            ill = ill || !rm_ok;
          end
          5'b11010: begin
            u.fu = FU_I2F; u.rs1_class = (rs1 == 5'd0) ? RC_NONE : RC_INT; u.rs2_class = RC_NONE;
            unique case (rs2)
              5'd0: u.op = I2F_CVTW;
              5'd1: u.op = I2F_CVTWU;
              5'd2: u.op = I2F_CVTL;
              5'd3: u.op = I2F_CVTLU;
              default: ill = 1'b1;
            endcase
            ill = ill || !rm_ok;
          end
          5'b11110: begin
            if (rs2 == 5'd1) begin             // Zfa fli.{h,s,d}：rs1 字段是常数下标
              u.fu = FU_FMISC; u.op = FMISC_FLI; u.rs1_class = RC_NONE; u.rs2_class = RC_NONE;
              ill = ill || (f3 != 3'b000);
            end else begin
              u.fu = FU_I2F; u.op = (i[26:25] == 2'b10) ? I2F_MVH : I2F_MV;
              u.rs1_class = (rs1 == 5'd0) ? RC_NONE : RC_INT; u.rs2_class = RC_NONE;
              ill = ill || (f3 != 3'b000) || (rs2 != 5'd0);
            end
          end
          default: ill = 1'b1;
        endcase
      end
      default: ill = 1'b1;
    endcase

    // move 候选：64 位 ADD / ADDI 且另一操作数为 0 (x0 或立即数 0)，无 Zba 预处理；rd != x0
    // (rs 是 x0 时就是清零：映射到恒为 0 的 p0)
    if (u.fu == FU_ALU && u.op == ALU_ADD && !u.is_w && !u.auw && u.ash == 2'd0 && u.rd_class == RC_INT) begin
      if (u.use_imm && u.imm == 32'd0)                    begin u.mvc = 1'b1; u.mvsrc = 1'b0; end
      else if (!u.use_imm && u.rs2_class == RC_NONE)      begin u.mvc = 1'b1; u.mvsrc = 1'b0; end
      else if (!u.use_imm && u.rs1_class == RC_NONE)      begin u.mvc = 1'b1; u.mvsrc = 1'b1; end
    end

    // RVC 解压非法 / 取指异常优先
    if (fi_i.fetch_excp) begin
      u = '0;
      u.fu = FU_NONE; u.excp = 1'b1; u.cause = fi_i.fetch_cause; u.tval_hi = fi_i.fetch_hi;
      u.raw = fi_i.raw; u.is_rvc = fi_i.is_rvc;
      u.ftq_idx = fi_i.ftq_idx; u.ftq_off = fi_i.ftq_off; u.pred_taken = fi_i.pred_taken; u.ld_wait = fi_i.ld_wait;
      u.rd_class = RC_NONE; u.rs1_class = RC_NONE; u.rs2_class = RC_NONE;
    end else if (ill || fi_i.rvc_ill) begin
      u.fu = FU_NONE; u.excp = 1'b1; u.cause = (vill && !fi_i.rvc_ill) ? EXC_VIRT_INST : EXC_ILLEGAL_INST;
      u.rd_class = RC_NONE; u.rs1_class = RC_NONE; u.rs2_class = RC_NONE; u.rs3_en = 1'b0;
      u.is_br = 1'b0; u.wait_head = 1'b0; u.flush_after = 1'b0;
    end
    uop_o = u;
  end
endmodule
