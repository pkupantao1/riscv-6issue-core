//==============================================================================
// o6_uvm_pkg.sv - OOO6 处理器级 UVM 环境 (core 模式，tb_top 作为测试台骨架)
//
//   组成：
//     o6_monitor     时钟块读提交口 / 陷入 / 每拍微架构事件，分别广播 commit / trap / cycle 事务
//     o6_scoreboard  与 Spike 对拍之外的独立检查：程序流 (下一条 PC)、陷入 epc、写回值 X、提交槽连续；
//                    结束时汇总 tb 的对拍结果 (对拍本身仍在 tb_top 里逐条做，见 cosim_commit / cosim_trap)
//     o6_irq_agent   中断代理：sequencer + driver，向 TB 的中断设备 (0x0200_C000) 发 MEIP / SEIP 置位脉冲
//     o6_cov         功能覆盖率：指令 (助记符 / RVC / 寄存器 / 写回值 / RAW 距离 / 提交槽)、CSR、浮点舍入、
//                    向量配置 / 访存、陷入与中断、微架构事件、发射队列占用
//   测试：o6_base_test (任意 ELF，只监视) / o6_irq_test (加随机外部中断注入，配 verif/tests/irq_ext.S) /
//         o6_irq_burst_test (连发 / 短间隔注入，配 verif/tests/irq_csr.S)
//==============================================================================
`timescale 1ns/1ps
`include "uvm_macros.svh"

package o6_uvm_pkg;
  import uvm_pkg::*;
  import o6_pkg::W, o6_pkg::N_IQ;

  //--------------------------------------------------------------------------
  // 微架构事件编号 (o6_uvm_if.ev 的位)
  //--------------------------------------------------------------------------
  typedef enum int {
    EV_REDIRECT, EV_MIS_COND, EV_MIS_COND_TK, EV_MIS_RET, EV_MIS_IND, EV_MIS_JAL,
    EV_FL_REPLAY, EV_FL_TRAP, EV_FL_AFTER, EV_VIOL, EV_LD_FAIL,
    EV_FE_EMPTY, EV_FTQ_FULL, EV_IC_MISS, EV_F3_OVR, EV_F1_BTB,
    EV_REN_STALL, EV_RN_Q, EV_RN_TAG, EV_RN_IFL, EV_RN_FFL,
    EV_DS_STALL, EV_DS_ROB, EV_DS_LQ, EV_DS_SQ, EV_DS_IQ,
    EV_DC_LD_MISS, EV_DC_ST_MISS, EV_DC_NACK, EV_MSHR_FULL, EV_STFWD, EV_STFWD_WAIT,
    EV_PTW_I, EV_PTW_D, EV_PTW_BUSY, EV_DPF, EV_MMIO, EV_VU_BUSY, EV_CMT_FULL,
    EV_SQ_FULL, EV_LQ_FULL, EV_SQFULL_FLUSH,     // SQ / LQ 所有项有效；SQ 满时发生冲刷 (#26 类)
    EV_NUM
  } ev_e;

  //--------------------------------------------------------------------------
  // 指令分类 (32 位编码按助记符；RVC 另按象限 / funct3 覆盖)
  //--------------------------------------------------------------------------
  typedef enum int {
    MN_OTHER,
    MN_LUI, MN_AUIPC, MN_JAL, MN_JALR,
    MN_BEQ, MN_BNE, MN_BLT, MN_BGE, MN_BLTU, MN_BGEU,
    MN_LB, MN_LH, MN_LW, MN_LD, MN_LBU, MN_LHU, MN_LWU,
    MN_SB, MN_SH, MN_SW, MN_SD,
    MN_ADDI, MN_SLTI, MN_SLTIU, MN_XORI, MN_ORI, MN_ANDI, MN_SLLI, MN_SRLI, MN_SRAI,
    MN_ADD, MN_SUB, MN_SLL, MN_SLT, MN_SLTU, MN_XOR, MN_SRL, MN_SRA, MN_OR, MN_AND,
    MN_ADDIW, MN_SLLIW, MN_SRLIW, MN_SRAIW, MN_ADDW, MN_SUBW, MN_SLLW, MN_SRLW, MN_SRAW,
    MN_FENCE, MN_FENCE_I, MN_ECALL, MN_EBREAK, MN_MRET, MN_SRET, MN_WFI, MN_SFENCE_VMA,
    MN_CSRRW, MN_CSRRS, MN_CSRRC, MN_CSRRWI, MN_CSRRSI, MN_CSRRCI,
    MN_MUL, MN_MULH, MN_MULHSU, MN_MULHU, MN_DIV, MN_DIVU, MN_REM, MN_REMU,
    MN_MULW, MN_DIVW, MN_DIVUW, MN_REMW, MN_REMUW,
    // A：W 组与 D 组顺序相同 (D = W + AMO_D_OFF)
    MN_LR_W, MN_SC_W, MN_AMOSWAP_W, MN_AMOADD_W, MN_AMOXOR_W, MN_AMOAND_W, MN_AMOOR_W,
    MN_AMOMIN_W, MN_AMOMAX_W, MN_AMOMINU_W, MN_AMOMAXU_W,
    MN_LR_D, MN_SC_D, MN_AMOSWAP_D, MN_AMOADD_D, MN_AMOXOR_D, MN_AMOAND_D, MN_AMOOR_D,
    MN_AMOMIN_D, MN_AMOMAX_D, MN_AMOMINU_D, MN_AMOMAXU_D,
    MN_FLW, MN_FSW, MN_FLD, MN_FSD,
    // F / D：S 组与 D 组顺序相同 (D = S + FP_D_OFF)
    MN_FMADD_S, MN_FMSUB_S, MN_FNMSUB_S, MN_FNMADD_S, MN_FADD_S, MN_FSUB_S, MN_FMUL_S, MN_FDIV_S,
    MN_FSQRT_S, MN_FSGNJ_S, MN_FSGNJN_S, MN_FSGNJX_S, MN_FMIN_S, MN_FMAX_S,
    MN_FCVT_W_S, MN_FCVT_WU_S, MN_FCVT_L_S, MN_FCVT_LU_S, MN_FCVT_S_W, MN_FCVT_S_WU, MN_FCVT_S_L, MN_FCVT_S_LU,
    MN_FMV_X_W, MN_FMV_W_X, MN_FEQ_S, MN_FLT_S, MN_FLE_S, MN_FCLASS_S,
    MN_FMADD_D, MN_FMSUB_D, MN_FNMSUB_D, MN_FNMADD_D, MN_FADD_D, MN_FSUB_D, MN_FMUL_D, MN_FDIV_D,
    MN_FSQRT_D, MN_FSGNJ_D, MN_FSGNJN_D, MN_FSGNJX_D, MN_FMIN_D, MN_FMAX_D,
    MN_FCVT_W_D, MN_FCVT_WU_D, MN_FCVT_L_D, MN_FCVT_LU_D, MN_FCVT_D_W, MN_FCVT_D_WU, MN_FCVT_D_L, MN_FCVT_D_LU,
    MN_FMV_X_D, MN_FMV_D_X, MN_FEQ_D, MN_FLT_D, MN_FLE_D, MN_FCLASS_D,
    MN_FCVT_S_D, MN_FCVT_D_S,
    // B (Zba / Zbb / Zbs)
    MN_ADD_UW, MN_SH1ADD, MN_SH2ADD, MN_SH3ADD, MN_SH1ADD_UW, MN_SH2ADD_UW, MN_SH3ADD_UW, MN_SLLI_UW,
    MN_ANDN, MN_ORN, MN_XNOR, MN_CLZ, MN_CLZW, MN_CTZ, MN_CTZW, MN_CPOP, MN_CPOPW,
    MN_MAX, MN_MAXU, MN_MIN, MN_MINU, MN_SEXT_B, MN_SEXT_H, MN_ZEXT_H,
    MN_ROL, MN_ROLW, MN_ROR, MN_RORI, MN_RORIW, MN_RORW, MN_ORC_B, MN_REV8,
    MN_BCLR, MN_BCLRI, MN_BEXT, MN_BEXTI, MN_BINV, MN_BINVI, MN_BSET, MN_BSETI,
    // V (按格式类别)
    MN_VSETVLI, MN_VSETIVLI, MN_VSETVL, MN_VLOAD, MN_VSTORE,
    MN_V_IVV, MN_V_FVV, MN_V_MVV, MN_V_IVI, MN_V_IVX, MN_V_FVF, MN_V_MVX
  } mn_e;
  localparam int AMO_D_OFF = int'(MN_LR_D) - int'(MN_LR_W);
  localparam int FP_D_OFF  = int'(MN_FMADD_D) - int'(MN_FMADD_S);

  // 执行类别 (含 RVC)：用于提交槽交叉、"中断打在哪类指令之后"
  typedef enum int {
    C_ALU, C_BIT, C_MUL, C_DIV, C_LD, C_ST, C_AMO, C_BR, C_JMP, C_CSR, C_SYS, C_FENCE,
    C_FPLD, C_FPST, C_FPU, C_FDIV, C_FCVT, C_VCFG, C_VMEM, C_VARITH, C_OTHER
  } cls_e;

  function automatic mn_e mn_dec(bit [31:0] i);
    bit [4:0] op = i[6:2];
    bit [2:0] f3 = i[14:12];
    bit [6:0] f7 = i[31:25];
    bit [4:0] f5 = i[31:27];
    bit [4:0] r2 = i[24:20];
    bit       d  = i[25];                 // 浮点 fmt：0 = S，1 = D
    if (i[1:0] != 2'b11) return MN_OTHER;
    case (op)
      5'h0D: return MN_LUI;
      5'h05: return MN_AUIPC;
      5'h1B: return MN_JAL;
      5'h19: return MN_JALR;
      5'h18: case (f3) 0: return MN_BEQ; 1: return MN_BNE; 4: return MN_BLT; 5: return MN_BGE;
                       6: return MN_BLTU; 7: return MN_BGEU; default: return MN_OTHER; endcase
      5'h00: case (f3) 0: return MN_LB; 1: return MN_LH; 2: return MN_LW; 3: return MN_LD;
                       4: return MN_LBU; 5: return MN_LHU; 6: return MN_LWU; default: return MN_OTHER; endcase
      5'h08: case (f3) 0: return MN_SB; 1: return MN_SH; 2: return MN_SW; 3: return MN_SD;
                       default: return MN_OTHER; endcase
      5'h04: case (f3)
               0: return MN_ADDI; 2: return MN_SLTI; 3: return MN_SLTIU; 4: return MN_XORI;
               6: return MN_ORI;  7: return MN_ANDI;
               1: begin
                 if (i[31:26] == 6'h00) return MN_SLLI;
                 if (i[31:26] == 6'h0A) return MN_BSETI;
                 if (i[31:26] == 6'h12) return MN_BCLRI;
                 if (i[31:26] == 6'h1A) return MN_BINVI;
                 if (f7 == 7'h30) case (r2) 0: return MN_CLZ; 1: return MN_CTZ; 2: return MN_CPOP;
                                            4: return MN_SEXT_B; 5: return MN_SEXT_H; default: ; endcase
                 return MN_OTHER;
               end
               5: begin
                 if (i[31:26] == 6'h00) return MN_SRLI;
                 if (i[31:26] == 6'h10) return MN_SRAI;
                 if (i[31:26] == 6'h18) return MN_RORI;
                 if (i[31:26] == 6'h12) return MN_BEXTI;
                 if (i[31:20] == 12'h287) return MN_ORC_B;
                 if (i[31:20] == 12'h6B8) return MN_REV8;
                 return MN_OTHER;
               end
             endcase
      5'h06: case (f3)
               0: return MN_ADDIW;
               1: begin
                 if (f7 == 7'h00) return MN_SLLIW;
                 if (i[31:26] == 6'h02) return MN_SLLI_UW;
                 if (f7 == 7'h30) case (r2) 0: return MN_CLZW; 1: return MN_CTZW; 2: return MN_CPOPW; default: ; endcase
                 return MN_OTHER;
               end
               5: begin
                 if (f7 == 7'h00) return MN_SRLIW;
                 if (f7 == 7'h20) return MN_SRAIW;
                 if (f7 == 7'h30) return MN_RORIW;
                 return MN_OTHER;
               end
               default: return MN_OTHER;
             endcase
      5'h0C: case ({f7, f3})
               {7'h00, 3'd0}: return MN_ADD;  {7'h20, 3'd0}: return MN_SUB;  {7'h00, 3'd1}: return MN_SLL;
               {7'h00, 3'd2}: return MN_SLT;  {7'h00, 3'd3}: return MN_SLTU; {7'h00, 3'd4}: return MN_XOR;
               {7'h00, 3'd5}: return MN_SRL;  {7'h20, 3'd5}: return MN_SRA;  {7'h00, 3'd6}: return MN_OR;
               {7'h00, 3'd7}: return MN_AND;
               {7'h01, 3'd0}: return MN_MUL;  {7'h01, 3'd1}: return MN_MULH; {7'h01, 3'd2}: return MN_MULHSU;
               {7'h01, 3'd3}: return MN_MULHU; {7'h01, 3'd4}: return MN_DIV; {7'h01, 3'd5}: return MN_DIVU;
               {7'h01, 3'd6}: return MN_REM;  {7'h01, 3'd7}: return MN_REMU;
               {7'h10, 3'd2}: return MN_SH1ADD; {7'h10, 3'd4}: return MN_SH2ADD; {7'h10, 3'd6}: return MN_SH3ADD;
               {7'h20, 3'd7}: return MN_ANDN; {7'h20, 3'd6}: return MN_ORN;  {7'h20, 3'd4}: return MN_XNOR;
               {7'h05, 3'd4}: return MN_MIN;  {7'h05, 3'd5}: return MN_MINU; {7'h05, 3'd6}: return MN_MAX;
               {7'h05, 3'd7}: return MN_MAXU;
               {7'h30, 3'd1}: return MN_ROL;  {7'h30, 3'd5}: return MN_ROR;
               {7'h14, 3'd1}: return MN_BSET; {7'h24, 3'd1}: return MN_BCLR; {7'h34, 3'd1}: return MN_BINV;
               {7'h24, 3'd5}: return MN_BEXT;
               default: return MN_OTHER;
             endcase
      5'h0E: case ({f7, f3})
               {7'h00, 3'd0}: return MN_ADDW; {7'h20, 3'd0}: return MN_SUBW; {7'h00, 3'd1}: return MN_SLLW;
               {7'h00, 3'd5}: return MN_SRLW; {7'h20, 3'd5}: return MN_SRAW;
               {7'h01, 3'd0}: return MN_MULW; {7'h01, 3'd4}: return MN_DIVW; {7'h01, 3'd5}: return MN_DIVUW;
               {7'h01, 3'd6}: return MN_REMW; {7'h01, 3'd7}: return MN_REMUW;
               {7'h04, 3'd0}: return MN_ADD_UW;
               {7'h10, 3'd2}: return MN_SH1ADD_UW; {7'h10, 3'd4}: return MN_SH2ADD_UW; {7'h10, 3'd6}: return MN_SH3ADD_UW;
               {7'h30, 3'd1}: return MN_ROLW; {7'h30, 3'd5}: return MN_RORW;
               {7'h04, 3'd4}: return (r2 == 0) ? MN_ZEXT_H : MN_OTHER;
               default: return MN_OTHER;
             endcase
      5'h03: return (f3 == 0) ? MN_FENCE : (f3 == 1) ? MN_FENCE_I : MN_OTHER;
      5'h1C: begin
        if (f3 == 0) begin
          if (i == 32'h0000_0073) return MN_ECALL;
          if (i == 32'h0010_0073) return MN_EBREAK;
          if (i == 32'h3020_0073) return MN_MRET;
          if (i == 32'h1020_0073) return MN_SRET;
          if (i == 32'h1050_0073) return MN_WFI;
          if (f7 == 7'h09)        return MN_SFENCE_VMA;
          return MN_OTHER;
        end
        case (f3) 1: return MN_CSRRW; 2: return MN_CSRRS; 3: return MN_CSRRC;
                  5: return MN_CSRRWI; 6: return MN_CSRRSI; 7: return MN_CSRRCI; default: return MN_OTHER; endcase
      end
      5'h0B: begin
        mn_e w;
        case (f5)
          5'h02: w = MN_LR_W;     5'h03: w = MN_SC_W;     5'h01: w = MN_AMOSWAP_W; 5'h00: w = MN_AMOADD_W;
          5'h04: w = MN_AMOXOR_W; 5'h0C: w = MN_AMOAND_W; 5'h08: w = MN_AMOOR_W;   5'h10: w = MN_AMOMIN_W;
          5'h14: w = MN_AMOMAX_W; 5'h18: w = MN_AMOMINU_W; 5'h1C: w = MN_AMOMAXU_W;
          default: return MN_OTHER;
        endcase
        if (f3 == 2) return w;
        if (f3 == 3) return mn_e'(int'(w) + AMO_D_OFF);
        return MN_OTHER;
      end
      5'h01: return (f3 == 2) ? MN_FLW : (f3 == 3) ? MN_FLD : MN_VLOAD;
      5'h09: return (f3 == 2) ? MN_FSW : (f3 == 3) ? MN_FSD : MN_VSTORE;
      5'h10, 5'h11, 5'h12, 5'h13: begin
        if (i[26]) return MN_OTHER;
        return mn_e'(int'(MN_FMADD_S) + int'(op - 5'h10) + (d ? FP_D_OFF : 0));
      end
      5'h14: begin
        mn_e s;
        if (i[26]) return MN_OTHER;       // 只有 S / D (标量 H / Q 不在 ISA 里)
        case (f5)
          5'h00: s = MN_FADD_S; 5'h01: s = MN_FSUB_S; 5'h02: s = MN_FMUL_S; 5'h03: s = MN_FDIV_S;
          5'h0B: s = MN_FSQRT_S;
          5'h04: case (f3) 0: s = MN_FSGNJ_S; 1: s = MN_FSGNJN_S; 2: s = MN_FSGNJX_S; default: return MN_OTHER; endcase
          5'h05: case (f3) 0: s = MN_FMIN_S;  1: s = MN_FMAX_S; default: return MN_OTHER; endcase
          5'h08: return (!d && r2 == 1) ? MN_FCVT_S_D : (d && r2 == 0) ? MN_FCVT_D_S : MN_OTHER;
          5'h14: case (f3) 2: s = MN_FEQ_S; 1: s = MN_FLT_S; 0: s = MN_FLE_S; default: return MN_OTHER; endcase
          5'h18: case (r2) 0: s = MN_FCVT_W_S; 1: s = MN_FCVT_WU_S; 2: s = MN_FCVT_L_S; 3: s = MN_FCVT_LU_S;
                           default: return MN_OTHER; endcase
          5'h1A: case (r2) 0: s = MN_FCVT_S_W; 1: s = MN_FCVT_S_WU; 2: s = MN_FCVT_S_L; 3: s = MN_FCVT_S_LU;
                           default: return MN_OTHER; endcase
          5'h1C: case (f3) 0: s = MN_FMV_X_W; 1: s = MN_FCLASS_S; default: return MN_OTHER; endcase
          5'h1E: s = MN_FMV_W_X;
          default: return MN_OTHER;
        endcase
        return mn_e'(int'(s) + (d ? FP_D_OFF : 0));
      end
      5'h15: case (f3)
               7: begin
                 if (!i[31])               return MN_VSETVLI;
                 if (i[31:30] == 2'b11)    return MN_VSETIVLI;
                 if (f7 == 7'h40)          return MN_VSETVL;
                 return MN_OTHER;
               end
               0: return MN_V_IVV; 1: return MN_V_FVV; 2: return MN_V_MVV; 3: return MN_V_IVI;
               4: return MN_V_IVX; 5: return MN_V_FVF; 6: return MN_V_MVX;
             endcase
      default: return MN_OTHER;
    endcase
    return MN_OTHER;
  endfunction

  function automatic cls_e mn_cls(mn_e m);
    if (m inside {[MN_LUI:MN_AUIPC], [MN_ADDI:MN_SRAW]})         return C_ALU;
    if (m inside {MN_JAL, MN_JALR})                               return C_JMP;
    if (m inside {[MN_BEQ:MN_BGEU]})                              return C_BR;
    if (m inside {[MN_LB:MN_LWU]})                                return C_LD;
    if (m inside {[MN_SB:MN_SD]})                                 return C_ST;
    if (m inside {MN_FENCE, MN_FENCE_I, MN_SFENCE_VMA})           return C_FENCE;
    if (m inside {MN_ECALL, MN_EBREAK, MN_MRET, MN_SRET, MN_WFI}) return C_SYS;
    if (m inside {[MN_CSRRW:MN_CSRRCI]})                          return C_CSR;
    if (m inside {[MN_MUL:MN_MULHU], MN_MULW})                    return C_MUL;
    if (m inside {[MN_DIV:MN_REMU], [MN_DIVW:MN_REMUW]})          return C_DIV;
    if (m inside {[MN_LR_W:MN_AMOMAXU_D]})                        return C_AMO;
    if (m inside {MN_FLW, MN_FLD})                                return C_FPLD;
    if (m inside {MN_FSW, MN_FSD})                                return C_FPST;
    if (m inside {MN_FDIV_S, MN_FSQRT_S, MN_FDIV_D, MN_FSQRT_D})  return C_FDIV;
    if (m inside {[MN_FCVT_W_S:MN_FMV_W_X], [MN_FCVT_W_D:MN_FMV_D_X], MN_FCVT_S_D, MN_FCVT_D_S}) return C_FCVT;
    if (m inside {[MN_FMADD_S:MN_FCLASS_D]})                      return C_FPU;
    if (m inside {[MN_ADD_UW:MN_BSETI]})                          return C_BIT;
    if (m inside {[MN_VSETVLI:MN_VSETVL]})                        return C_VCFG;
    if (m inside {MN_VLOAD, MN_VSTORE})                           return C_VMEM;
    if (m inside {[MN_V_IVV:MN_V_MVX]})                           return C_VARITH;
    return C_OTHER;
  endfunction

  // RVC：cq = {象限, funct3}
  function automatic cls_e rvc_cls(bit [15:0] i);
    bit [4:0] cq = {i[1:0], i[15:13]};
    case (cq)
      5'o00, 5'o10, 5'o11, 5'o12, 5'o13, 5'o14, 5'o20: return C_ALU;
      5'o01, 5'o21: return C_FPLD;
      5'o05, 5'o25: return C_FPST;
      5'o02, 5'o03, 5'o22, 5'o23: return C_LD;
      5'o06, 5'o07, 5'o26, 5'o27: return C_ST;
      5'o15: return C_JMP;
      5'o16, 5'o17: return C_BR;
      5'o24: begin
        if (i[6:2] != 0)  return C_ALU;            // c.mv / c.add
        if (i[11:7] == 0) return C_SYS;            // c.ebreak
        return C_JMP;                              // c.jr / c.jalr
      end
      default: return C_OTHER;
    endcase
  endfunction

  //--------------------------------------------------------------------------
  // 事务
  //--------------------------------------------------------------------------
  class o6_commit_item extends uvm_sequence_item;
    longint      pc, wd, cyc;
    bit [31:0]   insn;
    int          rdt, rd, slot, ncmt;
    bit          is_vec;
    bit [1:0]    priv;
    bit          in_dbg;                  // 提交时处于调试模式
    `uvm_object_utils(o6_commit_item)
    function new(string name = "o6_commit_item"); super.new(name); endfunction
    function bit rvc(); return insn[1:0] != 2'b11; endfunction
    function string convert2string();
      return $sformatf("cyc=%0d slot=%0d pc=%h insn=%h rdt=%0d rd=%0d wd=%h", cyc, slot, pc, insn, rdt, rd, wd);
    endfunction
  endclass

  class o6_trap_item extends uvm_sequence_item;
    bit          irq;
    int          cause;
    longint      epc, tval, cyc;
    bit [1:0]    priv;                    // 陷入前的特权级
    `uvm_object_utils(o6_trap_item)
    function new(string name = "o6_trap_item"); super.new(name); endfunction
  endclass

  class o6_cycle_item extends uvm_sequence_item;
    bit [63:0]   ev;
    bit [7:0]    chk;
    bit          dbg_enter;               // 本拍进入调试模式
    int          ncmt;
    int          iq_lvl [N_IQ];           // 0 空 1 <1/4 2 <1/2 3 <满 4 满
    longint      cyc;
    `uvm_object_utils(o6_cycle_item)
    function new(string name = "o6_cycle_item"); super.new(name); endfunction
  endclass

  // 中断注入：kind 1 = MEIP，2 = SEIP，3 = 同时；delay = 距上一次注入的拍数
  class o6_irq_item extends uvm_sequence_item;
    rand bit [1:0]    kind;
    rand int unsigned delay;
    constraint c_kind  { kind != 0; }
    // 注入策略：连发 (1~3 拍，压「上一次 xRET 未完 / 清中断的 sw+lw 窗口」) 15%、短间隔 25%、
    //   中等间隔 (中断均匀落在各类指令上) 45%、长间隔 (中断延迟大档、长时间无中断) 15%；平均约 600 拍一次。
    //   同时置位 10%：S 态下先进 M 处理 MEI，SEI 留到回 S 后再进，覆盖嵌套路径
    constraint c_policy {
      delay dist { [1:3] :/ 15, [4:50] :/ 25, [51:1000] :/ 45, [1001:4000] :/ 15 };
      kind  dist { 2'd1 := 45, 2'd2 := 45, 2'd3 := 10 };
    }
    `uvm_object_utils(o6_irq_item)
    function new(string name = "o6_irq_item"); super.new(name); endfunction
    function string convert2string(); return $sformatf("kind=%0d delay=%0d", kind, delay); endfunction
  endclass

  `uvm_analysis_imp_decl(_cm)
  `uvm_analysis_imp_decl(_trap)
  `uvm_analysis_imp_decl(_cyc)
  `uvm_analysis_imp_decl(_irq)
  `uvm_analysis_imp_decl(_clint)

  //--------------------------------------------------------------------------
  // 监视器
  //--------------------------------------------------------------------------
  class o6_monitor extends uvm_monitor;
    `uvm_component_utils(o6_monitor)
    virtual o6_uvm_if vif;
    uvm_analysis_port #(o6_commit_item) cm_ap;
    uvm_analysis_port #(o6_trap_item)   trap_ap;
    uvm_analysis_port #(o6_cycle_item)  cyc_ap;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      if (!uvm_config_db#(virtual o6_uvm_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "没有 vif")
      cm_ap = new("cm_ap", this); trap_ap = new("trap_ap", this); cyc_ap = new("cyc_ap", this);
    endfunction
    task run_phase(uvm_phase phase);
      forever begin
        @(vif.cb);
        if (vif.cb.rst_n !== 1'b1 || vif.cb.tb_done === 1'b1) continue;
        begin
          int n = $countones(vif.cb.cm_v);
          o6_cycle_item ci;
          for (int c = 0; c < W; c++) if (vif.cb.cm_v[c]) begin
            o6_commit_item t = o6_commit_item::type_id::create("cm");
            t.pc = vif.cb.cm_pc[c]; t.insn = vif.cb.cm_insn[c]; t.rdt = vif.cb.cm_rdt[c]; t.rd = vif.cb.cm_rd[c];
            t.wd = vif.cb.cm_wd[c]; t.is_vec = vif.cb.cm_vec[c]; t.slot = c; t.ncmt = n; t.cyc = vif.cb.cyc;
            t.priv = vif.cb.priv; t.in_dbg = vif.cb.dbg_mode;
            cm_ap.write(t);
          end
          if (vif.cb.trap_v) begin
            o6_trap_item t = o6_trap_item::type_id::create("trap");
            t.irq = vif.cb.trap_irq; t.cause = int'(vif.cb.trap_cause); t.epc = vif.cb.trap_epc;
            t.tval = vif.cb.trap_tval; t.priv = vif.cb.priv; t.cyc = vif.cb.cyc;
            trap_ap.write(t);
          end
          ci = o6_cycle_item::type_id::create("cyc");
          ci.ev = vif.cb.ev; ci.chk = vif.cb.chk; ci.dbg_enter = vif.cb.dbg_enter; ci.ncmt = n; ci.cyc = vif.cb.cyc;
          for (int q = 0; q < N_IQ; q++) begin
            int o = vif.cb.iq_occ[q], s = vif.cb.iq_size[q];
            ci.iq_lvl[q] = (o == 0) ? 0 : (o == s) ? 4 : (4 * o < s) ? 1 : (2 * o < s) ? 2 : 3;
          end
          cyc_ap.write(ci);
        end
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // 计分板：与 Spike 对拍互补的独立检查
  //   1) 程序流：每条提交的 PC 必须等于上一条推出的下一 PC (顺序指令 = pc + 长度；JAL / 条件分支
  //      / C.J / C.BEQZ / C.BNEZ 由编码算目标；JALR / xRET / 陷入后不检查一条)
  //   2) 陷入 epc：异常 = 下一条该提交的指令；中断 = 被打断处 (同样是下一条)
  //   3) 写回整数 / 浮点寄存器的值不能有 X；4) 一拍里有效的提交槽必须连续
//   5) MMIO load 开始访问设备时比它老的 store 都已排空 (Spike 的 MMIO 读值由 DUT 注入，对拍查不出访存顺序)
//   6) 后端不出现 JAL 误预测 (前端 F3 应已改正)
  //--------------------------------------------------------------------------
  class o6_scoreboard extends uvm_component;
    `uvm_component_utils(o6_scoreboard)
    uvm_analysis_imp_cm   #(o6_commit_item, o6_scoreboard) cm_imp;
    uvm_analysis_imp_trap #(o6_trap_item,   o6_scoreboard) trap_imp;
    uvm_analysis_imp_cyc  #(o6_cycle_item,  o6_scoreboard) cyc_imp;
    virtual o6_uvm_if vif;
    bit     exp_v;
    longint exp_a, exp_b;
    longint n_cm, n_trap, n_irq, n_chk_pc, n_chk_epc, n_err;
    longint last_cyc = -1;
    int     last_slot;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      cm_imp = new("cm_imp", this); trap_imp = new("trap_imp", this); cyc_imp = new("cyc_imp", this);
      if (!uvm_config_db#(virtual o6_uvm_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "没有 vif")
    endfunction
    function void err(string id, string msg);
      n_err++;
      if (n_err <= 20) `uvm_error(id, msg)
    endfunction

    function void write_cm(o6_commit_item t);
      longint len = t.rvc() ? 2 : 4;
      bit [31:0] i = t.insn;
      n_cm++;
      // 提交槽连续：同一拍里槽号逐个递增
      if (t.cyc == last_cyc && t.slot != last_slot + 1)
        err("SLOT", $sformatf("同拍提交槽不连续：上一个 %0d，本条 %0d (%s)", last_slot, t.slot, t.convert2string()));
      last_cyc = t.cyc; last_slot = t.slot;
      if (t.rdt != 0 && $isunknown(t.wd)) err("XWB", $sformatf("写回值含 X：%s", t.convert2string()));
      // 调试模式里的程序流 (调试 ROM / progbuf，由调试模块生成) 与 dret 的返回地址 (dpc) 不检查
      if (t.in_dbg || i == 32'h7b20_0073) begin exp_v = 0; return; end
      if (exp_v) begin
        n_chk_pc++;
        if (t.pc != exp_a && t.pc != exp_b)
          err("PCFLOW", $sformatf("程序流断开：应为 %h / %h，实际 %s", exp_a, exp_b, t.convert2string()));
      end
      // 推出下一条
      exp_v = 1; exp_a = t.pc + len; exp_b = exp_a;
      if (!t.rvc()) begin
        mn_e m = mn_dec(i);
        case (m)
          MN_JAL: begin
            longint imm = {{44{i[31]}}, i[19:12], i[20], i[30:21], 1'b0};
            exp_a = t.pc + imm; exp_b = exp_a;
          end
          MN_BEQ, MN_BNE, MN_BLT, MN_BGE, MN_BLTU, MN_BGEU:
            exp_b = t.pc + {{52{i[31]}}, i[7], i[30:25], i[11:8], 1'b0};
          MN_JALR, MN_MRET, MN_SRET, MN_ECALL, MN_EBREAK: exp_v = 0;
          default: ;
        endcase
      end else begin
        bit [4:0] cq = {i[1:0], i[15:13]};
        if (cq == 5'o15) begin                 // c.j
          longint imm = {{53{i[12]}}, i[8], i[10:9], i[6], i[7], i[2], i[11], i[5:3], 1'b0};
          exp_a = t.pc + imm; exp_b = exp_a;
        end else if (cq == 5'o16 || cq == 5'o17)   // c.beqz / c.bnez
          exp_b = t.pc + {{56{i[12]}}, i[6:5], i[2], i[11:10], i[4:3], 1'b0};
        else if (cq == 5'o24 && i[6:2] == 0) exp_v = 0;   // c.jr / c.jalr / c.ebreak
      end
    endfunction

    function void write_trap(o6_trap_item t);
      n_trap++;
      if (t.irq) n_irq++;
      if (exp_v) begin
        n_chk_epc++;
        if (t.epc != exp_a && t.epc != exp_b)
          err("EPC", $sformatf("陷入 epc 不在程序流上：应为 %h / %h，实际 epc=%h irq=%0d cause=%0d", exp_a, exp_b,
                               t.epc, t.irq, t.cause));
      end
      exp_v = 0;                               // 陷入入口 (xtvec) 不检查
    endfunction

    function void write_cyc(o6_cycle_item t);
      if (t.dbg_enter) exp_v = 0;              // 进调试模式：跳调试 ROM，不经陷入
      // WV-02：JAL 目标由前端 F3 预译码改正，后端不应出现 JAL 误预测 (出现即前端漏改正，属设计意图违例)
      if (t.ev[EV_MIS_JAL]) err("JALMIS", $sformatf("cyc=%0d 后端出现 JAL 误预测", t.cyc));
      if (t.chk[0]) err("MMIO_ORD", $sformatf("cyc=%0d MMIO load 开始访问设备时，比它老的 store 还在 SQ 里没排空", t.cyc));
    endfunction

    function void check_phase(uvm_phase phase);
      if (vif.tb_done !== 1'b1)      `uvm_error("TB", "测试没有结束 (tb 未给出 PASS / FAIL)")
      else if (vif.tb_pass !== 1'b1) `uvm_error("TB", $sformatf("tb 判 FAIL (对拍错误 %0d)", vif.tb_err))
      if (n_cm != vif.n_commit)
        `uvm_error("CNT", $sformatf("UVM 看到 %0d 条提交，tb (与 Spike 对拍) 计 %0d 条", n_cm, vif.n_commit))
    endfunction
    function void report_phase(uvm_phase phase);
      `uvm_info("SB", $sformatf("提交 %0d 条 (程序流检查 %0d 次)；陷入 %0d 次 (中断 %0d，epc 检查 %0d 次)；独立检查错误 %0d",
                                n_cm, n_chk_pc, n_trap, n_irq, n_chk_epc, n_err), UVM_NONE)
    endfunction
  endclass

  //--------------------------------------------------------------------------
  // 中断代理
  //--------------------------------------------------------------------------
  typedef uvm_sequencer #(o6_irq_item) o6_irq_sequencer;

  class o6_irq_driver extends uvm_driver #(o6_irq_item);
    `uvm_component_utils(o6_irq_driver)
    virtual o6_uvm_if vif;
    uvm_analysis_port #(o6_irq_item) ap;      // 实际发出的注入 (给覆盖率)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      if (!uvm_config_db#(virtual o6_uvm_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "没有 vif")
      ap = new("ap", this);
    endfunction
    task run_phase(uvm_phase phase);
      vif.cb.meip_set <= 1'b0; vif.cb.seip_set <= 1'b0;
      wait (vif.rst_n === 1'b1);
      forever begin
        seq_item_port.get_next_item(req);
        repeat (req.delay) @(vif.cb);
        if (vif.tb_done !== 1'b1) begin
          vif.cb.meip_set <= req.kind[0]; vif.cb.seip_set <= req.kind[1];
          @(vif.cb);
          vif.cb.meip_set <= 1'b0; vif.cb.seip_set <= 1'b0;
          ap.write(req);
        end
        seq_item_port.item_done();
      end
    endtask
  endclass

  class o6_irq_agent extends uvm_agent;
    `uvm_component_utils(o6_irq_agent)
    o6_irq_sequencer sqr;
    o6_irq_driver    drv;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      sqr = o6_irq_sequencer::type_id::create("sqr", this);
      drv = o6_irq_driver::type_id::create("drv", this);
    endfunction
    function void connect_phase(uvm_phase phase); drv.seq_item_port.connect(sqr.seq_item_export); endfunction
  endclass

  // 随机外部中断序列：n 次注入
  class o6_irq_seq extends uvm_sequence #(o6_irq_item);
    `uvm_object_utils(o6_irq_seq)
    int unsigned n = 200;
    function new(string name = "o6_irq_seq"); super.new(name); endfunction
    task body();
      for (int k = 0; k < n; k++) begin
        req = o6_irq_item::type_id::create("req");
        start_item(req);
        if (!req.randomize()) `uvm_fatal("RAND", "o6_irq_item 随机化失败")
        finish_item(req);
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // CLINT 代理：kind 0 = 定时器 (mtimecmp = mtime + tdelta，tdelta 个 mtime 周期 = 8·tdelta 拍后到期)，1 = 软件中断 (msip)
  //--------------------------------------------------------------------------
  class o6_clint_item extends uvm_sequence_item;
    rand bit          kind;
    rand int unsigned delay;
    rand bit [15:0]   tdelta;
    constraint c_policy {
      kind   dist { 1'b0 := 60, 1'b1 := 40 };
      delay  dist { [1:3] :/ 10, [4:100] :/ 30, [101:1500] :/ 45, [1501:5000] :/ 15 };
      tdelta dist { [0:2] :/ 20, [3:50] :/ 50, [51:300] :/ 30 };
    }
    `uvm_object_utils(o6_clint_item)
    function new(string name = "o6_clint_item"); super.new(name); endfunction
  endclass

  typedef uvm_sequencer #(o6_clint_item) o6_clint_sequencer;

  class o6_clint_driver extends uvm_driver #(o6_clint_item);
    `uvm_component_utils(o6_clint_driver)
    virtual o6_uvm_if vif;
    uvm_analysis_port #(o6_clint_item) ap;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      if (!uvm_config_db#(virtual o6_uvm_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "没有 vif")
      ap = new("ap", this);
    endfunction
    task run_phase(uvm_phase phase);
      vif.cb.msip_set <= 1'b0; vif.cb.tmr_set <= 1'b0; vif.cb.tmr_delta <= '0;
      wait (vif.rst_n === 1'b1);
      forever begin
        seq_item_port.get_next_item(req);
        repeat (req.delay) @(vif.cb);
        if (vif.tb_done !== 1'b1) begin
          if (req.kind) vif.cb.msip_set <= 1'b1;
          else begin vif.cb.tmr_set <= 1'b1; vif.cb.tmr_delta <= req.tdelta; end
          @(vif.cb);
          vif.cb.msip_set <= 1'b0; vif.cb.tmr_set <= 1'b0;
          ap.write(req);
        end
        seq_item_port.item_done();
      end
    endtask
  endclass

  class o6_clint_agent extends uvm_agent;
    `uvm_component_utils(o6_clint_agent)
    o6_clint_sequencer sqr;
    o6_clint_driver    drv;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      sqr = o6_clint_sequencer::type_id::create("sqr", this);
      drv = o6_clint_driver::type_id::create("drv", this);
    endfunction
    function void connect_phase(uvm_phase phase); drv.seq_item_port.connect(sqr.seq_item_export); endfunction
  endclass

  class o6_clint_seq extends uvm_sequence #(o6_clint_item);
    `uvm_object_utils(o6_clint_seq)
    int unsigned n = 300;
    function new(string name = "o6_clint_seq"); super.new(name); endfunction
    task body();
      for (int k = 0; k < n; k++) begin
        req = o6_clint_item::type_id::create("req");
        start_item(req);
        if (!req.randomize()) `uvm_fatal("RAND", "o6_clint_item 随机化失败")
        finish_item(req);
      end
    endtask
  endclass

  //--------------------------------------------------------------------------
  // 功能覆盖率
  //--------------------------------------------------------------------------
  class o6_cov extends uvm_component;
    `uvm_component_utils(o6_cov)
    uvm_analysis_imp_cm   #(o6_commit_item, o6_cov) cm_imp;
    uvm_analysis_imp_trap #(o6_trap_item,   o6_cov) trap_imp;
    uvm_analysis_imp_cyc  #(o6_cycle_item,  o6_cov) cyc_imp;
    uvm_analysis_imp_irq  #(o6_irq_item,    o6_cov) irq_imp;
    uvm_analysis_imp_clint #(o6_clint_item, o6_cov) clint_imp;

    // 本条指令的字段 (covergroup 直接引用)
    mn_e       mn;
    cls_e      cls, last_cls;
    bit        rvc;
    bit [4:0]  cq, rd, rs1, rs2;
    bit [2:0]  rm;
    bit [11:0] csr;
    int        rdt, slot, raw_d;
    bit [63:0] wd;
    bit [2:0]  vsew, vlmul;
    bit [2:0]  vmop_w;                     // 向量访存：宽度编码 (insn[14:12])
    bit [1:0]  vmop;                       // mop (insn[27:26])
    // 陷入
    bit        t_irq;
    int        t_cause;
    bit [1:0]  t_priv;
    int        irq_lat;
    // 每拍
    o6_cycle_item cy;
    // 中断注入
    bit [1:0]    i_kind;
    int unsigned i_delay;
    longint      irq_set_cyc [4];          // [0] MEI [1] SEI [2] MTI (预计到期拍) [3] MSI 最近一次置位的拍
    bit          c_kind;
    int unsigned c_delay;
    int          c_tdelta;
    longint      cur_cyc;

    longint n_idx = 0;
    longint lastw [32];

    covergroup cg_insn;
      option.per_instance = 1;
      // WV-01：ECALL / EBREAK 不提交 (由 cg_trap 的 cause 8/9/11/3 覆盖)
      cp_mn: coverpoint mn iff (!rvc) { ignore_bins oth = {MN_OTHER, MN_ECALL, MN_EBREAK}; }
      cp_rvc: coverpoint cq iff (rvc) {
        bins q0[] = {5'o00, 5'o01, 5'o02, 5'o03, 5'o05, 5'o06, 5'o07};
        bins q1[] = {[5'o10:5'o17]};
        bins q2[] = {[5'o20:5'o27]};
      }
      cp_cls: coverpoint cls { ignore_bins oth = {C_OTHER}; }
      cp_slot: coverpoint slot { bins s[] = {[0:W-1]}; }
      cx_cls_slot: cross cp_cls, cp_slot { ignore_bins oth = binsof(cp_cls) intersect {C_OTHER}; }   // WV-04
      cp_rd: coverpoint rd iff (rdt == 1) { bins r[] = {[1:31]}; }
      cp_frd: coverpoint rd iff (rdt == 2) { bins f[] = {[0:31]}; }
      cp_rs1: coverpoint rs1 iff (!rvc && raw_d >= 0) { bins r[] = {[0:31]}; }
      cp_rs2: coverpoint rs2 iff (!rvc && cls inside {C_ALU, C_BIT, C_MUL, C_DIV, C_ST, C_BR, C_AMO} &&
                                   mn inside {[MN_BEQ:MN_BGEU], [MN_SB:MN_SD], [MN_ADD:MN_AND], [MN_ADDW:MN_SRAW],
                                              [MN_MUL:MN_REMUW], [MN_SC_W:MN_AMOMAXU_D]}) { bins r[] = {[0:31]}; }
      cp_raw: coverpoint raw_d iff (raw_d > 0) {
        bins d1 = {1}; bins d2 = {2}; bins d3 = {3}; bins d4_6 = {[4:6]}; bins d7_16 = {[7:16]};
        bins d17_64 = {[17:64]}; bins far = {[65:$]};
      }
      cx_raw_cls: cross cp_raw, cp_cls {
        ignore_bins nsrc = binsof(cp_cls) intersect {C_SYS, C_FENCE, C_JMP, C_VCFG, C_VARITH, C_FPU, C_FDIV, C_FCVT, C_OTHER};
      }
      cp_wd: coverpoint wd iff (rdt == 1) {
        bins zero = {0}; bins one = {1}; bins neg1 = {64'hFFFF_FFFF_FFFF_FFFF};
        bins smin = {64'h8000_0000_0000_0000}; bins smax = {64'h7FFF_FFFF_FFFF_FFFF};
        bins small_ = {[2:255]}; bins mid = {[256:64'h7FFF_FFFF]}; bins big = {[64'h8000_0000:64'h7FFF_FFFF_FFFF_FFFE]};
        bins neg32 = {[64'hFFFF_FFFF_8000_0000:64'hFFFF_FFFF_FFFF_FFFE]};
        bins neg = {[64'h8000_0000_0000_0001:64'hFFFF_FFFF_7FFF_FFFF]};
      }
    endgroup

    covergroup cg_csr;
      option.per_instance = 1;
      cp_op: coverpoint mn { bins op[] = {[MN_CSRRW:MN_CSRRCI]}; }
      cp_csr: coverpoint csr {
        bins fflags = {12'h001}; bins frm = {12'h002}; bins fcsr = {12'h003};
        bins vstart = {12'h008}; bins vxsat = {12'h009}; bins vxrm = {12'h00A}; bins vcsr = {12'h00F};
        bins sstatus = {12'h100}; bins sie = {12'h104}; bins stvec = {12'h105}; bins scounteren = {12'h106};
        bins senvcfg = {12'h10A}; bins sscratch = {12'h140}; bins sepc = {12'h141}; bins scause = {12'h142};
        bins stval = {12'h143}; bins sip = {12'h144}; bins satp = {12'h180};
        bins mstatus = {12'h300}; bins misa = {12'h301}; bins medeleg = {12'h302}; bins mideleg = {12'h303};
        bins mie = {12'h304}; bins mtvec = {12'h305}; bins mcounteren = {12'h306}; bins menvcfg = {12'h30A};
        bins mcountinhibit = {12'h320}; bins mhpmevent = {[12'h323:12'h33F]};
        bins mscratch = {12'h340}; bins mepc = {12'h341}; bins mcause = {12'h342}; bins mtval = {12'h343};
        bins mip = {12'h344}; bins pmpcfg = {[12'h3A0:12'h3AF]}; bins pmpaddr = {[12'h3B0:12'h3EF]};
        bins mcycle = {12'hB00}; bins minstret = {12'hB02}; bins mhpmcounter = {[12'hB03:12'hB1F]};
        bins cycle = {12'hC00}; bins time_ = {12'hC01}; bins instret = {12'hC02}; bins hpmcounter = {[12'hC03:12'hC1F]};
        bins vl = {12'hC20}; bins vtype = {12'hC21}; bins vlenb = {12'hC22};
        bins mvendorid = {12'hF11}; bins marchid = {12'hF12}; bins mimpid = {12'hF13}; bins mhartid = {12'hF14};
        bins other = default;
      }
      // 按组的读写方式
      cp_grp: coverpoint csr {
        bins fp = {[12'h001:12'h003]}; bins vec = {[12'h008:12'h00F], [12'hC20:12'hC22]};
        bins s_trap = {[12'h100:12'h1FF]}; bins m_trap = {[12'h300:12'h30F], [12'h340:12'h34F]};
        bins m_cnt = {[12'h320:12'h33F], [12'hB00:12'hB1F]}; bins u_cnt = {[12'hC00:12'hC1F]};
        bins pmp = {[12'h3A0:12'h3EF]}; bins id = {[12'hF11:12'hF15]};
      }
      cx_grp_op: cross cp_grp, cp_op {
        // 只读 CSR 组 (用户计数器 / 机器 ID) 不能写：只要 csrrs / csrrc x0 形式的读
        ignore_bins ro = binsof(cp_grp) intersect {[12'hC00:12'hC1F], [12'hF11:12'hF15]} &&
                         binsof(cp_op) intersect {MN_CSRRW, MN_CSRRWI, MN_CSRRSI, MN_CSRRCI};
      }
    endgroup

    covergroup cg_fp;
      option.per_instance = 1;
      cp_rm: coverpoint rm { bins rne = {0}; bins rtz = {1}; bins rdn = {2}; bins rup = {3}; bins rmm = {4}; bins dyn = {7}; }
      cp_op: coverpoint mn {
        bins s_arith[] = {MN_FADD_S, MN_FSUB_S, MN_FMUL_S, MN_FDIV_S, MN_FSQRT_S, MN_FMADD_S};
        bins d_arith[] = {MN_FADD_D, MN_FSUB_D, MN_FMUL_D, MN_FDIV_D, MN_FSQRT_D, MN_FMADD_D};
        bins cvt_i[]   = {MN_FCVT_W_S, MN_FCVT_L_D, MN_FCVT_S_W, MN_FCVT_D_L};
      }
      cx_op_rm: cross cp_op, cp_rm;
    endgroup

    covergroup cg_vec;
      option.per_instance = 1;
      cp_cfg: coverpoint mn { bins c[] = {MN_VSETVLI, MN_VSETIVLI, MN_VSETVL}; }
      cp_sew: coverpoint vsew iff (mn inside {MN_VSETVLI, MN_VSETIVLI}) { bins e8 = {0}; bins e16 = {1}; bins e32 = {2}; bins e64 = {3}; }
      cp_lmul: coverpoint vlmul iff (mn inside {MN_VSETVLI, MN_VSETIVLI}) {
        bins m1 = {0}; bins m2 = {1}; bins m4 = {2}; bins m8 = {3}; bins mf8 = {5}; bins mf4 = {6}; bins mf2 = {7};
      }
      cx_sew_lmul: cross cp_sew, cp_lmul {
        // ELEN = 64：LMUL >= SEW / 64，即 e16 无 mf8，e32 无 mf8 / mf4，e64 无分数 LMUL
        ignore_bins illegal = (binsof(cp_sew.e64) && binsof(cp_lmul) intersect {5, 6, 7}) ||
                              (binsof(cp_sew.e32) && binsof(cp_lmul) intersect {5, 6}) ||
                              (binsof(cp_sew.e16) && binsof(cp_lmul.mf8));
      }
      cp_ar: coverpoint mn { bins a[] = {[MN_V_IVV:MN_V_MVX]}; }
    endgroup

    covergroup cg_vmem;
      option.per_instance = 1;
      cp_dir: coverpoint mn { bins ld = {MN_VLOAD}; bins st = {MN_VSTORE}; }
      cp_mop: coverpoint vmop { bins unit = {0}; bins idx_u = {1}; bins strided = {2}; bins idx_o = {3}; }
      cp_w: coverpoint vmop_w { bins e8 = {0}; bins e16 = {5}; bins e32 = {6}; bins e64 = {7}; }
      cx: cross cp_dir, cp_mop, cp_w;
    endgroup

    covergroup cg_trap;
      option.per_instance = 1;
      cp_exc: coverpoint t_cause iff (!t_irq) {
        bins i_acc = {1}; bins illegal = {2}; bins bkpt = {3}; bins ld_mis = {4}; bins ld_acc = {5};
        bins st_mis = {6}; bins st_acc = {7}; bins ecall_u = {8}; bins ecall_s = {9}; bins ecall_m = {11};
        bins i_pf = {12}; bins ld_pf = {13}; bins st_pf = {15};
      }
      cp_irq: coverpoint t_cause iff (t_irq) {
        bins ssi = {1}; bins msi = {3}; bins sti = {5}; bins mti = {7}; bins sei = {9}; bins mei = {11};
      }
      cp_priv: coverpoint t_priv { bins u = {0}; bins s = {1}; bins m = {3}; }
      cx_exc_priv: cross cp_exc, cp_priv {
        ignore_bins ec = (binsof(cp_exc.ecall_u) && !binsof(cp_priv.u)) || (binsof(cp_exc.ecall_s) && !binsof(cp_priv.s)) ||
                         (binsof(cp_exc.ecall_m) && !binsof(cp_priv.m)) ||
                         (binsof(cp_exc.i_pf) && binsof(cp_priv.m));     // WV-03：M 态取指不做地址翻译 (MPRV 不影响取指)
      }
      cx_irq_priv: cross cp_irq, cp_priv;
      // 中断打在哪类指令之后 (上一条提交的指令类别)
      cp_last: coverpoint last_cls iff (t_irq) {
        bins c[] = {C_ALU, C_BIT, C_MUL, C_DIV, C_LD, C_ST, C_AMO, C_BR, C_JMP, C_CSR, C_FPLD, C_FPST, C_FPU, C_FDIV, C_FCVT};
      }
      cx_irq_last: cross cp_irq, cp_last {
        ignore_bins nonext = !binsof(cp_irq) intersect {9, 11};   // 只要求外部中断 (UVM 注入的) 打遍各类
      }
      cp_lat: coverpoint irq_lat iff (t_irq && irq_lat >= 0) {
        bins l0_15 = {[0:15]}; bins l16_63 = {[16:63]}; bins l64_255 = {[64:255]}; bins l256_up = {[256:$]};
      }
    endgroup

    covergroup cg_irq_drv;
      option.per_instance = 1;
      cp_kind: coverpoint i_kind { bins mei = {1}; bins sei = {2}; bins both = {3}; }
      cp_delay: coverpoint i_delay {
        bins b2b = {[1:3]}; bins short_ = {[4:50]}; bins mid = {[51:1000]}; bins long_ = {[1001:$]};
      }
      cx: cross cp_kind, cp_delay;
    endgroup

    covergroup cg_clint_drv;
      option.per_instance = 1;
      cp_kind: coverpoint c_kind { bins mti = {0}; bins msi = {1}; }
      cp_delay: coverpoint c_delay {
        bins b2b = {[1:3]}; bins short_ = {[4:100]}; bins mid = {[101:1500]}; bins long_ = {[1501:$]};
      }
      cp_tdelta: coverpoint c_tdelta iff (c_kind == 0) { bins now = {[0:2]}; bins near = {[3:50]}; bins far = {[51:$]}; }
      cx: cross cp_kind, cp_delay;
    endgroup

    `define O6_EVCP(n) cp_``n: coverpoint cy.ev[EV_``n] { bins hit = {1}; }
    `define O6_IQCP(q, nm) cp_iq_``nm: coverpoint cy.iq_lvl[q] { bins empty = {0}; bins low = {1}; bins half = {2}; bins high = {3}; bins full = {4}; }
    covergroup cg_uarch;
      option.per_instance = 1;
      `O6_EVCP(REDIRECT) `O6_EVCP(MIS_COND) `O6_EVCP(MIS_COND_TK) `O6_EVCP(MIS_RET) `O6_EVCP(MIS_IND)
      `O6_EVCP(FL_REPLAY) `O6_EVCP(FL_TRAP) `O6_EVCP(FL_AFTER) `O6_EVCP(VIOL) `O6_EVCP(LD_FAIL)
      `O6_EVCP(FE_EMPTY) `O6_EVCP(FTQ_FULL) `O6_EVCP(IC_MISS) `O6_EVCP(F3_OVR) `O6_EVCP(F1_BTB)
      `O6_EVCP(REN_STALL) `O6_EVCP(RN_Q) `O6_EVCP(RN_TAG) `O6_EVCP(RN_IFL) `O6_EVCP(RN_FFL)
      `O6_EVCP(DS_STALL) `O6_EVCP(DS_ROB) `O6_EVCP(DS_LQ) `O6_EVCP(DS_SQ) `O6_EVCP(DS_IQ)
      `O6_EVCP(DC_LD_MISS) `O6_EVCP(DC_ST_MISS) `O6_EVCP(DC_NACK) `O6_EVCP(MSHR_FULL) `O6_EVCP(STFWD) `O6_EVCP(STFWD_WAIT)
      `O6_EVCP(PTW_I) `O6_EVCP(PTW_D) `O6_EVCP(PTW_BUSY) `O6_EVCP(DPF) `O6_EVCP(MMIO) `O6_EVCP(VU_BUSY) `O6_EVCP(CMT_FULL)
      `O6_EVCP(SQ_FULL) `O6_EVCP(LQ_FULL) `O6_EVCP(SQFULL_FLUSH)
      cp_ncmt: coverpoint cy.ncmt { bins n[] = {[0:W]}; }
      `O6_IQCP(0, a) `O6_IQCP(1, b) `O6_IQCP(2, m) `O6_IQCP(3, f) `O6_IQCP(4, s) `O6_IQCP(5, d) `O6_IQCP(6, c) `O6_IQCP(7, n)
      // 冲刷 / 重定向与 D$ 缺失、MSHR 满同拍 (恢复路径与访存缺失交叠)
      cx_mis_dmiss: cross cp_REDIRECT, cp_DC_LD_MISS;
      cx_viol_mshr: cross cp_FL_REPLAY, cp_MSHR_FULL;
      cx_ncmt_ds: cross cp_ncmt, cp_DS_STALL;
    endgroup
    `undef O6_EVCP
    `undef O6_IQCP

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg_insn = new; cg_csr = new; cg_fp = new; cg_vec = new; cg_vmem = new; cg_trap = new;
      cg_irq_drv = new; cg_clint_drv = new; cg_uarch = new;
      foreach (lastw[r]) lastw[r] = -1_000_000;
      irq_set_cyc = '{-1, -1, -1, -1};
      last_cls = C_OTHER;
    endfunction
    function void build_phase(uvm_phase phase);
      cm_imp = new("cm_imp", this); trap_imp = new("trap_imp", this);
      cyc_imp = new("cyc_imp", this); irq_imp = new("irq_imp", this); clint_imp = new("clint_imp", this);
    endfunction

    function void write_cm(o6_commit_item t);
      bit [31:0] i = t.insn;
      bit [4:0]  op = i[6:2];
      rvc = t.rvc(); rdt = t.rdt; rd = t.rd; slot = t.slot; wd = t.wd;
      cq = {i[1:0], i[15:13]};
      mn = rvc ? MN_OTHER : mn_dec(i);
      cls = rvc ? rvc_cls(i[15:0]) : mn_cls(mn);
      rs1 = i[19:15]; rs2 = i[24:20]; rm = i[14:12]; csr = i[31:20];
      // RAW 距离：本条读整数 rs1，与最近写它的那条之间隔几条 (-1 = 不读整数 rs1)
      raw_d = -1;
      if (!rvc && (op inside {5'h00, 5'h01, 5'h03, 5'h04, 5'h06, 5'h08, 5'h09, 5'h0B, 5'h0C, 5'h0E, 5'h18, 5'h19} ||
                   (op == 5'h1C && i[13:12] != 0 && !i[14])))
        raw_d = (rs1 == 0) ? 0 : int'(n_idx - lastw[rs1]);
      cg_insn.sample();
      if (mn inside {[MN_CSRRW:MN_CSRRCI]}) cg_csr.sample();
      if (mn inside {[MN_FMADD_S:MN_FCLASS_D]} && op != 5'h14 || mn inside {MN_FADD_S, MN_FSUB_S, MN_FMUL_S, MN_FDIV_S,
          MN_FSQRT_S, MN_FADD_D, MN_FSUB_D, MN_FMUL_D, MN_FDIV_D, MN_FSQRT_D, MN_FCVT_W_S, MN_FCVT_L_D, MN_FCVT_S_W,
          MN_FCVT_D_L}) cg_fp.sample();
      if (mn inside {[MN_VSETVLI:MN_VSETVL], [MN_V_IVV:MN_V_MVX]}) begin
        vsew = i[25:23]; vlmul = i[22:20];
        cg_vec.sample();
      end
      if (mn inside {MN_VLOAD, MN_VSTORE}) begin vmop = i[27:26]; vmop_w = i[14:12]; cg_vmem.sample(); end
      if (t.rdt == 1 && t.rd != 0) lastw[t.rd] = n_idx;
      n_idx++;
      last_cls = cls;
    endfunction

    function void write_trap(o6_trap_item t);
      t_irq = t.irq; t_cause = t.cause; t_priv = t.priv;
      irq_lat = -1;
      if (t.irq && t.cause == 11 && irq_set_cyc[0] >= 0) begin irq_lat = int'(t.cyc - irq_set_cyc[0]); irq_set_cyc[0] = -1; end
      if (t.irq && t.cause == 9  && irq_set_cyc[1] >= 0) begin irq_lat = int'(t.cyc - irq_set_cyc[1]); irq_set_cyc[1] = -1; end
      if (t.irq && t.cause == 7  && irq_set_cyc[2] >= 0) begin
        irq_lat = (t.cyc >= irq_set_cyc[2]) ? int'(t.cyc - irq_set_cyc[2]) : 0; irq_set_cyc[2] = -1;
      end
      if (t.irq && t.cause == 3  && irq_set_cyc[3] >= 0) begin irq_lat = int'(t.cyc - irq_set_cyc[3]); irq_set_cyc[3] = -1; end
      cg_trap.sample();
    endfunction

    function void write_cyc(o6_cycle_item t);
      cy = t; cur_cyc = t.cyc;
      cg_uarch.sample();
    endfunction

    function void write_irq(o6_irq_item t);
      i_kind = t.kind; i_delay = t.delay;
      if (t.kind[0] && irq_set_cyc[0] < 0) irq_set_cyc[0] = cur_cyc;
      if (t.kind[1] && irq_set_cyc[1] < 0) irq_set_cyc[1] = cur_cyc;
      cg_irq_drv.sample();
    endfunction

    function void write_clint(o6_clint_item t);
      c_kind = t.kind; c_delay = t.delay; c_tdelta = t.tdelta;
      if (!t.kind) irq_set_cyc[2] = cur_cyc + 8 * longint'(t.tdelta);     // 定时器：从预计到期时刻算延迟
      else if (irq_set_cyc[3] < 0) irq_set_cyc[3] = cur_cyc;
      cg_clint_drv.sample();
    endfunction

    function void report_phase(uvm_phase phase);
      `uvm_info("COV", $sformatf("功能覆盖率：指令 %0.1f%%  CSR %0.1f%%  浮点舍入 %0.1f%%  向量配置 %0.1f%%  向量访存 %0.1f%%  陷入 %0.1f%%  中断注入 %0.1f%%  CLINT 注入 %0.1f%%  微架构 %0.1f%%",
                                 cg_insn.get_inst_coverage(), cg_csr.get_inst_coverage(), cg_fp.get_inst_coverage(),
                                 cg_vec.get_inst_coverage(), cg_vmem.get_inst_coverage(), cg_trap.get_inst_coverage(),
                                 cg_irq_drv.get_inst_coverage(), cg_clint_drv.get_inst_coverage(), cg_uarch.get_inst_coverage()), UVM_NONE)
    endfunction
  endclass

  //--------------------------------------------------------------------------
  // 环境与测试
  //--------------------------------------------------------------------------
  class o6_env extends uvm_env;
    `uvm_component_utils(o6_env)
    o6_monitor    mon;
    o6_scoreboard sb;
    o6_cov        cov;
    o6_irq_agent  irq;
    o6_clint_agent clint;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      mon = o6_monitor::type_id::create("mon", this);
      sb  = o6_scoreboard::type_id::create("sb", this);
      cov = o6_cov::type_id::create("cov", this);
      irq = o6_irq_agent::type_id::create("irq", this);
      clint = o6_clint_agent::type_id::create("clint", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      mon.cm_ap.connect(sb.cm_imp);     mon.cm_ap.connect(cov.cm_imp);
      mon.trap_ap.connect(sb.trap_imp); mon.trap_ap.connect(cov.trap_imp);
      mon.cyc_ap.connect(sb.cyc_imp);   mon.cyc_ap.connect(cov.cyc_imp);
      irq.drv.ap.connect(cov.irq_imp);
      clint.drv.ap.connect(cov.clint_imp);
    endfunction
  endclass

  class o6_base_test extends uvm_test;
    `uvm_component_utils(o6_base_test)
    o6_env env;
    virtual o6_uvm_if vif;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      env = o6_env::type_id::create("env", this);
      if (!uvm_config_db#(virtual o6_uvm_if)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "没有 vif")
    endfunction
    virtual task stimulus(); endtask
    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      fork
        stimulus();
      join_none
      wait (vif.tb_done === 1'b1);
      repeat (2) @(vif.cb);
      phase.drop_objection(this);
    endtask
    function void report_phase(uvm_phase phase);
      uvm_report_server rs = uvm_report_server::get_server();
      int ne = rs.get_severity_count(UVM_ERROR) + rs.get_severity_count(UVM_FATAL);
      $display("\n=== UVM %s === %s  错误 %0d", (ne == 0) ? "PASS" : "FAIL", get_type_name(), ne);
    endfunction
  endclass

  // 随机外部中断：+irq_n=<次数> (默认 200)
  class o6_irq_test extends o6_base_test;
    `uvm_component_utils(o6_irq_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task stimulus();
      o6_irq_seq s = o6_irq_seq::type_id::create("irq_seq");
      void'($value$plusargs("irq_n=%d", s.n));
      s.start(env.irq.sqr);
    endtask
  endclass

  // 连发 / 短间隔注入 (#28 类：头部 CSR / xRET 选中后 1~2 拍内中断到达)：1~50 拍占 80%，平均约 60 拍一次
  class o6_irq_burst_item extends o6_irq_item;
    constraint c_policy {
      delay dist { [1:3] :/ 20, [4:50] :/ 60, [51:300] :/ 20 };
      kind  dist { 2'd1 := 45, 2'd2 := 45, 2'd3 := 10 };
    }
    `uvm_object_utils(o6_irq_burst_item)
    function new(string name = "o6_irq_burst_item"); super.new(name); endfunction
  endclass

  // 配 verif/tests/irq_csr.S：+irq_n=<次数> (默认 3000，程序先结束则其余不发)
  class o6_irq_burst_test extends o6_base_test;
    `uvm_component_utils(o6_irq_burst_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      o6_irq_item::type_id::set_type_override(o6_irq_burst_item::get_type());
    endfunction
    task stimulus();
      o6_irq_seq s = o6_irq_seq::type_id::create("irq_seq");
      s.n = 3000;
      void'($value$plusargs("irq_n=%d", s.n));
      s.start(env.irq.sqr);
    endtask
  endclass

  // 定时器 / 软件中断 + 外部中断同时注入 (配 verif/tests/irq_tmr.S)：+clint_n=<次数> (默认 300)、+irq_n=<次数> (默认 100)
  class o6_tmr_test extends o6_base_test;
    `uvm_component_utils(o6_tmr_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task stimulus();
      o6_clint_seq c = o6_clint_seq::type_id::create("clint_seq");
      o6_irq_seq   e = o6_irq_seq::type_id::create("irq_seq");
      e.n = 100;
      void'($value$plusargs("clint_n=%d", c.n));
      void'($value$plusargs("irq_n=%d", e.n));
      fork
        c.start(env.clint.sqr);
        e.start(env.irq.sqr);
      join
    endtask
  endclass
endpackage
