//==============================================================================
// o6_isa_pkg.sv - RISC-V 指令集常量：操作码、CSR 地址、异常/中断编号
//
// 只放 ISA 规范里定死的东西（与微架构参数分开，后者在 o6_pkg）。
// 依据：RISC-V Unprivileged ISA 20191213, Privileged Architecture 1.12
//==============================================================================
`ifndef O6_ISA_PKG_SV
`define O6_ISA_PKG_SV
`default_nettype none

package o6_isa_pkg;

  //--------------------------------------------------------------------------
  // 主操作码 inst[6:0]
  //--------------------------------------------------------------------------
  localparam logic [6:0] OPC_LOAD     = 7'b0000011;
  localparam logic [6:0] OPC_LOAD_FP  = 7'b0000111;
  localparam logic [6:0] OPC_MISC_MEM = 7'b0001111;
  localparam logic [6:0] OPC_OP_IMM   = 7'b0010011;
  localparam logic [6:0] OPC_AUIPC    = 7'b0010111;
  localparam logic [6:0] OPC_OP_IMM32 = 7'b0011011;
  localparam logic [6:0] OPC_STORE    = 7'b0100011;
  localparam logic [6:0] OPC_STORE_FP = 7'b0100111;
  localparam logic [6:0] OPC_AMO      = 7'b0101111;
  localparam logic [6:0] OPC_OP       = 7'b0110011;
  localparam logic [6:0] OPC_LUI      = 7'b0110111;
  localparam logic [6:0] OPC_OP32     = 7'b0111011;
  localparam logic [6:0] OPC_MADD     = 7'b1000011;
  localparam logic [6:0] OPC_MSUB     = 7'b1000111;
  localparam logic [6:0] OPC_NMSUB    = 7'b1001011;
  localparam logic [6:0] OPC_NMADD    = 7'b1001111;
  localparam logic [6:0] OPC_OP_FP    = 7'b1010011;
  localparam logic [6:0] OPC_OP_V     = 7'b1010111;     // RVV (向量运算 / vsetvl*)
  localparam logic [6:0] OPC_BRANCH   = 7'b1100011;
  localparam logic [6:0] OPC_JALR     = 7'b1100111;
  localparam logic [6:0] OPC_JAL      = 7'b1101111;
  localparam logic [6:0] OPC_SYSTEM   = 7'b1110011;
  localparam logic [6:0] OPC_OP_VE    = 7'b1110111;     // OP-VE：向量点积 (Zvqwdota* / Zvfqwdota8f / Zvfwdota16bf)

  //--------------------------------------------------------------------------
  // 特权级
  //--------------------------------------------------------------------------
  typedef enum logic [1:0] {
    PRV_U = 2'b00,
    PRV_S = 2'b01,
    PRV_M = 2'b11
  } priv_e;

  //--------------------------------------------------------------------------
  // 异常编号 (mcause, interrupt=0)
  //--------------------------------------------------------------------------
  localparam logic [5:0] EXC_INST_MISALIGN  = 6'd0;
  localparam logic [5:0] EXC_INST_ACCESS    = 6'd1;
  localparam logic [5:0] EXC_ILLEGAL_INST   = 6'd2;
  localparam logic [5:0] EXC_BREAKPOINT     = 6'd3;
  localparam logic [5:0] EXC_LOAD_MISALIGN  = 6'd4;
  localparam logic [5:0] EXC_LOAD_ACCESS    = 6'd5;
  localparam logic [5:0] EXC_STORE_MISALIGN = 6'd6;
  localparam logic [5:0] EXC_STORE_ACCESS   = 6'd7;
  localparam logic [5:0] EXC_ECALL_U        = 6'd8;
  localparam logic [5:0] EXC_ECALL_S        = 6'd9;
  localparam logic [5:0] EXC_ECALL_VS       = 6'd10;    // H：VS 态 ecall
  localparam logic [5:0] EXC_ECALL_M        = 6'd11;
  localparam logic [5:0] EXC_INST_PAGE      = 6'd12;
  localparam logic [5:0] EXC_LOAD_PAGE      = 6'd13;
  localparam logic [5:0] EXC_STORE_PAGE     = 6'd15;
  // H 扩展 (RVA23 R5，docs/15)
  localparam logic [5:0] EXC_INST_GPAGE     = 6'd20;    // guest page fault (G-stage)
  localparam logic [5:0] EXC_LOAD_GPAGE     = 6'd21;
  localparam logic [5:0] EXC_VIRT_INST      = 6'd22;    // virtual instruction
  localparam logic [5:0] EXC_STORE_GPAGE    = 6'd23;

  // 中断编号 (mcause, interrupt=1)
  localparam int IRQ_S_SOFT  = 1;
  localparam int IRQ_M_SOFT  = 3;
  localparam int IRQ_S_TIMER = 5;
  localparam int IRQ_M_TIMER = 7;
  localparam int IRQ_S_EXT   = 9;
  localparam int IRQ_M_EXT   = 11;

  //--------------------------------------------------------------------------
  // CSR 地址
  //--------------------------------------------------------------------------
  localparam logic [11:0] CSR_FFLAGS     = 12'h001;
  localparam logic [11:0] CSR_FRM        = 12'h002;
  localparam logic [11:0] CSR_FCSR       = 12'h003;
  // 向量 CSR (RVV 1.0)：都在向量单元里 (访问它们的 csrr* 译成 FU_VEC)
  localparam logic [11:0] CSR_VSTART     = 12'h008;
  localparam logic [11:0] CSR_VXSAT      = 12'h009;
  localparam logic [11:0] CSR_VXRM       = 12'h00A;
  localparam logic [11:0] CSR_VCSR       = 12'h00F;
  localparam logic [11:0] CSR_VL         = 12'hC20;
  localparam logic [11:0] CSR_VTYPE      = 12'hC21;
  localparam logic [11:0] CSR_VLENB      = 12'hC22;
  localparam logic [11:0] CSR_CYCLE      = 12'hC00;
  localparam logic [11:0] CSR_TIME       = 12'hC01;
  localparam logic [11:0] CSR_INSTRET    = 12'hC02;
  localparam logic [11:0] CSR_HPMCNT3    = 12'hC03;
  localparam logic [11:0] CSR_HPMCNT31   = 12'hC1F;

  localparam logic [11:0] CSR_SSTATUS    = 12'h100;
  localparam logic [11:0] CSR_SIE        = 12'h104;
  localparam logic [11:0] CSR_STVEC      = 12'h105;
  localparam logic [11:0] CSR_SCOUNTEREN = 12'h106;
  localparam logic [11:0] CSR_SENVCFG    = 12'h10A;
  localparam logic [11:0] CSR_SSCRATCH   = 12'h140;
  localparam logic [11:0] CSR_SEPC       = 12'h141;
  localparam logic [11:0] CSR_SCAUSE     = 12'h142;
  localparam logic [11:0] CSR_STVAL      = 12'h143;
  localparam logic [11:0] CSR_SIP        = 12'h144;
  localparam logic [11:0] CSR_SATP       = 12'h180;
  localparam logic [11:0] CSR_STIMECMP   = 12'h14D;
  // H 扩展：HS 级 (0x6xx) 与 VS 级 (0x2xx)
  localparam logic [11:0] CSR_HSTATUS    = 12'h600;
  localparam logic [11:0] CSR_HEDELEG    = 12'h602;
  localparam logic [11:0] CSR_HIDELEG    = 12'h603;
  localparam logic [11:0] CSR_HIE        = 12'h604;
  localparam logic [11:0] CSR_HTIMEDELTA = 12'h605;
  localparam logic [11:0] CSR_HCOUNTEREN = 12'h606;
  localparam logic [11:0] CSR_HGEIE      = 12'h607;
  localparam logic [11:0] CSR_HENVCFG    = 12'h60A;
  localparam logic [11:0] CSR_HSTATEEN0  = 12'h60C;
  localparam logic [11:0] CSR_HTVAL      = 12'h643;
  localparam logic [11:0] CSR_HIP        = 12'h644;
  localparam logic [11:0] CSR_HVIP       = 12'h645;
  localparam logic [11:0] CSR_HTINST     = 12'h64A;
  localparam logic [11:0] CSR_HGATP      = 12'h680;
  localparam logic [11:0] CSR_HCONTEXT   = 12'h6A8;
  localparam logic [11:0] CSR_HGEIP      = 12'hE12;
  localparam logic [11:0] CSR_VSSTATUS   = 12'h200;
  localparam logic [11:0] CSR_VSIE       = 12'h204;
  localparam logic [11:0] CSR_VSTVEC     = 12'h205;
  localparam logic [11:0] CSR_VSSCRATCH  = 12'h240;
  localparam logic [11:0] CSR_VSEPC      = 12'h241;
  localparam logic [11:0] CSR_VSCAUSE    = 12'h242;
  localparam logic [11:0] CSR_VSTVAL     = 12'h243;
  localparam logic [11:0] CSR_VSIP       = 12'h244;
  localparam logic [11:0] CSR_VSTIMECMP  = 12'h24D;
  localparam logic [11:0] CSR_VSATP      = 12'h280;
  localparam logic [11:0] CSR_MTINST     = 12'h34A;
  localparam logic [11:0] CSR_MTVAL2     = 12'h34B;

  localparam logic [11:0] CSR_MVENDORID  = 12'hF11;
  localparam logic [11:0] CSR_MARCHID    = 12'hF12;
  localparam logic [11:0] CSR_MIMPID     = 12'hF13;
  localparam logic [11:0] CSR_MHARTID    = 12'hF14;
  localparam logic [11:0] CSR_MCONFIGPTR = 12'hF15;
  localparam logic [11:0] CSR_MSTATUS    = 12'h300;
  localparam logic [11:0] CSR_MISA       = 12'h301;
  localparam logic [11:0] CSR_MEDELEG    = 12'h302;
  localparam logic [11:0] CSR_MIDELEG    = 12'h303;
  localparam logic [11:0] CSR_MIE        = 12'h304;
  localparam logic [11:0] CSR_MTVEC      = 12'h305;
  localparam logic [11:0] CSR_MCOUNTEREN = 12'h306;
  localparam logic [11:0] CSR_MENVCFG    = 12'h30A;
  localparam logic [11:0] CSR_MCOUNTINHIBIT = 12'h320;
  localparam logic [11:0] CSR_MHPMEVENT3 = 12'h323;
  localparam logic [11:0] CSR_MHPMEVENT31 = 12'h33F;
  localparam logic [11:0] CSR_MSCRATCH   = 12'h340;
  localparam logic [11:0] CSR_MEPC       = 12'h341;
  localparam logic [11:0] CSR_MCAUSE     = 12'h342;
  localparam logic [11:0] CSR_MTVAL      = 12'h343;
  localparam logic [11:0] CSR_MIP        = 12'h344;
  localparam logic [11:0] CSR_PMPCFG0    = 12'h3A0;
  localparam logic [11:0] CSR_PMPCFG2    = 12'h3A2;
  localparam logic [11:0] CSR_PMPADDR0   = 12'h3B0;
  localparam logic [11:0] CSR_MCYCLE     = 12'hB00;
  localparam logic [11:0] CSR_MINSTRET   = 12'hB02;
  localparam logic [11:0] CSR_MHPMCNT3   = 12'hB03;
  localparam logic [11:0] CSR_MHPMCNT31  = 12'hB1F;

  localparam logic [11:0] CSR_TSELECT    = 12'h7A0;
  localparam logic [11:0] CSR_TDATA1     = 12'h7A1;
  localparam logic [11:0] CSR_DCSR       = 12'h7B0;
  localparam logic [11:0] CSR_DPC        = 12'h7B1;
  localparam logic [11:0] CSR_DSCRATCH0  = 12'h7B2;
  localparam logic [11:0] CSR_DSCRATCH1  = 12'h7B3;

  //--------------------------------------------------------------------------
  // satp 模式
  //--------------------------------------------------------------------------
  localparam logic [3:0] SATP_BARE = 4'd0;
  localparam logic [3:0] SATP_SV39 = 4'd8;
  localparam logic [3:0] SATP_SV48 = 4'd9;
  localparam logic [3:0] SATP_SV57 = 4'd10;

endpackage

`endif
