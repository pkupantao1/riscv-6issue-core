//==============================================================================
// o6_dm.sv - RISC-V 调试模块 (Debug Spec 0.13.2)，单 hart，执行式 (程序缓冲)
//
// DMI 侧：dmcontrol / dmstatus / hartinfo / abstractcs / command / data0-1 / progbuf0-7 / haltsum0
//   抽象命令只支持 Access Register (cmdtype 0)：GPR / FPR / CSR，32/64 位，transfer + postexec
//   不支持 System Bus Access、abstractauto、quick access、hartreset、resethaltreq
// 核侧：调试存储区 0x000-0xFFF (只在调试模式可访问)：
//   0x100 HALTED    hart 写 hartid：已进入停机循环 (也表示上一条抽象命令执行完)
//   0x104 GOING     hart 写：已取走 go 标志
//   0x108 RESUMING  hart 写 hartid：正在恢复运行
//   0x10C EXCEPTION hart 写：抽象命令 / progbuf 执行中发生异常
//   0x300 WHERETO   jal 到抽象命令区
//   0x338 抽象命令区 (10 条)，0x360 progbuf (8 条)，0x380 隐式 ebreak (impebreak = 1)
//   0x3C0 data0，0x3C4 data1
//   0x400 FLAGS     bit0 = go，bit1 = resume
//   0x800 ROM       入口 0x800，调试模式异常入口 0x808
// 核取指走行读口 (64 B，不进 I$)，读写走 MMIO 口 (64 位 + 字节掩码)。
//==============================================================================
`default_nettype none
module o6_dm
  import o6_isa_pkg::*;
(
  input  wire          clk_i,
  input  wire          rst_ni,           // 只受上电复位 (不受 ndmreset)

  // DMI (来自 DTM，已在本时钟域)
  input  wire          dmi_req_i,
  input  wire  [6:0]   dmi_addr_i,
  input  wire  [31:0]  dmi_wdata_i,
  input  wire  [1:0]   dmi_op_i,         // 1 读，2 写
  output logic         dmi_resp_o,       // 下一拍给结果
  output logic [31:0]  dmi_rdata_o,

  // 到核
  output logic         haltreq_o,        // 停机请求 (电平)
  output logic         ndmreset_o,       // 复位除调试模块外的全系统
  input  wire          hart_unavail_i,   // hart 在复位中

  // 核的 MMIO 访问 (调试模式下的 load/store)
  input  wire          mem_req_i,
  input  wire          mem_we_i,
  input  wire  [11:0]  mem_addr_i,
  input  wire  [63:0]  mem_wdata_i,
  input  wire  [7:0]   mem_wmask_i,
  output logic [63:0]  mem_rdata_o,      // 组合输出，与 req 同拍

  // 核取指 (行读，64 B)
  input  wire  [11:6]  line_addr_i,
  output logic [511:0] line_data_o
);
  localparam logic [11:0] A_HALTED = 12'h100, A_GOING = 12'h104, A_RESUMING = 12'h108, A_EXCEPTION = 12'h10C;
  localparam logic [11:0] A_WHERETO = 12'h300, A_ABST = 12'h338, A_PROGBUF = 12'h360, A_IMPEBREAK = 12'h380;
  localparam logic [11:0] A_DATA = 12'h3C0, A_FLAGS = 12'h400, A_ROM = 12'h800;
  localparam int NPB = 8;

  //---------------- 调试 ROM (源码 sw/debug/debug_rom.S，这里是汇编结果，已用 binutils 核对)
  //  0x800: j _entry            0x804: nop               0x808: j _exception
  //  _entry (0x80C):  fence; csrw dscratch0, s0
  //  loop:   csrr s0, mhartid; sw s0, 0x100(zero); lbu s0, 0x400(zero)
  //          andi s0, s0, 1; bnez s0, going
  //          lbu s0, 0x400(zero); andi s0, s0, 2; bnez s0, resume; j loop
  //  _exception: sw zero, 0x10C(zero); ebreak
  //  going:  csrr s0, dscratch0; sw zero, 0x104(zero); jalr zero, 0x300(zero)
  //  resume: csrr s0, mhartid; sw s0, 0x108(zero); csrr s0, dscratch0; dret
  localparam int NROM = 24;
  logic [31:0] rom [NROM];
  assign rom[0]  = 32'h00c0006f;   // 800 j     80c
  assign rom[1]  = 32'h00000013;   // 804 nop
  assign rom[2]  = 32'h0340006f;   // 808 j     83c (_exception)
  assign rom[3]  = 32'h0ff0000f;   // 80c fence
  assign rom[4]  = 32'h7b241073;   // 810 csrw  dscratch0, s0
  assign rom[5]  = 32'hf1402473;   // 814 csrr  s0, mhartid           (loop)
  assign rom[6]  = 32'h10802023;   // 818 sw    s0, 0x100(zero)
  assign rom[7]  = 32'h40004403;   // 81c lbu   s0, 0x400(zero)
  assign rom[8]  = 32'h00147413;   // 820 andi  s0, s0, 1
  assign rom[9]  = 32'h02041063;   // 824 bnez  s0, 844 (going)
  assign rom[10] = 32'h40004403;   // 828 lbu   s0, 0x400(zero)
  assign rom[11] = 32'h00247413;   // 82c andi  s0, s0, 2
  assign rom[12] = 32'h02041263;   // 830 bnez  s0, 854 (resume)
  assign rom[13] = 32'hfe1ff06f;   // 834 j     814 (loop)
  assign rom[14] = 32'h00000013;   // 838 nop
  assign rom[15] = 32'h10002623;   // 83c sw    zero, 0x10c(zero)    (_exception)
  assign rom[16] = 32'h00100073;   // 840 ebreak
  assign rom[17] = 32'h7b202473;   // 844 csrr  s0, dscratch0        (going)
  assign rom[18] = 32'h10002223;   // 848 sw    zero, 0x104(zero)
  assign rom[19] = 32'h30000067;   // 84c jalr  zero, 0x300(zero)
  assign rom[20] = 32'h00000013;   // 850 nop
  assign rom[21] = 32'hf1402473;   // 854 csrr  s0, mhartid          (resume)
  assign rom[22] = 32'h10802423;   // 858 sw    s0, 0x108(zero)
  assign rom[23] = 32'h7b202473;   // 85c csrr  s0, dscratch0
  // 0x860: dret (单独一项，NROM 之外)
  localparam logic [31:0] DRET = 32'h7b200073;
  localparam logic [31:0] NOP = 32'h00000013, EBREAK = 32'h00100073;

  //---------------- 指令编码
  function automatic logic [31:0] i_csrrw(input logic [4:0] rd, input logic [11:0] csr, input logic [4:0] rs1);
    return {csr, rs1, 3'b001, rd, 7'b1110011};
  endfunction
  function automatic logic [31:0] i_csrrs(input logic [4:0] rd, input logic [11:0] csr, input logic [4:0] rs1);
    return {csr, rs1, 3'b010, rd, 7'b1110011};
  endfunction
  function automatic logic [31:0] i_load(input logic [4:0] rd, input logic [2:0] f3, input logic [11:0] imm,
                                         input logic fp);
    return {imm, 5'd0, f3, rd, fp ? 7'b0000111 : 7'b0000011};
  endfunction
  function automatic logic [31:0] i_store(input logic [4:0] rs2, input logic [2:0] f3, input logic [11:0] imm,
                                          input logic fp);
    return {imm[11:5], rs2, 5'd0, f3, imm[4:0], fp ? 7'b0100111 : 7'b0100011};
  endfunction

  //---------------- 寄存器
  logic        dmactive, ndmreset, haltreq;
  logic        halted, resumeack, havereset;
  logic [31:0] data [2];
  logic [31:0] progbuf [NPB];
  logic        busy;
  logic [2:0]  cmderr;
  logic [31:0] cmd_q;
  logic        flag_go, flag_resume;
  logic [31:0] abst [10];
  logic [9:0]  hartsel;

  assign haltreq_o  = haltreq && dmactive;
  assign ndmreset_o = ndmreset && dmactive;

  wire sel_ok = (hartsel == 10'd0);

  //---------------- DMI 读
  logic [31:0] rd_dmi;
  always_comb begin
    rd_dmi = '0;
    unique case (dmi_addr_i)
      7'h04: rd_dmi = data[0];
      7'h05: rd_dmi = data[1];
      7'h10: rd_dmi = {haltreq, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, hartsel, 14'b0, ndmreset, dmactive};
      7'h11: begin
        logic nonex, unav, hlt, run;
        nonex = !sel_ok;
        unav  = sel_ok && hart_unavail_i;
        hlt   = sel_ok && !unav && halted;
        run   = sel_ok && !unav && !halted;
        rd_dmi = {9'b0, 1'b1 /*impebreak*/, 2'b0, {2{sel_ok && havereset}}, {2{sel_ok && resumeack}},
                  {2{nonex}}, {2{unav}}, {2{run}}, {2{hlt}}, 1'b1 /*authenticated*/, 1'b0, 1'b0 /*hasresethaltreq*/,
                  1'b0 /*confstrptrvalid*/, 4'd2 /*version 0.13*/};
      end
      7'h12: rd_dmi = {8'b0, 4'd2 /*nscratch*/, 3'b0, 1'b1 /*dataaccess*/, 4'd2 /*datasize*/, A_DATA};
      7'h16: rd_dmi = {3'b0, 5'(NPB), 11'b0, busy, 1'b0, cmderr, 4'b0, 4'd2};
      7'h17: rd_dmi = '0;                // command 只写
      7'h18: rd_dmi = '0;                // abstractauto 不支持
      7'h38: rd_dmi = '0;                // sbcs：无系统总线访问
      7'h40: rd_dmi = {31'b0, sel_ok && halted};
      default: begin
        if (dmi_addr_i >= 7'h20 && dmi_addr_i < 7'h20 + 7'(NPB)) rd_dmi = progbuf[dmi_addr_i[2:0]];
      end
    endcase
  end

  //---------------- 抽象命令译码
  // regno：0x0000-0x0FFF CSR，0x1000-0x101F GPR，0x1020-0x103F FPR
  logic        c_ok;
  logic [31:0] c_prog [10];
  always_comb begin
    logic [31:0] cm;
    logic [2:0]  sz;
    logic        tr, wr, pe;
    logic [15:0] rn;
    logic [2:0]  f3;
    cm = dmi_wdata_i;
    sz = cm[22:20]; pe = cm[18]; tr = cm[17]; wr = cm[16]; rn = cm[15:0];
    f3 = (sz == 3'd3) ? 3'b011 : 3'b010;
    for (int i = 0; i < 10; i++) c_prog[i] = NOP;
    c_ok = (cm[31:24] == 8'd0) && !cm[19] /*postincrement*/ && (!tr || sz == 3'd2 || sz == 3'd3);
    if (tr && c_ok) begin
      if (rn[15:12] == 4'h1 && rn[11:5] == 7'd0) begin
        // GPR
        c_prog[0] = wr ? i_load(rn[4:0], f3, A_DATA, 1'b0) : i_store(rn[4:0], f3, A_DATA, 1'b0);
      end else if (rn[15:12] == 4'h1 && rn[11:5] == 7'd1) begin
        // FPR (mstatus.FS 为 0 时会异常 -> cmderr = exception)
        c_prog[0] = wr ? i_load(rn[4:0], f3, A_DATA, 1'b1) : i_store(rn[4:0], f3, A_DATA, 1'b1);
      end else if (rn[15:12] == 4'h0) begin
        // CSR：用 s0 中转，原值暂存 dscratch1
        c_prog[0] = i_csrrw(5'd0, CSR_DSCRATCH1, 5'd8);
        if (wr) begin
          c_prog[1] = i_load(5'd8, f3, A_DATA, 1'b0);
          c_prog[2] = i_csrrw(5'd0, rn[11:0], 5'd8);
        end else begin
          c_prog[1] = i_csrrs(5'd8, rn[11:0], 5'd0);
          c_prog[2] = i_store(5'd8, f3, A_DATA, 1'b0);
        end
        c_prog[3] = i_csrrs(5'd8, CSR_DSCRATCH1, 5'd0);
      end else begin
        c_ok = 1'b0;
      end
    end
    // 最后一条：postexec 落到 progbuf (0x360 紧接其后)，否则 ebreak 回 ROM
    c_prog[9] = pe ? NOP : EBREAK;
  end

  //---------------- 状态更新
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dmactive <= 1'b0; ndmreset <= 1'b0; haltreq <= 1'b0; hartsel <= '0;
      halted <= 1'b0; resumeack <= 1'b0; havereset <= 1'b1;
      busy <= 1'b0; cmderr <= '0; cmd_q <= '0;
      flag_go <= 1'b0; flag_resume <= 1'b0;
      for (int i = 0; i < 2; i++) data[i] <= '0;
      for (int i = 0; i < NPB; i++) progbuf[i] <= NOP;
      for (int i = 0; i < 10; i++) abst[i] <= NOP;
      dmi_resp_o <= 1'b0; dmi_rdata_o <= '0;
    end else begin
      dmi_resp_o <= dmi_req_i;
      if (dmi_req_i) dmi_rdata_o <= rd_dmi;
      if (hart_unavail_i) havereset <= 1'b1;

      // ---- DMI 写
      if (dmi_req_i && dmi_op_i == 2'd2) begin
        unique case (dmi_addr_i)
          7'h04: if (!busy) data[0] <= dmi_wdata_i; else if (cmderr == 3'd0) cmderr <= 3'd1;
          7'h05: if (!busy) data[1] <= dmi_wdata_i; else if (cmderr == 3'd0) cmderr <= 3'd1;
          7'h10: begin
            dmactive <= dmi_wdata_i[0];
            if (!dmi_wdata_i[0]) begin
              // dmactive = 0：调试模块复位
              ndmreset <= 1'b0; haltreq <= 1'b0; hartsel <= '0; busy <= 1'b0; cmderr <= '0;
              flag_go <= 1'b0; flag_resume <= 1'b0;
            end else begin
              ndmreset <= dmi_wdata_i[1];
              haltreq  <= dmi_wdata_i[31];
              hartsel  <= dmi_wdata_i[25:16];
              if (dmi_wdata_i[28]) havereset <= 1'b0;                     // ackhavereset
              // resumereq：只在已停机且没有同时 haltreq 时有效
              if (dmi_wdata_i[30] && !dmi_wdata_i[31] && sel_ok && halted) begin
                flag_resume <= 1'b1; resumeack <= 1'b0;
              end
            end
          end
          7'h16: cmderr <= cmderr & ~dmi_wdata_i[10:8];                   // W1C
          7'h17: if (dmactive) begin
            if (busy) begin
              if (cmderr == 3'd0) cmderr <= 3'd1;
            end else if (cmderr != 3'd0) begin
              // 有未清的错误：忽略
            end else if (!c_ok) begin
              cmderr <= 3'd2;
            end else if (!(sel_ok && halted)) begin
              cmderr <= 3'd4;
            end else begin
              cmd_q <= dmi_wdata_i;
              for (int i = 0; i < 10; i++) abst[i] <= c_prog[i];
              busy <= 1'b1; flag_go <= 1'b1;
            end
          end
          default: begin
            if (dmi_addr_i >= 7'h20 && dmi_addr_i < 7'h20 + 7'(NPB)) begin
              if (!busy) progbuf[dmi_addr_i[2:0]] <= dmi_wdata_i;
              else if (cmderr == 3'd0) cmderr <= 3'd1;
            end
          end
        endcase
      end

      // ---- 核写调试存储区
      if (mem_req_i && mem_we_i) begin
        unique case ({mem_addr_i[11:3], 3'b000})
          A_HALTED: begin        // 0x100 HALTED (低 4 字节) / 0x104 GOING (高 4 字节)
            if (mem_wmask_i[3:0] != 4'h0) begin
              halted <= 1'b1;
              busy   <= 1'b0;                                              // 回到停机循环 = 命令完成
            end
            if (mem_wmask_i[7:4] != 4'h0) flag_go <= 1'b0;
          end
          A_RESUMING: begin      // 0x108 RESUMING / 0x10C EXCEPTION
            if (mem_wmask_i[3:0] != 4'h0) begin
              halted <= 1'b0; resumeack <= 1'b1; flag_resume <= 1'b0;
            end
            if (mem_wmask_i[7:4] != 4'h0 && cmderr == 3'd0) cmderr <= 3'd3;
          end
          A_DATA: begin
            for (int b = 0; b < 4; b++) if (mem_wmask_i[b])     data[0][8*b +: 8] <= mem_wdata_i[8*b +: 8];
            for (int b = 0; b < 4; b++) if (mem_wmask_i[4 + b]) data[1][8*b +: 8] <= mem_wdata_i[32 + 8*b +: 8];
          end
          default: ;
        endcase
      end
      // hart 复位 / ndmreset：停机状态清零
      if (hart_unavail_i) begin halted <= 1'b0; flag_go <= 1'b0; flag_resume <= 1'b0; busy <= 1'b0; end
    end
  end

  //---------------- 调试存储区读 (按 32 位字)
  function automatic logic [31:0] mem_word(input logic [11:2] wa);
    logic [11:0] a;
    a = {wa, 2'b00};
    if (a == A_WHERETO)                                return 32'h0380006f;          // jal zero, +0x38 (-> 0x338)
    if (a >= A_ABST && a < A_ABST + 12'd40)            return abst[(a - A_ABST) >> 2];
    if (a >= A_PROGBUF && a < A_PROGBUF + 12'(4*NPB))  return progbuf[(a - A_PROGBUF) >> 2];
    if (a == A_IMPEBREAK)                              return EBREAK;
    if (a == A_DATA)                                   return data[0];
    if (a == A_DATA + 12'd4)                           return data[1];
    if (a == A_FLAGS)                                  return {30'b0, flag_resume, flag_go};
    if (a >= A_ROM && a < A_ROM + 12'(4*NROM))         return rom[(a - A_ROM) >> 2];
    if (a == A_ROM + 12'(4*NROM))                      return DRET;
    return 32'h0;
  endfunction

  assign mem_rdata_o = {mem_word({mem_addr_i[11:3], 1'b1}), mem_word({mem_addr_i[11:3], 1'b0})};
  always_comb begin
    for (int i = 0; i < 16; i++) line_data_o[32*i +: 32] = mem_word({line_addr_i, 4'(i)});
  end

  wire unused = ^cmd_q ^ ^mem_addr_i[2:0];
endmodule
