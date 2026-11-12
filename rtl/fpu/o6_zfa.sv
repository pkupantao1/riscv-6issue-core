//==============================================================================
// o6_zfa.sv - Zfa 运算 (纯组合，RVA23 R2b，docs/14)
//
// 都是对原始编码的简单运算，不经 fpnew；o6_fpu_exu 把它放在 U6 口旁的同延迟旁路里：
//   fli.{h,s,d}：32 项常数表 (按 rs1 下标)
//   fminm / fmaxm.{s,d}：任一操作数是 NaN 出规范 NaN；否则同 fmin / fmax (−0 < +0)；sNaN 报 NV
//   fround / froundnx.{s,d}：softfloat roundToInt (原始编码上加增量再屏蔽小数位)；froundnx 不精确时报 NX
//   fcvtmod.w.d：截断取整后模 2^32、符号扩展 (JavaScript 语义)；NaN / Inf / 超出 int32 报 NV (优先于 NX)
//   fleq / fltq.{s,d}：静默比较 (只有 sNaN 报 NV)
// 逐句对照 Spike (riscv/insns/{fli,fminm,fmaxm,fround,froundnx,fcvtmod_w_d,fleq,fltq}_*.h) 与 softfloat f*_roundToInt。
// .h 只有 fli.h (Spike：Zfh 或 Zvfh)；其余 .h 要 Zfh (本核只有 Zfhmin)，译码已判非法。
// 单精度输入未按 NaN-boxing 装箱时按规范 NaN；结果按格式装箱。
//==============================================================================
`default_nettype none
module o6_zfa_f #(
  parameter int EW = 11,
  parameter int MW = 52
) (
  input  wire  [EW+MW:0]  a_i,
  input  wire  [EW+MW:0]  b_i,
  input  wire  [2:0]      rm_i,        // 000 RNE 001 RTZ 010 RDN 011 RUP 100 RMM
  input  wire             exact_i,     // froundnx：不精确时报 NX
  output logic [EW+MW:0]  minm_o,
  output logic [EW+MW:0]  maxm_o,
  output logic [EW+MW:0]  rnd_o,
  output logic            rnd_nv_o,
  output logic            rnd_nx_o,
  output logic            lt_o,        // a < b (静默)
  output logic            le_o,        // a <= b (静默)
  output logic            nv_q_o       // 任一操作数是 sNaN
);
  localparam int W = EW + MW + 1;
  localparam logic [EW-1:0] BIAS = EW'((1 << (EW - 1)) - 1);
  localparam logic [W-1:0]  CNAN = {1'b0, {EW{1'b1}}, 1'b1, {(MW-1){1'b0}}};
  localparam logic [W-1:0]  ONE  = {1'b0, BIAS, {MW{1'b0}}};

  wire          sa = a_i[W-1], sb = b_i[W-1];
  wire [EW-1:0] ea = a_i[W-2:MW], eb = b_i[W-2:MW];
  wire [MW-1:0] ma = a_i[MW-1:0], mb = b_i[MW-1:0];
  wire nan_a  = (&ea) && (|ma), nan_b = (&eb) && (|mb);
  wire snan_a = nan_a && !ma[MW-1], snan_b = nan_b && !mb[MW-1];
  wire zero_a = ~|a_i[W-2:0], zero_b = ~|b_i[W-2:0];

  // 静默比较 (NaN 时不成立)
  function automatic logic lt_mag(input logic [W-1:0] x, input logic [W-1:0] y, input logic zx, input logic zy);
    if (zx && zy)           return 1'b0;
    if (x[W-1] != y[W-1])   return x[W-1];
    if (!x[W-1])            return x[W-2:0] < y[W-2:0];
    return x[W-2:0] > y[W-2:0];
  endfunction
  wire any_nan = nan_a || nan_b;
  wire eq      = !any_nan && ((a_i == b_i) || (zero_a && zero_b));
  wire lt_ab   = !any_nan && lt_mag(a_i, b_i, zero_a, zero_b);
  wire lt_ba   = !any_nan && lt_mag(b_i, a_i, zero_b, zero_a);
  assign lt_o   = lt_ab;
  assign le_o   = lt_ab || eq;
  assign nv_q_o = snan_a || snan_b;
  // fminm：less = lt(a, b) || (eq && a 为负)；fmaxm：greater = lt(b, a) || (eq && b 为负)
  assign minm_o = any_nan ? CNAN : ((lt_ab || (eq && sa)) ? a_i : b_i);
  assign maxm_o = any_nan ? CNAN : ((lt_ba || (eq && sb)) ? a_i : b_i);

  // roundToInt (softfloat)
  always_comb begin
    logic [W-1:0] z, last, rbm;
    int           sh;
    rnd_nv_o = 1'b0;
    rnd_nx_o = 1'b0;
    z = a_i;
    if (ea <= BIAS - 1'b1) begin                         // |a| < 1
      if (a_i[W-2:0] != '0) begin
        rnd_nx_o = exact_i;
        z = {sa, {(W-1){1'b0}}};
        unique case (rm_i)
          3'b000: if (ma != '0 && ea == BIAS - 1'b1) z = z | ONE;   // RNE：frac = 0 时 (含 0.5) 为 0，否则同 RMM
          3'b100: if (ea == BIAS - 1'b1) z = z | ONE;               // RMM
          3'b010: if (sa)  z = {1'b1, BIAS, {MW{1'b0}}};            // RDN：负数 -> -1
          3'b011: if (!sa) z = ONE;                                 // RUP：正数 -> +1
          default: ;                                                // RTZ：±0
        endcase
      end
    end else if (ea >= BIAS + EW'(MW)) begin              // 已是整数 / Inf / NaN
      if (nan_a) begin z = CNAN; rnd_nv_o = snan_a; end
    end else begin
      sh   = int'(BIAS) + MW - int'(ea);                  // 1 .. MW
      last = W'(1) << sh;
      rbm  = last - 1'b1;
      unique case (rm_i)
        3'b100: z = z + (last >> 1);                                        // RMM
        3'b000: begin z = z + (last >> 1); if ((z & rbm) == '0) z = z & ~last; end   // RNE
        3'b010: if (sa)  z = z + rbm;                                       // RDN
        3'b011: if (!sa) z = z + rbm;                                       // RUP
        default: ;                                                          // RTZ
      endcase
      z = z & ~rbm;
      rnd_nx_o = exact_i && (z != a_i);
    end
    rnd_o = z;
  end
endmodule

module o6_zfa
  import o6_pkg::*;
(
  input  wire  [4:0]   op_i,
  input  wire          f2i_i,        // 1：F2I 类 (fcvtmod / fleq / fltq，整数目的)，0：FMISC 类 (浮点目的)
  input  wire  [1:0]   fmt_i,        // 00 S，01 D，10 H
  input  wire  [2:0]   rm_i,         // 已解析的舍入模式
  input  wire  [4:0]   idx_i,        // fli 的常数下标 (rs1 字段)
  input  wire  [63:0]  a_i,
  input  wire  [63:0]  b_i,
  output logic [63:0]  res_o,
  output logic [4:0]   flags_o       // {NV, DZ, OF, UF, NX}
);
  // 单精度拆箱
  wire [31:0] as = (&a_i[63:32]) ? a_i[31:0] : 32'h7FC0_0000;
  wire [31:0] bs = (&b_i[63:32]) ? b_i[31:0] : 32'h7FC0_0000;
  wire exact = (op_i == FMISC_ROUNDNX);

  logic [31:0] s_minm, s_maxm, s_rnd;
  logic        s_rnv, s_rnx, s_lt, s_le, s_nvq;
  logic [63:0] d_minm, d_maxm, d_rnd;
  logic        d_rnv, d_rnx, d_lt, d_le, d_nvq;
  o6_zfa_f #(.EW(8),  .MW(23)) u_s (.a_i(as),  .b_i(bs),  .rm_i, .exact_i(exact),
    .minm_o(s_minm), .maxm_o(s_maxm), .rnd_o(s_rnd), .rnd_nv_o(s_rnv), .rnd_nx_o(s_rnx), .lt_o(s_lt), .le_o(s_le), .nv_q_o(s_nvq));
  o6_zfa_f #(.EW(11), .MW(52)) u_d (.a_i(a_i), .b_i(b_i), .rm_i, .exact_i(exact),
    .minm_o(d_minm), .maxm_o(d_maxm), .rnd_o(d_rnd), .rnd_nv_o(d_rnv), .rnd_nx_o(d_rnx), .lt_o(d_lt), .le_o(d_le), .nv_q_o(d_nvq));

  // fli 常数表 (与 Spike 的 fli_{h,s,d}.h 逐项相同)
  function automatic logic [63:0] fli_d(input logic [4:0] k);
    logic [63:0] t [32];
    t = '{64'hbff0000000000000, 64'h0010000000000000, 64'h3ef0000000000000, 64'h3f00000000000000,
          64'h3f70000000000000, 64'h3f80000000000000, 64'h3fb0000000000000, 64'h3fc0000000000000,
          64'h3fd0000000000000, 64'h3fd4000000000000, 64'h3fd8000000000000, 64'h3fdc000000000000,
          64'h3fe0000000000000, 64'h3fe4000000000000, 64'h3fe8000000000000, 64'h3fec000000000000,
          64'h3ff0000000000000, 64'h3ff4000000000000, 64'h3ff8000000000000, 64'h3ffc000000000000,
          64'h4000000000000000, 64'h4004000000000000, 64'h4008000000000000, 64'h4010000000000000,
          64'h4020000000000000, 64'h4030000000000000, 64'h4060000000000000, 64'h4070000000000000,
          64'h40e0000000000000, 64'h40f0000000000000, 64'h7ff0000000000000, 64'h7ff8000000000000};
    return t[k];
  endfunction
  function automatic logic [31:0] fli_s(input logic [4:0] k);
    logic [31:0] t [32];
    t = '{32'hbf800000, 32'h00800000, 32'h37800000, 32'h38000000, 32'h3b800000, 32'h3c000000, 32'h3d800000, 32'h3e000000,
          32'h3e800000, 32'h3ea00000, 32'h3ec00000, 32'h3ee00000, 32'h3f000000, 32'h3f200000, 32'h3f400000, 32'h3f600000,
          32'h3f800000, 32'h3fa00000, 32'h3fc00000, 32'h3fe00000, 32'h40000000, 32'h40200000, 32'h40400000, 32'h40800000,
          32'h41000000, 32'h41800000, 32'h43000000, 32'h43800000, 32'h47000000, 32'h47800000, 32'h7f800000, 32'h7fc00000};
    return t[k];
  endfunction
  function automatic logic [15:0] fli_h(input logic [4:0] k);
    logic [15:0] t [32];
    t = '{16'hbc00, 16'h0400, 16'h0100, 16'h0200, 16'h1c00, 16'h2000, 16'h2c00, 16'h3000,
          16'h3400, 16'h3500, 16'h3600, 16'h3700, 16'h3800, 16'h3900, 16'h3a00, 16'h3b00,
          16'h3c00, 16'h3d00, 16'h3e00, 16'h3f00, 16'h4000, 16'h4100, 16'h4200, 16'h4400,
          16'h4800, 16'h4c00, 16'h5800, 16'h5c00, 16'h7800, 16'h7c00, 16'h7c00, 16'h7e00};
    return t[k];
  endfunction

  // fcvtmod.w.d (与 Spike fcvtmod_w_d.h 逐句对应)
  logic [63:0] cm_res;
  logic        cm_nv, cm_nx;
  always_comb begin
    logic        sg;
    logic [10:0] e;
    logic [63:0] m, v;
    int          te, sh;
    sg = a_i[63]; e = a_i[62:52];
    cm_nv = 1'b0; cm_nx = 1'b0; v = '0;
    if (e == 11'd0) begin
      cm_nx = (a_i[51:0] != '0);
    end else if (e == 11'h7FF) begin
      cm_nv = 1'b1;
    end else begin
      te = int'(e) - 1023;
      sh = te - 52;
      m  = {11'b0, 1'b1, a_i[51:0]};
      if (sh >= 64)       v = '0;
      else if (sh >= 0)   v = m << sh;
      else if (sh > -64) begin
        cm_nx = (m & ((64'd1 << (-sh)) - 64'd1)) != '0;
        v = m >> (-sh);
      end else begin
        v = '0; cm_nx = 1'b1;
      end
      if (te > 31 || v > (sg ? 64'h8000_0000 : 64'h7FFF_FFFF)) begin cm_nv = 1'b1; cm_nx = 1'b0; end
      if (sg) v = -v;
    end
    cm_res = {{32{v[31]}}, v[31:0]};
  end

  always_comb begin
    logic dbl;
    dbl = (fmt_i == 2'b01);
    res_o = '0; flags_o = '0;
    if (f2i_i) begin
      unique case (op_i)
        F2I_CVTMOD: begin res_o = cm_res; flags_o = {cm_nv, 3'b000, cm_nx}; end
        F2I_LEQ:    begin res_o = {63'b0, dbl ? d_le : s_le}; flags_o = {dbl ? d_nvq : s_nvq, 4'b0}; end
        default:    begin res_o = {63'b0, dbl ? d_lt : s_lt}; flags_o = {dbl ? d_nvq : s_nvq, 4'b0}; end   // F2I_LTQ
      endcase
    end else begin
      unique case (op_i)
        FMISC_FLI:  res_o = (fmt_i == 2'b10) ? {48'hFFFF_FFFF_FFFF, fli_h(idx_i)} :
                            dbl ? fli_d(idx_i) : {32'hFFFF_FFFF, fli_s(idx_i)};
        FMISC_MINM: begin res_o = dbl ? d_minm : {32'hFFFF_FFFF, s_minm}; flags_o = {dbl ? d_nvq : s_nvq, 4'b0}; end
        FMISC_MAXM: begin res_o = dbl ? d_maxm : {32'hFFFF_FFFF, s_maxm}; flags_o = {dbl ? d_nvq : s_nvq, 4'b0}; end
        default: begin                                                           // FMISC_ROUND / ROUNDNX
          res_o   = dbl ? d_rnd : {32'hFFFF_FFFF, s_rnd};
          flags_o = {dbl ? d_rnv : s_rnv, 3'b000, dbl ? d_rnx : s_rnx};
        end
      endcase
    end
  end
endmodule

// 入口寄存 + o6_zfa + (D-1) 级延迟：延迟级只为与 U6 对齐，综合时交给重定时 (dc_blk_2n.tcl 的 *o6_zfa_pipe*)
// 把寄存器推进运算里。整块只有一拍组合时 fround.d / fcvtmod 约 218 ps (h1，2026-10-30)，超 6 GHz 一拍。
// 入口每拍都采 (不带使能，重定时只碰普通 D 触发器)：结果只在 o6_fpu_exu 的 zq_v 那拍被取用
module o6_zfa_pipe #(parameter int D = 5) (
  input  wire          clk_i,
  input  wire  [4:0]   op_i,
  input  wire          f2i_i,
  input  wire  [1:0]   fmt_i,
  input  wire  [2:0]   rm_i,
  input  wire  [4:0]   idx_i,
  input  wire  [63:0]  a_i,
  input  wire  [63:0]  b_i,
  output logic [63:0]  res_o,        // 入口后第 D 拍
  output logic [4:0]   flags_o
);
  logic [4:0]  op, idx;
  logic        f2i;
  logic [1:0]  fmt;
  logic [2:0]  rm;
  logic [63:0] a, b, r;
  logic [4:0]  fl;
  logic [63:0] rq [1:D-1];
  logic [4:0]  fq [1:D-1];
  always_ff @(posedge clk_i) begin
    op <= op_i; f2i <= f2i_i; fmt <= fmt_i; rm <= rm_i; idx <= idx_i; a <= a_i; b <= b_i;
    rq[1] <= r; fq[1] <= fl;
    for (int k = 2; k < D; k++) begin rq[k] <= rq[k-1]; fq[k] <= fq[k-1]; end
  end
  o6_zfa u_zfa (.op_i(op), .f2i_i(f2i), .fmt_i(fmt), .rm_i(rm), .idx_i(idx), .a_i(a), .b_i(b), .res_o(r), .flags_o(fl));
  assign res_o   = rq[D-1];
  assign flags_o = fq[D-1];
endmodule
`default_nettype wire
