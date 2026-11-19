//==============================================================================
// o6_vex.sv - 向量执行器 VL (V1，docs/11_vector_uarch.md §9h)
//
// 全流水、固定延迟 (6 GHz 目标)：每拍收一 "拍" = 一个 1024 位寄存器 (指令组里的第 k 个寄存器)，LAT 拍后出整寄存器结果。
//   一条指令不拆 uop：调用方 (VIQ / 发射) 按组内寄存器逐拍送进来，上一条最后一拍之后下一条紧接着送，执行器不停顿。
//   4 个执行器各做一条不同的指令 -> 每拍 4 × 1024 位。
// 输入 (第 k 拍)：编码 raw、SEW、本寄存器的体内元素数 nel (= min(vl - k × E, E)，E = 128 >> SEW)、
//   本寄存器各元素的掩码位 m (v0 已按 k 移好；不带掩码时调用方给全 1 或忽略)、vs2 / vs1 / vd 旧值、
//   标量源 xs (.vx / .vi 的 x[rs1]，.vf 的 f[rs1] 原值)、frm、标签 tag。
// 输出：体内的活动元素写结果；被掩掉的与尾部保持 vd 旧值 (与 o6_vecu 原执行流水一致：不按 agnostic 置 1)。
// 第一步 (V1.1) 支持：
//   整数 (OPIVV / OPIVX / OPIVI)：vfast_op 那几类 (vadd vsub vrsub vmin(u) vmax(u) vand vor vxor vsll vsrl vsra vmerge / vmv.v)，
//     SEW 8..64，按 O1 的 fx_calc 算整寄存器，再补流水级到 LAT；
//   浮点 (OPFVV / OPFVF)：vfadd vfsub vfrsub vfmul 与 8 种乘加 (vfmacc ... vfnmsub)，SEW 16 / 32 / 64 (SEW 16 按 IEEE FP16)。
//     16 条 64 位 fpnew 通道，向量模式 (每条一拍 4 × FP16 / 2 × FP32 / 1 × FP64)，ADDMUL 5 级 (同标量 FPU 的 6 GHz 配置)。
//     被掩掉 / 尾部的元素送 0 (不产生异常标志)，fflags = 各通道状态之或 (只来自活动元素)。
//   其余类别 (整数乘加、比较、加宽 / 收窄、饱和、浮点比较 / 转换 / 归约、点积 ...) 后续加入；vex_op() 给调用方判定能不能送进来
// V7 (2026-11-19 用户定)：分两种，都是 1024 位、延迟相同 (VIQ 按固定延迟唤醒)：
//   HP = 0 低精度合成单元：fpnew 只开 FP16 / BF16 (FP16ALT) / FP8，加整数通路 (fx_calc)；点积单元 o6_vdot 在外面同一个执行器里。
//   HP = 1 FP64 / FP32 合成单元：fpnew 只开 FP32 / FP64，没有整数通路 (VIQ 只把 SEW 32 / 64 的浮点运算发来)
//==============================================================================
`default_nettype none
module o6_vex
  import o6_pkg::*;
#(
  parameter int TW = 16,
  parameter bit HP = 1'b0
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 in_v_i,
  input  wire  [31:0]         in_raw_i,
  input  wire  [1:0]          in_sew_i,
  input  wire  [7:0]          in_nel_i,
  input  wire  [CL_B-1:0]     in_m_i,
  input  wire  [VLEN-1:0]     in_vs2_i,
  input  wire  [VLEN-1:0]     in_vs1_i,
  input  wire  [VLEN-1:0]     in_vd_i,
  input  wire  [63:0]         in_xs_i,
  input  wire  [2:0]          in_frm_i,
  input  wire  [TW-1:0]       in_tag_i,
  output logic                out_v_o,
  output logic [VLEN-1:0]     out_d_o,
  output logic [TW-1:0]       out_tag_o,
  output logic [4:0]          out_ff_o
);
  localparam int NFL = VLEN / 64;                 // fpnew 通道数
  localparam int LF  = 5;                         // fpnew ADDMUL 的流水级数 (PipeRegs)
  localparam int LAT = 1 + LF;                    // 输入寄存 1 + fpnew

  //--------------------------------------------------------------------------
  // 输入寄存 (E0)
  //--------------------------------------------------------------------------
  logic            e_v;
  logic [31:0]     e_raw;
  logic [1:0]      e_sew;
  logic [7:0]      e_nel;
  logic [CL_B-1:0] e_m;
  logic [VLEN-1:0] e_vs2, e_vs1, e_vd;
  logic [63:0]     e_xs;
  logic [2:0]      e_frm;
  logic [TW-1:0]   e_tag;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) e_v <= 1'b0;
    else         e_v <= in_v_i;
  always_ff @(posedge clk_i)
    if (in_v_i) begin
      e_raw <= in_raw_i; e_sew <= in_sew_i; e_nel <= in_nel_i; e_m <= in_m_i;
      e_vs2 <= in_vs2_i; e_vs1 <= in_vs1_i; e_vd <= in_vd_i; e_xs <= in_xs_i; e_frm <= in_frm_i; e_tag <= in_tag_i;
    end

  wire e_fp = (e_raw[14:12] == 3'b001) || (e_raw[14:12] == 3'b101);   // OPFVV / OPFVF

  //--------------------------------------------------------------------------
  // 整数：O1 的 fx_calc (整寄存器一拍；综合时由流水级重定时)
  //--------------------------------------------------------------------------
  function automatic logic [63:0] fx_el(input logic [5:0] f6, input logic [63:0] a, input logic [63:0] b, input int w);
    logic [63:0] m, sa, sb, r;
    int          sh;
    m  = (w == 64) ? '1 : ((64'd1 << w) - 64'd1);
    sa = $signed(a << (64 - w)) >>> (64 - w);
    sb = $signed(b << (64 - w)) >>> (64 - w);
    sh = int'(b & 64'(w - 1));
    unique case (f6)
      6'b000000: r = a + b;
      6'b000010: r = a - b;
      6'b000011: r = b - a;
      6'b000100: r = (a < b) ? a : b;
      6'b000101: r = ($signed(sa) < $signed(sb)) ? a : b;
      6'b000110: r = (a > b) ? a : b;
      6'b000111: r = ($signed(sa) > $signed(sb)) ? a : b;
      6'b001001: r = a & b;
      6'b001010: r = a | b;
      6'b001011: r = a ^ b;
      6'b100101: r = a << sh;
      6'b101000: r = a >> sh;
      6'b101001: r = $signed(sa) >>> sh;
      default:   r = b;
    endcase
    return r & m;
  endfunction
  function automatic logic [VLEN-1:0] fx_calc(input logic [31:0] ins, input logic [63:0] xs,
                                              input logic [VLEN-1:0] v2, input logic [VLEN-1:0] v1,
                                              input logic [CL_B-1:0] v0, input logic [VLEN-1:0] vo,
                                              input logic [1:0] sw, input logic [7:0] nel);
    logic [VLEN-1:0] r;
    logic [5:0]      f6;
    logic [2:0]      f3;
    logic [63:0]     bs;
    logic            shf, mrg;
    int              w, ne;
    f6  = ins[31:26]; f3 = ins[14:12];
    w   = 8 << sw; ne = VLEN / w;
    shf = (f6 == 6'b100101) || (f6 == 6'b101000) || (f6 == 6'b101001);
    mrg = (f6 == 6'b010111);
    bs  = (f3 == 3'b100) ? xs : shf ? 64'(ins[19:15]) : {{59{ins[19]}}, ins[19:15]};
    r   = vo;
    for (int e = 0; e < VLEN / 8; e++)
      if (e < ne && e < int'(nel)) begin
        logic [63:0] a, b, m, x;
        logic        act;
        m = (w == 64) ? '1 : ((64'd1 << w) - 64'd1);
        unique case (sw)
          2'd0:    begin a = 64'(v2[8*e +: 8]);   b = 64'(v1[8*e +: 8]);   end
          2'd1:    begin a = 64'(v2[16*e +: 16]); b = 64'(v1[16*e +: 16]); end
          2'd2:    begin a = 64'(v2[32*e +: 32]); b = 64'(v1[32*e +: 32]); end
          default: begin a = v2[64*e +: 64];      b = v1[64*e +: 64];      end
        endcase
        if (f3 != 3'b000) b = bs & m;
        act = ins[25] || v0[e] || mrg;
        x = (mrg && !ins[25] && !v0[e]) ? a : fx_el(f6, a, b, w);
        if (act)
          unique case (sw)
            2'd0:    r[8*e +: 8]   = x[7:0];
            2'd1:    r[16*e +: 16] = x[15:0];
            2'd2:    r[32*e +: 32] = x[31:0];
            default: r[64*e +: 64] = x;
          endcase
      end
    return r;
  endfunction

  logic [VLEN-1:0] ir_out;              // 整数结果 (送入后 LAT 拍)
  if (!HP) begin : g_int
    logic [VLEN-1:0] ir_q [LAT];        // ir_q[0] = E0 之后一拍
    always_ff @(posedge clk_i) begin
      ir_q[0] <= fx_calc(e_raw, e_xs, e_vs2, e_vs1, e_m, e_vd, e_sew, e_nel);
      for (int s = 1; s < LAT; s++) ir_q[s] <= ir_q[s-1];
    end
    assign ir_out = ir_q[LAT-1];
  end else begin : g_noint
    assign ir_out = '0;                 // FP64 / FP32 单元没有整数通路
  end

  //--------------------------------------------------------------------------
  // 浮点：16 条 fpnew 通道 (向量模式)
  //--------------------------------------------------------------------------
  localparam fpnew_pkg::fpu_features_t FEAT = '{
    Width:         64,
    EnableVectors: 1'b1,
    EnableNanBox:  1'b1,
    // [0:4] = FP32 FP64 FP16 FP8 FP16ALT。V7：低精度单元 FP16 / FP8 / BF16，FP64 / FP32 单元 FP32 / FP64
    FpFmtMask:     HP ? 5'b11000 : 5'b00111,
    IntFmtMask:    4'b0000
  };
  localparam fpnew_pkg::fmt_unit_types_t UT_HP =
    '{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED};
  localparam fpnew_pkg::fmt_unit_types_t UT_LP =
    '{fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED};
  localparam fpnew_pkg::fpu_implementation_t IMPL = '{
    PipeRegs:   '{'{LF, LF, LF, LF, LF}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}},
    UnitTypes:  '{HP ? UT_HP : UT_LP,
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  // fpnew 每条 64 位通道的子通道数 (SIMD 掩码位数) = 64 / 最窄格式宽度
  localparam int NML = HP ? 2 : 8;
  function automatic fpnew_pkg::fp_format_e ffmt(input logic [1:0] w);
    unique case (w)
      2'd0:    return fpnew_pkg::FP8;
      2'd1:    return fpnew_pkg::FP16;
      2'd2:    return fpnew_pkg::FP32;
      default: return fpnew_pkg::FP64;
    endcase
  endfunction
  // .vf 的 f[rs1]：窄格式要正确 NaN-boxing，否则按规范当规范 NaN
  function automatic logic [63:0] funbox(input logic [63:0] v, input logic [1:0] w);
    unique case (w)
      2'd1:    return (v[63:16] == '1) ? {48'b0, v[15:0]} : 64'h7E00;
      2'd2:    return (v[63:32] == '1) ? {32'b0, v[31:0]} : 64'h7FC0_0000;
      default: return v;
    endcase
  endfunction

  // 各元素是否活动 (按字节：元素宽 8 << sew)
  logic [CL_B-1:0] e_act;              // 按元素号
  logic [VLEN/8-1:0] e_actb;           // 按字节
  always_comb begin
    int ew;
    ew = 1 << e_sew;                     // 字节
    for (int e = 0; e < CL_B; e++) e_act[e] = (e < int'(e_nel)) && (e_raw[25] || e_m[e]);
    for (int b = 0; b < VLEN / 8; b++) e_actb[b] = e_act[b >> e_sew];
  end

  typedef struct packed {
    fpnew_pkg::operation_e  op;
    logic                   mod;
    logic [2:0][63:0]       opnd;
  } freq_t;
  freq_t fq [NFL];
  always_comb begin
    logic [63:0] sc;
    sc = funbox(e_xs, e_sew);
    for (int l = 0; l < NFL; l++) begin
      logic [63:0] a, b, c, bm;
      // 本通道里不活动的元素字节送 0
      for (int b = 0; b < 8; b++) bm[8*b +: 8] = {8{e_actb[8*l + b]}};
      a = e_vs2[64*l +: 64] & bm;
      c = e_vd[64*l +: 64] & bm;
      if (e_raw[14:12] == 3'b101) begin
        unique case (e_sew)                                // .vf：标量复制到通道的各元素
          2'd1:    b = {4{sc[15:0]}};
          2'd2:    b = {2{sc[31:0]}};
          default: b = sc;
        endcase
      end else b = e_vs1[64*l +: 64];
      b = b & bm;
      fq[l].op = fpnew_pkg::ADD; fq[l].mod = 1'b0; fq[l].opnd = {b, a, 64'd0};
      unique case (e_raw[31:26])
        6'b000000: ;                                                                       // vfadd
        6'b000010: fq[l].mod = 1'b1;                                                       // vfsub
        6'b100111: begin fq[l].mod = 1'b1; fq[l].opnd = {a, b, 64'd0}; end                // vfrsub (.vf)
        6'b100100: begin fq[l].op = fpnew_pkg::MUL; fq[l].opnd = {64'd0, b, a}; end      // vfmul
        6'b101100: begin fq[l].op = fpnew_pkg::FMADD;  fq[l].opnd = {c, a, b}; end                    // vfmacc
        6'b101101: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].mod = 1'b1; fq[l].opnd = {c, a, b}; end // vfnmacc
        6'b101110: begin fq[l].op = fpnew_pkg::FMADD;  fq[l].mod = 1'b1; fq[l].opnd = {c, a, b}; end // vfmsac
        6'b101111: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].opnd = {c, a, b}; end                    // vfnmsac
        6'b101000: begin fq[l].op = fpnew_pkg::FMADD;  fq[l].opnd = {a, b, c}; end                    // vfmadd
        6'b101001: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].mod = 1'b1; fq[l].opnd = {a, b, c}; end // vfnmadd
        6'b101010: begin fq[l].op = fpnew_pkg::FMADD;  fq[l].mod = 1'b1; fq[l].opnd = {a, b, c}; end // vfmsub
        6'b101011: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].opnd = {a, b, c}; end                    // vfnmsub
        default: ;
      endcase
    end
  end

  logic [63:0]        fo_r  [NFL];
  fpnew_pkg::status_t fo_st [NFL];
  logic [NFL-1:0]     fo_v;
  // 各子通道 (16 位一个) 的状态掩码：只有本格式用到、且对应元素活动的子通道的状态算进 fflags。
  //   (fpnew 向量模式下所有子通道都会启动；只支持更窄格式的子通道对宽格式运算的状态是无意义的值，
  //   EnableSIMDMask = 0 时 simd_mask_i 被当成全 1，会把它们或进来 —— vfadd 曾因此多报 UF)
  //   子通道 j 是本格式的第 j 个元素 (字节 j × 元素宽)，本格式只有 8 / 元素宽 个
  logic [NML-1:0] fm [NFL];
  always_comb
    for (int l = 0; l < NFL; l++)
      for (int j = 0; j < NML; j++) begin
        int ew;
        ew = 1 << e_sew;
        fm[l][j] = (j < 8 / ew) ? e_actb[8*l + j*ew] : 1'b0;
      end
  for (genvar l = 0; l < NFL; l++) begin : g_fl
    fpnew_top #(
      .Features       (FEAT),
      .Implementation (IMPL),
      .DivSqrtSel     (fpnew_pkg::THMULTI),
      .TagType        (logic),
      .EnableSIMDMask (1)
    ) u_fpnew (
      .clk_i, .rst_ni,
      .operands_i    (fq[l].opnd),
      .rnd_mode_i    (fpnew_pkg::roundmode_e'(e_frm)),
      .op_i          (fq[l].op),
      .op_mod_i      (fq[l].mod),
      .src_fmt_i     (ffmt(e_sew)),
      .dst_fmt_i     (ffmt(e_sew)),
      .int_fmt_i     (fpnew_pkg::INT64),
      .vectorial_op_i(1'b1),
      .tag_i         (1'b0),
      .simd_mask_i   (fm[l]),
      .in_valid_i    (e_v && e_fp),
      .in_ready_o    (),
      .flush_i       (1'b0),
      .result_o      (fo_r[l]),
      .status_o      (fo_st[l]),
      .tag_o         (),
      .out_valid_o   (fo_v[l]),
      .out_ready_i   (1'b1),
      .busy_o        (),
      .early_valid_o ()
    );
  end

  //--------------------------------------------------------------------------
  // 随拍走的信息 (与 fpnew 同步)：有效、浮点、标签、vd 旧值、各字节活动
  //--------------------------------------------------------------------------
  logic [LAT-1:0]    p_v, p_fp;
  logic [TW-1:0]     p_tag  [LAT];
  logic [VLEN-1:0]   p_vd   [LAT];
  logic [VLEN/8-1:0] p_act  [LAT];
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) p_v <= '0;
    else         p_v <= {p_v[LAT-2:0], e_v};
  always_ff @(posedge clk_i) begin
    p_fp[0] <= e_fp; p_tag[0] <= e_tag; p_vd[0] <= e_vd; p_act[0] <= e_actb;
    for (int s = 1; s < LAT; s++) begin
      p_fp[s] <= p_fp[s-1]; p_tag[s] <= p_tag[s-1]; p_vd[s] <= p_vd[s-1]; p_act[s] <= p_act[s-1];
    end
  end
  // fpnew 结果在 e_v 之后 LF 拍出来，即 p 的第 LF-1 级同拍 (组合)；与整数结果对齐到 LAT：再寄存一拍
  logic [VLEN-1:0] fr;
  logic [4:0]      ff;
  always_comb begin
    ff = '0;
    for (int l = 0; l < NFL; l++) begin
      fr[64*l +: 64] = fo_r[l];
      ff = ff | fo_st[l];
    end
  end
  logic [VLEN-1:0] fr_q;
  logic [4:0]      ff_q;
  always_ff @(posedge clk_i) begin fr_q <= fr; ff_q <= ff; end

  always_comb begin
    out_v_o   = p_v[LAT-1];
    out_tag_o = p_tag[LAT-1];
    out_ff_o  = p_fp[LAT-1] ? ff_q : 5'd0;
    if (p_fp[LAT-1])
      for (int b = 0; b < VLEN / 8; b++)
        out_d_o[8*b +: 8] = p_act[LAT-1][b] ? fr_q[8*b +: 8] : p_vd[LAT-1][8*b +: 8];
    else
      out_d_o = ir_out;
  end

`ifndef SYNTHESIS
  // V7：送来的运算与单元的种类相符 (HP 只收 SEW 32 / 64 的浮点；低精度单元不收)
  always_ff @(posedge clk_i)
    if (rst_ni && e_v)
      assert ((e_fp && e_sew[1]) == HP)
        else $error("o6_vex: %s 单元收到 %h (SEW %0d，浮点 %b)", HP ? "FP64/FP32" : "低精度", e_raw, 8 << e_sew, e_fp);
  // fpnew 结果与随拍信息对齐：浮点拍进来后正好 LF 拍各通道出结果
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      assert (fo_v == {NFL{p_v[LF-1] && p_fp[LF-1]}})
        else $error("o6_vex: fpnew 输出 %b 与随拍有效 %b 不同步 (LF = %0d)", fo_v, p_v[LF-1] && p_fp[LF-1], LF);
    end
`endif
endmodule
