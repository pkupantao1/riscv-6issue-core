//==============================================================================
// o6_mul.sv - 64x64 乘法，EX1..EX4 四拍 (2026-10-25 由三拍加到四拍，6 GHz)
//
// 时序：EX1 输入操作数 -> 乘积组合逻辑 -> R1 -> R2 -> R3 -> EX4 组合选择输出 (进 wbreg)。
// 三拍时 EX1 前面还有模块外的旁路选择 (重定时推不过去)，2nm 重定时后中间一级仍超 57 ps。
// 第 1 级寄存器 (v1 / u1) 引出给唤醒与写回槽预约 (s1_v_o / s1_u_o)。
// 手工切分 (2026-10-25)：重定时在 2nm 上只切出 2 段 (第 1→2 段 204–206 ps)，改为显式分级：
//   EX1：旁路后的操作数寄存 (a1 / b1)；
//   EX2：b 按 SEGW 位 (缺省 17，`O6_MUL_SEGW` 可改) 切 NSEG 段，NSEG 个 65×(SEGW+1) 小乘法并行，
//        段积以进位保存形式 (两个向量) 寄存 (m2s / m2c)；
//   EX3：2·NSEG 个向量 CSA 压成 2 个 (DW02_tree)，寄存 (p3s / p3c)；
//   EX4：128 位进位传播加法，按 op 选高 / 低 64 位。
// (2026-10-25 b180u39 实测 EX1 106 / EX2 172 / EX3 158 / EX4 75 ps：两头空、中间两级各带一个进位传播加法器；
//  改成进位保存后两个加法器合成一个，挪到余量 81 ps 的 EX4)
// 进位保存的两个向量只有"和"有意义 (模 2^OW)，不能各自做符号扩展：第 c 段输出宽 OW = max(128 - SEGW·c, 65 + SEGW + 3)，
// 左移 SEGW·c 后模 2^128 仍精确，最后只取低 128 位 (MUL 用低 64，MULH* 用高 64)。
// 综合 (O6_ASIC) 用 DesignWare DW02_multp / DW02_tree；仿真 / FPGA 用行为模型 (out0 = 和，out1 = 0)，
// +define+O6_DW_SIM 时仿真也编 DW 的仿真模型核对。b 的低 3 段按无符号、最高段按有符号，与 65×65 有符号乘逐位相同 (断言)
// 乘法器写成一个 65x65 有符号乘，后面跟两级寄存器；综合时对本模块打开
// register retiming (DC: set_optimize_registers / Vivado: retiming)，把两级寄存器
// 推进乘法阵列里。不手工拆部分积，便于 DesignWare / DSP48 推断。
//==============================================================================
`default_nettype none
`ifdef O6_ASIC
`define O6_MUL_DW
`elsif O6_DW_SIM
`define O6_MUL_DW
`endif
module o6_mul
  import o6_pkg::*;
(
  input  wire              clk_i,
  input  wire              rst_ni,
  input  wire              flush_i,
  input  var br_update_t   br_upd_i,
  input  wire              valid_i,     // EX1
  input  var uop_t         uop_i,
  input  wire  [63:0]      a_i,
  input  wire  [63:0]      b_i,
  output wire              valid_o,     // EX4 (组合)
  output uop_t             uop_o,
  output logic [63:0]      y_o,
  output logic             s1_v_o,      // 第 1 级寄存器 (= EX2)：有效 (未计本拍分支杀)
  output uop_t             s1_u_o
);
  wire a_sgn = (uop_i.op == MUL_MULH) || (uop_i.op == MUL_MULHSU);
  wire b_sgn = (uop_i.op == MUL_MULH);
  wire signed [64:0]  a_x = {a_sgn & a_i[63], a_i};
  wire signed [64:0]  b_x = {b_sgn & b_i[63], b_i};
  logic signed [64:0] a1, b1;
  // EX2：b 按 SEGW 位切 NSEG 段，段积 (进位保存)，移位到位后取低 128 位寄存。
  // 段越窄，每段 Booth 部分积越少、EX2 的 CSA 级数越少，压缩挪到 EX3 的树 (2026-10-28：SEGW 17 时 EX2 ~171 ps @SP155)
`ifdef O6_MUL_SEGW
  localparam int SEGW = `O6_MUL_SEGW;
`else
  localparam int SEGW = 17;
`endif
  localparam int NSEG = (65 + SEGW - 1) / SEGW;
  localparam int BW   = SEGW + 1;                     // 段乘数宽 (低段补 0 当无符号，最高段符号扩展)
  localparam int TW   = 65 - SEGW * (NSEG - 1);       // 最高段的有效位数
  logic [127:0] m2s [NSEG], m2c [NSEG];
  for (genvar c = 0; c < NSEG; c++) begin : g_pp
    localparam int OW = (128 - SEGW * c > 65 + BW + 2) ? 128 - SEGW * c : 65 + BW + 2;
    wire [BW-1:0] bc;
    wire [OW-1:0] o0, o1;
    if (c == NSEG - 1) begin : g_hi
      assign bc = {{(BW - TW){b1[64]}}, b1[64 -: TW]};
    end else begin : g_lo
      assign bc = {1'b0, b1[SEGW * c +: SEGW]};
    end
`ifdef O6_MUL_DW
    DW02_multp #(.a_width(65), .b_width(BW), .out_width(OW)) u_pp (
      .a(a1), .b(bc), .tc(1'b1), .out0(o0), .out1(o1));
`else
    assign o0 = $signed(a1) * $signed(bc);      // 上下文宽 OW：符号扩展后相乘，模 2^OW
    assign o1 = '0;
`endif
    always_ff @(posedge clk_i) begin
      m2s[c] <= 128'(o0) << (SEGW * c);
      m2c[c] <= 128'(o1) << (SEGW * c);
    end
  end
  // EX3：2·NSEG 个向量压成 2 个
  logic [127:0] t0, t1;
  logic [2*NSEG*128-1:0] m2all;
  always_comb
    for (int c = 0; c < NSEG; c++) begin
      m2all[(2*c)*128 +: 128]   = m2s[c];
      m2all[(2*c+1)*128 +: 128] = m2c[c];
    end
`ifdef O6_MUL_DW
  DW02_tree #(.num_inputs(2 * NSEG), .input_width(128)) u_tree (
    .INPUT(m2all), .OUT0(t0), .OUT1(t1));
`else
  always_comb begin
    logic [127:0] s;
    s = '0;
    for (int k = 0; k < 2 * NSEG; k++) s = s + m2all[k*128 +: 128];
    t0 = s;
  end
  assign t1 = '0;
`endif
  logic [127:0] p3s, p3c;
  // EX4：进位传播
  wire  [127:0] p3 = p3s + p3c;

  logic          v1, v2, v3;
  uop_t          u1, u2, u3;

  // Zbb 计数 (clz / ctz / cpop，W 形式只看低 32 位)：EX2 用寄存的 a1 算，随流水寄存到 EX4 输出
  function automatic logic [6:0] clz64(input logic [63:0] x);
    logic [6:0] n;
    n = 7'd64;
    for (int k = 0; k < 64; k++) if (x[k]) n = 7'(63 - k);
    return n;
  endfunction
  function automatic logic [6:0] ctz64(input logic [63:0] x);
    logic [6:0] n;
    n = 7'd64;
    for (int k = 63; k >= 0; k--) if (x[k]) n = 7'(k);
    return n;
  endfunction
  function automatic logic [6:0] pop64(input logic [63:0] x);
    logic [6:0] n;
    n = '0;
    for (int k = 0; k < 64; k++) n = n + 7'(x[k]);
    return n;
  endfunction
  logic [6:0] cnt_c, cnt2, cnt3;
  always_comb begin
    logic [63:0] x;
    logic [31:0] aw;
    aw = a1[31:0];
    unique case (u1.op)
      MUL_CLZ: begin x = u1.is_w ? {aw, 32'hFFFF_FFFF} : a1[63:0]; cnt_c = clz64(x); end   // clzw：低 32 位放到高位，下面补 1 截住
      MUL_CTZ: begin x = u1.is_w ? {32'hFFFF_FFFF, aw} : a1[63:0]; cnt_c = ctz64(x); end   // ctzw
      default: begin x = u1.is_w ? {32'b0, aw} : a1[63:0];         cnt_c = pop64(x); end
    endcase
  end
  assign s1_v_o = v1;
  assign s1_u_o = u1;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
    end else if (flush_i) begin
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
    end else begin
      v1 <= valid_i && !br_killed(uop_i.br_mask, br_upd_i);
      v2 <= v1      && !br_killed(u1.br_mask, br_upd_i);
      v3 <= v2      && !br_killed(u2.br_mask, br_upd_i);
    end
  end

  always_ff @(posedge clk_i) begin
    uop_t n1, n2, n3;
    n1 = uop_i; n1.br_mask = uop_i.br_mask & ~br_upd_i.resolve_mask;
    n2 = u1;    n2.br_mask = u1.br_mask    & ~br_upd_i.resolve_mask;
    n3 = u2;    n3.br_mask = u2.br_mask    & ~br_upd_i.resolve_mask;
    u1 <= n1;
    u2 <= n2;
    u3 <= n3;
    a1 <= a_x;
    b1 <= b_x;
    p3s <= t0;
    p3c <= t1;
    cnt2 <= cnt_c;
    cnt3 <= cnt2;
  end

  // EX4：本拍到达的 br_update 也要考虑
  assign valid_o = v3 && !br_killed(u3.br_mask, br_upd_i);
  always_comb begin
    uop_o = u3;
    uop_o.br_mask = u3.br_mask & ~br_upd_i.resolve_mask;
    unique case (u3.op)
      MUL_MUL:                    y_o = u3.is_w ? {{32{p3[31]}}, p3[31:0]} : p3[63:0];
      MUL_CLZ, MUL_CTZ, MUL_CPOP: y_o = {57'b0, cnt3};
      default:                    y_o = p3[127:64];
    endcase
  end
`ifndef SYNTHESIS
  // 切分 + 进位保存后的积与整体乘一致 (EX2 时的操作数 a1 / b1 -> EX3 的 t0 + t1 -> EX4 的 p3)
  logic signed [129:0] ref2, ref3;
  always_ff @(posedge clk_i) begin
    ref2 <= a1 * b1;
    ref3 <= ref2;
    if (rst_ni && v2) assert (t0 + t1 == ref2[127:0]) else $error("mul: 切分乘积与整体乘不一致 (EX3)");
    if (rst_ni && v3) assert (p3 == ref3[127:0]) else $error("mul: 切分乘积与整体乘不一致 (EX4)");
  end
`endif
endmodule
`undef O6_MUL_DW
