//==============================================================================
// o6_pmp.sv - PMP 访问检查 (组合；8 项；粒度 2^(PMP_G+2) = 64 字节，见 o6_pkg::PMP_G)
//
// 规则 (Privileged 1.12 §3.7)：
//   - 序号最低的匹配项决定结果；
//   - M 态：只有匹配到锁定 (L=1) 的项才检查权限；无匹配则允许；
//   - S/U 态：匹配项按 R/W/X 检查；无匹配则拒绝 (本实现有 PMP 项)。
// 地址只检查访问起始字节：粒度 64 B 时这是精确的 —— 自然对齐的标量访存 (<= 8 B)、向量逐行访存 (64 B 对齐)、
// 取指块 (不跨 64 B 行) 都不会跨越粒度单元 (#34 之前粒度为 4 B，取指块内的区域边界会被漏检)。
// 粒度规则 (Privileged 1.12 §3.7.1)：TOR 的上下界按粒度对齐 (pmpaddr 低 G 位视为 0)；NAPOT 最小 2^(G+2) 字节
// (pmpaddr 低 G-1 位视为 1)；NA4 不可选 (CSR 写入时已转成 NAPOT)。
//
// 分两部分 (2026-10-27，6 GHz)：o6_pmp_dec 从 pmpcfg / pmpaddr 算出各项的区间 (TOR 上下界、NAPOT 基址与掩码)，
// o6_core 寄存一拍后分给取指 / LSU / PTW 的 o6_pmp，检查只剩比较。原来每次检查现场算 NAPOT 掩码
// (na ^ (na + 1)，54 位进位链)，LSU 里 pmpaddr -> l2 excp 要 186 ps。
// 晚一拍生效是安全的：PMP CSR 写带 flush_after，提交后冲刷重取，新访存最早几拍之后才查到。
//==============================================================================
`default_nettype none
module o6_pmp_dec
  import o6_pkg::*;
#(
  parameter int N = 8
) (
  input  wire  [7:0]       cfg_i  [N],     // {L, 0, 0, A[1:0], X, W, R}
  input  wire  [53:0]      addr_i [N],     // pmpaddr (PA[55:2])
  output pmp_dec_t         dec_o  [N]
);
  localparam logic [53:0] TOR_MASK  = ~((54'd1 << PMP_G) - 54'd1);     // TOR 地址低 G 位清 0
  localparam logic [53:0] NAPOT_ONE = (54'd1 << (PMP_G - 1)) - 54'd1;  // NAPOT 低 G-1 位置 1

  always_comb begin
    for (int i = 0; i < N; i++) begin
      logic [53:0] na;
      dec_o[i].lk  = cfg_i[i][7];
      dec_o[i].xwr = cfg_i[i][2:0];
      dec_o[i].tor = 1'b0; dec_o[i].nap = 1'b0;
      dec_o[i].lo  = '0;   dec_o[i].hi  = '0;
      unique case (cfg_i[i][4:3])
        2'b01: begin   // TOR
          dec_o[i].tor = 1'b1;
          dec_o[i].lo  = (i == 0) ? 54'd0 : (addr_i[i == 0 ? 0 : i-1] & TOR_MASK);
          dec_o[i].hi  = addr_i[i] & TOR_MASK;
        end
        2'b10: begin   // NA4：整个地址相等
          dec_o[i].nap = 1'b1;
          dec_o[i].lo  = addr_i[i];
          dec_o[i].hi  = '1;
        end
        2'b11: begin   // NAPOT：低位连续的 1 决定区域大小
          na = addr_i[i] | NAPOT_ONE;
          dec_o[i].nap = 1'b1;
          dec_o[i].lo  = na;
          dec_o[i].hi  = ~(na ^ (na + 54'd1));        // 尾部 1 及其上一位以下为 0
        end
        default: ;                                   // OFF
      endcase
    end
  end
endmodule

module o6_pmp
  import o6_pkg::*;
#(
  parameter int N = 8
) (
  input  pmp_dec_t         dec_i  [N],     // o6_pmp_dec 的结果 (已寄存)
  input  wire  [PA_W-1:0]  pa_i,
  input  wire  [1:0]       priv_i,         // 有效特权
  input  wire  [2:0]       acc_i,          // {X, W, R} 本次访问类型 (独热)
  output logic             ok_o
);
  wire [53:0] a = {{(56-PA_W){1'b0}}, pa_i[PA_W-1:2]};

  logic [N-1:0] hit;
  always_comb
    for (int i = 0; i < N; i++)
      hit[i] = (dec_i[i].tor && a >= dec_i[i].lo && a < dec_i[i].hi) ||
               (dec_i[i].nap && ((a ^ dec_i[i].lo) & dec_i[i].hi) == 54'd0);

  always_comb begin
    logic found;
    found = 1'b0;
    ok_o  = (priv_i == 2'b11);        // 无匹配：M 允许，S/U 拒绝
    for (int i = 0; i < N; i++) begin
      if (!found && hit[i]) begin
        found = 1'b1;
        if (priv_i == 2'b11 && !dec_i[i].lk) ok_o = 1'b1;
        else ok_o = (acc_i & dec_i[i].xwr) == acc_i;   // 所需权限都要有 (HLVX 是 R + X)
      end
    end
  end
endmodule
