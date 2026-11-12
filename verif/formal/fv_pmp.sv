// fv_pmp.sv - 形式证明 F2：o6_pmp 与按规范「地址区间」写的参考模型等价 (8 项，粒度 2^(PMP_G+2) = 64 B)
//   参考模型 (Privileged 1.12 §3.7)：
//     TOR   区间 [pmpaddr[i-1] << 2, pmpaddr[i] << 2)，两端按粒度对齐 (pmpaddr 低 G 位视为 0)；i = 0 时下界为 0
//     NAPOT pmpaddr 低 G-1 位视为 1；尾部连续 1 的个数 n -> 区间大小 2^(n+3) 字节，基址 = 按大小对齐
//     序号最低的匹配项决定；无匹配：M 允许、S/U 拒绝；匹配：M 态且未锁定 -> 允许，否则按 R/W/X
//   实现 (o6_pmp) 用「异或 + 掩码」判 NAPOT、对 pmpaddr 直接比较判 TOR —— 写法不同，等价才说明粒度 / 优先级 / M 态规则对
`default_nettype none
module fv_pmp
  import o6_pkg::*;
(
  input  wire              clk,
  input  wire  [7:0]       cfg  [8],
  input  wire  [53:0]      addr [8],
  input  wire  [PA_W-1:0]  pa,
  input  wire  [1:0]       priv,
  input  wire  [2:0]       acc
);
  logic ok_dut;
  pmp_dec_t dec [8];      // 译码 + 检查一起证 (o6_core 里两者之间只隔一级寄存器)
  o6_pmp_dec #(.N(8)) dut_dec (.cfg_i(cfg), .addr_i(addr), .dec_o(dec));
  o6_pmp #(.N(8)) dut (.dec_i(dec), .pa_i(pa), .priv_i(priv), .acc_i(acc), .ok_o(ok_dut));

  // ---- 参考模型 (按字节地址算区间)
  localparam int G = PMP_G;
  function automatic logic in_napot(logic [53:0] a, logic [55:0] pb);
    logic [53:0] na;
    int n;
    na = a | 54'((1 << (G - 1)) - 1);
    n = 54;
    for (int b = 53; b >= 0; b--) if (!na[b]) n = b;     // n = 尾部连续 1 的个数 (全 1 时 n = 54：覆盖全部)
    if (n >= 54) return 1'b1;
    // 区间大小 2^(n+3) 字节 = pmpaddr 粒度下 2^(n+1)：比较 pmpaddr 与地址的高位
    return (({2'b00, na} >> (n + 1)) == ({2'b00, pb[55:2]} >> (n + 1)));
  endfunction

  logic ok_ref;
  always_comb begin
    logic [55:0] pb, lo, hi;
    logic found;
    pb = {{(56-PA_W){1'b0}}, pa};
    found = 1'b0;
    ok_ref = (priv == 2'b11);
    for (int i = 0; i < 8; i++) begin
      logic m;
      m = 1'b0;
      unique case (cfg[i][4:3])
        2'b01: begin
          lo = (i == 0) ? 56'd0 : {addr[i == 0 ? 0 : i - 1] & ~54'((1 << G) - 1), 2'b00};
          hi = {addr[i] & ~54'((1 << G) - 1), 2'b00};
          m = (pb >= lo) && (pb < hi);
        end
        2'b11: m = in_napot(addr[i], pb);
        default: m = 1'b0;
      endcase
      if (!found && m) begin
        found = 1'b1;
        ok_ref = (priv == 2'b11 && !cfg[i][7]) ? 1'b1 : |(acc & cfg[i][2:0]);
      end
    end
  end

  // CSR 永不写入 NA4 (粒度 64 B 时转为 NAPOT)；访问类型独热；特权级合法
  for (genvar i = 0; i < 8; i++) begin : g_a
    asm_no_na4: assume property (@(posedge clk) cfg[i][4:3] != 2'b10);
  end
  asm_acc:  assume property (@(posedge clk) $onehot(acc));
  asm_priv: assume property (@(posedge clk) priv != 2'b10);

  ast_equiv: assert property (@(posedge clk) ok_dut == ok_ref);
  // 关键情形可达 (防止约束过严使证明空转)
  cov_tor_edge:   cover property (@(posedge clk) cfg[1][4:3] == 2'b01 && {addr[1][53:G], {G{1'b0}}, 2'b00} == {{(56-PA_W){1'b0}}, pa} + 56'd1);
  cov_napot_min:  cover property (@(posedge clk) cfg[0][4:3] == 2'b11 && addr[0][G-1:0] == '0 && ok_dut != (priv == 2'b11));
  cov_m_locked:   cover property (@(posedge clk) priv == 2'b11 && cfg[0][7] && cfg[0][4:3] != 2'b00 && !ok_dut);
endmodule
