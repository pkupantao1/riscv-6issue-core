//==============================================================================
// o6_vdot.sv - 执行器内的点积单元 (V4，docs/11 §9h)
//
// 指令：vqwdotau / vqwdotas (INT8 -> 32 位) 与 vfqwdota / vfqwdota_alt (OFP8 E4M3 / E5M2 -> FP32)，SEW = 8。
//   语义同 Spike (ZVLDOT_LOOP)：vd[0] (32 位) = c ⊕ sum_{i < VLMAX} a[i] * b[i]，a = vs1、b = vs2，不活动 (掩掉 / 尾部) 的元素当 0；
//   vd 其余位不变。整数：模 2^32 相加。浮点 (bulk_norm_dot_no_mult + f32_add_odd)：
//     各乘积的近似指数 e_i = (a, b 的指数 (次正规按 1) 之和 + 127 - 两个 bias)，零乘积取最小；整组最大 M；
//     乘积有效位左移 (23 - ma - mb + g) 位 (g = log2 VLMAX)，再按 M - e_i 右移并把移出的位或进最低位 (jam)，带符号求和；
//     规格化：截断 + jam (向奇数舍入)，特殊值 / 上溢照 Spike；最后与 c 按向奇数舍入相加 (上溢 -> Inf)，只保留 NV / OF。
//
// 结构 (每拍 128 对，全流水)：
//   A 段：A1 每拍算 128 个近似指数与特殊值标志 (原始字节写进乒乓缓冲，2 块各 8 拍)；A2 本拍最大 (两级) 并入整条的 M。
//   B 段：整条的 A 做完后逐拍：B1 读缓冲，B2 乘积 / 右移量，B3 对齐 + jam，B4～B6 求和 128 -> 16 -> 4 -> 1，再累加；
//     本条在做 B 时下一条可在另一块做 A。
//   收尾：S0 累加完成并读 vd (累加器转发：此时才读)，S1a / S1 规格化 (整数直接加 c)，FP32 加法 (fpnew，ROD)，拼回 vd 写出。
//   写回口与 o6_vex 共用：本单元的输出时刻在进收尾时就确定，提前 XL 拍发 blk_o，执行器那拍不往 o6_vex 送 (不撞拍)。
// 误预测 (br_mask) / 冲刷：各级带 br_mask，作废的丢弃；A 段作废的释放缓冲
//==============================================================================
`default_nettype none
module o6_vdot
  import o6_pkg::*;
#(
  parameter int TW = 8,
  parameter int XL = 7                       // 同执行器 o6_vex 的延迟
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,
  // 每拍一个寄存器 (组里第 k 个)
  input  wire                 in_v_i,
  input  wire                 in_first_i,
  input  wire                 in_last_i,
  input  wire  [2:0]          in_k_i,
  input  wire  [31:0]         in_raw_i,
  input  wire                 in_alt_i,
  input  wire  [2:0]          in_lm_i,
  input  wire  [VLEN-1:0]     in_vs2_i,
  input  wire  [VLEN-1:0]     in_vs1_i,
  input  var vsrc_t          in_vdo_i,     // 第一拍有效：vd 旧值的来源 (S0 时才读：累加器转发)
  input  var preg_t          in_pd_i,      // 第一拍有效：目的物理寄存器
  input  wire  [CL_B-1:0]     in_act_i,     // 本拍各元素是否活动
  input  var br_mask_t        in_bm_i,
  input  wire  [TW-1:0]       in_tag_i,
  output logic                a_rdy_o,      // 有空的 A 缓冲 (新的一条可以开始)
  output vsrc_t               rd_o,         // S0 读 vd 旧值 (执行器按它读 vprf，当拍给 rd_d_i)
  input  wire  [VLEN-1:0]     rd_d_i,
  output logic                blk_o,        // XL 拍后本单元写回：这拍不要往 o6_vex 送
  output logic                out_v_o,
  output logic [VLEN-1:0]     out_d_o,
  output logic [TW-1:0]       out_tag_o,
  output logic [4:0]          out_ff_o
);
  localparam int NE = CL_B;                   // 每拍元素数 (e8)
  localparam int PF = 5;                      // FP32 加法 (fpnew) 的流水级数
  localparam int OUTD = 3 + PF;               // 收尾第 0 级 (S0) 到输出的拍数 (S1a、S1、fpnew、输出级)
  initial assert (OUTD >= XL) else $error("o6_vdot: OUTD = %0d < XL = %0d，blk_o 来不及提前", OUTD, XL);

  // 一条点积的上下文 (B 段起随流水走)
  typedef struct packed {
    logic [7:0]   mx;                         // 整条的最大近似指数
    logic [3:0]   g;                          // guard 位数
    logic         fa, fb;                     // a / b 是 E5M2 (否则 E4M3)
    logic         fp;                         // 浮点 (否则整数)
    logic         sa, sb;                     // 整数：a / b 带符号
    logic [3:0]   fl;                         // {正无穷, 负无穷, NaN, 无效 (Inf × 0)}
    vsrc_t        vdo;                        // vd 旧值的来源
    preg_t        pd;
    logic [TW-1:0] tag;
  } ctx_t;

  //--------------------------------------------------------------------------
  // OFP8 解码
  //--------------------------------------------------------------------------
  typedef struct packed {
    logic       s;
    logic [4:0] es;                           // 指数 (次正规 / 零按 1)
    logic [3:0] sig;                          // 有效位 (含隐含位)
    logic       z, inf, nan;
  } f8_t;
  function automatic f8_t f8(input logic [7:0] v, input logic e5);
    f8_t r;
    r.s = v[7];
    if (e5) begin
      r.es  = (v[6:2] == '0) ? 5'd1 : v[6:2];
      r.sig = {1'b0, v[6:2] != '0, v[1:0]};
      r.z   = v[6:0] == '0;
      r.inf = v[6:2] == 5'h1F && v[1:0] == '0;
      r.nan = v[6:2] == 5'h1F && v[1:0] != '0;
    end else begin
      r.es  = (v[6:3] == '0) ? 5'd1 : {1'b0, v[6:3]};
      r.sig = {v[6:3] != '0, v[2:0]};
      r.z   = v[6:0] == '0;
      r.inf = 1'b0;
      r.nan = v[6:0] == 7'h7F;
    end
    return r;
  endfunction
  // 近似指数：零乘积取最小 (127 - 两个 bias)
  function automatic logic [7:0] pexp(input logic [7:0] a, input logic [7:0] b, input logic fa, input logic fb);
    f8_t x, y;
    logic [7:0] emin;
    x = f8(a, fa); y = f8(b, fb);
    emin = 8'(127 - (fa ? 15 : 7) - (fb ? 15 : 7));
    return (x.z || y.z) ? emin : emin + 8'(x.es) + 8'(y.es);
  endfunction
  // 特殊值标志 {正无穷, 负无穷, NaN, 无效}
  function automatic logic [3:0] pflag(input logic [7:0] a, input logic [7:0] b, input logic fa, input logic fb);
    f8_t x, y;
    logic ei, en, ez, inv;
    x = f8(a, fa); y = f8(b, fb);
    ei = x.inf || y.inf; en = x.nan || y.nan; ez = x.z || y.z;
    inv = (x.inf && y.z) || (y.inf && x.z);
    return {ei && !en && !ez && x.s == y.s, ei && !en && !ez && x.s != y.s, inv || en, inv};
  endfunction
  // 对齐后的带符号乘积 (浮点) / 乘积 (整数)
  localparam int VW = 50;
  function automatic logic signed [VW-1:0] pval(input logic [7:0] a, input logic [7:0] b, input ctx_t c);
    if (c.fp) begin
      f8_t x, y;
      logic [7:0]  e, d;
      logic [63:0] v, sh, jm;
      logic signed [VW-1:0] m;
      x = f8(a, c.fa); y = f8(b, c.fb);
      v  = (64'(x.sig) * 64'(y.sig)) << (23 - (c.fa ? 2 : 3) - (c.fb ? 2 : 3) + int'(c.g));
      e  = pexp(a, b, c.fa, c.fb);
      d  = c.mx - e;
      sh = (d >= 8'd64) ? 64'd0 : (v >> d);
      jm = (d >= 8'd64) ? v : (v & ((64'd1 << d) - 64'd1));
      sh = sh | 64'(jm != '0);
      m  = VW'(sh);
      return (x.s ^ y.s) ? -m : m;
    end else begin
      logic signed [VW-1:0] p, q;
      p = c.sa ? VW'(signed'(a)) : VW'({1'b0, a});
      q = c.sb ? VW'(signed'(b)) : VW'({1'b0, b});
      return p * q;
    end
  endfunction

  //--------------------------------------------------------------------------
  // 乒乓缓冲与分配
  //--------------------------------------------------------------------------
  localparam logic [1:0] B_FREE = 2'd0, B_FILL = 2'd1, B_FULL = 2'd2;
  logic [2*VLEN-1:0] bufm [2][8];             // {b (vs2), a (vs1)} 原始字节 (不活动的已清 0)
  logic [1:0]        st  [2];
  ctx_t              mt  [2];                 // 各块的上下文 (mx / fl 在 A 段累积)
  br_mask_t          mbm [2];
  logic [3:0]        mnb [2];
  logic              oq  [2];                 // 分配顺序 (B 段按序取)
  logic [1:0]        oqn;
  logic              cur;                     // 正在 A 段送拍的那块
  logic              fr_v, fr_id;
  always_comb begin
    fr_v = (st[0] == B_FREE) || (st[1] == B_FREE);
    fr_id = (st[0] == B_FREE) ? 1'b0 : 1'b1;
  end
  assign a_rdy_o = fr_v;
  wire alloc = in_v_i && in_first_i;

  //--------------------------------------------------------------------------
  // 输入级 / A 段
  //--------------------------------------------------------------------------
  logic             i_v, i_first, i_last, i_id;
  logic [2:0]       i_k;
  logic [VLEN-1:0]  i_a, i_b;
  logic             i_fa, i_fb, i_fp;
  br_mask_t         i_bm;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) i_v <= 1'b0;
    else         i_v <= in_v_i && !flush_i && !br_killed(in_bm_i, br_upd_i);
  always_ff @(posedge clk_i)
    if (in_v_i) begin
      for (int e = 0; e < NE; e++) begin
        i_a[8*e +: 8] <= in_act_i[e] ? in_vs1_i[8*e +: 8] : 8'd0;
        i_b[8*e +: 8] <= in_act_i[e] ? in_vs2_i[8*e +: 8] : 8'd0;
      end
      i_first <= in_first_i; i_last <= in_last_i; i_k <= in_k_i;
      i_id    <= alloc ? fr_id : cur;
      i_fa    <= in_alt_i; i_fb <= in_raw_i[26]; i_fp <= in_raw_i[12];
      i_bm    <= br_mask_upd(in_bm_i, br_upd_i);
    end
  wire i_ok = i_v && !flush_i && !br_killed(i_bm, br_upd_i);

  // A1：各元素近似指数与标志；原始字节写缓冲
  logic             a1_v, a1_first, a1_last, a1_id;
  logic [2:0]       a1_k;
  logic [7:0]       a1_e [NE];
  logic [3:0]       a1_fl;
  br_mask_t         a1_bm;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) a1_v <= 1'b0;
    else         a1_v <= i_ok;
  always_ff @(posedge clk_i)
    if (i_ok) begin
      logic [3:0] f;
      f = '0;
      for (int e = 0; e < NE; e++) begin
        a1_e[e] <= i_fp ? pexp(i_a[8*e +: 8], i_b[8*e +: 8], i_fa, i_fb) : 8'd0;
        if (i_fp) f = f | pflag(i_a[8*e +: 8], i_b[8*e +: 8], i_fa, i_fb);
      end
      a1_fl <= f;
      a1_first <= i_first; a1_last <= i_last; a1_k <= i_k; a1_id <= i_id;
      a1_bm <= br_mask_upd(i_bm, br_upd_i);
      bufm[i_id][i_k] <= {i_b, i_a};
    end
  wire a1_ok = a1_v && !flush_i && !br_killed(a1_bm, br_upd_i);

  // A2：本拍最大，两级 (128 -> 16 -> 1)
  logic             am_v, am_first, am_last, am_id;
  logic [2:0]       am_k;
  logic [7:0]       am_g [16];
  logic [3:0]       am_fl;
  br_mask_t         am_bm;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) am_v <= 1'b0;
    else         am_v <= a1_ok;
  always_ff @(posedge clk_i)
    if (a1_ok) begin
      for (int j = 0; j < 16; j++) begin
        logic [7:0] m;
        m = a1_e[8*j];
        for (int e = 1; e < 8; e++) if (a1_e[8*j + e] > m) m = a1_e[8*j + e];
        am_g[j] <= m;
      end
      am_fl <= a1_fl;
      am_first <= a1_first; am_last <= a1_last; am_k <= a1_k; am_id <= a1_id;
      am_bm <= br_mask_upd(a1_bm, br_upd_i);
    end
  wire am_ok = am_v && !flush_i && !br_killed(am_bm, br_upd_i);
  logic             a2_v, a2_first, a2_last, a2_id;
  logic [2:0]       a2_k;
  logic [7:0]       a2_mx;
  logic [3:0]       a2_fl;
  br_mask_t         a2_bm;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) a2_v <= 1'b0;
    else         a2_v <= am_ok;
  always_ff @(posedge clk_i)
    if (am_ok) begin
      logic [7:0] m;
      m = am_g[0];
      for (int j = 1; j < 16; j++) if (am_g[j] > m) m = am_g[j];
      a2_mx <= m; a2_fl <= am_fl;
      a2_first <= am_first; a2_last <= am_last; a2_k <= am_k; a2_id <= am_id;
      a2_bm <= br_mask_upd(am_bm, br_upd_i);
    end
  wire a2_ok = a2_v && !flush_i && !br_killed(a2_bm, br_upd_i);

  //--------------------------------------------------------------------------
  // B 段：读 (B1) -> 乘积 / 移位量 (B2) -> 移位 + jam (B3) -> 8 路和 (B4) -> 4 路和 (B5) -> 4 路和 (B6) -> 累加 (S0)
  //--------------------------------------------------------------------------
  typedef struct packed {
    logic     first, last;
    br_mask_t bm;
    ctx_t     c;
  } bst_t;
  localparam int NB = 6;                      // B1..B6
  logic             b_on, b_id;
  logic [2:0]       b_k;
  logic [3:0]       b_nb;
  bst_t             b_c;                      // 正在逐拍读的这条
  wire  b_ok    = b_on && !flush_i && !br_killed(b_c.bm, br_upd_i);
  wire  b_lnow  = b_ok && 4'(b_k) + 4'd1 >= b_nb;          // 本拍读最后一拍：下一条可以接着开始
  logic [NB:1]      bv;                       // 各级有效
  bst_t             bs [NB+1];                // 各级的上下文 (下标 1..NB)
  logic [NB:1]      bok;
  always_comb
    for (int i = 1; i <= NB; i++) bok[i] = bv[i] && !flush_i && !br_killed(bs[i].bm, br_upd_i);
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) bv <= '0;
    else begin
      bv[1] <= b_ok;
      for (int i = 2; i <= NB; i++) bv[i] <= bok[i-1];
    end
  always_ff @(posedge clk_i) begin
    if (b_ok) bs[1] <= '{first: b_k == '0, last: 4'(b_k) + 4'd1 >= b_nb, bm: br_mask_upd(b_c.bm, br_upd_i), c: b_c.c};
    for (int i = 2; i <= NB; i++) begin bs[i] <= bs[i-1]; bs[i].bm <= br_mask_upd(bs[i-1].bm, br_upd_i); end
  end
  // B1：读出一拍
  logic [2*VLEN-1:0] b1_ab;
  always_ff @(posedge clk_i) if (b_ok) b1_ab <= bufm[b_id][b_k];
  // B2：各元素的有效位乘积 (浮点 4 × 4 位 / 整数 9 × 9 位带符号)、符号、右移量
  logic [15:0]      b2_p [NE];
  logic             b2_s [NE];
  logic [7:0]       b2_d [NE];
  always_ff @(posedge clk_i)
    for (int e = 0; e < NE; e++) begin
      logic [7:0] a, b;
      a = b1_ab[8*e +: 8]; b = b1_ab[VLEN + 8*e +: 8];
      if (bs[1].c.fp) begin
        f8_t x, y;
        x = f8(a, bs[1].c.fa); y = f8(b, bs[1].c.fb);
        b2_p[e] <= 16'(x.sig) * 16'(y.sig);
        b2_s[e] <= x.s ^ y.s;
        b2_d[e] <= bs[1].c.mx - pexp(a, b, bs[1].c.fa, bs[1].c.fb);
      end else begin
        logic signed [17:0] p, q, r;
        p = bs[1].c.sa ? 18'(signed'(a)) : 18'({1'b0, a});
        q = bs[1].c.sb ? 18'(signed'(b)) : 18'({1'b0, b});
        r = p * q;
        b2_s[e] <= r < 0;
        b2_p[e] <= 16'(r < 0 ? -r : r);
        b2_d[e] <= '0;
      end
    end
  // B3：浮点左移 (23 - ma - mb + g)、按 d 右移并 jam；带符号
  logic signed [VW-1:0] b3_val [NE];
  always_ff @(posedge clk_i)
    for (int e = 0; e < NE; e++) begin
      logic [63:0] v, sh, jm;
      logic signed [VW-1:0] m;
      if (bs[2].c.fp) begin
        v  = 64'(b2_p[e]) << (23 - (bs[2].c.fa ? 2 : 3) - (bs[2].c.fb ? 2 : 3) + int'(bs[2].c.g));
        sh = (b2_d[e] >= 8'd64) ? 64'd0 : (v >> b2_d[e]);
        jm = (b2_d[e] >= 8'd64) ? v : (v & ((64'd1 << b2_d[e]) - 64'd1));
        sh = sh | 64'(jm != '0);
      end else sh = 64'(b2_p[e]);
      m = VW'(sh);
      b3_val[e] <= b2_s[e] ? -m : m;
    end
  // B4 / B5 / B6：128 -> 16 -> 4 -> 1 (实际是 carry-save 树，最后一级才进位)
  logic signed [VW-1:0] b4_ps [16];
  logic signed [VW-1:0] b5_ps [4];
  logic signed [VW-1:0] b6_sum;
  always_ff @(posedge clk_i) begin
    for (int j = 0; j < 16; j++) begin
      logic signed [VW-1:0] t;
      t = '0;
      for (int e = 0; e < 8; e++) t = t + b3_val[8*j + e];
      b4_ps[j] <= t;
    end
    for (int j = 0; j < 4; j++) begin
      logic signed [VW-1:0] t;
      t = '0;
      for (int e = 0; e < 4; e++) t = t + b4_ps[4*j + e];
      b5_ps[j] <= t;
    end
    b6_sum <= b5_ps[0] + b5_ps[1] + b5_ps[2] + b5_ps[3];
  end
  logic signed [VW+3:0] acc;

  //--------------------------------------------------------------------------
  // 收尾 S0：累加完成
  //--------------------------------------------------------------------------
  logic             s0_v;
  ctx_t             s0_c;
  br_mask_t         s0_bm;
  logic signed [VW+3:0] s0_acc;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) s0_v <= 1'b0;
    else         s0_v <= bok[NB] && bs[NB].last;
  always_ff @(posedge clk_i)
    if (bok[NB]) begin
      logic signed [VW+3:0] a;
      a = (bs[NB].first ? '0 : acc) + (VW+4)'(b6_sum);
      acc <= a;
      if (bs[NB].last) begin s0_acc <= a; s0_c <= bs[NB].c; s0_bm <= br_mask_upd(bs[NB].bm, br_upd_i); end
    end
  wire s0_ok = s0_v && !flush_i && !br_killed(s0_bm, br_upd_i);
  // 累加器转发：vd 旧值的生产者 (同一单元里更早的点积) 还在 B 段 / S0 时先不开始 —— 它过了 S0，OUTD 拍内写回；
  //   本条最早 nb + NB + 1 拍后到 S0 读 vd，这时它已写进 vprf 或正在输出级 (旁路)
  logic b_dep;
  always_comb begin
    vsrc_t o;
    o = mt[oq[0]].vdo;
    b_dep = (b_on && b_c.c.pd == o.p) || (s0_v && s0_c.pd == o.p);
    for (int i = 1; i <= NB; i++) if (bv[i] && bs[i].c.pd == o.p) b_dep = 1'b1;
    b_dep = b_dep && o.use_ && !o.cr;
  end
  wire  b_start = (!b_ok || b_lnow) && oqn != '0 && st[oq[0]] == B_FULL && !b_dep && !flush_i && !br_killed(mbm[oq[0]], br_upd_i);

  // S0 读 vd 旧值：同单元刚输出的那条正好是生产者时旁路 (它这拍才写 vprf)
  logic [VLEN-1:0]  s0_vd;
  preg_t            out_pd;
  assign rd_o  = s0_c.vdo;
  assign s0_vd = !s0_c.vdo.use_ ? '0 : (out_v_o && !s0_c.vdo.cr && out_pd == s0_c.vdo.p) ? out_d_o : rd_d_i;
  // S1a：符号、绝对值、最高位位置
  logic             sa_v;
  ctx_t             sa_c;
  br_mask_t         sa_bm;
  logic             sa_sg;
  logic [63:0]      sa_mag;
  logic [5:0]       sa_nd;
  logic [31:0]      sa_ir;                    // 整数结果
  logic [VLEN-1:0]  sa_vd;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) sa_v <= 1'b0;
    else         sa_v <= s0_ok;
  always_ff @(posedge clk_i)
    if (s0_ok) begin
      logic [63:0] m;
      int          nd;
      m  = 64'(s0_acc < 0 ? -s0_acc : s0_acc);
      nd = 0;
      for (int i = 0; i < 64; i++) if (m[i]) nd = i;
      sa_sg <= s0_acc < 0; sa_mag <= m; sa_nd <= 6'(nd);
      sa_ir <= s0_vd[31:0] + 32'(s0_acc);
      sa_vd <= s0_vd; sa_c <= s0_c;
      sa_bm <= br_mask_upd(s0_bm, br_upd_i);
    end
  wire sa_ok = sa_v && !flush_i && !br_killed(sa_bm, br_upd_i);
  // S1：规格化 (bulk_norm_dot_no_mult 的后半)；整数直接加 c
  logic             s1_v, s1_fp;
  logic [31:0]      s1_r, s1_c;
  logic [4:0]       s1_ff;
  logic [VLEN-1:0]  s1_vd;
  logic [TW-1:0]    s1_tag;
  preg_t            s1_pd;
  br_mask_t         s1_bm;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) s1_v <= 1'b0;
    else         s1_v <= sa_ok;
  always_ff @(posedge clk_i)
    if (sa_ok) begin
      logic        sg, ov, ovf, nano, inf, opsi;
      logic [63:0] mag, t, rs, jm;
      int          nd, ex, sb;
      logic [31:0] o;
      sg  = sa_sg; mag = sa_mag; nd = int'(sa_nd);
      ex  = int'(sa_c.mx) - 23 - int'(sa_c.g) + nd;
      sb  = (ex <= 0) ? 23 - (1 - ex) : 23;
      if (sb < 0) sb = 0;
      t   = mag << sb;                                  // 同 Spike：uint64 移位 (超出 64 位的丢掉)
      rs  = t >> nd;
      jm  = t & ((64'd1 << nd) - 64'd1);
      rs  = rs | 64'(jm != '0);
      inf  = sa_c.fl[3] || sa_c.fl[2];
      ov   = (ex >= 255 && mag != '0) || inf;
      opsi = sa_c.fl[3] && sa_c.fl[2];
      nano = sa_c.fl[1] || opsi;
      ovf  = (ex >= 255 && mag != '0) && !inf && !nano;
      o = rs[31:0];
      if (nano)           o = 32'h7FC0_0000;
      else if (ov)        o = {inf ? sa_c.fl[2] : sg, 8'hFF, 23'd0};
      else if (mag == '0) o = {sg, 8'd0, rs[22:0]};
      else if (ex <= 0)   o = {sg, 8'd0, rs[22:0]};
      else                o = {sg, 8'(ex), rs[22:0]};
      s1_fp <= sa_c.fp;
      s1_r  <= sa_c.fp ? o : sa_ir;
      s1_ff <= sa_c.fp ? {sa_c.fl[0] || opsi, 1'b0, ovf, 2'b00} : 5'd0;
      s1_c  <= sa_vd[31:0];
      s1_vd <= sa_vd; s1_tag <= sa_c.tag; s1_pd <= sa_c.pd;
      s1_bm <= br_mask_upd(sa_bm, br_upd_i);
    end
  wire s1_ok = s1_v && !flush_i && !br_killed(s1_bm, br_upd_i);

  // FP32 加法 (向奇数舍入)：f32_add_odd(res, c)
  localparam fpnew_pkg::fpu_features_t FEAT = '{
    Width:         32,
    EnableVectors: 1'b0,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b10000,
    IntFmtMask:    4'b0000
  };
  localparam fpnew_pkg::fpu_implementation_t IMPL = '{
    PipeRegs:   '{'{PF, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}},
    UnitTypes:  '{'{fpnew_pkg::PARALLEL, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  logic [31:0]        fa_r;
  fpnew_pkg::status_t fa_st;
  logic               fa_v;
  fpnew_top #(
    .Features       (FEAT),
    .Implementation (IMPL),
    .DivSqrtSel     (fpnew_pkg::THMULTI),
    .TagType        (logic)
  ) u_fadd (
    .clk_i, .rst_ni,
    .operands_i    ({s1_c, s1_r, 32'd0}),
    .rnd_mode_i    (fpnew_pkg::ROD),
    .op_i          (fpnew_pkg::ADD),
    .op_mod_i      (1'b0),
    .src_fmt_i     (fpnew_pkg::FP32),
    .dst_fmt_i     (fpnew_pkg::FP32),
    .int_fmt_i     (fpnew_pkg::INT32),
    .vectorial_op_i(1'b0),
    .tag_i         (1'b0),
    .simd_mask_i   ('1),
    .in_valid_i    (s1_v && s1_fp),
    .in_ready_o    (),
    .flush_i       (1'b0),
    .result_o      (fa_r),
    .status_o      (fa_st),
    .tag_o         (),
    .out_valid_o   (fa_v),
    .out_ready_i   (1'b1),
    .busy_o        (),
    .early_valid_o ()
  );
  // 旁路链 (与 fpnew 同深度)
  logic             d_v  [PF];
  logic             d_rv [PF];
  logic             d_fp [PF];
  logic [31:0]      d_r  [PF];
  logic [4:0]       d_ff [PF];
  logic [VLEN-1:0]  d_vd [PF];
  logic [TW-1:0]    d_tag[PF];
  preg_t            d_pd [PF];
  br_mask_t         d_bm [PF];
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) for (int s = 0; s < PF; s++) begin d_v[s] <= 1'b0; d_rv[s] <= 1'b0; end
    else begin
      d_v[0] <= s1_ok; d_rv[0] <= s1_v && s1_fp;
      for (int s = 1; s < PF; s++) begin
        d_v[s]  <= d_v[s-1] && !flush_i && !br_killed(d_bm[s-1], br_upd_i);
        d_rv[s] <= d_rv[s-1];
      end
    end
  always_ff @(posedge clk_i) begin
    d_fp[0] <= s1_fp; d_r[0] <= s1_r; d_ff[0] <= s1_ff; d_vd[0] <= s1_vd; d_tag[0] <= s1_tag; d_pd[0] <= s1_pd;
    d_bm[0] <= br_mask_upd(s1_bm, br_upd_i);
    for (int s = 1; s < PF; s++) begin
      d_fp[s] <= d_fp[s-1]; d_r[s] <= d_r[s-1]; d_ff[s] <= d_ff[s-1]; d_vd[s] <= d_vd[s-1]; d_tag[s] <= d_tag[s-1]; d_pd[s] <= d_pd[s-1];
      d_bm[s] <= br_mask_upd(d_bm[s-1], br_upd_i);
    end
  end
  wire d_ok = d_v[PF-1] && !flush_i && !br_killed(d_bm[PF-1], br_upd_i);

  // 输出级
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) out_v_o <= 1'b0;
    else         out_v_o <= d_ok;
  always_ff @(posedge clk_i)
    if (d_ok) begin
      logic [31:0] r;
      logic [4:0]  f;
      r = d_r[PF-1]; f = d_ff[PF-1];
      if (d_fp[PF-1]) begin
        r = fa_st.OF ? {fa_r[31], 8'hFF, 23'd0} : fa_r;    // f32_add_odd：上溢 (ROD 给最大有限值) -> Inf
        f = f | {fa_st.NV, 1'b0, fa_st.OF, 2'b00};
      end
      out_d_o   <= {d_vd[PF-1][VLEN-1:32], r};
      out_tag_o <= d_tag[PF-1];
      out_pd    <= d_pd[PF-1];
      out_ff_o  <= f;
    end

  // blk_o：S0 起 OUTD 拍后输出；o6_vex 在第 s 拍收的拍 s + XL 拍出，所以 S0 后 OUTD - XL 拍那拍挡住 o6_vex
  logic [OUTD:0] q;                           // q[j] = s0_v 晚 j + 1 拍
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) q <= '0;
    else         q <= {q[OUTD-1:0], s0_v};
  assign blk_o = (OUTD == XL) ? s0_v : q[OUTD - XL - 1];

  //--------------------------------------------------------------------------
  // 缓冲状态 / B 引擎
  //--------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st[0] <= B_FREE; st[1] <= B_FREE; oqn <= '0; b_on <= 1'b0; cur <= 1'b0;
    end else if (flush_i) begin
      st[0] <= B_FREE; st[1] <= B_FREE; oqn <= '0; b_on <= 1'b0;
    end else begin
      logic [1:0] s [2];
      logic       q0, q1;
      logic [1:0] n;
      s[0] = st[0]; s[1] = st[1]; q0 = oq[0]; q1 = oq[1]; n = oqn;
      // B 引擎
      if (b_on) begin
        if (!b_ok || b_lnow) b_on <= 1'b0;
        b_k <= b_k + 3'd1;
      end
      if (b_start) begin
        b_on <= 1'b1; b_id <= q0; b_k <= '0; b_nb <= mnb[q0];
        s[q0] = B_FREE;                                   // 读的拍总比新写的早 (见上面的说明)
        q0 = q1; n = n - 2'd1;
      end
      // A 段：分配 / 写满
      if (alloc && !br_killed(in_bm_i, br_upd_i)) begin
        s[fr_id] = B_FILL; cur <= fr_id;
        if (n == 2'd0) q0 = fr_id; else q1 = fr_id;
        n = n + 2'd1;
      end
      if (a2_ok && a2_last) s[a2_id] = B_FULL;
      // 作废
      if (br_upd_i.kill)
        for (int j = 0; j < 2; j++)
          if (s[j] != B_FREE && !(alloc && 1'(j) == fr_id) && mbm[j][br_upd_i.kill_tag]) s[j] = B_FREE;   // 本拍刚分配的看 in_bm (已排除)
      if (br_upd_i.kill) begin
        // 顺序队列去掉已释放的 (B_FREE 且不是刚分配的)
        logic [1:0] m;
        logic       r0, r1;
        m = '0; r0 = q0; r1 = q1;
        if (n >= 2'd1 && s[q0] != B_FREE) begin r0 = q0; m = 2'd1; end
        if (n == 2'd2 && s[q1] != B_FREE) begin if (m == 2'd0) r0 = q1; else r1 = q1; m = m + 2'd1; end
        q0 = r0; q1 = r1; n = m;
      end
      st[0] <= s[0]; st[1] <= s[1]; oq[0] <= q0; oq[1] <= q1; oqn <= n;
    end
  end
  // 各块上下文 / br_mask
  always_ff @(posedge clk_i) begin
    for (int j = 0; j < 2; j++) mbm[j] <= br_mask_upd(mbm[j], br_upd_i);
    if (alloc) begin
      mt[fr_id].g   <= 4'(7 + (in_lm_i[2] ? int'(in_lm_i) - 8 : int'(in_lm_i)));
      mt[fr_id].fa  <= in_alt_i;
      mt[fr_id].fb  <= in_raw_i[26];
      mt[fr_id].fp  <= in_raw_i[12];
      mt[fr_id].sa  <= in_alt_i;
      mt[fr_id].sb  <= in_raw_i[26];
      mt[fr_id].vdo <= in_vdo_i;
      mt[fr_id].pd  <= in_pd_i;
      mt[fr_id].tag <= in_tag_i;
      mbm[fr_id]    <= br_mask_upd(in_bm_i, br_upd_i);
    end
    if (a2_ok) begin
      if (a2_first) begin mt[a2_id].mx <= a2_mx; mt[a2_id].fl <= a2_fl; end
      else begin
        if (a2_mx > mt[a2_id].mx) mt[a2_id].mx <= a2_mx;
        mt[a2_id].fl <= mt[a2_id].fl | a2_fl;
      end
      if (a2_last) mnb[a2_id] <= 4'(a2_k) + 4'd1;
    end
    if (b_start) begin
      b_c.c  <= mt[oq[0]];
      b_c.bm <= br_mask_upd(mbm[oq[0]], br_upd_i);
    end else b_c.bm <= br_mask_upd(b_c.bm, br_upd_i);
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni && !flush_i) begin
      if (alloc) assert (fr_v) else $error("o6_vdot: 没有空的 A 缓冲却来了新的一条");
      assert (fa_v == d_rv[PF-1]) else $error("o6_vdot: fpnew 输出与旁路链不对齐");
    end
`endif
endmodule
