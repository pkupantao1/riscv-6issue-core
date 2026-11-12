//==============================================================================
// o6_sc.sv - 统计校正器 (Statistical Corrector，GEHL 形式；TAGE-SC-L 的 SC 部分)
//
// 作用：TAGE 学不好的"有偏但带噪声"的分支，用几张不带标签的计数器表求和来纠正 TAGE 的方向。
//   SC_NT 张表：表 0 只按块地址索引 (偏置表)，表 1..3 再异或 TAGE 的折叠历史 0/1/2 (历史 6/10/18)。
//   每表 2^SC_LG 行；每行 4 个槽组 (块内 8 B 一组，槽号 [3:2]) x TAGE 方向 2 种，各一个 SC_CW 位有符号计数器。
//   (按块索引、行内按槽组区分：F0 就能读，不必等预译码给出条件分支的位置)
//
// 预测 (每槽)：sum = Σ_t (2*c_t + 1) + (TAGE 方向 ? +8 : -8) * TAGE 置信度 (提供者计数器强弱 1/3/5/7，无提供者 1)
//   SC 方向 = sum >= 0；与 TAGE 不同时，只有 |sum| >= theta 且 TAGE 计数器未饱和才推翻
//   (bpsim 评估：不加门控 SC 反而变差)
//   时序：F0 读表 (触发器，按 F0 的 PC / 折叠历史) -> F1 寄存 -> F1->F2 求各槽组两种方向的和并寄存 ->
//         F2 拿 TAGE 的结果加置信度项、比较，输出修正后的 fdir (前端寄存进 F3，F3 逻辑不变)
// 训练 (提交时，块里所有已解析的条件分支)：最终方向错，或 |sum| < theta 时，按实际方向把各表对应计数器走一步
//   (同一计数器上的多次先按方向合并成净增减再饱和)；theta 自适应 (Seznec)。表是触发器，训练直接读改写。
//   不能只跟 TAGE 的两个训练候选走：候选是"误预测优先"挑的，SC 会学成"TAGE 常错"而推翻过多
//   (CoreMark：只训候选时推翻 716 次只对 380 次)
//==============================================================================
`default_nettype none
module o6_sc
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // ---- 预测
  input  wire                 f0_sent_i,        // 本拍 F0 -> F1
  input  wire  [63:0]         f0_pc_i,
  input  var bp_fold_t        f0_fold_i,
  input  wire                 f1_move_i,        // 本拍 F1 -> F2
  input  var bp_slot_t [15:0] f2_tage_i,        // F2：TAGE 每槽结果
  output bp_slot_t [15:0]     f2_slot_o,        // F2：加上 SC 修正 (fdir / scd / scl)

  // ---- 训练
  input  wire                 tr_v_i,
  input  wire  [63:0]         tr_pc_i,
  input  var bp_fold_t        tr_fold_i,
  input  wire  [15:0]         tr_res_i,         // 已解析的条件分支槽
  input  var bp_slot_t [15:0] tr_sinfo_i,       // 各槽预测时的信息
  input  wire  [15:0]         tr_brt_i          // 各槽实际方向
);
  localparam int NR = 1 << SC_LG;
  typedef logic signed [SC_CW-1:0] ctr_t;
  typedef ctr_t [3:0][1:0] row_t;               // [槽组][TAGE 方向]

  // 索引：块地址 (3 段异或) ^ 折叠历史 (表 1..3 用 TAGE 折叠历史 0..2，10 位再折到 SC_LG 位)
  function automatic logic [SC_LG-1:0] s_idx(input logic [63:0] pc, input bp_fold_t f, input int t);
    logic [33:0]      a;
    logic [SC_LG-1:0] h;
    logic [BP_LG-1:0] fv;
    a = pc[38:5];
    h = a[SC_LG-1:0] ^ a[2*SC_LG-1:SC_LG] ^ a[3*SC_LG-1:2*SC_LG];
    if (t > 0) begin
      fv = f.fi[(t > 0) ? t - 1 : 0];
      h  = h ^ fv[SC_LG-1:0] ^ SC_LG'(fv >> SC_LG) ^ SC_LG'(t);
    end
    return h;
  endfunction

  row_t tb [SC_NT][NR];

  //--------------------------------------------------------------------------
  // 预测
  //--------------------------------------------------------------------------
  // 读表在 F1 (2026-10-20，6 GHz)：F0 只寄存 PC / 折叠历史，F1 用寄存值算索引、读表、求和，F1 -> F2 寄存。
  // 原来 F0 当拍读表：F3 覆盖重定向 -> F0 选 PC -> 索引异或 -> 128 行触发器表读出串在一拍
  // (2nm 综合前端最长路径 283 ps / 44 级)。F1 原来只做求和，读表放进去后约 100–130 ps。
  // 与原写法的差别：读表晚一拍，可能看到这一拍里训练写入的新值 (只影响预测，不影响正确性)。
  // 2026-10-21 (6 GHz 第二轮)：索引在 F0 算好寄存 (pc0 -> 散列 -> 寄存器，不进宏)，F1 只读表；
  // 原来 F1 里 f1_pc -> 散列 -> 128 行读出 -> 求和 -> 加偏置 / 取绝对值 / 比 theta 串成 274 ps。
  logic [SC_LG-1:0] f1_idx [SC_NT];
  row_t f1_row [SC_NT];
  always_ff @(posedge clk_i)
    if (f0_sent_i) for (int t = 0; t < SC_NT; t++) f1_idx[t] <= s_idx(f0_pc_i, f0_fold_i, t);
  always_comb
    for (int t = 0; t < SC_NT; t++) f1_row[t] = tb[t][f1_idx[t]];
  // F1 -> F2：按 TAGE 置信度的 4 种取值 (1/3/5/7) × 2 种方向预先算好 {SC 方向, 低置信}，寄存给 F2；
  // F2 拿到 TAGE 结果只做 8 选 1 (2026-10-20，6 GHz：原来 F2 里 TAGE 最长匹配 -> 置信度 -> 加法 -> 比较 theta
  // 串成 47 级 / 275 ps)。
  // sum = Σ(2c+1) + off = 2q + SC_NT + off，q = Σc，off = ±8(2k+1)：
  //   方向 sum >= 0       <=> 2q >= -SC_NT - off            (常数比较)
  //   低置信 |sum| < theta <=> th_lo < 2q < th_hi，th_lo = -theta - SC_NT - off，th_hi = theta - SC_NT - off
  // 两个阈值随 theta 每拍寄存 (比 theta 晚一拍，只影响预测)，F1 里只剩 读表 -> 4 数求和 -> 比较。
  logic [7:0] theta;
  logic signed [10:0] th_lo [1:0][3:0], th_hi [1:0][3:0];
  always_ff @(posedge clk_i)
    for (int d = 0; d < 2; d++)
      for (int c = 0; c < 4; c++) begin
        logic signed [10:0] off;
        off = d ? 11'sd8 * 11'(2 * c + 1) : -11'sd8 * 11'(2 * c + 1);
        th_lo[d][c] <= -$signed({3'b0, theta}) - 11'(SC_NT) - off;
        th_hi[d][c] <=  $signed({3'b0, theta}) - 11'(SC_NT) - off;
      end
  logic [1:0] f2_pre [3:0][1:0][3:0];          // [槽组][TAGE 方向][置信度序号] -> {scd, scl}
  always_ff @(posedge clk_i) begin
    if (f1_move_i)
      for (int g = 0; g < 4; g++)
        for (int d = 0; d < 2; d++) begin
          logic signed [10:0] q2;
          q2 = '0;
          for (int t = 0; t < SC_NT; t++) q2 = q2 + 11'(f1_row[t][g][d]);
          q2 = q2 <<< 1;
          for (int c = 0; c < 4; c++) begin
            logic signed [10:0] off;
            off = d ? 11'sd8 * 11'(2 * c + 1) : -11'sd8 * 11'(2 * c + 1);
            f2_pre[g][d][c] <= {q2 >= -11'(SC_NT) - off, q2 > th_lo[d][c] && q2 < th_hi[d][c]};
          end
        end
  end

  // TAGE 提供者计数器 (0..7，>=4 跳) 的置信度 |2v+1|，v = pctr - 4
  function automatic logic [3:0] tconf(input bp_slot_t i);
    if (!i.phit) return 4'd1;
    return i.pctr[2] ? {i.pctr[1:0], 1'b1} : {~i.pctr[1:0], 1'b1};
  endfunction
  always_comb begin
    for (int sl = 0; sl < 16; sl++) begin
      bp_slot_t          r;
      logic              tp;
      logic [3:0]        cf;
      r   = f2_tage_i[sl];
      tp  = r.tdir;
      cf  = tconf(r);                                  // 1 / 3 / 5 / 7 -> 序号 cf[2:1]
      {r.scd, r.scl} = f2_pre[sl / 4][tp][cf[2:1]];
      // 门控：SC 与 TAGE 不同时，|sum| < theta 或 TAGE 提供者计数器已饱和 (置信度 7) 都不推翻
      // (只用前一条时 CoreMark 上推翻 716 次只对 380 次；bpsim：两条都加最好)
      r.fdir = (r.scd != tp && (r.scl || tconf(r) == 4'd7)) ? tp : r.scd;
`ifdef O6_NO_SC
      r.fdir = tp;                     // 性能对比实验：关掉 SC
`endif
      f2_slot_o[sl] = r;
    end
  end

  //--------------------------------------------------------------------------
  // 训练
  //--------------------------------------------------------------------------
  function automatic ctr_t c_step(input ctr_t c, input logic tk);
    if (tk) return (c == ctr_t'((1 << (SC_CW - 1)) - 1)) ? c : c + 1'b1;
    else    return (c == ctr_t'(-(1 << (SC_CW - 1))))    ? c : c - 1'b1;
  endfunction

  localparam int CMAX = (1 << (SC_CW - 1)) - 1;
  localparam int CMIN = -(1 << (SC_CW - 1));
  // 饱和加净步数
  function automatic ctr_t c_add(input ctr_t c, input logic signed [4:0] d);
    logic signed [6:0] x;
    x = 7'(c) + 7'(d);
    if (x > 7'(CMAX)) x = 7'(CMAX);
    if (x < 7'(CMIN)) x = 7'(CMIN);
    return ctr_t'(x);
  endfunction

  // 训练分两拍 (2026-10-20，6 GHz：原来 tr 输入 -> 各槽净步数 / 对错计数 -> 读表加步数 -> theta 串成 53 级 / 291 ps)
  //   T1：按训练输入算各 [槽组][方向] 净步数、是否更新、对错计数、各表索引，寄存；
  //   T2：读改写计数器表、更新 tc / theta。读改写都在 T2，连续两次训练同一行也不丢更新，只是晚一拍生效。
  logic [SC_LG-1:0]  ti [SC_NT];
  row_t              nrow [SC_NT];
  logic              upd;
  logic signed [6:0] tc, tc_n;
  logic [7:0]        theta_n;
  logic signed [4:0] stp [3:0][1:0];       // T1：各 [槽组][TAGE 方向] 的净步数
  logic [4:0]        nw, nl;
  logic              t2_upd;               // T2 寄存
  logic signed [4:0] t2_stp [3:0][1:0];
  logic [4:0]        t2_nw, t2_nl;
  logic [SC_LG-1:0]  t2_ti [SC_NT];
  always_comb begin
    upd = 1'b0; nw = '0; nl = '0;
    for (int g = 0; g < 4; g++) for (int d = 0; d < 2; d++) stp[g][d] = '0;
    for (int sl = 0; sl < 16; sl++)
      if (tr_v_i && tr_res_i[sl]) begin
        bp_slot_t i;
        logic     tk, wrong;
        i = tr_sinfo_i[sl]; tk = tr_brt_i[sl];
        wrong = i.fdir != tk;
        if (wrong || i.scl) begin
          upd = 1'b1;
          stp[sl / 4][i.tdir] = stp[sl / 4][i.tdir] + (tk ? 5'sd1 : -5'sd1);
        end
        if (wrong)       nw = nw + 5'd1;
        else if (i.scl)  nl = nl + 5'd1;
      end
    for (int t = 0; t < SC_NT; t++) ti[t] = s_idx(tr_pc_i, tr_fold_i, t);
  end

  // T2：读改写 + 自适应阈值 (错得多就抬高、少推翻；低置信却对得多就降低)
  always_comb begin
    for (int t = 0; t < SC_NT; t++) begin
      nrow[t] = tb[t][t2_ti[t]];
      for (int g = 0; g < 4; g++)
        for (int d = 0; d < 2; d++) nrow[t][g][d] = c_add(nrow[t][g][d], t2_stp[g][d]);
    end
    tc_n = tc + 7'(t2_nw) - 7'(t2_nl); theta_n = theta;
    if (tc_n >= 7'sd31)       begin tc_n = '0; if (theta_n != 8'd255) theta_n = theta_n + 8'd1; end
    else if (tc_n <= -7'sd32) begin tc_n = '0; if (theta_n > 8'd6)    theta_n = theta_n - 8'd1; end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      theta <= 8'd20; tc <= '0;
      t2_upd <= 1'b0; t2_nw <= '0; t2_nl <= '0;
      for (int t = 0; t < SC_NT; t++)
        for (int r = 0; r < NR; r++) tb[t][r] <= '0;
    end else begin
      theta <= theta_n; tc <= tc_n;
      if (t2_upd) for (int t = 0; t < SC_NT; t++) tb[t][t2_ti[t]] <= nrow[t];
      t2_upd <= upd; t2_nw <= nw; t2_nl <= nl;
    end
  end
  always_ff @(posedge clk_i) begin
    t2_stp <= stp;
    for (int t = 0; t < SC_NT; t++) t2_ti[t] <= ti[t];
  end

`ifndef SYNTHESIS
  // 统计：训练到的分支里 SC 推翻了 TAGE 的次数、推翻后对 / 错
  longint st_ov = 0, st_ov_ok = 0;
  always_ff @(posedge clk_i)
    if (rst_ni)
      for (int sl = 0; sl < 16; sl++)
        if (tr_v_i && tr_res_i[sl] && tr_sinfo_i[sl].fdir != tr_sinfo_i[sl].tdir) begin
          st_ov++;
          if (tr_sinfo_i[sl].fdir == tr_brt_i[sl]) st_ov_ok++;
        end
  final $display("[perf] SC 推翻 TAGE %0d 次 (推翻后对 %0d)，阈值 %0d", st_ov, st_ov_ok, theta);
`endif
endmodule
