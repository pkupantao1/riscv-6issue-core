//==============================================================================
// o6_ittage.sv - 间接跳转目标预测 ITTAGE
//
// 原来非返回的 JALR 只用 uBTB / BTB 记的"上次目标"：同一条 JALR 在不同调用路径上去不同地方
// (函数指针、系统调用表、驱动 ops、switch 跳转表) 就会反复预测错。bpsim (Linux 启动 trace)：
// 间接跳转 MPKI 0.76 -> 0.07。
//
// 结构：IT_NT 张带标签表，每张 2^IT_LG 项，每项 {有效, IT_TW 位标签, 2 位置信, 1 位有用, 38 位目标}，
//   索引 / 标签 = 块地址 ^ TAGE 第 0/2/4/6 表的折叠历史 (6/18/56/172)。一个取指块最多以一条间接跳转
//   结束，所以按块预测一个目标。每表一块单口 SRAM：F0 读；训练的写进每表 2 项写队列，F0 不读该表时写出，
//   队列满时写优先、抢掉这一拍的读 (同 o6_tage)。上电 2^IT_LG 拍清零。
// 预测 (F2)：最长命中表为提供者，置信 > 0 用它的目标；置信 = 0 时用次长命中表的；都没有则不给 (前端用 uBTB)。
// 训练 (提交时，块里有已解析的非返回 JALR)：
//   提供者目标对 -> 置信 +1 (本次最终预测也对则有用位置 1)；目标错 -> 置信 > 0 则 -1，否则换成实际目标、置信 1
//   最终预测错且提供者不是最长表 -> 在更长的表里找第一个"无效或无用"的项分配；找不到则把这些项的有用位清掉
//==============================================================================
`default_nettype none
module o6_ittage
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // ---- 预测
  input  wire                 f0_rd_i,
  input  wire                 f0_sent_i,
  input  wire  [63:0]         f0_pc_i,
  input  var bp_fold_t        f0_fold_i,
  input  wire                 f1_move_i,
  output logic                f2_v_o,           // F2：有可用目标
  output logic [38:1]         f2_tgt_o,
  output it_info_t            f2_info_o,        // F2：随 FTQ 带到训练

  // ---- 训练
  input  wire                 tr_v_i,           // 该块有已解析的非返回 JALR
  input  wire  [63:0]         tr_pc_i,
  input  var bp_fold_t        tr_fold_i,
  input  var it_info_t        tr_info_i,
  input  wire  [38:1]         tr_act_i,         // 实际目标
  input  wire                 tr_mis_i,         // 前端最终预测的目标错了 (含没预测跳)

  // ---- MBIST
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int NE = 1 << IT_LG;
  localparam int EW = 1 + IT_TW + 2 + 1 + 38;   // 51
  typedef struct packed {
    logic             v;
    logic [IT_TW-1:0] tag;
    logic [1:0]       ctr;
    logic             u;
    logic [38:1]      tgt;
  } ent_t;

  // 第 t 张表用 TAGE 第 2t 张的折叠历史 (10 位索引折 / 12 位标签折)
  function automatic logic [IT_LG-1:0] i_idx(input logic [63:0] pc, input bp_fold_t f, input int t);
    logic [33:0]      a;
    logic [BP_LG-1:0] fv;
    a  = pc[38:5];
    fv = f.fi[2 * t];
    return a[IT_LG-1:0] ^ a[2*IT_LG-1:IT_LG] ^ fv[IT_LG-1:0] ^ IT_LG'(fv >> IT_LG) ^ IT_LG'(t * 37);
  endfunction
  function automatic logic [IT_TW-1:0] i_tag(input logic [63:0] pc, input bp_fold_t f, input int t);
    logic [33:0]      a;
    logic [BP_TW-1:0] g;
    a = pc[38:5];
    g = f.g1[2 * t];
    return a[IT_TW+7:8] ^ a[IT_TW-1:0] ^ g[IT_TW-1:0] ^ IT_TW'(g >> IT_TW);
  endfunction

  //--------------------------------------------------------------------------
  // 上电清零
  //--------------------------------------------------------------------------
  logic             ini_busy;
  logic [IT_LG-1:0] ini_a;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ini_busy <= 1'b1; ini_a <= '0;
    end else if (ini_busy) begin
      ini_a <= ini_a + 1'b1;
      if (ini_a == '1) ini_busy <= 1'b0;
    end
  end

  //--------------------------------------------------------------------------
  // 写队列 (每表 2 项)
  //--------------------------------------------------------------------------
  typedef struct packed {
    logic [IT_LG-1:0] addr;
    logic [EW-1:0]    data;
    logic [EW-1:0]    mask;
  } wr_t;
  wr_t        wq   [IT_NT][2];
  logic [1:0] wq_n [IT_NT];
  logic       w_go [IT_NT];
  logic       wr_v [IT_NT];
  wr_t        wr_r [IT_NT];

  //--------------------------------------------------------------------------
  // SRAM
  //--------------------------------------------------------------------------
  logic [IT_LG-1:0] f0_idx [IT_NT];
  logic [IT_TW-1:0] f0_tag [IT_NT];
  always_comb
    for (int t = 0; t < IT_NT; t++) begin
      f0_idx[t] = i_idx(f0_pc_i, f0_fold_i, t);
      f0_tag[t] = i_tag(f0_pc_i, f0_fold_i, t);
    end

  logic [EW-1:0]    rdata [IT_NT];
  logic [IT_NT-1:0] bd, bf, w_pri, f0_rok;
  for (genvar t = 0; t < IT_NT; t++) begin : g_t
    assign w_pri[t]  = wq_n[t] == 2'd2;
    assign f0_rok[t] = !w_pri[t];
    wire rd = f0_rd_i && !ini_busy && f0_rok[t];
    wire wr = !rd && w_go[t];
    o6_sram_sp #(.DEPTH(NE), .WIDTH(EW), .STYLE("block")) u_m (
      .clk_i,
      .ce_i   (ini_busy || rd || wr),
      .we_i   (ini_busy || !rd),
      .addr_i (ini_busy ? ini_a : rd ? f0_idx[t] : wq[t][0].addr),
      .wdata_i(ini_busy ? '0 : wq[t][0].data),
      .wmask_i(ini_busy ? '1 : wq[t][0].mask),
      .rdata_o(rdata[t]),
      .bist_start_i, .bist_done_o(bd[t]), .bist_fail_o(bf[t])
    );
    assign w_go[t] = !ini_busy && wq_n[t] != 2'd0 && (w_pri[t] || !f0_rd_i);
  end
  assign bist_done_o = &bd;
  assign bist_fail_o = |bf;

  //--------------------------------------------------------------------------
  // F1 / F2
  //--------------------------------------------------------------------------
  logic             f1_ok, f2_ok;
  logic [IT_NT-1:0] f1_tok, f2_tok;
  logic [IT_TW-1:0] f1_tag [IT_NT], f2_tag [IT_NT];
  ent_t             f2_e   [IT_NT];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      f1_ok <= 1'b0; f2_ok <= 1'b0; f1_tok <= '0; f2_tok <= '0;
    end else begin
      if (f0_sent_i) begin f1_ok <= !ini_busy; f1_tok <= f0_rok; end
      if (f1_move_i) begin f2_ok <= f1_ok; f2_tok <= f1_tok; end
    end
  end
  // F1 进站那一拍寄存宏输出 (F1 停住时早读会冲掉宏输出)
  logic          f1_new;
  logic [EW-1:0] f1_hold [IT_NT];
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) f1_new <= 1'b0; else f1_new <= f0_sent_i;
  always_ff @(posedge clk_i) begin
    if (f0_sent_i) for (int t = 0; t < IT_NT; t++) f1_tag[t] <= f0_tag[t];
    if (f1_new) for (int t = 0; t < IT_NT; t++) f1_hold[t] <= rdata[t];
    if (f1_move_i) for (int t = 0; t < IT_NT; t++) begin
      f2_e[t] <= ent_t'(f1_new ? rdata[t] : f1_hold[t]); f2_tag[t] <= f1_tag[t];
    end
  end

  always_comb begin
    logic [IT_NT-1:0] hit;
    logic             ahit;
    logic [38:1]      atgt;
    it_info_t         inf;
    inf = '0; ahit = 1'b0; atgt = '0;
    for (int t = 0; t < IT_NT; t++) begin
      hit[t] = f2_ok && f2_tok[t] && f2_e[t].v && f2_e[t].tag == f2_tag[t];
      // 读被写抢掉的表：报"有效且有用"，训练不往这里分配
      inf.uv[t] = !f2_tok[t] ? 2'b11 : {f2_ok && f2_e[t].v, f2_e[t].u};
    end
    // 表号从低到高：后命中的覆盖先命中的成为提供者，被覆盖的是替补
    for (int t = 0; t < IT_NT; t++)
      if (hit[t]) begin
        if (inf.phit) begin ahit = 1'b1; atgt = inf.ptgt; end
        inf.phit = 1'b1; inf.pt = 2'(t); inf.pctr = f2_e[t].ctr; inf.pu = f2_e[t].u; inf.ptgt = f2_e[t].tgt;
      end
    f2_info_o = inf;
    f2_v_o    = inf.phit && (inf.pctr != 2'd0 || ahit);
    f2_tgt_o  = (inf.pctr != 2'd0) ? inf.ptgt : atgt;
  end

  //--------------------------------------------------------------------------
  // 训练
  //--------------------------------------------------------------------------
  wr_t  wr_r_n [IT_NT];
  logic wr_v_n [IT_NT];
  always_comb begin
    it_info_t i;
    logic     done;
    int       st;
    i = tr_info_i;
    for (int t = 0; t < IT_NT; t++) begin
      wr_r_n[t] = '0;
      wr_r_n[t].addr = i_idx(tr_pc_i, tr_fold_i, t);
      wr_v_n[t] = 1'b0;
    end
    if (tr_v_i) begin
      // 提供者
      if (i.phit) begin
        ent_t d, m;
        d = '0; m = '0;
        if (i.ptgt == tr_act_i) begin
          d.ctr = (i.pctr == 2'd3) ? 2'd3 : i.pctr + 2'd1; m.ctr = '1;
          if (!tr_mis_i) begin d.u = 1'b1; m.u = 1'b1; end
        end else if (i.pctr != 2'd0) begin
          d.ctr = i.pctr - 2'd1; m.ctr = '1;
        end else begin
          d.ctr = 2'd1; d.tgt = tr_act_i; m.ctr = '1; m.tgt = '1;
        end
        for (int t = 0; t < IT_NT; t++)
          if (2'(t) == i.pt) begin wr_r_n[t].data = d; wr_r_n[t].mask = m; wr_v_n[t] = 1'b1; end
      end
      // 分配
      if (tr_mis_i && !(i.phit && i.pt == 2'(IT_NT - 1))) begin
        st = i.phit ? int'(i.pt) + 1 : 0;
        done = 1'b0;
        for (int t = 0; t < IT_NT; t++)
          if (t >= st && !done && (!i.uv[t][1] || !i.uv[t][0])) begin
            ent_t d;
            d = '{v: 1'b1, tag: i_tag(tr_pc_i, tr_fold_i, t), ctr: 2'd0, u: 1'b0, tgt: tr_act_i};
            wr_r_n[t].data = d; wr_r_n[t].mask = '1; wr_v_n[t] = 1'b1;
            done = 1'b1;
          end
        if (!done)
          for (int t = 0; t < IT_NT; t++)
            if (t >= st && i.uv[t][1] && i.uv[t][0]) begin
              ent_t m;
              m = '0; m.u = 1'b1;
              wr_r_n[t].data = '0; wr_r_n[t].mask = m; wr_v_n[t] = 1'b1;
            end
      end
    end
  end

  logic [31:0] wq_drop;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wq_drop <= '0;
      for (int t = 0; t < IT_NT; t++) begin wr_v[t] <= 1'b0; wq_n[t] <= '0; end
    end else begin
      for (int t = 0; t < IT_NT; t++) begin
        logic [1:0] n;
        wr_v[t] <= wr_v_n[t] && !ini_busy;
        n = wq_n[t];
        if (w_go[t]) begin wq[t][0] <= wq[t][1]; n = n - 2'd1; end
        if (wr_v[t]) begin
          if (n == 2'd2) wq_drop <= wq_drop + 1'b1;
          else begin wq[t][n[0]] <= wr_r[t]; n = n + 2'd1; end
        end
        wq_n[t] <= n;
      end
    end
  end
  always_ff @(posedge clk_i) for (int t = 0; t < IT_NT; t++) wr_r[t] <= wr_r_n[t];

`ifndef SYNTHESIS
  longint st_tr = 0, st_hit = 0, st_ok = 0;
  always_ff @(posedge clk_i)
    if (rst_ni && tr_v_i) begin
      st_tr++;
      if (tr_info_i.phit) st_hit++;
      if (!tr_mis_i) st_ok++;
    end
  final $display("[perf] ITTAGE 训练 %0d 次间接跳转 (有提供者 %0d，最终预测对 %0d)，写队列满丢弃 %0d",
                 st_tr, st_hit, st_ok, wq_drop);
`endif
endmodule
