//==============================================================================
// o6_tage.sv - 分支预测器 v2 的 TAGE 部分 (带标签表；bimodal 与历史寄存器在前端)
//
// 结构 (参数见 o6_pkg 的 BP_*，评估见 docs/08_branch_predictor.md)：
//   BP_NT 张带标签表，每张 2^BP_LG 行 x 2 路；每路 {有效, 标签, 块内槽号, 3 位计数器, 2 位有用位}，
//   "标签 + 槽号"区分同一取指块里的不同分支。
//   每张表按索引最低位分成两个体 (单口 SRAM，512 行 x 44 位)：F0 读一个体，训练写入可以同拍写另一个体；
//   同体冲突的写入进每表 2 项的写队列。队列满时写入优先、抢掉这一拍该表的读 (这次预测该表按不命中算)：
//   紧循环里 F0 每拍都在读的正是要训练的那一行，读一直优先的话写永远出不去 (t41：CoreMark 误预测翻倍)。
//
// 时序 (与原触发器版 TAGE 的节拍相同，结果仍在 F3 使用)：
//   F0：用 (块 PC, 折叠历史) 一级异或算出索引 / 标签，读 SRAM (同步读，F1 出数据)
//   F1：SRAM 读出；F1 -> F2 时把读出行与标签寄存
//   F2：比较标签、逐槽选提供者 / 替补，算出最终方向 (含"新项弱时用替补") —— 组合输出，前端寄存进 F3
//   训练：FTQ 项释放时，前端送来该块的 PC、折叠历史快照、两个候选分支的预测信息与实际方向，
//         以及预测时读到的各表两路 {v, u}；本模块算出各表的写 (提供者计数器 / 有用位、误预测时分配、
//         分配失败时衰减有用位)，寄存一拍后进写队列。不做有用位的周期老化 (模型评估：不老化不变差)。
//   上电：512 拍把所有行清零 (SRAM 上电内容随机)；清零完成前读出的行视为不命中。
//==============================================================================
`default_nettype none
module o6_tage
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // ---- 预测
  input  wire                 f0_rd_i,          // 本拍读表 (F0 可能发；早于 f0_sent，多读无害)
  input  wire                 f0_sent_i,        // 本拍 F0 -> F1
  input  wire  [63:0]         f0_pc_i,
  input  var bp_fold_t        f0_fold_i,
  input  wire                 f1_move_i,        // 本拍 F1 -> F2
  input  wire  [15:0]         f2_base_i,        // F2 块的 bimodal 方向 (每槽)
  output bp_slot_t [15:0]     f2_slot_o,        // F2 每槽结果 (组合)
  output logic [BP_NT-1:0][1:0][2:0] f2_uv_o,   // F2 各表两路的 {有效, 有用位}

  // ---- 训练
  input  wire                 tr_v_i,
  input  wire  [63:0]         tr_pc_i,
  input  var bp_fold_t        tr_fold_i,
  input  wire  [1:0]          tr_cv_i,          // 候选分支有效且已解析
  input  wire  [3:0]          tr_cslot_i [2],
  input  var bp_slot_t        tr_cinfo_i [2],
  input  wire  [1:0]          tr_ctk_i,         // 实际方向
  input  wire  [BP_NT-1:0][1:0][2:0] tr_uv_i,

  // ---- MBIST
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int RW = 2 * 22;            // 一行两路
  localparam int BA = BP_LG - 1;         // 体内地址宽度

  // 一路的位段：[21] 有效 [20:9] 标签 [8:5] 槽号 [4:2] 计数器 [1:0] 有用位
  typedef struct packed {
    logic              v;
    logic [BP_TW-1:0]  tag;
    logic [3:0]        slot;
    logic [2:0]        ctr;
    logic [1:0]        u;
  } way_t;

  //--------------------------------------------------------------------------
  // 索引 / 标签散列 (bpsim 的 v2 按同一公式)
  //--------------------------------------------------------------------------
  function automatic logic [BP_LG-1:0] t_idx(input logic [63:0] pc, input bp_fold_t f, input int t);
    logic [33:0] a, s;
    a = pc[38:5];
    s = a >> (11 + t);
    return a[9:0] ^ a[19:10] ^ s[9:0] ^ f.fi[t];
  endfunction
  function automatic logic [BP_TW-1:0] t_tag(input logic [63:0] pc, input bp_fold_t f, input int t);
    logic [33:0] a, s;
    a = pc[38:5];
    s = a >> 11;
    return a[11:0] ^ s[11:0] ^ f.g1[t] ^ {f.g2[t], 1'b0};
  endfunction

  //--------------------------------------------------------------------------
  // 上电清零
  //--------------------------------------------------------------------------
  logic          ini_busy;
  logic [BA-1:0] ini_a;
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
    logic          bank;
    logic [BA-1:0] addr;
    logic [RW-1:0] data;
    logic [RW-1:0] mask;
  } wr_t;
  wr_t        wq    [BP_NT][2];
  logic [1:0] wq_n  [BP_NT];
  logic       w_go  [BP_NT];      // 本拍队头写出
  // 训练算出的写请求 (寄存一拍再入队)
  logic       wr_v  [BP_NT];
  wr_t        wr_r  [BP_NT];

  //--------------------------------------------------------------------------
  // SRAM：每表两体
  //--------------------------------------------------------------------------
  logic [BP_LG-1:0] f0_idx [BP_NT];
  logic [BP_TW-1:0] f0_tag [BP_NT];
  always_comb begin
    for (int t = 0; t < BP_NT; t++) begin
      f0_idx[t] = t_idx(f0_pc_i, f0_fold_i, t);
      f0_tag[t] = t_tag(f0_pc_i, f0_fold_i, t);
    end
  end

  logic [RW-1:0] rdata [BP_NT][2];
  logic [BP_NT*2-1:0] bd, bf;
  logic [BP_NT-1:0] w_pri;          // 写队列满：写优先
  logic [BP_NT-1:0] f0_rok;         // 本拍该表的读没被写抢掉
  for (genvar t = 0; t < BP_NT; t++) begin : g_t
    assign w_pri[t]  = wq_n[t] == 2'd2;
    assign f0_rok[t] = !(w_pri[t] && f0_idx[t][0] == wq[t][0].bank);
    for (genvar b = 0; b < 2; b++) begin : g_b
      wire rd  = f0_rd_i && !ini_busy && f0_idx[t][0] == 1'(b) && f0_rok[t];
      wire wr  = !rd && w_go[t] && wq[t][0].bank == 1'(b);
      o6_sram_sp #(.DEPTH(1 << BA), .WIDTH(RW), .STYLE("block")) u_m (
        .clk_i,
        .ce_i   (ini_busy || rd || wr),
        .we_i   (ini_busy || !rd),
        .addr_i (ini_busy ? ini_a : rd ? f0_idx[t][BP_LG-1:1] : wq[t][0].addr),
        .wdata_i(ini_busy ? '0 : wq[t][0].data),
        .wmask_i(ini_busy ? '1 : wq[t][0].mask),
        .rdata_o(rdata[t][b]),
        .bist_start_i, .bist_done_o(bd[2*t+b]), .bist_fail_o(bf[2*t+b])
      );
    end
    // 队头写出：本拍该体没被读，或队列已满 (抢读)
    assign w_go[t] = !ini_busy && wq_n[t] != 2'd0 &&
                     (w_pri[t] || !(f0_rd_i && f0_idx[t][0] == wq[t][0].bank));
  end
  assign bist_done_o = &bd;
  assign bist_fail_o = |bf;

  //--------------------------------------------------------------------------
  // F1 / F2 寄存
  //--------------------------------------------------------------------------
  logic             f1_bank [BP_NT];
  logic [BP_TW-1:0] f1_tag  [BP_NT];
  logic             f1_ok, f2_ok;     // 读的时候清零已完成
  logic [BP_NT-1:0] f1_tok, f2_tok;   // 各表的读有效 (没被写抢掉)
  way_t             f2_way  [BP_NT][2];
  logic [BP_TW-1:0] f2_tag  [BP_NT];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      f1_ok <= 1'b0; f2_ok <= 1'b0; f1_tok <= '0; f2_tok <= '0;
    end else begin
      if (f0_sent_i) begin f1_ok <= !ini_busy; f1_tok <= f0_rok; end
      if (f1_move_i) begin f2_ok <= f1_ok; f2_tok <= f1_tok; end
    end
  end
  // F1 进站那一拍宏输出就是本块的读结果，寄存下来：F1 停住时 F0 仍按早读使能读表，宏输出会被冲掉
  logic          f1_new;
  logic [RW-1:0] f1_hold [BP_NT];
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) f1_new <= 1'b0; else f1_new <= f0_sent_i;
  always_ff @(posedge clk_i) begin
    if (f0_sent_i)
      for (int t = 0; t < BP_NT; t++) begin f1_bank[t] <= f0_idx[t][0]; f1_tag[t] <= f0_tag[t]; end
    if (f1_new)
      for (int t = 0; t < BP_NT; t++) f1_hold[t] <= rdata[t][f1_bank[t]];
    if (f1_move_i) begin
      for (int t = 0; t < BP_NT; t++) begin
        logic [RW-1:0] r;
        r = f1_new ? rdata[t][f1_bank[t]] : f1_hold[t];
        f2_way[t][0] <= way_t'(r[21:0]);
        f2_way[t][1] <= way_t'(r[43:22]);
        f2_tag[t]    <= f1_tag[t];
      end
    end
  end

  //--------------------------------------------------------------------------
  // F2：逐槽选提供者 / 替补
  //--------------------------------------------------------------------------
  logic signed [3:0] altna;          // "新分配的弱项改用替补"的选择计数器
  logic [1:0]        hitw [BP_NT];
  always_comb begin
    for (int t = 0; t < BP_NT; t++)
      for (int w = 0; w < 2; w++) begin
        hitw[t][w]   = f2_ok && f2_tok[t] && f2_way[t][w].v && f2_way[t][w].tag == f2_tag[t];
        // 读被抢掉的表：报"有效且有用"，训练不往这里分配
        f2_uv_o[t][w] = !f2_tok[t] ? 3'b111 : {f2_ok && f2_way[t][w].v, f2_way[t][w].u};
      end
  end
  always_comb begin
    for (int sl = 0; sl < 16; sl++) begin
      bp_slot_t r;
      logic     ahit, adir_t, wk;
      r = '0; ahit = 1'b0; adir_t = 1'b0;
      // 表号从低到高：后命中的覆盖先命中的成为提供者，被覆盖的那个就是替补 (次长命中)
      for (int t = 0; t < BP_NT; t++)
        for (int w = 0; w < 2; w++)
          if (hitw[t][w] && f2_way[t][w].slot == 4'(sl)) begin
            if (r.phit) begin ahit = 1'b1; adir_t = r.pctr[2]; end
            r.phit = 1'b1; r.pt = 3'(t); r.pw = 1'(w);
            r.pctr = f2_way[t][w].ctr; r.pu = f2_way[t][w].u;
          end
      r.adir = ahit ? adir_t : f2_base_i[sl];
      wk   = (r.pctr == 3'd3 || r.pctr == 3'd4) && r.pu == 2'd0;
      r.fdir = !r.phit ? f2_base_i[sl] : (wk && altna >= 0) ? r.adir : r.pctr[2];
      r.tdir = r.fdir;          // 前端再用 SC 修正 fdir
      f2_slot_o[sl] = r;
    end
  end

  //--------------------------------------------------------------------------
  // 训练
  //--------------------------------------------------------------------------
  logic [15:0] lfsr;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) lfsr <= 16'hACE1;
    else         lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

  function automatic logic [2:0] ctr_step(input logic [2:0] c, input logic tk);
    return tk ? ((c == 3'd7) ? 3'd7 : c + 3'd1) : ((c == 3'd0) ? 3'd0 : c - 3'd1);
  endfunction
  function automatic logic [1:0] u_inc(input logic [1:0] u); return (u == 2'd3) ? 2'd3 : u + 2'd1; endfunction
  function automatic logic [1:0] u_dec(input logic [1:0] u); return (u == 2'd0) ? 2'd0 : u - 2'd1; endfunction

  // 训练分两拍 (2026-10-22，6 GHz：原来两个候选分支串在一拍里——提供者更新 -> 更长表优先分配 ->
  // 第二个候选看第一个的占用再分配，tr_sinfo -> wr_r 44 级 / 245 ps)。
  //   TA：处理候选 0，把各表各路的写标记 / 数据 / 占用状态寄存；TB：在寄存的状态上处理候选 1，出行写请求。
  // 写请求后面本来就有写队列，多一拍只是晚一拍写进表 (只影响预测)；altna 的两次增减都在 TB 生效。
  // 每表每路的写：full = 整路 (分配)；cu = 计数器 + 有用位 (提供者)；u = 只有用位 (衰减)
  typedef struct packed {
    logic [BP_NT-1:0][1:0]      full, cu, u;
    logic [BP_NT-1:0][1:0][2:0] uvl;         // 本次训练过程中的 {v, u} (分配后标记占用)
    way_t [BP_NT-1:0][1:0]      dat;
  } tst_t;
  // 一个候选分支的训练；da = {增减有效, 增 (1) / 减 (0)} 给 altna
  function automatic tst_t tr_cand(input tst_t s0, input logic en, input bp_slot_t i, input logic tk,
                                   input logic [3:0] cslot, input logic rnd,
                                   input logic [BP_NT-1:0][BP_TW-1:0] tg, output logic [1:0] da);
    tst_t s;
    logic mis, pdir, wk, done;
    int   st, na;
    s = s0; da = 2'b00;
    if (en) begin
      mis  = i.tdir != tk;           // TAGE 自己的对错 (fdir 可能被 SC 改过)
      pdir = i.pctr[2];
      wk = (i.pctr == 3'd3 || i.pctr == 3'd4) && i.pu == 2'd0;
      // 提供者：计数器向结果走一步；与替补不同时按对错增减有用位
      if (i.phit && !(s.full[i.pt][i.pw] || s.cu[i.pt][i.pw])) begin
        s.cu[i.pt][i.pw]      = 1'b1;
        s.dat[i.pt][i.pw].ctr = ctr_step(i.pctr, tk);
        s.dat[i.pt][i.pw].u   = (pdir != i.adir) ? (pdir == tk ? u_inc(i.pu) : u_dec(i.pu)) : i.pu;
        if (wk && pdir != i.adir) da = {1'b1, i.adir == tk};
      end
      // 误预测且提供者不是最长表：在更长的表里分配至多 2 项 (随机跳过一张，减少乒乓)
      if (mis && !(i.phit && i.pt == 3'(BP_NT - 1))) begin
        st = i.phit ? int'(i.pt) + 1 : 0;
        if (st < BP_NT - 1 && rnd) st = st + 1;
        na = 0; done = 1'b0;
        for (int t = 0; t < BP_NT; t++)
          if (t >= st && na < 2) begin
            logic f, wv;
            f = 1'b0; wv = 1'b0;
            if (!s.uvl[t][0][2] || s.uvl[t][0][1:0] == 2'd0) begin f = 1'b1; wv = 1'b0; end
            else if (!s.uvl[t][1][2] || s.uvl[t][1][1:0] == 2'd0) begin f = 1'b1; wv = 1'b1; end
            if (f && !(s.full[t][wv] || s.cu[t][wv])) begin
              s.full[t][wv] = 1'b1;
              s.dat[t][wv]  = '{v: 1'b1, tag: tg[t], slot: cslot, ctr: tk ? 3'd4 : 3'd3, u: 2'd0};
              s.uvl[t][wv]  = 3'b111;            // 已占用：第二个候选不再选它
              na = na + 1; done = 1'b1;
            end
          end
        // 一项都没分配到：这些表里该行两路的有用位各减 1
        if (!done)
          for (int t = 0; t < BP_NT; t++)
            if (t >= st)
              for (int w = 0; w < 2; w++)
                if (s.uvl[t][w][2] && s.uvl[t][w][1:0] != 2'd0 && !(s.full[t][w] || s.cu[t][w] || s.u[t][w])) begin
                  s.u[t][w]       = 1'b1;
                  s.dat[t][w].u   = u_dec(s.uvl[t][w][1:0]);
                end
      end
    end
    return s;
  endfunction
  function automatic logic signed [3:0] alt_step(input logic signed [3:0] a, input logic [1:0] da);
    if (!da[1]) return a;
    return da[0] ? ((a == 4'sd7) ? a : a + 4'sd1) : ((a == -4'sd8) ? a : a - 4'sd1);
  endfunction

  // TA
  tst_t                        ta_s;
  logic [1:0]                  ta_da;
  logic [BP_NT-1:0][BP_TW-1:0] ta_tg;
  logic [BP_NT-1:0][BP_LG-1:0] ta_ti;
  always_comb begin
    tst_t s0;
    for (int t = 0; t < BP_NT; t++) begin
      ta_ti[t] = t_idx(tr_pc_i, tr_fold_i, t);
      ta_tg[t] = t_tag(tr_pc_i, tr_fold_i, t);
    end
    s0 = '0;
    s0.uvl = tr_uv_i;
    ta_s = tr_cand(s0, tr_v_i && tr_cv_i[0], tr_cinfo_i[0], tr_ctk_i[0], tr_cslot_i[0], lfsr[0], ta_tg, ta_da);
  end
  // TA -> TB 寄存
  logic                        tb_v, tb_en1, tb_tk1, tb_rnd1;
  bp_slot_t                    tb_i1;
  logic [3:0]                  tb_cs1;
  logic [1:0]                  tb_da0;
  tst_t                        tb_s;
  logic [BP_NT-1:0][BP_TW-1:0] tb_tg;
  logic [BP_NT-1:0][BP_LG-1:0] tb_ti;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin tb_v <= 1'b0; tb_en1 <= 1'b0; tb_da0 <= '0; end
    else begin
      tb_v   <= tr_v_i;
      tb_en1 <= tr_v_i && tr_cv_i[1];
      tb_da0 <= ta_da;
    end
  always_ff @(posedge clk_i) begin
    tb_s <= ta_s; tb_tg <= ta_tg; tb_ti <= ta_ti;
    tb_i1 <= tr_cinfo_i[1]; tb_tk1 <= tr_ctk_i[1]; tb_cs1 <= tr_cslot_i[1]; tb_rnd1 <= lfsr[1];
  end

  // TB
  wr_t        wr_r_n [BP_NT];
  logic       wr_v_n [BP_NT];
  logic signed [3:0] altna_n;
  always_comb begin
    tst_t       sb;
    logic [1:0] da1;
    sb = tr_cand(tb_s, tb_en1, tb_i1, tb_tk1, tb_cs1, tb_rnd1, tb_tg, da1);
    altna_n = alt_step(alt_step(altna, tb_da0), da1);
    // 合成每表的一次行写
    for (int t = 0; t < BP_NT; t++) begin
      wr_t q;
      q.bank = tb_ti[t][0];
      q.addr = tb_ti[t][BP_LG-1:1];
      q.data = {sb.dat[t][1], sb.dat[t][0]};
      for (int w = 0; w < 2; w++) begin
        way_t m;
        m = '0;
        if (sb.full[t][w]) m = '1;
        if (sb.cu[t][w])   begin m.ctr = '1; m.u = '1; end
        if (sb.u[t][w])    m.u = '1;
        q.mask[22*w +: 22] = m;
      end
      wr_r_n[t] = q;
      wr_v_n[t] = tb_v && (|q.mask);
    end
  end
  logic [31:0] wq_drop;     // 写队列满丢弃的次数 (性能统计)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      altna <= '0; wq_drop <= '0;
      for (int t = 0; t < BP_NT; t++) begin wr_v[t] <= 1'b0; wq_n[t] <= '0; end
    end else begin
      altna <= altna_n;
      for (int t = 0; t < BP_NT; t++) begin
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
  always_ff @(posedge clk_i) for (int t = 0; t < BP_NT; t++) wr_r[t] <= wr_r_n[t];

`ifndef SYNTHESIS
  // 性能统计：入队的写、被丢的写、被写抢掉的读 (表次)
  longint st_wr = 0, st_steal = 0;
  always_ff @(posedge clk_i)
    if (rst_ni && !ini_busy)
      for (int t = 0; t < BP_NT; t++) begin
        if (wr_v[t]) st_wr++;
        if (f0_sent_i && !f0_rok[t]) st_steal++;   // 只计真正取用的读
      end
  final $display("[perf] TAGE 写 %0d 次 (队列满丢弃 %0d)，读被写抢 %0d 表次", st_wr, wq_drop, st_steal);
`endif
endmodule
