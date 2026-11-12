//==============================================================================
// tb_lsu_fwd.sv - o6_sq_fwd (store -> load 转发搜索) 单元 TB (docs/13 §3 L0、§16 P2「单元 TB」)
//
// 随机 SQ 状态：head h、在途 n 项 (0..NSQ，含正好满)，项 [h, h+n) 有效、带正确的翻转位；
//   load 的 sq_idx = h + j (0 <= j <= n)：[h, h+j) 是比它老的，[h+j, h+n) 是比它新的 (必须忽略)；
//   无效项的地址 / 掩码 / 翻转位填垃圾 (必须忽略)。地址取自 3 个双字，store 宽度 1/2/4/8 对齐。
// 参考模型 (与实现无关)：对 load 读的每个字节，沿环从 h+j-1 往回找第一个「地址已知、同一双字、写这个字节」的 store
//   -> hit；它可转发 (fok) 则数据 = 它的字节，否则它进等待集合 wv。
// 检查 hit_o / data_o (只看 hit 且可转发的字节，其余必须为 0) 全等；wait_o = (wv 非空)，
// wsq_o = 编号最小的要等字节选中的那条 store (2026-10-27 起接口从等待集合改成这两个)。
//   +vec=N (默认 200000)。另用 VDEFS="+define+O6_SQ_SIZE=8" 编一份测 C1 配置
//==============================================================================
`default_nettype none
module tb_lsu_fwd;
  import o6_pkg::*;
  localparam int NSQ = SQ_SIZE;

  logic [NSQ-1:0]  sqv, flag, aok, fok;
  logic [PA_W-1:0] pa   [NSQ];
  logic [7:0]      mask [NSQ];
  logic [63:0]     sd   [NSQ];
  logic [PA_W-1:0] ld_pa;
  logic [7:0]      ld_mask;
  sq_ptr_t         ld_sq;
  logic [7:0]      hit;
  logic [63:0]     data;
  logic            wt;
  logic [SQ_W-1:0] wsq;

  o6_sq_fwd #(.NSQ(NSQ)) dut (
    .sqv_i(sqv), .flag_i(flag), .addr_ok_i(aok), .paddr_i(pa), .mask_i(mask), .sdata_i(sd), .fok_i(fok),
    .ld_pa_i(ld_pa), .ld_mask_i(ld_mask), .ld_sq_i(ld_sq), .hit_o(hit), .data_o(data), .wait_o(wt), .wsq_o(wsq));

  int errs, nvec;
  // 覆盖率采样量
  int c_n, c_j, c_ncand, c_sel;
  bit c_wrap, c_split, c_yng, c_wait, c_part, c_all, c_multi;
  int c_lsz;
  covergroup cg_fwd;
    cp_n:     coverpoint c_n { bins empty = {0}; bins one = {1}; bins mid = {[2:NSQ-1]}; bins full = {NSQ}; }
    cp_j:     coverpoint c_j { bins none_older = {0}; bins some = {1}; bins all = {2}; }      // load 前面没有 / 有部分 / 全部是老 store
    cp_full_all: cross cp_n, cp_j { option.cross_auto_bin_max = 0; bins full_all = binsof(cp_n.full) && binsof(cp_j.all); }  // 正好满且全部更老 (#26 那一点)
    cp_wrap:  coverpoint c_wrap { bins y = {1}; }       // 老 store 的窗口跨过下标 NSQ-1 -> 0
    cp_split: coverpoint c_split { bins y = {1}; }      // 同一字节的候选在分界两侧都有 (sq_pick 的高 / 低两段)
    cp_yng:   coverpoint c_yng { bins y = {1}; }        // 有更年轻的匹配 store (必须忽略)
    cp_multi: coverpoint c_multi { bins y = {1}; }      // 同一字节多个较老候选 (取最年轻)
    cp_wait:  coverpoint c_wait { bins y = {1}; }       // 选中的 store 数据不可用 -> 等
    cp_part:  coverpoint c_part { bins y = {1}; }       // 部分字节命中
    cp_all:   coverpoint c_all { bins y = {1}; }        // 全部字节命中
    cp_lsz:   coverpoint c_lsz { bins b[] = {[0:3]}; }  // load 宽度
    cx_lsz_part: cross cp_lsz, cp_part { ignore_bins b1 = binsof(cp_lsz) intersect {0}; }   // 1 字节不会部分命中
  endgroup
  cg_fwd cg = new();

  function automatic logic [PA_W-1:0] rand_pa(output logic [7:0] m, input int sz);
    logic [PA_W-1:0] a;
    int off;
    a = PA_W'(64'h8000_1000) + PA_W'($urandom_range(2) * 8);
    off = $urandom_range((8 >> sz) - 1) << sz;
    m = 8'(((1 << (1 << sz)) - 1) << off);
    return a + PA_W'(off);
  endfunction

  initial begin
    if (!$value$plusargs("vec=%d", nvec)) nvec = 200000;
    errs = 0;
    for (int v = 0; v < nvec; v++) begin
      int h, n, j, mode;
      logic [7:0]  e_hit;
      logic [63:0] e_data;
      logic [NSQ-1:0] e_wv;
      int             e_wsq;
      // 占用：偏向空 / 满 / 小
      mode = $urandom_range(9);
      n = mode == 0 ? 0 : mode == 1 ? NSQ : mode < 5 ? $urandom_range(1, 4) : $urandom_range(1, NSQ);
      h = $urandom_range(NSQ - 1);
      j = $urandom_range(3) == 0 ? n : $urandom_range(n);
      // 无效项：垃圾
      for (int e = 0; e < NSQ; e++) begin
        sqv[e] = 0; flag[e] = $urandom_range(1); aok[e] = $urandom_range(1); fok[e] = $urandom_range(1);
        pa[e] = rand_pa(mask[e], $urandom_range(3)); sd[e] = {$urandom, $urandom};
      end
      // 有效项 [h, h+n)：翻转位 = 跨过 NSQ 的那一圈；head 的翻转位随机
      begin
        bit hf;
        hf = $urandom_range(1);
        for (int i = 0; i < n; i++) begin
          int e;
          e = (h + i) % NSQ;
          sqv[e] = 1; flag[e] = hf ^ ((h + i) >= NSQ);
          aok[e] = $urandom_range(9) < 8; fok[e] = $urandom_range(9) < 7;
          pa[e] = rand_pa(mask[e], $urandom_range(3));
          sd[e] = {$urandom, $urandom};
        end
        // 正好满、全部更老时，一半情况下只让最老那项 (下标 = load 的 sq_idx.idx) 与 load 同双字：
        // #26 那一点 (标志不同、下标相同) 只有它是唯一候选时才看得出来，否则总有更年轻的候选把它盖住
        if (mode == 1 && $urandom_range(1))
          for (int i = 1; i < n; i++) pa[(h + i) % NSQ] = PA_W'(64'h8000_2000) + PA_W'(pa[(h + i) % NSQ][2:0]);
        // load：sq_idx = h + j；地址 / 宽度随机
        begin
          int t, lsz;
          logic [7:0] lm;
          t = h + j;
          ld_sq = '{flag: hf ^ (t >= NSQ), idx: SQ_W'(t % NSQ)};
          lsz = $urandom_range(3);
          ld_pa = rand_pa(lm, lsz); ld_mask = lm;
          c_lsz = lsz;
        end
      end
      #1;
      // ---- 参考模型 ----
      e_hit = '0; e_data = '0; e_wv = '0; e_wsq = -1;
      c_multi = 0; c_split = 0; c_yng = 0;
      for (int b = 0; b < 8; b++) if (ld_mask[b]) begin
        int ncand, first;
        bit lo, hi;
        ncand = 0; first = -1; lo = 0; hi = 0;
        for (int i = j - 1; i >= 0; i--) begin
          int e;
          e = (h + i) % NSQ;
          if (aok[e] && pa[e][PA_W-1:3] == ld_pa[PA_W-1:3] && mask[e][b]) begin
            ncand++;
            if (e < ld_sq.idx) lo = 1; else hi = 1;
            if (first < 0) first = e;
          end
        end
        for (int i = j; i < n; i++) begin
          int e;
          e = (h + i) % NSQ;
          if (aok[e] && pa[e][PA_W-1:3] == ld_pa[PA_W-1:3] && mask[e][b]) c_yng = 1;
        end
        if (ncand > 1) c_multi = 1;
        if (lo && hi) c_split = 1;
        if (first >= 0) begin
          e_hit[b] = 1;
          if (fok[first]) e_data[8*b +: 8] = sd[first][8*b +: 8];
          else begin e_wv[first] = 1; if (e_wsq < 0) e_wsq = first; end
        end
      end
      if (hit !== e_hit || data !== e_data || wt !== (e_wv != '0) || (wt && wsq != SQ_W'(e_wsq))) begin
        errs++;
        if (errs <= 20)
          $display("错误 vec %0d：h=%0d n=%0d j=%0d ld_sq=%0d/%0d mask=%b | hit %b/%b data %h/%h wait %b/%h wsq %0d/%0d", v, h, n, j,
                   ld_sq.flag, ld_sq.idx, ld_mask, hit, e_hit, data, e_data, wt, e_wv, wsq, e_wsq);
      end
      // ---- 覆盖率 ----
      c_n = n; c_j = j == 0 ? 0 : j == n ? 2 : 1;
      c_wrap = j > 0 && (h + j - 1) >= NSQ;
      c_wait = e_wv != '0;
      c_part = e_hit != '0 && e_hit != ld_mask;
      c_all  = e_hit == ld_mask;
      cg.sample();
    end
    if (errs == 0) $display("=== UNIT PASS === lsu_fwd NSQ=%0d %0d 组向量", NSQ, nvec);
    else           $display("=== UNIT FAIL === lsu_fwd 错误 %0d", errs);
    $finish;
  end
endmodule
