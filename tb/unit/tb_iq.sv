//==============================================================================
// tb_iq.sv - o6_iq 单元 TB (docs/13 §3 L0、§16 P2「单元 TB」)
//
// 参考模型按「绝对拍号」记每个源操作数的状态，不看 RTL 的毒化位怎么存：
//   源 = 忙 / 已唤醒 (带若干待定判定 (k, d)：load 流水 k 在第 d 拍的 L2 判定)；
//   唤醒的待定判定：sa -> d = 本拍 + 2，sn -> + 1，lwake 的 sb -> + 3；sc 本拍判定，失败则这条唤醒作废；
//   判定 (k, d) 失败 -> 带它的源重新变忙，带它发射出去的 uop 作废 (要再发一次)；
//   入队口另外听 cwake (确认，清忙不带判定) 与 ewake (load 的 L1 唤醒，sn)；入队时的 src_busy = 「上一拍及以前没有确认唤醒」
// 检查：
//   I1 每拍各端口的发射 == 模型按规格选出的 (端口 0：它能执行的最老就绪项；端口 1：只能走端口 1 的最老就绪项，
//      没有则两边都能走的最年轻就绪项，与端口 0 同一项则让出；NP = 1：最老就绪项)
//      就绪 = 已入队 (下一拍起)、不在等判定、三个源都已唤醒、wait_head 项要上拍是 ROB 头且本拍 head_go
//   I2 发射 uop 的内容 (rob_idx / pdst / 源 / fu / br_mask (已清掉解析过的位) / port) 与入队时一致
//   I3 发射 uop 的毒化位 pois / poia / poib == 模型里该 uop 各源待定判定在本拍 / 下拍 / 再下拍的流水集合
//   I4 free_cnt_o == SIZE - 上拍占用 - 上拍入队 (冲刷后 SIZE)，入队从不超过它
//   I5 每条 uop 最终恰好有效发射一次 (作废的会重发；被杀 / 冲刷的不再发)；收尾排空后队列为空
// 环境：外部生产者 (其它端口的 ALU、长延迟、浮点)、3 条 load 流水 (L0 投机唤醒 sa、L1 ewake、L2 判定 + cwake；
//   失败后重发或晚些非投机唤醒；部分 load 在 AG 级发 lwake)；本队列发出的 ALU / BRU uop 在发射当拍按毒化位广播唤醒
//   (sc = pois，sn = poia；带 sn 的下一拍判定后补发确认唤醒，同 o6_core 的 awk_q)，其余 fu 在有效发射后按延迟非投机唤醒；
//   分支标签分配 / 解析 / 误预测杀；冲刷；ROB 头 = 最老的未完成 uop，head_go 随机
// 两个实例：u2 = SIZE 48、NP 2 (IQ_B 形：端口 0 ALU / BRU / MUL，端口 1 ALU / MUL / DIV / CSR)，接 lwake；
//           u1 = SIZE 16、NP 1 (常满)
//   +cycles=N (默认 100000)
//==============================================================================
`default_nettype none
class iq_uop;
  int     seq, enq_c, lat;
  o6_pkg::uop_t u;
  bit     busy[3];
  int     deps[3][$];        // 每源待定判定，编码 d * 4 + k
  bit     pend;              // 已发出、在等判定
  int     pdeps[$];
  int     iss_c;
  bit     lw_ok;             // rs1 可以被 lwake 唤醒 (只给不写寄存器的 uop：它发射时的 poib 没有对应的确认路径)
endclass

module iq_env
  import o6_pkg::*;
#(parameter int SIZE = 48, parameter int NP = 2, parameter int PBASE = 2, parameter bit LW = 1)
 (input wire clk, input wire rst_n, input wire drain, output int errs, output int n_done);
  localparam int ENQ = IQ_ENQ_MAX;
  localparam int NIP = 128;           // 整数 / 浮点各用 p0..p127

  logic                    flush = 0;
  br_update_t              bu = '0;
  logic [ENQ-1:0]          enq_v = '0;
  uop_t [ENQ-1:0]          enq_u = '0;
  logic [$clog2(SIZE+1)-1:0] free_cnt;
  wakeup_t [N_IWAKE-1:0]   iwake = '0;
  wakeup_t [N_FWAKE-1:0]   fwake = '0;
  wakeup_t [N_CWAKE-1:0]   cwake = '0;
  wakeup_t [N_LDP-1:0]     ewake = '0;
  lwake_t  [N_LDP-1:0]     lwake = '0;
  logic [N_LDP-1:0]        ld_fail = '0;
  rob_ptr_t                rob_head = '0, head2 = '0;
  logic                    head_go = 0;
  logic [15:0]             fu_ok [NP];
  logic [NP-1:0]           iss_v;
  uop_t [NP-1:0]           iss_u;

  o6_iq #(.SIZE(SIZE), .NP(NP), .PBASE(PBASE), .N_IWAKE(N_IWAKE), .N_FWAKE(N_FWAKE)) dut (
    .clk_i(clk), .rst_ni(rst_n), .flush_i(flush), .br_upd_i(bu),
    .enq_valid_i(enq_v), .enq_uop_i(enq_u), .free_cnt_o(free_cnt),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(ewake), .lwake_i(lwake), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(head2), .head2_go_i(1'b0),
    .fu_ok_i(fu_ok), .fu_ok_nx_i(fu_ok), .iss_valid_o(iss_v), .iss_uop_o(iss_u));

  // 端口能力：NP = 2 时 端口 0 = ALU / BRU / MUL，端口 1 = ALU / MUL / DIV / CSR；NP = 1 只有 ALU / CSR
  initial begin
    if (NP == 2) begin
      fu_ok[0] = (16'(1) << FU_ALU) | (16'(1) << FU_BRU) | (16'(1) << FU_MUL);
      fu_ok[NP-1] = (16'(1) << FU_ALU) | (16'(1) << FU_MUL) | (16'(1) << FU_DIV) | (16'(1) << FU_CSR);
    end else fu_ok[0] = (16'(1) << FU_ALU) | (16'(1) << FU_CSR);
  end

  //--------------------------------------------------------------------------
  // 模型状态
  //--------------------------------------------------------------------------
  iq_uop live[$];                 // 在队列里的 (年龄序)
  iq_uop inflt[$];                // 已入队、未完成、未被杀的 (ROB 头 = 其中最老的)
  int    conf_t [2][NIP];         // 确认唤醒的拍号 (-1 = 未确认)
  bit    used   [2][NIP];         // 已分配 (有生产者或已就绪)
  int    quar   [2][NIP];         // 释放后隔离到这一拍 (防止刚被杀的 uop 当拍广播的唤醒混进新生产者)
  int    seq_n, cyc;
  // 每条 load 流水：第 d 拍判定的 load (pdst，是否失败，是否发 lwake)
  int    ld_p [N_LDP][int];
  bit    ld_f [N_LDP][int];
  bit    ld_l [N_LDP][int];
  // 外部生产者的定时唤醒：拍号 -> 若干 {类, preg}
  int    ext_w [int][$];
  // 带 sn 的本队列唤醒：下一拍判定后补发确认
  int    awk_p [NP];  logic [N_LDP-1:0] awk_m [NP];  bit awk_v [NP];  iq_uop awk_x [NP];
  // 分支标签
  int    tags[$];                 // 在途标签 (分派序)
  bit    tag_used [MAX_BR];
  int    head_prev;
  int    blk_p = -1, blk_until = 0;   // 拥塞阶段：新 uop 的 rs1 都等同一个长延迟寄存器，队列被填满
  int    occ_prev, enq_prev;  bit flush_prev;
  // ROB 指针编码：序号 mod 1024 拆成 {flag, row, col} (IQ 只比相等)
  function automatic rob_ptr_t rp(int s);
    logic [9:0] v;
    v = 10'(s);
    return rob_ptr_t'(v);
  endfunction

  function automatic void err(string s);
    errs++;
    if (errs <= 30) $display("[%0t] iq_env S%0d/NP%0d 错误：%s", $time, SIZE, NP, s);
  endfunction

  //--------------------------------------------------------------------------
  // 覆盖率
  //--------------------------------------------------------------------------
  int  c_niss, c_nenq, c_occ, c_p1, c_void, c_ewk, c_kill, c_scv, c_refail;
  bit  c_wh, c_fl, c_full;
  covergroup cg_iq @(posedge clk);
    option.per_instance = 1;
    cp_niss: coverpoint c_niss { bins n[] = {[0:NP]}; }
    cp_nenq: coverpoint c_nenq { bins n[] = {[0:ENQ]}; }
    cp_occ:  coverpoint c_occ  { bins empty = {0}; bins low = {[1:SIZE/2]}; bins high = {[SIZE/2+1:SIZE-1]}; bins full = {SIZE}; }
    cp_full: coverpoint c_full { bins y = {1}; }                                  // free_cnt = 0：分派被挡
    cp_void: coverpoint c_void { bins pois = {1}; bins poia = {2}; bins poib = {3}; ignore_bins nolw = {LW ? 99 : 3}; }   // 作废发射：判定在发射当拍 / 下拍 / 再下拍 (poib 只来自 lwake)
    cp_ewk:  coverpoint c_ewk  { bins cw = {1}; bins ew = {2}; bins iw = {3}; bins fw = {4}; bins lw = {5}; ignore_bins nolw = {LW ? 99 : 5}; }  // 入队当拍被哪种唤醒接住
    cp_scv:  coverpoint c_scv  { bins y = {1}; }                                  // sc 唤醒被本拍失败作废
    cp_refail: coverpoint c_refail { bins y = {1}; }                              // 等待中的项源被判定失败重新置忙
    cp_wh:   coverpoint c_wh   { bins y = {1}; }                                  // wait_head 项发射
    cp_kill: coverpoint c_kill { bins victims = {1}; bins enq_drop = {2}; bins pend = {3}; }        // 杀掉队列里的 / 当拍入队的 / 等判定的
    cp_fl:   coverpoint c_fl   { bins y = {1}; }                                  // 队列非空时冲刷
  endgroup
  cg_iq cg = new();
  // 端口 1 的三种选法：只在 NP = 2 的实例上建
  covergroup cg_p1 @(posedge clk);
    option.per_instance = 1;
    cp_p1:   coverpoint c_p1 { bins excl = {1}; bins yng = {2}; bins yield_ = {3}; }
  endgroup
  cg_p1 cgp;
  initial if (NP == 2) cgp = new();

  //--------------------------------------------------------------------------
  // 工具
  //--------------------------------------------------------------------------
  function automatic int alloc_preg(int cls);
    for (int t = 0; t < 400; t++) begin
      int p;
      p = $urandom_range(32, NIP - 1);
      if (!used[cls][p] && quar[cls][p] < cyc) begin used[cls][p] = 1; conf_t[cls][p] = -1; return p; end
    end
    return -1;
  endfunction
  function automatic void free_preg(int cls, int p);
    used[cls][p] = 0; quar[cls][p] = cyc + 20;
  endfunction
  function automatic int pick_src(int cls);
    // 偏向最近分配的忙寄存器 (造依赖)，也取已就绪的
    for (int t = 0; t < 50; t++) begin
      int p;
      p = $urandom_range(0, NIP - 1);
      if (used[cls][p]) return p;
    end
    return $urandom_range(0, 31);
  endfunction
  function automatic bit elig(int p, iq_uop x);
    return fu_ok[p][x.u.fu];
  endfunction
  function automatic bit rdy(iq_uop x);
    if (x.enq_c >= cyc || x.pend) return 0;
    for (int s = 0; s < 3; s++) if (x.busy[s]) return 0;
    if (x.u.wait_head && !(head_go && x.enq_c <= cyc - 2 && head_prev == x.seq)) return 0;
    return 1;
  endfunction
  function automatic bit is_ld_preg(int p);   // 是某条在途 load 的结果 (lwake 的对象)
    for (int k = 0; k < N_LDP; k++) foreach (ld_p[k][d]) if (ld_p[k][d] == p && ld_l[k][d]) return 1;
    return 0;
  endfunction

  // 一个源在本拍被唤醒：w_* 是本拍的唤醒列表 (类、preg、判定集合、是否 lwake、是否被 sc 作废)
  int    wv_cls[$], wv_p[$], wv_kind[$];  bit wv_void[$];  int wv_deps[$][$];
  function automatic void add_wake(int cls, int p, int deps[$], bit voided, int kind);
    wv_cls.push_back(cls); wv_p.push_back(p); wv_void.push_back(voided); wv_kind.push_back(kind);
    wv_deps.push_back(deps);
  endfunction
  // kind：0 = iwake / fwake (表项与入队口都听)，1 = ewake (只入队口)，2 = lwake (只 rs1)，3 = cwake (只入队口，确认)
  function automatic bit hit(iq_uop x, int s, bit at_enq, output int deps[$], output int kind);
    int cls, p;
    bit got;
    got = 0; deps.delete(); kind = 0;
    cls = (s == 2 || (s == 1 && x.u.rs2_class == RC_FP)) ? 1 : 0;
    p   = (s == 0) ? x.u.psrc1 : (s == 1) ? x.u.psrc2 : x.u.psrc3;
    if (at_enq) foreach (wv_p[i]) if (wv_kind[i] == 3 && wv_cls[i] == cls && wv_p[i] == p) begin kind = 3; return 1; end
    foreach (wv_p[i]) begin
      if (wv_cls[i] != cls || wv_p[i] != p || wv_void[i]) continue;
      if (wv_kind[i] == 3) continue;
      if (wv_kind[i] == 1 && !at_enq) continue;
      if (wv_kind[i] == 2 && (s != 0 || cls != 0)) continue;
      foreach (wv_deps[i][j]) deps.push_back(wv_deps[i][j]);
      kind = wv_kind[i] == 2 ? 2 : wv_kind[i] == 1 ? 1 : (cls ? 4 : 0);
      got = 1;
    end
    return got;
  endfunction
  function automatic bit failed(int dep);
    int k, d;
    k = dep % 4; d = dep / 4;
    return d == cyc && ld_fail[k];
  endfunction

  //--------------------------------------------------------------------------
  // 主循环：negedge 看 DUT 本拍输出、定本拍输入，并把模型推进到下一拍
  //--------------------------------------------------------------------------
  initial begin
    errs = 0; n_done = 0;
    for (int c = 0; c < 2; c++) for (int p = 0; p < NIP; p++) begin
      used[c][p] = p < 32; conf_t[c][p] = p < 32 ? -100 : -1; quar[c][p] = -1;
    end
    head_prev = 0; occ_prev = 0; enq_prev = 0; flush_prev = 0;
    @(posedge rst_n);
    forever begin
      @(negedge clk);
      step();
    end
  end

  task automatic step();
      iq_uop gx [NP];
      int    occ_now;
      cyc++;
      c_niss = 0; c_nenq = 0; c_p1 = 0; c_void = 0; c_ewk = 0; c_kill = 0; c_scv = 0; c_refail = 0; c_wh = 0; c_fl = 0;
      occ_now = live.size();
      c_occ = occ_now;
      // ---- I4：空位数 ----
      begin
        int e;
        e = flush_prev ? SIZE : SIZE - occ_prev - enq_prev;
        if (e < 0) e = 0;
        if (free_cnt != e) err($sformatf("I4 free_cnt %0d != 期望 %0d", free_cnt, e));
        c_full = free_cnt == 0;
      end
      // ---- 本拍的 load 判定 / ROB 头 / head_go (选择要用) ----
      for (int k = 0; k < N_LDP; k++) ld_fail[k] = ld_p[k].exists(cyc) ? ld_f[k][cyc] : 1'b0;
      head_go = $urandom_range(99) < 70;
      // 分支解析 / 误预测杀 / 冲刷 (先定：本拍的确认唤醒要按它门控)
      bu = '0; flush = 0;
      if (!drain && $urandom_range(999) < 4 && (live.size() > 0)) begin flush = 1; c_fl = 1; end
      else if (tags.size() > 0 && $urandom_range(99) < 12) begin
        int ti, t;
        ti = $urandom_range(tags.size() - 1); t = tags[ti];
        bu.resolve_mask[t] = 1'b1;
        if ($urandom_range(99) < 30) begin bu.kill = 1; bu.kill_tag = BR_TAG_W'(t); end
      end
      #1;    // head_go 组合进选择：等它稳定再看发射
      // ---- I1：模型选择 ----
      for (int p = 0; p < NP; p++) gx[p] = null;
      foreach (live[i]) if (rdy(live[i]) && elig(0, live[i])) begin gx[0] = live[i]; break; end
      if (NP == 2) begin
        iq_uop ex, yg;
        ex = null; yg = null;
        foreach (live[i]) if (rdy(live[i]) && elig(1, live[i]) && !elig(0, live[i])) begin ex = live[i]; break; end
        for (int i = live.size() - 1; i >= 0; i--) if (rdy(live[i]) && elig(1, live[i])) begin yg = live[i]; break; end
        if (ex != null) begin gx[1] = ex; c_p1 = 1; end
        else if (yg != null && yg != gx[0]) begin gx[1] = yg; c_p1 = 2; end
        else if (yg != null) c_p1 = 3;
      end
      for (int p = 0; p < NP; p++) begin
        iq_uop x;
        x = null;
        if (iss_v[p]) foreach (live[i]) if (live[i].u.rob_idx == iss_u[p].rob_idx) x = live[i];
        if (iss_v[p] != (gx[p] != null) || (x != gx[p]))
          err($sformatf("I1 端口 %0d：RTL %s，模型 %s", p,
                        iss_v[p] ? (x == null ? $sformatf("发了不在队列的 rob %0d", iss_u[p].rob_idx) : $sformatf("seq %0d", x.seq)) : "不发",
                        gx[p] == null ? "不发" : $sformatf("seq %0d", gx[p].seq)));
        if (x == null) continue;
        // I2 / I3
        begin
          uop_t a, b;
          logic [N_LDP-1:0] po, pa, pb;
          a = iss_u[p]; b = x.u;
          if (a.pdst != b.pdst || a.psrc1 != b.psrc1 || a.psrc2 != b.psrc2 || a.psrc3 != b.psrc3 || a.fu != b.fu ||
              a.wait_head != b.wait_head || a.br_mask != b.br_mask || a.port != PORT_W'(PBASE + p))
            err($sformatf("I2 端口 %0d seq %0d 内容不符 (br_mask %h / %h，port %0d)", p, x.seq, a.br_mask, b.br_mask, a.port));
          po = '0; pa = '0; pb = '0;
          x.pdeps.delete();
          for (int s = 0; s < 3; s++) foreach (x.deps[s][j]) begin
            int d, k;
            d = x.deps[s][j] / 4; k = x.deps[s][j] % 4;
            if (d < cyc) err($sformatf("模型：seq %0d 源 %0d 有过期判定", x.seq, s));
            if (d == cyc) po[k] = 1; else if (d == cyc + 1) pa[k] = 1; else if (d == cyc + 2) pb[k] = 1;
            else err($sformatf("I3 seq %0d 判定在 %0d 拍后", x.seq, d - cyc));
            x.pdeps.push_back(x.deps[s][j]);
          end
          if (a.pois != po || a.poia != pa || a.poib != pb)
            err($sformatf("I3 seq %0d 毒化位 pois/poia/poib = %b/%b/%b，期望 %b/%b/%b", x.seq, a.pois, a.poia, a.poib, po, pa, pb));
          x.pend = 1; x.iss_c = cyc;
          if (x.u.wait_head) c_wh = 1;
          c_niss++;
        end
      end

      // ---- 本拍输入 ----
      iwake = '0; fwake = '0; cwake = '0; ewake = '0; lwake = '0; enq_v = '0; enq_u = '0;
      wv_cls.delete(); wv_p.delete(); wv_void.delete(); wv_deps.delete(); wv_kind.delete();
      // load 流水
      for (int k = 0; k < N_LDP; k++) begin
        int nodeps[$];
        int dd[$];
        // 新 load：3 拍后判定
        if (!drain && !ld_p[k].exists(cyc + 3) && $urandom_range(99) < 30) begin
          int p;
          p = alloc_preg(0);
          if (p >= 0) begin ld_p[k][cyc + 3] = p; ld_f[k][cyc + 3] = $urandom_range(99) < 25; ld_l[k][cyc + 3] = $urandom_range(99) < 50; end
        end
        // lwake (判定前 3 拍)、L0 投机唤醒 (前 2 拍)、L1 ewake (前 1 拍)、L2 判定 + cwake
        if (ld_p[k].exists(cyc + 3) && ld_l[k][cyc + 3] && LW) begin
          lwake[k] = '{valid: 1'b1, pdst: preg_t'(ld_p[k][cyc + 3]), sb: N_LDP'(1) << k, sc: '0};
          dd = '{(cyc + 3) * 4 + k}; add_wake(0, ld_p[k][cyc + 3], dd, 0, 2);
        end
        if (ld_p[k].exists(cyc + 2)) begin
          iwake[N_ALU + 1 + k] = '{valid: 1'b1, is_fp: 1'b0, pdst: preg_t'(ld_p[k][cyc + 2]), sa: N_LDP'(1) << k, sn: '0, sc: '0};
          dd = '{(cyc + 2) * 4 + k}; add_wake(0, ld_p[k][cyc + 2], dd, 0, 0);
        end
        if (ld_p[k].exists(cyc + 1)) begin
          ewake[k] = '{valid: 1'b1, is_fp: 1'b0, pdst: preg_t'(ld_p[k][cyc + 1]), sa: '0, sn: N_LDP'(1) << k, sc: '0};
          dd = '{(cyc + 1) * 4 + k}; add_wake(0, ld_p[k][cyc + 1], dd, 0, 1);
        end
        if (ld_p[k].exists(cyc)) begin
          int p;
          p = ld_p[k][cyc];
          if (!ld_f[k][cyc]) begin
            cwake[k] = '{valid: 1'b1, is_fp: 1'b0, pdst: preg_t'(p), sa: '0, sn: '0, sc: '0};
            add_wake(0, p, nodeps, 0, 3);
            conf_t[0][p] = cyc;
          end else if ($urandom_range(1)) ext_w[cyc + $urandom_range(1, 25)].push_back(p);       // 缺失回填：晚些非投机唤醒
          else begin                                                                                // 重发：再走一次投机
            int k2, d2;
            k2 = $urandom_range(N_LDP - 1); d2 = cyc + $urandom_range(4, 12);
            if (!ld_p[k2].exists(d2)) begin ld_p[k2][d2] = p; ld_f[k2][d2] = $urandom_range(99) < 25; ld_l[k2][d2] = 0; end
            else ext_w[d2].push_back(p);
          end
        end
      end
      // 外部生产者的定时唤醒 (非投机)：整数用槽 0 / 1 / 6 (本队列的端口 PBASE.. 不用)，浮点 fwake 0..5
      if (ext_w.exists(cyc)) begin
        int ni, nf;
        int nodeps[$];
        ni = 0; nf = 0;
        foreach (ext_w[cyc][j]) begin
          int v, cls, p;
          v = ext_w[cyc][j]; cls = v >= 1000; p = v % 1000;
          if (!used[cls][p]) continue;
          if (cls == 0 && ni < 3) begin
            int slot;
            slot = ni == 0 ? 0 : ni == 1 ? 1 : N_ALU;
            iwake[slot] = '{valid: 1'b1, is_fp: 1'b0, pdst: preg_t'(p), sa: '0, sn: '0, sc: '0};
            add_wake(0, p, nodeps, 0, 0); conf_t[0][p] = cyc; ni++;
          end else if (cls == 1 && nf < N_FWAKE) begin
            fwake[nf] = '{valid: 1'b1, is_fp: 1'b1, pdst: preg_t'(p), sa: '0, sn: '0, sc: '0};
            add_wake(1, p, nodeps, 0, 0); conf_t[1][p] = cyc; nf++;
          end else ext_w[cyc + 1].push_back(v);
        end
        ext_w.delete(cyc);
      end
      // 上拍带 sn 发射的本队列 ALU 唤醒：本拍判定后补发确认 (cwake[N_LDP + 端口])
      for (int p = 0; p < NP; p++) if (awk_v[p]) begin
        awk_v[p] = 0;
        if (!(|(awk_m[p] & ld_fail)) && awk_x[p] != null && inflt_has(awk_x[p]) && !flush &&
            !(bu.kill && awk_x[p].u.br_mask[bu.kill_tag])) begin
          int nodeps[$];
          cwake[N_LDP + PBASE + p] = '{valid: 1'b1, is_fp: 1'b0, pdst: preg_t'(awk_p[p]), sa: '0, sn: '0, sc: '0};
          add_wake(0, awk_p[p], nodeps, 0, 3);
          conf_t[0][awk_p[p]] = cyc;
        end
      end
      // 本拍发射的 ALU / BRU：发射当拍按毒化位广播 (sc = pois，sn = poia)
      for (int p = 0; p < NP; p++) if (iss_v[p] && iss_u[p].rd_class == RC_INT && (iss_u[p].fu == FU_ALU || iss_u[p].fu == FU_BRU)) begin
        int dd[$];
        bit vd;
        iwake[PBASE + p] = '{valid: 1'b1, is_fp: 1'b0, pdst: iss_u[p].pdst, sa: iss_u[p].poib, sn: iss_u[p].poia, sc: iss_u[p].pois};
        vd = |(iss_u[p].pois & ld_fail);
        for (int k = 0; k < N_LDP; k++) if (iss_u[p].poia[k]) dd.push_back((cyc + 1) * 4 + k);
        for (int k = 0; k < N_LDP; k++) if (iss_u[p].poib[k]) dd.push_back((cyc + 2) * 4 + k);
        add_wake(0, iss_u[p].pdst, dd, vd, 0);
        if (vd) c_scv = 1;
        if (!vd && iss_u[p].poia == '0) conf_t[0][iss_u[p].pdst] = cyc;
        else if (!vd) begin
          awk_v[p] = !flush && !(bu.kill && iss_u[p].br_mask[bu.kill_tag]); awk_p[p] = iss_u[p].pdst; awk_m[p] = iss_u[p].poia;
          foreach (live[i]) if (live[i].u.rob_idx == iss_u[p].rob_idx) awk_x[p] = live[i];
        end
      end

      // 拥塞阶段 (每 2500 拍一次，300 拍)
      if (!drain && cyc % 2500 == 100) begin
        blk_p = alloc_preg(0);
        if (blk_p >= 0) begin ext_w[cyc + 400].push_back(blk_p); blk_until = cyc + 300; end
      end
      // 入队
      if (!flush && !drain && $urandom_range(99) < 75) begin
        int n;
        n = $urandom_range(1, ENQ);
        if (n > free_cnt) n = free_cnt;
        for (int j = 0; j < n; j++) begin
          iq_uop x;
          uop_t u;
          x = new; u = '0;
          case ($urandom_range(9))
            0, 1, 2, 3: u.fu = FU_ALU;
            4:          u.fu = NP == 2 ? FU_BRU : FU_ALU;
            5, 6:       u.fu = NP == 2 ? FU_MUL : FU_ALU;
            7:          u.fu = NP == 2 ? FU_DIV : FU_ALU;
            8:          u.fu = FU_CSR;
            default:    u.fu = FU_ALU;
          endcase
          u.wait_head = u.fu == FU_CSR || $urandom_range(99) < 2;
          x.lat = u.fu == FU_MUL ? 3 : u.fu == FU_DIV ? $urandom_range(8, 30) : 1;
          u.rs1_class = $urandom_range(9) < 8 ? RC_INT : RC_NONE;
          case ($urandom_range(9)) 0, 1, 2, 3, 4: u.rs2_class = RC_INT; 5, 6: u.rs2_class = RC_FP; default: u.rs2_class = RC_NONE; endcase
          u.rs3_en = $urandom_range(9) == 0;
          u.rd_class = $urandom_range(9) < 7 ? RC_INT : RC_NONE;
          x.lw_ok = u.rd_class == RC_NONE;
          if (u.rs1_class == RC_INT) begin
            int p;
            p = pick_src(0);
            if (!x.lw_ok) for (int t = 0; t < 20 && is_ld_preg(p); t++) p = pick_src(0);
            if (!x.lw_ok && is_ld_preg(p)) p = 1;
            if (cyc < blk_until && blk_p >= 0) p = blk_p;
            u.psrc1 = preg_t'(p);
          end
          if (u.rs2_class != RC_NONE) u.psrc2 = preg_t'(pick_src(u.rs2_class == RC_FP));
          if (u.rs3_en) u.psrc3 = preg_t'(pick_src(1)); else u.psrc3 = preg_t'($urandom_range(0, 31));
          if (u.rd_class == RC_INT) begin
            int p;
            p = alloc_preg(0);
            if (p < 0) begin u.rd_class = RC_NONE; x.lw_ok = 1; end else u.pdst = preg_t'(p);
          end
          u.src1_busy = u.rs1_class == RC_INT && !(conf_t[0][u.psrc1] != -1 && conf_t[0][u.psrc1] <= cyc - 1);
          u.src2_busy = u.rs2_class != RC_NONE && !(conf_t[u.rs2_class == RC_FP][u.psrc2] != -1 && conf_t[u.rs2_class == RC_FP][u.psrc2] <= cyc - 1);
          u.src3_busy = u.rs3_en && !(conf_t[1][u.psrc3] != -1 && conf_t[1][u.psrc3] <= cyc - 1);
          foreach (tags[i]) u.br_mask[tags[i]] = 1'b1;
          x.seq = seq_n++; u.rob_idx = rp(x.seq);
          x.u = u; x.enq_c = cyc;
          x.busy[0] = u.src1_busy; x.busy[1] = u.src2_busy; x.busy[2] = u.src3_busy;
          enq_v[j] = 1'b1; enq_u[j] = u;
          // 被本拍的杀命中：不入队
          if (bu.kill && u.br_mask[bu.kill_tag]) begin c_kill = 2; if (u.rd_class == RC_INT) free_preg(0, u.pdst); end
          else begin live.push_back(x); inflt.push_back(x); end
          c_nenq++;
          // 外部生产者：新的忙寄存器，定时唤醒 (整数 / 浮点)
          if ($urandom_range(99) < 30) begin
            int cls, p;
            cls = $urandom_range(1);
            p = alloc_preg(cls);
            if (p >= 0) ext_w[cyc + $urandom_range(1, 40)].push_back(cls * 1000 + p);
          end
        end
        // 新分支标签 (之后入队的 uop 都带它)
        if ($urandom_range(99) < 25) begin
          for (int t = 0; t < MAX_BR; t++) if (!tag_used[t] && !bu.resolve_mask[t]) begin tag_used[t] = 1; tags.push_back(t); break; end
        end
      end
      // 收尾排空时还要让外部唤醒继续推进：给所有没人唤醒的忙寄存器安排唤醒
      if (drain && cyc % 50 == 0) begin
        foreach (live[i]) for (int s = 0; s < 3; s++) if (live[i].busy[s]) begin
          int cls, p;
          cls = (s == 2 || (s == 1 && live[i].u.rs2_class == RC_FP)) ? 1 : 0;
          p = (s == 0) ? live[i].u.psrc1 : (s == 1) ? live[i].u.psrc2 : live[i].u.psrc3;
          if (!has_producer(cls, p)) ext_w[cyc + 1].push_back(cls * 1000 + p);
        end
      end

      // ---- 模型推进到下一拍 ----
      // 1) 已在队列里的项：判定失败 -> 源重新置忙；否则吃本拍唤醒；判定到期的去掉
      foreach (live[i]) begin
        iq_uop x;
        x = live[i];
        if (x.enq_c == cyc) continue;
        for (int s = 0; s < 3; s++) begin
          int dd[$];
          int kind;
          bit f;
          f = 0;
          foreach (x.deps[s][j]) if (failed(x.deps[s][j])) f = 1;
          if (f) begin x.busy[s] = 1; x.deps[s].delete(); if (!x.pend) c_refail = 1; end
          else if (x.busy[s] && hit(x, s, 0, dd, kind)) begin x.busy[s] = 0; x.deps[s] = dd; end
          else if (!x.busy[s]) begin
            int keep[$];
            foreach (x.deps[s][j]) if (x.deps[s][j] / 4 > cyc) keep.push_back(x.deps[s][j]);
            x.deps[s] = keep;
          end
        end
      end
      // 2) 本拍入队的：先听 cwake，再按表项规则加上 ewake
      foreach (live[i]) if (live[i].enq_c == cyc) begin
        iq_uop x;
        x = live[i];
        for (int s = 0; s < 3; s++) if (x.busy[s]) begin
          int dd[$];
          int kind;
          if (hit(x, s, 1, dd, kind)) begin
            x.busy[s] = 0; x.deps[s] = dd;
            c_ewk = kind == 3 ? 1 : kind == 1 ? 2 : kind == 2 ? 5 : kind == 4 ? 4 : 3;
          end
        end
      end
      // 3) 发射的结局：判定失败作废；判定都过了 -> 完成 (出队)
      foreach (live[i]) if (live[i].pend) begin
        iq_uop x;
        bit f;
        int keep[$];
        x = live[i];
        f = 0;
        foreach (x.pdeps[j]) if (failed(x.pdeps[j])) f = 1;
        if (f) begin
          int dmin;
          dmin = 99;
          foreach (x.pdeps[j]) if (failed(x.pdeps[j])) dmin = x.pdeps[j] / 4 - x.iss_c;
          c_void = dmin + 1;
          x.pend = 0; x.pdeps.delete();
          if (x.u.rd_class == RC_INT) conf_t[0][x.u.pdst] = -1;       // 它当拍的唤醒作废 / 将被判失败
        end else begin
          foreach (x.pdeps[j]) if (x.pdeps[j] / 4 > cyc) keep.push_back(x.pdeps[j]);
          x.pdeps = keep;
        end
      end
      begin
        iq_uop keep_l[$];
        foreach (live[i]) begin
          iq_uop x;
          x = live[i];
          if (x.pend && x.pdeps.size() == 0 && !(flush || (bu.kill && x.u.br_mask[bu.kill_tag]))) begin
            // 有效发射完成：非单拍 fu 在延迟后非投机唤醒
            n_done++;
            if (x.u.rd_class == RC_INT && !(x.u.fu == FU_ALU || x.u.fu == FU_BRU)) ext_w[cyc + x.lat].push_back(x.u.pdst);
            inflt_del(x);
          end else keep_l.push_back(x);
        end
        live = keep_l;
      end
      // 4) 分支解析 / 杀 / 冲刷
      if (bu.kill || flush) begin
        iq_uop keep_l[$];
        foreach (live[i]) begin
          if (flush || live[i].u.br_mask[bu.kill_tag]) begin
            if (live[i].pend) c_kill = 3; else if (c_kill == 0) c_kill = 1;
            kill_uop(live[i]);
          end else keep_l.push_back(live[i]);
        end
        live = keep_l;
        // 不在队列但还在途的 (已发射完成的不在 inflt 里) 不存在；inflt 里同步删掉被杀的
        begin
          iq_uop k2[$];
          foreach (inflt[i]) if (!(flush || inflt[i].u.br_mask[bu.kill_tag])) k2.push_back(inflt[i]);
          inflt = k2;
        end
      end
      if (flush) begin
        foreach (tags[i]) tag_used[tags[i]] = 0;
        tags.delete();
        for (int p = 0; p < NP; p++) awk_v[p] = 0;
      end else if (bu.resolve_mask != '0) begin
        int t, ti;
        int k2[$];
        t = -1;
        foreach (tags[i]) if (bu.resolve_mask[tags[i]]) begin t = tags[i]; ti = i; end
        foreach (live[i]) live[i].u.br_mask[t] = 1'b0;
        foreach (inflt[i]) inflt[i].u.br_mask[t] = 1'b0;
        if (bu.kill) begin   // 被杀分支之后分配的标签一并回收
          for (int i = 0; i < tags.size(); i++) if (i < ti) k2.push_back(tags[i]); else tag_used[tags[i]] = 0;
        end else begin
          foreach (tags[i]) if (i != ti) k2.push_back(tags[i]);
          tag_used[t] = 0;
        end
        tags = k2;
      end
      // 5) ROB 头 (下一拍用)、I4 用的占用 / 入队
      head_prev = inflt.size() > 0 ? inflt[0].seq : seq_n;
      rob_head = rp(head_prev);
      occ_prev = occ_now; enq_prev = c_nenq; flush_prev = flush;
      // 看门狗：源都确认了很久还不发
      foreach (live[i]) if (cyc - live[i].enq_c == 3000)
        err($sformatf("I5 seq %0d 入队 3000 拍还没发 (busy %0d%0d%0d pend %0d wh %0d)", live[i].seq,
                      live[i].busy[0], live[i].busy[1], live[i].busy[2], live[i].pend, live[i].u.wait_head));
      // 回收：已确认很久、没有在队列里的 uop 引用的物理寄存器
      if (cyc % 64 == 0) gc();
      for (int k = 0; k < N_LDP; k++) if (ld_p[k].exists(cyc)) begin ld_p[k].delete(cyc); ld_f[k].delete(cyc); ld_l[k].delete(cyc); end
  endtask

  function automatic void gc();
    bit ref_ [2][NIP];
    foreach (live[i]) begin
      uop_t u;
      u = live[i].u;
      ref_[0][u.psrc1] = 1; ref_[u.rs2_class == RC_FP][u.psrc2] = 1; ref_[1][u.psrc3] = 1;
      if (u.rd_class == RC_INT) ref_[0][u.pdst] = 1;
    end
    for (int c = 0; c < 2; c++) for (int p = 32; p < NIP; p++)
      if (used[c][p] && !ref_[c][p] && conf_t[c][p] != -1 && conf_t[c][p] < cyc - 50) free_preg(c, p);
  endfunction

  function automatic bit inflt_has(iq_uop x);
    foreach (inflt[i]) if (inflt[i] == x) return 1;
    return 0;
  endfunction
  function automatic void inflt_del(iq_uop x);
    foreach (inflt[i]) if (inflt[i] == x) begin inflt.delete(i); return; end
  endfunction
  function automatic void kill_uop(iq_uop x);
    if (x.u.rd_class == RC_INT) free_preg(0, x.u.pdst);
  endfunction
  function automatic bit has_producer(int cls, int p);
    if (conf_t[cls][p] != -1) return 1;
    foreach (ext_w[c]) foreach (ext_w[c][j]) if (ext_w[c][j] == cls * 1000 + p) return 1;
    if (cls == 0) for (int k = 0; k < N_LDP; k++) foreach (ld_p[k][d]) if (ld_p[k][d] == p) return 1;
    if (cls == 0) foreach (live[i]) if (live[i].u.rd_class == RC_INT && live[i].u.pdst == p) return 1;
    return 0;
  endfunction

  // 收尾：排空后队列应为空
  function automatic void final_check();
    if (live.size() != 0) err($sformatf("I5 排空后队列里还有 %0d 项 (最老 seq %0d busy %0d%0d%0d pend %0d wh %0d)", live.size(),
                                        live[0].seq, live[0].busy[0], live[0].busy[1], live[0].busy[2], live[0].pend, live[0].u.wait_head));
    if (free_cnt != SIZE) err($sformatf("I5 排空后 free_cnt = %0d", free_cnt));
  endfunction
endmodule

module tb_iq;
  logic clk = 0, rst_n = 0, drain = 0;
  int   e2, e1, d2, d1, ncyc;
  always #5 clk = ~clk;
  iq_env #(.SIZE(48), .NP(2), .PBASE(2), .LW(1)) u2 (.clk, .rst_n, .drain, .errs(e2), .n_done(d2));
  iq_env #(.SIZE(16), .NP(1), .PBASE(4), .LW(0)) u1 (.clk, .rst_n, .drain, .errs(e1), .n_done(d1));
  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 100000;
    repeat (3) @(posedge clk);
    rst_n = 1;
    repeat (ncyc) @(posedge clk);
    drain = 1;
    repeat (2000) @(posedge clk);
    @(negedge clk);
    u2.final_check(); u1.final_check();
    if (e2 + e1 == 0) $display("=== UNIT PASS === iq %0d 拍，发射完成 %0d / %0d", ncyc, d2, d1);
    else              $display("=== UNIT FAIL === iq 错误 %0d / %0d", e2, e1);
    $finish;
  end
endmodule
