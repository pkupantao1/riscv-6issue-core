//==============================================================================
// tb_dcache.sv - o6_dcache 单元 TB (docs/13 §5.6；MSHR 合并 / 回填 / 写回 / AMO / fence.i)
//
// TB 同时扮演 LSU 与 L2：
//   golden：按字节的「架构内存」；backing：L2 / 内存里的内容
//   load (3 口，L0 发 -> L1 给物理地址、随机 kill -> L2 看 hit / nack / miss)：发出时记下 golden 里那个双字，
//     命中时必须相等；缺失等该 MSHR 回填后重发；nack 下一拍重发
//   store / AMO：SQ 头排空协议 (请求保持到 done、done 那拍出队、st_next_paddr 给下一条)，done 时更新 golden；
//     AMO 的旧值必须等于 golden；不给与 SQ 里的 store 同双字的 load (LSU 本来会从 SQ 转发)
//   store 预取：随机地址
//   L2：请求按到达顺序处理 (写回立即写 backing；读在请求时取快照)，随机延迟、乱序返回
//   fence.i：周期性停发、清空后 flush_req 到 flush_done，之后 backing 必须逐字节等于 golden (所有脏行都写回了)
// 地址：3 个组 x 24 个标签 (> 16 路)，替换 / 脏写回频繁
//   +cycles=N (默认 100000)
//==============================================================================
`default_nettype none
module tb_dcache;
  import o6_pkg::*;
  localparam int NLD = N_LDP, NM = DC_NMSHR, NT = 24;
  localparam logic [PA_W-1:0] BASE = PA_W'(64'h8000_0000);
  localparam int SETS[3] = '{5, 6, 7};

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic [NLD-1:0]  ld_req = '0, ld_kill = '0;
  logic [PA_W-1:0] ld_addr [NLD], ld_paddr [NLD];
  logic [NLD-1:0]  ld_hit, ld_nack;
  logic [63:0]     ld_data [NLD];
  logic [MSHR_W-1:0] ld_mshr [NLD];
  logic            refill;
  logic [MSHR_W-1:0] refill_mshr;
  logic            st_req = 0, st_amo = 0;
  logic [PA_W-1:0] st_paddr = '0, st_next = '0;
  logic [63:0]     st_data = '0, st_rdata;
  logic [7:0]      st_mask = '0;
  logic [4:0]      st_op = '0;
  logic [1:0]      st_size = '0;
  logic            st_done;
  logic [511:0]    st_lrdata;
  logic            pf_req = 0, flush_req = 0, flush_done;
  logic [PA_W-1:0] pf_paddr = '0;
  logic            mreq_v, mreq_rdy = 0, mreq_we, mresp_v = 0, mresp_rdy;
  logic [PA_W-1:6] mreq_addr;
  logic [511:0]    mreq_data, mresp_data = '0;
  logic [3:0]      mreq_id, mresp_id = '0;
  logic            bist_done, bist_fail;

  o6_dcache dut (
    .clk_i(clk), .rst_ni(rst_n),
    .ld_req_i(ld_req), .ld_addr_i(ld_addr), .ld_paddr_i(ld_paddr), .ld_kill_i(ld_kill),
    .ld_hit_o(ld_hit), .ld_nack_o(ld_nack), .ld_data_o(ld_data), .ld_mshr_o(ld_mshr),
    .refill_o(refill), .refill_mshr_o(refill_mshr),
    .st_req_i(st_req), .st_paddr_i(st_paddr), .st_next_paddr_i(st_next), .st_lk_paddr_i(st_paddr),
    .st_data_i(st_data), .st_mask_i(st_mask), .st_amo_i(st_amo), .st_amo_op_i(st_op), .st_size_i(st_size),
    .st_done_o(st_done), .st_rdata_o(st_rdata),
    .st_line_i(1'b0), .st_ldata_i('0), .st_lmask_i('0), .st_lrdata_o(st_lrdata),
    .pf_req_i(pf_req), .pf_paddr_i(pf_paddr), .flush_req_i(flush_req), .flush_done_o(flush_done),
    .mem_req_valid_o(mreq_v), .mem_req_ready_i(mreq_rdy), .mem_req_we_o(mreq_we), .mem_req_addr_o(mreq_addr),
    .mem_req_data_o(mreq_data), .mem_req_id_o(mreq_id),
    .mem_resp_valid_i(mresp_v), .mem_resp_ready_o(mresp_rdy), .mem_resp_id_i(mresp_id), .mem_resp_data_i(mresp_data),
    .bist_start_i(1'b0), .bist_done_o(bist_done), .bist_fail_o(bist_fail));

  //--------------------------------------------------------------------------
  // 内存模型
  //--------------------------------------------------------------------------
  logic [7:0] golden [logic [PA_W-1:0]];
  logic [7:0] backing [logic [PA_W-1:0]];
  function automatic logic [PA_W-1:0] mk(int s, int t, int off);
    return BASE + PA_W'(t * 4096 + SETS[s] * 64 + off);
  endfunction
  function automatic logic [63:0] g_dw(logic [PA_W-1:0] a);   // golden 里 a 所在双字
    logic [63:0] r;
    for (int b = 0; b < 8; b++) r[8*b +: 8] = golden[{a[PA_W-1:3], 3'b000} + b];
    return r;
  endfunction
  function automatic logic [511:0] b_line(logic [PA_W-1:6] l);
    logic [511:0] r;
    for (int b = 0; b < 64; b++) r[8*b +: 8] = backing.exists({l, 6'd0} + b) ? backing[{l, 6'd0} + b] : 8'h00;
    return r;
  endfunction

  int errs, cyc, ncyc;
  function automatic void err(string s);
    errs++;
    if (errs <= 25) $display("[%0t] tb_dcache 错误：%s", $time, s);
  endfunction

  //--------------------------------------------------------------------------
  // load 口状态
  //--------------------------------------------------------------------------
  typedef struct { bit v; logic [PA_W-1:0] a; logic [63:0] exp; bit kill; int t0; } ld_t;
  ld_t p1 [NLD], p2 [NLD];          // L1 / L2 级
  typedef struct { int m; logic [PA_W-1:0] a; int t; } wt_t;
  wt_t wq[$];                       // 等某个 MSHR 回填的 load (不占口：口可以继续发新的)
  logic [PA_W-1:0] rpq[$];          // 待重发 (nack / 回填到了)
  bit  burst;                       // 拥塞阶段：L2 延迟长、预取多、load 密 -> MSHR 全满
  // store 队列
  typedef struct { logic [PA_W-1:0] a; logic [63:0] d; logic [7:0] m; bit amo; logic [4:0] op; logic [1:0] sz; } st_t;
  st_t sq[$];
  // L2 读请求
  typedef struct { int id; logic [511:0] d; int t; } rd_t;
  rd_t rq[$];
  bit  quiet;                         // fence.i 前停发
  logic [2:0] s_st_q = '0;
  int smiss_n = 0;

  // 覆盖率采样量
  bit c_hit, c_miss, c_nack, c_merge, c_full, c_wb, c_sthit, c_stmiss, c_amo, c_pf, c_same, c_flushd, c_ooo, c_kill;
  covergroup cg_dc @(posedge clk);
    cp_hit:    coverpoint c_hit    { bins y = {1}; }
    cp_miss:   coverpoint c_miss   { bins y = {1}; }
    cp_nack:   coverpoint c_nack   { bins y = {1}; }   // bank 冲突 / MSHR 满
    cp_merge:  coverpoint c_merge  { bins y = {1}; }   // 缺失合并到已有 MSHR
    cp_full:   coverpoint c_full   { bins y = {1}; }   // MSHR 全满
    cp_wb:     coverpoint c_wb     { bins y = {1}; }   // 脏行写回
    cp_sthit:  coverpoint c_sthit  { bins y = {1}; }
    cp_stmiss: coverpoint c_stmiss { bins y = {1}; }   // store 缺失挂 MSHR、回填后重试
    cp_amo:    coverpoint c_amo    { bins y = {1}; }
    cp_pf:     coverpoint c_pf     { bins y = {1}; }   // store 预取分配
    cp_same:   coverpoint c_same   { bins y = {1}; }   // store 排空缺失与同行 load 缺失同拍 (#18)
    cp_flushd: coverpoint c_flushd { bins y = {1}; }   // fence.i 写回脏行
    cp_ooo:    coverpoint c_ooo    { bins y = {1}; }   // L2 乱序返回
    cp_kill:   coverpoint c_kill   { bins y = {1}; }
  endgroup
  cg_dc cg = new();

  function automatic logic [PA_W-1:0] rand_addr(int sz);
    return mk($urandom_range(2), $urandom_range(NT - 1), ($urandom_range(63) >> sz) << sz);
  endfunction
  function automatic bit sq_conflict(logic [PA_W-1:0] a);
    foreach (sq[i]) if (sq[i].a[PA_W-1:3] == a[PA_W-1:3]) return 1;
    return 0;
  endfunction

  //--------------------------------------------------------------------------
  // 主循环
  //--------------------------------------------------------------------------
  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 100000;
    errs = 0;
    for (int s = 0; s < 3; s++) for (int t = 0; t < NT; t++) for (int o = 0; o < 64; o++) begin
      logic [7:0] v;
      v = $urandom;
      golden[mk(s, t, o)] = v; backing[mk(s, t, o)] = v;
    end
    for (int k = 0; k < NLD; k++) begin p1[k].v = 0; p2[k].v = 0; ld_addr[k] = '0; ld_paddr[k] = '0; end
    repeat (3) @(posedge clk);
    rst_n = 1;
    for (int i = 0; i < ncyc; i++) begin
      @(negedge clk);
      step();
      if (i % 6000 == 5999) do_flush();
    end
    do_flush();
    if (errs == 0) $display("=== UNIT PASS === dcache %0d 拍", ncyc);
    else           $display("=== UNIT FAIL === dcache 错误 %0d", errs);
    $finish;
  end

  task automatic step();
    cyc++;                            // 拍号在每拍 step 里推进 (fence.i 等待里也要走，否则 L2 响应永远不到期)
    c_hit = 0; c_miss = 0; c_nack = 0; c_merge = 0; c_wb = 0; c_sthit = 0; c_stmiss = 0; c_amo = 0; c_pf = 0;
    c_same = 0; c_ooo = 0; c_kill = 0;
    c_full = 1; for (int i = 0; i < NM; i++) if (dut.m_st[i] == 0) c_full = 0;
    // ---- store 完成：done 那拍出队、更新 golden、核对 AMO 旧值 ----
    if (st_done) begin
      st_t s;
      s = sq.pop_front();
      if (s.amo) begin
        logic [63:0] old, a, b, r;
        int sh;
        old = g_dw(s.a);
        if (st_rdata !== old) err($sformatf("AMO %h 旧值 %h != golden %h", s.a, st_rdata, old));
        sh = 8 * s.a[2:0];
        a = old >> sh; b = s.d >> sh;
        if (s.sz == 2) begin a = {{32{a[31]}}, a[31:0]}; b = {{32{b[31]}}, b[31:0]}; end
        r = (s.op == AMO_ADD) ? a + b : b;
        r = r << sh;
        for (int i = 0; i < 8; i++) if (s.m[i]) golden[{s.a[PA_W-1:3], 3'b000} + i] = r[8*i +: 8];
        c_amo = 1;
      end else begin
        for (int i = 0; i < 8; i++) if (s.m[i]) golden[{s.a[PA_W-1:3], 3'b000} + i] = s.d[8*i +: 8];
        c_sthit = 1;
      end
    end
    if (dut.s_st == 3'd5 && s_st_q != 3'd5) c_stmiss = 1;   // 进 S_MISS
    s_st_q = dut.s_st;
    if (dut.s_st == 3'd5) smiss_n++; else smiss_n = 0;
    if (smiss_n == 1000) begin
      string m;
      m = "";
      for (int i = 0; i < NM; i++) m = {m, $sformatf("%0d:%0d/%h ", i, dut.m_st[i], dut.m_line[i])};
      err($sformatf("store 停在 S_MISS 1000 拍：s_m=%0d spa=%h | MSHR %s| rq %0d 项 | wb_v=%0d f_st=%0d mresp_rdy=%0d cyc=%0d",
                    dut.s_m, dut.spa_q, m, rq.size(), dut.wb_v, dut.f_st, mresp_rdy, cyc));
      foreach (rq[i]) $display("    rq id=%0d t=%0d", rq[i].id, rq[i].t);
    end
    burst = (cyc / 3000) % 2 == 1;
    // ---- 回填：等它的 load 改为重发 ----
    if (refill) begin
      wt_t keep[$];
      foreach (wq[i]) if (wq[i].m == int'(refill_mshr)) rpq.push_back(wq[i].a); else keep.push_back(wq[i]);
      wq = keep;
    end
    // ---- 上拍在 L1 的 load 本拍出 L2 结果：先移进 p2 再看 ----
    for (int k = 0; k < NLD; k++) begin p2[k] = p1[k]; p1[k].v = 0; end
    // ---- load L2 结果 ----
    for (int k = 0; k < NLD; k++) if (p2[k].v) begin
      if (p2[k].kill) begin
        if (ld_hit[k] || ld_nack[k]) err($sformatf("口 %0d 被 kill 的 load 仍有结果", k));
        c_kill = 1;
      end else begin
        if (ld_hit[k] && ld_nack[k]) err($sformatf("口 %0d hit 与 nack 同时", k));
        if (ld_hit[k]) begin
          if (ld_data[k] !== p2[k].exp) err($sformatf("口 %0d load %h 读到 %h，期望 %h", k, p2[k].a, ld_data[k], p2[k].exp));
          c_hit = 1;
        end else if (ld_nack[k]) begin
          rpq.push_back(p2[k].a); c_nack = 1;
        end else begin
          // 缺失：等 ld_mshr 回填 (同拍已回填则直接重发)
          foreach (wq[j]) if (wq[j].m == int'(ld_mshr[k])) c_merge = 1;
          if (refill && refill_mshr == ld_mshr[k]) rpq.push_back(p2[k].a);
          else begin wt_t w; w.m = ld_mshr[k]; w.a = p2[k].a; w.t = cyc; wq.push_back(w); end
          if (dut.m_st[ld_mshr[k]] == 0 && !(refill && refill_mshr == ld_mshr[k]))
          begin
            string m;
            m = "";
            for (int i = 0; i < 4; i++) m = {m, $sformatf("%0d:%0d/%h ", i, dut.m_st[i], dut.m_line[i])};
            err($sformatf("口 %0d 缺失挂在空闲 MSHR %0d 上 (cyc=%0d a=%h l2_miss=%0d nack=%0d hit=%0d f_st=%0d s_st=%0d refill=%0d/%0d | %s)",
                          k, ld_mshr[k], cyc, p2[k].a, dut.l2_miss[k], ld_nack[k], ld_hit[k], dut.f_st, dut.s_st, refill, refill_mshr, m));
          end
          c_miss = 1;
        end
      end
    end
    foreach (wq[i]) if (cyc - wq[i].t == 3000) err($sformatf("load %h 等 MSHR %0d 回填超时", wq[i].a, wq[i].m));
    // ---- 上拍 L0 发出的进 L1 ----
    for (int k = 0; k < NLD; k++)
      if (ld_req[k]) begin p1[k].v = 1; p1[k].a = ld_addr[k]; p1[k].exp = '0; end
    // L0 发出的那拍记期望值 (见上：发出后的 store 写入晚于它读 SRAM)
    for (int k = 0; k < NLD; k++) if (p1[k].v) p1[k].exp = g_dw(p1[k].a);
    // L1 级输入：物理地址、随机 kill
    for (int k = 0; k < NLD; k++) begin
      ld_paddr[k] = p1[k].a;
      ld_kill[k]  = p1[k].v && $urandom_range(99) < 4;
      p1[k].kill  = ld_kill[k];
    end
    // L0：先重发，再发新的；口 0 偶尔对准「SQ 头的 store 下一拍查标签」发同一行的 load (#18：两者缺失同拍分配)
    for (int k = 0; k < NLD; k++) begin
      ld_req[k] = 0;
      if (k == 0 && !quiet && sq.size() > 0 && !sq[0].amo && dut.s_st == 3'd0 && $urandom_range(99) < 40) begin
        logic [PA_W-1:0] a;
        a = {sq[0].a[PA_W-1:6], 3'(sq[0].a[5:3] + 3'd1), 3'b000};
        if (!sq_conflict(a)) begin ld_req[k] = 1; ld_addr[k] = a; end
      end
      if (!ld_req[k] && rpq.size() > 0 && !sq_conflict(rpq[0])) begin ld_req[k] = 1; ld_addr[k] = rpq.pop_front(); end
      else if (!ld_req[k] && !quiet && $urandom_range(99) < (burst ? 90 : 40)) begin
        logic [PA_W-1:0] a;
        a = rand_addr(3);
        if (!sq_conflict(a)) begin ld_req[k] = 1; ld_addr[k] = a; end
      end
    end
    // ---- store 队列 ----
    if (!quiet && sq.size() < 8 && $urandom_range(99) < 25) begin
      st_t s;
      int sz;
      sz = $urandom_range(3);
      s.a = rand_addr(sz);
      s.amo = $urandom_range(9) == 0;
      if (s.amo) begin sz = $urandom_range(2, 3); s.a = rand_addr(sz); s.op = $urandom_range(1) ? AMO_ADD : AMO_SWAP; end
      s.sz = sz;
      s.d = {$urandom, $urandom};
      s.m = 8'(((1 << (1 << sz)) - 1) << s.a[2:0]);
      // 与在途 load 同双字的不进 (那些 load 的期望值已按发出时定)
      begin
        bit c;
        c = 0;
        for (int k = 0; k < NLD; k++) begin
          if (p1[k].v && p1[k].a[PA_W-1:3] == s.a[PA_W-1:3]) c = 1;
          if (p2[k].v && p2[k].a[PA_W-1:3] == s.a[PA_W-1:3]) c = 1;
          if (ld_req[k] && ld_addr[k][PA_W-1:3] == s.a[PA_W-1:3]) c = 1;
        end
        foreach (wq[i])  if (wq[i].a[PA_W-1:3] == s.a[PA_W-1:3]) c = 1;
        foreach (rpq[i]) if (rpq[i][PA_W-1:3] == s.a[PA_W-1:3]) c = 1;
        if (!c) sq.push_back(s);
      end
    end
    st_req = sq.size() > 0;
    if (sq.size() > 0) begin
      st_paddr = sq[0].a; st_data = sq[0].d; st_mask = sq[0].m; st_amo = sq[0].amo; st_op = sq[0].op; st_size = sq[0].sz;
    end
    st_next = sq.size() > 1 ? sq[1].a : sq.size() > 0 ? sq[0].a : '0;
    // ---- 预取 ----
    pf_req = !quiet && $urandom_range(99) < (burst ? 40 : 10);
    pf_paddr = rand_addr(3);
    // ---- L2：请求 ----
    mreq_rdy = $urandom_range(99) < 80;
    #1;
    // 组合量要在本拍输入都驱动之后采 (negedge 刚到时 DUT 看到的还是上拍的输入)
    if (dut.st_miss_alloc) for (int k = 0; k < NLD; k++)
      // 修复后同行 load 直接合并到 store 本拍新分配的 MSHR (l1_alloc 为 0)，所以按「L1 缺失且同行」判
      if (dut.l1_v[k] && dut.l1_go[k] && !ld_kill[k] && !(|dut.l1_hit_oh[k]) && ld_paddr[k][PA_W-1:6] == dut.spa_q[PA_W-1:6]) c_same = 1;
    c_pf = dut.pf_alloc;
    if (mreq_v && mreq_rdy) begin
      if (mreq_we) begin
        for (int b = 0; b < 64; b++) backing[{mreq_addr, 6'd0} + b] = mreq_data[8*b +: 8];
        c_wb = 1;
        if (dut.x_st != 0) c_flushd = 1;
      end else begin
        rd_t r;
        r.id = mreq_id; r.d = b_line(mreq_addr); r.t = cyc + (burst ? $urandom_range(100, 300) : $urandom_range(5, 60));
        rq.push_back(r);
      end
    end
    // ---- L2：响应 (随机挑一个到期的) ----
    mresp_v = 0;
    if (mresp_rdy) begin
      int cand[$];
      foreach (rq[i]) if (rq[i].t <= cyc) cand.push_back(i);
      if (cand.size() > 0) begin
        int i;
        i = cand[$urandom_range(cand.size() - 1)];
        if (i != 0) c_ooo = 1;
        mresp_v = 1; mresp_id = 4'(rq[i].id); mresp_data = rq[i].d;
        rq.delete(i);
      end
    end
  endtask

  // fence.i：停发、等清空、flush 到 done，然后逐字节比 backing 与 golden
  task automatic do_flush();
    int n;
    quiet = 1;
    n = 0;
    while ((sq.size() > 0 || rq.size() > 0 || ld_req != 0 || dut.f_st != 0 ||
            p1[0].v || p1[1].v || p1[2].v || p2[0].v || p2[1].v || p2[2].v ||
            wq.size() > 0 || rpq.size() > 0) && n < 20000) begin
      @(negedge clk); step(); n++;
    end
    begin   // 预取的 MSHR 也要等回填完
      bit busy;
      busy = 1;
      while (busy && n < 40000) begin
        busy = rq.size() > 0 || dut.f_st != 0;
        for (int i = 0; i < NM; i++) if (dut.m_st[i] != 0) busy = 1;
        if (busy) begin @(negedge clk); step(); n++; end
      end
    end
    flush_req = 1;
    n = 0;
    while (!flush_done && n < 50000) begin @(negedge clk); step(); n++; end
    flush_req = 0;
    repeat (4) begin @(negedge clk); step(); end
    if (!flush_done && n >= 50000) err("fence.i 没有完成");
    foreach (golden[a]) if (backing[a] !== golden[a]) begin
      err($sformatf("fence.i 后 %h：内存 %h != golden %h", a, backing[a], golden[a]));
      break;
    end
    quiet = 0;
  endtask
endmodule
