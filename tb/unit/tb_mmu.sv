//==============================================================================
// tb_mmu.sv - o6_ptw + o6_tlb 单元 TB (docs/13 §5.6；RVA23 R4 扩到 Sv48 / Sv57 / Svnapot / Svpbmt，docs/14)
//
// 被测：PTW + 两个 TLB (I：1 个查找口；D：2 个口；各 8 项，替换频繁)，PTW 的填写同时接两边 (按 ifill / dfill)
// 参考模型：按 Sv39 / Sv48 / Sv57 (每次建表随机选，起始级 2 / 3 / 4；PBMTE 也随机) + o6 的实现规则对同一份稀疏页表
//   做软件遍历，得到 {故障 (0 无 / 1 页错误 / 2 访问错误), 级, NAPOT, PPN, 权限, PBMT}：
//   PTE 地址不在 DRAM [0x8000_0000, +2G) 或被 PMP 拒绝 (S 态读) -> 访问错误；无效 / 只写 / 保留位 [60:54] 非零 /
//   PBMT 取 3 或 PBMTE = 0 时非零 / 非叶 D/A/U/N/PBMT / 0 级非叶 -> 页错误；超页 PPN 未对齐 / N 不在 0 级或 PPN[3:0] != 1000
//   -> 页错误；叶子 PPN 超出 PA_W -> 访问错误；查找时 4 级页 VA[47:40] 非零 -> 访问错误 (PA 超出 PA_W)
// 检查：
//   M1 每次 PTW 填写的 {vpn, 级, nap, ppn, 权限, pbmt, 故障} == 参考遍历
//   M2 每拍每个查找口命中时 {ppn (已拼低位), 权限, pbmt, 故障} == 参考 (超页 / NAPOT 项被同一大页里别的 VPN 命中)
//   M3 不该命中 (冲刷后) 时不命中：冲刷当拍之后两个 TLB 全空
// 页表：随机生成，覆盖上面每种结局 + 256T / 512G / 1G / 2M / 64K / 4K 叶子；运行中随机冲刷 (中断在途遍历)，
//   部分冲刷同时改写整张表 (并重抽模式)
// 两级 (H，RVA23 R5b)：每次建表一半概率为两级 —— 请求 V = 1，hgatp 随机 Sv39x4 / 48x4 / 57x4，vsatp 开 (S1 表在 GPA 空间)
//   或 Bare (VPN 就是 GPA 页号)。G-stage 表按用到的 GPA 懒建：S1 表页多为恒等映射 (偶尔随机 PA)，叶子目标映到随机 PA；
//   每级空项随机建下级表 / 超页叶子 (偶尔未对齐) / 4K 叶子；G 叶子属性随机 (U = 0、A = 0、D = 0、W 无 R、只 X、无 W、无 X、
//   无效、保留位、PBMT、坏 N)。参考遍历按 Spike (docs/15 §6)：VS PTE 地址先过 G-stage (隐式：要 R，失败 -> guest page fault、
//   GPA = PTE GPA)，最终 GPA 再过 G-stage；合并项页级取较小者。核对 PTW 填写与查找的 {故障 (含 3 = guest page fault), 级,
//   PA, VS 权限, G 级 {D, X, W, R}, GPA, 隐式标记}
// 存储口：同 LSU (ptw_own)：一次只有一个读在途，回来之前不再给 gnt；数据可能在 PTW 已放弃后才回 (必须被丢掉)
//   +cycles=N (默认 100000)
//==============================================================================
`default_nettype none
module tb_mmu;
  import o6_pkg::*;
  localparam int NTLB = 8;
  localparam logic [55:0] DRAM = 56'h8000_0000;
  localparam logic [55:0] PTB  = 56'h8010_0000;     // 页表页从这里分配
  localparam logic [55:0] FORB = 56'h80ff_f000;     // PMP 不许 S 态读的那一页 (也当页表页用)

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic        flush = 0;
  logic [43:0] satp_ppn;
  logic [1:0]  vam;
  logic        pbmte;
  // 两级
  bit          two = 0, vson = 0;
  logic [3:0]  gmode = 4'd8;
  int          gtop = 2;
  logic [55:0] groot = '0;
  logic [7:0]  pmpcfg [8];
  logic [53:0] pmpaddr [8];
  pmp_dec_t    pmpd [8];
  o6_pmp_dec #(.N(8)) u_pmpd (.cfg_i(pmpcfg), .addr_i(pmpaddr), .dec_o(pmpd));
  logic        ireq = 0, dreq = 0, busy;
  logic [VPN_W-1:0] ivpn = '0, dvpn = '0;
  logic        mreq, mgnt = 0, mvalid = 0;
  logic [PA_W-1:0] maddr;
  logic [63:0] mdata = '0;
  logic        ifill, dfill;
  logic [VPN_W-1:0] fvpn;
  logic [2:0]  flvl;
  logic [1:0]  ffault, fpbmt;
  logic        fnap, fv, fgimp;
  logic [3:0]  fgperm;
  logic [56:0] fgpa;
  logic [PA_W-13:0] fppn;
  logic [7:0]  fperm;

  o6_ptw u_ptw (
    .clk_i(clk), .rst_ni(rst_n), .flush_i(flush), .satp_ppn_i(satp_ppn), .vam_i(vam), .pbmte_i(pbmte), .pmpd_i(pmpd),
    .vsatp_ppn_i(satp_ppn), .vvam_i(vam), .vson_i(vson), .vpbmte_i(pbmte),     // 两级时 vsatp 用同一棵表
    .hgmode_i(two ? gmode : 4'd0), .hgppn_i(44'(groot >> 12)),
    .ireq_i(ireq), .ireq_vpn_i(ivpn), .ireq_v_i(two), .dreq_i(dreq), .dreq_vpn_i(dvpn), .dreq_v_i(two), .busy_o(busy),
    .mem_req_o(mreq), .mem_addr_o(maddr), .mem_gnt_i(mgnt), .mem_valid_i(mvalid), .mem_data_i(mdata),
    .ifill_o(ifill), .dfill_o(dfill), .fill_v_o(fv), .fill_vpn_o(fvpn), .fill_level_o(flvl), .fill_nap_o(fnap), .fill_ppn_o(fppn),
    .fill_perm_o(fperm), .fill_pbmt_o(fpbmt), .fill_fault_o(ffault),
    .fill_gperm_o(fgperm), .fill_gpa_o(fgpa), .fill_gimp_o(fgimp));

  logic [VPN_W-1:0] lk_vpn [3];          // [0] = I，[1..2] = D
  logic [2:0]       lk_hit;
  logic [PA_W-13:0] lk_ppn [3];
  logic [7:0]       lk_perm [3];
  logic [1:0]       lk_fault [3], lk_pbmt [3];
  logic [VPN_W-1:0] iv [1], dv [2];
  logic [0:0]       ih;  logic [1:0] dh;
  logic [PA_W-13:0] ip [1], dp [2];
  logic [7:0]       ipm [1], dpm [2];
  logic [1:0]       ifl [1], dfl [2], ipb [1], dpb [2];
  logic [3:0]       igp [1], dgp [2], lk_gperm [3];
  logic [56:0]      iga [1], dga [2], lk_gpa [3];
  logic             igi [1], dgi [2], lk_gimp [3];
  assign iv[0] = lk_vpn[0]; assign dv[0] = lk_vpn[1]; assign dv[1] = lk_vpn[2];
  o6_tlb #(.N(NTLB), .NL(1)) u_itlb (.clk_i(clk), .rst_ni(rst_n), .flush_i(flush),
    .lk_vpn_i(iv), .lk_v_i(two), .lk_hit_o(ih), .lk_ppn_o(ip), .lk_perm_o(ipm), .lk_fault_o(ifl), .lk_pbmt_o(ipb),
    .lk_gperm_o(igp), .lk_gpa_o(iga), .lk_gimp_o(igi),
    .fill_i(ifill), .fill_v_i(fv), .fill_vpn_i(fvpn), .fill_level_i(flvl), .fill_nap_i(fnap), .fill_ppn_i(fppn), .fill_perm_i(fperm),
    .fill_pbmt_i(fpbmt), .fill_fault_i(ffault), .fill_gperm_i(fgperm), .fill_gpa_i(fgpa), .fill_gimp_i(fgimp));
  o6_tlb #(.N(NTLB), .NL(2)) u_dtlb (.clk_i(clk), .rst_ni(rst_n), .flush_i(flush),
    .lk_vpn_i(dv), .lk_v_i({two, two}), .lk_hit_o(dh), .lk_ppn_o(dp), .lk_perm_o(dpm), .lk_fault_o(dfl), .lk_pbmt_o(dpb),
    .lk_gperm_o(dgp), .lk_gpa_o(dga), .lk_gimp_o(dgi),
    .fill_i(dfill), .fill_v_i(fv), .fill_vpn_i(fvpn), .fill_level_i(flvl), .fill_nap_i(fnap), .fill_ppn_i(fppn), .fill_perm_i(fperm),
    .fill_pbmt_i(fpbmt), .fill_fault_i(ffault), .fill_gperm_i(fgperm), .fill_gpa_i(fgpa), .fill_gimp_i(fgimp));
  always_comb begin
    lk_hit = {dh, ih};
    lk_ppn[0] = ip[0]; lk_ppn[1] = dp[0]; lk_ppn[2] = dp[1];
    lk_perm[0] = ipm[0]; lk_perm[1] = dpm[0]; lk_perm[2] = dpm[1];
    lk_fault[0] = ifl[0]; lk_fault[1] = dfl[0]; lk_fault[2] = dfl[1];
    lk_pbmt[0] = ipb[0]; lk_pbmt[1] = dpb[0]; lk_pbmt[2] = dpb[1];
    lk_gperm[0] = igp[0]; lk_gperm[1] = dgp[0]; lk_gperm[2] = dgp[1];
    lk_gpa[0] = iga[0]; lk_gpa[1] = dga[0]; lk_gpa[2] = dga[1];
    lk_gimp[0] = igi[0]; lk_gimp[1] = dgi[0]; lk_gimp[2] = dgi[1];
  end

  //--------------------------------------------------------------------------
  // 页表 (稀疏) 与参考遍历
  //--------------------------------------------------------------------------
  logic [63:0] mem [logic [55:0]];
  logic [55:0] root;
  int          npage;
  int          top;                      // 起始级 = 2 + vam
  logic [VPN_W-1:0] pool[$];             // 查找用的 VPN 池

  function automatic logic [55:0] new_page();
    npage++;
    return PTB + 56'(npage) * 56'h1000;
  endfunction
  function automatic logic [63:0] pte(logic [43:0] ppn, logic [7:0] flags);
    return {10'b0, ppn, 2'b0, flags};
  endfunction
  // 规范化：按当前模式把 VPN 高位做符号扩展 (VA 位 38 / 47 / 56 = VPN 位 26 / 35 / 44)
  function automatic logic [VPN_W-1:0] canon(logic [VPN_W-1:0] v);
    int b;
    if (two && !vson) return v;                 // vsatp = Bare：VPN 是 GPA 页号
    b = 9 * (top + 1) - 1;
    for (int i = b + 1; i < VPN_W; i++) v[i] = v[b];
    return v;
  endfunction
  function automatic logic [7:0] leaf_flags();   // V + 合法的 R/W/X 组合 + 随机 U/G/A/D
    logic [2:0] rwx;
    case ($urandom_range(4)) 0: rwx = 3'b001; 1: rwx = 3'b011; 2: rwx = 3'b100; 3: rwx = 3'b101; default: rwx = 3'b111; endcase
    return {1'($urandom), 1'($urandom), 1'($urandom), 1'($urandom), rwx[2], rwx[1], rwx[0], 1'b1};
  endfunction
  // 填一张表：lvl 级，VPN 前缀 pre (高位)，返回表页地址
  //   kind 抽签：0..2 叶子 (对齐)、3 叶子未对齐 (超页)、4 指下级、5 无效、6 只写保留、7 保留位非零、8 非叶带 D/A/U、
  //              9 指到 DRAM 外、10 指到 PMP 禁区页、11 叶子 PPN 超出 PA_W、
  //              12 NAPOT 叶子 (PPN[3:0] = 1000：0 级合法，超页级页错误)、13 N 但 PPN[3:0] 不是 1000、
  //              14 叶子带随机 PBMT、15 非叶带 N / PBMT
  function automatic void fill_table(logic [55:0] tab, int lvl, logic [VPN_W-1:0] pre, int depth);
    bit used [512];
    for (int t = 0; t < 6; t++) begin
      int idx, kind;
      logic [VPN_W-1:0] vp;
      logic [43:0] ppn;
      logic [55:0] nt;
      logic [3:0]  lo;
      kind = $urandom_range(lvl > 0 ? 17 : 15);
      // 合法 NAPOT (0 级 kind 12)：16 个相同 PTE 占一整组 (规范要求软件这样写；TLB 按 64K 泛化)
      idx = $urandom_range(511);
      if (lvl == 0 && kind == 12) begin
        idx = idx & ~15;
        for (int k = 0; k < 16; k++) if (used[idx + k]) kind = 0;
      end
      for (int g = 0; g < 32 && used[idx]; g++) idx = $urandom_range(511);
      if (used[idx]) continue;
      if (kind == 12 && lvl == 0) for (int k = 0; k < 16; k++) used[idx + k] = 1;
      else used[idx] = 1;
      vp = pre;
      vp[9*lvl +: 9] = 9'(idx);
      if (kind >= 16) kind = 4;                                 // 超页级多建子树 (4 / 5 级表才走得深)
      if (lvl == 0 && kind >= 8 && kind <= 11) kind = $urandom_range(3);
      if (lvl == 0 && (kind == 4 || kind == 15)) kind = 0;
      ppn = {12'b0, $urandom} & 44'h0_0fff_ffff;                // 28 位以内
      if (lvl > 0) ppn = ppn & ~((44'd1 << (9 * lvl)) - 44'd1); // 超页对齐 (3 / 4 级在 28 位内只剩 0 / 最高位)
      case (kind)
        0, 1, 2: mem[tab + 56'(idx) * 8] = pte(ppn, leaf_flags());
        3:       mem[tab + 56'(idx) * 8] = pte(lvl > 0 ? ppn | 44'd1 : ppn, leaf_flags());
        4: begin
             if (depth < 40) begin
               nt = new_page();
               mem[tab + 56'(idx) * 8] = pte(44'(nt >> 12), 8'h01);
               fill_table(nt, lvl - 1, vp, depth + 1);
             end else mem[tab + 56'(idx) * 8] = 64'd0;
           end
        5:       mem[tab + 56'(idx) * 8] = 64'd0 | 64'(leaf_flags() & 8'hFE);   // V = 0
        6:       mem[tab + 56'(idx) * 8] = pte(ppn, 8'h05);                       // W 无 R
        7:       mem[tab + 56'(idx) * 8] = pte(ppn, leaf_flags()) | (64'(1) << (54 + $urandom_range(6)));
        8:       mem[tab + 56'(idx) * 8] = pte(44'(new_page() >> 12), 8'h01 | (8'h10 << $urandom_range(2) * 1) | 8'h00);
        9:       mem[tab + 56'(idx) * 8] = pte(44'h00001, 8'h01);                 // 下一级表在 0x1000：DRAM 外
        10: begin
              mem[tab + 56'(idx) * 8] = pte(44'(FORB >> 12), 8'h01);
              mem[FORB + 56'($urandom_range(511)) * 8] = pte(ppn, leaf_flags());
            end
        11:      mem[tab + 56'(idx) * 8] = pte(ppn | (44'd1 << (28 + $urandom_range(15))), leaf_flags());
        12: begin
              logic [63:0] e;
              e = pte({ppn[43:4], 4'b1000}, leaf_flags()) | (64'd1 << 63);
              if (lvl == 0) for (int k = 0; k < 16; k++) mem[tab + 56'(idx + k) * 8] = e;
              else mem[tab + 56'(idx) * 8] = e;                     // 超页级的 N：页错误
            end
        13: begin
              lo = 4'($urandom_range(15));
              if (lo == 4'b1000) lo = 4'b0100;
              mem[tab + 56'(idx) * 8] = pte({ppn[43:4], lo}, leaf_flags()) | (64'd1 << 63);
            end
        14:      mem[tab + 56'(idx) * 8] = pte(ppn, leaf_flags()) | (64'($urandom_range(3)) << 61);
        15: begin
              nt = new_page();
              mem[tab + 56'(idx) * 8] = pte(44'(nt >> 12), 8'h01) |
                                        ($urandom_range(1) ? (64'd1 << 63) : (64'($urandom_range(1, 3)) << 61));
              fill_table(nt, lvl - 1, vp, 99);
            end
        default: ;
      endcase
      // 0 级非叶 (V = 1，R = X = 0)
      if (lvl == 0 && kind != 12 && $urandom_range(9) == 0) mem[tab + 56'(idx) * 8] = pte(ppn, 8'h01);
      // VPN 池：本项的几个 VPN (超页里取不同的低位；NAPOT 换低 4 位)
      for (int r = 0; r < 3; r++) begin
        logic [VPN_W-1:0] q;
        q = vp;
        for (int l = 0; l < lvl; l++) q[9*l +: 9] = 9'($urandom_range(511));
        if (lvl == 0 && kind == 12) q[3:0] = 4'($urandom);
        pool.push_back(canon(q));
      end
    end
  endfunction
  typedef struct { int flt; int lvl; bit nap; logic [43:0] ppn; logic [7:0] perm; logic [1:0] pbmt; } ref_t;
  function automatic bit pmp_ok(logic [55:0] a);
    return !(a[55:12] == FORB[55:12]);
  endfunction
  function automatic void build();
    mem.delete(); pool.delete(); npage = 0;
    vam = 2'($urandom_range(2)); top = 2 + int'(vam); pbmte = 1'($urandom);
    two = $urandom_range(1); vson = $urandom_range(3) != 0;
    gmode = 4'(8 + $urandom_range(2)); gtop = 2 + int'(gmode) - 8;
    root = new_page();
    if (!two || vson) begin
      fill_table(root, top, '0, 0);
      // 没定义的 VPN (查到全 0 PTE -> 页错误)
      for (int i = 0; i < 6; i++) pool.push_back(canon(VPN_W'({$urandom, $urandom})));
    end
    if (two) begin
      groot = new_groot();
      if (vson) begin
        foreach (pool[i]) s1_touch(pool[i]);
      end else begin
        // vsatp = Bare：VPN = GPA 页号 (DRAM 附近 / 28 位内随机 / 超宽)
        for (int i = 0; i < 40; i++) begin
          logic [VPN_W-1:0] q;
          unique case ($urandom_range(3))
            0, 1: q = VPN_W'(20'h80000 + $urandom_range(20'hFFFF));
            2:    q = VPN_W'($urandom) & VPN_W'(28'hFFF_FFFF);
            default: q = VPN_W'({$urandom, $urandom});
          endcase
          pool.push_back(q);
          g_map({q, 12'b0}, $urandom_range(1));
        end
      end
    end
  endfunction

  //--------------------------------------------------------------------------
  // 两级：G-stage 表的懒建与参考遍历
  //--------------------------------------------------------------------------
  function automatic logic [10:0] gidx(logic [58:0] g, int l, bit topl);
    logic [10:0] i;
    i = 11'(g >> (12 + 9 * l));
    return topl ? i : {2'b0, i[8:0]};
  endfunction
  function automatic bit gpa_ok(logic [58:0] g);
    if (gmode == 4'd8) return g[58:41] == 0;
    if (gmode == 4'd9) return g[58:50] == 0;
    return 1;
  endfunction
  function automatic logic [55:0] new_groot();     // 16 KB 对齐的 4 页
    logic [55:0] a;
    do a = new_page(); while (a[13:12] != 2'b00);
    void'(new_page()); void'(new_page()); void'(new_page());
    return a;
  endfunction
  function automatic logic [63:0] g_leaf(logic [43:0] ppn);   // G 叶子：属性随机
    logic [7:0] f;
    logic [63:0] e;
    int c;
    c = $urandom_range(19);
    f = 8'hDF;                                     // V R W X U A D
    case (c)
      0: f = 8'hCF;  1: f = 8'h9F;  2: f = 8'h5F;  3: f = 8'hDD;
      4: f = 8'hD9;  5: f = 8'hDB;  6: f = 8'hD7;  7: f = 8'hDE;
      default: ;
    endcase
    e = pte(ppn, f);
    if (c == 8)  e = e | (64'd1 << (54 + $urandom_range(6)));
    if (c == 9)  e = e | (64'($urandom_range(1, 3)) << 61);
    if (c == 10) e = e | (64'd1 << 63);            // N：PPN[3:0] 保证不是 1000 (下面) -> guest page fault
    if (c == 10 && e[13:10] == 4'b1000) e[13] = 1'b0;
    return e;
  endfunction
  // 映射一个 GPA 页 (已有就不动)：逐级向下，空项随机决定
  function automatic void g_map(logic [58:0] gpa, bit ident);
    logic [55:0] t;
    int l;
    if (!gpa_ok(gpa)) return;
    t = groot; l = gtop;
    forever begin
      logic [55:0] ea;
      logic [63:0] p;
      ea = t + 56'(gidx(gpa, l, l == gtop)) * 8;
      if (mem.exists(ea)) begin
        p = mem[ea];
        if (p[3:0] == 4'b0001 && p[63:54] == 0 && p[7:4] == 0 && l > 0) begin t = {p[53:10], 12'b0}; l--; continue; end
        return;
      end
      if (l > 0 && $urandom_range(5) == 0) begin   // 超页叶子
        logic [43:0] pp;
        pp = ident ? 44'(gpa >> 12) : ({12'b0, $urandom} & 44'h0_0fff_ffff);
        pp = pp & ~((44'd1 << (9 * l)) - 44'd1);
        if ($urandom_range(9) == 0) pp = pp | 44'd1;   // 未对齐
        mem[ea] = g_leaf(pp);
        return;
      end
      if (l > 0) begin
        logic [55:0] nt;
        nt = new_page();
        mem[ea] = pte(44'(nt >> 12), 8'h01) | (($urandom_range(40) == 0) ? 64'h80 : 64'h0);   // 偶尔非叶带 D
        t = nt; l--;
        continue;
      end
      mem[ea] = g_leaf(ident ? 44'(gpa >> 12) : ({12'b0, $urandom} & 44'h0_0fff_ffff));
      return;
    end
  endfunction
  // 给一个 VPN 的 S1 遍历用到的 GPA 建 G 映射 (PTE 地址恒等为主，叶子目标随机 PA)
  function automatic void s1_touch(logic [VPN_W-1:0] vpn);
    logic [55:0] a;
    int lvl;
    lvl = top;
    a = root + 56'(vpn[9*lvl +: 9]) * 8;
    forever begin
      logic [63:0] p;
      g_map({3'b0, a}, $urandom_range(15) != 0);
      p = mem.exists(a) ? mem[a] : 64'd0;
      if (!p[0]) return;
      if (p[1] || p[3]) begin
        logic [43:0] m;
        m = (44'd1 << (9 * lvl)) - 44'd1;
        g_map({3'b0, (p[53:10] & ~m) | (44'(vpn) & m), 12'b0}, $urandom_range(3) == 0);
        return;
      end
      if (lvl == 0) return;
      lvl--;
      a = {p[53:10], 12'b0} + 56'(vpn[9*lvl +: 9]) * 8;
    end
  endfunction

  // pa 为合并后的 PA (可能超出 PA_W)；bxf = G-PTE 自己的 PPN 位超出 PA_W (整页超)
  typedef struct { int f; logic [58:0] pa; int lvl; bit nap; logic [3:0] gp; bit bxf; } gres_t;
  function automatic gres_t gwalk(logic [58:0] gpa, bit imp);
    gres_t g;
    logic [55:0] a;
    int l;
    g = '{0, 0, gtop, 0, 0, 0};
    if (!gpa_ok(gpa)) begin g.f = 3; g.lvl = (gmode == 4'd8) ? 3 : 4; return g; end   // 超宽：项不能覆盖到超出的位
    l = gtop;
    a = groot + 56'(gidx(gpa, l, 1)) * 8;
    forever begin
      logic [63:0] p;
      bit v, rr, w, x, u, ad, dd, leaf, bad, n;
      logic [1:0] pb;
      logic [46:0] m, hp;
      if (!(a >= DRAM && a < DRAM + 56'h8000_0000) || !pmp_ok(a)) begin g.f = 2; g.lvl = l; return g; end
      p = mem.exists(a) ? mem[a] : 64'd0;
      v = p[0]; rr = p[1]; w = p[2]; x = p[3]; u = p[4]; ad = p[6]; dd = p[7]; n = p[63]; pb = p[62:61];
      leaf = rr || x;
      bad = !v || (!rr && w) || p[60:54] != 0 || (!pbmte && pb != 0) || pb == 3 || (!leaf && (dd || ad || u || n || pb != 0));
      if (bad || (!leaf && l == 0)) begin g.f = 3; g.lvl = l; return g; end
      if (leaf) begin
        m = n ? 47'hF : (47'd1 << (9 * l)) - 47'd1;
        if ((n && (l != 0 || p[13:10] != 4'b1000)) || !u || (p[53:10] & ((44'd1 << (9 * l)) - 44'd1)) != 0 || !ad || (imp && !rr))
          begin g.f = 3; g.lvl = l; return g; end
        hp = ({3'b0, p[53:10]} & ~m) | (gpa[58:12] & m);
        g.bxf = ((({3'b0, p[53:10]} & ~m) >> (PA_W - 12)) != 0);
        // 隐式 (PTE 地址) 的 PA 超宽直接是访问错误；最终 GPA 的由调用方按合并页级判
        if (imp && hp[46:PA_W-12] != 0) begin g.f = 2; g.lvl = l; return g; end
        g.pa = {hp, gpa[11:0]}; g.lvl = l; g.nap = n; g.gp = {dd, x, w, rr};
        return g;
      end
      l--;
      a = {p[53:10], 12'b0} + 56'(gidx(gpa, l, 0)) * 8;
    end
  endfunction
  function automatic int prank(int l, bit n);
    return (l == 0) ? int'(n) : l + 1;
  endfunction

  typedef struct { int flt; int lvl; bit nap; logic [43:0] ppn; logic [7:0] perm; logic [1:0] pbmt;
                   logic [3:0] gp; logic [58:0] gpa; bit gimp; } ref2_t;
  function automatic ref2_t walk2(logic [VPN_W-1:0] vpn);
    ref2_t r;
    gres_t g;
    logic [55:0] sa, a;
    int lvl;
    r = '{0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!vson) begin                                // vsatp = Bare：GPA = VA，只做 G-stage
      logic [58:0] fg;
      fg = {vpn, 12'b0};
      g = gwalk(fg, 0);
      r.lvl = g.lvl; r.gpa = fg;
      if (g.f == 0 && g.pa[58:PA_W] != 0 && (g.bxf || g.lvl != 4)) g.f = 2;   // PA 超宽 (4 级页由查找时按 VA 判)
      if (g.f != 0) begin r.flt = g.f; if (g.f == 2) r.gpa = 0; return r; end
      r.nap = g.nap; r.ppn = 44'(g.pa >> 12); r.perm = 8'hDF; r.gp = g.gp;
      return r;
    end
    lvl = top;
    sa = root + 56'(vpn[9*lvl +: 9]) * 8;
    forever begin
      logic [63:0] p;
      bit v, rr, w, x, leaf, bad, n;
      logic [1:0] pb;
      g = gwalk({3'b0, sa}, 1);                     // VS PTE 地址过 G-stage (隐式)
      if (g.f == 3) begin r = '{3, lvl, 0, 0, 0, 0, 0, {3'b0, sa}, 1}; return r; end
      if (g.f == 2) begin r = '{2, lvl, 0, 0, 0, 0, 0, 0, 0}; return r; end
      a = g.pa[55:0];
      if (!(a >= DRAM && a < DRAM + 56'h8000_0000) || !pmp_ok(a)) begin r = '{2, lvl, 0, 0, 0, 0, 0, 0, 0}; return r; end
      p = mem.exists(a) ? mem[a] : 64'd0;
      v = p[0]; rr = p[1]; w = p[2]; x = p[3]; leaf = rr || x; n = p[63]; pb = p[62:61];
      bad = !v || (!rr && w) || p[60:54] != 0 || (!pbmte && pb != 0) || pb == 3 ||
            (!leaf && (p[7] || p[6] || p[4] || n || pb != 0));
      if (bad || (!leaf && lvl == 0)) begin r = '{1, lvl, 0, 0, 0, 0, 0, 0, 0}; return r; end
      if (leaf) begin
        logic [46:0] m;
        logic [58:0] fg;
        if ((p[53:10] & ((44'd1 << (9 * lvl)) - 44'd1)) != 0 || (n && (lvl != 0 || p[13:10] != 4'b1000)))
          begin r = '{1, lvl, 0, 0, 0, 0, 0, 0, 0}; return r; end
        m  = n ? 47'hF : (47'd1 << (9 * lvl)) - 47'd1;
        fg = {({3'b0, p[53:10]} & ~m) | (vpn & m), 12'b0};
        g  = gwalk(fg, 0);
        // 合并页级 = 两级较小者 (故障项同理)
        if (prank(g.lvl, (g.f == 0) ? g.nap : 1'b0) < prank(lvl, (g.f == 0) ? n : 1'b0)) begin r.lvl = g.lvl; r.nap = (g.f == 0) ? g.nap : 0; end
        else begin r.lvl = lvl; r.nap = (g.f == 0) ? n : 0; end
        r.gpa = fg;
        if (g.f == 0 && g.pa[58:PA_W] != 0 && (g.bxf || r.lvl != 4)) g.f = 2;   // PA 超宽 (合并 4 级由查找时按 VA 判)
        if (g.f != 0) begin r.flt = g.f; if (g.f == 2) r.gpa = 0; return r; end
        r.ppn = 44'(g.pa >> 12); r.perm = p[7:0]; r.pbmt = pb; r.gp = g.gp;
        return r;
      end
      lvl--;
      sa = {p[53:10], 12'b0} + 56'(vpn[9*lvl +: 9]) * 8;
    end
  endfunction
  // 一级或两级的参考 (统一成 ref2_t)
  function automatic ref2_t refw(logic [VPN_W-1:0] vpn);
    ref2_t q;
    ref_t  r;
    if (two) return walk2(vpn);
    r = walk(vpn);
    q = '{r.flt, r.lvl, r.nap, r.ppn, r.perm, r.pbmt, 4'hF, 0, 0};
    return q;
  endfunction

  function automatic ref_t walk(logic [VPN_W-1:0] vpn);
    ref_t r;
    logic [55:0] a;
    int lvl;
    lvl = top;
    a = root + 56'(vpn[9*lvl +: 9]) * 8;
    forever begin
      logic [63:0] p;
      bit v, rr, w, x, leaf, bad, n;
      logic [1:0] pb;
      if (!(a >= DRAM && a < DRAM + 56'h8000_0000) || !pmp_ok(a)) begin r = '{2, lvl, 0, 0, 0, 0}; return r; end
      p = mem.exists(a) ? mem[a] : 64'd0;
      v = p[0]; rr = p[1]; w = p[2]; x = p[3]; leaf = rr || x; n = p[63]; pb = p[62:61];
      bad = !v || (!rr && w) || p[60:54] != 0 || (!pbmte && pb != 0) || pb == 3 ||
            (!leaf && (p[7] || p[6] || p[4] || n || pb != 0));
      if (bad || (!leaf && lvl == 0)) begin r = '{1, lvl, 0, 0, 0, 0}; return r; end
      if (leaf) begin
        if ((p[53:10] & ((44'd1 << (9 * lvl)) - 44'd1)) != 0 || (n && (lvl != 0 || p[13:10] != 4'b1000)))
          begin r = '{1, lvl, n, p[53:10], 0, pb}; return r; end
        if (p[53:10+PA_W-12] != 0) begin r = '{2, lvl, n, p[53:10], 0, pb}; return r; end
        r = '{0, lvl, n, p[53:10], p[7:0], pb}; return r;
      end
      lvl--;
      a = {p[53:10], 12'b0} + 56'(vpn[9*lvl +: 9]) * 8;
    end
  endfunction
  // 查找结果：页内 VPN 位来自 VA；4 级页 VA[47:40] (VPN[35:28]) 非零 -> 访问错误
  function automatic logic [PA_W-13:0] comb_ppn(ref_t r, logic [VPN_W-1:0] vpn);
    logic [43:0] m;
    m = r.nap ? 44'hF : (44'd1 << (9 * r.lvl)) - 44'd1;
    return (PA_W-12)'((r.ppn & ~m) | (44'(vpn) & m));
  endfunction
  function automatic int lk_flt(ref_t r, logic [VPN_W-1:0] vpn);
    if (r.flt == 0 && r.lvl == 4 && vpn[35:PA_W-12] != 0) return 2;
    return r.flt;
  endfunction
  function automatic logic [PA_W-13:0] comb_ppn2(ref2_t r, logic [VPN_W-1:0] vpn);
    logic [43:0] m;
    m = r.nap ? 44'hF : (44'd1 << (9 * r.lvl)) - 44'd1;
    return (PA_W-12)'((r.ppn & ~m) | (44'(vpn) & m));
  endfunction
  function automatic int lk_flt2(ref2_t r, logic [VPN_W-1:0] vpn);
    if (r.flt == 0 && r.lvl == 4 && vpn[35:PA_W-12] != 0) return 2;
    return r.flt;
  endfunction
  // 查找口的 GPA：隐式 -> PTE GPA[58:2]；否则 = 该 VPN 的 GPA 页号 (低 10 位 0)
  function automatic logic [56:0] lk_gpa_exp(ref2_t r);
    return r.gimp ? r.gpa[58:2] : {r.gpa[58:12], 10'b0};
  endfunction

  int errs, ncyc, nfill, nhit;
  function automatic void err(string s);
    errs++;
    if (errs <= 25) $display("[%0t] tb_mmu 错误：%s", $time, s);
  endfunction

  // 覆盖率
  int c_flt, c_lvl, c_pb, c_vam, c_two, c_gpf;  bit c_fill, c_abort, c_both, c_super, c_repl, c_late, c_nap, c_l4af;
  covergroup cg_mmu @(posedge clk);
    cp_res: coverpoint c_flt iff (c_fill) { bins ok = {0}; bins pf = {1}; bins af = {2}; bins gpf = {3}; }
    cp_two: coverpoint c_two iff (c_fill) { bins one = {0}; bins gonly = {1}; bins vsg = {2}; }
    cx_two_res: cross cp_two, cp_res {
      ignore_bins na = binsof(cp_two.gonly) && binsof(cp_res.pf) ||    // vsatp = Bare：没有 VS 页错误
                       binsof(cp_two.one)   && binsof(cp_res.gpf);     // 一级：没有 guest page fault
    }
    cp_gpf: coverpoint c_gpf { bins fin = {1}; bins imp = {2}; }   // 查找命中的 guest page fault 项 (最终 / 隐式)
    cp_lvl: coverpoint c_lvl iff (c_fill) { bins l[] = {[0:4]}; }
    cp_vam: coverpoint c_vam iff (c_fill) { bins m[] = {[0:2]}; }
    cx_res_lvl: cross cp_res, cp_lvl;                           // 每种结局 × 每级
    cp_pb:    coverpoint c_pb iff (c_fill) { bins p[] = {[0:2]}; }   // 正常叶子的 PBMT
    cp_abort: coverpoint c_abort { bins y = {1}; }             // 遍历中被冲刷
    cp_late:  coverpoint c_late  { bins y = {1}; }             // 冲刷后读数据才回来 (必须丢掉)
    cp_both:  coverpoint c_both  { bins y = {1}; }             // I / D 同拍请求 (D 优先)
    cp_super: coverpoint c_super { bins y = {1}; }             // 超页项被别的 VPN 命中
    cp_nap:   coverpoint c_nap   { bins y = {1}; }             // NAPOT 项被别的 VPN 命中
    cp_l4af:  coverpoint c_l4af  { bins y = {1}; }             // 4 级页查找报访问错误
    cp_repl:  coverpoint c_repl  { bins y = {1}; }             // TLB 满后轮转替换
  endgroup
  cg_mmu cg = new();

  // 存储口 (同 LSU 的 ptw_own)
  bit          out_v = 0;  int out_t;  logic [55:0] out_a;
  logic [VPN_W-1:0] last_fill_vpn [3];

  initial begin
    if (!$value$plusargs("cycles=%d", ncyc)) ncyc = 100000;
    errs = 0; nfill = 0; nhit = 0;
    build();
    satp_ppn = 44'(root >> 12);
    for (int i = 0; i < 8; i++) begin pmpcfg[i] = 8'h00; pmpaddr[i] = '0; end
    pmpcfg[0] = 8'h18; pmpaddr[0] = 54'(FORB >> 2) | 54'h1FF;    // NAPOT 4K：禁区页，无权限
    pmpcfg[1] = 8'h1F; pmpaddr[1] = '1;                           // NAPOT 全地址 RWX
    for (int p = 0; p < 3; p++) begin lk_vpn[p] = pool[0]; last_fill_vpn[p] = pool[0]; end   // 未初始化是 X：查到 X 的 VPN 会把 X 送进 PTW
    repeat (3) @(posedge clk);
    rst_n = 1;
    for (int c = 0; c < ncyc; c++) begin
      @(negedge clk);
      step(c);
    end
    if (errs == 0) $display("=== UNIT PASS === mmu %0d 拍，填写 %0d 次、命中核对 %0d 次", ncyc, nfill, nhit);
    else           $display("=== UNIT FAIL === mmu 错误 %0d", errs);
    $finish;
  end

  task automatic step(int c);
    c_fill = 0; c_abort = 0; c_both = 0; c_super = 0; c_repl = 0; c_late = 0; c_nap = 0; c_l4af = 0; c_gpf = 0;
    // ---- 上拍决定的冲刷已生效：两边 TLB 应全空 ----
    if (flush) begin
      flush = 0;
      if (u_itlb.v != '0 || u_dtlb.v != '0) err("冲刷后 TLB 仍有有效项");
    end
    // ---- 本拍查找 ----
    for (int p = 0; p < 3; p++) lk_vpn[p] = pool[$urandom_range(pool.size() - 1)];
    // 偶尔查刚填过那个 VPN 所在大页里的别的 VPN
    for (int p = 0; p < 3; p++) if ($urandom_range(3) == 0) begin
      lk_vpn[p] = last_fill_vpn[p];
      if ($urandom_range(3) == 0) lk_vpn[p][3:0] = 4'($urandom);
      else begin
        lk_vpn[p][8:0] = 9'($urandom);
        if ($urandom_range(1)) lk_vpn[p][17:9] = 9'($urandom);
        if ($urandom_range(2) == 0) lk_vpn[p][35:18] = 18'($urandom);
      end
      lk_vpn[p] = canon(lk_vpn[p]);
    end
    #1;
    for (int p = 0; p < 3; p++) if (lk_hit[p]) begin
      ref2_t r;
      int    f;
      r = refw(lk_vpn[p]);
      f = lk_flt2(r, lk_vpn[p]);
      nhit++;
      if (lk_fault[p] != 2'(f)) err($sformatf("口 %0d VPN %h 命中 (两级 %0d)，故障 %0d != 参考 %0d (级 %0d)", p, lk_vpn[p], two, lk_fault[p], f, r.lvl));
      else if (f == 0 && (lk_ppn[p] !== comb_ppn2(r, lk_vpn[p]) || lk_perm[p] !== r.perm || lk_pbmt[p] !== r.pbmt ||
                          (two && (lk_gperm[p] !== r.gp || lk_gpa[p] !== lk_gpa_exp(r)))))
        err($sformatf("口 %0d VPN %h 命中，ppn %h / %h，权限 %h / %h，pbmt %0d / %0d，gperm %h / %h，gpa %h / %h", p, lk_vpn[p],
                      lk_ppn[p], comb_ppn2(r, lk_vpn[p]), lk_perm[p], r.perm, lk_pbmt[p], r.pbmt, lk_gperm[p], r.gp, lk_gpa[p], lk_gpa_exp(r)));
      else if (f == 3 && (lk_gimp[p] !== r.gimp || lk_gpa[p] !== lk_gpa_exp(r)))
        err($sformatf("口 %0d VPN %h guest page fault：隐式 %0d / %0d，gpa %h / %h", p, lk_vpn[p], lk_gimp[p], r.gimp, lk_gpa[p], lk_gpa_exp(r)));
      if (f == 3) c_gpf = r.gimp ? 2 : 1;
      if (lk_vpn[p] != last_fill_vpn[p] && r.lvl > 0) c_super = 1;
      if (lk_vpn[p] != last_fill_vpn[p] && r.nap && r.flt == 0) c_nap = 1;
      if (f == 2 && r.flt == 0) c_l4af = 1;
    end
    // ---- PTW 填写 (本拍组合输出，拍末写进 TLB) ----
    if (ifill || dfill) begin
      ref2_t r;
      r = refw(fvpn);
      nfill++;
      c_fill = 1; c_flt = r.flt; c_lvl = r.lvl; c_vam = int'(vam); c_pb = (r.flt == 0) ? int'(r.pbmt) : 3;
      c_two = two ? (vson ? 2 : 1) : 0;
      if (ffault != 2'(r.flt) || flvl != 3'(r.lvl) ||
          (r.flt == 0 && (fppn !== r.ppn[PA_W-13:0] || fperm !== r.perm || fnap !== r.nap || fpbmt !== r.pbmt ||
                          (two && (fgperm !== r.gp || fgpa !== r.gpa[58:2])))) ||
          (r.flt == 3 && (fgimp !== r.gimp || fgpa !== r.gpa[58:2])))
        err($sformatf("填写 VPN %h (vam %0d 两级 %0d/%0d g%0d)：故障 %0d 级 %0d nap %0d ppn %h 权限 %h pbmt %0d gp %h gpa %h imp %0d；参考 %0d %0d %0d %h %h %0d %h %h %0d",
                      fvpn, vam, two, vson, gmode, ffault, flvl, fnap, fppn, fperm, fpbmt, fgperm, fgpa, fgimp,
                      r.flt, r.lvl, r.nap, r.ppn[PA_W-13:0], r.perm, r.pbmt, r.gp, r.gpa[58:2], r.gimp));
      if (ifill) begin last_fill_vpn[0] = fvpn; if (&u_itlb.v) c_repl = 1; end
      if (dfill) begin last_fill_vpn[1] = fvpn; last_fill_vpn[2] = fvpn; if (&u_dtlb.v) c_repl = 1; end
    end
    // ---- 缺失 -> 请求 PTW (组合：本拍缺失的 VPN；D 优先) ----
    ireq = !lk_hit[0];
    ivpn = lk_vpn[0];
    dreq = !lk_hit[1] || !lk_hit[2];
    dvpn = !lk_hit[1] ? lk_vpn[1] : lk_vpn[2];
    if (ireq && dreq && !busy) c_both = 1;
    // ---- 存储口：一次一个读；回来之前不给新的 gnt ----
    mvalid = 0;
    if (out_v && c >= out_t) begin
      mvalid = 1; mdata = mem.exists(out_a) ? mem[out_a] : 64'd0; out_v = 0;
      if (u_ptw.st != 2'd2) c_late = 1;          // 数据回来时 PTW 不在等它 (已放弃)
    end
    mgnt = 0;
    if (!out_v && !mvalid && mreq && $urandom_range(99) < 70) begin
      mgnt = 1; out_v = 1; out_t = c + 1 + $urandom_range(0, 8); out_a = {{(56-PA_W){1'b0}}, maddr};
    end
    // ---- 冲刷 (下拍生效检查)；部分冲刷同时改写页表 (并重抽模式 / PBMTE：只在冲刷时变，同 satp / menvcfg 写清 TLB) ----
    // 遍历进行中 (尤其在等数据时) 冲刷更频繁：查「放弃后回来的数据必须丢掉」
    if ($urandom_range(999) < (u_ptw.st == 2'd2 ? 40 : 8)) begin
      flush = 1;
      if (busy) c_abort = 1;
      if ($urandom_range(2) == 0) begin build(); satp_ppn = 44'(root >> 12); end
    end
  endtask
endmodule
