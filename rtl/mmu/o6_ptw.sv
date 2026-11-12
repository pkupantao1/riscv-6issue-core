//==============================================================================
// o6_ptw.sv - 页表遍历器 (I/D 共享，D 优先)：Sv39 / Sv48 / Sv57 (起始级 2 / 3 / 4 按 vam)，
//             H 扩展的两级翻译 (VS-stage = vsatp，G-stage = hgatp 的 Sv39x4 / Sv48x4 / Sv57x4)
//
// 读 PTE 走 D$ (经 LSU 的 D$ 读口仲裁)，所以能看到软件刚写进 D$ 的页表。
// 结果总是填回请求方的 TLB：
//   正常叶子 -> 普通项；
//   无效 PTE / 保留编码 / 超页未对齐 / 0 级仍非叶子 -> 页错误项；
//   保留位 [60:54]、PBMT (PBMTE = 0 时非零 / 取 3)、N (Svnapot：只许 0 级叶子且 PPN[3:0] = 1000)、
//   非叶子的 D / A / U / N / PBMT -> 页错误项 (检查次序与 Spike mmu_t::walk 一致，结果都是页错误)；
//   PTE 地址不在存储区或 PMP 不允许 (隐式访问按 S 态读检查)，或叶子 PPN 超出实现的物理地址 -> 访问错误项。
// 两级 (请求 V = 1 且 hgatp.MODE != Bare，docs/15 §6)：
//   VS-stage 每一级的 PTE 地址是 GPA，先做一次 G-stage 遍历得到 PA 再读 PTE；叶子得到最终 GPA 再做一次 G-stage。
//   vsatp = Bare 时 GPA = VA，只做 G-stage。G-stage 根表 16 KB (顶级索引 11 位)，GPA 超出 41 / 50 / 59 位 -> guest page fault。
//   G-stage PTE 的检查 (Spike s2xlate)：同上的保留位 / PBMT (看 menvcfg.PBMTE) / 非叶 D/A/U/N/PBMT，叶子还要 U = 1、A = 1、
//   超页对齐；为 VS PTE 地址做的翻译要 R (MXR 无效)。失败 -> guest page fault 项 (fault = 3)，带 GPA：
//   VS PTE 地址的 (隐式，tinst = 0x3000) 存完整 PTE GPA；最终 GPA 的存 GPA 页号 (TLB 查找时按页级拼 VA 低位)。
//   G-stage PTE 自身读取的 PMP / 存储区失败 -> 访问错误项。
//   合并项：PA 页号、页级 = 两级较小者 (含 NAPOT 64 KB)、VS 权限位、G 级的 {D, X, W, R} (按访问类型在查找时判)、GPA 页号。
// flush (sfence.vma / hfence / satp / vsatp / hgatp 写) 时放弃当前遍历，在途读回来的数据丢弃。
// A/D 位不做硬件更新 (Svade)，权限与 A/D 检查在 TLB 查找时做 (G-stage 的 A = 0 在这里判；D 在查找时按 store 判)。
//==============================================================================
`default_nettype none
module o6_ptw
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  wire  [43:0]         satp_ppn_i,
  input  wire  [1:0]          vam_i,             // 0 = Sv39，1 = Sv48，2 = Sv57
  input  wire                 pbmte_i,           // menvcfg.PBMTE (V = 0 的页表与 G-stage)
  input  wire  [43:0]         vsatp_ppn_i,       // H：V = 1 的请求按 vsatp
  input  wire  [1:0]          vvam_i,
  input  wire                 vson_i,            // vsatp.MODE != Bare
  input  wire                 vpbmte_i,          // henvcfg.PBMTE 读值 (VS-stage)
  input  wire  [3:0]          hgmode_i,          // hgatp.MODE：0 Bare，8 / 9 / 10 = Sv39x4 / Sv48x4 / Sv57x4
  input  wire  [43:0]         hgppn_i,           // hgatp.PPN (低 2 位恒 0)
  input  pmp_dec_t            pmpd_i    [8],      // PMP 预译码 (o6_core 寄存)

  input  wire                 ireq_i,
  input  wire  [VPN_W-1:0]    ireq_vpn_i,
  input  wire                 ireq_v_i,
  input  wire                 dreq_i,
  input  wire  [VPN_W-1:0]    dreq_vpn_i,
  input  wire                 dreq_v_i,
  output logic                busy_o,

  // D$ 读
  output logic                mem_req_o,
  output logic [PA_W-1:0]     mem_addr_o,
  input  wire                 mem_gnt_i,         // 本拍请求被接受
  input  wire                 mem_valid_i,       // 数据返回
  input  wire  [63:0]         mem_data_i,

  // 填 TLB
  output logic                ifill_o,
  output logic                dfill_o,
  output logic                fill_v_o,
  output logic [VPN_W-1:0]    fill_vpn_o,
  output logic [2:0]          fill_level_o,
  output logic                fill_nap_o,
  output logic [PA_W-13:0]    fill_ppn_o,
  output logic [7:0]          fill_perm_o,
  output logic [1:0]          fill_pbmt_o,
  output logic [1:0]          fill_fault_o,      // 0 无，1 页错误，2 访问错误，3 guest page fault
  output logic [3:0]          fill_gperm_o,      // G-stage 叶子的 {D, X, W, R} (非两级时全 1)
  output logic [56:0]         fill_gpa_o,        // GPA[58:2]：最终 GPA 的页号在 [56:10]；隐式 (gimp) 时是完整 PTE GPA
  output logic                fill_gimp_o        // guest page fault 发生在为 VS PTE 地址做的 G-stage 翻译 (tinst = 0x3000)
);
  typedef enum logic [1:0] {P_IDLE, P_REQ, P_WAIT, P_DONE} st_e;
  st_e         st;
  logic        is_d;
  logic [VPN_W-1:0] vpn;
  logic        rv;         // 本次遍历的 V (选 satp / vsatp、PBMTE；填写时带出)
  logic        gon;        // 两级 (V = 1 且 hgatp != Bare)
  logic        ph;         // 当前读的是 0 = S1 (VS / 普通) PTE，1 = G-stage PTE
  logic        gfin;       // G-stage 翻译的是最终 GPA (1) 还是 VS PTE 地址 (0)
  logic [2:0]  lvl;        // S1 级
  logic [2:0]  glvl;       // G-stage 级
  logic [55:0] a;          // 当前要读的 PTE 物理地址 (56 位，越界检查用)
  logic [58:0] ga;         // G-stage 正在翻译的 GPA
  logic [55:0] sa;         // 两级时 VS PTE 的 GPA (隐式 guest page fault 用)
  logic        abort;      // 遍历期间被冲刷：等在途数据回来后丢弃
  // VS-stage 叶子 (两级时先存下，G-stage 完成后合并)
  logic [2:0]  vs_lvl;
  logic        vs_nap;
  logic [7:0]  vs_perm;
  logic [1:0]  vs_pbmt;

  logic [1:0]  r_fault;
  logic [43:0] r_ppn;
  logic [7:0]  r_perm;
  logic [2:0]  r_lvl;
  logic        r_nap;
  logic [1:0]  r_pbmt;
  logic [3:0]  r_gperm;
  logic [56:0] r_gpa;
  logic        r_gimp;
  wire         nv    = dreq_i ? dreq_v_i : ireq_v_i;                   // 新请求的 V
  wire  [VPN_W-1:0] nvpn = dreq_i ? dreq_vpn_i : ireq_vpn_i;
  wire  [1:0]  nvam  = nv ? vvam_i : vam_i;
  wire  [2:0]  lvl0  = (nvam == 2'd2) ? 3'd4 : (nvam == 2'd1) ? 3'd3 : 3'd2;
  wire         pbe   = rv ? vpbmte_i : pbmte_i;
  wire         ngon  = nv && (hgmode_i != 4'd0);
  wire  [2:0]  glvl0 = (hgmode_i == 4'd10) ? 3'd4 : (hgmode_i == 4'd9) ? 3'd3 : 3'd2;

  assign busy_o     = (st != P_IDLE);
  assign mem_req_o  = (st == P_REQ) && !abort;
  assign mem_addr_o = a[PA_W-1:0];

  function automatic logic [8:0] vpn_slice(input logic [VPN_W-1:0] v, input logic [2:0] l);
    unique case (l)
      3'd4:    return v[44:36];
      3'd3:    return v[35:27];
      3'd2:    return v[26:18];
      3'd1:    return v[17:9];
      default: return v[8:0];
    endcase
  endfunction
  // 叶子 PPN 的超页对齐：低 9*lvl 位须为 0
  function automatic logic sp_mis(input logic [43:0] ppn, input logic [2:0] l);
    unique case (l)
      3'd4:    return ppn[35:0] != '0;
      3'd3:    return ppn[26:0] != '0;
      3'd2:    return ppn[17:0] != '0;
      3'd1:    return ppn[8:0]  != '0;
      default: return 1'b0;
    endcase
  endfunction
  // 页内的页号位 (来自地址的位)
  function automatic logic [46:0] pmask(input logic [2:0] l, input logic n);
    unique case (l)
      3'd4:    return 47'h0_000F_FFFF_FFFF;
      3'd3:    return 47'h0_0000_07FF_FFFF;
      3'd2:    return 47'h0_0000_0003_FFFF;
      3'd1:    return 47'h0_0000_0000_01FF;
      default: return n ? 47'hF : 47'h0;
    endcase
  endfunction
  // G-stage：GPA 宽度 (41 / 50 / 59 位) 内的第 l 级索引 (顶级 11 位)
  function automatic logic gpa_ok(input logic [58:0] g, input logic [3:0] m);
    unique case (m)
      4'd8:    return g[58:41] == '0;
      4'd9:    return g[58:50] == '0;
      default: return 1'b1;
    endcase
  endfunction
  function automatic logic [10:0] gidx(input logic [58:0] g, input logic [2:0] l, input logic top);
    logic [10:0] i;
    unique case (l)
      3'd4:    i = g[58:48];
      3'd3:    i = g[49:39];
      3'd2:    i = g[40:30];
      3'd1:    i = {2'b0, g[29:21]};
      default: i = {2'b0, g[20:12]};
    endcase
    return top ? i : {2'b0, i[8:0]};
  endfunction
  // 页大小的序：4K < 64K (NAPOT) < 2M < 1G < 512G < 256T
  function automatic logic [2:0] prank(input logic [2:0] l, input logic n);
    return (l == 3'd0) ? {2'b0, n} : l + 3'd1;
  endfunction

  // 页表访问的 PMP 检查：S 态读
  logic pmp_ok;
  o6_pmp #(.N(8)) u_pmp (
    .dec_i(pmpd_i), .pa_i(a[PA_W-1:0]), .priv_i(2'b01), .acc_i(3'b001), .ok_o(pmp_ok)
  );

  function automatic logic mem_ok(input logic [55:0] pa);
    return (pa >= {16'b0, DRAM_BASE[39:0]}) && (pa < {16'b0, DRAM_BASE[39:0]} + 56'h8000_0000);
  endfunction

  // 结束一次遍历 (填故障项 / 普通项) 的公共写法
  task automatic fin(input logic [1:0] f, input logic [2:0] l, input logic n, input logic [43:0] p, input logic [7:0] pm,
                     input logic [1:0] pb, input logic [3:0] gp, input logic [56:0] g, input logic gi);
    r_fault <= f; r_lvl <= l; r_nap <= n; r_ppn <= p; r_perm <= pm; r_pbmt <= pb;
    r_gperm <= gp; r_gpa <= g; r_gimp <= gi;
    st <= P_DONE;
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st <= P_IDLE; abort <= 1'b0; is_d <= 1'b0; vpn <= '0; lvl <= '0; a <= '0; rv <= 1'b0;
      gon <= 1'b0; ph <= 1'b0; gfin <= 1'b0; glvl <= '0; ga <= '0; sa <= '0;
      vs_lvl <= '0; vs_nap <= 1'b0; vs_perm <= '0; vs_pbmt <= '0;
      r_fault <= '0; r_ppn <= '0; r_perm <= '0; r_lvl <= '0; r_nap <= 1'b0; r_pbmt <= '0;
      r_gperm <= '0; r_gpa <= '0; r_gimp <= 1'b0;
    end else begin
      if (flush_i && st != P_IDLE) abort <= 1'b1;
      unique case (st)
        P_IDLE: begin
          abort <= 1'b0;
          if (!flush_i && (dreq_i || ireq_i)) begin
            logic [55:0] s1a;
            is_d <= dreq_i;
            vpn  <= nvpn;
            rv   <= nv;
            gon  <= ngon;
            lvl  <= lvl0;
            s1a  = {nv ? vsatp_ppn_i : satp_ppn_i, 12'b0} + {44'b0, vpn_slice(nvpn, lvl0), 3'b000};
            if (!ngon) begin
              ph <= 1'b0; a <= s1a;
              st <= P_REQ;
            end else begin
              // 两级：先翻译 VS 根表项的 GPA；vsatp = Bare 时直接翻译 GPA = VA
              logic [58:0] g;
              g = vson_i ? {3'b0, s1a} : {nvpn, 12'b0};
              gfin <= !vson_i; sa <= s1a;
              if (!vson_i) begin vs_lvl <= 3'd4; vs_nap <= 1'b0; vs_perm <= 8'hDF; vs_pbmt <= '0; end
              ph <= 1'b1; glvl <= glvl0; ga <= g;
              a  <= {hgppn_i, 12'b0} + {45'b0, gidx(g, glvl0, 1'b1)} * 56'd8;
              if (!gpa_ok(g, hgmode_i)) begin
                // GPA 超宽：guest page fault (项覆盖的范围不能把超出的位也包进来：Sv39x4 用 3 级)
                fin(2'd3, vson_i ? lvl0 : ((hgmode_i == 4'd8) ? 3'd3 : 3'd4), 1'b0, '0, '0, '0, '0, g[58:2], vson_i);
              end else
                st <= P_REQ;
            end
          end
        end
        P_REQ: begin
          if (abort || flush_i) st <= P_IDLE;
          else if (!mem_ok(a) || !pmp_ok) begin
            // PTE (含 G-stage PTE) 读取访问错误
            fin(2'd2, (ph && gfin) ? ((prank(glvl, 1'b0) < prank(vs_lvl, vs_nap)) ? glvl : vs_lvl) : lvl,
                1'b0, '0, '0, '0, '0, '0, 1'b0);
          end else if (mem_gnt_i) st <= P_WAIT;
        end
        P_WAIT: begin
          if (mem_valid_i) begin
            logic [63:0] pte;
            logic v, r, w, x, u, ad, dd, leaf, bad, n, nbad;
            logic [1:0] pb;
            logic [43:0] pp;
            pte  = mem_data_i;
            v = pte[0]; r = pte[1]; w = pte[2]; x = pte[3]; u = pte[4]; ad = pte[6]; dd = pte[7];
            n = pte[63]; pb = pte[62:61]; pp = pte[53:10];
            leaf = r || x;
            if (abort || flush_i) begin
              st <= P_IDLE;
            end else if (!ph) begin
              //------------------------------------------------------------ S1 (VS / 普通) PTE
              // 非叶子 PTE 的 D / A / U / N / PBMT 是保留位 (特权规范 1.12+)：置了就报页错误 (#32，riscv-arch-test Sv_sv39_nleaf_pte_DAU)
              bad  = !v || (!r && w) || (pte[60:54] != 7'd0) || (!pbe && pb != 2'd0) || (pb == 2'd3) ||
                     (!leaf && (dd || ad || u || n || pb != 2'd0));
              // Svnapot：只支持 64K (PPN[3:0] = 1000)，且只在 0 级
              nbad = n && (lvl != 3'd0 || pte[13:10] != 4'b1000);
              if (bad || (!leaf && lvl == 3'd0)) begin
                fin(2'd1, lvl, 1'b0, '0, '0, '0, '0, '0, 1'b0);
              end else if (leaf) begin
                // 故障项不按 64K 泛化 (nap = 0)：同一 64K 里别的 VPN 读的是别的 PTE (软件只对合法 NAPOT 保证 16 项相同)
                if (nbad || sp_mis(pp, lvl)) begin
                  fin(2'd1, lvl, 1'b0, '0, '0, '0, '0, '0, 1'b0);        // 超页未对齐 / NAPOT 编码不支持
                end else if (!gon) begin
                  if (pp[43:PA_W-12] != '0) fin(2'd2, lvl, 1'b0, '0, '0, '0, '0, '0, 1'b0);   // 物理地址超出实现宽度
                  else                       fin(2'd0, lvl, n, pp, pte[7:0], pb, 4'hF, '0, 1'b0);
                end else begin
                  // 两级：VS 叶子 -> 最终 GPA (按 VS 页级拼 VA 低位) 再过 G-stage
                  logic [46:0] m, gp;
                  logic [58:0] g;
                  m  = pmask(lvl, n);
                  gp = ({3'b0, pp} & ~m) | (vpn & m);
                  g  = {gp, 12'b0};
                  vs_lvl <= lvl; vs_nap <= n; vs_perm <= pte[7:0]; vs_pbmt <= pb;
                  ph <= 1'b1; gfin <= 1'b1; glvl <= glvl0; ga <= g;
                  a  <= {hgppn_i, 12'b0} + {45'b0, gidx(g, glvl0, 1'b1)} * 56'd8;
                  if (!gpa_ok(g, hgmode_i))
                    fin(2'd3, (prank(lvl, n) < prank((hgmode_i == 4'd8) ? 3'd3 : 3'd4, 1'b0)) ? lvl : ((hgmode_i == 4'd8) ? 3'd3 : 3'd4),
                        1'b0, '0, '0, '0, '0, g[58:2], 1'b0);
                  else
                    st <= P_REQ;
                end
              end else begin
                logic [55:0] s1a;
                s1a = {pp, 12'b0} + {44'b0, vpn_slice(vpn, lvl - 1'b1), 3'b000};
                lvl <= lvl - 1'b1;
                if (!gon) begin
                  a  <= s1a;
                  st <= P_REQ;
                end else begin
                  ph <= 1'b1; gfin <= 1'b0; glvl <= glvl0; ga <= {3'b0, s1a}; sa <= s1a;
                  a  <= {hgppn_i, 12'b0} + {45'b0, gidx({3'b0, s1a}, glvl0, 1'b1)} * 56'd8;
                  if (!gpa_ok({3'b0, s1a}, hgmode_i)) fin(2'd3, lvl - 1'b1, 1'b0, '0, '0, '0, '0, {3'b0, s1a[55:2]}, 1'b1);
                  else st <= P_REQ;
                end
              end
            end else begin
              //------------------------------------------------------------ G-stage PTE
              logic [2:0] fl;      // 故障项的页级
              fl   = !gfin ? lvl : (prank(glvl, 1'b0) < prank(vs_lvl, vs_nap)) ? glvl : vs_lvl;
              bad  = !v || (!r && w) || (pte[60:54] != 7'd0) || (!pbmte_i && pb != 2'd0) || (pb == 2'd3) ||
                     (!leaf && (dd || ad || u || n || pb != 2'd0));
              nbad = n && (glvl != 3'd0 || pte[13:10] != 4'b1000);
              if (bad || (!leaf && glvl == 3'd0) ||
                  (leaf && (nbad || !u || sp_mis(pp, glvl) || !ad || (!gfin && !r)))) begin
                fin(2'd3, fl, 1'b0, '0, '0, '0, '0, gfin ? ga[58:2] : {1'b0, sa[55:0]} >> 2, !gfin);
              end else if (!leaf) begin
                glvl <= glvl - 1'b1;
                a    <= {pp, 12'b0} + {45'b0, gidx(ga, glvl - 1'b1, 1'b0)} * 56'd8;
                st   <= P_REQ;
              end else begin
                // G 叶子：PA 页号 = G 叶子 PPN 按 G 页级拼 GPA 低位
                logic [46:0] gm, hp;
                logic [2:0]  cl;
                logic        cn, bxf, bxv;
                gm = pmask(glvl, n);
                hp = ({3'b0, pp} & ~gm) | (ga[58:12] & gm);
                if (prank(glvl, n) < prank(vs_lvl, vs_nap)) begin cl = glvl; cn = n; end
                else                                         begin cl = vs_lvl; cn = vs_nap; end
                // PA 超出 PA_W：G-PTE 自己的 PPN 位超宽 (bxf) 时整页都超；只来自 GPA 位 (只有 4 级 G 页) 时依 VA 而定 ——
                //   合并页级 < 4 时这些 GPA 位已被 VS 叶子固定，照样填访问错误项；合并页级 = 4 时填普通项，
                //   由 TLB 查找的「4 级页 VA[47:40] 非零 -> 访问错误」逐个 VPN 判 (同一级的普通项)
                bxf = ((({3'b0, pp} & ~gm) >> (PA_W - 12)) != '0);
                bxv = (hp[46:PA_W-12] != '0) && !bxf;
                if (bxf || (bxv && (!gfin || cl != 3'd4))) begin
                  fin(2'd2, fl, 1'b0, '0, '0, '0, '0, '0, 1'b0);       // 物理地址超出实现宽度
                end else if (!gfin) begin
                  // VS PTE 的物理地址到了：回 S1 读它
                  ph <= 1'b0;
                  a  <= {hp[43:0], ga[11:0]};
                  st <= P_REQ;
                end else begin
                  // 两级完成：合并页级 (两级较小者)
                  fin(2'd0, cl, cn, hp[43:0], vs_perm, vs_pbmt, {dd, x, w, r}, ga[58:2], 1'b0);
                end
              end
            end
          end
        end
        P_DONE: st <= P_IDLE;
        default: st <= P_IDLE;
      endcase
    end
  end

  assign ifill_o      = (st == P_DONE) && !is_d && !abort && !flush_i;
  assign dfill_o      = (st == P_DONE) &&  is_d && !abort && !flush_i;
  assign fill_vpn_o   = vpn;
  assign fill_v_o     = rv;
  assign fill_level_o = r_lvl;
  assign fill_nap_o   = r_nap;
  assign fill_ppn_o   = r_ppn[PA_W-13:0];
  assign fill_perm_o  = r_perm;
  assign fill_pbmt_o  = r_pbmt;
  assign fill_fault_o = r_fault;
  assign fill_gperm_o = r_gperm;
  assign fill_gpa_o   = r_gpa;
  assign fill_gimp_o  = r_gimp;
endmodule
`default_nettype wire
