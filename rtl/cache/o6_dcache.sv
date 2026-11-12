//==============================================================================
// o6_dcache.sv - L1 数据缓存：64 KB / 16 路 / 64 B 行 / 8 bank / VIPT / 写回 + 写分配 / 非阻塞
//
// 组织：
//   标签、有效、脏、PLRU：触发器 (两个 load 口 + store 口 + 回填同时查，触发器阵列多读口免费)
//   数据：8 个 bank (地址 [5:3])，每 bank 一块 64 组 x 1024 位 (16 路 x 64 位) 的单口 SRAM
//         一个 load 只读一个 bank；NLD 个 load 口 (= N_LDP 条 load 流水) 不同 bank 并行；同 bank 同组共享一次读，
//         同 bank 不同组时编号小的口优先，其余回 nack (LSU 重放)
//
// load 流水 (与 LSU 对齐)：
//   L0：bank 仲裁 + SRAM 读 (组号来自虚地址 [11:6]，VIPT 无别名)
//   L1：SRAM 输出 16 路；用 L1 的物理地址比标签 -> 命中路选数；缺失则查 / 分配 MSHR
//   L2：给 LSU hit / nack / miss(+MSHR 号) / 数据
//
// 缺失：MSHR 只记行地址；数据回来时才选牺牲路 (回填 A 拍读牺牲行进写回缓冲，B 拍写新行并改标签)，
//   所以等回填期间旧行照常可用，不需要"预留路"。同一行的后续缺失合并到同一个 MSHR。
//
// store / AMO (SQ 头排空)：s0 查标签；命中则写 bank (AMO 先读、下一拍算并写回)；向量的整行模式同时动 8 个 bank，
//   完成脉冲在写入后一拍；缺失则挂 MSHR，回填后重试。请求在完成前保持。
//
// 端口优先级 (每个 bank 单口)：回填 > store/AMO > load (被挤掉的 load 回 nack，LSU 重放)。
//
// fence.i：全量写回脏行 (逐组逐路)，完成后给 flush_done。
//==============================================================================
`default_nettype none
module o6_dcache
  import o6_pkg::*;
#(
  parameter int NMSHR = DC_NMSHR,
  parameter int NLD   = N_LDP
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // ---- LSU load 口 (NLD 个)
  input  wire  [NLD-1:0]      ld_req_i,           // L0
  input  wire  [PA_W-1:0]     ld_addr_i [NLD],    // L0：用 [11:3]
  input  wire  [PA_W-1:0]     ld_paddr_i [NLD],   // L1
  input  wire  [NLD-1:0]      ld_kill_i,          // L1
  output logic [NLD-1:0]      ld_hit_o,           // L2
  output logic [NLD-1:0]      ld_nack_o,          // L2
  output logic [63:0]         ld_data_o [NLD],    // L2
  output logic [MSHR_W-1:0]   ld_mshr_o [NLD],    // L2
  output logic                refill_o,           // 某 MSHR 回填完成 (脉冲)
  output logic [MSHR_W-1:0]   refill_mshr_o,

  // ---- LSU store / AMO 口
  input  wire                 st_req_i,
  input  wire  [PA_W-1:0]     st_paddr_i,
  input  wire  [PA_W-1:0]     st_next_paddr_i,    // 下一条 store 的地址：done 拍预查标签，省掉一拍
  input  wire  [PA_W-1:0]     st_lk_paddr_i,      // 只由 LSU 寄存器决定的查表地址 (查标签与动作拍核对都用它)
  input  wire  [63:0]         st_data_i,          // 双字对齐
  input  wire  [7:0]          st_mask_i,
  input  wire                 st_amo_i,
  input  wire  [4:0]          st_amo_op_i,
  input  wire  [1:0]          st_size_i,
  output logic                st_done_o,
  output logic [63:0]         st_rdata_o,
  // 整行模式 (向量访存)：一次读 / 写一整行 64 B，8 个 bank 同时动 (那一拍 load 全部 nack)。
  // 读走 AMO_LR 的流程 (S_LOOK 读 -> S_AMO_RD 锁存整行 -> S_AMO_WR 完成)，写走普通 store (64 位字节掩码)
  input  wire                 st_line_i,
  input  wire  [511:0]        st_ldata_i,
  input  wire  [63:0]         st_lmask_i,
  output logic [511:0]        st_lrdata_o,

  // ---- fence.i 写回
  // store 预取：store 地址确定时 (LSU 的 STA) 提前为它的行申请 MSHR，SQ 排到它时多半已命中
  input  wire                 pf_req_i,
  input  wire  [PA_W-1:0]     pf_paddr_i,
  input  wire                 flush_req_i,
  output logic                flush_done_o,

  // ---- 下游 (L2 / 总线)
  output logic                mem_req_valid_o,
  input  wire                 mem_req_ready_i,
  output logic                mem_req_we_o,       // 1 = 写回整行
  output logic [PA_W-1:6]     mem_req_addr_o,
  output logic [511:0]        mem_req_data_o,
  output logic [3:0]          mem_req_id_o,
  input  wire                 mem_resp_valid_i,
  output logic                mem_resp_ready_o,
  input  wire  [3:0]          mem_resp_id_i,
  input  wire  [511:0]        mem_resp_data_i,
  // MBIST (各 SRAM 的 March C− 引擎；done 全与、fail 全或)
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int NSET = 64;
  localparam int NWAY = L1_NWAY;
  localparam int WB   = L1_WB;
  localparam int BW   = 64 * NWAY;     // 每 bank 一组的位宽
  localparam int TW   = PA_W - 12;     // 标签宽 (组号 6 位 + 行内 6 位)

  //==========================================================================
  // 标签与状态 (触发器)
  //==========================================================================
  logic [TW-1:0]   tag   [NSET][NWAY];
  logic [NWAY-1:0] valid [NSET];
  logic [NWAY-1:0] dirty [NSET];
  logic [NWAY-2:0] plru  [NSET];

  function automatic logic [NWAY-1:0] lookup(input logic [PA_W-1:0] pa,
                                             input logic [TW-1:0] tg [NSET][NWAY],
                                             input logic [NWAY-1:0] vd [NSET]);
    logic [NWAY-1:0] h;
    for (int w = 0; w < NWAY; w++) h[w] = vd[pa[11:6]][w] && tg[pa[11:6]][w] == pa[PA_W-1:12];
    return h;
  endfunction

  function automatic logic [WB-1:0] oh2i(input logic [NWAY-1:0] oh);
    logic [WB-1:0] r;
    r = '0;
    for (int w = 0; w < NWAY; w++) if (oh[w]) r = WB'(w);
    return r;
  endfunction
  function automatic logic [MSHR_W-1:0] m2i(input logic [NMSHR-1:0] oh);
    logic [MSHR_W-1:0] r;
    r = '0;
    for (int i = 0; i < NMSHR; i++) if (oh[i]) r = MSHR_W'(i);
    return r;
  endfunction


  //==========================================================================
  // 数据 SRAM：8 bank
  //==========================================================================
  logic [7:0]      b_ce, b_we;
  logic [5:0]      b_addr [8];
  logic [BW-1:0]   b_wdata [8], b_wmask [8], b_rdata [8];

  logic [7:0]      bd, bf;
  for (genvar b = 0; b < 8; b++) begin : g_bank
    o6_sram_sp #(.DEPTH(NSET), .WIDTH(BW)) u_bank (
      .clk_i, .ce_i(b_ce[b]), .we_i(b_we[b]), .addr_i(b_addr[b]),
      .wdata_i(b_wdata[b]), .wmask_i(b_wmask[b]), .rdata_o(b_rdata[b]),
      .bist_start_i, .bist_done_o(bd[b]), .bist_fail_o(bf[b])
    );
  end
  assign bist_done_o = &bd;
  assign bist_fail_o = |bf;

  //==========================================================================
  // MSHR
  //==========================================================================
  typedef enum logic [1:0] {M_FREE, M_REQ, M_WAIT, M_FILL} mst_e;
  mst_e             m_st   [NMSHR];
  logic [PA_W-1:6]  m_line [NMSHR];
  logic [511:0]     m_data [NMSHR];

  function automatic logic [NMSHR-1:0] mshr_match(input logic [PA_W-1:6] l, input mst_e st [NMSHR],
                                                  input logic [PA_W-1:6] ml [NMSHR]);
    logic [NMSHR-1:0] r;
    for (int i = 0; i < NMSHR; i++) r[i] = (st[i] != M_FREE) && ml[i] == l;
    return r;
  endfunction

  //==========================================================================
  // 回填 FSM：A 读牺牲行，B 写新行
  //==========================================================================
  typedef enum logic [1:0] {F_IDLE, F_RDV, F_WR} fst_e;
  fst_e             f_st;
  logic [MSHR_W-1:0] f_m;          // 正在回填的 MSHR
  logic [WB-1:0]    f_way;
  logic [5:0]       f_set;

  // 写回缓冲 (1 项)
  logic             wb_v;
  logic [PA_W-1:6]  wb_line;
  logic [511:0]     wb_data;
  logic             wb_fill;      // 写回缓冲的数据要在 F_WR 拍从 SRAM 输出取

  //==========================================================================
  // store / AMO FSM
  //==========================================================================
  typedef enum logic [2:0] {S_IDLE, S_LOOK, S_AMO_RD, S_AMO_EX, S_AMO_WR, S_DONE, S_MISS, S_CBO} sst_e;   // S_CBO：Zicbom 写回 / 作废
  sst_e             s_st;
  logic [WB-1:0]    s_way;
  logic [MSHR_W-1:0] s_m;
  logic [63:0]      amo_old_q;     // AMO 旧值 (S_AMO_RD 拍锁存，之后 SRAM 输出可能被 load 冲掉)
  logic [511:0]     lrd_q;         // 整行读的数据 (S_AMO_RD 拍锁存)
  logic [63:0]      amo_old_dw, amo_new, amo_mask;
  logic [63:0]      amo_new_q, amo_mask_q;   // S_AMO_EX 拍算好寄存，S_AMO_WR 拍写 (6 GHz：移位 + 64 位运算 + 移位 + bank 写数据一拍放不下)
  logic [NMSHR-1:0] s_match;
  logic             st_miss_alloc;
  logic [MSHR_W-1:0] st_alloc_idx;
  logic             st_alloc_ok;

  //==========================================================================
  // fence.i 全量写回
  //==========================================================================
  typedef enum logic [1:0] {X_IDLE, X_SCAN, X_RD, X_WAIT} xst_e;
  xst_e             x_st;
  logic [5:0]       x_set;
  logic [WB-1:0]    x_way;

  //==========================================================================
  // bank 仲裁 (本拍)
  //==========================================================================
  // 回填 A/B、fence 读：占全部 bank
  // 回填与 fence 写回期间独占全部 bank (load 一律 nack，store 等)：
  // 它们要在下一拍从 SRAM 输出取数，期间任何读都会冲掉输出
  wire        fill_all = (f_st != F_IDLE) || (x_st != X_IDLE);
  // store / AMO：一个 bank
  // store / AMO 两拍：S_IDLE 拍用输入地址查标签、结果寄存 (spa_q / s_hit_q)，S_LOOK 拍按寄存结果动作
  // (写数据 / 分配 MSHR / 进 AMO)。之间若有回填 (fill_all) 则回 S_IDLE 重查；MSHR 匹配在 S_LOOK 用寄存地址重算。
  // LSU 一旦发起某次请求就保持到 done，地址不变 (断言核对)
  logic [PA_W-1:0] spa_q;
  logic [NWAY-1:0] s_hit_q;
  logic            s_lkf_q;        // 查标签那拍正在回填 (标签可能在那拍末被改写)：S_LOOK 重查
  // 查标签用的地址：S_IDLE 用当前请求，S_DONE (本条已完成、LSU 本拍出队) 预查下一条
  wire  [PA_W-1:0] s_lk_pa  = (s_st == S_DONE) ? st_next_paddr_i : st_lk_paddr_i;
  wire  [NWAY-1:0] s_lk_oh  = lookup(s_lk_pa, tag, valid);
  wire  [PA_W-1:0] spa      = spa_q;
  wire  [NWAY-1:0] s_hit_oh = s_hit_q;
  wire             s_hit    = |s_hit_oh;
  wire [2:0]      s_bank   = spa[5:3];
  logic           st_bank_use;    // 本拍 store/AMO 用 bank
  logic           st_bank_we;

  always_comb begin
    st_bank_use = 1'b0;
    st_bank_we  = 1'b0;
    if (!fill_all) begin
      if (s_st == S_LOOK && st_req_i && !s_lkf_q && st_lk_paddr_i == spa_q && s_hit) begin
        st_bank_use = 1'b1;
        st_bank_we  = !st_amo_i;               // 普通 store 直接写；AMO 先读
      end else if (s_st == S_AMO_WR) begin
        st_bank_use = 1'b1;
        st_bank_we  = 1'b1;
      end
    end
  end

  // load 的 bank 仲裁：与 store 同 bank 让 store；与编号更小的口同 bank 不同组让它 (同组共享一次读)
  logic [NLD-1:0] l0_go;          // 本拍 load 拿到 bank
  always_comb begin
    for (int k = 0; k < NLD; k++) begin
      l0_go[k] = ld_req_i[k] && !fill_all && !(st_bank_use && (st_line_i || ld_addr_i[k][5:3] == s_bank));
      for (int j = 0; j < k; j++)
        if (ld_req_i[j] && ld_addr_i[j][5:3] == ld_addr_i[k][5:3] && ld_addr_i[j][11:6] != ld_addr_i[k][11:6])
          l0_go[k] = 1'b0;
    end
  end

  always_comb begin
    for (int b = 0; b < 8; b++) begin
      b_ce[b] = 1'b0; b_we[b] = 1'b0; b_addr[b] = '0; b_wdata[b] = '0; b_wmask[b] = '0;
    end
    if (f_st == F_RDV) begin
      for (int b = 0; b < 8; b++) begin b_ce[b] = 1'b1; b_addr[b] = f_set; end
    end else if (f_st == F_WR) begin
      for (int b = 0; b < 8; b++) begin
        b_ce[b] = 1'b1; b_we[b] = 1'b1; b_addr[b] = f_set;
        b_wdata[b] = {NWAY{m_data[f_m][64*b +: 64]}};
        b_wmask[b] = '0;
        b_wmask[b][64*f_way +: 64] = '1;
      end
    end else if (x_st == X_RD) begin
      for (int b = 0; b < 8; b++) begin b_ce[b] = 1'b1; b_addr[b] = x_set; end
    end else begin
      if (st_bank_use && st_line_i) begin
        // 整行：8 个 bank 同组同路；整行读 (LR) 在 S_AMO_WR 不写
        logic [WB-1:0] w;
        w = (s_st == S_AMO_WR) ? s_way : oh2i(s_hit_oh);
        for (int b = 0; b < 8; b++) begin
          b_ce[b]    = 1'b1;
          b_we[b]    = st_bank_we && (s_st != S_AMO_WR);
          b_addr[b]  = spa[11:6];
          b_wdata[b] = {NWAY{st_ldata_i[64*b +: 64]}};
          for (int i = 0; i < 8; i++) b_wmask[b][64*w + 8*i +: 8] = {8{st_lmask_i[8*b + i]}};
        end
      end else if (st_bank_use) begin
        logic [WB-1:0] w;
        w = (s_st == S_AMO_WR) ? s_way : oh2i(s_hit_oh);
        b_ce[s_bank]   = 1'b1;
        b_we[s_bank]   = st_bank_we;
        b_addr[s_bank] = spa[11:6];
        if (s_st == S_AMO_WR) begin
          b_wdata[s_bank] = {NWAY{amo_new_q}};
          b_wmask[s_bank][64*w +: 64] = amo_mask_q;
        end else begin
          b_wdata[s_bank] = {NWAY{st_data_i}};
          for (int i = 0; i < 8; i++) b_wmask[s_bank][64*w + 8*i +: 8] = {8{st_mask_i[i]}};
        end
      end
      for (int k = 0; k < NLD; k++) begin
        if (l0_go[k]) begin
          b_ce[ld_addr_i[k][5:3]]   = 1'b1;
          b_addr[ld_addr_i[k][5:3]] = ld_addr_i[k][11:6];
        end
      end
    end
  end

  //==========================================================================
  // load L1 / L2
  //==========================================================================
  logic [NLD-1:0] l1_v, l1_go;
  logic [2:0]  l1_bank [NLD];
  logic [NLD-1:0] l2_hit, l2_nack, l2_miss;
  logic [63:0] l2_data [NLD];
  logic [MSHR_W-1:0] l2_mshr [NLD];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin l1_v <= '0; l1_go <= '0; end
    else begin l1_v <= ld_req_i; l1_go <= l0_go; end
  end
  always_ff @(posedge clk_i) begin
    for (int k = 0; k < NLD; k++) l1_bank[k] <= ld_addr_i[k][5:3];
  end

  // L1：比标签、选数、MSHR 查找 / 分配 (两个口按序分配)
  logic [NWAY-1:0]  l1_hit_oh [NLD];
  logic [NLD-1:0]   l1_alloc;            // 需要新分配 MSHR
  logic [MSHR_W-1:0] l1_alloc_idx [NLD];
  logic [NLD-1:0]   l1_alloc_ok;
  logic [NMSHR-1:0] l1_match [NLD];
  // MSHR 分配按"秩"并行算 (2026-10-24，6 GHz)：原来 store 先占最低号空闲项，3 个 load 口再依次
  // "找第一个空闲 -> 占用 -> 下一个口再找"，后面还有预取数剩余空闲 (2nm：spa_q -> m_line 253 ps)。
  // 现在：各空闲项在空闲者中的序号并行算出 (fr_pick[r] = 第 r 个空闲项)；每个口拿第几个只取决于它前面
  // 有几个要分配 (store 算第 0 个)；同行合并只在 3 个口之间比较。原逐个查找的写法留作参考模型 (下面断言)。
  localparam int NPICK = NLD + 2;              // store + 各 load 口 + 预取
  logic [NMSHR-1:0]  fr0;                      // 本拍开始时空闲的 MSHR
  logic [MSHR_W-1:0] fr_pick [NPICK];          // 第 r 个空闲项 (按编号从小到大)
  logic [MSHR_W:0]   fr_rk   [NMSHR];          // 各项在空闲者中的序号 (只对空闲项有意义)
  localparam int NHAVE = NPICK + 3;            // 预取要看 "已分配数 + 3"，最大 NLD + 1 + 3
  logic [NHAVE:0]    fr_have;                  // 至少有 r 个空闲 (fr_have[r] = 空闲数 >= r)
  always_comb begin
    for (int i = 0; i < NMSHR; i++) fr0[i] = (m_st[i] == M_FREE);
    for (int q = 0; q < NPICK; q++) fr_pick[q] = '0;
    for (int i = 0; i < NMSHR; i++) begin
      logic [MSHR_W:0] rk;
      rk = '0;
      for (int j = 0; j < i; j++) rk = rk + (MSHR_W+1)'(fr0[j]);
      fr_rk[i] = rk;
      for (int q = 0; q < NPICK; q++) if (fr0[i] && rk == (MSHR_W+1)'(q)) fr_pick[q] = MSHR_W'(i);
    end
    begin
      logic [MSHR_W:0] nf;
      nf = '0;
      for (int i = 0; i < NMSHR; i++) nf = nf + (MSHR_W+1)'(fr0[i]);
      for (int q = 0; q <= NHAVE; q++) fr_have[q] = nf >= (MSHR_W+1)'(q);
    end
  end
  logic [NLD-1:0] l1_want;                    // 自己分配新 MSHR (不与前面的口合并)
  logic [2:0]     l1_nal;                     // store + 成功分配的 load 口数 (预取取下一个空闲项)
  logic [2:0]     l1_rk [NLD];                // 各 load 口要的是第几个空闲项
  always_comb begin
    logic [NLD-1:0] miss0, dup;
    for (int k = 0; k < NLD; k++) begin
      l1_hit_oh[k] = lookup(ld_paddr_i[k], tag, valid);
      l1_match[k]  = mshr_match(ld_paddr_i[k][PA_W-1:6], m_st, m_line);
      // 同拍 store 缺失正在为同一行分配 MSHR：合并到它 (否则同一行取两次、回填进两路，
      // 先回填那份上的 store 被后回填的旧数据遮住 —— SoC 随机种子 1692 复现)
      if (st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin
        l1_match[k] = '0; l1_match[k][st_alloc_idx] = 1'b1;
      end
      miss0[k] = l1_v[k] && l1_go[k] && !ld_kill_i[k] && !(|l1_hit_oh[k]) && !(|l1_match[k]);
    end
    // 几个口缺同一行：后面的合并到前面第一个自己分配的口
    dup = '0;
    for (int k = 0; k < NLD; k++) begin
      for (int j = 0; j < k; j++)
        if (miss0[j] && !dup[j] && miss0[k] && ld_paddr_i[k][PA_W-1:6] == ld_paddr_i[j][PA_W-1:6]) dup[k] = 1'b1;
      l1_want[k] = miss0[k] && !dup[k];
    end
    // 静态秩 (2026-10-28，6 GHz)：store 固定第 0 个空闲项、load 口 k 固定第 k+1 个 (不管前面的口要不要)，
    // 这样 m_line 的写数据只取决于空闲项的秩 (来自 m_st) 与各口地址，不等命中 / 缺失判断 (见 ma_line)。
    // 代价：口 k 要至少 k+2 个空闲才拿得到 (原来按实际分配者紧凑排)，只在 15 个 MSHR 将满时有差别
    for (int k = 0; k < NLD; k++) begin
      l1_rk[k]        = 3'(k + 1);
      l1_alloc[k]     = l1_want[k];
      l1_alloc_ok[k]  = l1_want[k] && fr_have[k + 2];
      l1_alloc_idx[k] = l1_alloc_ok[k] ? fr_pick[k + 1] : '0;
    end
    // 合并到前面那个口：它拿到了就指向它的 MSHR；它没拿到 (MSHR 满) 则本口同样当"要分配但没拿到"(nack)
    for (int k = 0; k < NLD; k++)
      if (dup[k]) begin
        logic found;
        found = 1'b0;
        for (int j = 0; j < k; j++)
          if (!found && l1_want[j] && ld_paddr_i[k][PA_W-1:6] == ld_paddr_i[j][PA_W-1:6]) begin
            found = 1'b1;
            if (l1_alloc_ok[j]) begin l1_match[k] = '0; l1_match[k][l1_alloc_idx[j]] = 1'b1; end
            else                begin l1_alloc[k] = 1'b1; l1_alloc_ok[k] = 1'b0; l1_alloc_idx[k] = '0; end
          end
      end
    l1_nal = 3'(st_miss_alloc);
    for (int k = 0; k < NLD; k++) l1_nal = l1_nal + 3'(l1_want[k] && l1_alloc_ok[k]);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin l2_hit <= '0; l2_nack <= '0; l2_miss <= '0; end
    else begin
      for (int k = 0; k < NLD; k++) begin
        logic act;
        act = l1_v[k] && !ld_kill_i[k];
        l2_hit[k]  <= act && l1_go[k] && (|l1_hit_oh[k]);
        // 没拿到 bank，或缺失但 MSHR 满：nack (LSU 立即重放)
        l2_nack[k] <= act && (!l1_go[k] || (l1_alloc[k] && !l1_alloc_ok[k]));
        l2_miss[k] <= act && l1_go[k] && !(|l1_hit_oh[k]) && !(l1_alloc[k] && !l1_alloc_ok[k]);
      end
    end
  end
  always_ff @(posedge clk_i) begin
    for (int k = 0; k < NLD; k++) begin
      l2_data[k] <= b_rdata[l1_bank[k]][64*oh2i(l1_hit_oh[k]) +: 64];
      l2_mshr[k] <= l1_alloc[k] ? l1_alloc_idx[k] : m2i(l1_match[k]);
    end
  end

  assign ld_hit_o  = l2_hit;
  assign ld_nack_o = l2_nack;
  assign ld_data_o = l2_data;
  assign ld_mshr_o = l2_mshr;

  //==========================================================================
  // store / AMO
  //==========================================================================
  always_comb begin
    s_match = mshr_match(spa[PA_W-1:6], m_st, m_line);
    st_alloc_idx = fr_pick[0]; st_alloc_ok = fr_have[1];
    st_miss_alloc = (s_st == S_LOOK) && st_req_i && !fill_all && !s_lkf_q && st_lk_paddr_i == spa_q &&
                    !s_hit && !(|s_match) && st_alloc_ok;
  end

  // 预取口打拍 (2026-11-06，6 GHz：跨模块 LSU 选择 -> pf_paddr_i -> 查 tag / MSHR -> pf_alloc -> m_st −88.33)。
  // 预取只是提示，晚一拍无妨；地址每拍直接采样，不加使能 (下游只在 pf_v_q 时才有效)
  logic            pf_v_q;
  logic [PA_W-1:0] pf_pa_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) pf_v_q <= 1'b0;
    else         pf_v_q <= pf_req_i;
  always_ff @(posedge clk_i) pf_pa_q <= pf_paddr_i;

  // 预取分配：缺失、无同行 MSHR / 写回、本拍别的口没分配同一行、不在回填/全量写回期间，
  // 且分配后仍至少剩 2 个空 MSHR 给按需缺失
  logic       pf_alloc;
  logic [MSHR_W-1:0] pf_idx;
  logic       ld_same;        // 本拍某个 load 口正在为同一行分配
  always_comb begin
    ld_same = 1'b0;
    for (int k = 0; k < NLD; k++)
      if (l1_alloc[k] && ld_paddr_i[k][PA_W-1:6] == pf_pa_q[PA_W-1:6]) ld_same = 1'b1;
  end
  // 预取取编号最大的空闲项 (fr_last)；要求空闲数 >= NLD + 2，它的秩就大于 store / 各 load 口的静态秩，不会撞
  logic [NMSHR-1:0] fr_last;
  always_comb
    for (int i = 0; i < NMSHR; i++) begin
      fr_last[i] = fr0[i];
      for (int j = i + 1; j < NMSHR; j++) if (fr0[j]) fr_last[i] = 1'b0;
    end
  always_comb begin
    logic [PA_W-1:6]  pl;
    pl = pf_pa_q[PA_W-1:6];
    // 分配后至少剩 2 个：空闲数 >= 已分配数 + 3
    pf_idx = '0;
    for (int i = 0; i < NMSHR; i++) if (fr_last[i]) pf_idx = MSHR_W'(i);
    pf_alloc = pf_v_q && !fill_all && fr_have[l1_nal + 3'd3] && fr_have[NLD + 2] &&
               !(|lookup(pf_pa_q, tag, valid)) && !(|mshr_match(pl, m_st, m_line)) &&
               !(wb_v && wb_line == pl) &&
               !(st_miss_alloc && spa[PA_W-1:6] == pl) && !ld_same;
  end


`ifndef SYNTHESIS
  // 参考模型 (2026-10-28 改为静态秩)：按编号列出本拍开始时的空闲项，store 取第 0 个、load 口 k 取第 k+1 个
  // (要求空闲数 >= k+2)、预取取最后一个；逐拍核对并行版
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      logic [NMSHR-1:0] freem;
      logic [NLD-1:0]   ra, rok;
      logic [MSHR_W-1:0] ridx [NLD];
      logic [NMSHR-1:0] rm [NLD];
      logic [3:0]       nfree;
      logic [MSHR_W-1:0] rpf;
      logic [MSHR_W-1:0] fl [NMSHR];
      int               nf0;
      nf0 = 0;
      for (int i = 0; i < NMSHR; i++) begin fl[i] = '0; freem[i] = (m_st[i] == M_FREE); end
      for (int i = 0; i < NMSHR; i++) if (freem[i]) begin fl[nf0] = MSHR_W'(i); nf0++; end
      if (st_miss_alloc) freem[st_alloc_idx] = 1'b0;
      for (int k = 0; k < NLD; k++) begin
        logic [NWAY-1:0] h;
        h = lookup(ld_paddr_i[k], tag, valid);
        rm[k] = mshr_match(ld_paddr_i[k][PA_W-1:6], m_st, m_line);
        if (st_miss_alloc && spa[PA_W-1:6] == ld_paddr_i[k][PA_W-1:6]) begin rm[k] = '0; rm[k][st_alloc_idx] = 1'b1; end
        ra[k] = l1_v[k] && l1_go[k] && !ld_kill_i[k] && !(|h) && !(|rm[k]);
        rok[k]  = nf0 >= k + 2;
        ridx[k] = rok[k] ? fl[k + 1] : '0;
        for (int j = 0; j < k; j++)
          if (ra[k] && ra[j] && rok[j] && ld_paddr_i[k][PA_W-1:6] == ld_paddr_i[j][PA_W-1:6]) begin
            ra[k] = 1'b0; rm[k] = '0; rm[k][ridx[j]] = 1'b1;
          end
        if (ra[k] && rok[k]) freem[ridx[k]] = 1'b0;
      end
      nfree = '0;
      for (int i = 0; i < NMSHR; i++) nfree = nfree + 4'(freem[i]);
      rpf = (nf0 > 0) ? fl[nf0 - 1] : '0;
      for (int k = 0; k < NLD; k++) begin
        assert (ra[k] == l1_alloc[k]) else $error("dcache: MSHR 并行分配 口 %0d alloc %0d/参考 %0d", k, l1_alloc[k], ra[k]);
        if (ra[k]) assert (rok[k] == l1_alloc_ok[k] && (!rok[k] || ridx[k] == l1_alloc_idx[k]))
          else $error("dcache: MSHR 并行分配 口 %0d ok/idx %0d/%0d 参考 %0d/%0d", k, l1_alloc_ok[k], l1_alloc_idx[k], rok[k], ridx[k]);
        if (!ra[k]) assert (rm[k] == l1_match[k]) else $error("dcache: MSHR 并行分配 口 %0d match %h/参考 %h", k, l1_match[k], rm[k]);
      end
      if (pf_v_q && !fill_all) assert ((nfree >= 4'd3) == fr_have[l1_nal + 3'd3] && (nfree < 4'd3 || rpf == pf_idx))
        else $error("dcache: 预取分配 剩余 %0d 参考号 %0d / 并行号 %0d", nfree, rpf, pf_idx);
    end
`endif

  // MSHR 分配的按项写使能 (2026-10-25，6 GHz)：第 i 项被分配 <=> 它空闲且它的序号等于某个分配者要的序号
  // (store 要第 0 个，load 口 k 要第 k+1 个，预取要最后一个空闲项)。不再 "选出编号 -> 再按编号译码写使能"
  // (b180n：m_st -> 秩 -> 编号 -> 译码 -> m_line 204 ps)；编号 l1_alloc_idx 只给回给 LSU 的 MSHR 号用
  // 2026-10-28：行地址 ma_line 按静态秩选 (秩 0 = store、秩 k+1 = load 口 k、最后一个空闲项 = 预取)，空闲项每拍都写，
  // 与"谁真的分配"无关；只有状态 m_st 按 ma_we (真分配) 置 M_REQ。空闲项的 m_line 没人读 (查找 / 请求 / 回填都按状态门控)
  // (p155u56：m_st -> MSHR 匹配 -> 缺失 -> 分配 -> 写使能 -> m_line 185 ps；ld_paddr 起点 in 组 −89)
  logic [NMSHR-1:0] ma_we;
  logic [PA_W-1:6]  ma_line [NMSHR];
  always_comb
    for (int i = 0; i < NMSHR; i++) begin
      logic a_st, a_pf;
      logic [NLD-1:0] a_ld;
      a_st = st_miss_alloc && fr0[i] && fr_rk[i] == '0;
      a_pf = pf_alloc && fr_last[i];
      for (int k = 0; k < NLD; k++) a_ld[k] = l1_want[k] && fr0[i] && fr_rk[i] == (MSHR_W+1)'(k + 1);
      ma_we[i]   = a_st || a_pf || (|a_ld);
      ma_line[i] = fr_last[i] ? pf_pa_q[PA_W-1:6] : m_line[i];
      if (fr_rk[i] == '0) ma_line[i] = spa[PA_W-1:6];
      for (int k = 0; k < NLD; k++) if (fr_rk[i] == (MSHR_W+1)'(k + 1)) ma_line[i] = ld_paddr_i[k][PA_W-1:6];
    end
`ifndef SYNTHESIS
  // 与按编号写的原写法一致
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      logic [NMSHR-1:0] we;
      logic [PA_W-1:6]  ln [NMSHR];
      we = '0;
      for (int i = 0; i < NMSHR; i++) ln[i] = 'x;
      for (int k = 0; k < NLD; k++) if (l1_alloc[k] && l1_alloc_ok[k]) begin we[l1_alloc_idx[k]] = 1'b1; ln[l1_alloc_idx[k]] = ld_paddr_i[k][PA_W-1:6]; end
      if (st_miss_alloc) begin we[st_alloc_idx] = 1'b1; ln[st_alloc_idx] = spa[PA_W-1:6]; end
      if (pf_alloc) begin we[pf_idx] = 1'b1; ln[pf_idx] = pf_pa_q[PA_W-1:6]; end
      assert (we == ma_we) else $error("dcache: MSHR 按项写使能 %h 与按编号 %h 不一致", ma_we, we);
      for (int i = 0; i < NMSHR; i++)
        if (we[i]) assert (ln[i] == ma_line[i]) else $error("dcache: MSHR %0d 分配行地址不一致", i);
    end
`endif

  assign st_lrdata_o = lrd_q;

  // AMO 运算 (在 S_AMO_EX 拍，用 S_AMO_RD 锁存的旧值；结果寄存到 S_AMO_WR 写)
  // 2026-10-29 (6 GHz)：与旧值无关的部分在 S_AMO_RD 先算好寄存 —— store 侧操作数 b (移位 + W 型符号扩展)、
  // 移位量、W 标志、操作码 (原来 EX 拍 spa_q -> 移位量扇出到 a / b / 回移三个 64 位移位器 -> 运算，p155u61 −13.7)；
  // 地址 / 数据 / 宽度 / 操作码在整个 AMO 期间由 LSU 保持不变
  logic [63:0] amo_b_q, amo_b_c;
  logic [2:0]  amo_sh_q;
  logic        amo_w_q;
  logic [4:0]  amo_op_q;
  always_comb begin
    logic [63:0] b;
    b = st_data_i >> {spa[2:0], 3'b000};
    if (st_size_i == 2'd2) b = {{32{b[31]}}, b[31:0]};
    amo_b_c = b;
  end
  always_comb begin
    logic [5:0]  sh;
    logic [63:0] a, b, r;
    logic        w32;
    amo_old_dw = amo_old_q;
    sh  = {amo_sh_q, 3'b000};
    w32 = amo_w_q;
    a = amo_old_dw >> sh;
    b = amo_b_q;
    if (w32) a = {{32{a[31]}}, a[31:0]};
    unique case (amo_op_q)
      AMO_SWAP: r = b;
      AMO_ADD:  r = a + b;
      AMO_XOR:  r = a ^ b;
      AMO_AND:  r = a & b;
      AMO_OR:   r = a | b;
      AMO_MIN:  r = ($signed(a) < $signed(b)) ? a : b;
      AMO_MAX:  r = ($signed(a) > $signed(b)) ? a : b;
      AMO_MINU: r = (w32 ? (a[31:0] < b[31:0]) : (a < b)) ? a : b;
      AMO_MAXU: r = (w32 ? (a[31:0] > b[31:0]) : (a > b)) ? a : b;
      default:  r = a;     // LR：不写
    endcase
    amo_new  = r << sh;
    amo_mask = '0;
    for (int i = 0; i < 8; i++) amo_mask[8*i +: 8] = {8{st_mask_i[i]}};
    if (st_amo_op_i == AMO_LR) amo_mask = '0;
  end

  //==========================================================================
  // 下游请求：写回缓冲优先，其次 MSHR 读
  //==========================================================================
  logic [MSHR_W-1:0] req_m;
  logic       req_m_v;
  always_comb begin
    req_m = '0; req_m_v = 1'b0;
    for (int i = NMSHR-1; i >= 0; i--) if (m_st[i] == M_REQ) begin req_m = MSHR_W'(i); req_m_v = 1'b1; end
    mem_req_valid_o = (wb_v && !wb_fill) || req_m_v;
    mem_req_we_o    = wb_v && !wb_fill;
    mem_req_addr_o  = (wb_v && !wb_fill) ? wb_line : m_line[req_m];
    mem_req_data_o  = wb_data;
    mem_req_id_o    = 4'(req_m);           // 0..14 = MSHR 号 (15 留给 I$)
  end
  wire wb_sent  = mem_req_valid_o && mem_req_ready_i && mem_req_we_o;
  wire rd_sent  = mem_req_valid_o && mem_req_ready_i && !mem_req_we_o;

  // 回填：写回缓冲空才接收数据
  assign mem_resp_ready_o = (f_st == F_IDLE) && !wb_v && (x_st == X_IDLE);
  wire   resp_take = mem_resp_valid_i && mem_resp_ready_o;

  //==========================================================================
  // 状态更新
  //==========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int s = 0; s < NSET; s++) begin valid[s] <= '0; dirty[s] <= '0; plru[s] <= '0; end
      for (int i = 0; i < NMSHR; i++) m_st[i] <= M_FREE;
      f_st <= F_IDLE; s_st <= S_IDLE; x_st <= X_IDLE;
      wb_v <= 1'b0; wb_fill <= 1'b0;
      st_done_o <= 1'b0; refill_o <= 1'b0; flush_done_o <= 1'b0;
    end else begin
      st_done_o    <= 1'b0;
      refill_o     <= 1'b0;
      flush_done_o <= 1'b0;

      // ---- load 命中更新 PLRU
      for (int k = 0; k < NLD; k++)
        if (l1_v[k] && l1_go[k] && !ld_kill_i[k] && |l1_hit_oh[k])
          plru[ld_paddr_i[k][11:6]] <= l1_plru_touch(plru[ld_paddr_i[k][11:6]], oh2i(l1_hit_oh[k]));

      // ---- MSHR 分配 (load / store / 预取)：按项的写使能 (见 ma_we)
      for (int i = 0; i < NMSHR; i++) begin
        if (fr0[i])   m_line[i] <= ma_line[i];
        if (ma_we[i]) m_st[i]   <= M_REQ;
      end
      if (rd_sent) m_st[req_m] <= M_WAIT;
      if (resp_take) begin
        m_st[mem_resp_id_i[MSHR_W-1:0]]   <= M_FILL;
        m_data[mem_resp_id_i[MSHR_W-1:0]] <= mem_resp_data_i;
      end

      // ---- 写回缓冲
      if (wb_sent) wb_v <= 1'b0;

      // ---- 回填
      unique case (f_st)
        F_IDLE: begin
          logic found;
          found = 1'b0;
          for (int i = 0; i < NMSHR; i++)
            // AMO 读改写进行中不开始回填 (可能替换掉 AMO 那一行)
            if (!found && m_st[i] == M_FILL && !wb_v && x_st == X_IDLE &&
                (s_st == S_IDLE || s_st == S_MISS || s_st == S_DONE)) begin   // S_DONE：连续 store 不经 S_IDLE，别饿死回填
              found = 1'b1;
              f_m   <= MSHR_W'(i);
              f_set <= m_line[i][11:6];
              f_way <= l1_plru_victim(plru[m_line[i][11:6]]);
              f_st  <= F_RDV;
            end
        end
        F_RDV: begin
          // 牺牲行脏：下一拍从 SRAM 输出取进写回缓冲
          if (valid[f_set][f_way] && dirty[f_set][f_way]) begin
            wb_v    <= 1'b1;
            wb_fill <= 1'b1;
            wb_line <= {tag[f_set][f_way], f_set};
          end
          f_st <= F_WR;
        end
        F_WR: begin
          if (wb_fill) begin
            for (int b = 0; b < 8; b++) wb_data[64*b +: 64] <= b_rdata[b][64*f_way +: 64];
            wb_fill <= 1'b0;
          end
          tag[f_set][f_way]   <= m_line[f_m][PA_W-1:12];
          valid[f_set][f_way] <= 1'b1;
          dirty[f_set][f_way] <= 1'b0;
          plru[f_set]         <= l1_plru_touch(plru[f_set], f_way);
          m_st[f_m]           <= M_FREE;
          refill_o            <= 1'b1;
          refill_mshr_o       <= f_m;
          f_st                <= F_IDLE;
        end
        default: f_st <= F_IDLE;
      endcase

      // ---- store / AMO
      unique case (s_st)
        S_IDLE: begin
          if (st_req_i) begin
            spa_q   <= st_lk_paddr_i;
            s_hit_q <= s_lk_oh;
            s_lkf_q <= fill_all;
            s_st    <= S_LOOK;
          end
        end
        S_LOOK: begin
          // 预查的地址不是本次请求 (下一条不是 store 排空 / 换成了 PTW 等) 也重查
          if (!st_req_i || fill_all || s_lkf_q || st_lk_paddr_i != spa_q) begin
            s_st <= S_IDLE;                  // 请求撤了 / 回填改了标签：重查
          end else begin
            if (s_hit && st_bank_use) begin
              s_way <= oh2i(s_hit_oh);
              if (st_amo_i) s_st <= S_AMO_RD;
              else begin
                dirty[spa[11:6]][oh2i(s_hit_oh)] <= 1'b1;
                plru[spa[11:6]] <= l1_plru_touch(plru[spa[11:6]], oh2i(s_hit_oh));
                st_done_o <= 1'b1;
                s_st <= S_DONE;
              end
            end else if (!s_hit && (|s_match)) begin
              s_m <= m2i(s_match); s_st <= S_MISS;
            end else if (st_amo_i && (st_amo_op_i == AMO_CBOC || st_amo_op_i == AMO_CBOF)) begin
              // cbo.clean / flush 缺失 (且无同行 MSHR)：这一行不在本缓存里，无事可做，不分配
              st_done_o <= 1'b1;
              s_st <= S_DONE;
            end else if (st_miss_alloc) begin
              s_m <= st_alloc_idx; s_st <= S_MISS;
            end else begin
              // 缺失但 MSHR 全满 / bank 被占：回 S_IDLE 重查。不能停在 S_LOOK 等 ——
              // 回填只在 S_IDLE/S_MISS/S_DONE 开始，停在这里 MSHR 永远释放不了 (死锁)
              s_st <= S_IDLE;
            end
          end
        end
        S_AMO_RD: begin                      // 本拍 SRAM 输出旧值，锁存
          amo_old_q <= b_rdata[s_bank][64*s_way +: 64];
          amo_b_q   <= amo_b_c; amo_sh_q <= spa[2:0]; amo_w_q <= (st_size_i == 2'd2); amo_op_q <= st_amo_op_i;
          for (int b = 0; b < 8; b++) lrd_q[64*b +: 64] <= b_rdata[b][64*s_way +: 64];
          amo_mask_q <= '0;                    // LR 不写：跳过 S_AMO_EX，S_AMO_WR 用全 0 掩码
          if (st_line_i && (st_amo_op_i == AMO_CBOC || st_amo_op_i == AMO_CBOF)) begin
            s_st <= S_CBO;                     // cbo.clean / flush：整行已锁存进 lrd_q，下一拍写回 / 作废
          end else if (st_line_i) begin        // 整行读：不用回写，锁存即完成
            plru[spa[11:6]] <= l1_plru_touch(plru[spa[11:6]], s_way);
            st_done_o <= 1'b1;
            s_st <= S_DONE;
          end else s_st <= (st_amo_op_i == AMO_LR) ? S_AMO_WR : S_AMO_EX;
        end
        S_AMO_EX: begin                      // 运算，结果寄存 (填充只在 S_IDLE / S_MISS / S_DONE 开始，这一行不会被换走)
          amo_new_q  <= amo_new;
          amo_mask_q <= amo_mask;
          s_st <= S_AMO_WR;
        end
        S_AMO_WR: begin
          if (st_bank_use) begin
            st_rdata_o <= amo_old_dw;
            if (st_amo_op_i != AMO_LR) dirty[spa[11:6]][s_way] <= 1'b1;
            plru[spa[11:6]] <= l1_plru_touch(plru[spa[11:6]], s_way);
            st_done_o <= 1'b1;
            s_st <= S_DONE;
          end
        end
        S_DONE: begin                        // LSU 在 done 那拍出队；顺便预查下一条的标签
          spa_q   <= st_next_paddr_i;
          s_hit_q <= s_lk_oh;
          s_lkf_q <= fill_all;
          s_st    <= S_LOOK;
        end
        S_CBO: begin                         // Zicbom：脏则整行送写回缓冲 (等它空)，flush 再作废；填充只在 S_IDLE / S_MISS / S_DONE 开始
          if (!dirty[spa[11:6]][s_way] || !wb_v) begin
            if (dirty[spa[11:6]][s_way]) begin
              wb_data <= lrd_q;
              wb_v    <= 1'b1;
              wb_fill <= 1'b0;
              wb_line <= {tag[spa[11:6]][s_way], spa[11:6]};
              dirty[spa[11:6]][s_way] <= 1'b0;
            end
            if (st_amo_op_i == AMO_CBOF) valid[spa[11:6]][s_way] <= 1'b0;
            st_done_o <= 1'b1;
            s_st <= S_DONE;
          end
        end
        S_MISS: if (m_st[s_m] == M_FREE) s_st <= S_IDLE;   // 回填完成，重试
        default: s_st <= S_IDLE;
      endcase

      // ---- fence.i 全量写回
      unique case (x_st)
        X_IDLE: if (flush_req_i && f_st == F_IDLE && s_st == S_IDLE && !wb_v) begin
          x_st <= X_SCAN; x_set <= '0; x_way <= '0;
        end
        X_SCAN: begin
          if (valid[x_set][x_way] && dirty[x_set][x_way]) x_st <= X_RD;
          else if (x_set == 6'd63 && x_way == WB'(NWAY-1)) begin x_st <= X_IDLE; flush_done_o <= 1'b1; end
          else begin {x_set, x_way} <= {x_set, x_way} + 1'b1; end
        end
        X_RD: x_st <= X_WAIT;
        X_WAIT: begin
          if (!wb_v) begin
            for (int b = 0; b < 8; b++) wb_data[64*b +: 64] <= b_rdata[b][64*x_way +: 64];
            wb_v    <= 1'b1;
            wb_fill <= 1'b0;
            wb_line <= {tag[x_set][x_way], x_set};
            dirty[x_set][x_way] <= 1'b0;
            if (x_set == 6'd63 && x_way == WB'(NWAY-1)) begin x_st <= X_IDLE; flush_done_o <= 1'b1; end
            else begin {x_set, x_way} <= {x_set, x_way} + 1'b1; x_st <= X_SCAN; end
          end
        end
        default: x_st <= X_IDLE;
      endcase
    end
  end
`ifndef SYNTHESIS
  // 一致性断言：回填时同组里不能已有同一行 (重复行会让 store 写进一份、读到另一份)
  always @(posedge clk_i) if (rst_ni && f_st == F_WR)
    for (int w = 0; w < NWAY; w++)
      if (w != int'(f_way) && valid[f_set][w] && tag[f_set][w] == m_line[f_m][PA_W-1:12])
        $error("o6_dcache: 回填行 %h 在组 %0d 第 %0d 路已存在", m_line[f_m], f_set, w);
`endif
`ifndef SYNTHESIS
  // 动作拍：实际请求地址必须就是查表地址 (LSU 的占用状态已寄存)
  always_ff @(posedge clk_i)
    if (rst_ni && s_st == S_LOOK && st_req_i && !fill_all && !s_lkf_q && st_lk_paddr_i == spa_q)
      assert (st_paddr_i == spa_q) else $error("dcache: store 口动作拍地址 %h 与查表地址 %h 不符", st_paddr_i, spa_q);
`endif
endmodule
