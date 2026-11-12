//==============================================================================
// o6_iq.sv - 发射队列 (乱序，两个端口发射时动态分配，年龄矩阵)
//
// 端口在发射时才定 (uop.port 只在发射出去时填上实际端口)。按端口数 NP：
//   NP = 1：最老就绪项。
//   NP = 2：端口 0 取它能执行的最老就绪项；端口 1 取"只能走端口 1"的最老就绪项，没有则取两边都能走的
//           最年轻就绪项。三个选择并行，不串联。
//   NP = 3 (各端口执行同一类 uop，如 3 条 load 流水；端口 0 可能被临时屏蔽)：端口 0 取最老；
//           端口 1 取端口 1 可执行的最老，若它正是端口 0 选中的则取第二老；端口 2 取最年轻 (避开前两者)。
//
// 唤醒：比较 psrc 与广播 pdst (分整数/浮点类)。入队那一拍的唤醒也要算上
//   (分派级读到的 busy 是上一拍的)。
// wait_head：只在 head_go 且自己是 ROB 头时就绪。
//
// load 投机唤醒 (见 o6_pkg::wakeup_t)：每个源两级毒化 spa (下一拍判定) / spo (本拍判定)
//   - 带 sa (load 在 L0 发的) 的唤醒照常置就绪，记 spa；下一拍 spa 移到 spo；
//   - 带 sn 的唤醒置就绪并直接记 spo；
//   - load 的 L2 判定 ld_fail_i：spo 里有失败的流水 -> 该源重新置 busy；不论成败，spo 随即清掉；
//   - 带 sc 的唤醒若对应流水本拍失败 -> 作废 (不置就绪)；
//   - 本拍发射的项：有源带 spo 且本拍失败 -> 不释放 (源已重新置 busy，等真正的唤醒再发)；
//     有源带 spa (判定在下一拍) -> 先不释放、标记 isw (已发出、等判定，不再参加选择)，
//     下一拍判定失败则回到等待，成功则释放。
//     执行流水在 RF -> 下一级处丢弃被判失败的 uop (pois 看寄存的失败信号，poia 看当拍的)。
//   - 入队口另外听确认唤醒 (cwake_i：load 在 L2 的，以及整数口晚一拍补发的)：分派中的 uop 读的是
//     只吃确认唤醒的 busy 表，投机唤醒它看不到，若再错过确认就永远等不到了。
//     load 在 L0 唤醒、L2 确认，中间 L1 那拍入队的 uop 两头都听不到：入队口再听 load 在 L1 的唤醒 (ewake_i)。
// 其余时候发射即释放表项。
//==============================================================================
`default_nettype none
module o6_iq
  import o6_pkg::*;
#(
  parameter int SIZE    = 24,
  parameter int NP      = 2,            // 本地端口数
  parameter int PBASE   = 0,            // 本地端口 0 对应的全局端口号
  parameter int ENQ     = IQ_ENQ_MAX,
  parameter int N_IWAKE = 7,
  parameter int N_FWAKE = 5,
  // 1 = 唤醒用 WKCAM2_32、选择用 AGESEL_32_2 定制宏 (o6_cam_macros.sv；只对 SIZE = 32、NP <= 2)。
  // 原 RTL 的唤醒比较 / 年龄矩阵选择在仿真里保留作参考，逐拍断言一致
  parameter bit USE_CAM = 1'b0
) (
  input  wire                clk_i,
  input  wire                rst_ni,
  input  wire                flush_i,
  input  var br_update_t     br_upd_i,

  // 入队 (分派级已压缩到前 ENQ 个槽，程序序)
  input  wire  [ENQ-1:0]     enq_valid_i,
  input  var uop_t [ENQ-1:0] enq_uop_i,
  output logic [$clog2(SIZE+1)-1:0] free_cnt_o,

  // 唤醒
  input  var wakeup_t [N_IWAKE-1:0] iwake_i,
  input  var wakeup_t [N_FWAKE-1:0] fwake_i,
  input  var wakeup_t [N_CWAKE-1:0] cwake_i,      // 确认唤醒 (只给入队口)：load 在 L2 的 + 整数口补发的
  input  var wakeup_t [N_LDP-1:0]   ewake_i,      // load 在 L1 的投机唤醒 (只给入队口，带 sn)
  input  var lwake_t  [N_LDP-1:0]   lwake_i,      // load -> load 快速唤醒 (只 IQ_M 接，其余接 0)
  input  wire  [N_LDP-1:0]          ld_fail_i,    // 各 load 流水本拍 L2 判定：上拍投机唤醒的 load 失败

  // ROB 头
  input  var rob_ptr_t       rob_head_i,
  input  wire                head_go_i,
  input  var rob_ptr_t       head2_i,          // ROB 第二老项
  input  wire                head2_go_i,       // 头是已完成的向量指令：第二老的向量指令可以提前发射

  // 每端口可接受的功能单元 (位号 = fu_e)
  input  wire  [15:0]        fu_ok_i [NP],
  input  wire  [15:0]        fu_ok_nx_i [NP],  // 下一拍的 fu_ok (执行单元能算准的项算准，其余 = 本拍值)；只给 USE_CAM 的请求预测用

  // 发射
  output logic [NP-1:0]      iss_valid_o,
  output uop_t [NP-1:0]      iss_uop_o
);
  localparam int IW = $clog2(SIZE);
  localparam bit CAM = USE_CAM && SIZE == 32 && NP <= 2;
  logic [SIZE-1:0] cam_g0, cam_g1, rtl_g0, rtl_g1;
  logic            cam_c1;

  logic [SIZE-1:0] vld;
  logic [SIZE-1:0] rdy_q, rdw_q;    // 预算寄存的就绪位 (见状态更新处)
  uop_t            ent   [SIZE];
  logic [SIZE-1:0] older [SIZE];    // older[i][j] = 1：i 比 j 老

  //--------------------------------------------------------------------------
  // 唤醒判定
  //--------------------------------------------------------------------------
  // 一个源操作数本拍被唤醒：返回 {置就绪, 新毒化 sn (下一拍判定), 新毒化 sa (两拍后判定)}；
  // 带 sc 的唤醒若它依赖的 load 本拍判定失败则作废
  localparam int WHW = 3 * N_LDP + 1;
  localparam int NIW = N_IWAKE + N_LDP;      // 整数唤醒 + 入队口专用的 load L1 唤醒
  function automatic logic [WHW-1:0] whit(input preg_t p, input logic fp,
                                          input wakeup_t [NIW-1:0] iw, input wakeup_t [N_FWAKE-1:0] fw,
                                          input lwake_t [N_LDP-1:0] lw, input logic [N_LDP-1:0] fail);
    logic             set;
    logic [N_LDP-1:0] pn, pa, pb;
    set = 1'b0; pn = '0; pa = '0; pb = '0;
    if (fp) begin
      for (int k = 0; k < N_FWAKE; k++)
        if (fw[k].valid && fw[k].pdst == p && !(|(fw[k].sc & fail))) begin
          set = 1'b1; pn = pn | fw[k].sn; pa = pa | fw[k].sa;
        end
    end else begin
      for (int k = 0; k < NIW; k++)
        if (iw[k].valid && iw[k].pdst == p && !(|(iw[k].sc & fail))) begin
          set = 1'b1; pn = pn | iw[k].sn; pa = pa | iw[k].sa;
        end
      for (int k = 0; k < N_LDP; k++)
        if (lw[k].valid && lw[k].pdst == p && !(|(lw[k].sc & fail))) begin set = 1'b1; pb = pb | lw[k].sb; end
    end
    return {set, pn, pa, pb};
  endfunction
  // load 的确认唤醒 (只对入队的 uop)
  function automatic logic chit(input preg_t p, input logic fp, input wakeup_t [N_CWAKE-1:0] cw);
    logic r;
    r = 1'b0;
    for (int k = 0; k < N_CWAKE; k++) if (cw[k].valid && cw[k].is_fp == fp && cw[k].pdst == p) r = 1'b1;
    return r;
  endfunction

  // 一个源的下一状态 {busy, spo, spa, spb}：本拍判定的毒化 (spo) 遇到失败 -> 重新等待；
  // 否则按本拍唤醒置就绪，或把 spb -> spa -> spo 各推进一级
  localparam int SNW = 3 * N_LDP + 1;
  function automatic logic [SNW-1:0] snext(input logic busy, input logic [N_LDP-1:0] spo, input logic [N_LDP-1:0] spa,
                                           input logic [N_LDP-1:0] spb,
                                           input logic [WHW-1:0] wh, input logic [N_LDP-1:0] fail);
    logic             b;
    logic [N_LDP-1:0] po, pa, pb;
    b = busy; po = '0; pa = '0; pb = '0;
    if (|(spo & fail))             b = 1'b1;
    else if (busy && wh[WHW-1])    begin b = 1'b0; po = wh[3*N_LDP-1:2*N_LDP]; pa = wh[2*N_LDP-1:N_LDP]; pb = wh[N_LDP-1:0]; end
    else if (!busy)                begin po = spa; pa = spb; end
    return {b, po, pa, pb};
  endfunction

  // 各项各源的毒化：spo 本拍判定、spa 下一拍判定
  logic [2:0][N_LDP-1:0] spo [SIZE];
  logic [2:0][N_LDP-1:0] spa [SIZE];
  logic [2:0][N_LDP-1:0] spb [SIZE];   // 两拍后判定 (load -> load 快速唤醒)
  logic [N_LDP-1:0] epo [SIZE];      // 三个源合并
  logic [N_LDP-1:0] epa [SIZE];
  logic [N_LDP-1:0] epb [SIZE];
  logic [SIZE-1:0]  fhit;            // 本拍判定失败波及该项
  logic [1:0]       iswc [SIZE];     // 已发出、还要等几拍判定 (带 spa：1，带 spb：2)
  logic [SIZE-1:0]  isw;
  // 定制宏的唤醒载荷推迟一拍并入 (USE_CAM，2026-10-26，与 cam 会话的 WKCAM2_32 定稿约定)：
  // 宏的命中 H 约 119 ps 到，但载荷 Y (sc 位线各口共用，ALU 口输入 115 ps 才到) 要 138–140 ps；
  // 原来 "置就绪 = H & !(Y.sc & fail)、Y 进毒化位" 再经就绪位进 AGESEL 的锁存请求放不下。
  // 现在 t 拍命中只清 busy，Y 与本拍 load 失败寄存 (dq_*)；t+1 拍有效毒化 = 寄存值 | 未作废的待并入载荷，
  // 若那次唤醒按 sc & fail 应作废则该源重新算 busy (就绪位一并屏蔽)。读毒化的地方 (fhit、发射 pois、upd) 都用有效值。
  // 与原写法 (t 拍就并入) 状态逐拍等价 (g_cam 里用影子状态断言)；只在作废那拍宏若选中该项会空发一次
  logic [2:0]            dq_v  [SIZE];       // 上拍某源被宏唤醒 (已清 busy)、载荷待并入
  logic [2:0][11:0]      dq_y  [SIZE];       // 上拍的载荷 {sc, sb, sa, sn}
  logic [N_LDP-1:0]      dq_f;               // 上拍的 load 判定失败
  logic [2:0]            dq_c  [SIZE];       // 上拍那次唤醒应作废
  logic [SIZE-1:0]       dqc_any;
  logic [2:0][N_LDP-1:0] spo_e [SIZE], spa_e [SIZE], spb_e [SIZE];
  uop_t                  ent_e [SIZE];
  always_comb
    for (int i = 0; i < SIZE; i++) begin
      for (int sr = 0; sr < 3; sr++) begin
        logic tk;
        dq_c[i][sr] = dq_v[i][sr] && |(dq_y[i][sr][11:9] & dq_f);
        tk          = dq_v[i][sr] && !dq_c[i][sr];
        spo_e[i][sr] = spo[i][sr] | (tk ? N_LDP'(dq_y[i][sr][2:0]) : '0);
        spa_e[i][sr] = spa[i][sr] | (tk ? N_LDP'(dq_y[i][sr][5:3]) : '0);
        spb_e[i][sr] = spb[i][sr] | (tk ? N_LDP'(dq_y[i][sr][8:6]) : '0);
      end
      ent_e[i] = ent[i];
      ent_e[i].src1_busy = ent[i].src1_busy | dq_c[i][0];
      ent_e[i].src2_busy = ent[i].src2_busy | dq_c[i][1];
      ent_e[i].src3_busy = ent[i].src3_busy | dq_c[i][2];
      dqc_any[i] = |dq_c[i];
    end
  always_comb begin
    for (int i = 0; i < SIZE; i++) begin
      epo[i]  = spo_e[i][0] | spo_e[i][1] | spo_e[i][2];
      epa[i]  = spa_e[i][0] | spa_e[i][1] | spa_e[i][2];
      epb[i]  = spb_e[i][0] | spb_e[i][1] | spb_e[i][2];
      isw[i]  = iswc[i] != 2'd0;
      fhit[i] = |(epo[i] & ld_fail_i);
    end
  end

  //--------------------------------------------------------------------------
  // 选择
  //--------------------------------------------------------------------------
  logic [SIZE-1:0] killed, rdy;
  // wait_head 项"是 ROB 头"的寄存结果：ROB 头比较不进唤醒-选择环。
  // 本拍刚入队的项下拍才可能置位；头部不会越过一条还没发射的 wait_head 项，所以晚一拍不会错
  logic [SIZE-1:0] hd_q;
  logic [SIZE-1:0] hd2_q;          // 是 ROB 第二老项 (寄存；头前进后它要么成了头、要么头的条件不再成立，不会误发)
  always_comb begin
    for (int i = 0; i < SIZE; i++) begin
      uop_t u;
      u = ent[i];
      killed[i] = br_upd_i.kill && u.br_mask[br_upd_i.kill_tag];
      // 选择不看本拍的分支杀：被杀的项照样可能被选中发出，各执行单元锁存时按 br_upd 丢弃；
      // 它在发射时广播的唤醒只会唤醒同样被杀的更新指令。(否则 EXU 的分支解析 -> 杀 -> 选择 -> 发射
      // 串成一拍，也拉长唤醒-选择环)
      rdy[i]    = (rdy_q[i] || (rdw_q[i] && head_go_i && (hd_q[i] || (hd2_q[i] && head2_go_i && u.fu == FU_VEC)))) &&
                  !dqc_any[i];
    end
  end

  // 发射时动态选端口 (不再按分派时绑定的 uop.port)：
  //   端口 0：它能执行的就绪项里最老的
  //   端口 1：若有"只能走端口 1"的就绪项 (端口 0 本拍不能执行的 fu)，取其中最老的；否则取两边都能走的
  //           就绪项里最年轻的 (与端口 0 并行，不串联)；与端口 0 选中同一项时本拍让出
  // 每类指令都有一个"最老优先"的出口，不会饿死。原来静态绑定时常见一个端口有多条就绪、另一个端口空闲
  // (Dhrystone：访存队列平均 5.2 条就绪，每拍只发 0.96 条)
  logic [SIZE-1:0] gnt [NP];
  logic [SIZE-1:0] req [NP];
  localparam int I1 = (NP > 1) ? 1 : 0;      // 端口数少时死分支里的下标也要合法
  always_comb begin
    for (int p = 0; p < NP; p++)
      for (int i = 0; i < SIZE; i++) req[p][i] = rdy[i] && fu_ok_i[p][ent[i].fu];
    for (int i = 0; i < SIZE; i++) gnt[0][i] = req[0][i] && !(|(req[0] & column(i)));
    if (NP == 2) begin
      logic [SIZE-1:0] ex1, g_old1, g_yng1;
      for (int i = 0; i < SIZE; i++) begin
        ex1[i]    = req[NP-1][i] && !fu_ok_i[0][ent[i].fu];
        g_yng1[i] = req[NP-1][i] && !(|(req[NP-1] & younger(i)));
      end
      for (int i = 0; i < SIZE; i++) g_old1[i] = ex1[i] && !(|(ex1 & column(i)));
      gnt[NP-1] = (|ex1) ? g_old1 : (g_yng1 & ~gnt[0]);
    end else if (NP == 3) begin
      logic [SIZE-1:0] g1a, g1b, g_yng, c;
      for (int i = 0; i < SIZE; i++) begin
        c      = req[I1] & column(i);
        g1a[i] = req[I1][i] && !(|c);
        g1b[i] = req[I1][i] && (|c) && ((c & (c - 1'b1)) == '0);   // 恰有一个更老的 = 第二老
        g_yng[i] = req[NP-1][i] && !(|(req[NP-1] & younger(i)));
      end
      gnt[I1]   = (|(g1a & gnt[0])) ? g1b : g1a;
      gnt[NP-1] = g_yng & ~gnt[0] & ~gnt[I1];
    end
    rtl_g0 = gnt[0]; rtl_g1 = gnt[NP-1];
    if (CAM) begin
      gnt[0] = cam_g0 & req[0];
      if (NP == 2) gnt[NP-1] = (cam_c1 ? '0 : cam_g1) & req[NP-1];
    end
    for (int p = 0; p < NP; p++) begin
      iss_valid_o[p] = |gnt[p];
      iss_uop_o[p]   = '0;
      for (int i = 0; i < SIZE; i++)
        if (gnt[p][i]) begin
          iss_uop_o[p] = ent[i]; iss_uop_o[p].pois = epo[i]; iss_uop_o[p].poia = epa[i]; iss_uop_o[p].poib = epb[i];
        end
      iss_uop_o[p].port = PORT_W'(PBASE + p);
    end
  end

  // 第 i 行：哪些项比 i 年轻
  function automatic logic [SIZE-1:0] younger(input int i);
    logic [SIZE-1:0] r;
    for (int j = 0; j < SIZE; j++) r[j] = older[i][j];
    return r;
  endfunction

  // 第 i 列：哪些项比 i 老
  function automatic logic [SIZE-1:0] column(input int i);
    logic [SIZE-1:0] c;
    for (int j = 0; j < SIZE; j++) c[j] = older[j][i];
    return c;
  endfunction

  logic [SIZE-1:0] issued;
  always_comb begin
    issued = '0;
    for (int p = 0; p < NP; p++) issued |= gnt[p];
  end

  //--------------------------------------------------------------------------
  // 入队位置：最低的 ENQ 个空项。
  // 并行前缀计数：每项数一下比它低的空项有几个，空闲且计数 = k 就是第 k 个入队口的位置。
  // (原写法是 ENQ 个优先编码器串联：找一个、占掉、再找下一个；2nm 综合 32 项 IQ 时这条链 37 级，
  //  是表项更新的最长路径。2026-10-18 改，结果与原写法相同)
  //--------------------------------------------------------------------------
  // 2026-10-26，6 GHz：入队位置上一拍算好寄存 (cand_q / cpos_q)。定制宏在下降沿采样 WE (上升沿后约 63 ps 前要稳)，
  // 只能由寄存器直接驱动 (iqm_cam3：vld -> 前缀计数 -> WE 161 ps)。候选取"保守空闲" = 上拍空闲且上拍没被写入的项：
  // 上拍发射 / 杀掉腾出的项晚一拍才再分配，与给分派的空位数 free_cnt_o (fc_q - ec_q) 口径完全一致，所以不损失容量；
  // 不用 vld_n 是因为那样 cand_q -> wsel -> vld_n -> 前缀 -> cand_q 成环 (iqm_cam4 200 ps)。
  // 前缀计数写成饱和到 ENQ 的温度计码 (t[k] = 计数 >= k) 的 Sklansky 前缀树：每级合并两层与或，不用整数加法器
  logic [ENQ-1:0]  enq_pos_ok;
  logic [SIZE-1:0] enq_onehot [ENQ];
  typedef logic [ENQ:1] th_t;
  function automatic th_t th_add(input th_t a, input th_t b);
    th_t c;
    for (int k = 1; k <= ENQ; k++) begin
      c[k] = a[k] | b[k];
      for (int i = 1; i < k; i++) c[k] = c[k] | (a[i] & b[k - i]);
    end
    return c;
  endfunction
  // 最低的 ENQ 个空项 (独热，第 k 行 = 第 k 个入队口的位置)
  function automatic logic [ENQ-1:0][SIZE-1:0] first_free(input logic [SIZE-1:0] fr);
    th_t p [$clog2(SIZE)+1][SIZE];
    logic [ENQ-1:0][SIZE-1:0] oh;
    for (int i = 0; i < SIZE; i++) begin p[0][i] = '0; p[0][i][1] = fr[i]; end
    for (int d = 0; d < $clog2(SIZE); d++)
      for (int i = 0; i < SIZE; i++)
        if ((i >> d) & 1) p[d+1][i] = th_add(p[d][((i >> d) << d) - 1], p[d][i]);
        else              p[d+1][i] = p[d][i];
    for (int i = 0; i < SIZE; i++) begin
      th_t e;                                   // 比 i 低的空项数 (饱和)
      if (i == 0) e = '0;
      else        e = p[$clog2(SIZE)][i - 1];
      oh[0][i] = fr[i] && !e[1];
      for (int k = 1; k < ENQ; k++) oh[k][i] = fr[i] && e[k] && !e[k + 1];
    end
    return oh;
  endfunction
  logic [ENQ-1:0][SIZE-1:0] cand_q;
  logic [ENQ-1:0]           cpos_q;
  always_comb
    for (int k = 0; k < ENQ; k++) begin enq_onehot[k] = cand_q[k]; enq_pos_ok[k] = cpos_q[k]; end

  // 空位数寄存：上拍的空位数 - 上拍入队数 (保守：上拍发射 / 杀掉腾出的空位晚一拍才算)。
  // 原来是本拍 vld 的组合计数，串进分派就绪 -> ROB/IQ/LSQ 写使能
  logic [$clog2(SIZE+1)-1:0] fc_q;
  logic [2:0]                ec_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      fc_q <= SIZE; ec_q <= '0;
    end else begin
      logic [$clog2(SIZE+1)-1:0] c;
      logic [2:0] e;
      c = '0; e = '0;
      for (int i = 0; i < SIZE; i++) c += !vld[i];
      for (int k = 0; k < ENQ; k++) e += enq_valid_i[k];
      fc_q <= flush_i ? $clog2(SIZE+1)'(SIZE) : c;
      ec_q <= flush_i ? 3'd0 : e;
    end
  end
  assign free_cnt_o = (fc_q > ec_q) ? fc_q - ec_q : '0;

  //--------------------------------------------------------------------------
  // 状态更新
  //--------------------------------------------------------------------------
  // 每项本拍是否被某个入队口写 (按项展开，避免"按动态下标写数组"——
  // 那种写法 DC 2018 elaborate 极慢，综合出的也是每项一条优先级链)
  logic [ENQ-1:0]  wsel [SIZE];
  logic [SIZE-1:0] wany;
  logic [SIZE-1:0] enq_ok;
  always_comb begin
    for (int i = 0; i < SIZE; i++) begin
      for (int k = 0; k < ENQ; k++)
        wsel[i][k] = enq_valid_i[k] && enq_pos_ok[k] && enq_onehot[k][i];
      wany[i]   = |wsel[i];
      enq_ok[i] = 1'b0;
      for (int k = 0; k < ENQ; k++)
        if (wsel[i][k] && !br_killed(enq_uop_i[k].br_mask, br_upd_i)) enq_ok[i] = 1'b1;
    end
  end

  logic [SIZE-1:0] hd_d, hd2_d;
  always_comb
    for (int i = 0; i < SIZE; i++) begin
      hd_d[i]  = vld[i] && !wany[i] && ent[i].rob_idx == rob_head_i;
      hd2_d[i] = vld[i] && !wany[i] && ent[i].rob_idx == head2_i;
    end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin hd_q <= '0; hd2_q <= '0; end
    else begin hd_q <= hd_d; hd2_q <= hd2_d; end
  end

  // 本拍发射的项里有源带 spa / spb 的 (判定在下一拍 / 两拍后)
  logic [SIZE-1:0] iss_pa, iss_pb, isw1;
  always_comb
    for (int i = 0; i < SIZE; i++) begin
      iss_pb[i] = issued[i] && (|epb[i]);
      iss_pa[i] = issued[i] && (|epa[i]) && !(|epb[i]);
      isw1[i]   = iswc[i] == 2'd1;
    end
  // 下一状态 (组合)：就绪位也由它预算寄存 (rdy_q，见下)
  logic [SIZE-1:0] vld_n;
  logic [1:0]      iswc_n [SIZE];
  always_comb begin
    // 释放：本拍发射且不等判定、本拍判定没失败；或最后一拍等判定 (iswc = 1) 本拍判定没失败
    // 判定失败的项留下 (源已重新置 busy)，等真正的唤醒再发
    vld_n = flush_i ? '0 : (vld & ~killed & ~((issued & ~iss_pa & ~iss_pb) & ~fhit) & ~(isw1 & ~fhit)) | enq_ok;
    for (int i = 0; i < SIZE; i++)
      iswc_n[i] = (flush_i || killed[i] || fhit[i]) ? 2'd0 : iss_pb[i] ? 2'd2 : iss_pa[i] ? 2'd1 :
                  (iswc[i] == 2'd2) ? 2'd1 : 2'd0;
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vld <= '0;
      for (int i = 0; i < SIZE; i++) iswc[i] <= '0;
    end else begin
      vld <= vld_n;
      for (int i = 0; i < SIZE; i++) iswc[i] <= iswc_n[i];
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin
      cand_q <= first_free('1);
      for (int k = 0; k < ENQ; k++) cpos_q[k] <= (k < SIZE);
    end else begin
      logic [ENQ-1:0][SIZE-1:0] c;
      c = flush_i ? first_free('1) : first_free(~vld & ~wany);
      cand_q <= c;
      for (int k = 0; k < ENQ; k++) cpos_q[k] <= |c[k];
    end
`ifndef SYNTHESIS
  // 候选位置：每口至多一项、指向空闲项、各口互不相同；有效候选数不少于给分派的空位数 (封顶 ENQ)
  always_ff @(posedge clk_i) if (rst_ni) begin
    int nc;
    logic [SIZE-1:0] u;
    nc = 0; u = '0;
    for (int k = 0; k < ENQ; k++) begin
      assert ($onehot0(cand_q[k])) else $error("iq: 入队候选口 %0d 不是独热 %h", k, cand_q[k]);
      assert ((cand_q[k] & vld) == '0) else $error("iq: 入队候选口 %0d 指向有效项", k);
      assert ((cand_q[k] & u) == '0) else $error("iq: 入队候选口 %0d 与前面的口重复", k);
      assert (cpos_q[k] == |cand_q[k]) else $error("iq: cpos_q 与 cand_q 不符 口 %0d", k);
      u = u | cand_q[k];
      nc += int'(cpos_q[k]);
    end
    assert (nc >= ((int'(free_cnt_o) < ENQ) ? int'(free_cnt_o) : ENQ))
      else $error("iq: 入队候选 %0d 个少于给分派的空位数 %0d", nc, free_cnt_o);
  end
`endif

  // 应用唤醒 / load 判定 / 分支更新之后的项 (用于下一拍状态)；sp / sa = 各源本拍的两级毒化，npo / npa = 下一拍的
  // 三个源本拍的唤醒判定 (RTL 写法：逐口比较)
  function automatic logic [2:0][WHW-1:0] whit3(input uop_t u, input wakeup_t [NIW-1:0] iw, input wakeup_t [N_FWAKE-1:0] fw,
                                                input lwake_t [N_LDP-1:0] lw, input logic [N_LDP-1:0] fail);
    logic [2:0][WHW-1:0] w;
    lwake_t [N_LDP-1:0] z;
    z = '0;
    // 快速唤醒只用于基址 (rs1)
    w[0] = whit(u.psrc1, u.rs1_class == RC_FP, iw, fw, lw, fail);
    w[1] = whit(u.psrc2, u.rs2_class == RC_FP, iw, fw, z, fail);
    w[2] = whit(u.psrc3, 1'b1,                 iw, fw, z, fail);
    return w;
  endfunction
  function automatic uop_t upd(input uop_t u, input logic [2:0][N_LDP-1:0] sp, input logic [2:0][N_LDP-1:0] sa,
                               input logic [2:0][N_LDP-1:0] sb,
                               input logic [2:0][WHW-1:0] wh,
                               input logic [N_LDP-1:0] fail, input br_update_t bu,
                               output logic [2:0][N_LDP-1:0] npo, output logic [2:0][N_LDP-1:0] npa,
                               output logic [2:0][N_LDP-1:0] npb);
    uop_t r;
    logic [SNW-1:0] a, b, c;
    r = u;
    a = snext(u.src1_busy, sp[0], sa[0], sb[0], wh[0], fail);
    b = snext(u.src2_busy, sp[1], sa[1], sb[1], wh[1], fail);
    c = snext(u.src3_busy, sp[2], sa[2], sb[2], wh[2], fail);
    r.src1_busy = a[SNW-1]; npo[0] = a[3*N_LDP-1:2*N_LDP]; npa[0] = a[2*N_LDP-1:N_LDP]; npb[0] = a[N_LDP-1:0];
    r.src2_busy = b[SNW-1]; npo[1] = b[3*N_LDP-1:2*N_LDP]; npa[1] = b[2*N_LDP-1:N_LDP]; npb[1] = b[N_LDP-1:0];
    r.src3_busy = c[SNW-1]; npo[2] = c[3*N_LDP-1:2*N_LDP]; npa[2] = c[2*N_LDP-1:N_LDP]; npb[2] = c[N_LDP-1:0];
    r.br_mask = u.br_mask & ~bu.resolve_mask;
    r.pois    = '0;
    r.poia    = '0;
    r.poib    = '0;
    return r;
  endfunction

  // 入队数据：先把各入队口的 uop 统一更新一次，再按项选 (入队口另外听 load 的确认唤醒)
  wakeup_t [NIW-1:0] iw_ent, iw_enq;        // 表项用 / 入队口用的整数唤醒
  always_comb begin
    iw_ent = '0; iw_enq = '0;
    for (int k = 0; k < N_IWAKE; k++) begin iw_ent[k] = iwake_i[k]; iw_enq[k] = iwake_i[k]; end
    for (int k = 0; k < N_LDP; k++) iw_enq[N_IWAKE+k] = ewake_i[k];
  end
  // 表项本拍的唤醒判定：USE_CAM 时取 WKCAM 的命中 / 载荷 (见文件末尾)，否则逐口比较
  logic [2:0][WHW-1:0] ewh [SIZE], ewh_rtl [SIZE], ewh_cam [SIZE];
  always_comb for (int i = 0; i < SIZE; i++) ewh_rtl[i] = whit3(ent[i], iw_ent, fwake_i, lwake_i, ld_fail_i);
  always_comb for (int i = 0; i < SIZE; i++) ewh[i] = CAM ? ewh_cam[i] : ewh_rtl[i];
  uop_t             enq_upd [ENQ];
  logic [2:0][N_LDP-1:0] enq_po [ENQ], enq_pa [ENQ], enq_pb [ENQ];
  always_comb begin
    for (int k = 0; k < ENQ; k++) begin
      logic [2:0][N_LDP-1:0] z, po, pa, pb;
      uop_t u;
      z = '0;
      u = enq_uop_i[k];
      if (chit(u.psrc1, u.rs1_class == RC_FP, cwake_i)) u.src1_busy = 1'b0;
      if (chit(u.psrc2, u.rs2_class == RC_FP, cwake_i)) u.src2_busy = 1'b0;
      if (chit(u.psrc3, 1'b1,                 cwake_i)) u.src3_busy = 1'b0;
      enq_upd[k] = upd(u, z, z, z, whit3(u, iw_enq, fwake_i, lwake_i, ld_fail_i), ld_fail_i, br_upd_i, po, pa, pb);
      enq_po[k] = po; enq_pa[k] = pa; enq_pb[k] = pb;
    end
  end

  uop_t ent_n [SIZE];
  logic [2:0][N_LDP-1:0] spo_n [SIZE], spa_n [SIZE], spb_n [SIZE];
  always_comb begin
    for (int i = 0; i < SIZE; i++) begin
      uop_t n;
      logic [2:0][N_LDP-1:0] po, pa, pb;
      n = upd(ent_e[i], spo_e[i], spa_e[i], spb_e[i], ewh[i], ld_fail_i, br_upd_i, po, pa, pb);
      for (int k = 0; k < ENQ; k++)
        if (wsel[i][k]) begin n = enq_upd[k]; po = enq_po[k]; pa = enq_pa[k]; pb = enq_pb[k]; end
      ent_n[i] = n; spo_n[i] = po; spa_n[i] = pa; spb_n[i] = pb;
    end
  end
  always_ff @(posedge clk_i)
    for (int i = 0; i < SIZE; i++) begin
      ent[i] <= ent_n[i];
      spo[i] <= spo_n[i];
      spa[i] <= spa_n[i];
      spb[i] <= spb_n[i];
    end

  // 就绪位预算寄存 (2026-10-25，6 GHz)：AGESEL 要求请求沿后 <= 20 ps 到达，选择前不能再有
  // "有效 & 未等判定 & 三源都不忙 & 等头部" 的与门。用下一状态预算：rdy_q = 非 wait_head 项就绪，
  // rdw_q = wait_head 项除头部条件外就绪 (头部条件 head_go / hd_q 本拍再与)。与组合写法逐拍一致 (断言)
  logic [SIZE-1:0] rdy_d, rdw_d;
  always_comb
    for (int i = 0; i < SIZE; i++) begin
      logic b;
      b = vld_n[i] && iswc_n[i] == 2'd0 && !ent_n[i].src1_busy && !ent_n[i].src2_busy && !ent_n[i].src3_busy;
      rdy_d[i] = b && !ent_n[i].wait_head;
      rdw_d[i] = b &&  ent_n[i].wait_head;
    end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin rdy_q <= '0; rdw_q <= '0; end
    else begin rdy_q <= rdy_d; rdw_q <= rdw_d; end

  // 年龄矩阵：older[i][j] 的下一值按 (i, j) 是否本拍入队、入队口先后来定
  //   i、j 都不入队：保持
  //   只有 i 入队  ：0 (新项最年轻)
  //   只有 j 入队  ：1
  //   都入队       ：i 的入队口号 < j 的入队口号
  logic [$clog2(ENQ)-1:0] wport [SIZE];
  always_comb begin
    for (int i = 0; i < SIZE; i++) begin
      wport[i] = '0;
      for (int k = ENQ-1; k >= 0; k--) if (wsel[i][k]) wport[i] = k[$clog2(ENQ)-1:0];
    end
  end

  always_ff @(posedge clk_i) begin
    for (int i = 0; i < SIZE; i++)
      for (int j = 0; j < SIZE; j++) begin
        if (i == j)                     older[i][j] <= 1'b0;
        else if (wany[i] && wany[j])    older[i][j] <= (wport[i] < wport[j]);
        else if (wany[i])               older[i][j] <= 1'b0;
        else if (wany[j])               older[i][j] <= 1'b1;
      end
  end

  //--------------------------------------------------------------------------
  // 定制宏 (USE_CAM)：WKCAM2_32 唤醒、AGESEL_32_2 选择
  //--------------------------------------------------------------------------
  if (CAM) begin : g_cam
    // 口：0..5 ALU (iwake[0..5]，载荷 sn / sc；乘法唤醒并在 2 / 3 里，载荷为 0)、6..7 MUL (不接)、8 W6 (iwake[6])、
    //     9..11 L0 (iwake[7..9]，载荷 sa / sc)、12..14 LL (lwake，载荷 sb / sc)、15..20 浮点 (fwake)
    logic [15:0] bh [21], bl [21];
    logic [11:0] bp [21];
    logic [31:0] cwe [4];
    logic [7:0]  cet [4][3];
    logic [1:0]  cef [4];
    logic [2:0]  ch [32];
    logic [11:0] cy [32][2];
    function automatic logic [15:0] oh16(input logic [3:0] x); return 16'(1) << x; endfunction
    always_comb begin
      for (int p = 0; p < 21; p++) begin bh[p] = '0; bl[p] = '0; bp[p] = '0; end
      for (int k = 0; k < 6; k++)
        if (iwake_i[k].valid) begin
          bh[k] = oh16(iwake_i[k].pdst[7:4]); bl[k] = oh16(iwake_i[k].pdst[3:0]);
          bp[k] = {iwake_i[k].sc, 3'b0, 3'b0, iwake_i[k].sn};
        end
      if (iwake_i[6].valid) begin bh[8] = oh16(iwake_i[6].pdst[7:4]); bl[8] = oh16(iwake_i[6].pdst[3:0]); end
      for (int k = 0; k < 3; k++)
        if (iwake_i[7+k].valid) begin
          bh[9+k] = oh16(iwake_i[7+k].pdst[7:4]); bl[9+k] = oh16(iwake_i[7+k].pdst[3:0]);
          bp[9+k] = {iwake_i[7+k].sc, 3'b0, iwake_i[7+k].sa, 3'b0};
        end
      for (int k = 0; k < 3; k++)
        if (lwake_i[k].valid) begin
          bh[12+k] = oh16(lwake_i[k].pdst[7:4]); bl[12+k] = oh16(lwake_i[k].pdst[3:0]);
          bp[12+k] = {lwake_i[k].sc, lwake_i[k].sb, 3'b0, 3'b0};
        end
      for (int k = 0; k < 6; k++)
        if (fwake_i[k].valid) begin bh[15+k] = oh16(fwake_i[k].pdst[7:4]); bl[15+k] = oh16(fwake_i[k].pdst[3:0]); end
      // 入队写口 = 各口的候选空位 (第 k 个空项)，不看 enq_valid：宏在下降沿采样 WE / 写数据 (建立 10 ps，
      // 约上升沿后 73 ps 前要稳)，入队有效来自分派级很深的逻辑赶不上 (iqm_cam2：vld -> wsel -> WE 197 ps)。
      // 没入队的候选项照写也无害：它仍无效、不发请求，命中不用；年龄行在它真正入队那拍会重写为最年轻，
      // 有效项之间的先后不变 (下面的授权断言逐拍核对)。被杀的也写，同理
      for (int k = 0; k < 4; k++) begin
        for (int i = 0; i < 32; i++) cwe[k][i] = enq_onehot[k][i];
        cet[k][0] = enq_uop_i[k].psrc1; cet[k][1] = enq_uop_i[k].psrc2; cet[k][2] = enq_uop_i[k].psrc3;
        cef[k] = {enq_uop_i[k].rs2_class == RC_FP, enq_uop_i[k].rs1_class == RC_FP};
      end
    end
    o6_wkcam32 u_wk (.clk_i, .bh_i(bh), .bl_i(bl), .bp_i(bp), .we_i(cwe), .et_i(cet), .ef_i(cef), .h_o(ch), .y_o(cy));
    // 命中 + 载荷 -> 与 whit 同格式 {置就绪, sn, sa, sb}：一个源每拍至多一个口命中，置就绪 = 命中且载荷 sc 对应流水未失败
    logic [11:0] cyy [32][3];
    always_comb
      for (int i = 0; i < 32; i++)
        for (int sr = 0; sr < 3; sr++) begin
          cyy[i][sr]     = (sr < 2) ? cy[i][(sr < 2) ? sr : 0] : '0;
          ewh_cam[i][sr] = {ch[i][sr], 3'b0, 3'b0, 3'b0};          // 命中只清 busy，载荷下一拍并入
        end
    // 这次唤醒被采纳 (源本来 busy、本拍没有毒化失败、没被入队覆盖) 时记下载荷
    always_ff @(posedge clk_i or negedge rst_ni)
      if (!rst_ni) begin
        dq_f <= '0;
        for (int i = 0; i < 32; i++) dq_v[i] <= '0;
      end else begin
        dq_f <= ld_fail_i;
        for (int i = 0; i < 32; i++)
          for (int sr = 0; sr < 3; sr++) begin
            logic busy;
            busy = (sr == 0) ? ent_e[i].src1_busy : (sr == 1) ? ent_e[i].src2_busy : ent_e[i].src3_busy;
            dq_v[i][sr] <= busy && ch[i][sr] && !(|(spo_e[i][sr] & ld_fail_i)) && !wany[i];
          end
      end
    always_ff @(posedge clk_i)
      for (int i = 0; i < 32; i++) dq_y[i] <= {cyy[i][2], cyy[i][1], cyy[i][0]};
    // 选择 (2026-10-26 与 cam 会话定：AGESEL 的 R0 / R1 / EX 在宏内上升沿锁存，宏外只需普通 setup_rising)
    // 送的是下一拍的请求：就绪位用下一状态 (rdy_d / rdw_d / hd_d，含本拍唤醒，精确)，fu_ok 与 head_go
    // 用本拍值预测下一拍 —— 下一拍授权出来后与当拍真实 req 相与 (上面 gnt)，预测错只损失那一拍的发射
    logic [31:0] r0, r1, ex;
    logic [7:0]  api [4];
    logic [7:0]  q0, q1;
    always_comb begin
      for (int i = 0; i < 32; i++) begin
        logic rp;
        rp    = rdy_d[i] || (rdw_d[i] && head_go_i && (hd_d[i] || (hd2_d[i] && head2_go_i && ent_n[i].fu == FU_VEC)));
        r0[i] = rp && fu_ok_nx_i[0][ent_n[i].fu];
        r1[i] = (NP == 2) && rp && fu_ok_nx_i[NP-1][ent_n[i].fu];
        ex[i] = (NP == 2) && !fu_ok_nx_i[0][ent_n[i].fu];
      end
      for (int k = 0; k < 4; k++) api[k] = enq_uop_i[k].pdst;
    end
    o6_agesel32 u_ag (.clk_i, .we_i(cwe), .pi_i(api), .r0_i(r0), .r1_i(r1), .ex_i(ex),
                      .g0_o(cam_g0), .g1_o(cam_g1), .c1_o(cam_c1), .q0_o(q0), .q1_o(q1));
`ifndef SYNTHESIS
    // 参考：锁存的预测请求 / fu_ok / head_go，与 RTL 同写法的年龄选择，再与当拍真实请求相与
    logic [31:0] rq0, rq1, rex, pg0, pg1, fok0, fok1;
    logic        hgo_q, hgo2_q;
    int          n_mispred = 0;
    int          n_dqc = 0;              // 有项因上拍唤醒作废被屏蔽的拍数
    always_ff @(posedge clk_i) begin
      rq0 <= r0; rq1 <= r1; rex <= ex; hgo_q <= head_go_i; hgo2_q <= head2_go_i;
      for (int i = 0; i < 32; i++) begin fok0[i] <= fu_ok_nx_i[0][ent_n[i].fu]; fok1[i] <= fu_ok_nx_i[NP-1][ent_n[i].fu]; end
    end
    always_comb begin
      logic [31:0] g0, g1, ex1, go1, gy1;
      for (int i = 0; i < 32; i++) g0[i] = rq0[i] && !(|(rq0 & column(i)));
      g1 = '0;
      if (NP == 2) begin
        for (int i = 0; i < 32; i++) begin
          ex1[i] = rq1[i] && rex[i];
          gy1[i] = rq1[i] && !(|(rq1 & younger(i)));
        end
        for (int i = 0; i < 32; i++) go1[i] = ex1[i] && !(|(ex1 & column(i)));
        g1 = (|ex1) ? go1 : (gy1 & ~g0);
      end
      pg0 = g0 & req[0];
      pg1 = g1 & req[NP-1];
    end
    final if (n_mispred != 0) $display("[perf] iq PBASE=%0d：AGESEL 请求预测错 %0d 拍，其中有唤醒作废屏蔽的 %0d 拍", PBASE, n_mispred, n_dqc);
    initial assert (N_IWAKE == 10 && N_FWAKE == 6 && N_LDP == 3 && ENQ == 4)
      else $error("iq: USE_CAM 要求 N_IWAKE = 10、N_FWAKE = 6、N_LDP = 3、ENQ = 4");
    // 原写法的宏唤醒 (t 拍就带载荷、按 sc & fail 作废)
    logic [2:0][WHW-1:0] ewh_full [32];
    always_comb
      for (int i = 0; i < 32; i++)
        for (int sr = 0; sr < 3; sr++) begin
          logic [11:0] y;
          logic        st;
          y  = cyy[i][sr];
          st = ch[i][sr] && !(|(y[11:9] & ld_fail_i));
          ewh_full[i][sr] = {st, st ? y[2:0] : 3'b0, st ? y[5:3] : 3'b0, st ? y[8:6] : 3'b0};
        end
    // 影子状态：按原写法 (ewh_full，t 拍并入) 演进，与有效状态逐拍比
    logic [2:0]            sh_b   [32];
    logic [2:0][N_LDP-1:0] sh_po  [32], sh_pa [32], sh_pb [32];
    logic                  sh_ok = 1'b0;
    always_ff @(posedge clk_i) begin
      sh_ok <= rst_ni;
      for (int i = 0; i < 32; i++) begin
        uop_t n, u;
        logic [2:0][N_LDP-1:0] po, pa, pb;
        u = ent[i];
        u.src1_busy = sh_b[i][0]; u.src2_busy = sh_b[i][1]; u.src3_busy = sh_b[i][2];
        n = upd(u, sh_po[i], sh_pa[i], sh_pb[i], ewh_full[i], ld_fail_i, br_upd_i, po, pa, pb);
        for (int k = 0; k < ENQ; k++)
          if (wsel[i][k]) begin n = enq_upd[k]; po = enq_po[k]; pa = enq_pa[k]; pb = enq_pb[k]; end
        sh_b[i]  <= {n.src3_busy, n.src2_busy, n.src1_busy};
        sh_po[i] <= po; sh_pa[i] <= pa; sh_pb[i] <= pb;
      end
    end
    always_ff @(posedge clk_i) if (rst_ni) begin
      for (int i = 0; i < 32; i++)
        if (vld[i]) begin
          for (int sr = 0; sr < 3; sr++) begin
            logic busy;
            busy = (sr == 0) ? ent_e[i].src1_busy : (sr == 1) ? ent_e[i].src2_busy : ent_e[i].src3_busy;
            if (busy) assert (ewh_full[i][sr] == ewh_rtl[i][sr])
              else $error("iq: WKCAM 唤醒与逐口比较不一致 项 %0d 源 %0d cam=%h rtl=%h", i, sr, ewh_full[i][sr], ewh_rtl[i][sr]);
          end
          if (sh_ok)
            assert ({ent_e[i].src3_busy, ent_e[i].src2_busy, ent_e[i].src1_busy} == sh_b[i] &&
                    spo_e[i] == sh_po[i] && spa_e[i] == sh_pa[i] && spb_e[i] == sh_pb[i])
              else $error("iq: 载荷推迟并入后的有效状态与原写法不一致 项 %0d busy=%b/%b", i,
                          {ent_e[i].src3_busy, ent_e[i].src2_busy, ent_e[i].src1_busy}, sh_b[i]);
        end
      assert (gnt[0] == pg0) else $error("iq: AGESEL 口 0 与年龄矩阵 (预测请求) 选择不一致");
      if (NP == 2) assert (gnt[NP-1] == pg1) else $error("iq: AGESEL 口 1 与年龄矩阵 (预测请求) 选择不一致");
      if (|gnt[0]) assert (q0 == iss_uop_o[0].pdst) else $error("iq: AGESEL pdst 口 0 不符");
      // 预测请求本身：就绪部分精确，只有 fu_ok / head_go 会错
      for (int i = 0; i < 32; i++)
        if (rq0[i] != req[0][i] || (NP == 2 && rq1[i] != req[NP-1][i]))
          assert (fu_ok_i[0][ent[i].fu] != fok0[i] || (NP == 2 && fu_ok_i[NP-1][ent[i].fu] != fok1[i]) ||
                  (rdw_q[i] && (head_go_i != hgo_q || head2_go_i != hgo2_q)) || dqc_any[i])
            else $error("iq: 预测请求错了但 fu_ok / head_go 没变、也不是上拍唤醒作废 项 %0d", i);
      if (rq0 != req[0] || (NP == 2 && rq1 != req[NP-1])) n_mispred++;
      if (|dqc_any) n_dqc++;
    end
`endif
  end else begin : g_nocam
    assign cam_g0 = '0;
    assign cam_g1 = '0;
    assign cam_c1 = 1'b0;
    always_comb for (int i = 0; i < SIZE; i++) ewh_cam[i] = '0;
    always_comb begin
      dq_f = '0;
      for (int i = 0; i < SIZE; i++) begin dq_v[i] = '0; dq_y[i] = '0; end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni)
    for (int i = 0; i < SIZE; i++) begin
      uop_t u;
      logic rc;
      u  = ent_e[i];
      rc = vld[i] && !isw[i] && !u.src1_busy && !u.src2_busy && !u.src3_busy &&
           (!u.wait_head || (head_go_i && (hd_q[i] || (hd2_q[i] && head2_go_i && u.fu == FU_VEC))));
      assert (rc == rdy[i]) else $error("iq: 预算寄存的就绪位与组合写法不一致 项 %0d", i);
    end
  always_ff @(posedge clk_i) if (rst_ni) begin
    for (int k = 0; k < ENQ; k++)
      assert (!enq_valid_i[k] || enq_pos_ok[k]) else $error("iq: 入队时无空项 (分派级未检查容量)");
    // 各端口选中的项互不相同，且都是本端口能执行的就绪项
    for (int p = 0; p < NP; p++) begin
      assert ($countones(gnt[p]) <= 1) else $error("iq: 端口 %0d 选中多项", p);
      assert ((gnt[p] & ~req[p]) == '0) else $error("iq: 端口 %0d 选中了不能执行 / 未就绪的项", p);
      for (int q = p + 1; q < NP; q++)
        assert ((gnt[p] & gnt[q]) == '0) else $error("iq: 端口 %0d 与 %0d 选中同一项", p, q);
    end
  end
`endif
endmodule
