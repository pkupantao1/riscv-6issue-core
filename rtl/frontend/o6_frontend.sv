//==============================================================================
// o6_frontend.sv - M2 前端：F0 (uBTB 预测) / F1 (I$ + 方向表) / F2 (并行预译码) / F3 (最终预测)
//
// 与 M1 前端接口相同 (取指缓冲、FTQ 读口、重定向、MMU)，另加分支解析 (bres) 输入用于训练。
//
// F0：fetch PC 寄存器。uBTB (32 项全相联，按块起始 PC) 给出"本块第一个预测跳转的槽与目标"，
//     下一拍 PC = 命中 ? 目标 : 顺序下一块。同拍：ITLB / PMP，向 I$ 发请求，读方向表一行。
// F1：等 I$ 响应；把数据和方向表那一行打进 F2。
// F2：预译码，只做彼此独立、与预测无关的部分：指令起点 (4 级并行前缀)、压缩成 NI 个槽、
//     每个位置的控制流类型 (按原始编码识别) 与"直接目标 == F0 预测的下一块"(无进位比较)。
// F3：最终预测：JAL 必跳；条件分支看方向计数器；返回用 RAS；其它 JALR 用 uBTB 目标 (没有就预测不跳)。
//     与 F0 的 uBTB 推测 (是否跳、目标) 比较：不一致 -> 冲掉 F2/F1、F0 改从最终目标取 (3 拍气泡)，
//     并训练 uBTB。RVC 展开、截断到第一个预测跳转处，写取指缓冲与 FTQ。
//     (500 MHz 下原来的 F2 一拍里做完预译码 + 目标加法 + 比较 + 选下一 PC 是 155 级逻辑，故拆两级)
// 训练：方向表在 FTQ 项被提交释放时按记录的解析结果更新 (非推测)；
//       uBTB 在 F3 发现预测不一致时、以及后端误预测重定向时更新。
//
// 方向预测 = bimodal (每块一行 16 个 2 位计数器，触发器) + TAGE v2 (o6_tage：8 张带标签表 x 1024 行 x 2 路，SRAM)：
//   全局历史 = 跳转路径历史 gh0：每次 (预测的) 跳转推入 2 位 (槽号 ^ 目标 的散列)，不跳不推；
//     折叠寄存器 fold0 随推入增量更新；各级 / FTQ 带"本块预测前"的折叠快照与推入计数，
//     恢复时折叠取快照、原始历史按推入次数之差右移回去 (余量 BP_GHL - BP_HMAX 覆盖所有在途推入)；
//   F0 用 (块 PC, fold0) 查表 -> F1 读出 -> F2 逐槽选提供者 / 替补 -> F3 只取最终方向；
//   训练在 FTQ 项释放时：块内前两个条件分支 (预译码给出，已解析的才训)，
//     提供者计数器 / 有用位更新、误预测时在更长的表里分配 (至多 2 张)、分配失败衰减有用位。
//==============================================================================
`default_nettype none
module o6_frontend
  import o6_pkg::*;
  import o6_isa_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // I$
  output logic                imem_req_o,
  input  wire                 imem_ready_i,
  output logic [63:0]         imem_addr_o,
  input  wire                 imem_resp_valid_i,
  input  wire  [255:0]        imem_data_i,
  output logic                imem_kill_o,

  // 重定向
  input  wire                 be_redirect_i,
  input  wire  [63:0]         be_redirect_pc_i,
  input  wire  [FTQ_W-1:0]    be_redirect_ftq_i,
  input  wire                 be_redirect_call_i,   // 重定向的那条是调用 / 返回：恢复 RAS 后补做它的动作
  input  wire                 be_redirect_ret_i,
  input  wire  [63:0]         be_redirect_link_i,
  input  wire                 cm_flush_i,
  input  wire  [63:0]         cm_redirect_pc_i,
  input  wire  [FTQ_W-1:0]    cm_flush_ftq_i,       // 引起提交级冲刷的指令所在 FTQ 项 (按它恢复 RAS)
  // 访存相关预测训练：违例 load 的位置
  input  wire                 mdp_set_i,
  input  wire  [FTQ_W-1:0]    mdp_ftq_i,
  input  wire  [FTQ_OFF_W-1:0] mdp_off_i,

  // 提交释放 FTQ
  input  wire                 cm_ftq_v_i,
  input  wire  [FTQ_W-1:0]    cm_ftq_idx_i,

  // 分支解析 (训练用)
  input  wire  [N_BRU-1:0]    bres_valid_i,
  input  wire  [FTQ_W-1:0]    bres_ftq_i [N_BRU],
  input  wire  [FTQ_OFF_W-1:0] bres_off_i [N_BRU],
  input  wire  [N_BRU-1:0]    bres_taken_i,
  input  wire  [N_BRU-1:0]    bres_mispred_i,
  input  wire  [63:0]         bres_target_i [N_BRU],
  input  wire  [N_BRU-1:0]    bres_ind_i,          // 该分支是非返回的 JALR (ITTAGE 训练用实际目标)

  // FTQ 读口
  input  wire  [FTQ_W-1:0]    ftq_ridx_i [N_FTQ_RD],
  output logic [63:0]         ftq_pc_o [N_FTQ_RD],
  output logic [63:0]         ftq_tgt_o [N_FTQ_RD],

  // 去译码
  output logic [W-1:0]        ib_valid_o,
  output fetch_inst_t [W-1:0] ib_inst_o,
  input  wire  [2:0]          ib_pop_i,

  // MMU
  input  wire                 xlate_i,
  input  wire  [1:0]          priv_i,
  input  wire                 dbg_mode_i,       // 调试模式：允许从调试模块 (0x000-0xFFF) 取指
  input  wire                 tlb_flush_i,
  input  pmp_dec_t            pmpd_i    [8],      // PMP 预译码 (o6_core 寄存)
  output logic                ptw_ireq_o,
  output logic [VPN_W-1:0]    ptw_ivpn_o,
  output logic                ptw_iv_o,         // H：请求的 V (= 取指的 V)
  input  wire                 ifill_i,
  input  wire                 fill_v_i,
  input  wire  [VPN_W-1:0]    fill_vpn_i,
  input  wire  [2:0]          fill_level_i,
  input  wire                 fill_nap_i,
  input  wire  [1:0]          vam_i,            // 0 = Sv39，1 = Sv48，2 = Sv57 (规范地址检查)
  input  wire                 xv_i,
  input  wire                 cn_i,             // H：一级 (satp / vsatp) 翻译开 (vsatp = Bare 只有 G-stage 时为 0)
  output logic [63:0]         fgx_o,            // H：取指 guest page fault 的 {隐式, 6'b0, GPA[58:2]} (F3 交出异常时锁存)             // H：取指的 V (ITLB 项标记)
  input  wire  [PA_W-13:0]    fill_ppn_i,
  input  wire  [7:0]          fill_perm_i,
  input  wire  [1:0]          fill_fault_i,
  input  wire  [3:0]          fill_gperm_i,
  input  wire  [56:0]         fill_gpa_i,
  input  wire                 fill_gimp_i,

  // MBIST (TAGE 的 SRAM)
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int NI    = 17;
`ifdef O6_NUB
  localparam int NUB   = `O6_NUB;     // 性能实验用 (组数固定 32，路数 = NUB / 32)
`else
  localparam int NUB   = 128;         // uBTB 项数 (32 组 x 4 路；Linux 负载上气泡 162 -> 99 /千条，见 docs/08)
`endif
  localparam int UBW   = NUB / 32;    // uBTB 路数
  localparam int UBS   = NUB / UBW;   // uBTB 组数
  localparam int UWW   = $clog2(UBW); // 路号位宽
`ifdef O6_NBTB
  localparam int NBTB  = `O6_NBTB;    // 性能实验用
`else
  localparam int NBTB  = 256;         // F1 BTB 项数 (直接映射，按块起始 PC)
`endif
  localparam int BTW   = $clog2(NBTB);
  localparam int BIM_R = 256;         // 方向表行数 (每行 16 个 2 位计数器 = 一个 32 B 块)
  // uBTB 的方向：带全局历史的计数器表 uPHT (每个 uBTB 槽位一张，2 x NUP x 2 位，触发器)，
  // 用 (块 PC, 最近的路径历史 gh0) 散列索引。原来 uBTB 每槽一个 2 位计数器 (相当于 bimodal)，
  // CoreMark 上 F3 覆盖 4.7 万次里 4.4 万次是这个计数器与 TAGE/SC 的方向分歧 (2026-10-14 f0 统计)
`ifdef O6_NUP
  localparam int NUP   = `O6_NUP;
`else
  localparam int NUP   = 1024;
`endif
  localparam int UPW   = $clog2(NUP);

  function automatic logic [63:0] blk_base(input logic [63:0] pc);
    return {pc[63:5], 5'b0};
  endfunction
  function automatic logic exec_ok(input logic [63:0] a);
    return (a >= DRAM_BASE && a < DRAM_BASE + 64'h8000_0000) ||
           (a >= BOOTROM_BASE && a < BOOTROM_BASE + 64'h1_0000) ||
           (dbg_mode_i && a < 64'h1000);
  endfunction
  function automatic logic [7:0] bim_row(input logic [63:0] pc);
    return pc[12:5] ^ pc[20:13];
  endfunction

  //==========================================================================
  // FTQ
  //==========================================================================
  logic [63:0]       ftq_pc  [FTQ_SIZE];
  logic [63:0]       ftq_tgt [FTQ_SIZE];
  // RAS 快照 (链式 RAS：栈顶、分配指针、深度；错路压栈只写空闲槽，恢复这三个即可)
  logic [4:0]        ftq_rtos   [FTQ_SIZE];
  logic [4:0]        ftq_rprev  [FTQ_SIZE];   // 快照栈顶的下一层 (弹栈时不必再查 rs_prev)
  logic [4:0]        ftq_ralloc [FTQ_SIZE];
  logic [5:0]        ftq_rcnt   [FTQ_SIZE];
  logic [7:0]        ftq_row [FTQ_SIZE];     // 方向表行号
  logic [31:0]       ftq_ctr [FTQ_SIZE];     // 预测时读到的 16 个计数器
  logic [15:0]       ftq_brm [FTQ_SIZE];     // 已解析的条件分支槽 (按块内绝对槽号)
  logic [15:0]       ftq_brt [FTQ_SIZE];     // 对应的实际方向
  logic [3:0]        ftq_slot0 [FTQ_SIZE];   // 本 FTQ 项起始 PC 在块内的槽号
  // 本项第一条是跨块 RVI：ftq_pc 记的是上一块最后的半字 (hwpc)，而预测 (TAGE 索引、uBTB 键) 用的是本块的 PC。
  // 训练要用本块：ftq_pc + 2 (不分开的话，热循环里一条跨界指令就让后面整块的分支永远训不进 TAGE)
  logic              ftq_hwm   [FTQ_SIZE];
  // 历史快照 (本块预测前的折叠历史与推入计数：重定向恢复、TAGE 训练算索引都用它)
  bp_fold_t          ftq_fold [FTQ_SIZE];
  logic [BP_KW-1:0]  ftq_hcnt [FTQ_SIZE];
  logic [BP_NT-1:0][3:0] ftq_ob [FTQ_SIZE];   // 块预测时历史的移出位 (每表 g[L-1..L-4])，重定向恢复折叠用
  // TAGE 训练：每槽的预测信息、哪些槽是条件分支 (预译码)，预测时读到的各表两路 {v, u}。
  // 训练的两个分支在提交时按实际结果挑 (误预测的优先)：按位置固定取前两个会让块里靠后的误预测分支
  // 永远训不到 (模型：CoreMark 误预测 1.03 -> 2.58 MPKI)
  bp_slot_t [15:0]   ftq_sinfo [FTQ_SIZE];
  it_info_t          ftq_itinfo [FTQ_SIZE];  // ITTAGE 预测时的信息
  logic              ftq_itv   [FTQ_SIZE];   // 块里有已解析的非返回 JALR
  logic [38:1]       ftq_itgt  [FTQ_SIZE];   // 它的实际目标
  logic [15:0]       ftq_cmask [FTQ_SIZE];
  logic [BP_NT-1:0][1:0][2:0] ftq_uv [FTQ_SIZE];
  ftq_ptr_t          ftq_head, ftq_tail;
  ftq_ptr_t          ftq_trn;      // 训练指针：[trn, head) 是已释放、还没训练方向表的项 (仍算占用)
  // 训练读 FTQ 分两拍 (2026-10-23，6 GHz)：本拍选项 (trn_sel) 寄存，下拍才读该项。所以占用按上一拍的训练指针算，
  // 选中的项在读出之前不会被新块覆盖。原来 训练指针 -> 选项 -> 64 选 1 读 ftq_fold 等 -> tr_* 一拍 (fe9：210 ps)
  ftq_ptr_t          ftq_trn_q;
  wire  [FTQ_W:0]    ftq_cnt = (ftq_tail.flag == ftq_trn_q.flag) ? {1'b0, ftq_tail.idx} - {1'b0, ftq_trn_q.idx}
                                                                : {1'b0, ftq_tail.idx} + FTQ_SIZE - {1'b0, ftq_trn_q.idx};
  wire               ftq_full = (ftq_cnt >= FTQ_SIZE - 1);

  // 训练选择：[trn, head) 是已提交待训练的项。只有含已解析条件分支 (brm != 0) 的项要训练
  // (bimodal 改计数器 / TAGE 候选)；其余项直接越过。一拍最多看 TRN_LA 项、训练其中第一个要训练的。
  // 块很短时 (dhrystone 3~4 条/块) 提交可超过 1 块/拍，原来每拍只走一项，ROB 加大后 FTQ 被待训练项
  // 占满、前端停顿 (t52：取指缓冲空 20%，其中 FTQ 满 7k 拍)
  localparam int TRN_LA = 4;
  logic [FTQ_W-1:0] trn_sel;
  logic             trn_selv;
  logic [2:0]       trn_adv;
  always_comb begin
    logic [FTQ_W:0] avail;
    logic           found;
    avail = ftq_head - ftq_trn;               // {flag, idx} 按 2 的幂回绕相减 = 项数
    trn_sel = ftq_trn.idx; trn_selv = 1'b0; trn_adv = '0; found = 1'b0;
    for (int k = 0; k < TRN_LA; k++)
      if (!found && (FTQ_W+1)'(k) < avail) begin
        logic [FTQ_W-1:0] ix;
        ix = ftq_trn.idx + FTQ_W'(k);
        trn_adv = 3'(k + 1);
        if (|ftq_brm[ix]) begin found = 1'b1; trn_selv = 1'b1; trn_sel = ix; end
      end
  end


  always_comb begin
    for (int r = 0; r < N_FTQ_RD; r++) begin
      ftq_pc_o[r]  = ftq_pc[ftq_ridx_i[r]];
      ftq_tgt_o[r] = ftq_tgt[ftq_ridx_i[r]];
    end
  end

  //==========================================================================
  // 取指缓冲
  //==========================================================================
  fetch_inst_t       ib [IBUF_SIZE];
  logic [4:0]        ib_head, ib_tail;
  // 提交级冲刷先寄存一拍再用 (2026-10-29，6 GHz：fe19 in 组 cm_flush_i -> F1 / 历史 / TAGE / PTW 请求 −50.7；
  // flush 是 o6_commit_ctrl 按 ROB 头组合出来的，到得晚)。冲刷那拍后端已清空，前端多走的一拍错误路径在 fl_q 拍清掉，
  // 那拍 ib 输出与 ITLB 请求按 fl_q 门控 (不让错误路径进后端)；同拍的后端重定向必比提交级那条新，被 fl_q 覆盖。
  // 代价：提交级冲刷 (异常 / xRET / CSR 写等) 后取指晚 1 拍
  logic              fl_q;
  logic [63:0]       rpc_q;
  logic [FTQ_W-1:0]  fftq_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) fl_q <= 1'b0;
    else         fl_q <= cm_flush_i;
  always_ff @(posedge clk_i) begin rpc_q <= cm_redirect_pc_i; fftq_q <= cm_flush_ftq_i; end
  logic [5:0]        ib_cnt;
  always_comb begin
    for (int s = 0; s < W; s++) begin
      ib_inst_o[s]  = ib[5'(ib_head + s)];
      ib_valid_o[s] = (6'(s) < ib_cnt) && !fl_q;
    end
  end

  //==========================================================================
  // 预测器存储
  //==========================================================================
  // uBTB：块起始 PC -> 两个控制流槽 (按槽号升序：两个都有效时槽 0 在前)，每槽：槽号 (相对块基址的半字号)、
  // 是否条件分支、目标、是否返回 / 调用、返回地址偏移；条件分支的方向在 uPHT (按 PC 与全局历史索引)。
  // 预测取"第一个无条件、或 uPHT 说跳"的槽 (分支模型 bpsim：2 槽 + 计数器把 uBTB 造成的气泡降到约 1/3)
  logic [NUB-1:0]    ub_v;
  logic [30:1]       ub_tag  [NUB];     // 部分标签：别名只造成错误预测，F3 会纠正
  logic              ub_sv   [2][NUB];  // 槽有效
  logic [3:0]        ub_slot [2][NUB];
  logic              ub_cond [2][NUB];
  logic [38:1]       ub_tgt  [2][NUB];  // 目标 (Sv39 虚地址，读出时符号扩展)
  logic              ub_ret  [2][NUB];
  logic              ub_call [2][NUB];  // 跳转指令是调用：F0 预测时就压 RAS
  logic [6:0]        ub_lk   [2][NUB];  // 调用的返回地址相对块基址的偏移 (跨块 RVI 时为 +2)
  logic [1:0]        ub_ph   [2][NUB];  // 目标的路径散列 bp_phash (训练时算好存下，F0 推入历史不必取目标再散列)
  logic [$clog2(UBS)-1:0] ub_ts   [2][NUB];  // 目标块的 uBTB 组号 ub_set (训练时算好；F0 下一拍查找不必再散列)
  logic [UPW-1:0]    ub_tu   [2][NUB];  // 目标块 uPHT 索引的 PC 部分 up_pc (同上，下一拍 uPHT 索引 upi_q 预算用)
  logic [UWW-1:0]    ub_rr   [UBS];     // 每组替换的轮转指针
  logic [1:0]        upht    [2][NUP];  // uBTB 槽 k 的方向计数器 (按 up_idx 索引)
  // uPHT 索引：块 PC 与 gh0 低 UPW 位 (最近 UPW/2 次跳转的路径散列) 异或
  // up_idx 的 PC 部分 (跳转目标的这部分训练时算好存在 ub_tu)
  function automatic logic [UPW-1:0] up_pc(input logic [63:0] pc);
    return pc[UPW+4:5] ^ pc[2*UPW+4:UPW+5];
  endfunction
  // up_idx 的历史部分
  function automatic logic [UPW-1:0] up_gh(input logic [BP_GHL-1:0] g);
`ifdef O6_NO_UPHT
    return '0;                                                  // 对照实验：不带历史
`else
    return g[UPW-1:0];
`endif
  endfunction
  function automatic logic [UPW-1:0] up_idx(input logic [63:0] pc, input logic [BP_GHL-1:0] g);
    return up_pc(pc) ^ up_gh(g);
  endfunction
  // 组号：块入口 PC 的 5 位散列 (与 bpsim 的 ub_range 相同)
  function automatic logic [$clog2(UBS)-1:0] ub_set(input logic [30:1] a);
    return a[5:1] ^ a[10:6] ^ a[15:11];
  endfunction
  function automatic logic [63:0] ub_tx(input logic [38:1] t);
    return {{25{t[38]}}, t, 1'b0};
  endfunction
  // 预测目标：各表只存 [38:1]，高位 [63:39] 取所在块的 PC (Sv48 / Sv57 下跨 2^39 边界的跳转预测错，由后端纠正；
  //   Sv39 下与原来的符号扩展只差在跨正负半区的跳转)。不加宽表项：BTB / uBTB / LP / ITTAGE / FTQ 各省 18 位
  function automatic logic [63:0] tg_x(input logic [38:1] t, input logic [63:0] pc);
    return {pc[63:39], t, 1'b0};
  endfunction

  // F1 BTB：块起始 PC -> 第一个跳转的槽号、目标 (Sv39 虚地址 38:1 位)、是否返回 / 条件分支。
  // F0 读出、F1 比较标签；uBTB 没预测跳而 BTB 命中 (条件分支再看 bimodal) 时在 F1 改道 (1 拍气泡)。
  // 只由 F3 的预测不一致训练 (写入最终的跳转；最终不跳则作废)
  logic [NBTB-1:0]   bt_v;
  logic [15:0]       bt_tag  [NBTB];
  logic [3:0]        bt_slot [NBTB];
  logic [38:1]       bt_tgt  [NBTB];
  logic              bt_ret  [NBTB];
  logic              bt_cond [NBTB];
  logic              bt_call [NBTB];
  logic [6:0]        bt_lk   [NBTB];
  function automatic logic [BTW-1:0] bt_idx(input logic [63:0] pc);
    return pc[BTW+4:5] ^ BTW'({pc[4:1], 4'b0});      // 同一块不同入口 (调用返回后) 分到不同项
  endfunction
  function automatic logic [15:0] bt_tg(input logic [63:0] pc);
    return {pc[4:1], pc[24:13]};
  endfunction

  // 方向表：触发器 (256 x 32 位)
  logic [31:0]       bim [BIM_R];

  // 全局历史 (跳转路径历史)：gh0[0] 最新；hcnt0 = 推入次数 (取模)；fold0 = 各表折叠
  // gh0 = 生效的原始历史 (组合)：F0 预测跳的推入晚一拍才写进 gh0_r (2026-10-25，6 GHz)。F0 的跳转判定约 150 ps
  // 才出来，原来要当拍控制 430 位 gh0 每位的选择 (fe14：pc0_q -> uBTB -> f0_tk -> gh0 183 ps)；现在只写 gh_pend / gh_h
  // 3 个寄存器，下一拍由寄存器 gh_pend 把它并进 gh0_r。所有读 gh0 的地方读的都是并进待推入后的值，与原来逐拍相同
  logic [BP_GHL-1:0] gh0_r, gh0;
  logic              gh_pend;
  logic [1:0]        gh_h;
  assign gh0 = gh_pend ? {gh0_r[BP_GHL-3:0], gh_h[0], gh_h[1]} : gh0_r;
  logic [BP_KW-1:0]  hcnt0;
  bp_fold_t          fold0;

  // 折叠寄存器的一步 (推入 1 位)：循环左移 1，第 0 位异或进入位，第 L mod 宽度 位异或移出位
  function automatic logic [BP_LG-1:0] st_i(input logic [BP_LG-1:0] c, input logic n, input logic o, input int L);
    logic [BP_LG-1:0] r;
    r = {c[BP_LG-2:0], c[BP_LG-1]};
    r[0] = r[0] ^ n;
    r[L % BP_LG] = r[L % BP_LG] ^ o;
    return r;
  endfunction
  function automatic logic [BP_TW-1:0] st_g1(input logic [BP_TW-1:0] c, input logic n, input logic o, input int L);
    logic [BP_TW-1:0] r;
    r = {c[BP_TW-2:0], c[BP_TW-1]};
    r[0] = r[0] ^ n;
    r[L % BP_TW] = r[L % BP_TW] ^ o;
    return r;
  endfunction
  function automatic logic [BP_TW-2:0] st_g2(input logic [BP_TW-2:0] c, input logic n, input logic o, input int L);
    logic [BP_TW-2:0] r;
    r = {c[BP_TW-3:0], c[BP_TW-2]};
    r[0] = r[0] ^ n;
    r[L % (BP_TW - 1)] = r[L % (BP_TW - 1)] ^ o;
    return r;
  endfunction
  // 推入 2 位 (先 h[0] 后 h[1])；g 是推入前的原始历史，移出位依次是 g[L-1]、g[L-2]
  function automatic bp_fold_t fold_push(input bp_fold_t f, input logic [BP_GHL-1:0] g, input logic [1:0] h);
    bp_fold_t r;
    for (int t = 0; t < BP_NT; t++) begin
      int L;
      L = bp_hl(t);
      r.fi[t] = st_i (st_i (f.fi[t], h[0], g[L-1], L), h[1], g[L-2], L);
      r.g1[t] = st_g1(st_g1(f.g1[t], h[0], g[L-1], L), h[1], g[L-2], L);
      r.g2[t] = st_g2(st_g2(f.g2[t], h[0], g[L-1], L), h[1], g[L-2], L);
    end
    return r;
  endfunction
  // 同上，但移出位直接给出 (o[t] = {g[L-2], g[L-1]})：重定向恢复时不必把原始历史右移回去
  function automatic bp_fold_t fold_push_ob(input bp_fold_t f, input logic [BP_NT-1:0][1:0] o, input logic [1:0] h);
    bp_fold_t r;
    for (int t = 0; t < BP_NT; t++) begin
      int L;
      L = bp_hl(t);
      r.fi[t] = st_i (st_i (f.fi[t], h[0], o[t][0], L), h[1], o[t][1], L);
      r.g1[t] = st_g1(st_g1(f.g1[t], h[0], o[t][0], L), h[1], o[t][1], L);
      r.g2[t] = st_g2(st_g2(f.g2[t], h[0], o[t][0], L), h[1], o[t][1], L);
    end
    return r;
  endfunction
  // 各表的移出位：[0] = g[L-1]、[1] = g[L-2] (第一次推入)，[2] = g[L-3]、[3] = g[L-4] (再推一次)
  function automatic logic [BP_NT-1:0][3:0] hist_ob(input logic [BP_GHL-1:0] g);
    logic [BP_NT-1:0][3:0] r;
    for (int t = 0; t < BP_NT; t++) r[t] = {g[bp_hl(t)-4], g[bp_hl(t)-3], g[bp_hl(t)-2], g[bp_hl(t)-1]};
    return r;
  endfunction
  // 全量折叠 (仿真断言用)：年龄 i 的位落在第 i mod 宽度 位
  function automatic bp_fold_t fold_full(input logic [BP_GHL-1:0] g);
    bp_fold_t r;
    r = '0;
    for (int t = 0; t < BP_NT; t++)
      for (int i = 0; i < BP_HMAX; i++)
        if (i < bp_hl(t)) begin
          r.fi[t][i % BP_LG]       = r.fi[t][i % BP_LG] ^ g[i];
          r.g1[t][i % BP_TW]       = r.g1[t][i % BP_TW] ^ g[i];
          r.g2[t][i % (BP_TW - 1)] = r.g2[t][i % (BP_TW - 1)] ^ g[i];
        end
    return r;
  endfunction

  // 访存相关预测 (store-wait 表)：按 load PC 散列，1 = 曾越过未知地址的 store 违例
  // 64 行 x 16 位：行 = 块地址散列，位 = 块内半字号 (跨块 RVI 不预测)
  localparam int SWR = 64;
  logic [15:0]       swt [SWR];
  function automatic logic [5:0] swt_row(input logic [63:0] pc);
    return pc[10:5] ^ pc[16:11];
  endfunction

  // RAS
  // 链式 RAS：32 个槽环形分配，每槽 {返回地址, 下一层槽号}；压栈写在 alloc 处 (空闲槽)，
  // 弹栈只把栈顶指回下一层。错路压栈不会覆盖有效条目，恢复只需栈顶/分配指针/深度。
  localparam int NRAS = 32;
  logic [63:0]       rs_addr [NRAS];
  logic [4:0]        rs_prev [NRAS];
  logic [4:0]        rs_tos, rs_alloc;
  logic [5:0]        ras_cnt;              // 有效深度 (饱和在 NRAS)
  wire  [63:0]       ras_top = rs_addr[rs_tos];

  //==========================================================================
  // F0
  //==========================================================================
  // F0 的取指 PC：分支误预测重定向那一拍直接用重定向 PC (不等写进 pc0_q 再下一拍发，误预测代价少 1 拍)。
  // 只用于后端分支重定向：提交级冲刷 (xRET / CSR 写 / SFENCE / 异常) 那一拍特权级、satp 等正在变，
  // F0 同拍取指会用旧的转换状态 (ptw_pmp：mret 进 S 态的第一块按 M 态取到了)，仍按原来下一拍取。
  // 这一拍的预测用同拍恢复出的历史 (foldx / hcntx) 与 RAS 快照 (rx_rd)，但不做压 / 弹栈
  // (预测到调用 / 返回就当顺序块，由 F3 纠正)：RAS 一拍只有一个写口，已被重定向本身的压栈占用
  logic [63:0] pc0_q;
  wire         redirect = be_redirect_i || fl_q;
  wire         byp = be_redirect_i && !fl_q;
  wire  [63:0] pc0 = byp ? be_redirect_pc_i : pc0_q;
  logic        f0_stop;          // 取指异常后停止

  // uBTB 查找：用 pc0_q 而不是 pc0 (2026-10-22，6 GHz)。重定向拍 (byp) 的 uBTB 结果本来就不用 (ub_ok 带 !byp)，
  // 查找改用寄存器后，be_redirect_pc -> uBTB -> f0_tk / f0_npc -> 历史 / pc0_q 的结构路径不再存在
  // (fe7 'in'：be_redirect_pc -> fold0 250 ps，功能上是假路径)。训练用的 uPHT 索引 f0_upi 仍按真实 pc0 算。
  wire  [UPW-1:0] f0_upi    = up_idx(pc0, gh0);
  // 查找用的 uPHT 索引 = up_idx(pc0_q, gh0)，写 pc0_q 时按情况算好寄存 (2026-10-26，6 GHz：
  // fe15 pc0_q -> 三输入异或 -> 1024 项读出 -> 方向 -> 命中 -> f1_npc 185 ps；断言核对)
  logic [UPW-1:0] upi_q;
  wire  [UPW-1:0] f0_upi_lk = upi_q;
  // 各恢复情况的候选历史 (逻辑在历史恢复一节)
  logic [BP_GHL-1:0]     g_m3, g_f1, g_rd, rg_rd;
  logic [1:0]            h_m3, h_f1;
  logic                  push_rd;
  logic [1:0]  up_c [2];
  logic        ub_hit;
  logic        ub_mh;              // 组内多路同时命中 (重复项；只用于断言与统计)
  logic [3:0]  ub_hslot;
  logic [63:0] ub_htgt;
  logic        ub_hret, ub_hcall;
  logic [6:0]  ub_hlk;
  logic [1:0]  ub_hph;
  // 2026-10-25，6 GHz：(1) 组号用寄存的 ubs_q (写 pc0_q 时顺带算好；uBTB 目标的组号训练时存在 ub_ts)，
  // 组号散列不在 F0 环里；(2) 命中后的逻辑按路预先算好 (各路"是否跳 / 去向 / 推入位"与比标签并行)，
  // 命中独热出来后只剩一级与或选择。原来 pc0_q -> 散列 -> 比标签 -> 选槽 -> 是否跳 -> f0_h -> fold0 共 207 ps
  logic [$clog2(UBS)-1:0] ubs_q;      // = ub_set(pc0_q[30:1])，断言核对
  logic [$clog2(UBS)-1:0] ub_hts;     // 跳转目标块的组号
  logic [UPW-1:0]         ub_hupi;    // 跳转目标块的 uPHT 索引 (目标 up_pc ^ 推入后的历史低位)
  // 推入本块 2 位后的历史低位 (gh0 低位是 gh0_r / gh_h 的二选一，寄存器早到)
  function automatic logic [UPW-1:0] gh_push_lo(input logic [BP_GHL-1:0] g, input logic [1:0] h);
    logic [BP_GHL-1:0] x;
    x = {g[BP_GHL-3:0], h[0], h[1]};
    return up_gh(x);
  endfunction
  always_comb begin
    logic [UBW-1:0] m;
    logic [$clog2(UBS)-1:0] st;
    // 两种情况：p = 槽 0 计数器的方向 (up_c[0][1])。命中字段对两种 p 各算一份，与 uPHT 读出并行，
    // 最后按 p 二选一 (原来读出 -> 方向 -> 选槽 -> 与或串在一起)。槽 1 的方向只影响"是否命中"
    logic           hp [2], h1;
    logic [3:0]     slp [2];
    logic [63:0]    tgp [2];
    logic           rtp [2], clp [2];
    logic [6:0]     lkp [2];
    logic [1:0]     php [2];
    logic [$clog2(UBS)-1:0] tsp [2];
    logic [UPW-1:0] uip [2];
    st = ubs_q;
    for (int w = 0; w < UBW; w++) m[w] = ub_v[{st, UWW'(w)}] && ub_tag[{st, UWW'(w)}] == pc0_q[30:1];
    for (int k = 0; k < 2; k++) up_c[k] = upht[k][f0_upi_lk];
    ub_mh = (m & (m - 1'b1)) != '0;
    // 组内项按标签唯一 (训练时先查找已有项)，命中独热与或选择
    h1 = 1'b0;
    for (int q = 0; q < 2; q++) begin
      hp[q] = 1'b0; slp[q] = '0; tgp[q] = '0; rtp[q] = 1'b0; clp[q] = 1'b0; lkp[q] = '0; php[q] = '0; tsp[q] = '0; uip[q] = '0;
    end
    for (int w = 0; w < UBW; w++) begin
      logic [$clog2(NUB)-1:0] i;
      logic t1;
      i  = {st, UWW'(w)};
      t1 = ub_sv[1][i] && (!ub_cond[1][i] || up_c[1][1]);
      h1 = h1 | (m[w] & t1);
      for (int q = 0; q < 2; q++) begin
        logic sel0;
        // 槽 0 在前：它预测跳就取它，否则看槽 1
        sel0 = ub_sv[0][i] && (!ub_cond[0][i] || q[0]);
        hp[q]  = hp[q]  | (m[w] & sel0);
        slp[q] = slp[q] | ({4{m[w]}} & (sel0 ? ub_slot[0][i] : ub_slot[1][i]));
        tgp[q] = tgp[q] | ({64{m[w]}} & tg_x(sel0 ? ub_tgt[0][i] : ub_tgt[1][i], pc0));
        rtp[q] = rtp[q] | (m[w] & (sel0 ? ub_ret[0][i] : ub_ret[1][i]));
        clp[q] = clp[q] | (m[w] & (sel0 ? ub_call[0][i] : ub_call[1][i]));
        lkp[q] = lkp[q] | ({7{m[w]}} & (sel0 ? ub_lk[0][i] : ub_lk[1][i]));
        php[q] = php[q] | ({2{m[w]}} & (sel0 ? ub_ph[0][i] : ub_ph[1][i]));
        tsp[q] = tsp[q] | ({$clog2(UBS){m[w]}} & (sel0 ? ub_ts[0][i] : ub_ts[1][i]));
        uip[q] = uip[q] | ({UPW{m[w]}} & ((sel0 ? ub_tu[0][i] : ub_tu[1][i]) ^
                                          gh_push_lo(gh0, sel0 ? ub_ph[0][i] : ub_ph[1][i])));
      end
    end
    begin
      int q;
      q = int'(up_c[0][1]);
      ub_hit   = hp[q] | h1;
      ub_hslot = slp[q]; ub_htgt = tgp[q]; ub_hret = rtp[q]; ub_hcall = clp[q];
      ub_hlk   = lkp[q]; ub_hph  = php[q]; ub_hts  = tsp[q]; ub_hupi  = uip[q];
    end
  end
  // 重定向拍 F0 不用跳转预测 (当顺序块，F1 BTB / F3 纠正)，2026-10-22，6 GHz：
  // 重定向 PC 是 EXU 寄存器输出、过长线到前端 (约 0.3T 到达)，同拍再查表 -> 下一块地址 / 历史推入
  // 放不进一拍 (fe5 'in'：be_redirect -> uBTB -> fold0 253 ps)。CoreMark +0.27 %、dhrystone +0.06 % (u14 对 u13)。
  // uBTB (+ uPHT) 的结论只寄存给 F1 (见下：行预测器)
  wire        ub_ok1 = ub_hit && !byp;

  //--------------------------------------------------------------------------
  // 行预测器 LP (2026-10-27，用户定：6 GHz 下 128 项 uBTB 的 "读组 + 4 路比较 + 选择 + uPHT" 单拍环约 185 ps，
  // F0 改为只查 64 项直接映射的 LP，环约 120 ps；uBTB + uPHT 照旧在 F0 查、结论寄存进 F1，
  // 与 LP 不一致时 F1 改道 (多一拍气泡)。bpsim：损失周期 / 千条 CoreMark +5.8、Linux 启动 +10.4)
  //   每项：标签、预测跳 tk、槽、目标、返回 / 调用、链接偏移、路径散列，以及目标块的 uBTB 组号 / uPHT 索引 PC 部分 /
  //   LP 索引 (下一拍的 ubs_q / upi_q / lps_q 不必再散列)。F1 改道时按 F1 的最终预测写入 (见 F1 一节)
  //--------------------------------------------------------------------------
`ifdef O6_NLP
  localparam int NLP = `O6_NLP;          // 性能实验用
`else
  // 128 项 (2026-10-28)：fe18 综合 F0 环不在最差之列 (LP 相关最差 −12.9 @SP180)；
  // 比 64 项 CoreMark 好 0.2 %、dhrystone 好 3.3 % (相对无 LP：+2.0 % / +0.3 % 拍)
  localparam int NLP = 128;
`endif
  localparam int LPW = $clog2(NLP);
  localparam int LTW = 14;
  // 散列索引必须含块内偏移 pc[4:1]：只用行地址时，同一 32 B 行里不同起点的取指块 (跳转目标落在行中)
  // 永远抢同一项 —— lpsz 实测 64/128/256 项缺项数几乎不变 (dhrystone 10.8k/9.8k/9.8k)，就是这种冲突。
  // 标签 pc[LPW+LTW:LPW+1] 覆盖第二段折叠位，与索引一起唯一还原 pc[LPW+LTW:1]
  function automatic logic [LPW-1:0] lp_idx(input logic [63:0] a); return a[LPW:1] ^ a[2*LPW:LPW+1]; endfunction
  function automatic logic [LTW-1:0] lp_tg(input logic [63:0] a); return a[LPW+LTW:LPW+1]; endfunction
  logic              lp_v    [NLP];
  logic [LTW-1:0]    lp_tag  [NLP];
  logic              lp_tk   [NLP], lp_ret [NLP], lp_call [NLP];
  logic [3:0]        lp_slot [NLP];
  logic [38:1]       lp_tgt  [NLP];
  logic [6:0]        lp_lk   [NLP];
  logic [1:0]        lp_ph   [NLP];
  logic [$clog2(UBS)-1:0] lp_ts [NLP];
  logic [UPW-1:0]    lp_tu   [NLP];
  logic [LPW-1:0]    lp_tl   [NLP];
  logic [LPW-1:0]    lps_q;                   // = lp_idx(pc0_q)，写 pc0_q 时算好 (断言)
  wire               lp_hit  = lp_v[lps_q] && lp_tag[lps_q] == lp_tg(pc0_q);
  wire               lp_hret = lp_ret[lps_q];

  wire        f0_ok  = lp_hit && lp_tk[lps_q] && !(lp_hret && ras_cnt == 0) && !byp;
  wire [63:0] f0_seq = blk_base(pc0) + 64'd32;
  wire [63:0] f0_npc = f0_ok ? (lp_hret ? ras_top : tg_x(lp_tgt[lps_q], pc0)) : f0_seq;
  wire [$clog2(UBS)-1:0] f0_nset = !f0_ok ? ub_set(f0_seq[30:1]) : lp_hret ? ub_set(ras_top[30:1]) : lp_ts[lps_q];
  wire [LPW-1:0]         f0_nlps = !f0_ok ? lp_idx(f0_seq) : lp_hret ? lp_idx(ras_top) : lp_tl[lps_q];

  wire f0_tk = f0_ok;                                    // F0 对本块"以跳转结束"的推测 (进历史)
  // F0 推入历史的 2 位 (= bp_phash(槽, f0_npc))：目标的散列取训练时存好的 lp_ph，返回取 RAS 栈顶的散列
  wire [1:0] ras_ph = bp_phash(4'd0, ras_top);
  wire [1:0] f0_h   = lp_hret ? ras_ph : lp_ph[lps_q];
  wire [UPW-1:0] f0_nupi = !f0_ok ? (up_pc(f0_seq) ^ up_gh(gh0)) :
                           lp_hret ? (up_pc(ras_top) ^ gh_push_lo(gh0, ras_ph)) : (lp_tu[lps_q] ^ gh_push_lo(gh0, lp_ph[lps_q]));
  // RAS 在预测时就推测更新 (F0 / F1 改道 / F3 纠正)，各级带着"本块预测前"的 RAS 快照用于恢复
  wire        f0_push = f0_tk && lp_call[lps_q];
  wire        f0_pop  = f0_tk && lp_hret;
  wire [63:0] f0_link = blk_base(pc0) + {{57{lp_lk[lps_q][6]}}, lp_lk[lps_q]};

  // ITLB + PMP
  logic [VPN_W-1:0] itlb_vpn [1];
  logic [0:0]       itlb_hit;
  logic [PA_W-13:0] itlb_ppn [1];
  logic [7:0]       itlb_perm [1];
  logic [1:0]       itlb_fault [1];
  logic [3:0]       itlb_gperm [1];
  logic [56:0]      itlb_gpa   [1];
  logic             itlb_gimp  [1];
  assign itlb_vpn[0] = pc0[VA_W-1:12];
  o6_tlb #(.N(32), .NL(1)) u_itlb (
    .clk_i, .rst_ni, .flush_i(tlb_flush_i),
    .lk_vpn_i(itlb_vpn), .lk_v_i(xv_i), .lk_hit_o(itlb_hit), .lk_ppn_o(itlb_ppn), .lk_perm_o(itlb_perm),
    .lk_fault_o(itlb_fault), .lk_pbmt_o(), .lk_gperm_o(itlb_gperm), .lk_gpa_o(itlb_gpa), .lk_gimp_o(itlb_gimp),
    .fill_i(ifill_i), .fill_v_i, .fill_vpn_i, .fill_level_i, .fill_nap_i, .fill_ppn_i, .fill_perm_i, .fill_pbmt_i(2'b00), .fill_fault_i,
    .fill_gperm_i, .fill_gpa_i, .fill_gimp_i
  );
  logic       f0_umiss, f0_fflt, f0_flt;
  logic [5:0] f0_cause;
  logic [63:0] f0_gx;                         // H：guest page fault 的 GPA (随 F1 -> F3 走)
  logic [63:0] fgx_q;
  // L0 ITLB：1 项 (取指几乎总在同一页)。可执行区域 (DRAM / BootROM) 按页在装入时判好；
  // 调试模块区 (第 0 页) 随调试模式变化，用时再看
  logic             ut_v, ut_xok, ut_pg0;
  logic [VPN_W-1:0] ut_vpn;
  logic             ut_xv, uq_xv;               // H：L0 项的 V
  logic [PA_W-13:0] ut_ppn;
  logic [7:0]       ut_perm;
  logic [1:0]       ut_fault;
  logic             ut_gxp, uq_gxp;             // H：G-stage 可执行 (gperm.X)
  logic [56:0]      ut_gpa, uq_gpa;             // H：GPA (guest page fault 的 htval)
  logic             ut_gimp, uq_gimp;
  // 装入分两拍：f0_umiss 拍把 32 项 ITLB 的查找结果寄存 (uq_*)，下一拍判可执行区并写 L0 (期间 F0 一直等)
  logic             uq_v, uq_hit;
  logic [VPN_W-1:0] uq_vpn;
  logic [PA_W-13:0] uq_ppn;
  logic [7:0]       uq_perm;
  logic [1:0]       uq_fault;
  wire  [63:0]      uq_pbase = {{(64-PA_W){1'b0}}, uq_ppn, 12'b0};
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin ut_v <= 1'b0; uq_v <= 1'b0; end
    else if (tlb_flush_i) begin ut_v <= 1'b0; uq_v <= 1'b0; end
    else begin
      uq_v <= f0_umiss;
      if (uq_v) ut_v <= uq_hit;
    end
  end
  always_ff @(posedge clk_i) begin
    if (f0_umiss) begin
      uq_hit <= itlb_hit[0]; uq_vpn <= pc0[VA_W-1:12]; uq_ppn <= itlb_ppn[0]; uq_xv <= xv_i;
      uq_perm <= itlb_perm[0]; uq_fault <= itlb_fault[0];
      uq_gxp <= itlb_gperm[0][2]; uq_gpa <= itlb_gpa[0]; uq_gimp <= itlb_gimp[0];
    end
    if (uq_v) begin
      ut_vpn <= uq_vpn; ut_xv <= uq_xv; ut_ppn <= uq_ppn; ut_perm <= uq_perm; ut_fault <= uq_fault;
      ut_gxp <= uq_gxp; ut_gpa <= uq_gpa; ut_gimp <= uq_gimp;
      ut_xok <= (uq_pbase >= DRAM_BASE && uq_pbase < DRAM_BASE + 64'h8000_0000) ||
                (uq_pbase >= BOOTROM_BASE && uq_pbase < BOOTROM_BASE + 64'h1_0000);
      ut_pg0 <= uq_pbase[PA_W-1:12] == '0;
    end
  end

  logic [63:0] pa0;
  logic        ipmp_ok;
  o6_pmp #(.N(8)) u_ipmp (
    .dec_i(pmpd_i), .pa_i(pa0[PA_W-1:0]), .priv_i(priv_i), .acc_i(3'b100), .ok_o(ipmp_ok)
  );

  // f0_umiss：需要转换但 L0 ITLB 不是这一页 -> 本拍不发，查 32 项 ITLB 装入 L0 (缺失则请求 PTW)
  // f0_fflt：不经 PMP 就能判定的取指异常 (只看寄存器，很浅)，这种不发 I$ 请求
  // PMP 拒绝只进 F1 的异常标记：请求照发 (可执行区都是无副作用的存储器)，下拍作废
  always_comb begin
    logic [7:0] pm;
    f0_umiss = 1'b0; f0_fflt = 1'b0; f0_cause = EXC_INST_ACCESS;
    pa0 = pc0;
    pm = ut_perm;
    f0_gx = {7'b0, ut_gpa[56:10], pc0[11:2]};
    if (xlate_i) begin
      pa0 = {{(64-PA_W){1'b0}}, ut_ppn, pc0[11:0]};
      if (cn_i && !va_canon(pc0, vam_i))                  begin f0_fflt = 1'b1; f0_cause = EXC_INST_PAGE; end
      else if (!cn_i && pc0[63:VA_W] != '0)               begin f0_fflt = 1'b1; f0_cause = EXC_INST_GPAGE; f0_gx = {7'b0, pc0[58:2]}; end
      else if (!(ut_v && ut_xv == xv_i && ut_vpn == pc0[VA_W-1:12])) f0_umiss = 1'b1;
      else if (ut_fault == 2'd1)                          begin f0_fflt = 1'b1; f0_cause = EXC_INST_PAGE; end
      else if (ut_fault == 2'd2)                          begin f0_fflt = 1'b1; f0_cause = EXC_INST_ACCESS; end
      else if (ut_fault == 2'd3) begin
        f0_fflt = 1'b1; f0_cause = EXC_INST_GPAGE;
        if (ut_gimp) f0_gx = {1'b1, 6'b0, ut_gpa};
      end
      else if (cn_i && (!pm[3] || !pm[6] || (priv_i == 2'b00 && !pm[4]) || (priv_i == 2'b01 && pm[4])))
        begin f0_fflt = 1'b1; f0_cause = EXC_INST_PAGE; end
      else if (!ut_gxp)                                   begin f0_fflt = 1'b1; f0_cause = EXC_INST_GPAGE; end   // G-stage 不可执行
      else if (!(ut_xok || (dbg_mode_i && ut_pg0)))         f0_fflt = 1'b1;
    end else if (!exec_ok(pc0)) f0_fflt = 1'b1;
    f0_flt = f0_fflt || (!f0_umiss && !ipmp_ok);
  end
  // PTW 指令请求打拍 (2026-11-06，6 GHz：跨模块 be_redirect_i -> pc0 -> ITLB -> ptw_ireq_o -> PTW −17.55)。
  // vpn / v 直接用 f0_umiss 拍已寄存的 uq_vpn / uq_xv，只新增 ireq_q。请求晚一拍：
  // - 填写拍 (ifill_i) 的 itlb_hit 还是旧值，要挡掉，否则下拍同一页再走一遍；tlb_flush_i 拍同样不留请求
  // - 输出再与 !fl_q：提交级冲刷后的第一拍不发冲刷前的请求 (与打拍前一致)
  // - 后端重定向后多留一拍旧路径请求，只是多一次投机页表遍历，填入的仍是该页的正确翻译
  logic ireq_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) ireq_q <= 1'b0;
    else         ireq_q <= (!f0_stop || byp) && f0_umiss && !itlb_hit[0] && !fl_q && !ifill_i && !tlb_flush_i;
  assign ptw_ireq_o = ireq_q && !fl_q;
  assign ptw_ivpn_o = uq_vpn;
  assign ptw_iv_o   = uq_xv;
  assign fgx_o      = fgx_q;

  //==========================================================================
  // F1 / F2 寄存器
  //==========================================================================
  logic        f1_v, f1_flt;        // F1：请求在途 (或合成的取指异常)
  logic [63:0] f1_gx, f2_gx, f3_gx;          // H：取指 guest page fault 的 GPA
  logic [5:0]  f1_cause;
  logic [63:0] f1_pc;
  logic        f1_ubhit, f1_ubret;    // F0 (LP) 的预测：跳 / 是返回
  logic        f1_uok, f1_uret, f1_ucall;   // F0 并行查的 uBTB (+ uPHT) 结论
  logic [3:0]  f1_uslot;
  logic [63:0] f1_utgt;
  logic [6:0]  f1_ulk;
  logic        f1_lph;               // F0 时 LP 命中 (统计用)
  logic [UPW-1:0] f1_upi, f2_upi, f3_upi;   // 本块 F0 查 uPHT 用的索引 (训练用)
  logic [UPW-1:0] ftq_upi [FTQ_SIZE];
  logic [3:0]  f1_ubslot;
  logic [63:0] f1_npc;              // F0 按 uBTB 假设的下一块 PC
  logic [15:0] f1_sw, f2_sw;         // store-wait 表的本块一行
  logic        f1_bv, f1_bret, f1_bcond, f1_bcall;   // F0 读出的 BTB 项
  logic [6:0]  f1_blk;
  // 各级的 RAS 快照 (本块在 F0 预测之前) 与本块已推测做过的压 / 弹
  logic [4:0]  f1_rtos, f1_ralloc, f2_rtos, f2_ralloc, f3_rtos, f3_ralloc;
  logic [4:0]  f1_rprev, f2_rprev, f3_rprev;   // 快照栈顶的下一层
  logic [5:0]  f1_rcnt, f2_rcnt, f3_rcnt;
  logic        f1_ppush, f1_ppop, f2_ppush, f2_ppop, f3_ppush, f3_ppop;
  logic [15:0] f1_btag;
  logic [3:0]  f1_bslot;
  logic [38:1] f1_btgt;
  logic [31:0] f1_ctr;              // 方向表一行
  bp_fold_t         f1_fold;          // 本块预测前的历史快照
  // 本块预测时历史的移出位 (2026-10-25，6 GHz)：F0 发出时直接取 (那时历史还没被本块之后的推入改动)，
  // 随块寄存到 F3。原来 F3 纠正 / F1 改道时现算 gh0 >> 2 * (hcnt0 - fx_hcnt) 再取移出位 (fe13：hcnt0 -> 移位 -> 折叠 -> fold0 181 ps)
  logic [BP_NT-1:0][3:0] f1_ob, f2_ob;
  logic [BP_KW-1:0] f1_hcnt;
  logic        f1_hv;               // F1 保持寄存器有数据 (I$ 响应只有一拍，F2 堵住时要存住)
  logic [255:0] f1_hold;

  logic        f2_v, f2_flt;
  logic [5:0]  f2_cause;
  logic [63:0] f2_pc;
  logic [255:0] f2_data;
  logic        f2_ubhit;
  logic [3:0]  f2_ubslot;
  logic [63:0] f2_npc;
  logic [31:0] f2_ctr;
  logic [7:0]  f2_row;
  bp_fold_t         f2_fold;
  logic [BP_KW-1:0] f2_hcnt;

  // 跨块 RVI 的低半部分 (已写入取指缓冲的最后一块留下的)
  logic        hw_v;
  logic [15:0] hw_d;
  logic [63:0] hw_pc;
  logic [63:5] hw_nb;              // hw 所在块的下一块基址 (只给断言核对)

  // F1 的最终预测 p1：uBTB (+ uPHT) 预测跳就用它；否则 BTB 命中且 (非条件 / bimodal 预测跳) 就用 BTB；否则顺序块。
  // 与 F0 (LP) 走的不同 (跳与否不同，或槽 / 目标不同) 就改道 (2026-10-27：原来只有 "uBTB 没预测跳、BTB 预测跳" 一种)
  wire        f1_bhit = f1_bv && f1_btag == bt_tg(f1_pc);
  wire [63:0] f1_btgt_e = f1_bret ? rs_addr[f1_rtos] : tg_x(f1_btgt, f1_pc);
  // 条件分支方向只看 bimodal：TAGE 的标签比较串进改道 -> RAS 写入太深，实测加 TAGE 改道次数几乎不变
  wire        f1_bdir = f1_ctr[2*f1_bslot+1];
  wire        p1_u    = f1_uok && !(f1_uret && f1_rcnt == 6'd0);
  wire        p1_b    = !p1_u && f1_bhit && (!f1_bcond || f1_bdir) && !(f1_bret && f1_rcnt == 6'd0);
  wire        p1_tk   = p1_u || p1_b;
  wire [3:0]  p1_slot = p1_u ? f1_uslot : f1_bslot;
  wire [63:0] p1_npc  = !p1_tk ? blk_base(f1_pc) + 64'd32 : p1_u ? (f1_uret ? rs_addr[f1_rtos] : f1_utgt) : f1_btgt_e;
  wire        p1_ret  = p1_tk && (p1_u ? f1_uret : f1_bret);
  wire        p1_call = p1_tk && (p1_u ? f1_ucall : f1_bcall);
  wire [6:0]  p1_lk   = p1_u ? f1_ulk : f1_blk;
  wire        f1_ovr_c = f1_v && !f1_flt &&
                         (p1_tk != f1_ubhit || (p1_tk && (p1_slot != f1_ubslot || p1_npc != f1_npc)));
  logic       f1_ovr;                 // 再去掉被更老的冲刷盖掉的情况 (见 F3 之后)
  // F1 送往 F2 的预测 (改道后用 p1)
  wire        f1_ubhit_e = f1_ovr_c ? p1_tk : f1_ubhit;
  wire [3:0]  f1_ubslot_e = f1_ovr_c ? p1_slot : f1_ubslot;
  wire [63:0] f1_npc_e = f1_ovr_c ? p1_npc : f1_npc;
  wire        f1_ppush_e = f1_ovr_c ? p1_call : f1_ppush;
  wire        f1_ppop_e  = f1_ovr_c ? p1_ret  : f1_ppop;

  //==========================================================================
  // F2：并行预译码 (只做与预测无关、彼此独立的计算，结果打进 F3)
  //   位置 p = 0：上一块留下的跨块 RVI (hwm)；p = j+1：块内第 j 个半字
  //==========================================================================
  // 控制流类型 (按原始编码直接识别，不经过 RVC 展开)
  typedef struct packed {
    logic        jal, br, jalr, call, ret, rvc;
    logic [21:0] imm;               // 有符号字节偏移 (JAL / 条件分支)
  } cf_t;
  function automatic cf_t cf_dec(input logic [31:0] x);
    cf_t r;
    r = '0;
    r.rvc = x[1:0] != 2'b11;
    if (x[1:0] == 2'b01 && x[15:13] == 3'b101) begin                       // C.J
      r.jal = 1'b1;
      r.imm = {{10{x[12]}}, x[12], x[8], x[10:9], x[6], x[7], x[2], x[11], x[5:3], 1'b0};
    end
    if (x[1:0] == 2'b01 && x[15:14] == 2'b11) begin                        // C.BEQZ / C.BNEZ
      r.br  = 1'b1;
      r.imm = {{13{x[12]}}, x[12], x[6:5], x[2], x[11:10], x[4:3], 1'b0};
    end
    if (x[1:0] == 2'b10 && x[15:13] == 3'b100 && x[6:2] == 5'd0 && x[11:7] != 5'd0) begin   // C.JR / C.JALR
      r.jalr = 1'b1;
      r.call = x[12];
      r.ret  = !x[12] && (x[11:7] == 5'd1 || x[11:7] == 5'd5);
    end
    if (x[1:0] == 2'b11) begin
      r.jal  = x[6:0] == OPC_JAL;
      r.br   = x[6:0] == OPC_BRANCH;
      r.jalr = x[6:0] == OPC_JALR && x[14:12] == 3'b000;
      r.call = (r.jal || r.jalr) && (x[11:7] == 5'd1 || x[11:7] == 5'd5);
      r.ret  = r.jalr && x[11:7] == 5'd0 && (x[19:15] == 5'd1 || x[19:15] == 5'd5);
      r.imm  = r.jal ? {x[31], x[31], x[19:12], x[20], x[30:21], 1'b0}
                     : {{9{x[31]}}, x[31], x[7], x[30:25], x[11:8], 1'b0};
    end
    return r;
  endfunction
  // a + b == c 的无进位判断 (逐位推出所需进位，与实际进位比较)
  function automatic logic add_eq(input logic [63:0] a, input logic [63:0] b, input logic [63:0] c);
    logic [63:0] k;
    k = (a & b) | ((a ^ b) & ~c);
    return (a ^ b ^ c) == {k[62:0], 1'b0};
  endfunction
  function automatic logic [4:0] popc16(input logic [15:0] x);
    logic [1:0] a [8];
    logic [2:0] b [4];
    logic [3:0] c [2];
    for (int i = 0; i < 8; i++) a[i] = 2'(x[2*i]) + 2'(x[2*i+1]);
    for (int i = 0; i < 4; i++) b[i] = 3'(a[2*i]) + 3'(a[2*i+1]);
    for (int i = 0; i < 2; i++) c[i] = 4'(b[2*i]) + 4'(b[2*i+1]);
    return 5'(c[0]) + 5'(c[1]);
  endfunction

  localparam int NP = 17;            // 位置数
  wire [63:0] base2 = blk_base(f2_pc);

  // F3 寄存器 (F2 的前递要用，先声明)
  logic        f3_v, f3_flt;
  logic [5:0]  f3_cause;
  logic [63:0] f3_pc;
  logic        f3_ubhit;
  logic [3:0]  f3_ubslot;
  logic [63:0] f3_npc;
  logic [31:0] f3_ctr;
  logic [7:0]  f3_row;
  bp_fold_t         f3_fold;
  logic [BP_KW-1:0] f3_hcnt;
  bp_slot_t [15:0]  f3_bps;            // TAGE 每槽结果 (F2 算好)
  logic [BP_NT-1:0][1:0][2:0] f3_uv;   // 预测时各表两路 {v, u}
  bp_slot_t [15:0]  t_f2slot;          // o6_tage 的 F2 输出
  bp_slot_t [15:0]  s_f2slot;          // 经 o6_sc 修正后的 F2 输出
  logic             it_f2v, f3_itv;    // ITTAGE 有可用目标 (F2 / F3)
  logic [38:1]      it_f2tgt, f3_ittgt;
  it_info_t         it_f2info, f3_itinfo;
  logic [BP_NT-1:0][1:0][2:0] t_f2uv;
  logic        f3_hwm;               // 本块第一条是跨块 RVI
  logic [63:0] f3_hwpc;
  logic        f3_hwc;               // 本块最后半字是跨块 RVI 的前半 (候选，F3 再看是否跳走)
  logic [15:0] f3_h15;
  logic [31:0]          f3_praw [NP];  // 各位置的原始编码 (RVC 高半为 0)
  logic [31:0]          f3_pexp [NP];  // 各位置 RVC 展开后的指令 (F2 并行展开)
  logic [NP-1:0]        f3_pill;       // RVC 展开发现非法
  logic [FTQ_OFF_W-1:0] f3_poff [NP];  // 相对本组第一条的半字偏移
  logic [4:0]           f3_nall;
  logic [NP-1:0]        f3_pv;     // 位置是有效指令起点
  cf_t                  f3_cf  [NP];
  logic [NP-1:0]        f3_eq;     // 该位置直接目标 == F0 预测的下一块 PC
  logic [4:0]           f3_pk  [NP];   // 该位置的指令在本组里的序号
  logic [NI-1:0]        f3_pklt [NP];  // [p][i]：位置 p 的序号 < i (p 处跳转则槽 i 无效)
  logic [NI-1:0]        f3_pkeq [NP];  // [p][i]：位置 p 的序号 == i
  logic [63:0]          f3_rtop;       // 本块预测前的栈顶 (F2 读出)
  // 直接目标拆成"低 22 位和 + 对高位的调整 (-1/0/+1)"，在 F2 按位置算好；F3 只做选择不做加法
  logic [21:0]          f3_tlo  [NP];
  logic [1:0]           f3_tadj [NP];   // 01 = +1，11 = -1，00 = 0
  logic [63:22]         f3_bhp, f3_bhm; // 块基址高位 +1 / -1
  logic [63:5]          f3_b32;         // 块基址 + 32 (顺序下一块；返回地址跨块时的高位)
  logic                 f3_raseq;      // 栈顶 == F0 预测的下一块 PC (F2 比较)
  logic [15:0]          f3_sw;         // store-wait 表的本块一行

  // 跨块 RVI 来源：F3 里的前一块 (F2 只在 F3 同拍发出时才前进，所以前递它的候选即可：
  // 前一块若被 F0 预测跳走则不算；F3 的最终预测与 F0 不同时 F2 会被冲掉) 或 hw 寄存器
  wire        hwm = (f3_v ? (f3_hwc && !f3_ubhit) : hw_v) && f2_pc[4:0] == 5'd0;
  wire [15:0] hwd = f3_v ? f3_h15 : hw_d;
  wire [63:0] hwp = f3_v ? {f3_pc[63:5], 4'd15, 1'b0} : hw_pc;

  logic [15:0]   d_h [16];
  logic [15:0]   d_rvc, d_st, d_sv;
  logic [4:0]    d_idx [16];
  logic          d_hwc;
  logic [31:0]   d_praw [NP];
  logic [31:0]   d_pexp [NP];
  logic [NP-1:0] d_pill;
  logic [FTQ_OFF_W-1:0] d_poff [NP];
  logic [NP-1:0] d_pv, d_eq;
  cf_t           d_cf [NP];
  logic [4:0]    d_pk [NP];
  logic [21:0]   d_tlo  [NP];
  logic [1:0]    d_tadj [NP];

  // 指令起点：s[j+1] = !s[j] | rvc[j] (不是起点的半字是 RVI 后半，下一个必是起点)，
  // 写成 {常量 / 取反} 函数的复合，用 4 级并行前缀算出 (不随半字数串行)
  // hwm (是否有跨块 RVI) 来自 F3 / hw 寄存器的选择，若放在起点链头部会串长整条链：
  // 两种情况 (v=0：起点 = f2_pc[4:1]；v=1：起点 = 1、序号从 1 起) 各算一遍，最后按 hwm 二选一
  logic [15:0] v_st [2], v_sv [2];
  logic [4:0]  v_idx [2][16];
  always_comb begin
    for (int j = 0; j < 16; j++) begin
      d_h[j]   = f2_data[16*j +: 16];
      d_rvc[j] = d_h[j][1:0] != 2'b11;
    end
    for (int v = 0; v < 2; v++) begin
      logic [3:0]  p0;
      logic [15:0] ec, ev, nc, nv;
      p0 = (v == 1) ? 4'd1 : f2_pc[4:1];
      for (int j = 0; j < 16; j++) begin
        if (4'(j) < p0)       begin ec[j] = 1'b1; ev[j] = 1'b0; end
        else if (4'(j) == p0) begin ec[j] = 1'b1; ev[j] = 1'b1; end
        else                  begin ec[j] = d_rvc[(j > 0) ? j - 1 : 0]; ev[j] = 1'b1; end
      end
      for (int l = 0; l < 4; l++) begin
        nc = ec; nv = ev;
        for (int j = (1 << l); j < 16; j++)
          if (!ec[j]) begin
            nc[j] = ec[j - (1 << l)];
            nv[j] = ev[j - (1 << l)] ^ ev[j];
          end
        ec = nc; ev = nv;
      end
      v_st[v] = ev;
      for (int j = 0; j < 16; j++) v_sv[v][j] = ev[j] && (d_rvc[j] || j < 15);
      for (int j = 0; j < 16; j++) v_idx[v][j] = 5'(v) + popc16(v_sv[v] & 16'((32'd1 << j) - 1));
    end
    d_st = hwm ? v_st[1] : v_st[0];
    d_sv = hwm ? v_sv[1] : v_sv[0];
    for (int j = 0; j < 16; j++) d_idx[j] = hwm ? v_idx[1][j] : v_idx[0][j];
    d_hwc = d_st[15] && !d_rvc[15] && !f2_flt;
  end

  // 序号比较矩阵也两种各算一遍再选 (F3 用)
  logic [NI-1:0] v_pklt [2][NP], v_pkeq [2][NP];
  always_comb begin
    for (int v = 0; v < 2; v++)
      for (int q = 0; q < NP; q++) begin
        logic [4:0] pk;
        pk = (q == 0) ? 5'd0 : v_idx[v][q-1];
        for (int i = 0; i < NI; i++) begin v_pklt[v][q][i] = pk < 5'(i); v_pkeq[v][q][i] = pk == 5'(i); end
      end
  end

  // 各位置并行做 RVC 展开 (与起点 / 序号计算并行，不在它们之后)；压缩到槽在 F3 用寄存的"序号 == i"做与或
  for (genvar q = 0; q < NP; q++) begin : g_exp
    o6_rvc_expand u_x (.inst_i(d_praw[q]), .inst_o(d_pexp[q]), .ill_o(d_pill[q]));
  end

  // 每个位置的控制流类型、直接目标是否等于 F0 的预测 (各位置并行)
  always_comb begin
    for (int p = 0; p < NP; p++) begin
      logic [31:0] x;
      if (p == 0) x = {d_h[0], hwd};
      else        x = d_rvc[p-1] ? {16'b0, d_h[p-1]} : {d_h[(p < 16) ? p : 15], d_h[p-1]};
      d_praw[p] = x;
      d_poff[p] = (p == 0) ? '0 : hwm ? FTQ_OFF_W'(p) : FTQ_OFF_W'(5'(p - 1) - 5'(f2_pc[4:1]));
      d_cf[p] = cf_dec(x);
      begin
        logic [21:0] off;
        logic [23:0] sum;
        off = d_cf[p].imm + ((p == 0) ? -22'sd2 : 22'(2 * (p - 1)));
        sum = {2'b00, base2[21:0]} + {{2{off[21]}}, off};
        d_tlo[p]  = sum[21:0];
        d_tadj[p] = sum[23:22];
      end
      if (p == 0) d_cf[p].rvc = 1'b0;
      d_pv[p] = (p == 0) ? hwm : d_sv[p-1];
      d_pk[p] = (p == 0) ? 5'd0 : d_idx[p-1];
      // 目标 = 指令 PC + 立即数；指令 PC 是拼接 (p = 0 的跨块 RVI 用它自己的 PC)，与 F0 预测做无进位比较
      d_eq[p] = add_eq((p == 0) ? hwp : {base2[63:5], 4'(p - 1), 1'b0}, {{42{d_cf[p].imm[21]}}, d_cf[p].imm}, f2_npc);
    end
  end

  //==========================================================================
  // F3：最终预测 (JAL 必跳；条件分支看方向；返回用 RAS；其它 JALR 用 uBTB 目标)，
  //     与 F0 的 uBTB 预测比较，写取指缓冲与 FTQ
  //==========================================================================
  wire [63:0] base3 = blk_base(f3_pc);
  wire [63:0] it_tgt3 = tg_x(f3_ittgt, f3_pc);   // ITTAGE 目标 (高位取块 PC)


  logic [NP-1:0]  p_tk, p_first, p_retc, p_ujr;
  logic [11:0]    p_pch [NP];        // 各位置的 PC[12:1] (g_start + 位置偏移)
  logic           g_taken;
  logic           g_eq;              // 最终目标 == F0 的预测
  logic [4:0]     g_kt;              // 跳转指令在本组的序号
  logic [63:0]    g_npc;
  logic [3:0]     g_tslot;           // 跳转指令的块内槽号 (半字号)
  logic           g_tret;
  logic           g_tcond;           // 跳转指令是条件分支
  logic           g_push, g_pop;
  logic [63:0]    g_push_val;
  logic [63:0]    g_start;
  logic [NI-1:0]  g_v;
  fetch_inst_t    g_i [NI];
  logic [4:0]     g_n;
  wire            ras_eq = f3_raseq;
  logic [6:0]     g_lk;

  always_comb begin
    logic [NP-1:0] p_bef;
    logic [21:0]   tlo;
    logic [1:0]    tadj;
    logic [63:0]   dtgt;
    logic [6:0]    lk;
    logic          is_ret, is_ujr;
    for (int p = 0; p < NP; p++) begin
      logic [3:0] sl;
      logic       dir;
      sl  = (p == 0) ? 4'd15 : 4'(p - 1);   // 跨块 RVI 记 15 (只影响预测精度)
      dir = f3_bps[sl].fdir;              // TAGE 最终方向 (无提供者时即 bimodal)
      p_retc[p] = f3_cf[p].ret && f3_rcnt != 6'd0;
      // 非返回的 JALR：ITTAGE 有目标就用它 (按块预测：块里第一条这样的 JALR)；否则 uBTB 在该槽命中时用 uBTB 的
      p_ujr[p]  = f3_cf[p].jalr && !p_retc[p] && !f3_hwm && (f3_itv || (f3_ubhit && f3_ubslot == sl));
      p_tk[p]   = f3_pv[p] && (f3_cf[p].jal || (f3_cf[p].br && dir) || p_retc[p] || p_ujr[p]);
    end
    p_bef[0] = 1'b0;
    for (int p = 1; p < NP; p++) p_bef[p] = p_bef[p-1] || p_tk[p-1];
    p_first = p_tk & ~p_bef;
    g_taken = |p_tk;
    g_eq = 1'b0; g_kt = '0; tlo = '0; tadj = '0; lk = '0; g_tslot = '0; is_ret = 1'b0; is_ujr = 1'b0; g_push = 1'b0;
    g_tcond = 1'b0;
    for (int p = 0; p < NP; p++)
      if (p_first[p]) begin
        g_eq    = (f3_cf[p].jal || f3_cf[p].br) ? f3_eq[p] : p_retc[p] ? ras_eq :
                  f3_itv ? (it_tgt3 == f3_npc) : 1'b1;
        g_kt    = f3_pk[p];
        tlo     = f3_tlo[p];
        tadj    = f3_tadj[p];
        lk      = (p == 0) ? 7'sd2 : 7'(2 * (p - 1) + (f3_cf[p].rvc ? 2 : 4));
        g_tslot = (p == 0) ? 4'd15 : 4'(p - 1);
        is_ret  = p_retc[p];
        is_ujr  = p_ujr[p];
        g_push  = f3_cf[p].call;
        g_tcond = f3_cf[p].br;
      end
    g_tret = is_ret;
    g_pop  = is_ret;
    dtgt   = {(tadj == 2'b01) ? f3_bhp : (tadj == 2'b11) ? f3_bhm : base3[63:22], tlo};
    g_npc  = !g_taken ? {f3_b32, 5'b0} : is_ret ? f3_rtop : is_ujr ? (f3_itv ? it_tgt3 : f3_npc) : dtgt;
    g_lk   = lk;
    // 返回地址 = 块基址 + 2..34 (lk 非负、< 64)：只看 lk[5] 选基址或基址 + 32
    g_push_val = {lk[5] ? f3_b32 : base3[63:5], lk[4:0]};
    g_n = g_taken ? g_kt + 5'd1 : f3_nall;
    g_start = f3_hwm ? f3_hwpc : f3_pc;

    // 各位置的 PC[12:1] 先加好 (2026-10-23，6 GHz)：原来 选位置 (f3_pv & f3_pkeq) -> 与或取 off -> 12 位加法 -> ib
    // (fe8_250 228 ps)；现在加法与选择并行，选择直接取和
    for (int p = 0; p < NP; p++) p_pch[p] = 12'(g_start[12:1] + 12'(f3_poff[p]));
    // 压缩到槽：槽 i 取"有效起点且序号 == i"的位置 (独热，与或)
    for (int i = 0; i < NI; i++) begin
      fetch_inst_t fi;
      logic [NP-1:0] clt, sel;
      logic [31:0]   raw, ex;
      logic          ill, ldw;
      logic [FTQ_OFF_W-1:0] off_i;
      logic [11:0]   pch_i;
      for (int p = 0; p < NP; p++) begin clt[p] = f3_pklt[p][i]; sel[p] = f3_pv[p] && f3_pkeq[p][i]; end
      raw = '0; ex = '0; ill = 1'b0; ldw = 1'b0; off_i = '0; pch_i = '0;
      for (int p = 0; p < NP; p++) begin
        raw   = raw   | ({32{sel[p]}} & f3_praw[p]);
        ex    = ex    | ({32{sel[p]}} & f3_pexp[p]);
        ill   = ill   | (sel[p] & f3_pill[p]);
        off_i = off_i | ({FTQ_OFF_W{sel[p]}} & f3_poff[p]);
        pch_i = pch_i | ({12{sel[p]}} & p_pch[p]);
        if (p > 0) ldw = ldw | (sel[p] & f3_sw[p-1]);    // 跨块 RVI (p = 0) 不预测
      end
      fi = '0;
      fi.inst = ex; fi.raw = raw;
      fi.is_rvc = raw[1:0] != 2'b11; fi.rvc_ill = fi.is_rvc && ill;
      fi.ftq_idx = ftq_tail.idx;
      fi.ftq_off = off_i;
      fi.ld_wait = ldw;
      fi.pch     = pch_i;
      fi.cls     = fe_cls(ex);
      fi.pred_taken = |(p_first & sel);
      g_v[i] = (|sel) && !(|(p_first & clt));
      g_i[i] = fi;
    end
    if (f3_flt) begin
      g_v = '0; g_v[0] = 1'b1; g_n = 5'd1;
      g_i[0] = '0;
      g_i[0].fetch_excp = 1'b1; g_i[0].fetch_cause = f3_cause;
      g_i[0].ftq_idx = ftq_tail.idx;
      g_start = f3_pc;
      if (f3_hwm) begin g_start = f3_hwpc; g_i[0].fetch_hi = 1'b1; end
      g_taken = 1'b0; g_push = 1'b0; g_pop = 1'b0;
    end
  end

  // 块内哪些槽是条件分支 (预译码，给 TAGE 训练挑候选)
  logic [15:0] f3_cmask;
  always_comb begin
    f3_cmask = '0;
    for (int p = 0; p < NP; p++)
      if (f3_pv[p] && f3_cf[p].br) f3_cmask[(p == 0) ? 4'd15 : 4'(p - 1)] = 1'b1;
  end

  wire keep_hw  = f3_hwc && !g_taken;
  // 只看寄存的 ib_cnt、按最多 NI 条保守判断
  wire room     = (6'(IBUF_SIZE) - ib_cnt) >= 6'(NI) && !ftq_full;
  wire f3_fire  = f3_v && room;
  // F0 的推测 (是否跳、跳到哪) 与最终预测不一致 -> 冲掉 F2/F1，F0 改从最终目标取
  wire mispred3 = f3_fire && !f3_flt && ((g_taken != f3_ubhit) || (g_taken && !g_eq) ||
                                         g_push != f3_ppush || g_pop != f3_ppop);
  // 后端误预测的那条分支实际是否跳转 (FTQ 项在它处截断，历史位取实际结果)
  logic       rd_taken;
  logic [3:0] rd_slot;
  always_comb begin
    rd_taken = 1'b0; rd_slot = '0;
    for (int k = 0; k < N_BRU; k++)
      if (bres_valid_i[k] && bres_mispred_i[k]) begin
        rd_taken = bres_taken_i[k];
        rd_slot  = ftq_slot0[bres_ftq_i[k]] + bres_off_i[k][3:0];
      end
  end

  //==========================================================================
  // 推进与 F0 发请求
  //==========================================================================
  wire         f2_move = f2_v && (f3_fire || !f3_v);                     // 本拍 F2 -> F3
  wire         f1_dv   = f1_v && (f1_hv || imem_resp_valid_i || f1_flt);   // F1 数据就绪
  wire [255:0] f1_data = f1_hv ? f1_hold : imem_data_i;
  wire         f1_move = f1_dv && (f2_move || !f2_v);                     // 本拍 F1 -> F2
  wire         f1_free = !f1_v || f1_move;
  // F3 预测不一致不组合地挡 F0：本拍照常发 (错路)，下拍用寄存的 kill 作废它
  logic mispred_q;
  // 分支重定向那一拍：F1 / F2 / F3 都被冲掉，F0 照常发 (用重定向 PC)；提交级冲刷那一拍不发
  wire f0_go     = (byp || (!f0_stop && f1_free && !redirect)) && !f0_umiss;
  assign imem_req_o  = f0_go && !f0_fflt;
  assign imem_addr_o = blk_base(pa0);
  wire f0_sent   = f0_go && (f0_fflt || imem_ready_i);
  // TAGE / ITTAGE 宏的读使能 (2026-10-20，6 GHz)：不等 f1_free / f0_umiss，只要 F0 可能发就读。
  // f0_go 依赖 F1 能否前进 -> F2 -> F3 是否停 (f3_hwm 起 41 级)，用它在宏地址口选"读 / 写队列"
  // 会把整条 F3 判定串到宏的地址建立时间上 (262 ps)。多读的那一拍结果不被取用 (F1 只在 f0_sent 时接收)；
  // 代价是 F0 停顿时写队列不能借空拍写读到的那一体 (TAGE 另一体照写，队列满时写照样抢读)。
`ifdef O6_FE_OLDRD
  wire f0_rde    = f0_go;                          // 对照实验：原读使能
`else
  wire f0_rde    = byp || (!f0_stop && !redirect);
`endif
  // PMP 拒绝的块：请求已发，下拍作废 (F1 带异常标记不等数据)
  logic pmpk_q;
  assign f1_ovr  = f1_ovr_c && !mispred3 && !redirect;
  assign imem_kill_o = redirect || mispred_q || pmpk_q;

  //==========================================================================
  // 状态更新
  //==========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pc0_q <= RESET_PC; ubs_q <= ub_set(RESET_PC[30:1]); upi_q <= up_idx(RESET_PC, '0); lps_q <= lp_idx(RESET_PC);
      f0_stop <= 1'b0; mispred_q <= 1'b0; pmpk_q <= 1'b0;
      f1_v <= 1'b0; f2_v <= 1'b0; f3_v <= 1'b0;
      hw_v <= 1'b0; f1_hv <= 1'b0; hw_d <= '0; hw_pc <= '0; hw_nb <= '0;
      ftq_head <= '0; ftq_tail <= '0; ftq_trn <= '0; ftq_trn_q <= '0;
      ib_head <= '0; ib_tail <= '0; ib_cnt <= '0;
    end else begin
      // 训练指针：每拍越过 trn_adv 项 (没有条件分支的项不训练，直接跳过)
      ftq_trn <= ftq_ptr_t'(ftq_trn + ftq_ptr_t'(trn_adv));
      ftq_trn_q <= ftq_trn;
      // F1 改道时本拍 F0 若已发出顺序下一块，下拍同样作废它
      mispred_q <= (mispred3 || (f1_ovr && f0_sent)) && !redirect;
      pmpk_q    <= f0_sent && !f0_fflt && f0_flt;
      // ---------------- F3 -> 取指缓冲 / FTQ
      if (f3_fire) begin
        hw_v <= keep_hw; hw_d <= f3_h15; hw_pc <= {base3[63:5], 4'd15, 1'b0}; hw_nb <= base3[63:5] + 59'd1;
        if (f3_flt) f0_stop <= 1'b1;
      end
      if (cm_ftq_v_i) ftq_head <= '{flag: (cm_ftq_idx_i < ftq_head.idx) ? ~ftq_head.flag : ftq_head.flag,
                                    idx: cm_ftq_idx_i};
      if (f3_fire && (|g_v))
        ftq_tail <= '{flag: ftq_tail.idx == FTQ_W'(FTQ_SIZE-1) ? ~ftq_tail.flag : ftq_tail.flag,
                      idx: ftq_tail.idx + 1'b1};
      begin
        logic [5:0] cnt;
        cnt = ib_cnt - 6'(ib_pop_i);
        ib_head <= ib_head + 5'(ib_pop_i);
        if (f3_fire) begin
          ib_tail <= ib_tail + 5'(g_n);
          cnt = cnt + 6'(g_n);
        end
        ib_cnt <= cnt;
      end

      // ---------------- 流水推进
      if (f3_fire || !f3_v) f3_v <= f2_v;
      if (f2_move || !f2_v) f2_v <= f1_dv;
      // F1 保持寄存器
      if (f1_move || !f1_v) f1_hv <= 1'b0;
      else if (f1_v && imem_resp_valid_i && !f1_hv) f1_hv <= 1'b1;
      // F0 -> F1
      if (f1_free) f1_v <= f0_sent;
      // F0 PC 与推测历史
      if (f0_sent) begin pc0_q <= f0_npc; ubs_q <= f0_nset; upi_q <= f0_nupi; lps_q <= f0_nlps; end

      // ---------------- F1 BTB 改道：F0 改从 BTB 目标取，本拍刚发出的顺序块作废
      if (f1_ovr) begin
        pc0_q <= p1_npc; ubs_q <= ub_set(p1_npc[30:1]); lps_q <= lp_idx(p1_npc);
        upi_q <= up_pc(p1_npc) ^ (p1_tk ? gh_push_lo(g_f1, h_f1) : up_gh(g_f1));
        if (f0_sent) f1_v <= 1'b0;
      end

      // ---------------- F3 预测不一致：冲掉 F2、F1，F0 从最终目标取
      if (mispred3) begin
        f1_v  <= 1'b0;
        f1_hv <= 1'b0;
        f2_v  <= 1'b0;
        f3_v  <= 1'b0;
        pc0_q <= g_npc; ubs_q <= ub_set(g_npc[30:1]); lps_q <= lp_idx(g_npc);
        upi_q <= up_pc(g_npc) ^ (g_taken ? gh_push_lo(g_m3, h_m3) : up_gh(g_m3));
      end

      // ---------------- 后端 / 提交重定向 (覆盖以上)
      if (redirect) begin
        f1_v <= f0_sent; f2_v <= 1'b0; f3_v <= 1'b0; hw_v <= 1'b0; f0_stop <= 1'b0; f1_hv <= 1'b0;
        pc0_q <= f0_sent ? f0_npc : fl_q ? rpc_q : be_redirect_pc_i;
        ubs_q <= f0_sent ? f0_nset : ub_set(fl_q ? rpc_q[30:1] : be_redirect_pc_i[30:1]);
        lps_q <= f0_sent ? f0_nlps : lp_idx(fl_q ? rpc_q : be_redirect_pc_i);
        // 重定向拍 F0 当顺序块 (不推入历史)：历史就是恢复值 rg_rd
        upi_q <= up_pc(f0_sent ? f0_npc : fl_q ? rpc_q : be_redirect_pc_i) ^ up_gh(rg_rd);
        ib_head <= '0; ib_tail <= '0; ib_cnt <= '0;
        if (fl_q) begin
          ftq_head <= '0; ftq_tail <= '0; ftq_trn <= '0; ftq_trn_q <= '0;
        end else begin
          logic [FTQ_W-1:0] ni;
          ni = be_redirect_ftq_i + 1'b1;
          ftq_tail <= '{flag: (ni > ftq_head.idx) ? ftq_head.flag : ~ftq_head.flag, idx: ni};
        end
      end
    end
  end

  //==========================================================================
  // RAS 更新：每拍至多一次写 (最高优先级的那个；被它冲掉的更年轻的压栈不必写)
  //==========================================================================
  logic [4:0]  rn_tos, rn_alloc;
  logic [5:0]  rn_cnt;
  logic        rw_en;
  logic [4:0]  rw_idx, rw_prev;
  logic [63:0] rw_addr;
  // 每个来源 (F0 预测 / F1 改道 / F3 纠正 / 后端·提交重定向) 各自并行算出"恢复 + 弹/压"后的状态，
  // 最后按优先级四选一。弹栈用各级带着的快照下一层 (rprev)，选源之后不再查 rs_prev
  typedef struct packed {
    logic [4:0]  t, a;
    logic [5:0]  c;
    logic        wen;
    logic [4:0]  widx, wprev;
    logic [63:0] waddr;
  } ras_nx_t;
  function automatic ras_nx_t ras_step(input logic [4:0] t, input logic [4:0] a, input logic [5:0] c,
                                       input logic [4:0] prev, input logic ps, input logic pp,
                                       input logic [63:0] lv);
    ras_nx_t r;
    // 先弹后压 (前端解码时 call 已排除 ret；后端的 call+ret 同样先弹后压)
    r.t = pp ? prev : t;
    r.c = (pp && c != 6'd0) ? c - 1'b1 : c;
    r.wen = ps; r.widx = a; r.wprev = r.t; r.waddr = lv; r.a = a;
    if (ps) begin
      r.t = a; r.a = a + 1'b1;
      if (r.c != 6'(NRAS)) r.c = r.c + 1'b1;
    end
    return r;
  endfunction

  ras_nx_t rx_f0, rx_f1, rx_f3, rx_rd, rx;
  always_comb begin
    rx_f0 = ras_step(rs_tos, rs_alloc, ras_cnt, rs_prev[rs_tos], f0_sent && f0_push, f0_sent && f0_pop, f0_link);
    rx_f1 = ras_step(f1_rtos, f1_ralloc, f1_rcnt, f1_rprev, p1_call, p1_ret,
                     blk_base(f1_pc) + {{57{p1_lk[6]}}, p1_lk});
    rx_f3 = ras_step(f3_rtos, f3_ralloc, f3_rcnt, f3_rprev, g_push, g_pop, g_push_val);
    // 提交冲刷：回到该块之前的状态 (块内压/弹只发生在块尾的跳转处，而它在冲刷指令之后或就是不会改 RAS 的指令)
    // 后端重定向：恢复到误预测分支所在块之前，再补做它本身的调用 / 返回
    if (fl_q)
      rx_rd = ras_step(ftq_rtos[fftq_q], ftq_ralloc[fftq_q], ftq_rcnt[fftq_q],
                       ftq_rprev[fftq_q], 1'b0, 1'b0, '0);
    else
      rx_rd = ras_step(ftq_rtos[be_redirect_ftq_i], ftq_ralloc[be_redirect_ftq_i], ftq_rcnt[be_redirect_ftq_i],
                       ftq_rprev[be_redirect_ftq_i], be_redirect_call_i, be_redirect_ret_i, be_redirect_link_i);
    rx = redirect ? rx_rd : mispred3 ? rx_f3 : f1_ovr ? rx_f1 : rx_f0;
    rn_tos = rx.t; rn_alloc = rx.a; rn_cnt = rx.c;
    rw_en = rx.wen; rw_idx = rx.widx; rw_prev = rx.wprev; rw_addr = rx.waddr;
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rs_tos <= '0; rs_alloc <= '0; ras_cnt <= '0;
      for (int i = 0; i < NRAS; i++) begin rs_addr[i] <= '0; rs_prev[i] <= '0; end
    end else begin
      rs_tos <= rn_tos; rs_alloc <= rn_alloc; ras_cnt <= rn_cnt;
      if (rw_en) begin rs_addr[rw_idx] <= rw_addr; rs_prev[rw_idx] <= rw_prev; end
    end
  end

  // 全局历史更新：各来源先算"恢复到哪个快照 (推入次数差 k、折叠快照) + 是否推入、推入什么"，
  // 按优先级选一个，再共用一个右移器与推入逻辑：
  //   后端重定向：回到误预测分支所在块之前，实际跳了就推入它；提交冲刷：回到冲刷指令所在块之前
  //   F3 纠正：回到 F3 块之前，最终预测跳了就推入；F1 改道：回到 F1 块之前，推入 BTB 的跳转
  //   F0：uBTB 预测跳就推入
  logic                 hr_upd, hr_push;
  logic [BP_KW-1:0]     hr_k;
  bp_fold_t             hr_f;
  logic [1:0]           hr_h;
  logic [BP_GHL-1:0]    hr_g;            // 恢复后的原始历史
  // 恢复 (+ 推入) 之后的历史：重定向那一拍 F0 用它预测，F0 若再预测跳则在它之上再推一次
  logic [BP_GHL-1:0]    rg;
  bp_fold_t             rf, foldx;
  bp_fold_t             rf_be;           // 重定向拍 F0 用的折叠 (见下)
  logic [BP_KW-1:0]     rk, hcntx;
  logic [BP_KW-1:0]     rk_rd;           // 重定向后的推入计数 (见下：各恢复情况的候选)
  // 恢复点历史的移出位 (2026-10-20，6 GHz)：后端重定向 / F3 纠正直接取块入 FTQ / F3 时记下的值，
  // 原来要 hcnt0 - ftq_hcnt 再把 gh0 桶形右移才拿到，这条从 be_redirect_i 到 TAGE 宏地址口 290 ps
  logic [BP_NT-1:0][3:0] hr_ob, f3_ob;
  logic [BP_NT-1:0][1:0] ob1, ob2;      // 第一次推入 / 重定向拍 F0 再推一次 的移出位
  always_comb begin
    for (int t = 0; t < BP_NT; t++) begin
      ob1[t] = hr_ob[t][1:0];
      ob2[t] = hr_push ? hr_ob[t][3:2] : hr_ob[t][1:0];
    end
    rg = hr_push ? {hr_g[BP_GHL-3:0], hr_h[0], hr_h[1]} : hr_g;
    rf = hr_push ? fold_push_ob(hr_f, ob1, hr_h) : hr_f;
    rk = hcnt0 - hr_k + BP_KW'(hr_push);
    foldx = redirect ? rf_be : fold0;
    hcntx = redirect ? rk_rd : hcnt0;
  end
  always_comb begin
    hr_upd = 1'b0; hr_push = 1'b0; hr_k = '0; hr_f = fold0; hr_h = '0;
    if (redirect) begin
      hr_upd = 1'b1;
      if (fl_q) begin
        hr_k = hcnt0 - ftq_hcnt[fftq_q]; hr_f = ftq_fold[fftq_q];
      end else begin
        hr_k = hcnt0 - ftq_hcnt[be_redirect_ftq_i]; hr_f = ftq_fold[be_redirect_ftq_i];
        hr_push = rd_taken; hr_h = bp_phash(rd_slot, be_redirect_pc_i);
      end
    end else if (mispred3) begin
      hr_upd = 1'b1; hr_k = hcnt0 - f3_hcnt; hr_f = f3_fold;
      hr_push = g_taken; hr_h = bp_phash(g_tslot, g_npc);
    end else if (f1_ovr) begin
      hr_upd = 1'b1; hr_k = hcnt0 - f1_hcnt; hr_f = f1_fold;
      hr_push = p1_tk; hr_h = bp_phash(p1_slot, p1_npc);
    end
    hr_g = gh0 >> (2 * hr_k);
    hr_ob = redirect ? ftq_ob[fl_q ? fftq_q : be_redirect_ftq_i] : mispred3 ? f3_ob : hist_ob(hr_g);
  end
  // 重定向拍 F0 读表用的折叠单独算 (2026-10-21)：与 rf 在重定向时相同，但不经过 hr_* 的优先级选择。
  // 否则 F0 那一支 (pc0 -> uBTB 查找 -> f0_tk / f0_npc 散列) 在结构上也接到 foldx，STA 报出
  // be_redirect_pc_i -> pc0 -> uBTB -> 选择 -> TAGE 宏地址 241 ps (功能上是假路径)
  always_comb begin
    logic [FTQ_W-1:0]       ri;
    logic [BP_NT-1:0][1:0]  ob;
    ri = fl_q ? fftq_q : be_redirect_ftq_i;
    for (int t = 0; t < BP_NT; t++) ob[t] = ftq_ob[ri][t][1:0];
    rf_be = (!fl_q && rd_taken) ? fold_push_ob(ftq_fold[ri], ob, bp_phash(rd_slot, be_redirect_pc_i)) : ftq_fold[ri];
  end
  // F0 推入的折叠候选 (2026-10-23，6 GHz)：推入的 2 位只有 4 种，用 fold0 / gh0 的移出位并行算好，
  // F0 环里命中后只剩 4 选 1 (原来 uBTB 命中 -> 选目标 -> 散列 -> 折叠异或 -> hr_* 优先级选择 -> fold0，213 ps)。
  // F0 这一支也不再经过 hr_* 的公共选择 (hr_* 只管重定向 / F3 纠正 / F1 改道)
  bp_fold_t f0_fc [4];
  always_comb begin
    logic [BP_NT-1:0][3:0] ob4;
    logic [BP_NT-1:0][1:0] ob;
    ob4 = hist_ob(gh0);
    for (int t = 0; t < BP_NT; t++) ob[t] = ob4[t][1:0];
    for (int h = 0; h < 4; h++) f0_fc[h] = fold_push_ob(fold0, ob, 2'(h));
  end
  // F3 块预测时的历史 = 现在的 gh0 右移回它之后的推入次数 (寄存器到寄存器，入 FTQ 用)
  // (f3_ob 现在是随块寄存的，见 f1_ob；下面断言核对它等于右移恢复的值)

  // 各恢复情况的候选分别算好 (2026-10-25，6 GHz)：原来先按晚到的 mispred3 / f1_ovr 选出移位量 hr_k，
  // 再对 430 位 gh0 桶形右移、推入、折叠 (fe12：f3_ubslot -> mispred3 -> hr_k -> 移位 -> gh0 196 ps)。
  // 移位量只取决于寄存器，各情况各自先移好；推入的 2 位只有 4 种，折叠也按 4 种预算；最后按情况多选一。
  // 与 hr_* 的原写法逐拍一致 (下面断言)
  // (g_m3 / g_f1 / rg_rd / h_m3 / h_f1 / push_rd 的声明在 F0 一节：写 pc0_q 时预算 upi_q 要用)
  bp_fold_t              fc_m3 [4], fc_f1 [4];
  logic [1:0]            h_rd;
  always_comb begin
    logic [BP_NT-1:0][1:0] o3, o1;
    logic [FTQ_W-1:0]      ri;
    g_m3 = gh0 >> (2 * BP_KW'(hcnt0 - f3_hcnt));
    g_f1 = gh0 >> (2 * BP_KW'(hcnt0 - f1_hcnt));
    for (int t = 0; t < BP_NT; t++) begin o3[t] = f3_ob[t][1:0]; o1[t] = f1_ob[t][1:0]; end
    for (int h = 0; h < 4; h++) begin
      fc_m3[h] = fold_push_ob(f3_fold, o3, 2'(h));
      fc_f1[h] = fold_push_ob(f1_fold, o1, 2'(h));
    end
    h_m3 = bp_phash(g_tslot, g_npc);
    h_f1 = bp_phash(p1_slot, p1_npc);
    // 后端重定向 / 提交冲刷
    ri      = fl_q ? fftq_q : be_redirect_ftq_i;
    push_rd = !fl_q && rd_taken;
    h_rd    = bp_phash(rd_slot, be_redirect_pc_i);
    g_rd    = gh0 >> (2 * BP_KW'(hcnt0 - ftq_hcnt[ri]));
    rg_rd   = push_rd ? {g_rd[BP_GHL-3:0], h_rd[0], h_rd[1]} : g_rd;
    rk_rd   = ftq_hcnt[ri] + BP_KW'(push_rd);
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni && hr_upd && $test$plusargs("showhist"))
      $display("[hist %0t] rd=%0d cm=%0d m3=%0d f1=%0d f0=%0d k=%0d push=%0d h=%0d hcnt0=%0d", $time, redirect, fl_q,
               mispred3, f1_ovr, f0_sent && f0_tk, hr_k, hr_push, hr_h, hcnt0);
`endif
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      gh0_r <= '0; gh_pend <= 1'b0; gh_h <= '0; hcnt0 <= '0; fold0 <= '0;
    end else if (redirect) begin
      // 先推 h[0] 后推 h[1]：最新的 h[1] 在第 0 位
      gh0_r <= rg_rd; gh_pend <= 1'b0;
      fold0 <= rf_be;
      hcnt0 <= rk_rd;
    end else if (mispred3) begin
      gh0_r <= g_taken ? {g_m3[BP_GHL-3:0], h_m3[0], h_m3[1]} : g_m3; gh_pend <= 1'b0;
      fold0 <= g_taken ? fc_m3[h_m3] : f3_fold;
      hcnt0 <= f3_hcnt + BP_KW'(g_taken);
    end else if (f1_ovr) begin
      gh0_r <= p1_tk ? {g_f1[BP_GHL-3:0], h_f1[0], h_f1[1]} : g_f1; gh_pend <= 1'b0;
      fold0 <= p1_tk ? fc_f1[h_f1] : f1_fold;
      hcnt0 <= f1_hcnt + BP_KW'(p1_tk);
    end else begin
      gh0_r <= gh0;                                   // 并进上一拍的待推入 (gh_pend 是寄存器，早到)
      gh_pend <= f0_sent && f0_tk;
      gh_h    <= f0_h;
      if (f0_sent && f0_tk) begin
        // F0 预测跳：折叠的 4 种结果已并行算好，这里只做 4 选 1；原始历史的推入记在 gh_pend / gh_h
        fold0 <= f0_fc[f0_h];
        hcnt0 <= hcnt0 + 1'b1;
      end
    end
  end

`ifndef SYNTHESIS
  // 折叠寄存器必须始终等于原始历史的全量折叠；恢复的右移量不能超过余量
  bp_fold_t fold_chk;
  assign fold_chk = fold_full(gh0);
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      assert (fold0 == fold_chk)
        else $error("o6_frontend: 折叠历史与原始历史不一致 hcnt0=%0d fi0=%h 全量=%h gh0[15:0]=%h",
                    hcnt0, fold0.fi[0], fold_chk.fi[0], gh0[15:0]);
      assert (!redirect || rf_be == rf) else $error("o6_frontend: 重定向折叠 rf_be 与 rf 不一致");
      assert (!(redirect && f0_sent && f0_tk)) else $error("o6_frontend: 重定向拍 F0 不应预测跳");
      assert (ubs_q == ub_set(pc0_q[30:1])) else $error("o6_frontend: 寄存的 uBTB 组号 ubs_q 与 pc0_q 不符");
      assert (upi_q == up_idx(pc0_q, gh0)) else $error("o6_frontend: 寄存的 uPHT 索引 upi_q 与 up_idx(pc0_q, gh0) 不符");
      if (f3_v) assert (f3_ob == hist_ob(gh0 >> (2 * BP_KW'(hcnt0 - f3_hcnt)))) else $error("o6_frontend: 随块寄存的 f3_ob 与右移恢复的不符");
      if (f1_v) assert (f1_ob == hist_ob(gh0 >> (2 * BP_KW'(hcnt0 - f1_hcnt)))) else $error("o6_frontend: 随块寄存的 f1_ob 与右移恢复的不符");
      // 各情况候选与 hr_* 原写法一致
      if (hr_upd) begin
        logic [BP_GHL-1:0] ng; bp_fold_t nf; logic [BP_KW-1:0] nk;
        if (redirect)      begin ng = rg_rd; nf = rf_be; nk = rk_rd; end
        else if (mispred3) begin ng = g_taken ? {g_m3[BP_GHL-3:0], h_m3[0], h_m3[1]} : g_m3;
                                 nf = g_taken ? fc_m3[h_m3] : f3_fold; nk = f3_hcnt + BP_KW'(g_taken); end
        else               begin ng = p1_tk ? {g_f1[BP_GHL-3:0], h_f1[0], h_f1[1]} : g_f1;
                                 nf = p1_tk ? fc_f1[h_f1] : f1_fold; nk = f1_hcnt + BP_KW'(p1_tk); end
        assert (ng == rg && nf == rf && nk == rk) else $error("o6_frontend: 历史恢复候选与原写法不一致 rd=%0d m3=%0d", redirect, mispred3);
      end
      assert (!f0_tk || f0_h == bp_phash(lp_slot[lps_q], f0_npc)) else $error("o6_frontend: LP 预存的路径散列 f0_h 与目标散列不一致");
      assert (lps_q == lp_idx(pc0_q)) else $error("o6_frontend: 寄存的 LP 索引 lps_q 与 pc0_q 不符");
      if (f0_tk && !lp_hret) assert (f0_nset == ub_set(f0_npc[30:1]) && f0_nlps == lp_idx(f0_npc) &&
                                     f0_nupi == (up_pc(f0_npc) ^ gh_push_lo(gh0, f0_h)))
        else $error("o6_frontend: LP 预存的目标组号 / 索引与目标不符");
      assert (!hr_upd || hr_ob == hist_ob(hr_g))
        else $error("o6_frontend: 记下的历史移出位与右移恢复的不一致 rd=%0d m3=%0d", redirect, mispred3);
      assert (!hr_upd || 2 * int'(hr_k) <= BP_GHL - BP_HMAX)
        else $error("o6_frontend: 历史恢复右移 %0d 超过余量", 2 * int'(hr_k));
    end

  // 跨块 RVI 的前递/寄存来源必须正好是本块的上一块
  always_ff @(posedge clk_i)
    if (rst_ni && f2_move && hwm)
      assert (f3_v ? (f3_pc[63:5] + 59'd1 == f2_pc[63:5]) : (hw_nb == f2_pc[63:5]))
        else $error("o6_frontend: 跨块 RVI 来源与本块不相邻 f2_pc=%h", f2_pc);
`endif

  // F1 / F2 / F3 载荷
  always_ff @(posedge clk_i) begin
    if ((f1_free || redirect) && f0_sent) begin
      f1_pc <= pc0; f1_flt <= f0_flt; f1_cause <= f0_cause; f1_gx <= f0_gx;
      f1_ubhit <= f0_ok; f1_ubslot <= lp_slot[lps_q]; f1_ubret <= lp_hret; f1_lph <= lp_hit && !byp;
      f1_uok <= ub_ok1; f1_uslot <= ub_hslot; f1_utgt <= ub_htgt; f1_uret <= ub_hret; f1_ucall <= ub_hcall; f1_ulk <= ub_hlk;
      f1_upi <= f0_upi;
      f1_npc <= f0_npc;
      f1_sw  <= swt[swt_row(pc0)];
      f1_bv <= bt_v[bt_idx(pc0)]; f1_btag <= bt_tag[bt_idx(pc0)]; f1_bslot <= bt_slot[bt_idx(pc0)];
      f1_btgt <= bt_tgt[bt_idx(pc0)]; f1_bret <= bt_ret[bt_idx(pc0)]; f1_bcond <= bt_cond[bt_idx(pc0)];
      f1_bcall <= bt_call[bt_idx(pc0)]; f1_blk <= bt_lk[bt_idx(pc0)];
      if (redirect) begin
        f1_rtos <= rx_rd.t; f1_ralloc <= rx_rd.a; f1_rcnt <= rx_rd.c;
        f1_rprev <= rx_rd.wen ? rx_rd.wprev : rs_prev[rx_rd.t];
      end else begin
        f1_rtos <= rs_tos; f1_ralloc <= rs_alloc; f1_rcnt <= ras_cnt; f1_rprev <= rs_prev[rs_tos];
      end
      f1_ppush <= f0_push; f1_ppop <= f0_pop;
      f1_ctr <= bim[bim_row(pc0)];
      f1_fold <= foldx; f1_hcnt <= hcntx; f1_ob <= hist_ob(redirect ? rg_rd : gh0);
    end
    // F1 改道但本拍没进 F2：原地改成 p1 (之后不再重复改道)
    if (f1_ovr && !f1_move) begin
      f1_ubhit <= p1_tk; f1_ubslot <= p1_slot; f1_npc <= p1_npc; f1_ubret <= p1_ret;
      f1_ppush <= p1_call; f1_ppop <= p1_ret;
    end
    if (f1_v && imem_resp_valid_i && !f1_hv) f1_hold <= imem_data_i;
    if (f2_move || !f2_v) begin
      f2_pc <= f1_pc; f2_flt <= f1_flt; f2_cause <= f1_cause; f2_gx <= f1_gx;
      f2_data <= f1_data;
      f2_ubhit <= f1_ubhit_e; f2_ubslot <= f1_ubslot_e; f2_npc <= f1_npc_e;
      f2_rtos <= f1_rtos; f2_ralloc <= f1_ralloc; f2_rcnt <= f1_rcnt; f2_rprev <= f1_rprev; f2_ppush <= f1_ppush_e; f2_ppop <= f1_ppop_e;
      f2_ctr <= f1_ctr; f2_row <= bim_row(f1_pc); f2_sw <= f1_sw; f2_upi <= f1_upi;
      f2_fold <= f1_fold; f2_hcnt <= f1_hcnt; f2_ob <= f1_ob;
    end
    if (f3_fire || !f3_v) begin
      f3_pc <= f2_pc; f3_flt <= f2_flt; f3_cause <= f2_cause; f3_gx <= f2_gx;
    end
    // F3 交出取指异常后停取 (f0_stop) 直到重定向：在途的取指异常只有一个，GPA 锁一份给提交级
    if (f3_fire && f3_flt) fgx_q <= f3_gx;
    if (f3_fire || !f3_v) begin
      f3_ubhit <= f2_ubhit; f3_ubslot <= f2_ubslot; f3_npc <= f2_npc;
      f3_rtos <= f2_rtos; f3_ralloc <= f2_ralloc; f3_rcnt <= f2_rcnt; f3_rprev <= f2_rprev; f3_ppush <= f2_ppush; f3_ppop <= f2_ppop;
      f3_ctr <= f2_ctr; f3_row <= f2_row; f3_sw <= f2_sw; f3_upi <= f2_upi;
      f3_fold <= f2_fold; f3_hcnt <= f2_hcnt; f3_ob <= f2_ob; f3_bps <= s_f2slot; f3_uv <= t_f2uv;
      f3_itv <= it_f2v; f3_ittgt <= it_f2tgt; f3_itinfo <= it_f2info;
      f3_hwm <= hwm; f3_hwpc <= hwp; f3_hwc <= d_hwc; f3_h15 <= d_h[15];
      f3_nall <= hwm ? 5'd1 + popc16(v_sv[1]) : popc16(v_sv[0]);
      f3_pill <= d_pill;
      for (int q = 0; q < NP; q++) begin
        f3_praw[q] <= d_praw[q]; f3_pexp[q] <= d_pexp[q]; f3_poff[q] <= d_poff[q];
      end
      f3_pv <= d_pv; f3_eq <= d_eq;
      for (int p = 0; p < NP; p++) begin
        f3_cf[p] <= d_cf[p]; f3_pk[p] <= d_pk[p];
        f3_pklt[p] <= hwm ? v_pklt[1][p] : v_pklt[0][p];
        f3_pkeq[p] <= hwm ? v_pkeq[1][p] : v_pkeq[0][p];
      end
      // 返回地址预测的栈顶与比较提前到 F2 (F2 -> F3 之间只有更年轻块的压栈，写的是空闲槽)
      f3_rtop  <= rs_addr[f2_rtos];
      for (int q = 0; q < NP; q++) begin f3_tlo[q] <= d_tlo[q]; f3_tadj[q] <= d_tadj[q]; end
      f3_bhp <= base2[63:22] + 42'd1;
      f3_bhm <= base2[63:22] - 42'd1;
      f3_b32 <= base2[63:5] + 59'd1;
      f3_raseq <= rs_addr[f2_rtos] == f2_npc;
    end
    // 取指缓冲与 FTQ 写入
    if (f3_fire) begin
      for (int i = 0; i < NI; i++)
        if (g_v[i]) ib[5'(ib_tail + i)] <= g_i[i];
      if (|g_v) begin
        ftq_pc[ftq_tail.idx]    <= g_start;
        ftq_tgt[ftq_tail.idx]   <= g_npc;
        ftq_rtos[ftq_tail.idx]   <= f3_rtos;
        ftq_rprev[ftq_tail.idx]  <= f3_rprev;
        ftq_ralloc[ftq_tail.idx] <= f3_ralloc;
        ftq_rcnt[ftq_tail.idx]   <= f3_rcnt;
        ftq_row[ftq_tail.idx]   <= f3_row;
        ftq_upi[ftq_tail.idx]   <= f3_upi;
        ftq_ctr[ftq_tail.idx]   <= f3_ctr;
        ftq_brm[ftq_tail.idx]   <= '0;
        ftq_slot0[ftq_tail.idx] <= g_start[4:1];
        ftq_hwm[ftq_tail.idx]   <= f3_hwm;
        ftq_fold[ftq_tail.idx]  <= f3_fold;
        ftq_hcnt[ftq_tail.idx]  <= f3_hcnt;
        ftq_ob[ftq_tail.idx]    <= f3_ob;
        ftq_uv[ftq_tail.idx]    <= f3_uv;
        ftq_sinfo[ftq_tail.idx] <= f3_bps;
        ftq_itinfo[ftq_tail.idx] <= f3_itinfo;
        ftq_itv[ftq_tail.idx]   <= 1'b0;
        ftq_cmask[ftq_tail.idx] <= f3_cmask;
      end
    end
    // 分支解析记录 (按块内绝对槽号)
    for (int k = 0; k < N_BRU; k++) begin
      if (bres_valid_i[k]) begin
        logic [3:0] sl;
        sl = ftq_slot0[bres_ftq_i[k]] + bres_off_i[k][3:0];
        ftq_brm[bres_ftq_i[k]][sl] <= 1'b1;
        ftq_brt[bres_ftq_i[k]][sl] <= bres_taken_i[k];
        if (bres_ind_i[k]) begin
          ftq_itv[bres_ftq_i[k]]  <= 1'b1;
          ftq_itgt[bres_ftq_i[k]] <= bres_target_i[k][38:1];
        end
      end
    end
  end

  //==========================================================================
  // 方向表训练：FTQ 头前移时，把被释放的那一项的解析结果写回
  //==========================================================================
  logic             tr_v;
  logic [31:0]      bim_nrow;          // 训练后的那一行
  // 训练项的寄存副本 (该项已释放，不会再被分支解析写)
  logic [7:0]     tr_row;
  logic [31:0]    tr_c;
  logic [15:0]    tr_brm, tr_brt;
  logic [63:0]    tr_pc, tr_pcr;
  logic           tr_hwm;
  // 跨块 RVI 的块训练用 pc + 2：加法放在寄存器之后 (2026-10-22，6 GHz：原来 训练指针 -> 选项 -> 64 选 1 读 ftq_pc
  // -> 64 位 +2 -> tr_pc 一拍 229 ps)；tr_pc 只进各表训练的索引 / 标签散列，散列结果还要再寄存，有余量
  assign tr_pc = tr_hwm ? tr_pcr + 64'd2 : tr_pcr;
  bp_fold_t       tr_fold;
  bp_slot_t [15:0] tr_sinfo;
  logic [15:0]    tr_cmask;
  logic [BP_NT-1:0][1:0][2:0] tr_uv;
  logic           tr_itv;
  logic [38:1]    tr_itgt, tr_ptgt;
  it_info_t       tr_itinfo;
  logic [FTQ_W-1:0] tq_sel;
  logic             tq_v;
  always_ff @(posedge clk_i) tq_sel <= trn_sel;
  always_ff @(posedge clk_i) begin
    tr_itv <= ftq_itv[tq_sel]; tr_itgt <= ftq_itgt[tq_sel]; tr_itinfo <= ftq_itinfo[tq_sel];
    tr_ptgt <= ftq_tgt[tq_sel][38:1];
    tr_row <= ftq_row[tq_sel]; tr_c <= ftq_ctr[tq_sel];
    tr_brm <= ftq_brm[tq_sel]; tr_brt <= ftq_brt[tq_sel];
    tr_pcr <= ftq_pc[tq_sel]; tr_hwm <= ftq_hwm[tq_sel];
    tr_fold <= ftq_fold[tq_sel];
    tr_uv  <= ftq_uv[tq_sel];
    tr_sinfo <= ftq_sinfo[tq_sel]; tr_cmask <= ftq_cmask[tq_sel];
  end
  always_comb begin
    bim_nrow = tr_c;
    for (int sl = 0; sl < 16; sl++)
      if (tr_brm[sl]) begin
        if (tr_brt[sl]) bim_nrow[2*sl +: 2] = (bim_nrow[2*sl +: 2] == 2'b11) ? 2'b11 : bim_nrow[2*sl +: 2] + 1'b1;
        else                     bim_nrow[2*sl +: 2] = (bim_nrow[2*sl +: 2] == 2'b00) ? 2'b00 : bim_nrow[2*sl +: 2] - 1'b1;
      end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tr_v <= 1'b0; tq_v <= 1'b0;
      for (int r = 0; r < BIM_R; r++) bim[r] <= {16{2'b01}};   // 弱不跳
    end else begin
      // 训练指针追赶头指针，每拍训练一项 (头一拍越过多项时不再丢训练)
      tq_v   <= trn_selv && !fl_q;
      tr_v   <= tq_v && !fl_q;
      if (tr_v) bim[tr_row] <= bim_nrow;
    end
  end

  //==========================================================================
  // store-wait 表：违例时置位；每 2^18 拍整表清零 (误标的 load 不会一直被拖慢)
  //==========================================================================
  // 违例位置先寄存一拍 (FTQ 读 + 加法)，下一拍写表
  logic [17:0] swt_age;
  logic        md_v;
  logic [16:1] md_pc;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      swt_age <= '0; md_v <= 1'b0; md_pc <= '0;
      for (int r = 0; r < SWR; r++) swt[r] <= '0;
    end else begin
      md_v  <= mdp_set_i;
      md_pc <= 16'(ftq_pc[mdp_ftq_i][16:1] + 16'(mdp_off_i));
      swt_age <= swt_age + 1'b1;
      if (swt_age == '1) for (int r = 0; r < SWR; r++) swt[r] <= '0;
      else if (md_v) swt[swt_row({47'b0, md_pc, 1'b0})][md_pc[4:1]] <= 1'b1;
    end
  end

  //==========================================================================
  // TAGE v2 (o6_tage)：F0 读表、F2 出每槽结果；训练与方向表同一拍 (tr_v + tr_* 寄存副本)
  //==========================================================================
  logic [15:0]      f2_bdir;
  logic [1:0]       tr_cv, tr_ctk;
  logic [3:0]       tr_cslot [2];
  bp_slot_t         tr_cinfo [2];
  // 训练候选：已解析的条件分支里，误预测的优先 (槽号小的先)，再取未误预测的
  // 2026-10-29 (6 GHz)：两个候选并行算 (原来"挑第一个 -> 从候选去掉 -> 再挑"串行，fe19 tr_sinfo -> u_tage/tb_* 180 ps)：
  //   mis / res 各求最低两位 (lo0 / lo1) 与 mis 个数 nm；候选 0 = nm ? lo0m : lo0r；
  //   候选 1 = nm >= 2 ? lo1m : nm == 1 ? (lo0r == lo0m ? lo1r : lo0r) : lo1r。原写法留作断言
  // 最低两位 (并行前缀)：p1[i] = x[i-1:0] 里至少 1 个，p2[i] = 至少 2 个；(a1,a2)∘(b1,b2) = (a1|b1, a2|b2|a1&b1) 可结合，
  // log 深度扫描。最低位独热 = x & ~p1，次低位 = x & p1 & ~p2，再或树编码
  function automatic logic [9:0] low2(input logic [15:0] x);   // {v1, i1, v0, i0}
    logic [15:0] g1, g2, p1, p2, o0, o1;
    logic [3:0]  i0, i1;
    g1 = x; g2 = '0;                       // 含本位的前缀 (先算到 [i:0])
    for (int d = 1; d < 16; d = d * 2) begin
      logic [15:0] n1, n2;
      for (int i = 0; i < 16; i++)
        if (i >= d) begin n1[i] = g1[i] | g1[i-d]; n2[i] = g2[i] | g2[i-d] | (g1[i] & g1[i-d]); end
        else        begin n1[i] = g1[i];           n2[i] = g2[i]; end
      g1 = n1; g2 = n2;
    end
    p1 = {g1[14:0], 1'b0}; p2 = {g2[14:0], 1'b0};   // 不含本位：[i-1:0]
    o0 = x & ~p1;
    o1 = x & p1 & ~p2;
    i0 = '0; i1 = '0;
    for (int i = 0; i < 16; i++) begin i0 = i0 | ({4{o0[i]}} & 4'(i)); i1 = i1 | ({4{o1[i]}} & 4'(i)); end
    return {|o1, i1, |o0, i0};
  endfunction
  logic [15:0] tr_res, tr_mis;
  always_comb begin
    logic [9:0] lm, lr;
    logic       m0, m1;               // nm >= 1 / nm >= 2
    for (int sl = 0; sl < 16; sl++) f2_bdir[sl] = f2_ctr[2*sl+1];
    tr_res = tr_brm & tr_cmask;
    // 按 TAGE 自己的方向挑"误预测优先" (tdir)：fdir 已被 SC 修正，SC 纠正了的分支 TAGE 自己其实错了，
    // 若按 fdir 挑它就不再被优先训练，TAGE 越学越差 (s8：SC 推翻 69% 正确，总误预测反而 +4%)
    for (int sl = 0; sl < 16; sl++) tr_mis[sl] = tr_res[sl] && (tr_sinfo[sl].tdir != tr_brt[sl]);
    lm = low2(tr_mis); lr = low2(tr_res);
    m0 = lm[4]; m1 = lm[9];
    tr_cv[0]    = lr[4];
    tr_cv[1]    = lr[9];
    tr_cslot[0] = m0 ? lm[3:0] : lr[3:0];
    tr_cslot[1] = m1 ? lm[8:5] : m0 ? ((lr[3:0] == lm[3:0]) ? lr[8:5] : lr[3:0]) : lr[8:5];
    if (!tr_cv[0]) tr_cslot[0] = '0;
    if (!tr_cv[1]) tr_cslot[1] = '0;
    for (int c = 0; c < 2; c++) begin
      tr_cinfo[c] = tr_sinfo[tr_cslot[c]];
      tr_ctk[c]   = tr_brt[tr_cslot[c]];
    end
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni && tr_v) begin      // 只在训练有效拍核对 (tr_* 无复位，空闲拍是 X)
      logic [15:0] rest;
      logic [1:0]  cv;
      logic [3:0]  cs [2];
      rest = tr_res;
      for (int c = 0; c < 2; c++) begin
        logic [15:0] pick;
        pick = (|(tr_mis & rest)) ? (tr_mis & rest) : rest;
        cv[c] = |pick; cs[c] = '0;
        for (int sl = 15; sl >= 0; sl--) if (pick[sl]) cs[c] = 4'(sl);
        rest[cs[c]] = rest[cs[c]] & ~cv[c];
      end
      assert (cv == tr_cv && cs[0] == tr_cslot[0] && cs[1] == tr_cslot[1])
        else $error("frontend: TAGE 训练候选并行版与串行原写法不一致 cv %b/%b slot %0d,%0d/%0d,%0d",
                    tr_cv, cv, tr_cslot[0], tr_cslot[1], cs[0], cs[1]);
    end
`endif
`ifndef SYNTHESIS
  // TAGE 训练统计：训练的分支数、预测时有提供者的、TAGE 最终方向错的；F3 每块有提供者的条件分支槽
  longint st_trn = 0, st_tphit = 0, st_tmis = 0, st_f3br = 0, st_f3hit = 0;
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      if (tr_v)
        for (int c = 0; c < 2; c++)
          if (tr_cv[c]) begin
            st_trn++;
            if (tr_cinfo[c].phit) st_tphit++;
            if (tr_cinfo[c].fdir != tr_ctk[c]) st_tmis++;
          end
      if (f3_fire && !f3_flt)
        for (int sl = 0; sl < 16; sl++)
          if (f3_cmask[sl]) begin st_f3br++; if (f3_bps[sl].phit) st_f3hit++; end
    end
  final $display("[perf] TAGE 训练 %0d 个分支 (有提供者 %0d，方向错 %0d)；F3 条件分支槽 %0d (有提供者 %0d)",
                 st_trn, st_tphit, st_tmis, st_f3br, st_f3hit);
  // +tagelog：每个训练的分支一行 (分支 PC、实际方向、TAGE 方向、提供者、提供者表 / 路 / 计数器 / u)
  int tlog_fd = 0;
  initial if ($test$plusargs("tagelog")) tlog_fd = $fopen("tage_train.log", "w");
  always_ff @(posedge clk_i)
    if (rst_ni && tlog_fd != 0 && tr_v)
      for (int c = 0; c < 2; c++)
        if (tr_cv[c])
          $fdisplay(tlog_fd, "%h %0d %0d %0d %0d %0d %0d %0d", {tr_pc[63:5], tr_cslot[c], 1'b0}, tr_ctk[c],
                    tr_cinfo[c].fdir, tr_cinfo[c].phit, tr_cinfo[c].pt, tr_cinfo[c].pw, tr_cinfo[c].pctr, tr_cinfo[c].pu);
`endif
  logic tg_bd, tg_bf, it_bd, it_bf;
  o6_tage u_tage (
    .clk_i, .rst_ni,
    .f0_rd_i(f0_rde), .f0_sent_i(f0_sent), .f0_pc_i(pc0), .f0_fold_i(foldx),
    .f1_move_i(f1_move), .f2_base_i(f2_bdir), .f2_slot_o(t_f2slot), .f2_uv_o(t_f2uv),
    .tr_v_i(tr_v), .tr_pc_i(tr_pc), .tr_fold_i(tr_fold), .tr_cv_i(tr_cv), .tr_cslot_i(tr_cslot),
    .tr_cinfo_i(tr_cinfo), .tr_ctk_i(tr_ctk), .tr_uv_i(tr_uv),
    .bist_start_i, .bist_done_o(tg_bd), .bist_fail_o(tg_bf)
  );

  // 统计校正器：在 F2 修正 TAGE 的方向 (训练：块里所有已解析的条件分支)
  o6_sc u_sc (
    .clk_i, .rst_ni,
    .f0_sent_i(f0_sent), .f0_pc_i(pc0), .f0_fold_i(foldx), .f1_move_i(f1_move),
    .f2_tage_i(t_f2slot), .f2_slot_o(s_f2slot),
    .tr_v_i(tr_v), .tr_pc_i(tr_pc), .tr_fold_i(tr_fold), .tr_res_i(tr_brm & tr_cmask),
    .tr_sinfo_i(tr_sinfo), .tr_brt_i(tr_brt)
  );

  // 间接跳转目标：块里有已解析的非返回 JALR 时训练；"最终预测错" = FTQ 记的预测下一块地址 != 实际目标
  o6_ittage u_itt (
    .clk_i, .rst_ni,
    .f0_rd_i(f0_rde), .f0_sent_i(f0_sent), .f0_pc_i(pc0), .f0_fold_i(foldx), .f1_move_i(f1_move),
    .f2_v_o(it_f2v), .f2_tgt_o(it_f2tgt), .f2_info_o(it_f2info),
    .tr_v_i(tr_v && tr_itv), .tr_pc_i(tr_pc), .tr_fold_i(tr_fold), .tr_info_i(tr_itinfo),
    .tr_act_i(tr_itgt), .tr_mis_i(tr_ptgt != tr_itgt),
    .bist_start_i, .bist_done_o(it_bd), .bist_fail_o(it_bf)
  );
  assign bist_done_o = tg_bd && it_bd;
  assign bist_fail_o = tg_bf || it_bf;

  //==========================================================================
  // uBTB 训练：每个 F3 块都训练 (方向计数器跟随 F3 的最终预测)，后端误预测按实际结果纠正，
  // F3 不出块时由 F1 BTB 改道补装。一拍至多一个请求 (后端 > F3 > F1)，寄存一拍再查 CAM、写表
  //   brv：本块经过的条件分支槽 (到第一个跳转为止)   brt：其中跳转的那个
  //   taken / tslot / tcond / tgt / ret / call / lk：本块最终跳转 (有则插入 / 更新该槽)
  //   rch：本块走到了的槽 (到跳转为止)；走到了却没跳的无条件槽说明项已过时 (别名 / 代码变了)，作废该槽
  //==========================================================================
  logic              ub_wr, ub_f3;
  logic [30:1]       ub_wtag;
  logic [15:0]       ub_wbrv, ub_wbrt;
  logic              ub_wtk, ub_wtcond;
  logic [3:0]        ub_wslot;
  logic [63:0]       ub_wtgt;
  logic              ub_wret, ub_wcall;
  logic [6:0]        ub_wlk;
  logic [UPW-1:0]    ub_wupi;
  logic [15:0]       f3_brv, f3_brt;
  always_comb begin
    logic bef;                 // 前面已有预测跳转的位置
    f3_brv = '0; f3_brt = '0;
    bef = 1'b0;
    for (int p = 0; p < NP; p++) begin
      logic [3:0] sl;
      sl = (p == 0) ? 4'd15 : 4'(p - 1);
      if (f3_pv[p] && f3_cf[p].br && !bef) begin
        f3_brv[sl] = 1'b1;
        f3_brt[sl] = p_first[p];
      end
      bef = bef || p_tk[p];
    end
  end
  always_comb begin
    ub_wr = 1'b0; ub_f3 = 1'b0; ub_wtag = f3_pc[30:1];
    ub_wbrv = f3_brv; ub_wbrt = f3_brt;
    ub_wtk = g_taken; ub_wtcond = g_tcond; ub_wslot = g_tslot; ub_wtgt = g_npc; ub_wret = g_tret;
    ub_wcall = g_push; ub_wlk = g_lk; ub_wupi = f3_upi;
    if (be_redirect_i) begin
      // 误预测分支：bres 里 mispred 的那条 (条件分支则计数器按实际方向走一步；跳转则插入 / 更新目标)
      for (int k = 0; k < N_BRU; k++)
        if (bres_valid_i[k] && bres_mispred_i[k]) begin
          logic [3:0] sl;
          sl = ftq_slot0[bres_ftq_i[k]] + bres_off_i[k][3:0];
          ub_wr     = 1'b1;
          ub_wtag   = ftq_hwm[bres_ftq_i[k]] ? 30'(ftq_pc[bres_ftq_i[k]][30:1] + 30'd1) : ftq_pc[bres_ftq_i[k]][30:1];
          ub_wupi   = ftq_upi[bres_ftq_i[k]];
          ub_wslot  = sl;
          ub_wtgt   = bres_target_i[k];
          ub_wret   = 1'b0;
          ub_wcall  = 1'b0;
          ub_wlk    = '0;
          ub_wtk    = bres_taken_i[k];
          ub_wtcond = 1'b1;                 // 后端只报条件分支与间接跳转；间接跳转的目标更新与条件无关
          ub_wbrv   = 16'(1) << sl;
          ub_wbrt   = bres_taken_i[k] ? 16'(1) << sl : 16'd0;
        end
    end else if (f3_fire && !f3_flt) begin
      ub_wr = 1'b1; ub_f3 = 1'b1;
    end else if (f1_ovr && p1_b) begin
      // BTB 改道：把这一项也装进 uBTB (下次 F0 就预测到)
      ub_wr = 1'b1; ub_wtag = f1_pc[30:1]; ub_wtk = 1'b1; ub_wtcond = f1_bcond; ub_wslot = f1_bslot;
      ub_wtgt = f1_btgt_e; ub_wret = f1_bret; ub_wcall = f1_bcall; ub_wlk = f1_blk; ub_wupi = f1_upi;
      ub_wbrv = f1_bcond ? 16'(1) << f1_bslot : 16'd0; ub_wbrt = ub_wbrv;
    end
  end

  // 行预测器训练：F1 改道 (F0 的 LP 预测与 F1 的最终预测不同) 时按 p1 写该块的项
  int unsigned lp_nwr = 0, lp_nprd = 0, lp_nmiss = 0, lp_ntk = 0, lp_nnt = 0, lp_ntgt = 0, lp_nmiss_nt = 0;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) for (int i = 0; i < NLP; i++) lp_v[i] <= 1'b0;
    else if (f1_ovr) lp_v[lp_idx(f1_pc)] <= 1'b1;
  always_ff @(posedge clk_i)
    if (f1_ovr) begin
      logic [LPW-1:0] w;
      w = lp_idx(f1_pc);
      lp_tag[w] <= lp_tg(f1_pc); lp_tk[w] <= p1_tk; lp_slot[w] <= p1_slot; lp_tgt[w] <= p1_npc[38:1];
      lp_ret[w] <= p1_ret; lp_call[w] <= p1_call; lp_lk[w] <= p1_lk; lp_ph[w] <= h_f1;
      lp_ts[w] <= ub_set(p1_npc[30:1]); lp_tu[w] <= up_pc(p1_npc); lp_tl[w] <= lp_idx(p1_npc);
    end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni) begin
    if (f1_ovr) begin
      lp_nwr++;
      if (!f1_lph)                     lp_nmiss++;      // LP 没这一项 (p1 跳)
      else if (f1_ubhit && !p1_tk)     lp_ntk++;        // LP 说跳、p1 不跳
      else if (!f1_ubhit && p1_tk)     lp_nnt++;        // LP 说不跳 (有项)、p1 跳
      else                             lp_ntgt++;       // 都跳，槽 / 目标不同
    end
    if (f0_sent) lp_nprd++;
  end
  final $display("[perf] 行预测器：F0 发出 %0d 块，F1 改道 %0d 次 (LP 缺项 %0d、LP 跳 p1 不跳 %0d、LP 不跳 p1 跳 %0d、目标 / 槽不同 %0d)",
                 lp_nprd, lp_nwr, lp_nmiss, lp_ntk, lp_nnt, lp_ntgt);
`endif

  // 训练请求寄存
  logic              uq_wr, uq_f3;
  logic [30:1]       uq_tag;
  logic [15:0]       uq_brv, uq_brt;
  logic              uq_tk, uq_tcond;
  logic [3:0]        uq_slot;
  logic [63:0]       uq_tgt;
  logic              uq_ret, uq_call;
  logic [6:0]        uq_lk;
  logic [UPW-1:0]    uq_upi;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      uq_wr <= 1'b0; uq_f3 <= 1'b0;
    end else begin
      uq_wr <= ub_wr && !tlb_flush_i;
      uq_f3 <= ub_f3;
    end
  end
  always_ff @(posedge clk_i) begin
    uq_tag <= ub_wtag; uq_brv <= ub_wbrv; uq_brt <= ub_wbrt; uq_tk <= ub_wtk; uq_tcond <= ub_wtcond;
    uq_slot <= ub_wslot; uq_tgt <= ub_wtgt; uq_ret <= ub_wret; uq_call <= ub_wcall; uq_lk <= ub_wlk;
    uq_upi <= ub_wupi;
  end

  logic             uq_found;
  logic             uq_fwd_v;          // UV / UW 级有同标签、将写入的项
  logic             uq_fwd_uv;         // 前递来自 UV 级 (否则 UW 级)
  logic [$clog2(NUB)-1:0] uq_fwd_idx;
  logic [$clog2(NUB)-1:0] uq_idx;
  logic [$clog2(UBS)-1:0] uq_set;
  logic             uq_sfound;         // 组内查找命中 (不含前递)
  logic [UBW-1:0]   uq_wsel;           // 读旧项用的路 (独热)：命中路，否则分配路
  // 2026-10-24，6 GHz：旧项按 "组号读出整组 4 路 -> 用路独热与或选择" 取 (同 F0 查找)，
  // 不再 "比标签 -> 编出项号 -> 按项号 128 选 1" 串联 (fe10：uq_tag -> uv_ocall 210 ps)；项号只用于写回
  always_comb begin
    logic [UBW-1:0] hit, alloc;
    logic           inv;
    uq_set = ub_set(uq_tag);
    for (int w = 0; w < UBW; w++) hit[w] = ub_v[{uq_set, UWW'(w)}] && ub_tag[{uq_set, UWW'(w)}] == uq_tag;
    // 分配：组内先找无效路 (编号小的优先)，否则用该组的轮转指针
    alloc = '0; inv = 1'b0;
    for (int w = 0; w < UBW; w++) if (!inv && !ub_v[{uq_set, UWW'(w)}]) begin inv = 1'b1; alloc[w] = 1'b1; end
    if (!inv) alloc[ub_rr[uq_set]] = 1'b1;
    uq_sfound = |hit;
    uq_wsel   = uq_sfound ? hit : alloc;
    uq_idx = {uq_set, UWW'(0)};
    for (int w = 0; w < UBW; w++) if (uq_wsel[w]) uq_idx = {uq_set, UWW'(w)};
    uq_found = uq_sfound;
    // 同一块还在 UV / UW 级没写进表：按已有项处理，用它的项号 (否则连续两次训练会分配出两个同标签的项)
    if (!uq_sfound && uq_fwd_v) begin uq_found = 1'b1; uq_idx = uq_fwd_idx; end
  end

  // UV 级新内容 (un_*) 与 UW 级寄存器 (uw_*) 的声明提前：查找级的前递要用
  logic        un_sv [2], un_cond [2], un_ret [2], un_call [2];
  logic [3:0]  un_slot [2];
  logic [1:0]  un_ctr [2];
  logic [63:0] un_tgt [2];
  logic [6:0]  un_lk [2];
  logic             uw_we, uw_pht;
  logic             uw_found;
  logic [$clog2(NUB)-1:0] uw_idx;
  logic [$clog2(UBS)-1:0] uw_set;
  logic [UPW-1:0]   uw_upi;
  logic [30:1]      uw_tag;
  logic             uw_sv [2], uw_cond [2], uw_ret [2], uw_call [2];
  logic [1:0]       uw_ph [2];
  logic [$clog2(UBS)-1:0] uw_ts [2];
  logic [UPW-1:0]   uw_tu [2];
  logic [3:0]       uw_slot [2];
  logic [1:0]       uw_ctr [2];
  logic [38:1]      uw_tgt [2];
  logic [6:0]       uw_lk [2];
  // UV 级 (2026-10-22，6 GHz)：uq 级只做组内查找 + 读出旧项，结果寄存；UV 级算新内容，UW 级写。
  // 原来 uq_tag -> 组号散列 -> 读组内标签 -> 比较 -> 读选中项 -> 算新内容 一拍 (fe5_250：37 级 / 248 ps)。
  // 查找看到的 ub_v / ub_rr 比写入早两拍：同一块连续两次训练时后一次可能覆盖前一次的更新 (只影响预测)。
  logic             uv_wr, uv_f3, uv_found;
  logic [$clog2(NUB)-1:0] uv_idx;
  logic [$clog2(UBS)-1:0] uv_set;
  logic [30:1]      uv_tag;
  logic [15:0]      uv_brv, uv_brt;
  logic             uv_tk, uv_tcond, uv_ret, uv_call;
  logic [3:0]       uv_slot;
  logic [63:0]      uv_tgt;
  logic [6:0]       uv_lk;
  logic [UPW-1:0]   uv_upi;
  logic             uv_osv [2], uv_ocond [2], uv_oret [2], uv_ocall [2];
  logic [3:0]       uv_oslot [2];
  logic [1:0]       uv_octr [2];
  logic [38:1]      uv_otgt [2];
  logic [6:0]       uv_olk [2];
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin uv_wr <= 1'b0; uv_f3 <= 1'b0; end
    else begin
      uv_wr <= uq_wr && !tlb_flush_i;
      uv_f3 <= uq_f3;
    end
  end
  always_ff @(posedge clk_i) begin
    uv_found <= uq_found; uv_idx <= uq_idx; uv_set <= uq_set; uv_tag <= uq_tag; uv_upi <= uq_upi;
    uv_brv <= uq_brv; uv_brt <= uq_brt; uv_tk <= uq_tk; uv_tcond <= uq_tcond; uv_ret <= uq_ret; uv_call <= uq_call;
    uv_slot <= uq_slot; uv_tgt <= uq_tgt; uv_lk <= uq_lk;
    for (int k = 0; k < 2; k++) begin
      // 旧项内容：前递时取还没写进表的那次训练的新内容 (表里该位置还是被替换的别的块)；计数器总按 uPHT 读
      if (uq_fwd_v && uq_fwd_uv) begin
        uv_osv[k] <= un_sv[k]; uv_ocond[k] <= un_cond[k]; uv_oret[k] <= un_ret[k]; uv_ocall[k] <= un_call[k];
        uv_oslot[k] <= un_slot[k]; uv_otgt[k] <= un_tgt[k][38:1]; uv_olk[k] <= un_lk[k];
      end else if (uq_fwd_v) begin
        uv_osv[k] <= uw_sv[k]; uv_ocond[k] <= uw_cond[k]; uv_oret[k] <= uw_ret[k]; uv_ocall[k] <= uw_call[k];
        uv_oslot[k] <= uw_slot[k]; uv_otgt[k] <= uw_tgt[k]; uv_olk[k] <= uw_lk[k];
      end else begin
        logic        sv, cd, rt, cl;
        logic [3:0]  sl;
        logic [38:1] tg;
        logic [6:0]  lk;
        sv = 1'b0; cd = 1'b0; rt = 1'b0; cl = 1'b0; sl = '0; tg = '0; lk = '0;
        for (int w = 0; w < UBW; w++) begin
          logic [$clog2(NUB)-1:0] i;
          i  = {uq_set, UWW'(w)};
          sv = sv | (uq_wsel[w] & ub_sv[k][i]);   cd = cd | (uq_wsel[w] & ub_cond[k][i]);
          rt = rt | (uq_wsel[w] & ub_ret[k][i]);  cl = cl | (uq_wsel[w] & ub_call[k][i]);
          sl = sl | ({4{uq_wsel[w]}} & ub_slot[k][i]);
          tg = tg | ({38{uq_wsel[w]}} & ub_tgt[k][i]);
          lk = lk | ({7{uq_wsel[w]}} & ub_lk[k][i]);
        end
        uv_osv[k] <= uq_sfound && sv; uv_ocond[k] <= cd; uv_oret[k] <= rt; uv_ocall[k] <= cl;
        uv_oslot[k] <= sl; uv_otgt[k] <= tg; uv_olk[k] <= lk;
      end
      uv_octr[k] <= upht[k][uq_upi];
    end
  end

  // 选中项的新内容 (读改写：项都是触发器)
  always_comb begin
    logic        o_sv [2], o_cond [2], o_ret [2], o_call [2], have;
    logic [3:0]  o_slot [2];
    logic [1:0]  o_ctr [2];
    logic [63:0] o_tgt [2];
    logic [6:0]  o_lk [2];
    logic        n_cond, n_ret, n_call;
    logic [3:0]  n_slot;
    logic [63:0] n_tgt;
    logic [6:0]  n_lk;
    for (int k = 0; k < 2; k++) begin
      o_sv[k]   = uv_osv[k];
      o_cond[k] = uv_ocond[k]; o_ret[k] = uv_oret[k]; o_call[k] = uv_ocall[k];
      o_slot[k] = uv_oslot[k]; o_ctr[k] = uv_octr[k];
      o_tgt[k]  = ub_tx(uv_otgt[k]);  o_lk[k]  = uv_olk[k];
      // 计数器：经过的条件分支按本块方向走一步
      if (o_sv[k] && o_cond[k] && uv_brv[o_slot[k]])
        o_ctr[k] = uv_brt[o_slot[k]] ? ((o_ctr[k] == 2'd3) ? 2'd3 : o_ctr[k] + 2'd1)
                                     : ((o_ctr[k] == 2'd0) ? 2'd0 : o_ctr[k] - 2'd1);
      // 走到了却没跳的无条件槽：项已过时
      if (uv_f3 && o_sv[k] && !o_cond[k] && (!uv_tk || o_slot[k] < uv_slot)) o_sv[k] = 1'b0;
    end
    for (int k = 0; k < 2; k++) begin
      un_sv[k] = o_sv[k]; un_cond[k] = o_cond[k]; un_ret[k] = o_ret[k]; un_call[k] = o_call[k];
      un_slot[k] = o_slot[k]; un_ctr[k] = o_ctr[k]; un_tgt[k] = o_tgt[k]; un_lk[k] = o_lk[k];
    end
    // 本块的跳转：已在项里则更新目标等，否则插入 (保持槽号升序；满了保留槽 0 与新槽，丢掉槽 1)
    have = 1'b0;
    if (uv_tk) begin
      for (int k = 0; k < 2; k++)
        if (o_sv[k] && o_slot[k] == uv_slot) begin
          have = 1'b1;
          un_tgt[k] = uv_tgt; un_ret[k] = uv_ret; un_call[k] = uv_call; un_lk[k] = uv_lk; un_cond[k] = uv_tcond;
        end
      if (!have) begin
        logic       ov;          // 留下的旧槽 (若有) 的下标
        logic       keep;        // 是否有旧槽要留
        keep = o_sv[0] || o_sv[1];
        ov   = o_sv[0] ? 1'b0 : 1'b1;
        if (keep && o_slot[ov] < uv_slot) begin
          // 旧槽在前，新槽在后
          un_sv[0] = 1'b1; un_slot[0] = o_slot[ov]; un_cond[0] = o_cond[ov]; un_ctr[0] = o_ctr[ov];
          un_tgt[0] = o_tgt[ov]; un_ret[0] = o_ret[ov]; un_call[0] = o_call[ov]; un_lk[0] = o_lk[ov];
          un_sv[1] = 1'b1; un_slot[1] = uv_slot; un_cond[1] = uv_tcond; un_ctr[1] = 2'd2;
          un_tgt[1] = uv_tgt; un_ret[1] = uv_ret; un_call[1] = uv_call; un_lk[1] = uv_lk;
        end else begin
          // 新槽在前，旧槽 (若有) 在后
          un_sv[0] = 1'b1; un_slot[0] = uv_slot; un_cond[0] = uv_tcond; un_ctr[0] = 2'd2;
          un_tgt[0] = uv_tgt; un_ret[0] = uv_ret; un_call[0] = uv_call; un_lk[0] = uv_lk;
          un_sv[1] = keep; un_slot[1] = o_slot[ov]; un_cond[1] = o_cond[ov]; un_ctr[1] = o_ctr[ov];
          un_tgt[1] = o_tgt[ov]; un_ret[1] = o_ret[ov]; un_call[1] = o_call[ov]; un_lk[1] = o_lk[ov];
        end
      end
    end
  end
  // 写：已有项总是写回 (计数器)；新项只在本块有跳转时分配
  wire uv_we = uv_wr && (uv_found || uv_tk);

  // 训练写入再寄存一拍 (UW 级，2026-10-20，6 GHz)：原来 uq 寄存器 -> 组内比标签 -> 读出旧项 -> 算新项
  // -> 写 1024 项 uPHT / uBTB 串成一拍 (2nm 综合 41 级 / 271 ps)。现在 uq 级算好新内容，UW 级只做写。
  // 连续两次训练同一项时后一次可能读到未写回的旧值，只是少学一步 (预测器，不影响正确性)；
  // TLB 刷新时 UW 级在途的写一并作废 (否则把刷新前的项重新写成有效)。
  // 同时修正原来的悬空 else：`else if (uq_we)` 曾绑到 for 里的内层 if 上，ub_v / ub_rr 的更新受槽位条件影响
  // (两槽都是条件分支时新分配的项不置有效)。
  // 训练查找的前递 (2026-10-23)：UV 级比 UW 级新，优先
  always_comb begin
    uq_fwd_v = 1'b0; uq_fwd_uv = 1'b0; uq_fwd_idx = uw_idx;
    if (uw_we && uw_tag == uq_tag) begin uq_fwd_v = 1'b1; uq_fwd_idx = uw_idx; end
    if (uv_we && uv_tag == uq_tag) begin uq_fwd_v = 1'b1; uq_fwd_uv = 1'b1; uq_fwd_idx = uv_idx; end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin uw_we <= 1'b0; uw_pht <= 1'b0; end
    else begin
      uw_we  <= uv_we && !tlb_flush_i;
      uw_pht <= uv_we;
    end
  end
  always_ff @(posedge clk_i) begin
    uw_found <= uv_found; uw_idx <= uv_idx; uw_set <= uv_set; uw_upi <= uv_upi; uw_tag <= uv_tag;
    for (int k = 0; k < 2; k++) begin
      uw_sv[k] <= un_sv[k]; uw_cond[k] <= un_cond[k]; uw_ret[k] <= un_ret[k]; uw_call[k] <= un_call[k];
      uw_ph[k] <= bp_phash(un_slot[k], un_tgt[k]);
      uw_ts[k] <= ub_set(un_tgt[k][30:1]);
      uw_tu[k] <= up_pc(un_tgt[k]);
      uw_slot[k] <= un_slot[k]; uw_ctr[k] <= un_ctr[k]; uw_tgt[k] <= un_tgt[k][38:1]; uw_lk[k] <= un_lk[k];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ub_v <= '0;
      for (int i = 0; i < UBS; i++) ub_rr[i] <= '0;
      for (int k = 0; k < 2; k++) for (int i = 0; i < NUB; i++) ub_sv[k][i] <= 1'b0;
      for (int k = 0; k < 2; k++) for (int i = 0; i < NUP; i++) upht[k][i] <= 2'd2;
    end else begin
      // uPHT：本次训练涉及的槽 (有条件分支经过 / 新插入的槽) 写回计数器
      if (uw_pht)
        for (int k = 0; k < 2; k++)
          if (uw_sv[k] && uw_cond[k]) upht[k][uw_upi] <= uw_ctr[k];
      if (uw_we && !tlb_flush_i) begin
        ub_v[uw_idx] <= uw_sv[0] || uw_sv[1];
        if (!uw_found) ub_rr[uw_set] <= ub_rr[uw_set] + 1'b1;
        for (int k = 0; k < 2; k++) ub_sv[k][uw_idx] <= uw_sv[k];
      end
      if (tlb_flush_i) ub_v <= '0;               // 页表改变 (虚地址映射可能变了)；放在最后，优先于本拍的写
    end
  end
  always_ff @(posedge clk_i) begin
    if (uw_we) begin
      ub_tag[uw_idx] <= uw_tag;
      for (int k = 0; k < 2; k++) begin
        ub_slot[k][uw_idx] <= uw_slot[k];
        ub_cond[k][uw_idx] <= uw_cond[k];
        ub_tgt[k][uw_idx]  <= uw_tgt[k];
        ub_ret[k][uw_idx]  <= uw_ret[k];
        ub_ph[k][uw_idx]   <= uw_ph[k];
        ub_ts[k][uw_idx]   <= uw_ts[k];
        ub_tu[k][uw_idx]   <= uw_tu[k];
        ub_call[k][uw_idx] <= uw_call[k];
        ub_lk[k][uw_idx]   <= uw_lk[k];
      end
    end
  end

  //==========================================================================
  // BTB 训练 (F3 预测不一致时，寄存一拍再写)
  //==========================================================================
  logic        bq_wr, bq_inv, bq_ret, bq_cond, bq_call;
  logic [6:0]  bq_lk;
  logic [BTW-1:0] bq_idx;
  logic [15:0] bq_tag;
  logic [3:0]  bq_slot;
  logic [38:1] bq_tgt;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      bq_wr <= 1'b0; bq_inv <= 1'b0;
      bt_v <= '0;
    end else begin
      bq_wr  <= mispred3 && g_taken && !tlb_flush_i;
      bq_inv <= mispred3 && !g_taken && !tlb_flush_i;
      if (tlb_flush_i) bt_v <= '0;
      else if (bq_wr) bt_v[bq_idx] <= 1'b1;
      else if (bq_inv && bt_tag[bq_idx] == bq_tag) bt_v[bq_idx] <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    bq_idx <= bt_idx(f3_pc); bq_tag <= bt_tg(f3_pc); bq_slot <= g_tslot; bq_tgt <= g_npc[38:1];
    bq_ret <= g_tret; bq_cond <= g_tcond; bq_call <= g_push; bq_lk <= g_lk;
    if (bq_wr) begin
      bt_tag[bq_idx] <= bq_tag; bt_slot[bq_idx] <= bq_slot; bt_tgt[bq_idx] <= bq_tgt;
      bt_ret[bq_idx] <= bq_ret; bt_cond[bq_idx] <= bq_cond; bt_call[bq_idx] <= bq_call; bt_lk[bq_idx] <= bq_lk;
    end
  end

`ifndef SYNTHESIS
  //==========================================================================
  // F3 覆盖细分 (仿真统计)：F3 时用 f3_pc 重新查 uBTB (表在两拍里可能被训练过，统计近似)
  //   a_miss   漏跳：uBTB 没有这一块               a_noslot 漏跳：有块、但跳转槽不在项里
  //   a_ctr    漏跳：槽在项里，计数器说不跳 (条件分支) b_ctr    误跳：uBTB 计数器说跳，F3 (TAGE/SC) 说不跳
  //   b_unc    误跳：uBTB 的无条件槽 F3 不跳 (过时 / 无目标的 JALR)
  //   c_early  都跳，F3 的跳转槽更靠前 (前面有条件分支 TAGE 说跳)   c_late 都跳，F3 槽更靠后 (uBTB 的槽 TAGE 说不跳)
  //   d_ret / d_ind / d_dir   同槽目标不同：返回 / 间接 / 直接 (过时)       e_ras  只有压 / 弹 RAS 不一致
  //==========================================================================
  longint sc_a_miss = 0, sc_a_noslot = 0, sc_a_ctr = 0, sc_b_ctr = 0, sc_b_unc = 0, sc_c_early = 0, sc_c_late = 0;
  longint sc_d_ret = 0, sc_d_ind = 0, sc_d_dir = 0, sc_e_ras = 0, sc_f1 = 0;
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      if (f1_ovr) sc_f1++;
      if (mispred3) begin
        logic hit, have;
        logic [$clog2(UBS)-1:0] st;
        logic [$clog2(NUB)-1:0] ix;
        st = ub_set(f3_pc[30:1]); hit = 1'b0; ix = '0;
        for (int w = 0; w < UBW; w++)
          if (ub_v[{st, UWW'(w)}] && ub_tag[{st, UWW'(w)}] == f3_pc[30:1]) begin hit = 1'b1; ix = {st, UWW'(w)}; end
        have = 1'b0;
        for (int k = 0; k < 2; k++) if (hit && ub_sv[k][ix] && ub_slot[k][ix] == g_tslot) have = 1'b1;
        if (g_taken && !f3_ubhit) begin
          if (!hit) sc_a_miss++; else if (!have) sc_a_noslot++; else sc_a_ctr++;
        end else if (!g_taken && f3_ubhit) begin
          logic uc;
          uc = 1'b0;
          for (int k = 0; k < 2; k++) if (hit && ub_sv[k][ix] && ub_slot[k][ix] == f3_ubslot && !ub_cond[k][ix]) uc = 1'b1;
          if (uc) sc_b_unc++; else sc_b_ctr++;
        end else if (g_taken && f3_ubhit && g_tslot != f3_ubslot) begin
          if (g_tslot < f3_ubslot) sc_c_early++; else sc_c_late++;
        end else if (g_taken && !g_eq) begin
          if (g_tret) sc_d_ret++;
          else begin
            // 直接跳转 / 条件分支的目标由预译码算出，不可能"预测错"，只可能是 uBTB 记的旧目标
            logic ind;
            ind = 1'b0;
            for (int p = 0; p < NP; p++) if (p_first[p] && f3_cf[p].jalr) ind = 1'b1;
            if (ind) sc_d_ind++; else sc_d_dir++;
          end
        end else sc_e_ras++;
      end
    end
  longint st_ubmh = 0;          // F0 发出时 uBTB 组内多路命中 (重复项) 的次数
  always_ff @(posedge clk_i) if (rst_ni && f0_sent && ub_mh) st_ubmh++;
  final $display("[perf] uBTB 多路命中 (重复项) %0d 次", st_ubmh);
  final $display("[perf] F3 覆盖细分：漏跳 (无项 %0d 无槽 %0d 计数器 %0d)  误跳 (计数器 %0d 无条件槽 %0d)  跳转槽不同 (F3 更前 %0d 更后 %0d)  同槽目标错 (返回 %0d 间接 %0d 直接 %0d)  仅 RAS %0d；F1 改道 %0d",
                 sc_a_miss, sc_a_noslot, sc_a_ctr, sc_b_ctr, sc_b_unc, sc_c_early, sc_c_late, sc_d_ret, sc_d_ind, sc_d_dir, sc_e_ras, sc_f1);
`endif

  wire unused = f1_ubret;
endmodule
