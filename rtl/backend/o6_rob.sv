//==============================================================================
// o6_rob.sv - 重排序缓冲 (32 行 x 6 列)
//
// 分配：一个分派组占一整行 (列 0..k-1 有效)。
// 完成：done_a (主完成 / STA)，done_b (STD；非 store 分派时直接置 1)。
// 提交：只从头行提交，从 head_col 开始的连续可提交前缀，每拍最多一行剩余部分。
//   停止条件：未完成；有异常 (若是第一条则发起陷入)；flush_after (提交它然后冲刷)。
// 中断：头项有效且不是"已启动的 wait_head 指令"时可接受 (在头项之前进入中断)。
//   wait_head 指令 (CSR/AMO/FENCE/MMIO) 一旦启动就必须提交，否则返回后会重复副作用。
//   已被 IQ 选中、还在 RF 级没启动的 CSR / 向量指令：中断被接受后 (ev_block) 由 o6_int_exu 不再发请求 (#28)。
//
// tval：只有一个"最老异常"寄存器存 64 位 tval，所有带异常的完成按年龄竞争写入。
//   与 PC 相关的 tval (EBREAK、取指错误) 在提交时由 tval_sel 选择，不占存储。
//==============================================================================
`default_nettype none
module o6_rob
  import o6_pkg::*;
#(
  parameter int N_CMPL = 10,
  parameter int N_STD  = 2,
  // 哪些完成口可能带异常 (只有它们参与"最老异常"竞争；其余口的 excp 被忽略)
  parameter logic [N_CMPL-1:0] XCPT_PORTS = '1
) (
  input  wire              clk_i,
  input  wire              rst_ni,

  //---------------- 分派
  input  wire  [W-1:0]     dis_valid_i,
  input  var uop_t [W-1:0] dis_uop_i,
  input  wire  [63:0]      dis_tval_i [W],   // 译码期异常的 tval (非法指令 = 原编码)
  input  wire              dis_fire_i,       // 分派级确实写入本行
  output wire              dis_ready_o,      // 有空行
  output rob_ptr_t         dis_ptr_o,        // 新行的指针 (列 0)

  //---------------- 完成
  input  var cmpl_t [N_CMPL-1:0] cmpl_i,
  input  wire  [63:0]      cmpl_x2_i [N_CMPL],   // H：各完成口的 guest page fault GPA (非 LSU 口接 0)
  input  wire  [N_STD-1:0]       std_done_i,
  input  var rob_ptr_t [N_STD-1:0] std_rob_i,
  input  wire              head_start_i,     // 头项的 wait_head 指令已启动
  input  wire              step_i,           // 调试单步：每拍最多提交 1 条
  // 访存违例：给 load 置 replay 位 (到头时不提交，冲刷后从它自己重取)，不改完成状态
  input  wire  [N_STA-1:0] viol_valid_i,        // 各 STA 流水的访存违例
  input  var rob_ptr_t [N_STA-1:0] viol_rob_i,

  //---------------- 广播
  input  var br_update_t   br_upd_i,
  input  wire              flush_i,          // 冲刷 (陷入 / flush_after 生效)

  //---------------- 提交
  output logic [W-1:0]     cm_valid_o,       // 本拍提交
  output logic [W-1:0]     cm_rd_valid_o,    // 有目的寄存器
  output logic [W-1:0]     cm_rd_fp_o,
  output logic [4:0]       cm_lrd_o [W],
  output preg_t [W-1:0]    cm_pdst_o,
  output preg_t [W-1:0]    cm_old_pdst_o,
  output logic [W-1:0]     cm_mvel_o,
  output logic [W-1:0]     cm_is_ld_o,
  output logic [W-1:0]     cm_is_st_o,
  output logic [W-1:0]     cm_is_br_o,
  output logic [FTQ_W-1:0] cm_ftq_idx_o [W],
  output logic [4:0]       cm_fflags_o,      // 本拍提交的 fflags 或
  output logic             cm_fp_dirty_o,    // 本拍提交的指令改了浮点状态

  // 提交级事件 (下一拍由 o6_commit_ctrl 处理)
  output logic             ev_valid_o,
  output logic             ev_trap_o,        // 1 = 陷入 (异常或中断)
  output logic             ev_replay_o,      // 1 = 头项重执行 (不提交，从它的 PC 重取)
                                             // trap/replay 都为 0 = flush_after (已提交，从下一条重取)
  output logic             ev_irq_o,
  output logic [5:0]       ev_cause_o,
  output logic [63:0]      ev_tval_o,        // tval_sel = REG 时有效
  output logic [63:0]      ev_tval2_o,       // H：guest page fault 的 {隐式, 6'b0, GPA[58:2]} (与 ev_tval_o 同一个最老异常)
  output logic [1:0]       ev_tval_sel_o,
  output logic [FTQ_W-1:0] ev_ftq_idx_o,
  output logic [FTQ_OFF_W-1:0] ev_ftq_off_o,
  output logic             ev_is_rvc_o,
  output fu_e              ev_fu_o,
  output logic [4:0]       ev_op_o,
  input  wire              irq_pending_i,    // 有可接受的中断 (已按特权与使能过滤)
  input  wire  [5:0]       irq_cause_i,
  input  wire              ev_block_i,       // 提交级正在处理上一事件

  output rob_ptr_t         head_o,
  output wire              head_go_o,        // 允许 wait_head 指令启动
  // 第二老项 (头行里紧跟头的有效项) 与"头是已完成、无异常 / 冲刷 / 重执行的向量指令"：
  // 此时第二老的向量指令可以提前发射 (它到 EX 时头已提交)
  output rob_ptr_t         head2_o,
  output logic             head2_go_o,
  output wire              empty_o
);
  localparam int R = ROB_ROWS;

  // tval_sel
  localparam logic [1:0] TV_REG = 2'd0, TV_PC = 2'd1, TV_PC2 = 2'd2, TV_ZERO = 2'd3;

  typedef struct packed {
    logic                 done_a, done_b, started;
    logic                 excp;
    logic [5:0]           cause;
    logic [1:0]           tval_sel;
    logic                 flush_after;
    logic                 replay;
    logic                 wait_head;
    rclass_e              rd_class;
    logic [4:0]           lrd;
    preg_t                pdst, old_pdst;
    logic                 is_ld, is_st, is_br;
    logic [FTQ_W-1:0]     ftq_idx;
    logic [FTQ_OFF_W-1:0] ftq_off;
    logic                 is_rvc;
    logic [4:0]           fflags;
    fu_e                  fu;
    logic [4:0]           op;
    logic                 mvel;        // 被消除的 move：没有分配新的物理寄存器
  } rob_ent_t;

  rob_ent_t                 ent   [R][W];
  logic [W-1:0]             vld   [R];
  logic [ROB_ROW_W-1:0]     head_row, tail_row;
  logic                     head_flag, tail_flag;
  logic [ROB_COL_W-1:0]     head_col;

  wire full  = (head_row == tail_row) && (head_flag != tail_flag);
  assign empty_o     = (head_row == tail_row) && (head_flag == tail_flag);
  assign dis_ready_o = !full;
  assign dis_ptr_o   = '{flag: tail_flag, row: tail_row, col: '0};
  assign head_o      = '{flag: head_flag, row: head_row, col: head_col};
`ifndef SYNTHESIS
  // 访存违例置 replay 的 load 必须还没提交 (LSU 违例上报晚到的保护：不得比头项更老)
  always_ff @(posedge clk_i)
    if (rst_ni)
      for (int s = 0; s < N_STA; s++)
        if (viol_valid_i[s])                 // 已提交的项 vld 已清，不能拿 vld 作前提
          assert (!rob_older(viol_rob_i[s], head_o)) else $error("rob: 违例 load %0d/%0d 已提交才收到 replay", viol_rob_i[s].row, viol_rob_i[s].col);
`endif

  //--------------------------------------------------------------------------
  // 最老异常寄存器
  //--------------------------------------------------------------------------
  logic        ox_valid;
  rob_ptr_t    ox_rob;
  logic [63:0] ox_tval;
  logic [63:0] ox_x2;
  logic        dx_v, dx_v_n;          // 分派组非法指令候选 (寄存一拍，见异常候选部分)
  rob_ptr_t    dx_p, dx_p_n;
  logic [63:0] dx_t, dx_t_n;

  //--------------------------------------------------------------------------
  // 提交判定 (组合)
  //--------------------------------------------------------------------------
  rob_ent_t   hent [W];
  logic [W-1:0] hvld;
  always_comb begin
    for (int c = 0; c < W; c++) begin
      hent[c] = ent[head_row][c];
      hvld[c] = vld[head_row][c] && !empty_o;
    end
  end

  wire        head_valid  = hvld[head_col];
  rob_ent_t   head_ent;
  assign      head_ent    = hent[head_col];
  // 本拍刚启动的也算 (started 位要到拍末才写进去)，否则 CSR 写与中断会同拍生效
  // 头项已启动 (CSR / 向量 / MMIO 访问 / AMO —— 只有这几类会给 head_start) 就不接受中断：MMIO load 在地址翻译后才知道是 MMIO，
  //   ROB 项的 wait_head 为 0，原来要求 wait_head 会漏掉它 —— 已读了有副作用的设备 (如 PLIC 认领) 又被中断、返回后重读 (#35)
  wire        head_locked = head_ent.started || head_start_i;
  wire        take_irq    = irq_pending_i && head_valid && !head_locked && !ev_block_i;
  assign      head_go_o   = !irq_pending_i && !ev_block_i;
  always_comb begin
    logic found;
    found = 1'b0;
    head2_o = '{flag: head_flag, row: head_row, col: head_col};
    for (int c = W - 1; c >= 0; c--)
      if (c > int'(head_col) && hvld[c]) begin found = 1'b1; head2_o.col = ROB_COL_W'(c); end
    head2_go_o = found && head_valid && head_ent.fu == FU_VEC && head_ent.done_a && head_ent.done_b &&
                 !head_ent.excp && !head_ent.flush_after && !head_ent.replay;
  end

  logic [W-1:0] commit;
  logic         stop_flush;     // 提交的最后一条要求冲刷
  logic         trap_excp;      // 头项异常
  logic         head_replay;    // 头项要求重执行
  logic [ROB_COL_W-1:0] last_col;
  logic         row_done;       // 头行在本拍提交完

  always_comb begin
    logic go;
    commit     = '0;
    stop_flush = 1'b0;
    trap_excp  = 1'b0;
    head_replay = 1'b0;
    last_col   = head_col;
    go         = !ev_block_i && !take_irq && !flush_i;
    for (int c = 0; c < W; c++) begin
      if (go && c >= head_col) begin
        if (!hvld[c]) begin
          go = 1'b0;
        end else if (!(hent[c].done_a && hent[c].done_b)) begin
          go = 1'b0;
        end else if (hent[c].excp) begin
          if (c == head_col && !dx_v) trap_excp = 1'b1;    // dx_v：非法指令候选还没并进 ox
          go = 1'b0;
        end else if (hent[c].replay) begin
          if (c == head_col) head_replay = 1'b1;
          go = 1'b0;
        end else begin
          commit[c] = 1'b1;
          last_col  = ROB_COL_W'(c);
          if (step_i) go = 1'b0;
          if (hent[c].flush_after) begin
            stop_flush = 1'b1;
            go = 1'b0;
          end
        end
      end
    end
    // 头行最后一个有效项被提交 -> 换行 (行内有效项总是前缀)。最后一个有效项 = 自己有效、右边一列无效，
    // 各列并行判断再或起来 (2026-10-25：原来是按列的优先级链，接在提交链后面，b180u32 head_row -> head_flag 160 ps)
    row_done = 1'b0;
    for (int c = 0; c < W; c++)
      row_done = row_done | (commit[c] && hvld[c] && (c == W - 1 || !hvld[(c == W - 1) ? c : c + 1]));
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i) begin
    logic rd;
    rd = 1'b0;
    for (int c = 0; c < W; c++) if (hvld[c]) rd = commit[c];
    if (rst_ni) assert (rd == row_done) else $error("rob: row_done 并行写法与原写法不一致");
  end
`endif

  //--------------------------------------------------------------------------
  // 提交输出
  //--------------------------------------------------------------------------
  always_comb begin
    cm_fflags_o   = '0;
    cm_fp_dirty_o = 1'b0;
    for (int c = 0; c < W; c++) begin
      cm_valid_o[c]    = commit[c];
      cm_rd_valid_o[c] = commit[c] && hent[c].rd_class != RC_NONE;
      cm_rd_fp_o[c]    = hent[c].rd_class == RC_FP;
      cm_lrd_o[c]      = hent[c].lrd;
      cm_pdst_o[c]     = hent[c].pdst;
      cm_old_pdst_o[c] = hent[c].old_pdst;
      cm_mvel_o[c]     = hent[c].mvel;
      cm_is_ld_o[c]    = commit[c] && hent[c].is_ld;
      cm_is_st_o[c]    = commit[c] && hent[c].is_st;
      cm_is_br_o[c]    = commit[c] && hent[c].is_br;
      cm_ftq_idx_o[c]  = hent[c].ftq_idx;
      if (commit[c]) begin
        cm_fflags_o = cm_fflags_o | hent[c].fflags;
        if (hent[c].rd_class == RC_FP || hent[c].fflags != 5'd0) cm_fp_dirty_o = 1'b1;
      end
    end
  end

  // 事件
  always_comb begin
    rob_ent_t e;
    e = (take_irq || trap_excp || head_replay) ? head_ent : hent[last_col];
    ev_valid_o    = take_irq || trap_excp || head_replay || stop_flush;
    ev_trap_o     = take_irq || trap_excp;
    ev_replay_o   = !take_irq && !trap_excp && head_replay;
    ev_irq_o      = take_irq;
    ev_cause_o    = take_irq ? irq_cause_i : e.cause;
    ev_tval_sel_o = take_irq ? TV_ZERO : e.tval_sel;
    ev_tval_o     = ox_tval;
    ev_tval2_o    = ox_x2;
    ev_ftq_idx_o  = e.ftq_idx;
    ev_ftq_off_o  = e.ftq_off;
    ev_is_rvc_o   = e.is_rvc;
    ev_fu_o       = e.fu;
    ev_op_o       = e.op;
  end

  //--------------------------------------------------------------------------
  // 状态更新
  //--------------------------------------------------------------------------
  function automatic logic [ROB_ROW_W:0] row_inc(input logic flag, input logic [ROB_ROW_W-1:0] row);
    return (row == ROB_ROW_W'(R-1)) ? {~flag, {ROB_ROW_W{1'b0}}} : {flag, row + 1'b1};
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      head_row <= '0; head_flag <= 1'b0; head_col <= '0;
      tail_row <= '0; tail_flag <= 1'b0;
      for (int r = 0; r < R; r++) vld[r] <= '0;
    end else if (flush_i) begin
      // 全部作废：从当前头行的下一行重新开始 (避免与头行残留状态纠缠)
      for (int r = 0; r < R; r++) vld[r] <= '0;
      {head_flag, head_row} <= row_inc(head_flag, head_row);
      {tail_flag, tail_row} <= row_inc(head_flag, head_row);
      head_col <= '0;
    end else begin
      // 提交
      for (int c = 0; c < W; c++)
        if (commit[c]) vld[head_row][c] <= 1'b0;
      if (row_done) begin
        {head_flag, head_row} <= row_inc(head_flag, head_row);
        head_col <= '0;
      end else if (|commit) begin
        head_col <= last_col + 1'b1;
      end

      // 误预测：尾回滚到误预测分支所在行之后，同行更后的列作废
      if (br_upd_i.kill) begin
        {tail_flag, tail_row} <= row_inc(br_upd_i.kill_rob.flag, br_upd_i.kill_rob.row);
        for (int c = 0; c < W; c++)
          if (c > br_upd_i.kill_rob.col) vld[br_upd_i.kill_rob.row][c] <= 1'b0;
      end else if (dis_fire_i) begin
        vld[tail_row] <= dis_valid_i;
        {tail_flag, tail_row} <= row_inc(tail_flag, tail_row);
      end
    end
  end

  // 表项内容
  always_ff @(posedge clk_i) begin
    // 分派写入
    if (dis_fire_i && !flush_i && !br_upd_i.kill) begin
      for (int c = 0; c < W; c++) begin
        rob_ent_t e;
        uop_t u;
        u = dis_uop_i[c];
        e = '0;
        e.done_a      = (u.fu == FU_NONE);      // 译码即异常 / 无需执行 (含被消除的 move)
        e.mvel        = u.mvel;
        e.done_b      = (u.fu != FU_ST);
        e.started     = 1'b0;
        e.excp        = u.excp;
        e.cause       = u.cause;
        e.tval_sel    = !u.excp ? TV_ZERO :
                        (u.cause == o6_isa_pkg::EXC_ILLEGAL_INST ||
                         u.cause == o6_isa_pkg::EXC_VIRT_INST)    ? TV_REG :
                        (u.cause == o6_isa_pkg::EXC_BREAKPOINT)   ? TV_PC  :
                        (u.cause == o6_isa_pkg::EXC_INST_PAGE ||
                         u.cause == o6_isa_pkg::EXC_INST_GPAGE ||
                         u.cause == o6_isa_pkg::EXC_INST_ACCESS)  ? (u.tval_hi ? TV_PC2 : TV_PC) :
                        TV_ZERO;
        e.flush_after = u.flush_after;
        e.replay      = 1'b0;
        e.wait_head   = u.wait_head;
        e.rd_class    = u.rd_class;
        e.lrd         = u.lrd;
        e.pdst        = u.pdst;
        e.old_pdst    = u.old_pdst;
        e.is_ld       = (u.fu == FU_LD);
        e.is_st       = (u.fu == FU_ST) || (u.fu == FU_AMO);   // AMO 也占 SQ 项，提交时要标记
        e.is_br       = u.is_br;
        e.ftq_idx     = u.ftq_idx;
        e.ftq_off     = u.ftq_off;
        e.is_rvc      = u.is_rvc;
        e.fu          = u.fu;
        e.op          = u.op;
        ent[tail_row][c] <= e;
      end
    end
    // 完成
    for (int k = 0; k < N_CMPL; k++) begin
      if (cmpl_i[k].valid) begin
        ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].done_a <= 1'b1;
        if (cmpl_i[k].excp) begin
          ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].excp     <= 1'b1;
          ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].cause    <= cmpl_i[k].cause;
          ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].tval_sel <= TV_REG;
        end
        if (cmpl_i[k].flush_after)
          ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].flush_after <= 1'b1;
        if (cmpl_i[k].replay)
          ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].replay <= 1'b1;
        ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].fflags <= cmpl_i[k].fflags;
      end
    end
    for (int k = 0; k < N_STD; k++)
      if (std_done_i[k]) ent[std_rob_i[k].row][std_rob_i[k].col].done_b <= 1'b1;
    if (head_start_i && head_valid)
      ent[head_row][head_col].started <= 1'b1;
    for (int s = 0; s < N_STA; s++)
      if (viol_valid_i[s])
        ent[viol_rob_i[s].row][viol_rob_i[s].col].replay <= 1'b1;
  end

  //--------------------------------------------------------------------------
  // 最老异常寄存器
  //--------------------------------------------------------------------------
  // 下一值集中在一处算；有效位带复位，内容不带复位 (分两个块，避免"复位当使能")
  //
  // 结构：候选 = 分派组里第一个非法指令 (组内列号小的更老，直接优先编码)
  //            + XCPT_PORTS 指定的完成口；两两比较成树，最后与现值比。
  //   串行逐个比较在 VU7P 上是 63 级逻辑 (实测)，改成树后只有 log2(候选数)+1 级比较。
  // DC 2018 不支持 $countones (哪怕只用于参数)，用常量函数
  function automatic int popc(input logic [N_CMPL-1:0] v);
    int n;
    n = 0;
    for (int i = 0; i < N_CMPL; i++) n += v[i];
    return n;
  endfunction
  localparam int NXP = popc(XCPT_PORTS);
  localparam int NC  = NXP + 1;

  logic        cv [NC];
  rob_ptr_t    cp [NC];
  logic [63:0] ct [NC];
  logic [63:0] ct2 [NC];      // H：guest page fault 的 GPA (只有 LSU 口非 0)

  // 分派组的非法指令候选先寄存一拍 (dis_fire 来自分派级很深的资源判断，不直接进比较树)。
  // 寄存器有效的那拍不允许发起异常陷入，保证 ox_tval 已更新。
  always_comb begin
    dx_v_n = 1'b0; dx_p_n = '0; dx_t_n = '0;
    for (int c = W-1; c >= 0; c--) begin
      if (dis_fire_i && !br_upd_i.kill && dis_valid_i[c] && dis_uop_i[c].excp &&
          (dis_uop_i[c].cause == o6_isa_pkg::EXC_ILLEGAL_INST || dis_uop_i[c].cause == o6_isa_pkg::EXC_VIRT_INST)) begin
        dx_v_n = 1'b1;
        dx_p_n = '{flag: tail_flag, row: tail_row, col: ROB_COL_W'(c)};
        dx_t_n = dis_tval_i[c];
      end
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)      dx_v <= 1'b0;
    else if (flush_i) dx_v <= 1'b0;
    else              dx_v <= dx_v_n;
  end
  always_ff @(posedge clk_i) begin
    dx_p <= dx_p_n;
    dx_t <= dx_t_n;
  end

  always_comb begin
    int n;
    // 分派 (上拍寄存的候选)
    cv[0] = dx_v && !(br_upd_i.kill && rob_older(br_upd_i.kill_rob, dx_p));
    cp[0] = dx_p;
    ct[0] = dx_t;
    ct2[0] = '0;
    // 完成口
    n = 1;
    for (int k = 0; k < N_CMPL; k++) begin
      if (XCPT_PORTS[k]) begin
        cv[n] = cmpl_i[k].valid && cmpl_i[k].excp &&
                !(br_upd_i.kill && rob_older(br_upd_i.kill_rob, cmpl_i[k].rob_idx));
        cp[n] = cmpl_i[k].rob_idx;
        ct[n] = cmpl_i[k].tval;
        ct2[n] = cmpl_x2_i[k];
        n++;
      end
    end
  end

  // 最老异常：全对比较 (2026-10-25，6 GHz)。原来 7 个候选两两比较的锦标赛树 3 层 + 再与 ox_rob 比一次，
  // 4 次 "ROB 指针年龄比较 -> 选择" 串联 (b180：dx_p -> ox_tval 206 ps)。现在 ox 与各候选共 NC + 1 项一次两两比较，
  // 胜者 = 对所有其它有效项 "更老，或同龄而优先" (ox 最优先、候选按编号)；与原树逐拍一致 (下面断言)
  localparam int NA = NC + 1;                  // [0] = 现有 ox，[1..NC] = 候选
  logic        ox_v_n;
  rob_ptr_t    ox_p_n;
  logic [63:0] ox_t_n, ox_x2_n;
  always_comb begin
    logic        av [NA];
    rob_ptr_t    ap [NA];
    logic [NA-1:0] win;
    av[0] = ox_valid && !(br_upd_i.kill && rob_older(br_upd_i.kill_rob, ox_rob));
    ap[0] = ox_rob;
    for (int i = 0; i < NC; i++) begin av[i+1] = cv[i]; ap[i+1] = cp[i]; end
    for (int i = 0; i < NA; i++) begin
      win[i] = av[i];
      for (int j = 0; j < NA; j++)
        if (j != i) begin
          logic beat;
          beat = !av[j] || rob_older(ap[i], ap[j]) || (!rob_older(ap[j], ap[i]) && i < j);
          win[i] = win[i] && beat;
        end
    end
    ox_v_n = |win;
    ox_p_n = '0; ox_t_n = '0; ox_x2_n = '0;
    for (int i = 0; i < NA; i++) begin
      ox_p_n  = ox_p_n  | ({$bits(rob_ptr_t){win[i]}} & ap[i]);
      ox_t_n  = ox_t_n  | ({64{win[i]}} & ((i == 0) ? ox_tval : ct[(i == 0) ? 0 : i - 1]));
      ox_x2_n = ox_x2_n | ({64{win[i]}} & ((i == 0) ? ox_x2 : ct2[(i == 0) ? 0 : i - 1]));
    end
    if (!(|win)) begin ox_p_n = ox_rob; ox_t_n = ox_tval; ox_x2_n = ox_x2; end
    if (flush_i) ox_v_n = 1'b0;
  end

`ifndef SYNTHESIS
  // 参考模型：原锦标赛树 + 与 ox 比较
  localparam int NL = (NC <= 1) ? 1 : $clog2(NC) + 1;
  logic        tv [NL][NC];
  rob_ptr_t    tp [NL][NC];
  logic [63:0] tt [NL][NC];
  always_comb begin
    for (int i = 0; i < NC; i++) begin tv[0][i] = cv[i]; tp[0][i] = cp[i]; tt[0][i] = ct[i]; end
    for (int l = 1; l < NL; l++) begin
      for (int i = 0; i < NC; i++) begin
        int a, b;
        a = 2*i; b = 2*i + 1;
        tv[l][i] = 1'b0; tp[l][i] = '0; tt[l][i] = '0;
        if (a < NC) begin
          logic pick_b;
          pick_b = (b < NC) && tv[l-1][b] && (!tv[l-1][a] || rob_older(tp[l-1][b], tp[l-1][a]));
          tv[l][i] = tv[l-1][a] || ((b < NC) && tv[l-1][b]);
          tp[l][i] = pick_b ? tp[l-1][b] : tp[l-1][a];
          tt[l][i] = pick_b ? tt[l-1][b] : tt[l-1][a];
        end
      end
    end
  end
  always_ff @(posedge clk_i)
    if (rst_ni) begin
      logic keep, rv;
      rob_ptr_t rp;
      logic [63:0] rt;
      keep = ox_valid && !(br_upd_i.kill && rob_older(br_upd_i.kill_rob, ox_rob));
      rv = keep; rp = ox_rob; rt = ox_tval;
      if (tv[NL-1][0] && (!keep || rob_older(tp[NL-1][0], ox_rob))) begin rv = 1'b1; rp = tp[NL-1][0]; rt = tt[NL-1][0]; end
      if (flush_i) rv = 1'b0;
      assert (rv == ox_v_n && (!rv || (rp == ox_p_n && rt == ox_t_n)))
        else $error("rob: 最老异常全对比较与锦标赛树不一致 v=%0d/%0d", ox_v_n, rv);
    end
`endif

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ox_valid <= 1'b0;
    else         ox_valid <= ox_v_n;
  end
  always_ff @(posedge clk_i) begin
    ox_rob  <= ox_p_n;
    ox_tval <= ox_t_n;
    ox_x2   <= ox_x2_n;
  end

`ifndef SYNTHESIS
  // 仅仿真：每项的原始编码 (对拍 / 调试打印用)
  logic [31:0] dbg_raw [R][W];
  always_ff @(posedge clk_i)
    if (dis_fire_i && !flush_i && !br_upd_i.kill)
      for (int c = 0; c < W; c++) dbg_raw[tail_row][c] <= dis_uop_i[c].raw;

  // 时序契约 (形式 F6 查出的隐含前提)：误预测广播那拍，被杀分支自己还没完成。
  //   否则它可能在被杀那拍提交，同行更后的项被杀，row_done 按杀前的有效位算成「没完」-> 头行空、head 不前进 -> 死锁。
  //   现在 BRU 的完成与 br_upd 都在 EX 后寄存一拍，完成再过一拍才写进 ROB，所以成立；改动完成时序时这里会报
  always_ff @(posedge clk_i) if (rst_ni && !flush_i && br_upd_i.kill)
    assert (!ent[br_upd_i.kill_rob.row][br_upd_i.kill_rob.col].done_a)
      else $error("rob: 误预测分支 row=%0d col=%0d 在广播 kill 那拍已完成", br_upd_i.kill_rob.row, br_upd_i.kill_rob.col);

  // 完成写到无效项 = 有人用了作废 / 已提交的 ROB 号
  always_ff @(posedge clk_i) if (rst_ni && !flush_i) begin
    for (int k = 0; k < N_CMPL; k++)
      if (cmpl_i[k].valid && !br_upd_i.kill)
        // 还要在窗口内：误预测只把 tail 退回、不清 tail 之后各行的 vld，被杀项的迟到完成单看 vld 查不出 (形式 F6，2026-10-30)
        assert (vld[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col] && !rob_older(cmpl_i[k].rob_idx, head_o) &&
                (full || rob_older(cmpl_i[k].rob_idx, '{flag: tail_flag, row: tail_row, col: '0})))   // 满时 tail = head 行，不比
          else $error("rob: 完成口 %0d 写入无效项 row=%0d col=%0d", k, cmpl_i[k].rob_idx.row, cmpl_i[k].rob_idx.col);
  end
  // 一项只完成一次 (形式 F6 的前提，2026-10-30)：已完成的项再收到 (带异常的) 完成，会在同拍按旧 done 提交、越过最老异常
  always_ff @(posedge clk_i) if (rst_ni && !flush_i) begin
    for (int k = 0; k < N_CMPL; k++)
      if (cmpl_i[k].valid && !(br_upd_i.kill && rob_older(br_upd_i.kill_rob, cmpl_i[k].rob_idx)))
        assert (!ent[cmpl_i[k].rob_idx.row][cmpl_i[k].rob_idx.col].done_a)
          else $error("rob: 完成口 %0d 写入已完成项 row=%0d col=%0d", k, cmpl_i[k].rob_idx.row, cmpl_i[k].rob_idx.col);
  end
`endif
endmodule
