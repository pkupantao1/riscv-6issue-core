//==============================================================================
// o6_commit_ctrl.sv - 提交级事件处理：陷入 / 中断 / flush_after / 重执行
//
// 时序：t 拍 ROB 给出事件 (组合) -> 寄存 -> t+1 拍：读 FTQ 得到 PC，
//   陷入时驱动 CSR 陷入口并取陷入向量；发全局 flush 与前端重定向。
//   FENCE.I 需要先等 I$ 作废 / D$ 写回完成 (M1 理想存储下立即完成)。
// 处理期间 ev_block 拉高，ROB 不再提交。
//
// 调试 (Debug 0.13.2)：
//   - 伪中断 IRQ_DBG_HALT / IRQ_DBG_STEP：进入调试模式 (dpc = 该指令 PC，未执行)，跳 DBG_ENTRY
//   - ebreak 且 dcsr.ebreak{m,s,u}：进入调试模式 (dpc = ebreak 的 PC)
//   - 调试模式下：ebreak 跳 DBG_ENTRY，其它异常跳 DBG_EXCP；都不更新任何 CSR
//   - DRET：CSR 执行口已切回原特权级，这里冲刷并跳 dpc
//==============================================================================
`default_nettype none
module o6_commit_ctrl
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // ROB 事件
  input  wire                 ev_valid_i,
  input  wire                 ev_trap_i,
  input  wire                 ev_replay_i,
  input  wire                 ev_irq_i,
  input  wire  [5:0]          ev_cause_i,
  input  wire  [63:0]         ev_tval_i,
  input  wire  [1:0]          ev_tval_sel_i,
  input  wire  [63:0]         ev_tval2_i,       // H：ROB 最老异常的 GPA ({隐式, 6'b0, GPA[58:2]})
  input  wire  [63:0]         fgx_i,            // H：取指 guest page fault 的 GPA (前端锁存)
  input  wire  [63:0]         vgx_i,            // H：向量访存 guest page fault 的 GPA (LSU 锁存)
  input  wire  [FTQ_W-1:0]    ev_ftq_idx_i,
  input  wire  [FTQ_OFF_W-1:0] ev_ftq_off_i,
  input  wire                 ev_is_rvc_i,
  input  var fu_e             ev_fu_i,
  input  wire  [4:0]          ev_op_i,
  output wire                 ev_block_o,

  // FTQ 读
  output logic [FTQ_W-1:0]    ftq_ridx_o,
  input  wire  [63:0]         ftq_pc_i,

  // CSR
  output logic                trap_o,
  output logic                trap_irq_o,
  output logic [5:0]          trap_cause_o,
  output logic [63:0]         trap_epc_o,
  output logic [63:0]         trap_tval_o,
  output logic                trap_hv_o,        // H：出错的是 HLV / HSV (CSR 算 GVA 用)
  output logic [63:0]         trap_tval2_o,     // H：htval / mtval2 = GPA >> 2 (guest page fault，其它 0)
  output logic [63:0]         trap_tinst_o,     // H：htinst / mtinst (VS PTE 地址的 guest page fault 写 0x3000，其它 0；同 Spike)
  input  wire  [63:0]         trap_vec_i,
  input  wire  [63:0]         mepc_i,
  input  wire  [63:0]         sepc_i,

  // 调试
  input  wire                 dbg_mode_i,
  input  wire                 ebreak_dbg_i,
  input  wire  [63:0]         dpc_i,
  output logic                dbg_enter_o,
  output logic [2:0]          dbg_cause_o,

  // 缓存 / TLB 维护
  output logic                fencei_req_o,
  input  wire                 fencei_done_i,
  output logic                sfence_o,
  input  wire                 sq_drained_i,     // 已提交的 store 全部写进 D$ (SFENCE.VMA 要等)

  input  wire                 ptw_busy_i,       // 重执行要等页表遍历结束 (否则会再次缺失)

  // 冲刷与重定向
  output logic                flush_o,
  output logic [63:0]         redirect_pc_o,
  output logic                lr_clear_o
);
  // S_EV0：事件锁存后多等一拍 —— 核里的提交包 (事件同拍及之前提交的指令) 要先落地，冲刷才发
  typedef enum logic [1:0] {S_IDLE, S_EV0, S_EV, S_FENCEI} st_e;
  st_e st;

  logic                 q_trap, q_replay, q_irq, q_rvc;
  logic [5:0]           q_cause;
  logic [63:0]          q_tval, q_x2;
  logic [1:0]           q_tsel;
  logic [FTQ_W-1:0]     q_idx;
  logic [FTQ_OFF_W-1:0] q_off;
  fu_e                  q_fu;
  logic [4:0]           q_op;

  assign ev_block_o = (st != S_IDLE);
  assign ftq_ridx_o = q_idx;

  wire [63:0] pc     = ftq_pc_i + {{(63-FTQ_OFF_W){1'b0}}, q_off, 1'b0};
  wire [63:0] pc_nxt = pc + (q_rvc ? 64'd2 : 64'd4);
  wire        is_fencei = (q_fu == FU_CSR) && (q_op == SYS_FENCEI);
  wire        is_mret   = (q_fu == FU_CSR) && (q_op == SYS_MRET);
  wire        is_sret   = (q_fu == FU_CSR) && (q_op == SYS_SRET);
  wire        is_sfence = (q_fu == FU_CSR) && (q_op == SYS_SFENCE || q_op == SYS_HFENCEV || q_op == SYS_HFENCEG);   // H：hfence 同 sfence
  wire        is_dret   = (q_fu == FU_CSR) && (q_op == SYS_DRET);
  wire        dbg_irq   = q_irq && (q_cause == IRQ_DBG_HALT || q_cause == IRQ_DBG_STEP);
  wire        is_bkpt   = q_trap && !q_irq && (q_cause == o6_isa_pkg::EXC_BREAKPOINT);

  // 处理拍的组合输出
  always_comb begin
    trap_o       = 1'b0;
    trap_irq_o   = q_irq;
    trap_cause_o = q_cause;
    trap_epc_o   = pc;
    trap_hv_o    = (q_fu == FU_LD || q_fu == FU_ST) && q_op[0];
    begin
      logic        gpf;
      logic [63:0] x;
      gpf = !q_irq && (q_cause == o6_isa_pkg::EXC_INST_GPAGE || q_cause == o6_isa_pkg::EXC_LOAD_GPAGE ||
                       q_cause == o6_isa_pkg::EXC_STORE_GPAGE);
      x   = (q_cause == o6_isa_pkg::EXC_INST_GPAGE) ? fgx_i : (q_fu == FU_VEC) ? vgx_i : q_x2;
      trap_tval2_o = gpf ? {7'b0, x[56:0]} : 64'd0;
      trap_tinst_o = (gpf && x[63]) ? 64'h3000 : 64'd0;
    end
    unique case (q_tsel)
      2'd0:    trap_tval_o = q_tval;
      2'd1:    trap_tval_o = pc;
      2'd2:    trap_tval_o = pc + 64'd2;
      default: trap_tval_o = 64'd0;
    endcase
    flush_o       = 1'b0;
    redirect_pc_o = pc_nxt;
    fencei_req_o  = 1'b0;
    sfence_o      = 1'b0;
    lr_clear_o    = 1'b0;
    dbg_enter_o   = 1'b0;
    dbg_cause_o   = 3'd3;
    if (st == S_EV) begin
      if (dbg_irq) begin
        // 进入调试模式 (haltreq / 单步)
        dbg_enter_o = 1'b1; dbg_cause_o = (q_cause == IRQ_DBG_HALT) ? 3'd3 : 3'd4;
        flush_o = 1'b1; redirect_pc_o = DBG_ENTRY; lr_clear_o = 1'b1;
      end else if (q_trap && dbg_mode_i) begin
        // 调试模式下的异常：回调试 ROM，不动 CSR
        flush_o = 1'b1; redirect_pc_o = is_bkpt ? DBG_ENTRY : DBG_EXCP;
      end else if (is_bkpt && ebreak_dbg_i) begin
        dbg_enter_o = 1'b1; dbg_cause_o = 3'd1;
        flush_o = 1'b1; redirect_pc_o = DBG_ENTRY; lr_clear_o = 1'b1;
      end else if (q_trap) begin
        trap_o = 1'b1; flush_o = 1'b1; redirect_pc_o = trap_vec_i; lr_clear_o = 1'b1;
      end else if (q_replay) begin
        flush_o = !ptw_busy_i; redirect_pc_o = pc;
      end else if (is_fencei) begin
        fencei_req_o = 1'b1;
        if (fencei_done_i) flush_o = 1'b1;
      end else if (is_sfence && !sq_drained_i) begin
        // SFENCE.VMA 要求之前的 store 先于之后的隐式页表读：等 SQ 里已提交的 store 排空
        flush_o = 1'b0;
      end else begin
        flush_o  = 1'b1;
        sfence_o = is_sfence;
        if (is_mret) redirect_pc_o = mepc_i;
        if (is_sret) redirect_pc_o = sepc_i;
        if (is_dret) redirect_pc_o = dpc_i;
      end
    end else if (st == S_FENCEI) begin
      fencei_req_o = 1'b1;
      if (fencei_done_i) flush_o = 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st <= S_IDLE;
    end else begin
      unique case (st)
        S_IDLE:   if (ev_valid_i) st <= S_EV0;
        S_EV0:    st <= S_EV;
        S_EV:     st <= flush_o ? S_IDLE : (is_fencei && !q_trap && !q_replay && !q_irq) ? S_FENCEI : S_EV;
        S_FENCEI: if (flush_o) st <= S_IDLE;
        default:  st <= S_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk_i) begin
    if (st == S_IDLE && ev_valid_i) begin
      q_trap <= ev_trap_i; q_replay <= ev_replay_i; q_irq <= ev_irq_i; q_rvc <= ev_is_rvc_i;
      q_cause <= ev_cause_i; q_tval <= ev_tval_i; q_tsel <= ev_tval_sel_i; q_x2 <= ev_tval2_i;
      q_idx <= ev_ftq_idx_i; q_off <= ev_ftq_off_i; q_fu <= ev_fu_i; q_op <= ev_op_i;
    end
  end
endmodule
