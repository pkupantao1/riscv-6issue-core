//==============================================================================
// o6_lsu.sv - 访存单元：LQ(64) / SQ(64) / 两条 load 流水 / STA / 排空 / MMIO / AMO
//
// 流水：k = 0..N_LDP-1 为 load 流水 (P4..)，k = LSU_KS..LSU_NP-1 为 N_STA 条 STA 流水 (STA / AMO 地址，P_STA..)。
// 违例检查每条 STA 流水各做一份、各自报 ROB (同拍两条都查出也不会丢)。
// (2026-10-13 前 STA 与第 2 条 load 流水共用；拆开后 load 与 STA 同拍都能发)
// 各流水同构：
//   IS -> RF (读 rs1 + 旁路) -> AG (再旁路一次 + 地址加法，虚地址寄存)
//      -> L0 (DTLB、PMA、未对齐、写 LQ/SQ 地址、D$ 请求、违例检查 (结果寄存一拍再报 ROB))
//      -> L1 (SQ 按字节转发搜索、D$ 读) -> L2 (判定：成功/等 store/缺失/MMIO/异常；对齐、符号扩展、NaN-boxing)
//      -> wbreg (写 PRF、完成)
//
// 唤醒 (整数结果)：L0 级按"会成功"投机唤醒 (iwake_o，带 sa)，两拍后 L2 判定失败时 ld_fail_o 取消 (见 o6_iq)；
//   L2 成功时另发确认唤醒 cwake_o (给重命名的 busy 表与 IQ 入队口)。浮点结果仍在 L2 唤醒 (不投机)。
//   消费者在 load 的 L1 拍被选中、L2 拍 RF、下一拍 EX 从 wbreg 旁路：load-to-use 4 拍
//   (2026-10-13 起 L1 唤醒 5 拍；2026-10-14 L3 并进 L2、唤醒提前到 L0)。
//   被毒化的 uop 在 RF -> 下一级处丢弃：poia (L2 判定在它 RF 那拍) 用 ld_fail_o，pois 用寄存的 ld_fail_q_i。
// 重放：LQ 里等待条件已满足的项 (缺失回填、等待的 store 有数据/已排空、D$ 拒绝)，
//   以及 ROB 头的 MMIO load / AMO 的结果，都从端口 0 的 RF 级重新进入流水
//   (该拍屏蔽 IQ 在 P4 的发射)。带数据的重放 (MMIO/AMO) 在 L2 直接成功。
//
// 转发：每个字节取"比 load 老、地址已知、覆盖该字节"的最年轻 store。
//   该 store 数据未到 / 是 AMO / 是 MMIO -> load 等它 (数据到或排空后重放)。
//   地址未知的老 store 不等 (投机)，由 STA 时的违例检查兜底。
//
// 违例：STA 在 L0 拿到地址时，检查比它新、地址已知、字节重叠的 load，
//   给最老的一个在 ROB 里置 replay (到头时不提交、从它重取)。
//   "地址已知"而非"已取到数据"：同拍正在 L1 搜索的 load 看不到这个 store，必须算进去。
//
// 头部串行：MMIO load (所有已提交 store 排空后) 与 AMO/LR/SC (SQ 头就是它) 只在 ROB 头执行。
// 提交后的 store 按序排空：可缓存的写 D$，MMIO 的走 MMIO 口。
//==============================================================================
`default_nettype none
module o6_lsu
  import o6_pkg::*;
#(
  parameter logic [63:0] DRAM_SIZE = 64'h8000_0000
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 dbg_mode_i,       // 调试模式：调试模块 0x000-0xFFF 可访问 (MMIO)
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,

  // 分派分配
  input  wire  [W-1:0]        lq_alloc_i,
  input  wire  [W-1:0]        sq_alloc_i,
  input  var uop_t [W-1:0]    alloc_uop_i,
  output lq_ptr_t             lq_tail_o,
  output sq_ptr_t             sq_tail_o,
  output logic [LQ_W:0]       lq_free_o,
  output logic [SQ_W:0]       sq_free_o,

  // 发射 (IQ_M)
  input  wire  [LSU_NP-1:0]   iss_valid_i,      // [0..N_LDP-1] 来自 IQ_M，[LSU_KS..] 来自 IQ_S
  input  var uop_t [LSU_NP-1:0] iss_uop_i,
  output logic [15:0]         fu_ok_o [LSU_NP],
  output logic [15:0]         fu_ok_nx_o [LSU_NP],  // 下一拍的 fu_ok：端口 0 的重放占用按 rp_q_v 的下一值

  // PRF 读与旁路
  output preg_t [LSU_NP-1:0]  prf_raddr_o,
  input  wire  [63:0]         prf_rdata_i [LSU_NP],
  input  var wb_t [N_IWB-1:0] iwb_i,

  // STD (0: 整数 P3，1: 浮点 P7)
  input  wire  [N_STD-1:0]    std_valid_i,
  input  var sq_ptr_t [N_STD-1:0] std_sq_i,
  input  wire  [63:0]         std_data_i [N_STD],

  // 写回 / 唤醒 / 完成
  output wb_t  [N_LDP-1:0]    iwb_o,          // 各 load 流水的整数写回
  output preg_t [N_LDP-1:0]   iwb_pn_o,       // 各 load 流水整数写回下一拍的目的号 (= iwb_o.pdst 的 D 端，给整数执行单元提前比较旁路)
  output wb_t  [N_LDP-1:0]    fwb_o,          // 各 load 流水的浮点写回
  output wakeup_t [N_LDP-1:0] iwake_o,         // L0 投机唤醒 (整数)
  output wakeup_t [N_LDP-1:0] ewake_o,
  output lwake_t  [N_LDP-1:0] lwake_o,         // AG 级 load -> load 快速唤醒 (只给 IQ_M)         // L1 投机唤醒 (只给 IQ 入队口：补上 L0 唤醒与 L2 确认之间那一拍入队的)
  output wakeup_t [N_LDP-1:0] cwake_o,         // L2 确认唤醒 (整数)
  output logic [N_LDP-1:0]    ld_fail_o,       // 本拍 L2：上拍投机唤醒的 load 没有成功
  input  wire  [N_LDP-1:0]    ld_fail_q_i,     // ld_fail_o 寄存一拍 (RF 级丢弃被毒化的 uop)
  output wakeup_t [N_LDP-1:0] fwake_o,
  output cmpl_t [LSU_NP-1:0]  cmpl_o,
  output logic [63:0]         cmpl_x2_o [LSU_NP],   // 与 cmpl_o 同拍：guest page fault 的 GPA (ROB 最老异常一起记)
  // store 预取 (STA 地址确定时)
  output logic                dc_pf_req_o,
  output logic [PA_W-1:0]     dc_pf_paddr_o,
  output logic [N_STA-1:0]    viol_valid_o,        // 各 STA 流水各一个 (给 ROB 置 replay)
  output rob_ptr_t [N_STA-1:0] viol_rob_o,
  output logic                viol_any_o,          // 有违例 (前端训练访存相关预测，取其中一个的位置)
  output logic [FTQ_W-1:0]    viol_ftq_o,          // 违例 load 的取指位置
  output logic [FTQ_OFF_W-1:0] viol_off_o,

  // 提交
  input  wire  [2:0]          cm_ld_cnt_i,
  input  wire  [2:0]          cm_st_cnt_i,

  // ROB 头
  input  var rob_ptr_t        rob_head_i,
  input  wire                 head_go_i,
  output logic                head_start_o,

  // D$ load 口 (k = 0..N_LDP-1)：L0 请求，L1 给物理地址/取消，L2 返回
  output logic [N_LDP-1:0]    dc_ld_req_o,
  output logic [PA_W-1:0]     dc_ld_addr_o [N_LDP],    // L0：虚拟地址 (索引)；裸机 = 物理地址
  output logic [PA_W-1:0]     dc_ld_paddr_o [N_LDP],   // L1
  output logic [N_LDP-1:0]    dc_ld_kill_o,            // L1
  input  wire  [N_LDP-1:0]    dc_ld_hit_i,             // L2
  input  wire  [N_LDP-1:0]    dc_ld_nack_i,            // L2：bank 冲突 / MSHR 满，稍后重放
  input  wire  [63:0]         dc_ld_data_i [N_LDP],    // L2：8 字节对齐的双字
  input  wire  [MSHR_W-1:0]   dc_ld_mshr_i [N_LDP],    // L2：缺失时分配的 MSHR 号
  input  wire                 dc_refill_i,         // 某个 MSHR 回填完成
  input  wire  [MSHR_W-1:0]   dc_refill_mshr_i,

  // D$ store / AMO 口
  output logic                dc_st_req_o,
  output logic [PA_W-1:0]     dc_st_paddr_o,
  output logic [PA_W-1:0]     dc_st_next_paddr_o,   // 下一条要排空的 store 的地址 (D$ 在 done 拍预查标签)
  output logic [PA_W-1:0]     dc_st_lk_paddr_o,     // 只由寄存器决定的查表地址 (D$ 查标签 / 动作拍核对用)
  output logic [63:0]         dc_st_data_o,        // 已按双字对齐
  output logic [7:0]          dc_st_mask_o,
  output logic                dc_st_amo_o,         // 1 = AMO/LR (读改写，返回旧值)
  output logic [4:0]          dc_st_amo_op_o,
  output logic [1:0]          dc_st_size_o,
  input  wire                 dc_st_done_i,        // 本请求完成 (可与 req 同拍)
  input  wire  [63:0]         dc_st_rdata_i,       // AMO 旧值 (双字对齐)
  output logic                dc_st_line_o,        // 整行模式 (向量访存)
  output logic [511:0]        dc_st_ldata_o,
  output logic [63:0]         dc_st_lmask_o,
  input  wire  [511:0]        dc_st_lrdata_i,

  // MMIO 口
  output logic                mmio_req_o,
  output logic                mmio_we_o,
  output logic [PA_W-1:0]     mmio_addr_o,
  output logic [1:0]          mmio_size_o,
  output logic [63:0]         mmio_wdata_o,        // 已按双字对齐
  output logic [7:0]          mmio_wmask_o,
  input  wire                 mmio_ack_i,
  input  wire  [63:0]         mmio_rdata_i,        // 双字对齐

  // 状态
  output logic                sq_drained_o,        // 没有已提交未排空的 store
  // 向量单元的访存口 (ROB 头的向量访存，每次一整行 64 B)：请求保持到 vm_ack (收下)，结果按序回 vm_done；
  // 两级流水 (T 翻译 / A 访问)，最多两行在途
  input  wire                 vm_req_i,
  output logic                vm_ack_o,
  input  wire                 vm_we_i,
  input  wire  [63:0]         vm_va_i,             // 本次第一个活动字节的虚地址 (行 = [63:6])
  input  wire  [511:0]        vm_wdata_i,          // 整行 (按行内字节位置)
  input  wire  [63:0]         vm_wmask_i,
  output logic                vm_done_o,           // 单拍
  output logic [511:0]        vm_rdata_o,
  output logic                vm_fault_o,          // 页错误 / 访问错误 (不访存)
  output logic [5:0]          vm_cause_o,
  input  wire                 lr_clear_i,          // 陷入时清 LR 保留

  // MMU：地址转换上下文
  // 翻译上下文 (o6_csr 算好)：普通访存用 dctx (含 MPRV / MPV)，HLV / HSV 用 hctx (H：V = 1、特权级 hstatus.SPVP)
  input  xctx_t               dctx_i,
  input  xctx_t               hctx_i,
  input  wire                 tlb_flush_i,
  input  pmp_dec_t            pmpd_i    [8],      // PMP 预译码 (o6_core 寄存)
  // DTLB <-> PTW
  output logic                ptw_dreq_o,
  output logic [VPN_W-1:0]    ptw_dvpn_o,
  output logic                ptw_dv_o,            // 这次请求的 V (TLB 项标记 / PTW 选 vsatp)
  input  wire                 dfill_i,
  input  wire                 fill_v_i,
  input  wire  [VPN_W-1:0]    fill_vpn_i,
  input  wire  [2:0]          fill_level_i,
  input  wire                 fill_nap_i,

  input  wire  [PA_W-13:0]    fill_ppn_i,
  input  wire  [7:0]          fill_perm_i,
  input  wire  [1:0]          fill_fault_i,
  input  wire  [3:0]          fill_gperm_i,
  input  wire  [56:0]         fill_gpa_i,
  input  wire                 fill_gimp_i,
  output logic [63:0]         vm_x2_o,             // H：向量访存 guest page fault 的 GPA (同 x2 格式；出错时锁存)
  // PTW 读 PTE (借用 D$ 的 store/AMO 口)
  input  wire                 ptw_mem_req_i,
  input  wire  [PA_W-1:0]     ptw_mem_addr_i,
  output logic                ptw_mem_gnt_o,
  output logic                ptw_mem_valid_o,
  output logic [63:0]         ptw_mem_data_o
);
  localparam int NLQ = LQ_SIZE;
  localparam int NSQ = SQ_SIZE;

  //==========================================================================
  // PMA
  //==========================================================================
  function automatic logic [1:0] pma(input logic [63:0] a, input logic dbg);   // {ok, mmio}
    if (dbg && a < 64'h1000)                                  return 2'b11;
    if (a >= DRAM_BASE && a < DRAM_BASE + DRAM_SIZE)          return 2'b10;
    if (a >= BOOTROM_BASE && a < BOOTROM_BASE + 64'h1_0000)   return 2'b11;
    if (a >= CLINT_BASE && a < CLINT_BASE + 64'h1_0000)       return 2'b11;
    if (a >= PLIC_BASE && a < PLIC_BASE + 64'h400_0000)       return 2'b11;
    if (a >= UART_BASE && a < UART_BASE + 64'h1000)           return 2'b11;
    return 2'b00;
  endfunction

  // uop 的翻译上下文：HLV / HSV (FU_LD / FU_ST 且 op[0]) 用 hctx，其余 dctx；HLVX 不做指针掩码 (Spike get_pmlen)
  function automatic xctx_t uctx(input uop_t u, input xctx_t d, input xctx_t h);
    xctx_t c;
    c = ((u.fu == FU_LD || u.fu == FU_ST) && u.op[0]) ? h : d;
    if (u.fu == FU_LD && u.op == LS_HLVX) c.pmm = 2'b00;
    return c;
  endfunction
  function automatic logic is_hlvx(input uop_t u);
    return u.fu == FU_LD && u.op == LS_HLVX;
  endfunction

  function automatic logic [7:0] bmask(input logic [2:0] off, input logic [1:0] sz);
    logic [7:0] m;
    unique case (sz)
      2'd0: m = 8'h01;
      2'd1: m = 8'h03;
      2'd2: m = 8'h0F;
      default: m = 8'hFF;
    endcase
    return m << off;
  endfunction

  //==========================================================================
  // LQ / SQ 表项
  //==========================================================================
  typedef enum logic [2:0] {
    LS_NEW,       // 已分配，还没走过流水
    LS_PIPE,      // 在流水里
    LS_WAIT,      // 等条件
    LS_READY,     // 条件满足，等重放
    LS_DONE       // 已取到数据 (或异常完成)
  } lst_e;

  typedef enum logic [2:0] {
    WK_SQ, WK_MSHR, WK_HEAD, WK_NONE,
    WK_SQA        // 等某条更早 store 的地址确定 (访存相关预测)
  } wk_e;

  typedef struct packed {
    logic             flag;       // 分配时的指针翻转位
    rob_ptr_t         rob;
    br_mask_t         br_mask;
    logic [1:0]       sz;
    logic             uns;
    rclass_e          rd_class;
    logic             fp_dbl;
    preg_t            pdst;
    sq_ptr_t          sq_bound;   // 比它老的 store 在这之前
    lst_e             st;
    wk_e              wk;
    sq_ptr_t          wk_sq;
    logic [MSHR_W-1:0] wk_mshr;
    logic             addr_ok;
    logic [PA_W-1:0]  paddr;
    logic [7:0]       mask;
    logic             mmio;
    logic             has_data;   // MMIO 读回的数据已在 data 里
    logic [63:0]      data;
    logic             ld_wait;    // 访存相关预测标记 (重放时恢复到 uop)
    logic [FTQ_W-1:0]     ftq_idx;
    logic [FTQ_OFF_W-1:0] ftq_off;
  } lq_ent_t;

  typedef struct packed {
    logic             flag;
    rob_ptr_t         rob;
    br_mask_t         br_mask;
    logic [1:0]       sz;
    logic             is_amo;     // AMO / LR / SC
    logic [4:0]       amo_op;
    preg_t            pdst;       // AMO 的目的寄存器
    rclass_e          rd_class;
    lq_ptr_t          lq_bound;   // 比它新的 load 从这里开始
    logic             addr_ok;
    logic             fault;      // STA 有异常 (L1 写入)：这条会在提交时陷入，AMO 不得在头部执行
    logic             fchk;       // fault 已写好 (地址在 L0 先置 addr_ok，异常晚一拍才知道)
    logic [PA_W-1:0]  paddr;
    logic [7:0]       mask;
    logic             mmio;
    logic             data_ok;
    logic [63:0]      data;       // 原始数据 (低位对齐)
    logic             committed;
    logic             amo_done;
  } sq_ent_t;

  lq_ent_t           lq [NLQ];
  logic [NLQ-1:0]    lqv;
  sq_ent_t           sq [NSQ];
  logic [NSQ-1:0]    sqv;
  lq_ptr_t           lq_head, lq_tail;
  sq_ptr_t           sq_head, sq_cmt, sq_tail;

  assign lq_tail_o = lq_tail;
  assign sq_tail_o = sq_tail;

  function automatic logic [LQ_W:0] lq_cnt(input lq_ptr_t t, input lq_ptr_t h);
    return (t.flag == h.flag) ? {1'b0, t.idx} - {1'b0, h.idx} : {1'b0, t.idx} + NLQ - {1'b0, h.idx};
  endfunction
  function automatic logic [SQ_W:0] sq_cnt(input sq_ptr_t t, input sq_ptr_t h);
    return (t.flag == h.flag) ? {1'b0, t.idx} - {1'b0, h.idx} : {1'b0, t.idx} + NSQ - {1'b0, h.idx};
  endfunction
  function automatic lq_ptr_t lq_inc(input lq_ptr_t p, input logic [3:0] k);
    logic [LQ_W:0] s;
    s = {1'b0, p.idx} + k;
    return '{flag: p.flag ^ s[LQ_W], idx: s[LQ_W-1:0]};
  endfunction
  function automatic sq_ptr_t sq_inc(input sq_ptr_t p, input logic [3:0] k);
    logic [SQ_W:0] s;
    s = {1'b0, p.idx} + k;
    return '{flag: p.flag ^ s[SQ_W], idx: s[SQ_W-1:0]};
  endfunction

  assign lq_free_o = (LQ_W+1)'(NLQ) - lq_cnt(lq_tail, lq_head);
  assign sq_free_o = (SQ_W+1)'(NSQ) - sq_cnt(sq_tail, sq_head);
  // 提交包寄存了一拍 (o6_core 的 cmq_st_cnt)：store 在 ROB 提交后的下一拍 sq_cmt 才前进，那一拍还要看在途的提交数，
  // 否则紧跟在 store 后面到 ROB 头的 MMIO load / fence.i / sfence 会把 SQ 误判为已排空 (UVM o6_irq_test 发现：
  // 中断处理里 sw 清中断后 lw 读回，读先于写到达设备)
  assign sq_drained_o = (sq_head == sq_cmt) && (cm_st_cnt_i == '0);

  //==========================================================================
  // 重放选择 (端口 0)
  //==========================================================================
  // 优先级：AMO 结果 > MMIO / 普通重放 (LQ 中最老的 READY 项)
  logic            amo_res_v;          // AMO 结果待注入
  logic [SQ_W-1:0] amo_idx;
  logic [63:0]     amo_res;

  logic [NLQ-1:0]  lq_ready;
  always_comb for (int i = 0; i < NLQ; i++) lq_ready[i] = lqv[i] && lq[i].st == LS_READY;

  // 重放选择分两拍：本拍从寄存的 LQ 状态里选最老的就绪项 (以 lq_head 为起点的环形优先，前缀或取独热) 并寄存，
  // 下拍注入端口 0 的 RF 级 (仍须有效且就绪)。上拍已选中、还没注入的那项本拍不再选。
  // 这样 IQ 的 P4 是否让位 (fu_ok) 也只看寄存器
  // 选中项寄存成独热码 rp_q_oh (2026-10-25，6 GHz)：排除上拍选中项只需与 ~rp_q_oh，选择结果直接寄存，
  // 项号在寄存器之后再编码。原来环上 "项号译码 -> 排除 -> 环形最老 -> 编码项号 -> 寄存" (b180u30：233 ps)
  logic            rp_q_v;
  logic [NLQ-1:0]  rp_q_oh;
  logic [LQ_W-1:0] rp_q_idx;
  logic            rp_lq_v;
  logic [LQ_W-1:0] rp_lq_idx;
  always_comb begin
    rp_q_idx = '0;
    for (int i = 0; i < NLQ; i++) rp_q_idx = rp_q_idx | ({LQ_W{rp_q_oh[i]}} & LQ_W'(i));
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rp_q_v <= 1'b0; rp_q_oh <= '0;
    end else begin
      logic [NLQ-1:0] rdy, hi, pk, pre, oh;
      rdy = lq_ready & ~(rp_q_v ? rp_q_oh : '0);
      for (int i = 0; i < NLQ; i++) hi[i] = rdy[i] && (LQ_W'(i) >= lq_head.idx);
      pk  = (|hi) ? hi : rdy;
      pre = pk;
      for (int d = 1; d < NLQ; d = d * 2) pre = pre | (pre << d);
      oh  = pk & ~(pre << 1);
      // AMO 带数据重放 (rp_amo) 优先占端口 0：本拍若有，就不选 (下拍再选)
      rp_q_v   <= (|rdy) && !flush_i && !amo_res_v;
      rp_q_oh  <= oh;
    end
  end
  assign rp_lq_v   = rp_q_v && |(rp_q_oh & lq_ready);
  assign rp_lq_idx = rp_q_idx;

  wire rp_amo = amo_res_v;
  wire rp_any = rp_amo || rp_q_v;          // 端口 0 本拍被重放占用 (只看寄存器)
  // rp_q_v 的下一值 (与上面寄存器的 D 同式)
  wire rp_nx  = (|(lq_ready & ~(rp_q_v ? rp_q_oh : '0))) && !flush_i && !amo_res_v;

  always_comb begin
    for (int k = 0; k < LSU_NP; k++) fu_ok_o[k] = '0;
    for (int k = 0; k < N_LDP; k++) fu_ok_o[k][FU_LD] = (k == 0) ? !rp_any : 1'b1;
    for (int k = LSU_KS; k < LSU_NP; k++) begin fu_ok_o[k][FU_ST] = 1'b1; fu_ok_o[k][FU_AMO] = 1'b1; end
    for (int k = 0; k < LSU_NP; k++) fu_ok_nx_o[k] = fu_ok_o[k];
    fu_ok_nx_o[0][FU_LD] = !(rp_nx || amo_res_v);
  end

  //==========================================================================
  // 流水寄存器
  //==========================================================================
  typedef struct packed {
    uop_t             u;
    logic             rp;         // 重放 (地址已知，不走 AGU)
    logic [LQ_W-1:0]  lqi;        // load 的 LQ 项
    logic             pre;        // 带数据重放 (MMIO / AMO)
    logic             pre_amo;    // 带数据重放来自 AMO (完成后标记 SQ)
    logic [63:0]      pdata;      // 预置数据 (双字对齐)
    logic [63:0]      va;
    logic [PA_W-1:0]  pa;
    logic [7:0]       mask;
    logic             excp;
    logic [5:0]       cause;
    logic             mmio;
    logic             tlbmiss;    // DTLB 缺失：带重执行标记完成
    logic [63:0]      x2;         // H：guest page fault 的 {隐式 (tinst = 0x3000), 6'b0, GPA[58:2]}，其它异常 0
  } lp_t;

  // 有效位 (带复位) 与载荷 (不带复位) 分开存，避免两个块写同一结构体
  logic [LSU_NP-1:0] rf_v, ag_v, l0_v, l1_v, l2_v;
  lp_t        rf [LSU_NP], ag [LSU_NP], ag_out [LSU_NP], l0r [LSU_NP], l1 [LSU_NP], l2 [LSU_NP];
  logic [63:0] rf_base [LSU_NP];
  logic [63:0] ag_base [LSU_NP];  // RF 级读到的基址 (AG 再旁路一次)
  lp_t         l0 [LSU_NP];       // L0 级组合结果：AGU / PMA / 未对齐 已算好
  // 向量单元访存口 (状态机在后面)
  // 向量访存口三级：T (查 DTLB、判缺失 / 页错误)、C (寄存的物理地址判 PMA / PMP)、A (占 D$ 口访问)；
  // A 有效即占着 D$ store 口 (vm_own)。VU 最多 2 个在途，T/C 一起只做下一行的翻译
  logic            vt_v, vt_we, vt_pw;
  logic            vc_v, vc_we, vc_pf, vc_af;
  logic [63:0]     vc_pa;
  logic [511:0]    vc_wd;
  logic [63:0]     vc_wm;
  logic [63:0]     vt_va;
  logic [511:0]    vt_wd;
  logic [63:0]     vt_wm;
  logic            va_v, va_we;
  logic [PA_W-1:0] va_pa;
  logic [511:0]    va_wd;
  logic [63:0]     va_wm;
  wire             vm_own = va_v;
  logic            vm_tmiss, vm_pf, vm_af, vm_gf;     // T 级的翻译结果 (逻辑见向量访存口一节；gf = guest page fault)
  logic [63:0]     vm_gx, vc_gx;
  logic            vc_gf;
  logic [63:0]     vm_tpa;
  logic [LSU_NP-1:0] l2_ld64;     // L2 里是对齐 64 位整数 ld (快速路径的生产者)
  logic [63:0] l2_data [LSU_NP];

  // 每次跨级都要清掉本拍已解析的标签位：否则标签被重新分配后会被误杀
  function automatic lp_t res_upd(input lp_t p, input br_update_t bu);
    lp_t r;
    r = p;
    r.u.br_mask = p.u.br_mask & ~bu.resolve_mask;
    return r;
  endfunction
  function automatic logic alive(input logic v, input lp_t p, input br_update_t bu, input logic fl);
    return v && !fl && !br_killed(p.u.br_mask, bu);
  endfunction

  // RF 级：读 rs1 + 旁路
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      prf_raddr_o[k] = rf[k].u.psrc1;
      rf_base[k] = prf_rdata_i[k];
      for (int w = 0; w < N_IWB; w++)
        if (iwb_i[w].valid && iwb_i[w].wen && iwb_i[w].pdst == rf[k].u.psrc1 && rf[k].u.psrc1 != '0)
          rf_base[k] = iwb_i[w].data;
      if (rf[k].u.rs1_class == RC_NONE) rf_base[k] = 64'd0;
    end
  end

  // 发射 / 重放进入 RF
  lp_t        rf_in [LSU_NP];
  logic [LSU_NP-1:0] rf_in_v;
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      rf_in[k] = '0;
      rf_in_v[k] = iss_valid_i[k];
      rf_in[k].u = iss_uop_i[k];
      rf_in[k].lqi = iss_uop_i[k].lq_idx.idx;
    end
    if (rp_amo) begin
      sq_ent_t s;
      s = sq[amo_idx];
      rf_in[0] = '0;
      rf_in_v[0] = 1'b1;
      rf_in[0].rp = 1'b1; rf_in[0].pre = 1'b1; rf_in[0].pre_amo = 1'b1;
      rf_in[0].pdata = amo_res;
      rf_in[0].u.fu = FU_AMO;
      rf_in[0].u.rob_idx = s.rob; rf_in[0].u.br_mask = s.br_mask;
      rf_in[0].u.pdst = s.pdst; rf_in[0].u.rd_class = s.rd_class;
      rf_in[0].u.mem_sz = s.sz; rf_in[0].u.mem_unsigned = 1'b0;
      rf_in[0].u.sq_idx = '{flag: s.flag, idx: amo_idx};
      rf_in[0].pa = s.paddr; rf_in[0].mask = s.mask;
    end else if (rp_lq_v) begin
      lq_ent_t e;
      e = lq[rp_lq_idx];
      rf_in[0] = '0;
      rf_in_v[0] = 1'b1;
      rf_in[0].rp = 1'b1;
      rf_in[0].lqi = rp_lq_idx;
      rf_in[0].pre = e.has_data;
      rf_in[0].pdata = e.data;
      rf_in[0].u.fu = FU_LD;
      rf_in[0].u.rob_idx = e.rob; rf_in[0].u.br_mask = e.br_mask;
      rf_in[0].u.pdst = e.pdst; rf_in[0].u.rd_class = e.rd_class;
      rf_in[0].u.mem_sz = e.sz; rf_in[0].u.mem_unsigned = e.uns; rf_in[0].u.fp_dbl = e.fp_dbl;
      rf_in[0].u.sq_idx = e.sq_bound;
      rf_in[0].u.lq_idx = '{flag: e.flag, idx: rp_lq_idx};
      rf_in[0].u.ld_wait = e.ld_wait;
      rf_in[0].pa = e.paddr; rf_in[0].mask = e.mask; rf_in[0].mmio = e.mmio;
      rf_in[0].va = {{(64-PA_W){1'b0}}, e.paddr};
    end
  end

  logic [LSU_NP-1:0] l1_drop;     // 预留：L1 级丢弃
  assign l1_drop = '0;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rf_v <= '0; ag_v <= '0; l0_v <= '0; l1_v <= '0; l2_v <= '0;
    end else begin
      for (int k = 0; k < LSU_NP; k++) begin
        rf_v[k] <= alive(rf_in_v[k], rf_in[k], br_upd_i, flush_i);
        ag_v[k] <= alive(rf_v[k], rf[k], br_upd_i, flush_i) && !(|(rf[k].u.pois & ld_fail_q_i)) &&
                   !(|(rf[k].u.poia & ld_fail_o));
        l0_v[k] <= alive(ag_v[k], ag[k], br_upd_i, flush_i) && !(|(ag[k].u.poib & ld_fail_o));
        l1_v[k] <= alive(l0_v[k], l0[k], br_upd_i, flush_i);
        l2_v[k] <= alive(l1_v[k], l1[k], br_upd_i, flush_i) && !l1_drop[k];
      end
    end
  end

  //==========================================================================
  // L0：AGU / 检查 / 写地址 / D$ 请求 / 违例
  //==========================================================================
  //==========================================================================
  // DTLB：每条流水一个查找口 (L0)
  //==========================================================================
  logic [VPN_W-1:0] dtlb_vpn [LSU_NP+1];
  logic [LSU_NP:0]  dtlb_v;                    // H：查找的 V (按 uop 的上下文)       // [LSU_NP]：向量单元的访存口
  logic [LSU_NP:0]  dtlb_hit;
  logic [PA_W-13:0] dtlb_ppn [LSU_NP+1];
  logic [7:0]       dtlb_perm [LSU_NP+1];
  logic [1:0]       dtlb_fault [LSU_NP+1];
  logic [3:0]       dtlb_gperm [LSU_NP+1];
  logic [56:0]      dtlb_gpa   [LSU_NP+1];
  logic             dtlb_gimp  [LSU_NP+1];
  logic [63:0]      l0_va [LSU_NP];

  o6_tlb #(.N(32), .NL(LSU_NP + 1)) u_dtlb (
    .clk_i, .rst_ni, .flush_i(tlb_flush_i),
    .lk_vpn_i(dtlb_vpn), .lk_v_i(dtlb_v), .lk_hit_o(dtlb_hit), .lk_ppn_o(dtlb_ppn), .lk_perm_o(dtlb_perm),
    .lk_fault_o(dtlb_fault), .lk_pbmt_o(), .lk_gperm_o(dtlb_gperm), .lk_gpa_o(dtlb_gpa), .lk_gimp_o(dtlb_gimp),
    // Svpbmt：PTW 照规范检查 PBMT 编码，但 NC / IO 不改访存通路 —— MMIO 通道到不了 DRAM，SoC 里也没有别的主设备，
    //   DRAM 是否经 D$ 对软件不可见；MMIO 区本来就按 IO 走 (docs/14 R4)
    .fill_i(dfill_i), .fill_v_i, .fill_vpn_i, .fill_level_i, .fill_nap_i, .fill_ppn_i, .fill_perm_i, .fill_pbmt_i(2'b00), .fill_fault_i,
    .fill_gperm_i, .fill_gpa_i, .fill_gimp_i
  );

  // PMP：在 L1 对寄存的物理地址检查 (两个口各一份)；结果并进 L1 的异常 (l1e)，
  // 各级 uop 的翻译上下文 (HLV / HSV 用 hctx；VCS 2018 不支持直接取函数返回结构体的成员，先落到数组)
  xctx_t ag_cx [LSU_NP], l0_cx [LSU_NP], l1_cx [LSU_NP];
  always_comb
    for (int k = 0; k < LSU_NP; k++) begin
      ag_cx[k] = uctx(ag[k].u,  dctx_i, hctx_i);
      l0_cx[k] = uctx(l0r[k].u, dctx_i, hctx_i);
      l1_cx[k] = uctx(l1[k].u,  dctx_i, hctx_i);
    end

  // L0 只做 DTLB / 权限 / PMA，免得 DTLB -> PMP 串在一拍里
  logic [LSU_NP-1:0] pmp_ok;
  for (genvar k = 0; k < LSU_NP; k++) begin : g_pmp
    o6_pmp #(.N(8)) u_pmp (
      .dec_i(pmpd_i), .pa_i(l1[k].pa), .priv_i(l1_cx[k].priv),
      // HLVX：PMP 要 R 且 X (同 Spike)
      .acc_i(is_hlvx(l1[k].u) ? 3'b101 :
             (l1[k].u.fu == FU_LD || (l1[k].u.fu == FU_AMO && amo_rdperm(l1[k].u.op))) ? 3'b001 : 3'b010), .ok_o(pmp_ok[k])
    );
  end

  // AG：再旁路一次基址 (覆盖"生产者上一拍刚发射"的背靠背情形)，然后地址加法，虚地址进 L0 寄存器
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      logic [63:0] b;
      b = ag_base[k];
      for (int w = 0; w < N_IWB; w++)
        if (iwb_i[w].valid && iwb_i[w].wen && iwb_i[w].pdst == ag[k].u.psrc1 && ag[k].u.psrc1 != '0)
          b = iwb_i[w].data;
      // load -> load 快速路径：生产者 (对齐 64 位 ld) 正在 L2，直接取它合并好的数据 (64 位对齐不用移位)；
      // 生产者判定失败时本 uop 带 poib，AG -> L0 丢弃，取到的错数据不会用
      for (int j = 0; j < N_LDP; j++)
        if (l2_v[j] && l2_ld64[j] && l2[j].u.pdst == ag[k].u.psrc1 && ag[k].u.psrc1 != '0)
          b = l2_data[j];
      if (ag[k].u.rs1_class == RC_NONE) b = 64'd0;
      ag_out[k] = ag[k];
      // 指针掩码 (Smnpm / Ssnpm)：加法之后换高位 (换的是 bit 56 / 47，比进位链末端早到)，后面的检查与 tval 都用掩码后的地址
      if (!ag[k].rp) ag_out[k].va = pm_x(b + {{32{ag[k].u.imm[31]}}, ag[k].u.imm}, ag_cx[k].pmm, ag_cx[k].zx);
    end
  end

  // L0：用寄存的虚地址查 DTLB / PMP
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      l0_va[k]    = l0r[k].va;
      dtlb_vpn[k] = l0_va[k][VA_W-1:12];
      dtlb_v[k]   = l0_cx[k].v;
    end
    dtlb_vpn[LSU_NP] = vt_va[VA_W-1:12];
    dtlb_v[LSU_NP]   = dctx_i.v;
  end

  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      lp_t p;
      logic [63:0] va;
      logic [1:0]  pm;
      logic        mis;
      p = l0r[k];
      p.x2 = '0;
      if (!l0r[k].rp) begin
        logic        is_ld, ld_cause, pf, af, hx, gf;
        logic [63:0] pa64, gx;
        logic [7:0]  pm8;
        logic [3:0]  gp;
        xctx_t       cx;
        va = l0_va[k];
        cx = l0_cx[k];
        hx = is_hlvx(l0r[k].u);
        // LR 按读处理 (#29)：只要读权限 (含 MXR)，报 load 类异常 (缺页 13 / 访问错误 5 / 未对齐 4)，与 Spike / Sail 一致；
        //   SC / AMO 仍按写 (store/AMO 类)。cbo.clean / flush / inval (Zicbom) 按读查权限、报 store/AMO 类异常 (与 Spike 一致)；
        //   cbo.zero 按写。缓存块操作不要求对齐 (作用于所在的 64 B 块)
        is_ld    = (l0r[k].u.fu == FU_LD) || (l0r[k].u.fu == FU_AMO && amo_rdperm(l0r[k].u.op));
        ld_cause = (l0r[k].u.fu == FU_LD) || (l0r[k].u.fu == FU_AMO && l0r[k].u.op == AMO_LR);
        unique case (l0r[k].u.mem_sz)
          2'd1: mis = va[0];
          2'd2: mis = |va[1:0];
          2'd3: mis = |va[2:0];
          default: mis = 1'b0;
        endcase
        if (l0r[k].u.fu == FU_AMO && amo_is_cbo(l0r[k].u.op)) mis = 1'b0;
        // 地址转换 (Sv39 / Sv48 / Sv57；H 两级：一级权限错 -> 页错误，优先于 G-stage 权限错 -> guest page fault)
        pf = 1'b0; af = 1'b0; gf = 1'b0; pm8 = '0; gp = '0;
        pa64 = va;
        gx = {1'b0, 6'b0, dtlb_gpa[k][56:10], va[11:2]};          // 最终 GPA (查找已按页级拼好 VA 高位)
        p.tlbmiss = 1'b0;
        if (cx.xl) begin
          pm8 = dtlb_perm[k];
          gp  = dtlb_gperm[k];
          if (cx.cn && !va_canon(va, cx.vam)) pf = 1'b1;     // 非规范地址
          else if (!cx.cn && va[63:VA_W] != '0) begin       // vsatp = Bare：GPA = VA，超出 59 位必是 guest page fault
            gf = 1'b1; gx = {7'b0, va[58:2]};
          end
          else if (!dtlb_hit[k])              p.tlbmiss = 1'b1;
          else if (dtlb_fault[k] == 2'd1)     pf = 1'b1;
          else if (dtlb_fault[k] == 2'd2)     af = 1'b1;
          else if (dtlb_fault[k] == 2'd3) begin
            gf = 1'b1;
            if (dtlb_gimp[k]) gx = {1'b1, 6'b0, dtlb_gpa[k]};  // VS PTE 地址的 G-stage 翻译出错：完整 PTE GPA
          end else begin
            // 一级权限位 {D, A, G, U, X, W, R, V} (vsatp = Bare 时不查)
            if (cx.cn) begin
              if (cx.priv == 2'b00 && !pm8[4])              pf = 1'b1;   // U 访问非 U 页
              if (cx.priv == 2'b01 && pm8[4] && !cx.sum)    pf = 1'b1;   // S 访问 U 页且 SUM=0
              if (!pm8[6])                                  pf = 1'b1;   // A=0 (Svade)
              if (hx && !pm8[3])                            pf = 1'b1;   // HLVX：要执行权限 (不看 R / MXR)
              if (is_ld && !hx && !(pm8[1] || (cx.mxr && pm8[3]))) pf = 1'b1;
              if (!is_ld && !pm8[2])                        pf = 1'b1;
              if (!is_ld && !pm8[7])                        pf = 1'b1;   // 写且 D=0
            end
            // G-stage {D, X, W, R} (非两级的项全 1)：MXR 只看 mstatus.MXR
            if (hx && !gp[2])                                  gf = 1'b1;
            if (is_ld && !hx && !(gp[0] || (cx.gmxr && gp[2]))) gf = 1'b1;
            if (!is_ld && !(gp[0] && gp[1]))                   gf = 1'b1;
            if (!is_ld && !gp[3])                              gf = 1'b1;   // G-stage D = 0 (Svade)
          end
          pa64 = {{(64-PA_W){1'b0}}, dtlb_ppn[k], va[11:0]};
        end else if (|va[63:PA_W]) begin
          af = 1'b1;     // 不转换时 PA_W 以上非零：原来 pma(64 位地址) 报访问错误；L1 的 PMA 只看截断后的 pa，这里先判 (va 早到)
        end
        pm = pma(pa64, dbg_mode_i);
        p.va   = va;
        p.pa   = pa64[PA_W-1:0];
        p.mask = bmask(va[2:0], l0r[k].u.mem_sz);
        p.mmio = pm[0];
        p.excp = 1'b0;
        p.x2   = '0;
        if (mis) begin
          p.excp = 1'b1; p.tlbmiss = 1'b0;
          p.cause = ld_cause ? o6_isa_pkg::EXC_LOAD_MISALIGN : o6_isa_pkg::EXC_STORE_MISALIGN;
        end else if (pf) begin
          p.excp = 1'b1; p.tlbmiss = 1'b0;
          p.cause = ld_cause ? o6_isa_pkg::EXC_LOAD_PAGE : o6_isa_pkg::EXC_STORE_PAGE;
        end else if (gf) begin
          p.excp = 1'b1; p.tlbmiss = 1'b0; p.x2 = gx;
          p.cause = ld_cause ? o6_isa_pkg::EXC_LOAD_GPAGE : o6_isa_pkg::EXC_STORE_GPAGE;
        end else if (p.tlbmiss) begin
          p.excp = 1'b0;
          p.mmio = 1'b0;
        end else if (af) begin
          // PMA 的访问错误 (不可访问区域、AMO / LR 访问 MMIO) 挪到 L1 与 PMP 一起并入 (见 l1e)；这里只剩 DTLB 报的访问错误
          p.excp = 1'b1;
          p.cause = ld_cause ? o6_isa_pkg::EXC_LOAD_ACCESS : o6_isa_pkg::EXC_STORE_ACCESS;
        end
      end
      l0[k] = p;
    end
  end

  always_ff @(posedge clk_i) begin
    for (int k = 0; k < LSU_NP; k++) begin
      rf[k]      <= res_upd(rf_in[k], br_upd_i);
      ag[k]      <= res_upd(rf[k], br_upd_i);
      ag_base[k] <= rf_base[k];
      l0r[k]     <= res_upd(ag_out[k], br_upd_i);
      l1[k]      <= res_upd(l0[k], br_upd_i);
    end
  end

  // L1 的有效结果：并入 PMP 与 PMA 访问检查 (首次执行、无其它异常、DTLB 命中的访存)。
  // PMA 原在 L0 (DTLB 出 ppn -> pma 区间比较 -> 异常优先级 -> l1.cause，p160u62 171.7 ps)，按寄存的 l1.pa 在这里查，
  // 优先级与原来相同 (排在未对齐 / 缺页 / TLB 缺失之后)；mmio 位仍在 L0 (单独的终点，L1 多处直接用)
  lp_t l1e [LSU_NP];
  logic [LSU_NP-1:0] l1_pmok;       // L1 的 PMA 可访问 (给 D$ kill：不可访问区域不能去 D$ / 下游取)
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      logic [1:0] pm1;
      pm1        = pma({{(64-PA_W){1'b0}}, l1[k].pa}, dbg_mode_i);
      l1_pmok[k] = pm1[1];
      l1e[k]     = l1[k];
      if (!l1[k].rp && !l1[k].excp && !l1[k].tlbmiss &&
          (!pmp_ok[k] || !pm1[1] || (l1[k].u.fu == FU_AMO && pm1[0])) &&
          (l1[k].u.fu == FU_LD || l1[k].u.fu == FU_ST || l1[k].u.fu == FU_AMO)) begin
        l1e[k].excp  = 1'b1;
        l1e[k].cause = (l1[k].u.fu == FU_LD || (l1[k].u.fu == FU_AMO && l1[k].u.op == AMO_LR)) ?
                       o6_isa_pkg::EXC_LOAD_ACCESS : o6_isa_pkg::EXC_STORE_ACCESS;
      end
    end
  end

  logic [LSU_NP-1:0] l0_live;
  always_comb for (int k = 0; k < LSU_NP; k++) l0_live[k] = alive(l0_v[k], l0[k], br_upd_i, flush_i);

  // DTLB 缺失 -> PTW (端口 0 优先)。不看本拍分支杀 (2026-10-28，6 GHz：p160u57 in 组 kill_tag -> ptw_dvpn_o −32.5)：
  // 本拍刚被杀的 load 也可能发起一次页表遍历 —— 结果仍是正确的翻译，填进 DTLB 无害，只是同拍别的缺失多等一次
  // 请求寄存一拍再给 PTW (2026-10-29，6 GHz：p160u62 out 组 l0r.va -> DTLB -> tlbmiss -> 选择 -> ptw_dvpn_o −23.7)：
  // PTW 空闲时锁存请求，晚一拍只让 DTLB 缺失的遍历晚 1 拍；冲刷后那拍到的请求最多多做一次遍历 (结果仍是正确翻译)
  logic        dreq_c;
  logic [VPN_W-1:0] dvpn_c;
  logic        dv_c;
  always_comb begin
    dreq_c = 1'b0; dvpn_c = '0; dv_c = 1'b0;
    for (int k = LSU_NP-1; k >= 0; k--)
      if (l0_v[k] && !flush_i && l0[k].tlbmiss) begin dreq_c = 1'b1; dvpn_c = l0[k].va[VA_W-1:12]; dv_c = l0_cx[k].v; end
    if (!dreq_c && vt_pw) begin dreq_c = 1'b1; dvpn_c = vt_va[VA_W-1:12]; dv_c = dctx_i.v; end
  end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) ptw_dreq_o <= 1'b0;
    else         ptw_dreq_o <= dreq_c && !flush_i;
  always_ff @(posedge clk_i) begin ptw_dvpn_o <= dvpn_c; ptw_dv_o <= dv_c; end

  // D$ 请求：非带数据重放的 load 都发 (VIPT：L0 只用页内偏移，取自寄存的虚地址，不等 DTLB / PMP / PMA)；
  // 异常、MMIO、TLB 缺失的在 L1 用 kill 取消 (D$ 在 L1 才查标签、分配 MSHR)
  always_comb begin
    for (int k = 0; k < N_LDP; k++) begin
      dc_ld_req_o[k]  = l0_live[k] && l0r[k].u.fu == FU_LD && !l0r[k].pre;
      dc_ld_addr_o[k] = PA_W'(l0r[k].va[11:0]);
    end
  end

  // STA 流水 (LSU_KS..)：L0 写 SQ 地址；违例检查在下一拍 (L1) 用寄存的物理地址做
  localparam int KS = LSU_KS;
  logic [N_STA-1:0] l1_live_s, stam_l1;      // 各 STA 流水 L1 活着 / L1 是有效的 STA 或 AMO 地址
  always_comb begin
    for (int s = 0; s < N_STA; s++) begin
      l1_live_s[s] = alive(l1_v[KS+s], l1[KS+s], br_upd_i, flush_i);
      stam_l1[s]   = l1_live_s[s] && !l1[KS+s].excp && !l1[KS+s].tlbmiss &&
                     (l1[KS+s].u.fu == FU_ST || (l1[KS+s].u.fu == FU_AMO && !l1[KS+s].rp));
    end
  end
  wire sta_l0 = |{l0_live[LSU_NP-1:KS]};     // 仅 tb 观察用 (+showviol)
  // load 步长预取 (o6_dpf)：每拍取一条 load 流水训练 (编号最小的、第一次执行、可缓存、无异常的 load)
  logic            dpf_tv, dpf_v;
  logic [11:0]     dpf_tpch;
  logic [PA_W-1:0] dpf_tpa, dpf_pa;
  always_comb begin
    dpf_tv = 1'b0; dpf_tpch = l1[0].u.pch; dpf_tpa = l1[0].pa;
    for (int k = N_LDP - 1; k >= 0; k--)
      if (alive(l1_v[k], l1[k], br_upd_i, flush_i) && l1[k].u.fu == FU_LD && !l1[k].rp && !l1[k].pre &&
          !l1[k].excp && !l1[k].tlbmiss && !l1[k].mmio) begin
        dpf_tv = 1'b1; dpf_tpch = l1[k].u.pch; dpf_tpa = l1[k].pa;
      end
  end
  o6_dpf u_dpf (
    .clk_i, .rst_ni, .tr_v_i(dpf_tv), .tr_pch_i(dpf_tpch), .tr_pa_i(dpf_tpa),
    .pf_v_o(dpf_v), .pf_pa_o(dpf_pa)
  );

  // D$ 只有一个预取口：store 预取 (第一条 STA 流水优先) 先，其次 load 步长预取 (预取只是提示，丢了无妨)
  // store 预取不看本拍分支杀 (2026-10-29，6 GHz：p160u62 in 组 kill_tag -> 优先选择 -> dc_pf_paddr_o −27.1)：预取只是提示
  always_comb begin
    dc_pf_req_o = dpf_v; dc_pf_paddr_o = dpf_pa;
    for (int s = N_STA - 1; s >= 0; s--)
      if (l1_v[KS+s] && !flush_i && !l1[KS+s].excp && !l1[KS+s].tlbmiss && l1[KS+s].u.fu == FU_ST && !l1[KS+s].mmio) begin
        dc_pf_req_o = 1'b1; dc_pf_paddr_o = l1[KS+s].pa;
      end
  end
  logic [NLQ-1:0] lq_pexcl;       // 在流水里、本拍已确定要等待后重新执行的 load (见 L2 判定之后)
  // 违例检查分两拍 (2026-10-25，6 GHz)：STA 的 L1 只算命中向量并寄存 (vh_q)，L2 再从中挑最老、取 rob / ftq，
  // 寄存后在 L3 报 ROB。原来 L1 一拍里 "命中 (含 lq_pexcl) -> 128 项环形最老 -> 与或取 rob / ftq" (b180：229 ps)。
  // ROB 置 replay 晚一拍：store 的完成在 L3 才报，它和更年轻的 load 最早 L4 提交，replay 在 L3 末写入仍来得及
  // (o6_rob 有断言：被置 replay 的 load 不得已提交)。L2 再与 lqv 相与：本拍被杀掉的 load 不再误报
  logic [N_STA-1:0]     viol_c;
  rob_ptr_t             viol_rob_c [N_STA];
  logic [FTQ_W-1:0]     viol_ftq_c [N_STA];
  logic [FTQ_OFF_W-1:0] viol_off_c [N_STA];
  logic [NLQ-1:0]       vh_c [N_STA], vh_q [N_STA];
  logic [NLQ-1:0]       vbm_q [N_STA];         // 分界温度计码：vbm_q[s][i] = (i >= store 的 lq_bound)，L1 译好寄存
  logic [N_STA-1:0]     vv_q;
  always_comb begin
    for (int s = 0; s < N_STA; s++) begin
      logic [NLQ-1:0] hit;
      for (int i = 0; i < NLQ; i++) begin
        lq_ptr_t e;
        e = '{flag: lq[i].flag, idx: LQ_W'(i)};
        // 只有已取得数据 (DONE) 或正在流水里可能拿旧数据 (PIPE) 的 load 才算违例；
        // 在等待 (WAIT/READY) 的 load 之后会重新读，不必冲刷。
        // 检查在 STA 的 L1 做：这条 store 的地址上拍末已写进 SQ，本拍在 L1 搜索的 load 看得到它 (lq_pexcl 排除)
        hit[i] = lqv[i] && lq[i].addr_ok && !lq[i].mmio &&
                 (lq[i].st == LS_DONE || (lq[i].st == LS_PIPE && !lq_pexcl[i])) &&
                 !lq_older(e, l1[KS+s].u.lq_idx) &&
                 lq[i].paddr[PA_W-1:3] == l1[KS+s].pa[PA_W-1:3] && |(lq[i].mask & l1[KS+s].mask);
      end
      vh_c[s] = hit;
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) vv_q <= '0;
    else for (int s = 0; s < N_STA; s++) vv_q[s] <= stam_l1[s] && (|vh_c[s]) && !flush_i;
  end
  always_ff @(posedge clk_i)
    for (int s = 0; s < N_STA; s++) begin
      vh_q[s] <= vh_c[s];
      for (int i = 0; i < NLQ; i++) vbm_q[s][i] <= LQ_W'(i) >= l1[KS+s].u.lq_idx.idx;
    end
  // 最老：从 store 的 lq_bound 开始的环形优先 (>= 分界里最低位，否则全体最低位)。
  // 2026-10-28 (6 GHz)：改成一棵归约树 (同 o6_sq_fwd)：节点 {等级 r, rob, ftq, off}，分界及以上的命中 r = 2、
  // 以下的命中 r = 1、非命中 0；两两比较时编号小的一半等级不低于另一半就胜出，7 级二选一得最老项。
  // 原来 "逐项与分界比较 -> |hi 选组 -> 128 位前缀或 -> 独热 -> 128 路与或" (p160u57：vb_q -> viol_ftq 170 ps)
  typedef struct packed {
    logic [1:0]           r;
    rob_ptr_t             rob;
    logic [FTQ_W-1:0]     ftq;
    logic [FTQ_OFF_W-1:0] off;
  } vn_t;
  function automatic vn_t vpick(input vn_t lo, input vn_t hi);
    return (lo.r[1] || (lo.r[0] && !hi.r[1])) ? lo : hi;
  endfunction
  always_comb begin
    for (int s = 0; s < N_STA; s++) begin
      logic [NLQ-1:0] hit;
      vn_t            nd [NLQ];
      hit = vh_q[s] & lqv;
      for (int i = 0; i < NLQ; i++) begin
        nd[i].r   = {hit[i] && vbm_q[s][i], hit[i] && !vbm_q[s][i]};
        nd[i].rob = lq[i].rob;
        nd[i].ftq = lq[i].ftq_idx;
        nd[i].off = lq[i].ftq_off;
      end
      // 原地归约：第 i 个结果只覆盖已读过的 nd[i] (i <= 2i)；lo = 编号小的一半
      for (int w = NLQ / 2; w >= 1; w = w / 2)
        for (int i = 0; i < w; i++) nd[i] = vpick(nd[2*i], nd[2*i+1]);
      viol_c[s]     = vv_q[s] && (|hit);
      viol_rob_c[s] = nd[0].rob;
      viol_ftq_c[s] = nd[0].ftq;
      viol_off_c[s] = nd[0].off;
    end
  end
`ifndef SYNTHESIS
  // 与原写法 (逐项比较分界 + 前缀或取独热 + 与或) 逐拍一致 (有违例时)
  always_ff @(posedge clk_i)
    if (rst_ni)
      for (int s = 0; s < N_STA; s++) begin
        logic [NLQ-1:0] hit, hi, pk, oh;
        rob_ptr_t r;
        logic [FTQ_W-1:0] f;
        logic [FTQ_OFF_W-1:0] o;
        hit = vh_q[s] & lqv;
        for (int i = 0; i < NLQ; i++) hi[i] = hit[i] && vbm_q[s][i];
        pk = (|hi) ? hi : hit;
        oh = pk & -pk;
        r = '0; f = '0; o = '0;
        for (int i = 0; i < NLQ; i++) if (oh[i]) begin r = lq[i].rob; f = lq[i].ftq_idx; o = lq[i].ftq_off; end
        if (viol_c[s]) assert (viol_rob_c[s] == r && viol_ftq_c[s] == f && viol_off_c[s] == o)
          else $error("lsu: 违例最老项归约树与原写法不一致 (流水 %0d)", s);
      end
`endif
  // 违例结果寄存一拍再报 ROB (现在是 STA 的 L3 拍，见上)：store 自己的完成在 L3 才标记，置 replay 与它同拍生效、仍早于更新的 load 提交；
  // 被冲刷 / 误预测杀掉的 load 晚一拍置 replay 无害 (ROB 那项已无效或会被重新分配前清掉)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin viol_valid_o <= '0; viol_any_o <= 1'b0; end
    else begin
      viol_valid_o <= viol_c & {N_STA{!flush_i}};
      viol_any_o   <= (|viol_c) && !flush_i;
    end
  end
  always_ff @(posedge clk_i) begin
    for (int s = 0; s < N_STA; s++) viol_rob_o[s] <= viol_rob_c[s];
    // 访存相关预测只训一个：取编号最小的那条流水查出的
    for (int s = N_STA - 1; s >= 0; s--)
      if (viol_c[s]) begin viol_ftq_o <= viol_ftq_c[s]; viol_off_o <= viol_off_c[s]; end
  end

  //==========================================================================
  // L1：转发搜索
  //==========================================================================
  always_comb begin
    for (int k = 0; k < N_LDP; k++) begin
      dc_ld_paddr_o[k] = l1[k].pa;
      // kill 不等 PMP (L1 才查)：PMP 拒绝而 PMA 可访问的只可能是主存，把这行取进 D$ 没有副作用；异常照常在 L2 报。
      // PMA 不可访问 (2026-10-29 起 PMA 异常也在 L1 并入) 必须杀：不能去下游取不存在的地址 (l1_pmok 由寄存的 pa 算)
      dc_ld_kill_o[k]  = !alive(l1_v[k], l1[k], br_upd_i, flush_i) || l1[k].excp || l1[k].mmio || l1[k].tlbmiss ||
                         (!l1[k].rp && !l1_pmok[k]);
    end
  end

  // 在候选里取"最年轻的较老 store"：下标 < 分界的里取最大，否则在 >= 分界的里取最大。
  // 用后缀或 (5 级并行前缀) 得到独热，不用逐项优先级链
  function automatic logic [NSQ-1:0] sq_pick(input logic [NSQ-1:0] c, input logic [SQ_W-1:0] bound);
    logic [NSQ-1:0] lo, p, sfx;
    for (int e = 0; e < NSQ; e++) lo[e] = c[e] && (SQ_W'(e) < bound);
    p = (|lo) ? lo : c;
    sfx = p;
    for (int d = 1; d < NSQ; d = d * 2) sfx = sfx | (sfx >> d);    // sfx[e] = |p[NSQ-1:e]
    return p & ~(sfx >> 1);
  endfunction
  function automatic logic [SQ_W-1:0] oh_enc(input logic [NSQ-1:0] oh);
    logic [SQ_W-1:0] r;
    r = '0;
    for (int e = 0; e < NSQ; e++) r = r | ({SQ_W{oh[e]}} & SQ_W'(e));
    return r;
  endfunction

  // 每项按自身地址对齐好的数据、能否转发 (与选择并行算，选中后只做与或)
  logic [63:0]     sq_sdata [NSQ];
  logic [NSQ-1:0]  sq_fok, sq_flagv;
  always_comb begin
    for (int e = 0; e < NSQ; e++) begin
      sq_flagv[e] = sq[e].flag;
      sq_sdata[e] = sq[e].data << {sq[e].paddr[2:0], 3'b000};
      sq_fok[e]   = sq[e].data_ok && !sq[e].is_amo && !sq[e].mmio;
    end
  end

  // 每个字节的转发源：每条 load 流水一个 o6_sq_fwd (搜索逻辑见该文件；单元 TB tb/unit/tb_lsu_fwd.sv)
  // (只在 load 流水上搜；STA 流水的这些量恒为 0)
  logic [7:0]       fw_hit   [LSU_NP];     // 该字节有 store 覆盖
  logic [63:0]      fw_data  [LSU_NP];
  logic             fw_wait  [LSU_NP];
  logic [SQ_W-1:0]  fw_wsq   [LSU_NP];     // 要等的 store 之一
  logic [NSQ-1:0]   sq_aokv;
  logic [PA_W-1:0]  sq_pav   [NSQ];
  logic [7:0]       sq_maskv [NSQ];
  always_comb
    for (int e = 0; e < NSQ; e++) begin sq_aokv[e] = sq[e].addr_ok; sq_pav[e] = sq[e].paddr; sq_maskv[e] = sq[e].mask; end
  for (genvar k = 0; k < LSU_NP; k++) begin : g_fwd
    if (k < N_LDP) begin : g_ld
      o6_sq_fwd #(.NSQ(NSQ)) u_fwd (
        .sqv_i(sqv), .flag_i(sq_flagv), .addr_ok_i(sq_aokv), .paddr_i(sq_pav), .mask_i(sq_maskv),
        .sdata_i(sq_sdata), .fok_i(sq_fok),
        .ld_pa_i(l1[k].pa), .ld_mask_i(l1[k].mask), .ld_sq_i(l1[k].u.sq_idx),
        .hit_o(fw_hit[k]), .data_o(fw_data[k]), .wait_o(fw_wait[k]), .wsq_o(fw_wsq[k]));
    end else begin : g_sta
      assign fw_hit[k] = '0; assign fw_data[k] = '0; assign fw_wait[k] = 1'b0; assign fw_wsq[k] = '0;
    end
  end

  // 访存相关预测：带 ld_wait 的 load，若有更早的 store 地址还没定，就等其中最年轻的那条
  logic        sa_wait [LSU_NP];
  sq_ptr_t     sa_wsq  [LSU_NP];
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      logic [NSQ-1:0] c, po;
      logic           f;
      logic [SQ_W-1:0] wi;
      for (int e = 0; e < NSQ; e++) begin
        sq_ptr_t ep;
        ep = '{flag: sq[e].flag, idx: SQ_W'(e)};
        c[e] = sqv[e] && !sq[e].addr_ok && sq_older(ep, l1[k].u.sq_idx);
      end
      f  = |c;
      po = sq_pick(c, l1[k].u.sq_idx.idx);
      wi = oh_enc(po);
      sa_wait[k] = f && l1[k].u.fu == FU_LD && l1[k].u.ld_wait && k < N_LDP;
      sa_wsq[k]  = '{flag: |(po & sq_flagv), idx: wi};
    end
  end

  logic        l2_sa_wait [LSU_NP];
  sq_ptr_t     l2_sa_wsq  [LSU_NP];
  logic [7:0]  l2_fw_hit [LSU_NP];
  logic [63:0] l2_fw_data [LSU_NP];
  logic        l2_fw_wait [LSU_NP];
  sq_ptr_t     l2_fw_wsq [LSU_NP];
  logic [SQ_W-1:0] l2_fw_wi [LSU_NP];
  // L2：要等的那条 store 补上翻转位
  always_comb
    for (int k = 0; k < LSU_NP; k++) l2_fw_wsq[k] = '{flag: sq_flagv[l2_fw_wi[k]], idx: l2_fw_wi[k]};
  always_ff @(posedge clk_i) begin
    for (int k = 0; k < LSU_NP; k++) begin
      l2[k] <= res_upd(l1e[k], br_upd_i);
      l2_fw_hit[k]  <= fw_hit[k];
      l2_fw_data[k] <= fw_data[k];
      l2_fw_wait[k] <= fw_wait[k];
      l2_fw_wi[k]   <= fw_wsq[k];
      l2_sa_wait[k] <= sa_wait[k];
      l2_sa_wsq[k]  <= sa_wsq[k];
    end
  end

  //==========================================================================
  // L2：判定
  //==========================================================================
  typedef enum logic [3:0] {R_OK, R_EXCP, R_WAITSQ, R_MISS, R_NACK, R_MMIO, R_STA, R_AMOADDR, R_REPLAY,
                            R_WAITSA} res_e;
  res_e        l2_res [LSU_NP];
  logic [LSU_NP-1:0] l2_live;

  // D$ load 口的返回按 LSU 流水编号展开 (STA 流水不用 D$ load 口，补 0)
  logic [LSU_NP-1:0] dcl_hit, dcl_nack;
  logic [63:0]       dcl_data [LSU_NP];
  logic [MSHR_W-1:0] dcl_mshr [LSU_NP];
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      dcl_hit[k] = 1'b0; dcl_nack[k] = 1'b0; dcl_data[k] = '0; dcl_mshr[k] = '0;
      if (k < N_LDP) begin
        dcl_hit[k]  = dc_ld_hit_i[(k < N_LDP) ? k : 0];
        dcl_nack[k] = dc_ld_nack_i[(k < N_LDP) ? k : 0];
        dcl_data[k] = dc_ld_data_i[(k < N_LDP) ? k : 0];
        dcl_mshr[k] = dc_ld_mshr_i[(k < N_LDP) ? k : 0];
      end
    end
  end
  always_comb
    for (int k = 0; k < LSU_NP; k++)
      l2_ld64[k] = k < N_LDP && l2[k].u.fu == FU_LD && l2[k].u.mem_sz == 2'd3 && l2[k].u.rd_class == RC_INT;
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      l2_live[k] = alive(l2_v[k], l2[k], br_upd_i, flush_i);
      l2_data[k] = '0;
      if (l2[k].tlbmiss && !l2[k].rp)              l2_res[k] = R_REPLAY;
      else if (l2[k].u.fu == FU_ST)                l2_res[k] = l2[k].excp ? R_EXCP : R_STA;
      else if (l2[k].u.fu == FU_AMO && !l2[k].pre) l2_res[k] = l2[k].excp ? R_EXCP : R_AMOADDR;
      else if (l2[k].excp)                         l2_res[k] = R_EXCP;
      else if (l2[k].pre)                          begin l2_res[k] = R_OK; l2_data[k] = l2[k].pdata; end
      else if (l2[k].mmio)                         l2_res[k] = R_MMIO;
      else if (l2_sa_wait[k])                      l2_res[k] = R_WAITSA;
      else if (l2_fw_wait[k])                      l2_res[k] = R_WAITSQ;
      else if ((l2_fw_hit[k] & l2[k].mask) == l2[k].mask) begin
        l2_res[k] = R_OK; l2_data[k] = l2_fw_data[k];              // 全部来自转发
      end
      else if (dcl_nack[k])                        l2_res[k] = R_NACK;
      else if (!dcl_hit[k])                        l2_res[k] = R_MISS;
      else begin
        l2_res[k] = R_OK;
        for (int b = 0; b < 8; b++)
          l2_data[k][8*b +: 8] = l2_fw_hit[k][b] ? l2_fw_data[k][8*b +: 8] : dcl_data[k][8*b +: 8];
      end
    end
  end

  // 违例检查的排除 (检查在 STA 的 L1 拍)：
  //   - 本拍在 L1 的 load：转发搜索已经看得到这条 store；
  //   - L2 里结果是 WAITSA / WAITSQ 的 load：之后会被唤醒重新执行，读到的旧值不会被用
  // (否则带 ld_wait 的 load 恰好在流水里时，正是它要等的那条 store 的 STA 把它冲掉，预测形同虚设)
  always_comb begin
    lq_pexcl = '0;
    for (int k = 0; k < LSU_NP; k++) begin
      if (l1_v[k] && l1[k].u.fu == FU_LD)
        lq_pexcl[l1[k].lqi] = 1'b1;
      // 等价于 l2_res 为 WAITSA / WAITSQ，但不经过 l2_res 编码 (它还依赖 D$ 命中等晚到的输入)
      if (l2_v[k] && l2[k].u.fu == FU_LD && !(l2[k].tlbmiss && !l2[k].rp) && !l2[k].excp && !l2[k].pre &&
          !l2[k].mmio && (l2_sa_wait[k] || l2_fw_wait[k]))
        lq_pexcl[l2[k].lqi] = 1'b1;
    end
  end

  // 唤醒：整数在 L0 投机，只看 L0 的寄存器 (DTLB / PMA 的结果是 L0 的组合逻辑，不串进唤醒)：
  // 命中、转发、等 store、bank 冲突、TLB 缺失、异常、MMIO 都按"会成功"猜，L2 判定后失败再取消 (窗口 2 拍)。
  // 只有已知不会在 L2 成功的不唤醒：不带数据的 MMIO 重放 (在 ROB 头执行，之后带数据重放时再唤醒)
  // 消费者在 L1 拍被选中、L2 拍 RF、下一拍 EX 从 load 的 wbreg 旁路 (L3 已并进 L2)：load-to-use 4 拍
  logic [N_LDP-1:0] spec_l0, l1_spec, l2_spec;
  // 快速唤醒 (AG 级)：对齐的 64 位整数 ld (指针)；条件与 L0 的投机唤醒一致 (它在 L0 一定还会投机唤醒，
  // 所以两拍后的 ld_fail 覆盖它)。生产者自己带 poib 时本拍判定，失败则作废 (sc)
  always_comb begin
    for (int k = 0; k < N_LDP; k++)
      lwake_o[k] = '{valid: alive(ag_v[k], ag[k], br_upd_i, flush_i) && ag[k].u.fu == FU_LD && ag[k].u.mem_sz == 2'd3 &&
                            ag[k].u.rd_class == RC_INT && !(ag[k].rp && ag[k].mmio && !ag[k].pre),
                     pdst: ag[k].u.pdst, sb: N_LDP'(1) << k, sc: ag[k].u.poib};
  end
  always_comb begin
    for (int k = 0; k < N_LDP; k++) begin
      spec_l0[k] = l0_live[k] && l0r[k].u.rd_class == RC_INT &&
                   ((l0r[k].u.fu == FU_LD && !(l0r[k].rp && l0r[k].mmio && !l0r[k].pre)) ||
                    (l0r[k].u.fu == FU_AMO && l0r[k].pre));
      iwake_o[k] = '{valid: spec_l0[k], is_fp: 1'b0, pdst: l0r[k].u.pdst, sa: N_LDP'(1) << k, sn: '0, sc: '0};
      // 同一个 load 在 L1：分派中的 uop 在 L0 唤醒那拍还没入队、L2 确认那拍已经入队，L1 这拍入队的两头都听不到
      ewake_o[k] = '{valid: l1_spec[k] && alive(l1_v[k], l1[k], br_upd_i, flush_i), is_fp: 1'b0, pdst: l1[k].u.pdst,
                     sa: '0, sn: N_LDP'(1) << k, sc: '0};
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin l1_spec <= '0; l2_spec <= '0; end
    else begin
      l1_spec <= spec_l0 & {N_LDP{!flush_i}};
      l2_spec <= l1_spec & {N_LDP{!flush_i}};
    end
  end
  always_comb begin
    for (int k = 0; k < N_LDP; k++) begin
      logic ok;
      ok = l2_live[k] && l2_res[k] == R_OK;
      cwake_o[k]   = '{valid: ok && l2[k].u.rd_class == RC_INT, is_fp: 1'b0, pdst: l2[k].u.pdst, sa: '0, sn: '0, sc: '0};
      fwake_o[k]   = '{valid: ok && l2[k].u.rd_class == RC_FP,  is_fp: 1'b1, pdst: l2[k].u.pdst, sa: '0, sn: '0, sc: '0};
      ld_fail_o[k] = l2_spec[k] && !ok;
    end
  end
`ifndef SYNTHESIS
  // 成功的整数 load 必须在 L1 投机唤醒过 (IQ 只在入队口听确认唤醒)
  always_ff @(posedge clk_i)
    if (rst_ni)
      for (int k = 0; k < N_LDP; k++)
        assert (!cwake_o[k].valid || l2_spec[k]) else $error("lsu: 流水 %0d 的 load 成功但没有在 L0 投机唤醒", k);
`endif

  //==========================================================================
  // 写回 / 完成：对齐、符号扩展、NaN-boxing 在 L2 做完直接进 wbreg
  // (原来单独一级 L3；2026-10-14 并进 L2：D$ 的命中 / 数据和转发结果在 L2 都是寄存器输出，
  //  L2 只剩字节合并 + 移位 + 扩展，wbreg 早一拍，load-to-use 5 -> 4)
  //==========================================================================
  logic [63:0] l2_fmt [LSU_NP];
  always_comb begin
    for (int k = 0; k < LSU_NP; k++) begin
      logic [63:0] sh;
      sh = l2_data[k] >> {l2[k].pa[2:0], 3'b000};
      unique case (l2[k].u.mem_sz)
        2'd0: l2_fmt[k] = l2[k].u.mem_unsigned ? {56'b0, sh[7:0]}  : {{56{sh[7]}},  sh[7:0]};
        2'd1: l2_fmt[k] = l2[k].u.mem_unsigned ? {48'b0, sh[15:0]} : {{48{sh[15]}}, sh[15:0]};
        2'd2: l2_fmt[k] = l2[k].u.mem_unsigned ? {32'b0, sh[31:0]} : {{32{sh[31]}}, sh[31:0]};
        default: l2_fmt[k] = sh;
      endcase
      // FLW：NaN-boxing
      if (l2[k].u.rd_class == RC_FP && l2[k].u.mem_sz == 2'd2) l2_fmt[k] = {32'hFFFF_FFFF, sh[31:0]};
      if (l2[k].u.rd_class == RC_FP && l2[k].u.mem_sz == 2'd1) l2_fmt[k] = {48'hFFFF_FFFF_FFFF, sh[15:0]};   // FLH
      // AMO / SC 的结果：W 型符号扩展 (旧值或 SC 成功标志已在 pdata 低位)
      if (l2[k].pre_amo) l2_fmt[k] = (l2[k].u.mem_sz == 2'd2) ? {{32{l2_data[k][31]}}, l2_data[k][31:0]} : l2_data[k];
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int k = 0; k < N_LDP; k++) begin iwb_o[k].valid <= 1'b0; fwb_o[k].valid <= 1'b0; end
      for (int k = 0; k < LSU_NP; k++) cmpl_o[k].valid <= 1'b0;
    end else if (flush_i) begin
      for (int k = 0; k < N_LDP; k++) begin iwb_o[k].valid <= 1'b0; fwb_o[k].valid <= 1'b0; end
      for (int k = 0; k < LSU_NP; k++) cmpl_o[k].valid <= 1'b0;
    end else begin
      for (int k = 0; k < N_LDP; k++) begin
        logic ok;
        ok = l2_live[k] && l2_res[k] == R_OK;
        iwb_o[k].valid  <= ok && l2[k].u.rd_class == RC_INT;
        fwb_o[k].valid  <= ok && l2[k].u.rd_class == RC_FP;
      end
      for (int k = 0; k < LSU_NP; k++)
        cmpl_o[k].valid <= l2_live[k] && (l2_res[k] == R_OK || l2_res[k] == R_EXCP || l2_res[k] == R_STA ||
                                          l2_res[k] == R_REPLAY);
    end
  end
  always_comb for (int k = 0; k < N_LDP; k++) iwb_pn_o[k] = l2[k].u.pdst;
  always_ff @(posedge clk_i) begin
    for (int k = 0; k < N_LDP; k++) begin
      iwb_o[k].pdst <= l2[k].u.pdst; iwb_o[k].wen <= 1'b1; iwb_o[k].data <= l2_fmt[k];
      fwb_o[k].pdst <= l2[k].u.pdst; fwb_o[k].wen <= 1'b1; fwb_o[k].data <= l2_fmt[k];
    end
    for (int k = 0; k < LSU_NP; k++) begin
      cmpl_o[k].rob_idx     <= l2[k].u.rob_idx;
      cmpl_o[k].excp        <= l2_res[k] == R_EXCP;
      cmpl_o[k].cause       <= l2[k].cause;
      cmpl_o[k].tval        <= l2[k].va;
      cmpl_x2_o[k]          <= l2[k].x2;
      cmpl_o[k].flush_after <= 1'b0;
      cmpl_o[k].replay      <= l2_res[k] == R_REPLAY;
      cmpl_o[k].fflags      <= '0;
    end
  end

  //==========================================================================
  // 头部串行：MMIO load、AMO
  //==========================================================================
  typedef enum logic [2:0] {H_IDLE, H_MMIO_RD, H_MMIO_WAIT, H_AMO, H_AMO_WAIT} hst_e;
  logic ptw_own;          // PTW 正占用 D$ store 口
  logic drain_own;        // 排空正占用 D$ store 口 (可缓存 store 已发出、等 done)
  hst_e            hst;
  logic [LQ_W-1:0] h_lqi;
  logic            lr_valid;
  logic [PA_W-1:0] lr_addr;

  // ROB 头是不是一个等在 LQ 里的 MMIO load
  logic            head_mmio_v;
  logic [LQ_W-1:0] head_mmio_i;
  always_comb begin
    head_mmio_v = 1'b0; head_mmio_i = '0;
    for (int i = 0; i < NLQ; i++)
      if (lqv[i] && lq[i].st == LS_WAIT && lq[i].wk == WK_HEAD && lq[i].rob == rob_head_i) begin
        head_mmio_v = 1'b1; head_mmio_i = LQ_W'(i);
      end
  end

  // SQ 头是不是 ROB 头的 AMO
  sq_ent_t sqh;
  assign sqh = sq[sq_head.idx];
  // D$ store 口的地址要早到 (D$ 当拍查标签)：寄存"头"与"头+1"两项的物理地址，按上拍是否出队二选一。
  // 已提交 / 待执行 AMO 的地址早在 STA 时写好 (提交与 AMO 执行都在 STA 若干拍之后)，晚一拍的副本不会过时
  logic [PA_W-1:0] pa_h0_q, pa_h1_q, pa_h2_q;
  logic            pa_adv_q;
  wire  [PA_W-1:0] sqh_pa = pa_adv_q ? pa_h1_q : pa_h0_q;
  // 向量访存占口时，下一条是 T 级的那一行 (D$ 在 A 完成那拍预查它的标签，背靠背访问)
  // (C 空时是旧值：D$ 在 S_LOOK 核对地址不符就重查，只损失一拍)
  assign dc_st_next_paddr_o = vm_own ? vc_pa[PA_W-1:0] : (pa_adv_q ? pa_h2_q : pa_h1_q);
  // 排空用的其余字段 (就绪、对齐后的数据、掩码、宽度、类型) 也寄存"头 / 头+1"副本：
  // 就绪的几个条件 (有效、已提交、地址、数据) 一旦成立只会在出队时清掉，晚一拍的副本只会更保守
  typedef struct packed {
    logic        ok, is_amo, mmio;
    logic [63:0] sdata;
    logic [7:0]  mask;
    logic [1:0]  sz;
    logic [4:0]  amo_op;
  } hc_t;
  function automatic hc_t mk_hc(input logic v, input sq_ent_t e);
    hc_t r;
    r.ok = v && e.committed && e.addr_ok && e.data_ok;
    r.is_amo = e.is_amo; r.mmio = e.mmio;
    r.sdata = e.data << {e.paddr[2:0], 3'b000};
    r.mask = e.mask; r.sz = e.sz; r.amo_op = e.amo_op;
    return r;
  endfunction
  hc_t hc0_q, hc1_q;
  hc_t hs;
  assign hs = pa_adv_q ? hc1_q : hc0_q;
  wire head_amo_v = sqv[sq_head.idx] && sqh.is_amo && !sqh.amo_done && sqh.addr_ok && sqh.fchk && !sqh.fault && sqh.data_ok &&
                    sqh.rob == rob_head_i && !amo_res_v;

  wire start_mmio = (hst == H_IDLE) && head_mmio_v && head_go_i && sq_drained_o;
  wire start_amo  = (hst == H_IDLE) && !start_mmio && head_amo_v && head_go_i && !ptw_own && !drain_own;
  assign head_start_o = start_mmio || start_amo;

  // SC 是否成功
  wire sc_ok = lr_valid && lr_addr == sqh.paddr;           // 保留集 = LR 的精确地址 (与 Spike 一致)

  // 排空 FSM、AMO、PTW 共用 D$ store 口 (请求必须保持到 done，所以开始后锁定占用者)
  logic drain_go, drain_mmio;
  wire  port_free = !ptw_own && !drain_own && (hst == H_IDLE) && !vm_own;
  // D$ 在 store 完成的下一拍按预查的下一条 store 直接动作，所以那一拍 PTW 不抢口 (只晚一两拍)
  logic dc_done_q;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) dc_done_q <= 1'b0;
    else         dc_done_q <= dc_st_done_i;
  wire  ptw_take  = ptw_mem_req_i && port_free && !start_amo && !dc_done_q;
  // 选地址 / 字段用不含 start_amo 的 ptw_sel (2026-10-28，6 GHz：p160u57 out 组 sq_head -> sqh -> head_amo_v -> start_amo
  // -> ptw_take -> dc_st_paddr_o −37.8)：start_amo 那拍 dc_st_req_o 本来就是 0，字段无意义；请求有效仍用 ptw_take
  wire  ptw_sel   = ptw_mem_req_i && port_free && !dc_done_q;
  // 查表地址只看寄存的占用状态：AMO 进行中 / 排空 -> SQ 头副本；PTW 已占口 -> PTW 地址。
  // 请求第一拍 (占用还没建立) D$ 只查表不动作，第二拍起占用已寄存，与实际请求地址一致
  assign dc_st_lk_paddr_o = vm_own ? va_pa :
                            (ptw_own && !(hst == H_AMO || hst == H_AMO_WAIT)) ? {ptw_mem_addr_i[PA_W-1:3], 3'b000} : sqh_pa;
  wire  sq_head_ready = sqv[sq_head.idx] && sqh.committed && sqh.addr_ok && sqh.data_ok;   // AMO 释放用 (实时)

  always_comb begin
    dc_st_req_o    = 1'b0;
    dc_st_paddr_o  = sqh_pa;
    dc_st_data_o   = hs.sdata;
    dc_st_mask_o   = hs.mask;
    dc_st_amo_o    = 1'b0;
    dc_st_amo_op_o = hs.amo_op;
    dc_st_line_o   = 1'b0;
    dc_st_ldata_o  = va_wd;
    dc_st_lmask_o  = va_wm;
    dc_st_size_o   = hs.sz;
    mmio_req_o     = 1'b0;
    mmio_we_o      = 1'b0;
    mmio_addr_o    = sqh.paddr;
    mmio_size_o    = sqh.sz;
    mmio_wdata_o   = sqh.data << {sqh.paddr[2:0], 3'b000};
    mmio_wmask_o   = sqh.mask;
    drain_go       = 1'b0;
    drain_mmio     = 1'b0;
    if (hst == H_MMIO_RD || hst == H_MMIO_WAIT) begin
      mmio_req_o  = 1'b1;
      mmio_addr_o = lq[h_lqi].paddr;
      mmio_size_o = lq[h_lqi].sz;
    end else if (hst == H_AMO || hst == H_AMO_WAIT) begin
      // SC 失败不访存；LR 当作 AMO 读
      dc_st_req_o = !(sqh.amo_op == AMO_SC && !sc_ok);
      dc_st_amo_o = (sqh.amo_op != AMO_SC);
      // Zicbom / Zicboz：整行模式。cbo.zero = 整行写全 0 (普通 store 路径，缺失照常回填后写)；
      // cbo.clean / flush = 整行读 (AMO 读) + D$ 内按操作码写回 / 作废 (见 o6_dcache S_CBO)
      if (amo_is_cbo(sqh.amo_op)) begin
        dc_st_line_o  = 1'b1;
        dc_st_ldata_o = '0;
        dc_st_lmask_o = '1;
        dc_st_amo_o   = (sqh.amo_op != AMO_CBOZ);
      end
    end else if (vm_own) begin
      // 向量访存：读按 AMO_LR (只读)，写按普通 store (字节掩码)
      dc_st_req_o    = 1'b1;
      dc_st_paddr_o  = va_pa;
      dc_st_line_o   = 1'b1;
      dc_st_data_o   = va_wd[63:0];
      dc_st_mask_o   = 8'hFF;
      dc_st_amo_o    = !va_we;
      dc_st_amo_op_o = AMO_LR;
      dc_st_size_o   = 2'd3;
    end else if (ptw_own || ptw_sel) begin
      dc_st_req_o    = ptw_own || ptw_take;
      dc_st_paddr_o  = {ptw_mem_addr_i[PA_W-1:3], 3'b000};
      dc_st_amo_o    = 1'b1;
      dc_st_amo_op_o = AMO_LR;            // 只读
      dc_st_size_o   = 2'd3;
      dc_st_mask_o   = 8'hFF;
    end else if (hs.ok && !hs.is_amo) begin
      // MMIO store 走 MMIO 口，不占 D$ 口
      if (hs.mmio) begin drain_go = 1'b1; mmio_req_o = 1'b1; mmio_we_o = 1'b1; drain_mmio = 1'b1; end
      else if (!ptw_mem_req_i || drain_own) begin drain_go = 1'b1; dc_st_req_o = 1'b1; end
    end
  end

  assign ptw_mem_gnt_o   = ptw_take;
  assign ptw_mem_valid_o = ptw_own && dc_st_done_i;
  assign ptw_mem_data_o  = dc_st_rdata_i;

  wire drain_done = drain_go && (drain_mmio ? mmio_ack_i : dc_st_done_i);
  wire amo_free   = sq_head_ready && sqh.is_amo && sqh.amo_done && hst == H_IDLE;   // 已执行的 AMO 提交后直接释放
  wire sq_pop     = drain_done || amo_free;
  always_ff @(posedge clk_i) begin
    // 无效项给 0 (上电后从未分配过的项载荷没有复位；分配时整项清零，有效项的地址总是确定值)
    pa_h0_q  <= sqv[sq_head.idx] ? sq[sq_head.idx].paddr : '0;
    pa_h1_q  <= sqv[SQ_W'(sq_head.idx + 1'b1)] ? sq[SQ_W'(sq_head.idx + 1'b1)].paddr : '0;
    pa_h2_q  <= sqv[SQ_W'(sq_head.idx + 2'd2)] ? sq[SQ_W'(sq_head.idx + 2'd2)].paddr : '0;
    pa_adv_q <= sq_pop;
    hc0_q    <= mk_hc(sqv[sq_head.idx], sq[sq_head.idx]);
    hc1_q    <= mk_hc(sqv[SQ_W'(sq_head.idx + 1'b1)], sq[SQ_W'(sq_head.idx + 1'b1)]);
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni && dc_st_req_o && !(ptw_own || ptw_take) && !vm_own)
      assert (dc_st_paddr_o == sqh.paddr) else $error("lsu: SQ 头地址副本与实际不符");
  // ptw_sel 但被 start_amo 挡住的那拍，原写法会落到"排空普通 store"分支：那时头是 AMO，那个分支本就不成立
  always_ff @(posedge clk_i)
    if (rst_ni && (hst == H_IDLE) && !vm_own && !ptw_own && ptw_sel && !ptw_take)
      assert (!(hs.ok && !hs.is_amo)) else $error("lsu: ptw_sel 选择与原 ptw_take 写法不等价 (头可排空)");
`endif

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      hst <= H_IDLE; amo_res_v <= 1'b0; lr_valid <= 1'b0; ptw_own <= 1'b0; drain_own <= 1'b0;
    end else begin
      if (lr_clear_i) lr_valid <= 1'b0;
      // D$ 口占用者
      if (ptw_take)                      ptw_own <= 1'b1;
      else if (ptw_own && dc_st_done_i)  ptw_own <= 1'b0;
      if (drain_go && !drain_mmio && !dc_st_done_i) drain_own <= 1'b1;
      else if (drain_own && dc_st_done_i)           drain_own <= 1'b0;
      if (rp_amo) amo_res_v <= 1'b0;       // 本拍注入
      unique case (hst)
        H_IDLE: begin
          if (start_mmio) begin hst <= H_MMIO_WAIT; h_lqi <= head_mmio_i; end
          else if (start_amo) hst <= H_AMO_WAIT;
        end
        H_MMIO_WAIT: if (mmio_ack_i) hst <= H_IDLE;
        H_AMO_WAIT: begin
          if (sqh.amo_op == AMO_SC && !sc_ok) begin
            hst <= H_IDLE; amo_res_v <= 1'b1; amo_idx <= sq_head.idx; amo_res <= 64'd1;
            lr_valid <= 1'b0;
          end else if (dc_st_done_i) begin
            hst <= H_IDLE; amo_res_v <= 1'b1; amo_idx <= sq_head.idx;
            if (sqh.amo_op == AMO_SC) begin amo_res <= 64'd0; lr_valid <= 1'b0; end
            else begin
              amo_res <= dc_st_rdata_i >> {sqh.paddr[2:0], 3'b000};
              if (sqh.amo_op == AMO_LR) begin lr_valid <= 1'b1; lr_addr <= sqh.paddr; end
            end
          end
        end
        default: hst <= H_IDLE;
      endcase
      if (flush_i) begin hst <= H_IDLE; amo_res_v <= 1'b0; end
    end
  end

  //==========================================================================
  // 向量单元访存口 (ROB 头的向量访存；更老的指令都已提交，等标量 SQ 排空后访问)
  //   T：查 DTLB (专用查找口)、判权限 / PMA / PMP；缺失等 A 空后走 PTW
  //   A：占 D$ store 口 (整行模式：读 = AMO_LR 只读，写 = 64 B 字节掩码)，连续的行背靠背
  //==========================================================================

  // T 拍：查 DTLB、页错误；C 拍：寄存的物理地址判 PMA / PMP (2026-10-25，6 GHz：TLB + PMP + 接收使能
  // 一拍串不下 (219 ps)，拆开后 T 的接收只等 TLB 命中、C 的前进只等 PMP)
  logic            vm_pmp_ok;
  o6_pmp #(.N(8)) u_vm_pmp (
    .dec_i(pmpd_i), .pa_i(vc_pa[PA_W-1:0]), .priv_i(dctx_i.priv),
    .acc_i(vc_we ? 3'b010 : 3'b001), .ok_o(vm_pmp_ok)
  );
  always_comb begin
    logic [63:0] pa64;
    logic [7:0]  pm8;
    logic [1:0]  pm;
    logic [3:0]  gp;
    vm_tmiss = 1'b0; vm_pf = 1'b0; vm_af = 1'b0; vm_gf = 1'b0;
    pa64 = vt_va; pm8 = dtlb_perm[LSU_NP]; gp = dtlb_gperm[LSU_NP];
    vm_gx = {7'b0, dtlb_gpa[LSU_NP][56:10], vt_va[11:2]};
    if (dctx_i.xl) begin
      if (dctx_i.cn && !va_canon(vt_va, dctx_i.vam)) vm_pf = 1'b1;
      else if (!dctx_i.cn && vt_va[63:VA_W] != '0) begin vm_gf = 1'b1; vm_gx = {7'b0, vt_va[58:2]}; end
      else if (!dtlb_hit[LSU_NP])               vm_tmiss = 1'b1;
      else if (dtlb_fault[LSU_NP] == 2'd1)      vm_pf = 1'b1;
      else if (dtlb_fault[LSU_NP] == 2'd2)      vm_af = 1'b1;
      else if (dtlb_fault[LSU_NP] == 2'd3) begin
        vm_gf = 1'b1;
        if (dtlb_gimp[LSU_NP]) vm_gx = {1'b1, 6'b0, dtlb_gpa[LSU_NP]};
      end else begin
        if (dctx_i.cn) begin
          if (dctx_i.priv == 2'b00 && !pm8[4])      vm_pf = 1'b1;
          if (dctx_i.priv == 2'b01 && pm8[4] && !dctx_i.sum) vm_pf = 1'b1;
          if (!pm8[6])                              vm_pf = 1'b1;
          if (!vt_we && !(pm8[1] || (dctx_i.mxr && pm8[3]))) vm_pf = 1'b1;
          if (vt_we && (!pm8[2] || !pm8[7]))        vm_pf = 1'b1;
        end
        if (!vt_we && !(gp[0] || (dctx_i.gmxr && gp[2]))) vm_gf = 1'b1;
        if (vt_we && !(gp[0] && gp[1] && gp[3]))  vm_gf = 1'b1;
      end
      pa64 = {{(64-PA_W){1'b0}}, dtlb_ppn[LSU_NP], vt_va[11:0]};
    end
    vm_tpa = pa64;
  end
  // C 拍：向量访存只支持主存 (MMIO / 不存在的区域报访问错误，与 AMO 相同)
  logic [1:0] vc_pm;
  logic       c_af;
  always_comb begin
    vc_pm = pma(vc_pa, dbg_mode_i);
    c_af  = vc_af || (!vc_pf && !vc_gf && (!vc_pm[1] || vc_pm[0] || !vm_pmp_ok));
  end

  // C -> A：C 检查通过且 A 空 (要先拿到 D$ 口：SQ 排空、口空闲、不是 D$ 刚完成 store 后预查下一条那拍) 或 A 本拍完成
  wire a_fin   = va_v && dc_st_done_i;
  wire c_ok    = vc_v && !vc_pf && !vc_gf && !c_af;
  wire take_ok = sq_drained_o && !ptw_own && !drain_own && (hst == H_IDLE) && !dc_done_q && !ptw_take;
  wire c2a     = c_ok && (a_fin || (!va_v && take_ok));
  // C 出错：等 A 空 (更老的那行已回) 再报，保证按序；T 里更年轻的那行同拍丢弃 (VU 出错即停，不再等它)
  wire c_flt   = vc_v && (vc_pf || vc_gf || c_af) && !va_v;
  // T -> C：命中 (或已判出页错误，带到 C 按序报) 且 C 空或本拍前进
  wire t2c     = vt_v && !vt_pw && !vm_tmiss && (!vc_v || c2a) && !c_flt;
  assign vm_ack_o = vm_req_i && !c_flt && (!vt_v || t2c);
  always_comb begin
    vm_done_o  = a_fin || c_flt;
    vm_fault_o = c_flt;
    vm_cause_o = vc_we ? (vc_pf ? o6_isa_pkg::EXC_STORE_PAGE : vc_gf ? o6_isa_pkg::EXC_STORE_GPAGE : o6_isa_pkg::EXC_STORE_ACCESS)
                       : (vc_pf ? o6_isa_pkg::EXC_LOAD_PAGE  : vc_gf ? o6_isa_pkg::EXC_LOAD_GPAGE  : o6_isa_pkg::EXC_LOAD_ACCESS);
  end
  assign vm_rdata_o = dc_st_lrdata_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vt_v <= 1'b0; vc_v <= 1'b0; va_v <= 1'b0; vt_pw <= 1'b0;
    end else begin
      if (vm_ack_o) vt_v <= 1'b1;
      else if (t2c || c_flt) vt_v <= 1'b0;
      if (t2c) vc_v <= 1'b1;
      else if (c2a || c_flt) vc_v <= 1'b0;
      if (c2a) va_v <= 1'b1;
      else if (a_fin) va_v <= 1'b0;
      // TLB 缺失：等 C、A 都空 (D$ 口放掉，PTW 要用；C 不会再出错丢弃 T) 再请求页表遍历，回填后重查
      if (dfill_i) vt_pw <= 1'b0;
      else if (vt_v && vm_tmiss && !va_v && !vc_v) vt_pw <= 1'b1;
    end
  end
  always_ff @(posedge clk_i) begin
    if (vm_ack_o) begin vt_va <= pm_x(vm_va_i, dctx_i.pmm, dctx_i.zx); vt_we <= vm_we_i; vt_wd <= vm_wdata_i; vt_wm <= vm_wmask_i; end
    if (t2c) begin vc_pa <= vm_tpa; vc_we <= vt_we; vc_pf <= vm_pf; vc_af <= vm_af; vc_gf <= vm_gf; vc_gx <= vm_gx; vc_wd <= vt_wd; vc_wm <= vt_wm; end
    if (c_flt) vm_x2_o <= vc_gf ? vc_gx : 64'd0;
    if (c2a) begin va_pa <= vc_pa[PA_W-1:0]; va_we <= vc_we; va_wd <= vc_wd; va_wm <= vc_wm; end
  end
`ifndef SYNTHESIS
  // C 出错丢弃 T 那行时它不能在做页表遍历 (遍历只在 C、A 都空时发起)
  always_ff @(posedge clk_i) if (rst_ni && c_flt && vt_v)
    assert (!vt_pw) else $error("lsu: 向量 C 级出错时 T 级正在页表遍历");
`endif

  //==========================================================================
  // LQ / SQ 状态更新
  //==========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      lqv <= '0; sqv <= '0;
      lq_head <= '0; lq_tail <= '0;
      sq_head <= '0; sq_cmt <= '0; sq_tail <= '0;
    end else begin
      // ---- 提交
      lq_ptr_t lh;
      sq_ptr_t sc, sh;
      lh = lq_inc(lq_head, {1'b0, cm_ld_cnt_i});
      for (int j = 0; j < W; j++) begin
        lq_ptr_t q;
        q = lq_inc(lq_head, 4'(j));     // VCS 2018 不支持对函数返回的结构体直接取成员
        if (3'(j) < cm_ld_cnt_i) lqv[q.idx] <= 1'b0;
      end
      lq_head <= lh;
      sc = sq_inc(sq_cmt, {1'b0, cm_st_cnt_i});
      sq_cmt <= sc;
      sh = sq_head;
      if (sq_pop) begin
        sqv[sq_head.idx] <= 1'b0;
        sh = sq_inc(sq_head, 4'd1);
      end
      sq_head <= sh;

      if (flush_i) begin
        // 全部未提交的作废；已提交未排空的 store 保留
        lqv <= '0;
        lq_tail <= lh;
        lq_head <= lh;
        for (int e = 0; e < NSQ; e++) begin
          sq_ptr_t ep;
          ep = '{flag: sq[e].flag, idx: SQ_W'(e)};
          if (!sq_older(ep, sc)) sqv[e] <= 1'b0;
        end
        sq_tail <= sc;
      end else if (br_upd_i.kill) begin
        for (int i = 0; i < NLQ; i++) if (lq[i].br_mask[br_upd_i.kill_tag]) lqv[i] <= 1'b0;
        for (int e = 0; e < NSQ; e++) if (sq[e].br_mask[br_upd_i.kill_tag] && !sq[e].committed) sqv[e] <= 1'b0;
        lq_tail <= br_upd_i.kill_lq;
        sq_tail <= br_upd_i.kill_sq;
      end else begin
        // ---- 分配
        logic [3:0] cl, cs;
        cl = '0; cs = '0;
        for (int s = 0; s < W; s++) begin
          if (lq_alloc_i[s]) begin lqv[alloc_uop_i[s].lq_idx.idx] <= 1'b1; cl++; end
          if (sq_alloc_i[s]) begin sqv[alloc_uop_i[s].sq_idx.idx] <= 1'b1; cs++; end
        end
        lq_tail <= lq_inc(lq_tail, cl);
        sq_tail <= sq_inc(sq_tail, cs);
      end
    end
  end

  // 表项内容 (无复位)。按项算下一值 (DC 2018 对"按动态下标写宽结构体数组"elaborate 极慢，
  // 综合结果也是每项一条长优先级链)。同一项同拍多个事件的优先级：
  //   掩码更新 < 等待条件满足 < 分配 < 重放选中 < L0 写地址 < L2 结果 < MMIO 读回
  // (实际上同一项不会同拍遇到两个互斥事件，这里只是把顺序写死)
  lq_ent_t lq_n [NLQ];
  sq_ent_t sq_n [NSQ];

  always_comb begin
    for (int i = 0; i < NLQ; i++) begin
      lq_ent_t e;
      e = lq[i];
      e.br_mask = lq[i].br_mask & ~br_upd_i.resolve_mask;

      // 等待条件满足 -> READY
      if (lqv[i] && lq[i].st == LS_WAIT) begin
        unique case (lq[i].wk)
          WK_SQ: begin
            sq_ent_t w;
            w = sq[lq[i].wk_sq.idx];
            if (!sqv[lq[i].wk_sq.idx] || w.flag != lq[i].wk_sq.flag || (w.data_ok && !w.is_amo && !w.mmio))
              e.st = LS_READY;
          end
          WK_MSHR: if (dc_refill_i && dc_refill_mshr_i == lq[i].wk_mshr) e.st = LS_READY;
          WK_SQA: begin
            sq_ent_t w;
            w = sq[lq[i].wk_sq.idx];
            if (!sqv[lq[i].wk_sq.idx] || w.flag != lq[i].wk_sq.flag || w.addr_ok) e.st = LS_READY;
          end
          default: ;
        endcase
      end

      // 分配
      for (int sl = 0; sl < W; sl++) begin
        if (lq_alloc_i[sl] && alloc_uop_i[sl].lq_idx.idx == LQ_W'(i)) begin
          uop_t u;
          u = alloc_uop_i[sl];
          e = '0;
          e.flag = u.lq_idx.flag; e.rob = u.rob_idx; e.br_mask = u.br_mask & ~br_upd_i.resolve_mask;
          e.sz = u.mem_sz; e.uns = u.mem_unsigned; e.rd_class = u.rd_class; e.fp_dbl = u.fp_dbl;
          e.pdst = u.pdst; e.sq_bound = u.sq_idx; e.st = LS_NEW; e.wk = WK_NONE;
          e.ld_wait = u.ld_wait; e.ftq_idx = u.ftq_idx; e.ftq_off = u.ftq_off;
        end
      end

      // 重放被选中
      if (rp_lq_v && !rp_amo && rp_lq_idx == LQ_W'(i)) e.st = LS_PIPE;

      // L0：写地址
      for (int k = 0; k < LSU_NP; k++) begin
        if (l0_live[k] && l0[k].u.fu == FU_LD && !l0[k].rp && !l0[k].tlbmiss && l0[k].lqi == LQ_W'(i)) begin
          e.addr_ok = 1'b1;      // 有异常的 load 也记地址 (只可能多报违例，它到头会陷入)
          e.paddr   = l0[k].pa;
          e.mask    = l0[k].mask;
          e.st      = LS_PIPE;
        end
      end

      // L1：mmio 位晚一拍写 (2026-10-27，6 GHz：L0 写时是 DTLB -> PMA -> 128 项扇出，183 ps)。
      // 它只有两个读者：违例检查 (L1 的 load 由 lq_pexcl 排除，这一拍看到旧值无妨) 与 L2 之后的重放 / 头部 MMIO
      for (int k = 0; k < LSU_NP; k++)
        if (alive(l1_v[k], l1[k], br_upd_i, flush_i) && l1[k].u.fu == FU_LD && !l1[k].rp && !l1[k].tlbmiss &&
            l1[k].lqi == LQ_W'(i))
          e.mmio = l1[k].mmio;

      // L2：结果
      for (int k = 0; k < LSU_NP; k++) begin
        if (l2_live[k] && l2[k].u.fu == FU_LD && l2[k].lqi == LQ_W'(i)) begin
          unique case (l2_res[k])
            R_OK, R_EXCP, R_REPLAY: e.st = LS_DONE;
            R_WAITSQ: begin e.st = LS_WAIT; e.wk = WK_SQ; e.wk_sq = l2_fw_wsq[k]; end
            R_WAITSA: begin e.st = LS_WAIT; e.wk = WK_SQA; e.wk_sq = l2_sa_wsq[k]; end
            R_MISS: begin
              // 回填完成脉冲可能正好与本 load 的 L2 同拍 (它在 L1 时还看到旧状态并挂到这个 MSHR)：
              // 这时直接可重放，否则会错过唤醒永远等下去
              if (dc_refill_i && dc_refill_mshr_i == dcl_mshr[k]) e.st = LS_READY;
              else begin e.st = LS_WAIT; e.wk = WK_MSHR; e.wk_mshr = dcl_mshr[k]; end
            end
            R_NACK: e.st = LS_READY;
            R_MMIO: begin e.st = LS_WAIT; e.wk = WK_HEAD; end
            default: ;
          endcase
        end
      end

      // MMIO 读回
      if (hst == H_MMIO_WAIT && mmio_ack_i && h_lqi == LQ_W'(i)) begin
        e.data = mmio_rdata_i; e.has_data = 1'b1; e.st = LS_READY;
      end
      lq_n[i] = e;
    end
  end

  always_comb begin
    for (int x = 0; x < NSQ; x++) begin
      sq_ent_t e;
      logic [SQ_W-1:0] d;
      e = sq[x];
      e.br_mask = sq[x].br_mask & ~br_upd_i.resolve_mask;

      // 提交：[sq_cmt, sq_cmt + 本拍提交数) 内的项标记已提交
      d = SQ_W'(x) - sq_cmt.idx;
      if ({1'b0, d} < (SQ_W+1)'(cm_st_cnt_i)) e.committed = 1'b1;

      // 分配
      for (int sl = 0; sl < W; sl++) begin
        if (sq_alloc_i[sl] && alloc_uop_i[sl].sq_idx.idx == SQ_W'(x)) begin
          uop_t u;
          u = alloc_uop_i[sl];
          e = '0;
          e.flag = u.sq_idx.flag; e.rob = u.rob_idx; e.br_mask = u.br_mask & ~br_upd_i.resolve_mask;
          e.sz = u.mem_sz; e.is_amo = (u.fu == FU_AMO); e.amo_op = u.op;
          e.pdst = u.pdst; e.rd_class = u.rd_class; e.lq_bound = u.lq_idx;
          e.data_ok = (u.fu == FU_AMO && amo_nodata(u.op));    // LR / CBO 没有数据
        end
      end

      // L0：STA / AMO 地址。不等 PMP / 未对齐等异常判定 (只等 DTLB 命中)：有异常的 store 提交时一定陷入，
      // 比它新、可能从它转发的 load 都会被冲掉；异常本身下一拍 (L1) 写进 fault
      for (int k = KS; k < LSU_NP; k++) begin
        if (l0_live[k] && (l0[k].u.fu == FU_ST || l0[k].u.fu == FU_AMO) && !l0[k].rp &&
            !l0[k].tlbmiss && l0[k].u.sq_idx.idx == SQ_W'(x)) begin
          e.addr_ok = 1'b1;
          e.paddr   = l0[k].pa;
          e.mask    = l0[k].mask;
        end
        if (l1_live_s[k-KS] && (l1[k].u.fu == FU_ST || l1[k].u.fu == FU_AMO) && !l1[k].rp &&
            !l1[k].tlbmiss && l1[k].u.sq_idx.idx == SQ_W'(x)) begin
          e.fault = l1e[k].excp;
          e.fchk  = 1'b1;
          // mmio 也在 L1 写 (同 LQ)：这一拍在 L1 的 load 看到旧值 0 也不会从 MMIO store 转发 ——
          // 同一 8 B 内的 load 必定也是 MMIO，L2 先判 R_MMIO，不看转发结果
          e.mmio  = l1[k].mmio;
        end
      end

      // STD
      for (int k = 0; k < N_STD; k++)
        if (std_valid_i[k] && std_sq_i[k].idx == SQ_W'(x)) begin e.data_ok = 1'b1; e.data = std_data_i[k]; end

      // AMO 执行完成
      if (hst == H_AMO_WAIT && ((sqh.amo_op == AMO_SC && !sc_ok) || dc_st_done_i) && sq_head.idx == SQ_W'(x))
        e.amo_done = 1'b1;
      sq_n[x] = e;
    end
  end

  always_ff @(posedge clk_i) begin
    for (int i = 0; i < NLQ; i++) lq[i] <= lq_n[i];
    for (int x = 0; x < NSQ; x++) sq[x] <= sq_n[x];
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni) begin
    assert (lq_cnt(lq_tail, lq_head) <= NLQ) else $error("lsu: LQ 计数越界");
    assert (sq_cnt(sq_tail, sq_head) <= NSQ) else $error("lsu: SQ 计数越界");
    // MMIO load 开始访问设备时，比它老的 store 必须都已离开 SQ (#27：提交包晚一拍时 sq_drained_o 曾误判)
    if (start_mmio)
      assert (sq_head == lq[head_mmio_i].sq_bound) else $error("lsu: MMIO load 越过了未排空的 store");
    // 违例口与 AMO 结果同拍不能出现两个重放源抢端口 0
    assert (!(rp_amo && rp_lq_v && 1'b0));
  end
`endif
endmodule
