//==============================================================================
// o6_frontend_m1.sv - M1 阶段前端：理想取指 + 预译码 + 静态预测 + RAS + FTQ + 取指缓冲
//
// 只用于 M1 打通后端；M2 换成 I$ + uBTB/BTB/TAGE-SC-L 的正式前端，FTQ/取指缓冲接口不变。
//
// 取指：每拍请求一个 32 B 对齐块，下一拍拿到数据 (imem 固定 1 拍)。
//   从 fetch PC 的块内偏移开始切指令；跨块的 RVI 低半部分存进 hw 缓冲，下一块补齐。
// 预测 (F1 组合)：JAL 必跳；条件分支向后跳 (BTFN)；返回 (JALR rs1∈{x1,x5}, rd=x0) 用 RAS；
//   其他 JALR 预测不跳 (必误预测，M1 接受)。组在第一条预测跳转处截断。
// 前端保证：送到后端的 JAL 一定预测为跳且目标正确；预测跳转的一定是控制流指令。
//==============================================================================
`default_nettype none
module o6_frontend_m1
  import o6_pkg::*;
  import o6_isa_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 取指存储 (I$)：请求被接受后若干拍 resp_valid；kill 丢弃在途响应
  output logic                imem_req_o,
  input  wire                 imem_ready_i,
  output logic [63:0]         imem_addr_o,      // 32 B 对齐
  input  wire                 imem_resp_valid_i,
  input  wire  [255:0]        imem_data_i,
  output logic                imem_kill_o,

  // 重定向
  input  wire                 be_redirect_i,    // 分支误预测
  input  wire  [63:0]         be_redirect_pc_i,
  input  wire  [FTQ_W-1:0]    be_redirect_ftq_i,
  input  wire                 cm_flush_i,       // 提交级冲刷
  input  wire  [63:0]         cm_redirect_pc_i,

  // 提交释放 FTQ
  input  wire                 cm_ftq_v_i,
  input  wire  [FTQ_W-1:0]    cm_ftq_idx_i,

  // FTQ 读口：0/1 = 分支单元，2 = 提交控制
  input  wire  [FTQ_W-1:0]    ftq_ridx_i [N_FTQ_RD],
  output logic [63:0]         ftq_pc_o [N_FTQ_RD],
  output logic [63:0]         ftq_tgt_o [N_FTQ_RD],

  // 去译码
  output logic [W-1:0]        ib_valid_o,
  output fetch_inst_t [W-1:0] ib_inst_o,
  input  wire  [2:0]          ib_pop_i,

  // MMU
  input  wire                 xlate_i,          // satp=Sv39 且 priv < M
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
  input  wire  [1:0]          vam_i,
  input  wire                 xv_i,
  input  wire                 cn_i,             // H：一级 (satp / vsatp) 翻译开 (vsatp = Bare 只有 G-stage 时为 0)
  output logic [63:0]         fgx_o,            // H：取指 guest page fault 的 {隐式, 6'b0, GPA[58:2]} (F3 交出异常时锁存)
  input  wire  [PA_W-13:0]    fill_ppn_i,
  input  wire  [7:0]          fill_perm_i,
  input  wire  [1:0]          fill_fault_i,
  input  wire  [3:0]          fill_gperm_i,
  input  wire  [56:0]         fill_gpa_i,
  input  wire                 fill_gimp_i
);
  localparam int NI = 17;   // 一组最多 16 条 (含跨块的那条)，留一位余量

  //--------------------------------------------------------------------------
  // FTQ
  //--------------------------------------------------------------------------
  logic [63:0]       ftq_pc  [FTQ_SIZE];
  logic [63:0]       ftq_tgt [FTQ_SIZE];
  logic [3:0]        ftq_ras [FTQ_SIZE];
  ftq_ptr_t          ftq_head, ftq_tail;
  wire  [FTQ_W:0]    ftq_cnt = (ftq_tail.flag == ftq_head.flag) ? {1'b0, ftq_tail.idx} - {1'b0, ftq_head.idx}
                                                               : {1'b0, ftq_tail.idx} + FTQ_SIZE - {1'b0, ftq_head.idx};
  wire               ftq_full = (ftq_cnt >= FTQ_SIZE - 1);

  always_comb begin
    for (int r = 0; r < N_FTQ_RD; r++) begin
      ftq_pc_o[r]  = ftq_pc[ftq_ridx_i[r]];
      ftq_tgt_o[r] = ftq_tgt[ftq_ridx_i[r]];
    end
  end

  //--------------------------------------------------------------------------
  // 取指缓冲
  //--------------------------------------------------------------------------
  fetch_inst_t       ib [IBUF_SIZE];
  logic [4:0]        ib_head, ib_tail;
  logic [5:0]        ib_cnt;

  always_comb begin
    for (int s = 0; s < W; s++) begin
      ib_inst_o[s]  = ib[5'(ib_head + s)];
      ib_valid_o[s] = (6'(s) < ib_cnt);
    end
  end

  //--------------------------------------------------------------------------
  // F1：切分与预测
  //--------------------------------------------------------------------------
  logic        f1_v;           // 有一个请求在途 (本拍数据有效)
  logic [63:0] f1_pc;          // 该请求对应的取指 PC (可能在块中间)
  logic        f1_fault;       // 本请求取指出错：不发 I$，直接送一条带异常的"指令"
  logic [5:0]  f1_cause;
  logic        hold_v;         // 停顿时保存的数据
  logic [255:0] hold_data;
  logic        hold_fault;
  logic [5:0]  hold_cause;
  logic        hw_v;           // 跨块 RVI 的低半部分
  logic [15:0] hw_d;
  logic [63:0] hw_pc;
  logic        stopped;        // 取指异常后停止，直到重定向
  logic [3:0]  ras_ptr;
  logic [4:0]  ras_cnt;        // 有效项数 (饱和)；为 0 时不做返回预测
  logic [63:0] ras [16];

  wire [255:0] blk    = hold_v ? hold_data  : imem_data_i;
  wire         fault  = hold_v ? hold_fault : f1_fault;
  wire         data_v = hold_v || imem_resp_valid_i || f1_fault;

  // 可执行区域：DRAM 与 BootROM
  function automatic logic exec_ok(input logic [63:0] a);
    return (a >= DRAM_BASE && a < DRAM_BASE + 64'h8000_0000) ||
           (a >= BOOTROM_BASE && a < BOOTROM_BASE + 64'h1_0000) ||
           (dbg_mode_i && a < 64'h1000);
  endfunction
  wire [63:0]  base  = {f1_pc[63:5], 5'b0};

  // 组内容
  logic [NI-1:0]  g_v;
  fetch_inst_t    g_i  [NI];
  logic [63:0]    g_pc [NI];
  logic [4:0]     g_n;
  logic           g_taken;
  logic [63:0]    g_npc;
  logic           g_hw_v;
  logic [15:0]    g_hw_d;
  logic [63:0]    g_hw_pc;
  logic [63:0]    g_start;
  logic           g_push, g_pop;
  logic [63:0]    g_push_val;

  logic [31:0] exp_in  [NI];
  logic [31:0] exp_out [NI];
  logic        exp_ill [NI];
  for (genvar i = 0; i < NI; i++) begin : g_exp
    o6_rvc_expand u_x (.inst_i(exp_in[i]), .inst_o(exp_out[i]), .ill_o(exp_ill[i]));
  end

  // 第一遍：找指令边界 (原始编码与 PC)
  logic [NI-1:0] r_v;
  logic [31:0]   r_raw [NI];
  logic [63:0]   r_pc  [NI];
  always_comb begin
    int pos, n;
    logic [15:0] h [16];
    for (int j = 0; j < 16; j++) h[j] = blk[16*j +: 16];
    r_v = '0; n = 0;
    g_hw_v = 1'b0; g_hw_d = '0; g_hw_pc = '0;
    for (int i = 0; i < NI; i++) begin r_raw[i] = '0; r_pc[i] = '0; end
    pos = int'(f1_pc[4:1]);
    if (hw_v && hw_pc + 64'd2 == base && f1_pc == base) begin
      r_v[0] = 1'b1; r_raw[0] = {h[0], hw_d}; r_pc[0] = hw_pc;
      n = 1; pos = 1;
    end
    for (int it = 0; it < 16; it++) begin
      if (pos < 16) begin
        if (h[pos][1:0] != 2'b11) begin
          r_v[n] = 1'b1; r_raw[n] = {16'b0, h[pos]}; r_pc[n] = base + 64'(2*pos);
          n = n + 1; pos = pos + 1;
        end else if (pos < 15) begin
          r_v[n] = 1'b1; r_raw[n] = {h[pos+1], h[pos]}; r_pc[n] = base + 64'(2*pos);
          n = n + 1; pos = pos + 2;
        end else begin
          g_hw_v = 1'b1; g_hw_d = h[15]; g_hw_pc = base + 64'd30;
          pos = 16;
        end
      end
    end
    for (int i = 0; i < NI; i++) exp_in[i] = r_raw[i];
  end

  // 第二遍：预测与截断
  always_comb begin
    logic stop;
    g_v = '0; g_n = '0; g_taken = 1'b0; g_npc = base + 64'd32;
    g_push = 1'b0; g_pop = 1'b0; g_push_val = '0;
    g_start = r_pc[0];
    stop = 1'b0;
    for (int i = 0; i < NI; i++) begin
      fetch_inst_t fi;
      logic [31:0] x;
      logic        rvc;
      g_pc[i] = r_pc[i];
      fi = '0;
      x = exp_out[i];
      rvc = (r_raw[i][1:0] != 2'b11);
      fi.inst = x; fi.raw = rvc ? {16'b0, r_raw[i][15:0]} : r_raw[i];
      fi.cls  = fe_cls(x);
      fi.is_rvc = rvc; fi.rvc_ill = rvc && exp_ill[i];
      fi.ftq_idx = ftq_tail.idx;
      fi.ftq_off = FTQ_OFF_W'((r_pc[i] - r_pc[0]) >> 1);
      if (r_v[i] && !stop) begin
        logic [6:0] opc;
        logic       is_jal, is_br, is_jalr, is_call, is_ret;
        logic [63:0] tgt;
        g_v[i] = 1'b1;
        g_n = g_n + 1'b1;
        opc = x[6:0];
        is_jal  = !fi.rvc_ill && opc == OPC_JAL;
        is_br   = !fi.rvc_ill && opc == OPC_BRANCH;
        is_jalr = !fi.rvc_ill && opc == OPC_JALR && x[14:12] == 3'b000;
        is_call = (is_jal || is_jalr) && (x[11:7] == 5'd1 || x[11:7] == 5'd5);
        is_ret  = is_jalr && x[11:7] == 5'd0 && (x[19:15] == 5'd1 || x[19:15] == 5'd5);
        tgt = '0;
        if (is_jal) tgt = r_pc[i] + {{43{x[31]}}, x[31], x[19:12], x[20], x[30:21], 1'b0};
        if (is_br)  tgt = r_pc[i] + {{51{x[31]}}, x[31], x[7], x[30:25], x[11:8], 1'b0};
        if (is_ret) tgt = ras[ras_ptr - 1'b1];
        // RAS 空时不预测返回 (否则目标是未定义值；仿真里 X 会让误预测判定失效)
        if (is_ret && ras_cnt == 5'd0) is_ret = 1'b0;
        if (is_jal || (is_br && x[31]) || is_ret) begin
          fi.pred_taken = 1'b1;
          g_taken = 1'b1; g_npc = tgt; stop = 1'b1;
        end
        if (is_call) begin g_push = 1'b1; g_push_val = r_pc[i] + (rvc ? 64'd2 : 64'd4); end
        if (is_ret)  g_pop = 1'b1;
        // JALR 调用：目标未知，预测不跳；但压栈仍要做
      end
      g_i[i] = fi;
    end
    // 取指异常：只送一条带异常的"指令"
    if (fault) begin
      g_v = '0; g_v[0] = 1'b1; g_n = 5'd1;
      g_i[0] = '0;
      g_i[0].fetch_excp = 1'b1; g_i[0].fetch_cause = hold_v ? hold_cause : f1_cause;
      g_i[0].ftq_idx = ftq_tail.idx;
      g_i[0].ftq_off = '0;
      g_start = f1_pc;
      if (hw_v && hw_pc + 64'd2 == base && f1_pc == base) begin
        g_start = hw_pc; g_i[0].fetch_hi = 1'b1;
      end
      g_taken = 1'b0;
    end
  end

  // 截断后不再保留跨块半条
  wire keep_hw = g_hw_v && !g_taken && !fault;

  // F1 是否能把这一组送出
  wire grp_any = |g_v;
  wire f1_ok   = f1_v && data_v && !stopped;
  wire room    = (6'(IBUF_SIZE) - ib_cnt + 6'(ib_pop_i)) >= 6'(g_n) && !ftq_full;
  wire f1_fire = f1_ok && room;
  wire redirect = be_redirect_i || cm_flush_i;

  // 下一次请求
  logic [63:0] nreq_pc;
  logic        nreq_v;
  logic [63:0] idle_pc;    // 停顿/空闲时要请求的 PC
  always_comb begin
    nreq_v  = 1'b0;
    nreq_pc = idle_pc;
    if (cm_flush_i)          begin nreq_v = 1'b1; nreq_pc = cm_redirect_pc_i; end
    else if (be_redirect_i)  begin nreq_v = 1'b1; nreq_pc = be_redirect_pc_i; end
    else if (f1_fire)        begin nreq_v = !fault; nreq_pc = g_npc; end
    else if (!f1_v && !stopped) begin nreq_v = 1'b1; nreq_pc = idle_pc; end
    // 处理当拍若 I$ 不接受，下拍从 idle_pc 重发 (idle_pc 记住了要请求的 PC)
  end
  //--------------------------------------------------------------------------
  // ITLB：请求 PC 先转换
  //--------------------------------------------------------------------------
  logic [VPN_W-1:0] itlb_vpn [1];
  logic [0:0]       itlb_hit;
  logic [PA_W-13:0] itlb_ppn [1];
  logic [7:0]       itlb_perm [1];
  logic [1:0]       itlb_fault [1];
  logic [3:0]       itlb_gperm [1];
  logic [56:0]      itlb_gpa   [1];
  logic             itlb_gimp  [1];
  assign itlb_vpn[0] = nreq_pc[VA_W-1:12];

  o6_tlb #(.N(32), .NL(1)) u_itlb (
    .clk_i, .rst_ni, .flush_i(tlb_flush_i),
    .lk_vpn_i(itlb_vpn), .lk_v_i(xv_i), .lk_hit_o(itlb_hit), .lk_ppn_o(itlb_ppn), .lk_perm_o(itlb_perm),
    .lk_fault_o(itlb_fault), .lk_pbmt_o(), .lk_gperm_o(itlb_gperm), .lk_gpa_o(itlb_gpa), .lk_gimp_o(itlb_gimp),
    .fill_i(ifill_i), .fill_v_i, .fill_vpn_i, .fill_level_i, .fill_nap_i, .fill_ppn_i, .fill_perm_i, .fill_pbmt_i(2'b00), .fill_fault_i,
    .fill_gperm_i, .fill_gpa_i, .fill_gimp_i
  );

  logic        ipmp_ok;
  logic [63:0] nreq_pa_c;
  o6_pmp #(.N(8)) u_ipmp (
    .dec_i(pmpd_i), .pa_i(nreq_pa_c[PA_W-1:0]), .priv_i(priv_i), .acc_i(3'b100),
    .ok_o(ipmp_ok)
  );
  assign nreq_pa_c = xlate_i ? {{(64-PA_W){1'b0}}, itlb_ppn[0], nreq_pc[11:0]} : nreq_pc;

  logic        nreq_miss;      // ITLB 缺失：本拍不发，等 PTW 填好后重试
  logic        nreq_flt;
  logic [5:0]  nreq_cause;
  logic [63:0] nreq_pa;
  always_comb begin
    logic [7:0] pm;
    nreq_miss = 1'b0; nreq_flt = 1'b0; nreq_cause = EXC_INST_ACCESS;
    nreq_pa = nreq_pc;
    pm = itlb_perm[0];
    if (xlate_i) begin
      if (!va_canon(nreq_pc, vam_i))  begin nreq_flt = 1'b1; nreq_cause = EXC_INST_PAGE; end
      else if (!itlb_hit[0])                    nreq_miss = 1'b1;
      else if (itlb_fault[0] == 2'd1)           begin nreq_flt = 1'b1; nreq_cause = EXC_INST_PAGE; end
      else if (itlb_fault[0] == 2'd2)           begin nreq_flt = 1'b1; nreq_cause = EXC_INST_ACCESS; end
      else if (!pm[3] || !pm[6] ||                              // X、A
               (priv_i == 2'b00 && !pm[4]) ||                   // U 态取非 U 页
               (priv_i == 2'b01 &&  pm[4]))                     // S 态不能执行 U 页
        begin nreq_flt = 1'b1; nreq_cause = EXC_INST_PAGE; end
      nreq_pa = {{(64-PA_W){1'b0}}, itlb_ppn[0], nreq_pc[11:0]};
    end
    if (!nreq_flt && !nreq_miss && (!exec_ok(nreq_pa) || !ipmp_ok)) begin nreq_flt = 1'b1; nreq_cause = EXC_INST_ACCESS; end
  end

  // 提交级冲刷 (陷入 / xRET / satp 写) 那一拍特权级与 satp 还是旧值：本拍只记 idle_pc 不发请求，
  // 下一拍用新的 priv / xlate 重新翻译 (否则 S 态陷入 M 态时 mtvec 会按 S 态页表翻译)
  wire   nreq_go     = nreq_v && !cm_flush_i;
  assign ptw_ireq_o = nreq_go && nreq_miss;
  assign ptw_ivpn_o = nreq_pc[VA_W-1:12];
  assign ptw_iv_o   = xv_i;
  assign fgx_o      = '0;                // M1 前端不做 H 的两级翻译检查 (FE1 配置不带 H)

  wire   nreq_exec = !nreq_flt;
  assign imem_req_o  = nreq_go && !nreq_flt && !nreq_miss;
  assign imem_addr_o = {nreq_pa[63:5], 5'b0};
  wire   req_sent    = nreq_go && !nreq_miss && (nreq_flt || imem_ready_i);
  assign imem_kill_o = redirect;

  //--------------------------------------------------------------------------
  // 状态更新
  //--------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      f1_v <= 1'b0; f1_pc <= '0; f1_fault <= 1'b0; hold_v <= 1'b0; hw_v <= 1'b0; stopped <= 1'b0;
      idle_pc <= RESET_PC;
      ras_ptr <= '0;
      ras_cnt <= '0;
      for (int i = 0; i < 16; i++) ras[i] <= '0;
      ftq_head <= '0; ftq_tail <= '0;
      ib_head <= '0; ib_tail <= '0; ib_cnt <= '0;
    end else begin
      // ---- 请求 / F1
      if (redirect || f1_fire || !f1_v) begin
        // 发新请求 (没被 I$ 接受就下拍从 idle_pc 重发)
        f1_v     <= req_sent;
        f1_fault <= nreq_go && nreq_flt;
        f1_cause <= nreq_cause;
        hold_v   <= 1'b0;
        if (nreq_v) f1_pc <= nreq_pc;
      end else if (data_v) begin
        // 数据到了但取指缓冲 / FTQ 没空间：存住，下拍继续处理同一组
        hold_v <= 1'b1;
        if (!hold_v) begin hold_data <= imem_data_i; hold_fault <= f1_fault; hold_cause <= f1_cause; end
      end
      // 否则：请求在途 (I$ 缺失)，等
      if (f1_fire) begin
        hw_v <= keep_hw; hw_d <= g_hw_d; hw_pc <= g_hw_pc;
        if (fault) stopped <= 1'b1;
        if (g_push) begin
          ras[ras_ptr] <= g_push_val; ras_ptr <= ras_ptr + 1'b1;
          if (ras_cnt != 5'd16) ras_cnt <= ras_cnt + 1'b1;
        end
        if (g_pop) begin
          ras_ptr <= ras_ptr - 1'b1;
          if (ras_cnt != 5'd0) ras_cnt <= ras_cnt - 1'b1;
        end
      end
      if (nreq_v) idle_pc <= nreq_pc;

      // ---- FTQ
      if (cm_ftq_v_i) ftq_head <= '{flag: (cm_ftq_idx_i < ftq_head.idx) ? ~ftq_head.flag : ftq_head.flag,
                                    idx: cm_ftq_idx_i};
      if (f1_fire && grp_any) begin
        ftq_pc[ftq_tail.idx]  <= g_start;
        ftq_tgt[ftq_tail.idx] <= g_npc;
        ftq_ras[ftq_tail.idx] <= ras_ptr;
        ftq_tail <= '{flag: ftq_tail.idx == FTQ_W'(FTQ_SIZE-1) ? ~ftq_tail.flag : ftq_tail.flag,
                      idx: ftq_tail.idx + 1'b1};
      end

      // ---- 取指缓冲
      begin
        logic [5:0] cnt;
        cnt = ib_cnt - 6'(ib_pop_i);
        ib_head <= ib_head + 5'(ib_pop_i);
        if (f1_fire) begin
          for (int i = 0; i < NI; i++)
            if (g_v[i]) ib[5'(ib_tail + i)] <= g_i[i];
          ib_tail <= ib_tail + 5'(g_n);
          cnt = cnt + 6'(g_n);
        end
        ib_cnt <= cnt;
      end

      // ---- 重定向 (覆盖以上)
      if (redirect) begin
        hw_v <= 1'b0; stopped <= 1'b0; hold_v <= 1'b0;
        ib_head <= '0; ib_tail <= '0; ib_cnt <= '0;
        if (cm_flush_i) begin
          ftq_head <= '0; ftq_tail <= '0;
        end else begin
          // 新尾 = 分支所在项 + 1。翻转位按与头的相对位置推：分支项一定在 [头, 尾) 内，
          // 且队列最多 31 项，所以新尾的下标不会等于头的下标
          logic [FTQ_W-1:0] ni;
          ni = be_redirect_ftq_i + 1'b1;
          ftq_tail <= '{flag: (ni > ftq_head.idx) ? ftq_head.flag : ~ftq_head.flag, idx: ni};
          ras_ptr <= ftq_ras[be_redirect_ftq_i];
        end
      end
    end
  end
endmodule
