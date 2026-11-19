//==============================================================================
// tb_top.sv - OOO6 系统级测试平台 (M1：理想存储 + MMIO 设备模型 + Spike 逐条对拍)
//
// plusargs:
//   +elf=<路径>        必需
//   +isa=<字符串>      默认 RV64IMAC_Zicsr_Zifencei
//   +max_cycles=<N>    默认 2000000
//   +trace=<文件>      对拍轨迹
//   +nocosim           只跑 DUT，不对拍 (看 tohost)
//
// 结束条件：tohost 被写 (1 = PASS，其它 = FAIL，值>>1 是 riscv-tests 的失败编号)；
//          对拍不一致；超时；连续 DEADLOCK_CYC 拍没有提交。
// 判定看日志里的 "=== PASS ===" / "=== FAIL"，不看退出码。
//==============================================================================
`timescale 1ns/1ps
`default_nettype none
`ifdef TB_SOC
  `define CORE u_soc.u_core
`else
  `define CORE u_core
`endif
`ifdef FE_M1
  `define FE g_fe1.u_fe
`else
  `define FE g_fe2.u_fe
  `define FE_M2
`endif

module tb_top;
  import o6_pkg::*;
`ifdef FE_M1
  localparam bit TB_FE2 = 1'b0;      // +define+FE_M1：M1 简化前端 (参考)
`else
  localparam bit TB_FE2 = 1'b1;
`endif
`ifdef NO_FPU
  localparam bit TB_FPU = 1'b0;
`else
  localparam bit TB_FPU = 1'b1;
`endif

  //--------------------------------------------------------------------------
  // DPI
  //--------------------------------------------------------------------------
  import "DPI-C" function int      cosim_init(input string elf, input string isa, input longint dram_size,
                                              input int verbose, input string trace);
  import "DPI-C" function longint  tb_mem_read64(input longint addr);
  import "DPI-C" function void     tb_mem_write64(input longint addr, input longint data, input int mask);
  import "DPI-C" function int      tb_mem_valid(input longint addr);
  import "DPI-C" function int      tb_tohost(output longint v);
  import "DPI-C" function longint  tb_tohost_addr();
  import "DPI-C" function int      cosim_commit(input longint pc, input int insn, input int rd_type,
                                                input int rd, input longint wdata);
  import "DPI-C" function int      cosim_trap(input int is_irq, input int cause, input longint epc, input longint tval);
  import "DPI-C" function longint  cosim_ncommit();
  import "DPI-C" function void     cosim_vabits(input int n);
  import "DPI-C" function void     cosim_trap_tgt(input int t);
  import "DPI-C" function void     cosim_set_reg(input int fp, input int r, input longint v);
  import "DPI-C" function longint  cosim_vreg(input int r, input int w);
  import "DPI-C" function longint  cosim_vcsr(input int which);
  import "DPI-C" function void     cosim_finish();

  localparam longint DRAM_SIZE   = 64'h8000_0000;
  localparam int     DEADLOCK_CYC = 20000;

  //--------------------------------------------------------------------------
  // 时钟复位
  //--------------------------------------------------------------------------
  logic clk = 1'b0, rst_n = 1'b0;
  always #0.5 clk = ~clk;
  longint cyc = 0;
  longint hb_n = 0, hb_pc = 0, hb_c0 = 0, hb_traps = 0;   // 心跳 (+hb=N)
  always @(posedge clk) cyc <= cyc + 1;

`ifdef TB_SOC
  //--------------------------------------------------------------------------
  // DUT：整个 SoC + AXI4 从机存储模型 (外设用 SoC 内的 CLINT / PLIC / UART)
  //--------------------------------------------------------------------------
  logic [3:0]   awid, bid, arid, rid;
  logic [PA_W-1:0] awaddr, araddr;
  logic [7:0]   awlen, arlen;
  logic [2:0]   awsize, arsize;
  logic [1:0]   awburst, arburst, bresp, rresp;
  logic         awvalid, awready, wlast, wvalid, wready, bvalid, bready;
  logic         arvalid, arready, rlast, rvalid, rready;
  logic [127:0] wdata, rdata;
  logic [15:0]  wstrb;
  logic         uart_tx;
  // JTAG (调试测试由 tb/top/tb_jtag.svh 驱动)
  logic         jtag_tck = 1'b0, jtag_tms = 1'b1, jtag_tdi = 1'b0, jtag_trst_n = 1'b0;
  wire          jtag_tdo;

  // UVM (+define+O6_UVM)：中断代理的 MEI / SEI 脉冲拉高 PLIC 源 2 / 源 3 的电平 (测试程序把源 2 配给 M 上下文、源 3 配给 S)；
  //   PLIC 是电平触发，TB 在该源被认领 (claimed 上升沿) 时撤掉电平 —— 相当于一个边沿触发的设备
  logic [1:0]   soc_eirq = '0;
  o6_soc #(.TICK_DIV(10), .HAS_FPU(TB_FPU), .HAS_MMU(1'b1), .FE_M2(TB_FE2)) u_soc (
    .clk_i(clk), .rst_ni(rst_n), .test_mode_i(1'b0),
    .m_awid(awid), .m_awaddr(awaddr), .m_awlen(awlen), .m_awsize(awsize), .m_awburst(awburst),
    .m_awvalid(awvalid), .m_awready(awready),
    .m_wdata(wdata), .m_wstrb(wstrb), .m_wlast(wlast), .m_wvalid(wvalid), .m_wready(wready),
    .m_bid(bid), .m_bresp(bresp), .m_bvalid(bvalid), .m_bready(bready),
    .m_arid(arid), .m_araddr(araddr), .m_arlen(arlen), .m_arsize(arsize), .m_arburst(arburst),
    .m_arvalid(arvalid), .m_arready(arready),
    .m_rid(rid), .m_rdata(rdata), .m_rresp(rresp), .m_rlast(rlast), .m_rvalid(rvalid), .m_rready(rready),
    .uart_rx_i(1'b1), .uart_tx_o(uart_tx), .ext_irq_i({6'b0, soc_eirq}),
    .jtag_tck_i(jtag_tck), .jtag_tms_i(jtag_tms), .jtag_tdi_i(jtag_tdi), .jtag_tdo_o(jtag_tdo),
    .jtag_tdo_oe_o(), .jtag_trst_ni(jtag_trst_n)
  );

  // AXI4 从机：读可多个在途 (AR 总是接收、排队)，每个读在 AR 后 axi_lat 拍可回，RID = ARID；
  //   +axilat=N 改延迟 (默认 20)，+axiooo=1 时就绪的读随机挑一个先回 (检查按 RID 回送)。
  //   写一次一个 (AW -> 4 拍 W -> B)，写数据在 W 拍直接进存储。
  int axi_lat = 20, axi_ooo = 0, axi_bp = 0;
  initial begin
    void'($value$plusargs("axilat=%d", axi_lat)); void'($value$plusargs("axiooo=%d", axi_ooo));
    void'($value$plusargs("axibp=%d", axi_bp));    // +axibp=P：AR / AW / W 每拍以 P% 概率不 ready (随机反压)
  end
  logic bp_ar = 1'b0, bp_aw = 1'b0, bp_w = 1'b0;
  always @(posedge clk) begin
    bp_ar <= (axi_bp > 0) && ($urandom_range(99) < axi_bp);
    bp_aw <= (axi_bp > 0) && ($urandom_range(99) < axi_bp);
    bp_w  <= (axi_bp > 0) && ($urandom_range(99) < axi_bp);
  end
`ifdef O6_UVM
  logic [1:0] plic_clm_q = '0;
  always @(posedge clk) begin
    logic [1:0] clm;
    clm = {u_soc.u_plic.claimed[3], u_soc.u_plic.claimed[2]};
    plic_clm_q <= clm;
    for (int b = 0; b < 2; b++) if (clm[b] && !plic_clm_q[b]) soc_eirq[b] <= 1'b0;   // 被认领：撤电平
    if (u_vif.meip_set) soc_eirq[0] <= 1'b1;
    if (u_vif.seip_set) soc_eirq[1] <= 1'b1;
  end
`endif
  typedef struct { longint addr; logic [3:0] id; longint due; logic [7:0] len; } ar_t;
  ar_t             arq [$];
  logic [PA_W-1:0] r_addr, w_addr;
  logic [3:0]      r_id;
  logic [7:0]      r_left;
  logic            r_busy, w_busy, w_data_done;
  assign arready = !bp_ar;
  assign awready = !w_busy && !bp_aw;
  assign wready  = w_busy && !w_data_done && !bp_w;
  assign bid = '0;
  assign bresp = 2'b00;
  assign rid = r_id;
  assign rresp = 2'b00;
  assign rvalid = r_busy;
  assign rlast  = (r_left == 8'd0);
  always @* for (int i = 0; i < 2; i++) rdata[64*i +: 64] = tb_mem_read64({{(64-PA_W){1'b0}}, r_addr} + 8*i);
  always @(posedge clk) begin
    if (!rst_n) begin
      r_busy <= 1'b0; w_busy <= 1'b0; w_data_done <= 1'b0; bvalid <= 1'b0; arq.delete();
    end else begin
      logic fin;
      fin = rvalid && rready && rlast;
      if (rvalid && rready) begin
        r_addr <= r_addr + 16;
        if (rlast) r_busy <= 1'b0; else r_left <= r_left - 1'b1;
      end
      // 开始下一个突发：挑一个到期的读
      if ((!r_busy || fin) && arq.size() > 0) begin
        int k, nr;
        k = -1; nr = 0;
        foreach (arq[i]) if (arq[i].due <= cyc) begin
          nr++;
          if (k < 0 || (axi_ooo != 0 && $urandom_range(nr - 1) == 0)) k = i;
          if (axi_ooo == 0) break;
        end
        if (k >= 0) begin
          r_busy <= 1'b1; r_addr <= arq[k].addr[PA_W-1:0]; r_id <= arq[k].id; r_left <= arq[k].len;
          arq.delete(k);
        end
      end
      if (arvalid && arready) begin
        assert (arlen == 8'(CL_B / 16 - 1)) else $error("tb AXI：只支持一行 (M1：128 B = 8 拍) 的读突发");
        arq.push_back('{addr: {{(64-PA_W){1'b0}}, araddr}, id: arid, due: cyc + axi_lat, len: arlen});
      end
      if (awvalid && awready) begin w_busy <= 1'b1; w_addr <= awaddr; w_data_done <= 1'b0; end
      if (wvalid && wready) begin
        for (int i = 0; i < 2; i++)
          tb_mem_write64({{(64-PA_W){1'b0}}, w_addr} + 8*i, wdata[64*i +: 64], wstrb[8*i +: 8]);
        w_addr <= w_addr + 16;
        if (wlast) begin w_data_done <= 1'b1; bvalid <= 1'b1; end
      end
      if (bvalid && bready) begin bvalid <= 1'b0; w_busy <= 1'b0; end
    end
  end

  // tohost (同 core 模式：监视 D$ store 口)
  longint tohost_a;
  longint tohost_v = 0;
  always @(posedge clk) if (rst_n) begin
    if (`CORE.dc_st_req && `CORE.dc_st_done && !`CORE.dc_st_amo &&
        {`CORE.dc_st_paddr[PA_W-1:3], 3'b000} == tohost_a[PA_W-1:0]) begin
      longint d;
      d = `CORE.dc_st_data;
      if (d != 0) tohost_v = d;
    end
  end

`else
  //--------------------------------------------------------------------------
  // DUT
  //--------------------------------------------------------------------------
  logic         mem_req_valid, mem_req_ready, mem_req_we, mem_resp_valid, mem_resp_ready;
  logic [PA_W-1:CL_O] mem_req_addr;
  logic [CL_W-1:0] mem_req_data, mem_resp_data;
  logic [3:0]   mem_req_id, mem_resp_id;
  logic         mmio_req, mmio_we, mmio_ack;
  logic [PA_W-1:0] mmio_addr;
  logic [1:0]   mmio_size;
  logic [63:0]  mmio_wdata, mmio_rdata;
  logic [7:0]   mmio_wmask;
  logic         mtip, msip;
  logic [63:0]  mtime;
  // 外部中断设备 (0x0200_C000，仅 +define+O6_UVM)：UVM 中断代理置位，程序写 bit0 / bit1 清 MEIP / SEIP，读回挂起位
  logic         meip_r = 1'b0, seip_r = 1'b0;

  o6_core #(.HAS_FPU(TB_FPU), .HAS_MMU(1'b1), .FE_M2(TB_FE2)) u_core (
    .clk_i(clk), .rst_ni(rst_n),
    .mem_req_valid_o(mem_req_valid), .mem_req_ready_i(mem_req_ready), .mem_req_we_o(mem_req_we),
    .mem_req_addr_o(mem_req_addr), .mem_req_data_o(mem_req_data), .mem_req_id_o(mem_req_id),
    .mem_resp_valid_i(mem_resp_valid), .mem_resp_ready_o(mem_resp_ready),
    .mem_resp_id_i(mem_resp_id), .mem_resp_data_i(mem_resp_data),
    .mmio_req_o(mmio_req), .mmio_we_o(mmio_we), .mmio_addr_o(mmio_addr), .mmio_size_o(mmio_size),
    .mmio_wdata_o(mmio_wdata), .mmio_wmask_o(mmio_wmask), .mmio_ack_i(mmio_ack), .mmio_rdata_i(mmio_rdata),
    .mtip_i(mtip), .msip_i(msip), .meip_i(meip_r), .seip_i(seip_r), .mtime_i(mtime), .dbg_haltreq_i(1'b0),
    .bist_start_i(1'b0), .bist_done_o(), .bist_fail_o()
  );

  //--------------------------------------------------------------------------
  // 行级存储模型：请求总是接受；写回立即写入；读在 mem_lat 拍后按序返回
  //   +memlat=N 改延迟 (默认 20；评估预取 / 访存并行度时用 100~200 模拟 DDR)
  //--------------------------------------------------------------------------
  int mem_lat = 20;
  initial void'($value$plusargs("memlat=%d", mem_lat));
  typedef struct { longint addr; int id; longint due; } rd_t;
  rd_t    rdq [$];
  assign mem_req_ready = 1'b1;
  always @* begin
    mem_resp_valid = (rdq.size() > 0) && (rdq[0].due <= cyc);
    mem_resp_id = '0; mem_resp_data = '0;
    if (rdq.size() > 0) begin
      mem_resp_id = rdq[0].id[3:0];
      for (int i = 0; i < CL_ND; i++) mem_resp_data[64*i +: 64] = tb_mem_read64(rdq[0].addr + 8*i);
    end
  end
  always @(posedge clk) if (!rst_n) rdq.delete();     // 复位时丢掉在途读 (+midrst：同 SoC 的 AXI 模型，互连随核一起复位)
  else begin
    if (mem_resp_valid && mem_resp_ready) void'(rdq.pop_front());
    if (mem_req_valid && mem_req_ready) begin
      longint a;
      a = {mem_req_addr, CL_O'(0)};
      if (mem_req_we) for (int i = 0; i < CL_ND; i++) tb_mem_write64(a + 8*i, mem_req_data[64*i +: 64], 8'hFF);
      else rdq.push_back('{addr: a, id: mem_req_id, due: cyc + mem_lat});
    end
  end

  // tohost：写回式 D$ 下 store 不会马上到存储，改为监视 D$ 的 store 口
  longint tohost_a;
  longint tohost_v = 0;
  always @(posedge clk) if (rst_n) begin
    if (`CORE.dc_st_req && `CORE.dc_st_done && !`CORE.dc_st_amo &&
        {`CORE.dc_st_paddr[PA_W-1:3], 3'b000} == tohost_a[PA_W-1:0]) begin
      longint d;
      d = `CORE.dc_st_data;
      if (d != 0) tohost_v = d;
    end
  end

  //--------------------------------------------------------------------------
  // MMIO：BootROM 数据读、CLINT、UART
  //--------------------------------------------------------------------------
  longint mtimecmp = -1;
  logic   msip_r = 1'b0;
  logic   mmio_busy;
  logic [2:0] mtime_div;
  assign mtip = (mtime >= mtimecmp);
  assign msip = msip_r;

  always @(posedge clk) begin
    if (!rst_n) begin
      mtime <= '0; mtime_div <= '0; mmio_busy <= 1'b0; mmio_ack <= 1'b0;
    end else begin
      mtime_div <= mtime_div + 1'b1;
      if (mtime_div == 3'd7) mtime <= mtime + 1;
      mmio_ack <= 1'b0;
      if (mmio_req && !mmio_busy) begin
        longint a, rd;
        a = {{(64-PA_W){1'b0}}, mmio_addr[PA_W-1:3], 3'b000};
        rd = 0;
        if (a >= 64'h1000 && a < 64'h11000) rd = tb_mem_read64(a);
        else if (a == 64'h0200_0000) begin
          rd = {63'b0, msip_r};
          if (mmio_we && mmio_wmask[0]) msip_r <= mmio_wdata[0];
        end else if (a == 64'h0200_4000) begin
          rd = mtimecmp;
          if (mmio_we) begin
            for (int b = 0; b < 8; b++) if (mmio_wmask[b]) mtimecmp[8*b +: 8] = mmio_wdata[8*b +: 8];
          end
        end else if (a == 64'h0200_BFF8) begin
          rd = mtime;
`ifdef O6_UVM
        end else if (a == 64'h0200_C000) begin
          rd = {62'b0, seip_r, meip_r};
          if (mmio_we && mmio_wmask[0]) begin
            if (mmio_wdata[0]) meip_r <= 1'b0;
            if (mmio_wdata[1]) seip_r <= 1'b0;
            // 软件置位 (riscv-arch-test 的 RVMODEL_SET_MEXT_INT / RVMODEL_SET_SEXT_INT)：bit2 置 MEIP、bit3 置 SEIP
            if (mmio_wdata[2]) meip_r <= 1'b1;
            if (mmio_wdata[3]) seip_r <= 1'b1;
          end
`endif
        end else if (a == 64'h1000_0000) begin
          // 16550：THR 在 +0，LSR 在 +5 (0x60 = 发送空)
          if (mmio_we && mmio_wmask[0]) $write("%c", mmio_wdata[7:0]);
          rd = {16'h0, 8'h60, 40'h0};
        end
        mmio_rdata <= rd;
        mmio_ack   <= 1'b1;
        mmio_busy  <= 1'b1;
      end else begin
        mmio_busy <= 1'b0;
      end
`ifdef O6_UVM
      if (u_vif.meip_set) meip_r <= 1'b1;
      if (u_vif.seip_set) seip_r <= 1'b1;
      // UVM CLINT 代理：置 msip / 让定时器在 tmr_delta 个 mtime 周期后到期 (覆盖本拍程序的写)
      if (u_vif.msip_set) msip_r <= 1'b1;
      if (u_vif.tmr_set)  mtimecmp = mtime + 64'(u_vif.tmr_delta);
`endif
    end
  end

`endif

  //--------------------------------------------------------------------------
  // 对拍
  //--------------------------------------------------------------------------
  string  elf, isa, trace;
  longint max_cycles;
  bit     do_cosim;
  longint last_commit_cyc = 0;
  longint n_commit = 0;
  longint n_icmt = 0;
  int     n_err = 0;
  // 结束：非 UVM 直接 $finish；UVM 下置 tb_done (非阻塞，同拍各块看到的一致)，由 UVM 测试收尾
  bit     tb_done = 0, tb_pass = 0, fin_called = 0;

  //--------------------------------------------------------------------------
  // 性能统计 (结束时打印)：前端供给、重命名/分派停顿、冲刷、每拍提交数分布
  //--------------------------------------------------------------------------
  longint pf_fe_e_ftq = 0, pf_fe_e_icm = 0;   // 取指缓冲空的原因：FTQ 满 / I$ 缺失
  longint pf_br_redirect = 0, pf_br_commit = 0, pf_flush = 0, pf_fe_empty = 0, pf_ren_stall = 0, pf_disp_stall = 0;
  longint pf_cmt_hist [7] = '{default: 0};
  longint pf_fl_replay = 0, pf_fl_trap = 0, pf_fl_after = 0, pf_viol = 0;
  longint pf_fe_ovr = 0, pf_fe_ovr_tk = 0, pf_fe_ovr_nt = 0, pf_fe_btb = 0;   // 前端 F3 覆盖 F0 的 uBTB 推测
  longint n_vacc = 0, n_vcmt = 0, n_vchk = 0;
  bit     v_gopen = 0;            // M3：展开的向量 load 提交到一半 (这期间不整体比对)    // 向量单元收下 / 提交的向量指令数、整体比对次数
  bit     v_dirty = 0, v_fpend = 0;
  logic [31:0] vh_inst [64];
  logic [63:0] vh_vt [64];
  longint      vh_vl [64];
  longint      vh_p = 0, vh_p0 = 0, vh_p0_n = 0;     // 收下的条数 / 上次比对时的条数
  bit hp_on = 0;                       // +headprof (见下)
  longint hp_cnt [longint];
  int     hp_fu  [longint];
  longint rd_t0 = 0, pf_rd_ib = 0, pf_rd_ds = 0, pf_rd_n = 0, pf_rd_dn = 0, pf_rd_ovr = 0;   // 重定向后的恢复时间
  int     rd_ib = 0, rd_ds = 0, rd_ovr = 0;
  longint pf_mis_cond = 0, pf_mis_cond_tk = 0, pf_mis_ret = 0, pf_mis_ind = 0, pf_mis_jal = 0;
  // 分派停顿按原因 (同一拍可多项)：ROB 满、各发射队列空位不够、LQ / SQ 空位不够
  longint pf_ds_rob = 0, pf_ds_lq = 0, pf_ds_sq = 0;
  // SQ 排出：出队数、占用、满 / 空、头部就绪待写 / 未提交；D$ store 状态机各状态拍数
  longint pf_sq_pair = 0, pf_sq_pop = 0, pf_sq_occ = 0, pf_sq_full = 0, pf_sq_empty = 0, pf_sq_hrdy = 0, pf_sq_hncm = 0;
  // 向量单元利用率 (P0 摸底)：请求等待、队列满、同步排空、访存、执行流水忙、访存口握手 / 背压 / 在途、完全空闲
  longint pf_v_req = 0, pf_v_qfull = 0, pf_v_sync = 0, pf_v_mem = 0, pf_v_exe = 0, pf_v_qocc = 0;
  longint pf_v_fast = 0;
  longint pf_vml = 0, pf_vmlp = 0, pf_vslow = 0;   // M3：向量 load 数据 uop 写完 / 写的遍数 / 要求走原路径
  longint pf_vms = 0, pf_vmsw = 0, pf_vms2 = 0;   // M3b：向量 store 数据 uop 收到数据 / 按行排空的次数 / 其中跨行的第一次
  longint pf_dr_rdy = 0, pf_dr_go = 0, pf_dr_cmt = 0, pf_dr_nc = 0, pf_dr_ss [8];   // M3b 调试：SQ 头 (向量 store) 可排空 / 正在排 / 已提交但数据或地址没好 / 没提交；可排空时 D$ store 状态
  initial foreach (pf_dr_ss[i]) pf_dr_ss[i] = 0;
  // M3b 调试：D$ store 进 S_MISS 时打印 (+smiss)
  bit smiss_on; logic [2:0] smiss_prev;
  initial smiss_on = $test$plusargs("smiss");
  always @(posedge clk) if (smiss_on) begin
    if (`CORE.u_dc.s_st == 3'd6 && smiss_prev != 3'd6)
      $display("[smiss %0d] spa=%h hitq=%h match=%b alloc=%b lkf=%b vms=%b vpass=%b", cyc, `CORE.u_dc.spa_q, `CORE.u_dc.s_hit_q,
               `CORE.u_dc.s_match, `CORE.u_dc.st_miss_alloc, `CORE.u_dc.s_lkf_q, `CORE.u_lsu.sqh.vms, `CORE.u_lsu.sqh.vpass);
    smiss_prev <= `CORE.u_dc.s_st;
    if (`CORE.u_lsu.drain_done && `CORE.u_lsu.sqh.vms)
      $display("[vdrain %0d] sq=%0d pa=%h vpa0=%h vnb=%0d vpass=%b nm=%0d line=%b", cyc, `CORE.u_lsu.sq_head.idx, `CORE.u_lsu.dc_st_paddr_o,
               `CORE.u_lsu.sqh.vpa0, `CORE.u_lsu.sqh.vnb, `CORE.u_lsu.sqh.vpass, $countones(`CORE.u_lsu.dc_st_lmask_o), `CORE.u_lsu.dc_st_line_o);
    if (`CORE.vsd_v)
      $display("[vsd %0d] sq=%0d rob=%0d/%0d", cyc, `CORE.vsd_sq.idx, `CORE.vsd_rob.row, `CORE.vsd_rob.col);
    if (`CORE.u_dc.f_st == 2'd1)
      $display("[evict %0d] set=%0d way=%0d old=%h v=%b new=%h", cyc, `CORE.u_dc.f_set, `CORE.u_dc.f_way,
               {`CORE.u_dc.tag[`CORE.u_dc.f_set][`CORE.u_dc.f_way], 5'(`CORE.u_dc.f_set), 7'd0},
               `CORE.u_dc.valid[`CORE.u_dc.f_set][`CORE.u_dc.f_way], {`CORE.u_dc.m_line[`CORE.u_dc.f_m], 7'd0});
  end
  longint pf_v_mreq = 0, pf_v_mpush = 0, pf_v_mbp = 0, pf_v_minfl = 0, pf_v_idle = 0, pf_v_cyc = 0;
  longint pf_dst [8] = '{default: 0};
  // 重命名停顿按原因：输出队列满 (被分派堵住)、分支标签不够、整数 / 浮点空闲寄存器不够
  longint pf_rn_q = 0, pf_rn_tag = 0, pf_rn_ifl = 0, pf_rn_ffl = 0;
  longint pf_ds_iq [N_IQ] = '{default: 0};
  // 发射队列占用：每拍累加 有效项数 / 操作数就绪项数 / 发射数，以及满的拍数 (判断是延迟受限还是端口受限)
  longint pf_iq_v [N_IQ] = '{default: 0}, pf_iq_r [N_IQ] = '{default: 0}, pf_iq_i [N_IQ] = '{default: 0}, pf_iq_full [N_IQ] = '{default: 0};
  always @(posedge clk) if (rst_n) begin
    int k;
    if (`CORE.redirect) pf_br_redirect++;
    // 后端重定向之后：到第一次写取指缓冲 / 第一次分派各几拍；其间 F3 覆盖几次
    if (`CORE.redirect || `CORE.flush) begin rd_t0 = cyc; rd_ib = 1; rd_ds = 1; rd_ovr = 0; end
    else begin
`ifdef FE_M2
      if (rd_ib && `CORE.`FE.f3_fire && (|`CORE.`FE.g_v)) begin pf_rd_ib += cyc - rd_t0; pf_rd_n++; rd_ib = 0; pf_rd_ovr += rd_ovr; end
      if (rd_ib && `CORE.`FE.mispred3) rd_ovr++;
`endif
      if (rd_ds && `CORE.rob_fire) begin pf_rd_ds += cyc - rd_t0; pf_rd_dn++; rd_ds = 0; end
    end
    pf_br_commit += $countones(`CORE.cm_is_br);
    // 误预测分类 (执行级，含错路上的)
    for (int p = 0; p < N_BRU; p++)
      if (`CORE.u_iexu.br_v[p] && `CORE.u_iexu.bru_mis[p]) begin
        uop_t u;
        u = `CORE.u_iexu.ex_u[p];
        if (u.op == BRU_JALR) begin
          if (u.lrd == 5'd0 && (u.lrs1 == 5'd1 || u.lrs1 == 5'd5)) begin
            pf_mis_ret++;
            if ($test$plusargs("showret") && pf_mis_ret <= 20)
              $display("[ret-mis %0d] pc=%h 预测跳=%0d 预测目标=%h 实际=%h", cyc, `CORE.u_iexu.ex_pc[p],
                       u.pred_taken, `CORE.u_iexu.ex_tgt[p], `CORE.u_iexu.bru_tgt[p]);
          end
          else pf_mis_ind++;
        end else if (u.op == BRU_JAL) pf_mis_jal++;
        else begin pf_mis_cond++; if (`CORE.u_iexu.bru_taken[p]) pf_mis_cond_tk++; end
      end
    if (`CORE.flush) begin
      pf_flush++;
      if (`CORE.u_cc.q_replay)    pf_fl_replay++;
      else if (`CORE.u_cc.q_trap) pf_fl_trap++;
      else                        pf_fl_after++;
    end
    if (`CORE.viol_any) begin
      pf_viol++;
      if ($test$plusargs("showviol") && pf_viol <= 30)
        begin
          logic [63:0] vp;
          vp = `CORE.`FE.ftq_pc[`CORE.viol_ftq] + {`CORE.viol_off, 1'b0};
          $display("[viol %0d] pc=%h", cyc, vp);
        end
    end
`ifdef FE_M2
    if (`CORE.`FE.f1_ovr) pf_fe_btb++;
    if (`CORE.`FE.mispred3) begin
      pf_fe_ovr++;
      if (`CORE.`FE.g_taken && !`CORE.`FE.f3_ubhit) begin
        pf_fe_ovr_tk++;
        if ($test$plusargs("showmiss") && pf_fe_ovr_tk <= 4000)
          $display("[miss] pc=%h slot=%0d cond=%0d ret=%0d", `CORE.`FE.f3_pc, `CORE.`FE.g_tslot, `CORE.`FE.g_tcond,
                   `CORE.`FE.g_tret);
      end
      if (!`CORE.`FE.g_taken && `CORE.`FE.f3_ubhit) pf_fe_ovr_nt++;
      if (`CORE.`FE.g_taken && `CORE.`FE.f3_ubhit && $test$plusargs("showmiss"))
        $display("[tgt] pc=%h slot=%0d/%0d ret=%0d npc=%h f3npc=%h", `CORE.`FE.f3_pc, `CORE.`FE.g_tslot,
                 `CORE.`FE.f3_ubslot, `CORE.`FE.g_tret, `CORE.`FE.g_npc, `CORE.`FE.f3_npc);
    end
`endif
    if (`CORE.ib_valid == '0) begin
      pf_fe_empty++;
      if (`CORE.`FE.ftq_full)             pf_fe_e_ftq++;
      else if (`CORE.u_ic.st >= 2)        pf_fe_e_icm++;    // I$ 缺失处理中 (I_MREQ 之后的状态)
    end
    if (`CORE.de_valid != '0 && !`CORE.rn_ready) begin
      pf_ren_stall++;
      if (|`CORE.u_rn.q1_v) pf_rn_q++;
      if (!`CORE.u_rn.tags_ok) pf_rn_tag++;
      if (`CORE.u_rn.ifl_cnt < `CORE.u_rn.n_i) pf_rn_ifl++;
      if (`CORE.u_rn.ffl_cnt < `CORE.u_rn.n_f) pf_rn_ffl++;
    end
    pf_iq_v[0] += $countones(`CORE.u_iqa.vld); pf_iq_r[0] += $countones(`CORE.u_iqa.rdy); pf_iq_i[0] += $countones(`CORE.u_iqa.issued);
    pf_iq_v[1] += $countones(`CORE.u_iqb.vld); pf_iq_r[1] += $countones(`CORE.u_iqb.rdy); pf_iq_i[1] += $countones(`CORE.u_iqb.issued);
    pf_iq_v[2] += $countones(`CORE.u_iqm.vld); pf_iq_r[2] += $countones(`CORE.u_iqm.rdy); pf_iq_i[2] += $countones(`CORE.u_iqm.issued);
    pf_iq_v[3] += $countones(`CORE.u_iqf.vld); pf_iq_r[3] += $countones(`CORE.u_iqf.rdy); pf_iq_i[3] += $countones(`CORE.u_iqf.issued);
    pf_iq_v[4] += $countones(`CORE.u_iqs.vld); pf_iq_r[4] += $countones(`CORE.u_iqs.rdy); pf_iq_i[4] += $countones(`CORE.u_iqs.issued);
    pf_iq_v[5] += $countones(`CORE.u_iqd.vld); pf_iq_r[5] += $countones(`CORE.u_iqd.rdy); pf_iq_i[5] += $countones(`CORE.u_iqd.issued);
    pf_iq_v[6] += $countones(`CORE.u_iqc.vld); pf_iq_r[6] += $countones(`CORE.u_iqc.rdy); pf_iq_i[6] += $countones(`CORE.u_iqc.issued);
    pf_iq_v[7] += $countones(`CORE.u_iqn.vld); pf_iq_r[7] += $countones(`CORE.u_iqn.rdy); pf_iq_i[7] += $countones(`CORE.u_iqn.issued);
    if (&`CORE.u_iqn.vld) pf_iq_full[7]++;
    if (&`CORE.u_iqa.vld) pf_iq_full[0]++;
    if (&`CORE.u_iqb.vld) pf_iq_full[1]++;
    if (&`CORE.u_iqm.vld) pf_iq_full[2]++;
    if (&`CORE.u_iqf.vld) pf_iq_full[3]++;
    if (&`CORE.u_iqs.vld) pf_iq_full[4]++;
    if (&`CORE.u_iqd.vld) pf_iq_full[5]++;
    if (&`CORE.u_iqc.vld) pf_iq_full[6]++;
    if (`CORE.rn_valid != '0 && !`CORE.ds_ready) begin
      pf_disp_stall++;
      if (!`CORE.u_ds.rob_ready_i) pf_ds_rob++;
      for (int q = 0; q < N_IQ; q++)
        if (7'(`CORE.u_ds.in_need_i.iqn[q]) > `CORE.u_ds.iq_free_i[q]) pf_ds_iq[q]++;
      if ((LQ_W+1)'(`CORE.u_ds.in_need_i.nld) > `CORE.u_ds.lq_free_i) pf_ds_lq++;
      if ((SQ_W+1)'(`CORE.u_ds.in_need_i.nst) > `CORE.u_ds.sq_free_i) pf_ds_sq++;
    end
    k = $countones(`CORE.cm_valid);
    pf_cmt_hist[k]++;
    pf_sq_pop += `CORE.u_lsu.sq_pop;   // r11 每拍至多出队 1 项 (成对合并版本见 *.sqpair)
`ifndef O6_NO_VEC
    if (n_vacc != 0) begin
      pf_v_cyc++;
      if (`CORE.u_vecu.req_v_i) pf_v_req++;
      pf_v_fast += $countones(`CORE.u_vecu.vxw_v_o);
      for (int k = 0; k < N_LDP; k++) begin
        if (`CORE.vlw_v[k]) pf_vmlp++;
        if (`CORE.vlw_v[k] && `CORE.vlw_last[k]) pf_vml++;
      end
      if (`CORE.vml_slow) pf_vslow++;
      if (`CORE.vsd_v) pf_vms++;
      if (`CORE.u_lsu.drain_done && `CORE.u_lsu.sqh.vms) pf_vmsw++;
      if (`CORE.u_lsu.vms_p0_done) pf_vms2++;
      if (`CORE.u_lsu.sqv[`CORE.u_lsu.sq_head.idx] && `CORE.u_lsu.sqh.vms) begin
        if (`CORE.u_lsu.sqh.committed && `CORE.u_lsu.sqh.addr_ok && `CORE.u_lsu.sqh.data_ok) begin
          pf_dr_rdy++; pf_dr_ss[`CORE.u_dc.s_st]++;
          if (`CORE.u_lsu.drain_go) pf_dr_go++;
        end else if (`CORE.u_lsu.sqh.committed) pf_dr_cmt++;
        else pf_dr_nc++;
      end
      if (`CORE.u_vecu.req_v_i && `CORE.u_vecu.rs == 2'd0 && `CORE.u_vecu.q_full) pf_v_qfull++;
      if (`CORE.u_vecu.rs == 2'd2) pf_v_sync++;
      if (`CORE.u_vecu.rs == 2'd3) pf_v_mem++;
      if (`CORE.u_vecu.x_busy || `CORE.u_vecu.s2_v || `CORE.u_vecu.s3_v) pf_v_exe++;
      pf_v_qocc += `CORE.u_vecu.q_n;
      if (`CORE.u_vecu.vm_req_o) pf_v_mreq++;
      if (`CORE.u_vecu.vm_req_o && `CORE.u_vecu.vm_ack_i) pf_v_mpush++;
      if (`CORE.u_vecu.vm_req_o && !`CORE.u_vecu.vm_ack_i) pf_v_mbp++;
      pf_v_minfl += `CORE.u_vecu.mq_n;
      if (`CORE.u_vecu.idle_o && !`CORE.u_vecu.req_v_i) pf_v_idle++;
    end
`endif
    pf_sq_occ += (1 << SQ_W) - `CORE.u_lsu.sq_free_o;
    if (`CORE.u_lsu.sq_free_o == 0) pf_sq_full++;
    if (`CORE.u_lsu.sq_free_o == (1 << SQ_W)) pf_sq_empty++;
    if (`CORE.u_lsu.hs.ok && !`CORE.u_lsu.hs.is_amo) pf_sq_hrdy++;
    if (`CORE.u_lsu.sqv[`CORE.u_lsu.sq_head.idx] && !`CORE.u_lsu.sqh.committed) pf_sq_hncm++;
    pf_dst[`CORE.u_dc.s_st]++;
`ifdef FE_M2
    // +headprof：零提交拍按 ROB 头指令的 PC 计数 (ROB 空的拍记 PC 0)，结束时写 headprof.txt
    if (hp_on && k == 0) begin
      int hr, hc;
      longint hp;
      hr = `CORE.u_rob.head_row; hc = `CORE.u_rob.head_col;
      hp = `CORE.u_rob.vld[hr][hc] ? `CORE.`FE.ftq_pc[`CORE.u_rob.ent[hr][hc].ftq_idx] + 64'(2 * `CORE.u_rob.ent[hr][hc].ftq_off) : 0;
      hp_cnt[hp]++;
      if (`CORE.u_rob.vld[hr][hc]) hp_fu[hp] = `CORE.u_rob.ent[hr][hc].fu;
    end
`endif
  end
`ifdef FE_M2
`ifndef O6_NO_VEC
  // +hot：两次提交 rdcycle 之间 (vbench 的热测区) 的统计，第二次时打印 [hot] 各行
  bit hw_on, hw_in;
  longint hw_c, hw_cm, hw_z, hw_vcm, hw_vblk, hw_rn, hw_rnq, hw_rnv, hw_ds, hw_ds_rob, hw_ds_sq, hw_ds_lq, hw_ds_iq, hw_ds_oth;
  longint hw_ds_vdq, hw_ds_viq, hw_vdqn, hw_sqfull, hw_sqpop, hw_vlw, hw_vsd, hw_vold, hw_vmem, hw_viqv, hw_viqi, hw_vsi, hw_ldiss;
  longint hw_xw [NVX];
  longint hw_xbusy [NVX];
  longint hw_dst [8];
  longint hw_hp [longint];
  int     hw_hf [longint];
  initial hw_on = $test$plusargs("hot");
  always @(posedge clk) if (hw_on && rst_n) begin
    int nrc, k;
    nrc = 0;
    for (int c = 0; c < W; c++)
      if (`CORE.cm_valid[c] && ((`CORE.u_rob.dbg_raw[`CORE.u_rob.head_row][c] & 32'hfff0707f) == 32'hc0002073)) nrc++;
    if (hw_in) begin
      k = $countones(`CORE.cm_valid);
      hw_c++; hw_cm += k;
      if (k == 0) begin
        int hr, hc;
        longint hp;
        hr = `CORE.u_rob.head_row; hc = `CORE.u_rob.head_col;
        hp = `CORE.u_rob.vld[hr][hc] ? `CORE.`FE.ftq_pc[`CORE.u_rob.ent[hr][hc].ftq_idx] + 64'(2 * `CORE.u_rob.ent[hr][hc].ftq_off) : 0;
        hw_z++; hw_hp[hp]++;
        if (`CORE.u_rob.vld[hr][hc]) hw_hf[hp] = `CORE.u_rob.ent[hr][hc].fu;
      end
      hw_vcm += `CORE.u_rob.vcm_o;
      if (`CORE.u_rob.vreach_o && !`CORE.u_rob.vcm_o) hw_vblk++;
      if (`CORE.de_valid != '0 && !`CORE.rn_ready) begin
        hw_rn++;
        if (|`CORE.u_rn.q1_v) hw_rnq++;
        if (!`CORE.u_rn.vfl_ge[`CORE.u_rn.n_v] || `CORE.u_rn.v_stall) hw_rnv++;
      end
      if (`CORE.rn_valid != '0 && !`CORE.ds_ready) begin
        logic any;
        any = 1'b0;
        hw_ds++;
        if (!`CORE.u_ds.rob_ready_i) begin hw_ds_rob++; any = 1'b1; end
        for (int q = 0; q < N_IQ; q++) if (7'(`CORE.u_ds.in_need_i.iqn[q]) > `CORE.u_ds.iq_free_i[q]) any = 1'b1;
        if (any && `CORE.u_ds.rob_ready_i) hw_ds_iq++;
        if ((LQ_W+1)'(`CORE.u_ds.in_need_i.nld) > `CORE.u_ds.lq_free_i) begin hw_ds_lq++; any = 1'b1; end
        if ((SQ_W+1)'(`CORE.u_ds.in_need_i.nst) > `CORE.u_ds.sq_free_i) begin hw_ds_sq++; any = 1'b1; end
        if (!any) hw_ds_oth++;
        if (!`CORE.vdq_ok) hw_ds_vdq++;
        if (!`CORE.viq_ok) hw_ds_viq++;
      end
      if (`CORE.u_lsu.sq_free_o == 0) hw_sqfull++;
      hw_vdqn += `CORE.vdq_n;
      hw_sqpop += `CORE.u_lsu.sq_pop;
      hw_dst[`CORE.u_dc.s_st]++;
      for (int p = 0; p < N_LDP; p++) if (`CORE.vlw_v[p]) hw_vlw++;
      if (`CORE.vsd_v) hw_vsd++;
      if (`CORE.u_vecu.x_busy || `CORE.u_vecu.s2_v || `CORE.u_vecu.s3_v) hw_vold++;
      if (`CORE.u_vecu.rs == 2'd3) hw_vmem++;
      hw_viqv += $countones(`CORE.u_viq.val);
      hw_viqi += $countones(`CORE.u_viq.xv);
      if (`CORE.u_viq.vs_v_o) hw_vsi++;
      for (int x = 0; x < NVX; x++) begin
        if (`CORE.u_vecu.xe_ok[x]) hw_xw[x]++;
        if (!`CORE.u_vecu.vx_rdy_o[x] || `CORE.u_viq.xv[x]) hw_xbusy[x]++;
      end
    end
    if (nrc != 0) begin
      if (!hw_in) begin
        hw_in = 1'b1;
        hw_c = 0; hw_cm = 0; hw_z = 0; hw_vcm = 0; hw_vblk = 0; hw_rn = 0; hw_rnq = 0; hw_rnv = 0; hw_ds = 0; hw_ds_rob = 0;
        hw_ds_sq = 0; hw_ds_lq = 0; hw_ds_vdq = 0; hw_ds_viq = 0; hw_vdqn = 0; hw_ds_iq = 0; hw_ds_oth = 0; hw_sqfull = 0; hw_sqpop = 0; hw_vlw = 0; hw_vsd = 0;
        hw_vold = 0; hw_vmem = 0; hw_viqv = 0; hw_viqi = 0; hw_vsi = 0;
        for (int x = 0; x < NVX; x++) begin hw_xw[x] = 0; hw_xbusy[x] = 0; end
        for (int s = 0; s < 8; s++) hw_dst[s] = 0;
        hw_hp.delete();
      end else begin
        hw_in = 1'b0;
        $display("[hot] 拍 %0d  提交 %0d (IPC %0.2f)  零提交 %0d 拍  向量提交 %0d (VEA 口被占 / 不收 %0d 拍)",
                 hw_c, hw_cm, 1.0 * hw_cm / hw_c, hw_z, hw_vcm, hw_vblk);
        $display("[hot] 重命名停 %0d (输出队列满 %0d、向量空闲表 / 在途写 %0d)  分派停 %0d (ROB %0d  IQ %0d  LQ %0d  SQ %0d  其它 (VIQ / VDQ 等) %0d)",
                 hw_rn, hw_rnq, hw_rnv, hw_ds, hw_ds_rob, hw_ds_iq, hw_ds_lq, hw_ds_sq, hw_ds_oth);
        $display("[hot] 分派停里：VDQ 满 %0d  VIQ 满 %0d  VDQ 平均占用 %0.2f / %0d", hw_ds_vdq, hw_ds_viq, 1.0 * hw_vdqn / hw_c, VDQ_N);
        $display("[hot] VIQ 平均占用 %0.2f  运算发射 %0d  store 数据 %0d  各执行器写回 %0d %0d %0d %0d  忙 %0d %0d %0d %0d",
                 1.0 * hw_viqv / hw_c, hw_viqi, hw_vsi, hw_xw[0], hw_xw[1], hw_xw[2], hw_xw[3], hw_xbusy[0], hw_xbusy[1], hw_xbusy[2], hw_xbusy[3]);
        $display("[hot] 向量 load 写回 %0d 行  SQ 出队 %0d  SQ 满 %0d 拍  D$ store 状态 %0d %0d %0d %0d %0d %0d %0d %0d  原路径执行忙 %0d  R_MEM %0d",
                 hw_vlw, hw_sqpop, hw_sqfull, hw_dst[0], hw_dst[1], hw_dst[2], hw_dst[3], hw_dst[4], hw_dst[5], hw_dst[6], hw_dst[7], hw_vold, hw_vmem);
        foreach (hw_hp[pc])
          if (hw_hp[pc] * 50 >= hw_z) $display("[hot] 零提交时 ROB 头 pc=%h fu=%0d：%0d 拍", pc, hw_hf.exists(pc) ? hw_hf[pc] : -1, hw_hp[pc]);
      end
    end
  end
`endif
`endif
  // +vlwlog：各 load 流水的向量 load 写回 / 向量 load uop 发射 (调试 load 吞吐)
  bit vlw_on;
  initial vlw_on = $test$plusargs("vlwlog");
  always @(posedge clk) if (vlw_on && rst_n) begin
    for (int p = 0; p < N_LDP; p++)
      if (`CORE.vlw_v[p]) $display("[vlw %0d] 流水 %0d -> p%0d last=%b", cyc, p, `CORE.vlw_pd[p], `CORE.vlw_last[p]);
  end
  initial hp_on = $test$plusargs("headprof");
  final if (hp_on) begin
    int fd;
    fd = $fopen("headprof.txt", "w");
    foreach (hp_cnt[pc]) $fdisplay(fd, "%h %0d %0d", pc, hp_cnt[pc], hp_fu.exists(pc) ? hp_fu[pc] : -1);
    $fclose(fd);
  end
  task automatic perf_report();
    if (n_vacc != 0)
      $display("[perf] 向量：收下 %0d 条、提交 %0d 条，整体比对 %0d 次", n_vacc, n_vcmt, n_vchk);
    if (pf_v_cyc != 0) begin
      $display("[vperf] 乱序 VX 执行 %0d 个 uop (O1)", pf_v_fast);
      $display("[vperf] 共用 LSU 的向量 load uop %0d 个 (写 %0d 遍)，转原路径 %0d 次 (M3)", pf_vml, pf_vmlp, pf_vslow);
      $display("[vperf] 共用 LSU 的向量 store uop %0d 个，按行排空 %0d 次 (其中跨行第一次 %0d) (M3b)", pf_vms, pf_vmsw, pf_vms2);
      $display("[vperf] SQ 头向量 store：可排空 %0d 拍 (其中在排 %0d)，已提交未就绪 %0d，未提交 %0d；可排空时 D$ store 状态 %0d %0d %0d %0d %0d %0d %0d %0d", pf_dr_rdy, pf_dr_go, pf_dr_cmt, pf_dr_nc, pf_dr_ss[0], pf_dr_ss[1], pf_dr_ss[2], pf_dr_ss[3], pf_dr_ss[4], pf_dr_ss[5], pf_dr_ss[6], pf_dr_ss[7]);
      $display("[vperf] 拍=%0d  执行流水忙 %0.1f%%  访存(R_MEM) %0.1f%%  同步排空 %0.1f%%  VECU 全空且无请求 %0.1f%%",
               pf_v_cyc, 100.0 * pf_v_exe / pf_v_cyc, 100.0 * pf_v_mem / pf_v_cyc, 100.0 * pf_v_sync / pf_v_cyc, 100.0 * pf_v_idle / pf_v_cyc);
      $display("[vperf] 请求在等 %0.1f%%  其中队列满 %0.1f%%  平均队列占用 %0.2f / 8",
               100.0 * pf_v_req / pf_v_cyc, 100.0 * pf_v_qfull / pf_v_cyc, 1.0 * pf_v_qocc / pf_v_cyc);
      $display("[vperf] 访存口：请求 %0d 拍  收下 %0d 行 (%0.3f 行/拍，R_MEM 内 %0.3f)  背压 %0d 拍  平均在途 %0.2f",
               pf_v_mreq, pf_v_mpush, 1.0 * pf_v_mpush / pf_v_cyc, 1.0 * pf_v_mpush / (pf_v_mem + 1), pf_v_mbp, 1.0 * pf_v_minfl / pf_v_cyc);
    end
    $display("[perf] 提交分支=%0d  误预测率=%0.2f%%  (每千条指令 %0.1f 次)", pf_br_commit,
             100.0 * pf_br_redirect / (pf_br_commit + 1), 1000.0 * pf_br_redirect / (n_commit + 1));
    $display("[perf] 误预测分类：条件 %0d (实际跳 %0d)  返回 %0d  间接 %0d  JAL %0d",
             pf_mis_cond, pf_mis_cond_tk, pf_mis_ret, pf_mis_ind, pf_mis_jal);
    if (pf_rd_n > 0 && pf_rd_dn > 0)
      $display("[perf] 重定向/冲刷后：到首次写取指缓冲平均 %.2f 拍 (其间 F3 覆盖 %0d 次)，到首次分派平均 %.2f 拍 (%0d 次)",
               1.0 * pf_rd_ib / pf_rd_n, pf_rd_ovr, 1.0 * pf_rd_ds / pf_rd_dn, pf_rd_dn);
    $display("[perf] F1 BTB 改道=%0d  F3 覆盖=%0d (漏跳 %0d  误跳 %0d  目标错 %0d)", pf_fe_btb, pf_fe_ovr,
             pf_fe_ovr_tk, pf_fe_ovr_nt, pf_fe_ovr - pf_fe_ovr_tk - pf_fe_ovr_nt);
    $display("[perf] 提交级冲刷分类：重放 %0d (其中访存违例 %0d)  陷入/中断 %0d  flush_after %0d",
             pf_fl_replay, pf_viol, pf_fl_trap, pf_fl_after);
    $display("[perf] 分支重定向=%0d  提交级冲刷=%0d  取指缓冲空=%0.1f%%  重命名停顿=%0.1f%%  分派停顿=%0.1f%%",
             pf_br_redirect, pf_flush, 100.0 * pf_fe_empty / cyc, 100.0 * pf_ren_stall / cyc,
             100.0 * pf_disp_stall / cyc);
    $display("[perf] 取指缓冲空的原因 (拍)：FTQ 满 %0d  I$ 缺失 %0d  其他 (重定向气泡等) %0d", pf_fe_e_ftq, pf_fe_e_icm,
             pf_fe_empty - pf_fe_e_ftq - pf_fe_e_icm);
    for (int q = 0; q < N_IQ; q++)
      $display("[perf] IQ_%s 平均：有效 %0.1f  就绪 %0.2f  发射 %0.2f /拍  满 %0.1f%%",
               (q == 0) ? "A" : (q == 1) ? "B" : (q == 2) ? "M" : (q == 3) ? "F" : (q == 4) ? "S" : (q == 5) ? "D" : (q == 6) ? "C" : "N",
               1.0 * pf_iq_v[q] / cyc, 1.0 * pf_iq_r[q] / cyc, 1.0 * pf_iq_i[q] / cyc, 100.0 * pf_iq_full[q] / cyc);
    $display("[perf] 重命名停顿原因 (拍)：输出队列满 %0d  分支标签 %0d  整数空闲寄存器 %0d  浮点空闲寄存器 %0d",
             pf_rn_q, pf_rn_tag, pf_rn_ifl, pf_rn_ffl);
    $display("[perf] 分派停顿原因 (拍)：ROB 满 %0d  IQ_A %0d  IQ_B %0d  IQ_C %0d  IQ_M %0d  IQ_N %0d  IQ_F %0d  IQ_S %0d  IQ_D %0d  LQ %0d  SQ %0d",
             pf_ds_rob, pf_ds_iq[0], pf_ds_iq[1], pf_ds_iq[6], pf_ds_iq[2], pf_ds_iq[7], pf_ds_iq[3], pf_ds_iq[4], pf_ds_iq[5], pf_ds_lq, pf_ds_sq);
    $display("[perf] SQ 排出：出队 %0d 个 (%0.3f /拍，成对 %0d 次)  平均占用 %0.1f  满 %0d 拍  空 %0d 拍  头部就绪 (可写 D$) %0d 拍  头部未提交 %0d 拍",
             pf_sq_pop, 1.0 * pf_sq_pop / cyc, pf_sq_pair, 1.0 * pf_sq_occ / cyc, pf_sq_full, pf_sq_empty, pf_sq_hrdy, pf_sq_hncm);
    $display("[perf] D$ store 状态机 (拍)：IDLE %0d  LOOK %0d  DONE %0d  MISS %0d  AMO/CBO %0d",
             pf_dst[0], pf_dst[1], pf_dst[5], pf_dst[6], pf_dst[2] + pf_dst[3] + pf_dst[4] + pf_dst[7]);
    $display("[perf] 每拍提交数分布 0..6: %0d %0d %0d %0d %0d %0d %0d", pf_cmt_hist[0], pf_cmt_hist[1],
             pf_cmt_hist[2], pf_cmt_hist[3], pf_cmt_hist[4], pf_cmt_hist[5], pf_cmt_hist[6]);
  endtask

  // +watch=<物理地址>：打印该 64 B 行在存储通路上的所有请求/响应 (调存储一致性问题用)
  longint watch_a = -1;
  initial void'($value$plusargs("watch=%h", watch_a));
  always @(posedge clk) if (rst_n && watch_a >= 0) begin
    logic [PA_W-1:CL_O] wl;
    wl = watch_a[PA_W-1:CL_O];
    if (`CORE.mem_req_valid_o && `CORE.mem_req_ready_i && `CORE.mem_req_addr_o == wl)
      $display("[watch %0d] 核->下游 %s id=%h d[63:0]=%h", cyc, `CORE.mem_req_we_o ? "写回" : "读", `CORE.mem_req_id_o,
               `CORE.mem_req_data_o[64*watch_a[CL_O-1:3] +: 64]);
    if (`CORE.u_dc.f_st == 2 && {`CORE.u_dc.tag[`CORE.u_dc.f_set][`CORE.u_dc.f_way], `CORE.u_dc.f_set} == wl &&
        `CORE.u_dc.valid[`CORE.u_dc.f_set][`CORE.u_dc.f_way])
      $display("[watch %0d] D$ 替换出该行 (dirty=%0d)", cyc, `CORE.u_dc.dirty[`CORE.u_dc.f_set][`CORE.u_dc.f_way]);
    if (`CORE.u_dc.f_st == 2 && `CORE.u_dc.m_line[`CORE.u_dc.f_m] == wl)
      $display("[watch %0d] D$ 回填该行", cyc);
    if (`CORE.u_dc.st_miss_alloc && `CORE.u_dc.spa[PA_W-1:CL_O] == wl)
      $display("[watch %0d] D$ store 缺失分配 MSHR%0d (f_st=%0d)", cyc, `CORE.u_dc.st_alloc_idx, `CORE.u_dc.f_st);
    for (int k = 0; k < N_LDP; k++)
      if (`CORE.u_dc.l1_alloc[k] && `CORE.u_dc.l1_alloc_ok[k] && `CORE.u_dc.ld_paddr_i[k][PA_W-1:CL_O] == wl)
        $display("[watch %0d] D$ load%0d 缺失分配 MSHR%0d (f_st=%0d hit=%b match=%b go=%b)", cyc, k,
                 `CORE.u_dc.l1_alloc_idx[k], `CORE.u_dc.f_st, `CORE.u_dc.l1_hit_oh[k], `CORE.u_dc.l1_match[k],
                 `CORE.u_dc.l1_go[k]);
    if (`CORE.dc_st_req && `CORE.dc_st_done && `CORE.dc_st_paddr[PA_W-1:CL_O] == wl)
      $display("[watch %0d] D$ store/AMO paddr=%h data=%h mask=%h amo=%0d", cyc, `CORE.dc_st_paddr, `CORE.dc_st_data,
               `CORE.dc_st_mask, `CORE.dc_st_amo);
`ifdef TB_SOC
    if (u_soc.mem_resp_valid && u_soc.mem_resp_ready && watch_a >= 0)
      ; // 响应不带地址，靠 id 对应
    if (u_soc.b_req_valid && u_soc.b_req_ready && u_soc.b_req_addr == wl)
      $display("[watch %0d] L2->桥 %s d[63:0]=%h", cyc, u_soc.b_req_we ? "写" : "读", u_soc.b_req_data[64*watch_a[CL_O-1:3] +: 64]);
`endif
  end

  // +lsuwin=<起始拍>：从该拍起逐拍打印 SQ 排空相关状态 (共 600 拍)，定位 store 排空卡死
  longint lsuwin = -1;
  initial void'($value$plusargs("lsuwin=%d", lsuwin));
  always @(posedge clk) if (rst_n && lsuwin >= 0 && cyc >= lsuwin && cyc < lsuwin + 600)
    $display("[lsu %0d] head=%0d/%0d cmt=%0d/%0d tail=%0d/%0d sqv_h=%0d com=%0d aok=%0d dok=%0d amo=%0d | hs.ok=%0d adv=%0d go=%0d own=%0d req=%0d done=%0d pop=%0d | flush=%0d cm_st=%0d dc_s=%0d",
             cyc, `CORE.u_lsu.sq_head.flag, `CORE.u_lsu.sq_head.idx, `CORE.u_lsu.sq_cmt.flag, `CORE.u_lsu.sq_cmt.idx,
             `CORE.u_lsu.sq_tail.flag, `CORE.u_lsu.sq_tail.idx, `CORE.u_lsu.sqv[`CORE.u_lsu.sq_head.idx],
             `CORE.u_lsu.sqh.committed, `CORE.u_lsu.sqh.addr_ok, `CORE.u_lsu.sqh.data_ok, `CORE.u_lsu.sqh.is_amo,
             `CORE.u_lsu.hs.ok, `CORE.u_lsu.pa_adv_q, `CORE.u_lsu.drain_go, `CORE.u_lsu.drain_own,
             `CORE.u_lsu.dc_st_req_o, `CORE.u_lsu.dc_st_done_i, `CORE.u_lsu.sq_pop, `CORE.u_lsu.flush_i,
             `CORE.u_lsu.cm_st_cnt_i, `CORE.u_dc.s_st);

  // 死锁时打印存储通路状态
  task automatic deadlock_dump();
    $display("[dump] D$: f_st=%0d s_st=%0d x_st=%0d wb_v=%0d wb_fill=%0d mem_req_v=%0d rdy=%0d we=%0d resp_v=%0d resp_rdy=%0d",
             `CORE.u_dc.f_st, `CORE.u_dc.s_st, `CORE.u_dc.x_st, `CORE.u_dc.wb_v, `CORE.u_dc.wb_fill,
             `CORE.u_dc.mem_req_valid_o, `CORE.u_dc.mem_req_ready_i, `CORE.u_dc.mem_req_we_o,
             `CORE.u_dc.mem_resp_valid_i, `CORE.u_dc.mem_resp_ready_o);
    for (int i = 0; i < 8; i++) $write("m_st[%0d]=%0d ", i, `CORE.u_dc.m_st[i]);
    $display("");
    $display("[dump] D$ store 口: req=%0d amo=%0d pa=%h spa_q=%h hit_q=%b lkf_q=%0d fill_all=%0d bank_use=%0d alloc_ok=%0d match=%b",
             `CORE.u_dc.st_req_i, `CORE.u_dc.st_amo_i, `CORE.u_dc.st_paddr_i, `CORE.u_dc.spa_q, `CORE.u_dc.s_hit_q,
             `CORE.u_dc.s_lkf_q, `CORE.u_dc.fill_all, `CORE.u_dc.st_bank_use, `CORE.u_dc.st_alloc_ok, `CORE.u_dc.s_match);
    $display("[dump] I$: st=%0d dead=%0d", `CORE.u_ic.st, `CORE.u_ic.dead);
    // ROB 头：卡在哪条指令、在等什么
    begin
      int hr, hc;
      hr = `CORE.u_rob.head_row; hc = `CORE.u_rob.head_col;
      $display("[dump] ROB 头 row=%0d col=%0d vld=%b fu=%0d op=%0d done_a=%0d done_b=%0d wait_head=%0d ld=%0d st=%0d excp=%0d pc=%h head_go=%0d",
               hr, hc, `CORE.u_rob.vld[hr], `CORE.u_rob.ent[hr][hc].fu, `CORE.u_rob.ent[hr][hc].op,
               `CORE.u_rob.ent[hr][hc].done_a, `CORE.u_rob.ent[hr][hc].done_b, `CORE.u_rob.ent[hr][hc].wait_head,
               `CORE.u_rob.ent[hr][hc].is_ld, `CORE.u_rob.ent[hr][hc].is_st, `CORE.u_rob.ent[hr][hc].excp,
`ifdef FE_M2
               `CORE.`FE.ftq_pc[`CORE.u_rob.ent[hr][hc].ftq_idx] + 64'(2 * `CORE.u_rob.ent[hr][hc].ftq_off),
`else
               64'd0,
`endif
               `CORE.head_go);
    end
    // 各 IQ 里有效的项 (看是谁在等什么)
    for (int i = 0; i < IQ_M_SIZE; i++) if (`CORE.u_iqm.vld[i])
      $display("[dump] IQ_M[%0d] isw=%0d rob=%0d fu=%0d b=%0d%0d%0d ps=%0d,%0d spo=%b spa=%b", i, `CORE.u_iqm.isw[i],
               `CORE.u_iqm.ent[i].rob_idx, `CORE.u_iqm.ent[i].fu, `CORE.u_iqm.ent[i].src1_busy, `CORE.u_iqm.ent[i].src2_busy,
               `CORE.u_iqm.ent[i].src3_busy, `CORE.u_iqm.ent[i].psrc1, `CORE.u_iqm.ent[i].psrc2, `CORE.u_iqm.spo[i], `CORE.u_iqm.spa[i]);
    for (int i = 0; i < IQ_A_SIZE; i++) if (`CORE.u_iqa.vld[i])
      $display("[dump] IQ_A[%0d] isw=%0d rob=%0d fu=%0d b=%0d%0d ps=%0d,%0d spo=%b spa=%b", i, `CORE.u_iqa.isw[i],
               `CORE.u_iqa.ent[i].rob_idx, `CORE.u_iqa.ent[i].fu, `CORE.u_iqa.ent[i].src1_busy, `CORE.u_iqa.ent[i].src2_busy,
               `CORE.u_iqa.ent[i].psrc1, `CORE.u_iqa.ent[i].psrc2, `CORE.u_iqa.spo[i], `CORE.u_iqa.spa[i]);
`ifndef O6_NO_VEC
    $display("[dump] VECU: rs=%0d ri=%h q_n=%0d x_busy=%0d m_fin=%0d m_i=%0d m_a=%h vm_req=%0d vl=%0d vill=%0d mq_n=%0d  LSU vt_v=%0d vt_pw=%0d va_v=%0d dc s_st=%0d",
             `CORE.u_vecu.rs, `CORE.u_vecu.ri, `CORE.u_vecu.q_n, `CORE.u_vecu.x_busy, `CORE.u_vecu.m_fin, `CORE.u_vecu.m_i,
             `CORE.u_vecu.m_a, `CORE.u_vecu.vm_req_o, `CORE.u_vecu.vl, `CORE.u_vecu.vill, `CORE.u_vecu.mq_n,
             `CORE.u_lsu.vt_v, `CORE.u_lsu.vt_pw, `CORE.u_lsu.va_v, `CORE.u_dc.s_st);
`endif
    for (int k = 0; k < 3; k++)
      $display("[dump] LSU 流水 %0d: rf_v=%0d ag_v=%0d l0_v=%0d l1_v=%0d l2_v=%0d", k, `CORE.u_lsu.rf_v[k], `CORE.u_lsu.ag_v[k],
               `CORE.u_lsu.l0_v[k], `CORE.u_lsu.l1_v[k], `CORE.u_lsu.l2_v[k]);
    $display("[dump] LSU 头部: hst=%0d head_mmio_v=%0d mmio_req=%0d we=%0d addr=%h ack=%0d sq_drained=%0d",
             `CORE.u_lsu.hst, `CORE.u_lsu.head_mmio_v, `CORE.u_lsu.mmio_req_o, `CORE.u_lsu.mmio_we_o,
             `CORE.u_lsu.mmio_addr_o, `CORE.u_lsu.mmio_ack_i, `CORE.u_lsu.sq_drained_o);
    $display("[dump] LSU store 口: ptw_own=%0d drain_own=%0d ptw_req=%0d ptw_gnt=%0d hs.ok=%0d hs.amo=%0d hs.mmio=%0d sq_head=%0d sq_cmt=%0d dc_done_q=%0d",
             `CORE.u_lsu.ptw_own, `CORE.u_lsu.drain_own, `CORE.u_lsu.ptw_mem_req_i, `CORE.u_lsu.ptw_mem_gnt_o,
             `CORE.u_lsu.hs.ok, `CORE.u_lsu.hs.is_amo, `CORE.u_lsu.hs.mmio, `CORE.u_lsu.sq_head.idx,
             `CORE.u_lsu.sq_cmt.idx, `CORE.u_lsu.dc_done_q);
    $display("[dump] PTW: st=%0d is_d=%0d vpn=%h  前端: pc0=%h f0_stop=%0d f0_umiss=%0d ireq=%0d f1_v=%0d f2_v=%0d f3_v=%0d ib_cnt=%0d ftq_full=%0d",
             `CORE.u_ptw.st, `CORE.u_ptw.is_d, `CORE.u_ptw.vpn,
`ifdef FE_M2
             `CORE.`FE.pc0, `CORE.`FE.f0_stop, `CORE.`FE.f0_umiss, `CORE.`FE.ptw_ireq_o, `CORE.`FE.f1_v,
             `CORE.`FE.f2_v, `CORE.`FE.f3_v, `CORE.`FE.ib_cnt, `CORE.`FE.ftq_full);
`else
             64'd0, 0, 0, 0, 0, 0, 0, 0, 0);
`endif
    $display("[dump] CSR: priv=%0d mip=%h mie=%h mstatus=%h",
             `CORE.u_csr.priv, `CORE.u_csr.mip, `CORE.u_csr.mie, `CORE.u_csr.mstatus);
    $display("[dump] core mem: req_v=%0d rdy=%0d we=%0d id=%h resp_v=%0d rdy=%0d id=%h",
             `CORE.mem_req_valid_o, `CORE.mem_req_ready_i, `CORE.mem_req_we_o, `CORE.mem_req_id_o,
             `CORE.mem_resp_valid_i, `CORE.mem_resp_ready_o, `CORE.mem_resp_id_i);
`ifdef TB_SOC
    $display("[dump] L2: st=%0d fq_n=%0d wb_v=%0d  bridge: w_act=%0d wt_n=%0d ar_v=%0d r_out=%0d fq_n=%0d  rl_v=%0d  axi arq=%0d",
             u_soc.g_l2.u_l2.st, u_soc.g_l2.u_l2.fq_n, u_soc.g_l2.u_l2.wb_v, u_soc.u_l2a.w_act, u_soc.u_l2a.wt_n,
             u_soc.u_l2a.ar_v, u_soc.u_l2a.r_out, u_soc.u_l2a.fq_n, u_soc.rl_v, arq.size());
    for (int i = 0; i < u_soc.g_l2.u_l2.NM; i++)
      $display("[dump] L2 MSHR%0d: st=%0d line=%h id=%0d ovr=%0d", i, u_soc.g_l2.u_l2.ms_st[i],
               u_soc.g_l2.u_l2.ms_line[i], u_soc.g_l2.u_l2.ms_id[i], u_soc.g_l2.u_l2.ms_ovr[i]);
`endif
  endtask

  task automatic finish(input bit pass, input string why);
    if (fin_called) return;
    fin_called = 1;
    perf_report();
    if (!pass) deadlock_dump();
    if (pass) $display("\n=== PASS === %s  cycles=%0d  commits=%0d  IPC=%0.3f", why, cyc, n_commit,
                       real'(n_commit) / real'(cyc));
    else      $display("\n=== FAIL === %s  cycles=%0d  commits=%0d", why, cyc, n_commit);
    if (do_cosim) cosim_finish();
    do_cosim = 0;
    tb_done <= 1'b1; tb_pass <= pass;
`ifndef O6_UVM
    $finish;
`endif
  endtask

  bit cmtlog_on;   // 调试：每个提交打印一行 (+cmtlog)
  initial cmtlog_on = $test$plusargs("cmtlog");
  // 提交监视 (拍沿前的组合值)
  always @(posedge clk) if (rst_n && !tb_done) begin
    for (int c = 0; c < W; c++) begin
      if (`CORE.u_rob.cm_valid_o[c]) begin
        logic [FTQ_W-1:0] fi;
        longint pc, wd;
        int insn, rdt, rd;
        fi   = `CORE.u_rob.hent[c].ftq_idx;
        pc   = `CORE.`FE.ftq_pc[fi] + {`CORE.u_rob.hent[c].ftq_off, 1'b0};
        insn = `CORE.u_rob.dbg_raw[`CORE.u_rob.head_row][c];
        rdt  = !`CORE.u_rob.cm_rd_valid_o[c] ? 0 : (`CORE.u_rob.cm_rd_fp_o[c] ? 2 : 1);
        rd   = `CORE.u_rob.cm_lrd_o[c];
        wd   = (rdt == 1) ? `CORE.u_iprf.mem[`CORE.u_rob.cm_pdst_o[c]] : 0;
`ifndef NO_FPU
        if (rdt == 2) wd = `CORE.g_fpu.u_fpu.u_fprf.mem[`CORE.u_rob.cm_pdst_o[c]];
`endif
        if (cmtlog_on) $display("[cmt %0d] #%0d pc=%h insn=%h fu=%0d vgrp=%b vmid=%b vlast=%b vea=%b vml=%b", cyc, n_commit, pc, insn,
                                `CORE.u_rob.hent[c].fu, `CORE.u_rob.hent[c].vgrp, `CORE.u_rob.hent[c].vmid, `CORE.u_rob.hent[c].vlast,
                                `CORE.u_rob.hent[c].vea, `CORE.u_rob.hent[c].fu == FU_LD);
        n_commit++;
        if (!(`CORE.u_rob.hent[c].vgrp && !`CORE.u_rob.hent[c].vlast)) n_icmt++;   // 按指令计 (展开的指令算一条，给 UVM 核对)
        if ((`CORE.u_rob.hent[c].fu == FU_VEC && !`CORE.u_rob.hent[c].vgrp) || `CORE.u_rob.hent[c].vea) n_vcmt++;   // vea：提交送 VECU (I1.3b)；M3 的检查 uop 不送 VECU
        if (`CORE.u_rob.hent[c].vgrp) v_gopen = !`CORE.u_rob.hent[c].vlast;   // M3：展开的指令提交到一半
        last_commit_cyc = cyc;
        hb_pc = pc;
        if (do_cosim && !(`CORE.u_rob.hent[c].vgrp && !`CORE.u_rob.hent[c].vlast) && cosim_commit(pc, insn, rdt, rd, wd) != 0) begin   // M3：展开的指令按一条对拍 (最后一个 uop)
          n_err++;
          finish(1'b0, "对拍不一致");
        end
      end
    end
`ifndef O6_NO_VEC
    // 向量单元：收下的向量指令都已提交 (Spike 也执行过) 且单元跑空时，整体比对向量寄存器与向量 CSR
    // I1.3b：提交口入队的也算收下 (与提交同拍)
    if (`CORE.u_vecu.cv_go_i) begin
      // V5：一拍至多 NVC 条 (头 + 跟着的切换标记)
      for (int k = 0; k < NVC; k++)
        if (3'(k) < `CORE.u_vecu.cv_gon_i) begin
          n_vacc++; v_dirty = 1;
          vh_inst[vh_p % 64] = (k == 0) ? `CORE.u_vecu.cv_inst_i : `CORE.u_vecu.cvx_inst_i[k];
          vh_vt[vh_p % 64] = `CORE.u_vecu.vtype_rd; vh_vl[vh_p % 64] = `CORE.u_vecu.vl;
          vh_p++;
        end
    end
    if (`CORE.u_vecu.rsp_v_o && !`CORE.u_vecu.rsp_excp_o) begin
      n_vacc++; v_dirty = 1;
      vh_inst[vh_p % 64] = `CORE.u_vecu.ri; vh_vt[vh_p % 64] = `CORE.u_vecu.vtype_rd; vh_vl[vh_p % 64] = `CORE.u_vecu.vl;
      vh_p++;
    end
    // 回了异常的向量指令 (访存出错会改 vstart / 已完成的元素)：Spike 在陷入提交时才执行它，等陷入发生再比
    if (`CORE.u_vecu.rsp_v_o && `CORE.u_vecu.rsp_excp_o) begin v_fpend = 1; v_dirty = 1; end
    if (`CORE.u_cc.trap_o || `CORE.flush) v_fpend = 0;
    if (do_cosim && v_dirty && !v_fpend && !v_gopen && !`CORE.u_cc.trap_o && `CORE.u_vecu.idle_o && n_vacc == n_vcmt) begin
      v_dirty = 0;
      n_vchk++;
      vh_p0_n = vh_p;
      for (int r = 0; r < 32; r++)
        for (int w = 0; w < VLEN / 64; w++) begin
          longint sv;
          sv = cosim_vreg(r, w);
          if (`CORE.u_vecu.vrf[r][64*w +: 64] !== sv) begin
            $display("[cosim] 向量寄存器不一致 (第 %0d 次比对，已提交 %0d 条) v%0d[%0d]: DUT=%h Spike=%h",
                     n_vchk, n_commit, r, w, `CORE.u_vecu.vrf[r][64*w +: 64], sv);
            n_err++;
          end
        end
      if (64'(`CORE.u_vecu.vl) != cosim_vcsr(0) || `CORE.u_vecu.vtype_rd != cosim_vcsr(1) ||
          64'(`CORE.u_vecu.vstart) != cosim_vcsr(2) || 64'(`CORE.u_vecu.vxsat) != cosim_vcsr(3) ||
          64'(`CORE.u_vecu.vxrm) != cosim_vcsr(4)) begin
        $display("[cosim] 向量 CSR 不一致: DUT vl=%0d vtype=%h vstart=%0d vxsat=%0d vxrm=%0d / Spike vl=%0d vtype=%h vstart=%0d vxsat=%0d vxrm=%0d",
                 `CORE.u_vecu.vl, `CORE.u_vecu.vtype_rd, `CORE.u_vecu.vstart, `CORE.u_vecu.vxsat, `CORE.u_vecu.vxrm,
                 cosim_vcsr(0), cosim_vcsr(1), cosim_vcsr(2), cosim_vcsr(3), cosim_vcsr(4));
        n_err++;
      end
      // fflags：向量浮点运算的标志由 VECU 迟到地并入 CSR；空闲时应与 Spike 一致
      //   (读的是沿前的值：CSR 里的 + 上一拍提交的 (cmq) + 本拍提交的 + VECU 本拍报的)
      begin
        logic [4:0] dff;
        dff = `CORE.u_csr.fflags | `CORE.cmq_fflags | `CORE.cm_fflags | `CORE.vecu_ff;
        if (64'(dff) != cosim_vcsr(5)) begin
          $display("[cosim] fflags 不一致: DUT=%b Spike=%b", dff, 5'(cosim_vcsr(5)));
          n_err++;
        end
      end
      if (n_err == 0) vh_p0 = vh_p0_n;
      if (n_err != 0) begin
        $display("[cosim] 上次比对以来收下的向量指令 (旧 -> 新)：");
        for (longint k = vh_p0; k < vh_p; k++)
          if (vh_p - k <= 64) $display("    inst=%h vtype=%h vl=%0d", vh_inst[k % 64], vh_vt[k % 64], vh_vl[k % 64]);
        finish(1'b0, "向量对拍不一致");
      end
    end
`endif
    if (`CORE.u_cc.trap_o) begin
      if (do_cosim) cosim_trap_tgt(`CORE.u_csr.trap_to_vs ? 2 : `CORE.u_csr.trap_to_s ? 1 : 0);
      if (do_cosim && cosim_trap(`CORE.u_cc.trap_irq_o, `CORE.u_cc.trap_cause_o,
                                 `CORE.u_cc.trap_epc_o, `CORE.u_cc.trap_tval_o) != 0) begin
        n_err++;
        finish(1'b0, "陷入不一致");
      end
    end
  end

  // +mmiolog=N：第 N 拍起打印核的每次 MMIO 读写 (完成那拍) 与每次陷入 (查外设 / 中断通路问题，如 Linux 串口)
  longint mmlog_c = -1;
  initial void'($value$plusargs("mmiolog=%d", mmlog_c));
  always @(posedge clk) if (rst_n && mmlog_c >= 0 && cyc >= mmlog_c) begin
    if (`CORE.u_lsu.mmio_req_o && `CORE.u_lsu.mmio_ack_i)
      $display("[mmio %0d] %s %h sz=%0d d=%h m=%h", cyc, `CORE.u_lsu.mmio_we_o ? "W" : "R", `CORE.u_lsu.mmio_addr_o,
               `CORE.u_lsu.mmio_size_o, `CORE.u_lsu.mmio_we_o ? `CORE.u_lsu.mmio_wdata_o : `CORE.u_lsu.mmio_rdata_i,
               `CORE.u_lsu.mmio_wmask_o);
    if (`CORE.u_cc.trap_o)
      $display("[trap %0d] irq=%0d cause=%0d epc=%h tval=%h priv=%0d", cyc, `CORE.u_cc.trap_irq_o, `CORE.u_cc.trap_cause_o,
               `CORE.u_cc.trap_epc_o, `CORE.u_cc.trap_tval_o, `CORE.u_csr.priv);
  end

  // 心跳 (+hb=N)：每 N 拍打印进度，区分"慢"与"活锁" (活锁时 PC 在少数几个值之间打转、陷入数暴涨)
  initial void'($value$plusargs("hb=%d", hb_n));
  always @(posedge clk) if (rst_n && hb_n != 0) begin
    if (`CORE.u_cc.trap_o) hb_traps++;
    if (cyc % hb_n == 0) begin
      $display("[hb] cyc=%0d commits=%0d (+%0d) pc=%h priv=%0d traps=%0d", cyc, n_commit, n_commit - hb_c0,
               hb_pc, `CORE.u_csr.priv, hb_traps);
      $fflush;                                 // VCS 输出重定向到文件时有缓冲：长程序要实时看进度
      hb_c0 = n_commit; hb_traps = 0;
    end
  end

  // 结束判定
  always @(posedge clk) if (rst_n) begin
    longint v;
    v = tohost_v;
    // 最高位为 1：程序报一个数 (性能基准的测量值)，打印后继续
    if (v[63]) begin
      $display("[bench] %0d", v[62:0]);
      tohost_v = 0;
      v = 0;
    end
    if (v != 0) begin
      if (v == 1) finish(1'b1, "tohost=1");
      else begin
        $display("tohost=0x%0h (测试编号 %0d 失败)", v, v >> 1);
        finish(1'b0, "tohost!=1");
      end
    end
    if (cyc > max_cycles) finish(1'b0, "超时");
`ifdef TB_SOC
    if (!u_soc.rst_n) last_commit_cyc = cyc;           // 系统被 ndmreset 保持复位 (MBIST 等)：不算死锁
`endif
    if (cyc - last_commit_cyc > DEADLOCK_CYC) finish(1'b0, "死锁 (长时间无提交)");
  end

  int vabits;
  initial begin
    if (!$value$plusargs("elf=%s", elf)) begin $display("需要 +elf="); $finish; end
    if (!$value$plusargs("isa=%s", isa)) isa = TB_FPU ? "RV64IMAFDCVH_Zicsr_Zifencei_Zicntr_Zihpm_Zba_Zbb_Zbs_Zfa_Zicond_Zcb_Zimop_Zcmop_Zawrs_Sstc_Sscofpmf_Smstateen_Svinval_Svnapot_Svpbmt_Smnpm_Ssnpm_Zicbom_Zicboz_Zicbop_Zvl1024b_Zvbb_Zvfh_Zvfbfmin_Zvfbfwma_Zvfofp8min_Zvqwdota8i_Zvqwdota16i_Zvfqwdota8f_Zvfwdota16bf" : "RV64IMACH_Zicsr_Zifencei_Zicntr_Zihpm_Zba_Zbb_Zbs_Zicond_Zcb_Zimop_Zcmop_Zawrs_Sstc_Sscofpmf_Smstateen_Svinval_Svnapot_Svpbmt_Smnpm_Ssnpm_Zicbom_Zicboz_Zicbop";
    if (!$value$plusargs("max_cycles=%d", max_cycles)) max_cycles = 2000000;
    if (!$value$plusargs("vabits=%d", vabits)) vabits = 57;   // Sv57 (RVA23 R4)：Spike 的 satp 可写模式
    if (!$value$plusargs("trace=%s", trace)) trace = "";
    do_cosim = !$test$plusargs("nocosim");
    cosim_vabits(vabits);
    if (cosim_init(elf, isa, DRAM_SIZE, 0, trace) != 0) begin $display("cosim_init 失败"); $finish; end
    tohost_a = tb_tohost_addr();
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
  end

  // +midrst=N：运行中复位 (docs/13 §16 P3)。第 N 拍拉低复位 20 拍再放开；期间对拍库重新载入 ELF、Spike 从复位向量重来，
  //   DUT 与 Spike 一起把整个程序再跑一遍 (存储被 ELF 映像覆盖；ELF 之外被改过的区域保持，用例要自己初始化数据)。
  //   查：复位后缓存 / MSHR / L2 / AXI 在途事务都干净，不会把复位前的残留交给新程序
  longint midrst_c = -1;
  logic   midrst_evt = 1'b0;      // 给 UVM 计分板：PC 流不连续 (同进调试)
  int     n_midrst = 0;
  initial if ($value$plusargs("midrst=%d", midrst_c)) begin
    wait (rst_n);
    wait (cyc >= midrst_c);
    @(negedge clk);
    rst_n = 1'b0; midrst_evt = 1'b1; n_midrst++;
    repeat (20) @(posedge clk);
    if (do_cosim && cosim_init(elf, isa, DRAM_SIZE, 0, trace) != 0) begin $display("cosim_init (复位后) 失败"); $finish; end
    // 复位后 x1..x31 / f0..f31 的值规范上不确定：RAT 复位成恒等映射 (x_i -> p_i)，物理寄存器保留旧值 -> 以 DUT 为准告诉 Spike
    if (do_cosim) for (int r = 1; r < 32; r++) cosim_set_reg(0, r, `CORE.u_iprf.mem[r]);
`ifndef NO_FPU
    if (do_cosim) for (int r = 0; r < 32; r++) cosim_set_reg(1, r, `CORE.g_fpu.u_fpu.u_fprf.mem[r]);
`endif
    v_dirty = 0; v_fpend = 0; n_vacc = n_vcmt;
    last_commit_cyc = cyc;
    $display("[midrst] 第 %0d 拍复位 20 拍，已提交 %0d 条；重新开始", midrst_c, n_commit);
    @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);
    midrst_evt = 1'b0;
  end

  //--------------------------------------------------------------------------
  // 调试监视 (+debug)：重定向、分支广播、陷入、发射、写回
  //--------------------------------------------------------------------------
`ifdef FE_M2
  always @(posedge clk) if (rst_n && $test$plusargs("showviol") && cyc < 20000) begin
    if (`CORE.`FE.f3_fire)
      for (int i = 0; i < 17; i++)
        if (`CORE.`FE.g_v[i] && `CORE.`FE.g_i[i].ld_wait)
          $display("[%0d] F3 ld_wait slot%0d blk=%h", cyc, i, `CORE.`FE.f3_pc);
  end
`endif
  // +showviol：带 ld_wait 的 load 在 LSU 的经过 + 违例时的 store / load
  always @(posedge clk) if (rst_n && $test$plusargs("showviol") && cyc < 3000) begin
    for (int k = 0; k < N_LDP; k++)
      if (`CORE.u_lsu.l1_v[k] && `CORE.u_lsu.l1[k].u.fu == FU_LD)
        $display("[%0d] L1.%0d ld lq=%0d sq=%0d.%0d ldw=%0d sa_wait=%0d pre=%0d rp=%0d", cyc, k,
                 `CORE.u_lsu.l1[k].u.lq_idx.idx, `CORE.u_lsu.l1[k].u.sq_idx.flag, `CORE.u_lsu.l1[k].u.sq_idx.idx,
                 `CORE.u_lsu.l1[k].u.ld_wait, `CORE.u_lsu.sa_wait[k], `CORE.u_lsu.l1[k].pre, `CORE.u_lsu.l1[k].rp);
    if (`CORE.u_lsu.sta_l0)
      $display("[%0d] STA sq=%0d.%0d pa=%h", cyc, `CORE.u_lsu.l0[LSU_KS].u.sq_idx.flag, `CORE.u_lsu.l0[LSU_KS].u.sq_idx.idx,
               `CORE.u_lsu.l0[LSU_KS].pa);
    if (`CORE.viol_any)
      $display("[%0d] VIOL", cyc);
  end
  bit dbg;
  initial dbg = $test$plusargs("debug");
  always @(posedge clk) if (rst_n && dbg) begin
    if (`CORE.redirect)
      $display("[%0d] BR-REDIRECT pc=%h ftq=%0d", cyc, `CORE.redirect_pc, `CORE.redirect_ftq);
    if (`CORE.br_upd.kill || `CORE.br_upd.resolve_mask != 0)
      $display("[%0d] BRUPD resolve=%h kill=%0d tag=%0d rob=%0d.%0d", cyc, `CORE.br_upd.resolve_mask,
               `CORE.br_upd.kill, `CORE.br_upd.kill_tag, `CORE.br_upd.kill_rob.row, `CORE.br_upd.kill_rob.col);
    if (`CORE.flush)
      $display("[%0d] FLUSH -> %h", cyc, `CORE.cm_redirect_pc);
    if (`CORE.ic_req && `CORE.ic_ready) $display("[%0d] IFETCH %h", cyc, `CORE.ic_addr);
    for (int q = 0; q < N_IQ; q++)
      for (int p = 0; p < 3; p++)
        if (`CORE.iss_v[q][p])
          $display("[%0d] ISSUE iq%0d.p%0d rob=%0d.%0d fu=%0d op=%0d pdst=p%0d ps1=p%0d(b%0d) ps2=p%0d(b%0d)", cyc, q, p,
                   `CORE.iss_u[q][p].rob_idx.row, `CORE.iss_u[q][p].rob_idx.col, `CORE.iss_u[q][p].fu,
                   `CORE.iss_u[q][p].op, `CORE.iss_u[q][p].pdst, `CORE.iss_u[q][p].psrc1,
                   `CORE.iss_u[q][p].src1_busy, `CORE.iss_u[q][p].psrc2, `CORE.iss_u[q][p].src2_busy);
    for (int w = 0; w < N_IWB; w++)
      if (`CORE.iwb[w].valid && `CORE.iwb[w].wen)
        $display("[%0d] WB%0d p%0d = %h", cyc, w, `CORE.iwb[w].pdst, `CORE.iwb[w].data);
    for (int k = 0; k < N_IWAKE; k++)
      if (`CORE.iwake[k].valid) $display("[%0d] WAKE%0d p%0d", cyc, k, `CORE.iwake[k].pdst);
`ifndef FE_M2
    if (`CORE.`FE.nreq_v && (`CORE.`FE.nreq_flt || `CORE.`FE.nreq_miss || `CORE.`FE.imem_req_o))
      $display("[%0d] FREQ va=%h pa=%h flt=%0d cause=%0d miss=%0d xlate=%0d", cyc, `CORE.`FE.nreq_pc,
               `CORE.`FE.nreq_pa, `CORE.`FE.nreq_flt, `CORE.`FE.nreq_cause, `CORE.`FE.nreq_miss, `CORE.`FE.xlate_i);
`endif
`ifdef FE_M2
    if (`CORE.`FE.f0_sent)
      $display("[%0d] F0 pc=%h ubhit=%0d npc=%h", cyc, `CORE.`FE.pc0, `CORE.`FE.ub_hit, `CORE.`FE.f0_npc);
    if (`CORE.`FE.f3_fire)
      $display("[%0d] F3 pc=%h n=%0d taken=%0d gnpc=%h f3npc=%h mis=%0d push=%0d pop=%0d ras_top=%h cnt=%0d", cyc,
               `CORE.`FE.f3_pc, `CORE.`FE.g_n, `CORE.`FE.g_taken, `CORE.`FE.g_npc, `CORE.`FE.f3_npc,
               `CORE.`FE.mispred3, `CORE.`FE.g_push, `CORE.`FE.g_pop, `CORE.`FE.ras_top, `CORE.`FE.ras_cnt);
`endif
  end

`ifdef TB_SOC
  // JTAG 调试自检 (+dbgtest)
  `include "tb_jtag.svh"
`endif

`ifdef O6_UVM
  `include "o6_uvm_bind.svh"
`endif

`ifdef DUMP_FSDB
  initial begin
    $fsdbDumpfile("wave.fsdb");
    $fsdbDumpvars(0, tb_top, "+all");
  end
`endif
endmodule
