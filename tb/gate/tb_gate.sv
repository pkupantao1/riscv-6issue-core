//==============================================================================
// tb_gate.sv - 门级 (DC 网表) 冒烟 TB (docs/13 §0「实现一致性」/ §10 第 5、6 步)
//
// 网表里没有 RTL 的结构体 / 数组 (ROB 表项、PRF 内容)，tb_top 的提交监视与对拍用不上：这里只做硅片能做的判定
//   —— 程序自检写 tohost，TB 在 D$ store 口 (u_core.u_dc 的端口，-no_autoungroup 保留了层次与端口名) 看到写入值：
//   1 = 通过，其它 = 失败；+max_cycles 内没写 = 超时
// 存储：经对拍库 (libcosim) 的 tb_mem_* (cosim_init 载入 ELF；Spike 不用)；AXI 从机同 tb_top (读可多个在途、固定延迟)
// 编译：+define+GATE 例化网表 (syn/out_o6_soc_<run>/o6_soc.v + syn/gate/cells.v + sram_models.v)；
//       不加则例化 RTL (O6_ASIC：同样的 SRAM 宏模型与门控单元)，作为同一 TB 的参照
//   +elf=  +max_cycles=N (默认 2e6)  +axilat=N
//==============================================================================
`timescale 1ns/1ps
module tb_gate;
  import "DPI-C" function int      cosim_init(input string elf, input string isa, input longint dram_size,
                                              input int verbose, input string trace);
  import "DPI-C" function longint  tb_mem_read64(input longint addr);
  import "DPI-C" function void     tb_mem_write64(input longint addr, input longint data, input int mask);
  import "DPI-C" function longint  tb_tohost_addr();
  localparam int PA_W = 40;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;
  longint cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  logic [3:0]  awid, arid, rid, bid;
  logic [PA_W-1:0] awaddr, araddr;
  logic [7:0]  awlen, arlen;
  logic [2:0]  awsize, arsize;
  logic [1:0]  awburst, arburst, bresp, rresp;
  logic        awvalid, awready, wlast, wvalid, wready, bvalid, bready, arvalid, arready, rlast, rvalid, rready;
  logic [127:0] wdata, rdata;
  logic [15:0] wstrb;
  logic        uart_tx, jtag_tdo, jtag_tdo_oe;

`ifdef GATE
  o6_soc dut (
`else
  o6_soc #(.TICK_DIV(10), .HAS_FPU(1'b1), .HAS_MMU(1'b1), .L2_RD_LAT(2)) dut (
`endif
    .clk_i(clk), .rst_ni(rst_n), .test_mode_i(1'b0),
    .m_awid(awid), .m_awaddr(awaddr), .m_awlen(awlen), .m_awsize(awsize), .m_awburst(awburst),
    .m_awvalid(awvalid), .m_awready(awready),
    .m_wdata(wdata), .m_wstrb(wstrb), .m_wlast(wlast), .m_wvalid(wvalid), .m_wready(wready),
    .m_bid(bid), .m_bresp(bresp), .m_bvalid(bvalid), .m_bready(bready),
    .m_arid(arid), .m_araddr(araddr), .m_arlen(arlen), .m_arsize(arsize), .m_arburst(arburst),
    .m_arvalid(arvalid), .m_arready(arready),
    .m_rid(rid), .m_rdata(rdata), .m_rresp(rresp), .m_rlast(rlast), .m_rvalid(rvalid), .m_rready(rready),
    .uart_rx_i(1'b1), .uart_tx_o(uart_tx), .ext_irq_i(8'b0),
    .jtag_tck_i(1'b0), .jtag_tms_i(1'b1), .jtag_tdi_i(1'b0), .jtag_tdo_o(jtag_tdo),
    .jtag_tdo_oe_o(jtag_tdo_oe), .jtag_trst_ni(1'b0));

  //--------------------------------------------------------------------------
  // AXI 从机 (同 tb_top：AR 排队、axi_lat 拍后按序回 4 拍突发；写一次一个)
  //--------------------------------------------------------------------------
  int axi_lat = 20;
  initial void'($value$plusargs("axilat=%d", axi_lat));
  typedef struct { longint addr; logic [3:0] id; longint due; } ar_t;
  ar_t             arq [$];
  logic [PA_W-1:0] r_addr, w_addr;
  logic [3:0]      r_id;
  logic [7:0]      r_left;
  logic            r_busy, w_busy, w_data_done;
  assign arready = 1'b1;
  assign awready = !w_busy;
  assign wready  = w_busy && !w_data_done;
  assign bid = '0;  assign bresp = 2'b00;
  assign rid = r_id; assign rresp = 2'b00;
  assign rvalid = r_busy;
  assign rlast  = (r_left == 8'd0);
  always @* for (int i = 0; i < 2; i++) rdata[64*i +: 64] = tb_mem_read64({{(64-PA_W){1'b0}}, r_addr} + 8*i);
  always @(posedge clk) begin
    if (!rst_n) begin
      r_busy <= 1'b0; w_busy <= 1'b0; w_data_done <= 1'b0; bvalid <= 1'b0; arq.delete();
    end else begin
      if (rvalid && rready) begin
        r_addr <= r_addr + 16;
        if (rlast) r_busy <= 1'b0; else r_left <= r_left - 1'b1;
      end
      if ((!r_busy || (rvalid && rready && rlast)) && arq.size() > 0 && arq[0].due <= cyc) begin
        r_busy <= 1'b1; r_addr <= arq[0].addr[PA_W-1:0]; r_id <= arq[0].id; r_left <= 8'd3;
        void'(arq.pop_front());
      end
      if (arvalid && arready) arq.push_back('{addr: {{(64-PA_W){1'b0}}, araddr}, id: arid, due: cyc + axi_lat});
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

  //--------------------------------------------------------------------------
  // tohost：看 D$ store 口 (网表里保留的模块端口)
  //--------------------------------------------------------------------------
  longint tohost_a, max_cycles;
  string  elf;
  wire st_fire = dut.u_core.u_dc.st_req_i && dut.u_core.u_dc.st_done_o && !dut.u_core.u_dc.st_amo_i;
  wire [PA_W-1:0] st_pa = dut.u_core.u_dc.st_paddr_i;
  wire [63:0]     st_d  = dut.u_core.u_dc.st_data_i;
  always @(posedge clk) if (rst_n) begin
    if (st_fire && {st_pa[PA_W-1:3], 3'b000} == tohost_a[PA_W-1:0] && st_d != 0) begin
      if (st_d == 1) $display("\n=== GATE PASS === tohost=1  cycles=%0d", cyc);
      else           $display("\n=== GATE FAIL === tohost=0x%0h (测试 %0d)  cycles=%0d", st_d, st_d >> 1, cyc);
      $finish;
    end
    if (cyc > max_cycles) begin $display("\n=== GATE FAIL === 超时  cycles=%0d", cyc); $finish; end
    if (^st_fire === 1'bx && cyc > 1000) begin
      $display("\n=== GATE FAIL === D$ store 口出现 X (门级 X 传播)  cycles=%0d", cyc); $finish;
    end
  end

  initial begin
    if (!$value$plusargs("elf=%s", elf)) begin $display("需要 +elf="); $finish; end
    if (!$value$plusargs("max_cycles=%d", max_cycles)) max_cycles = 2000000;
    if (cosim_init(elf, "RV64IMAFDC_Zicsr_Zifencei", 64'h8000_0000, 0, "") != 0) begin $display("载入 ELF 失败"); $finish; end
    tohost_a = tb_tohost_addr();
    repeat (20) @(posedge clk);
    rst_n = 1'b1;
  end
endmodule
