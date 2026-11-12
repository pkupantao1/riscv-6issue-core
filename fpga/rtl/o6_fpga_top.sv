//==============================================================================
// o6_fpga_top.sv - OOO6 SoC 的 VU7P 原型顶层
//
//   板载差分时钟 -> MMCM -> 核时钟 (默认 300 MHz 输入 -> 75 MHz)
//   主存：片上 URAM (o6_axi_ram，16 MB，映射在 0x8000_0000，按容量回绕)
//   调试/装载：JTAG-to-AXI Master IP (jtag_axi_0，AXI4 32 位) -> o6_jtag_bridge
//     - 写 DRAM 装载镜像，写 CTRL 寄存器放核 (上电保持复位)
//   串口：UART 16550 直出引脚
//   RISC-V 调试：DTM 的 JTAG 直接引到 4 个普通 IO (接外部 JTAG 适配器，OpenOCD + GDB)
//
// 板卡相关 (引脚、输入时钟频率、复位极性) 只在 XDC 与本文件参数里，等板卡资料确定。
//==============================================================================
`default_nettype none
module o6_fpga_top #(
  parameter real CLKIN_PERIOD = 3.333,   // 板载时钟 300 MHz
  parameter int  MMCM_MULT    = 4,       // VCO = 1200 MHz (800..1600)
  parameter int  MMCM_DIVIN   = 1,
  parameter int  CORE_DIV     = 16,      // 1200 / 16 = 75 MHz (OOC 在 100 MHz 已收敛，布线后留余量)
  parameter int  TICK_DIV     = 75,      // mtime 1 MHz
  parameter bit  RST_ACT_HIGH = 1'b1,    // 板载复位按键极性
  parameter int  RAM_BYTES    = 16 * 1024 * 1024
) (
  input  wire  sys_clk_p,
  input  wire  sys_clk_n,
  input  wire  sys_rst,
  input  wire  uart_rx,
  output logic uart_tx,
  // RISC-V JTAG (外部适配器)
  input  wire  rv_tck,
  input  wire  rv_tms,
  input  wire  rv_tdi,
  output wire  rv_tdo,
  output logic [3:0] led
);
  localparam int AW  = o6_pkg::PA_W;
  localparam int IDW = 4;

  //---------------- 时钟与复位
  logic clk_in, clk_fb, clk_mmcm, clk, locked;
  IBUFDS u_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_in));
  MMCME4_BASE #(
    .CLKIN1_PERIOD   (CLKIN_PERIOD),
    .CLKFBOUT_MULT_F (real'(MMCM_MULT)),
    .DIVCLK_DIVIDE   (MMCM_DIVIN),
    .CLKOUT0_DIVIDE_F(real'(CORE_DIV))
  ) u_mmcm (
    .CLKIN1(clk_in), .CLKFBIN(clk_fb), .CLKFBOUT(clk_fb), .CLKFBOUTB(),
    .CLKOUT0(clk_mmcm), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
    .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
    .LOCKED(locked), .PWRDWN(1'b0), .RST(RST_ACT_HIGH ? sys_rst : !sys_rst)
  );
  BUFG u_bufg (.I(clk_mmcm), .O(clk));

  logic sys_rst_n, core_hold, core_rst_n;
  o6_rst_sync u_rs_sys  (.clk_i(clk), .rst_ni(locked), .test_mode_i(1'b0), .rst_no(sys_rst_n));
  o6_rst_sync u_rs_core (.clk_i(clk), .rst_ni(locked && !core_hold), .test_mode_i(1'b0), .rst_no(core_rst_n));

  //---------------- RISC-V JTAG：TCK 走 BUFG；TDO 不驱动时弱上拉 (这里直接输出 1)
  logic rv_tck_g, rv_tdo_q, rv_tdo_oe;
  BUFG u_bufg_tck (.I(rv_tck), .O(rv_tck_g));
  assign rv_tdo = rv_tdo_oe ? rv_tdo_q : 1'b1;

  //---------------- AXI 连线：0 = 核，1 = JTAG 桥
  logic [IDW-1:0] awid [2];   logic [AW-1:0] awaddr [2]; logic [7:0] awlen [2];
  logic [2:0]     awsize [2]; logic [1:0]    awburst [2]; logic awvalid [2], awready [2];
  logic [127:0]   wdata [2];  logic [15:0]   wstrb [2];   logic wlast [2], wvalid [2], wready [2];
  logic [IDW-1:0] bid [2];    logic [1:0]    bresp [2];   logic bvalid [2], bready [2];
  logic [IDW-1:0] arid [2];   logic [AW-1:0] araddr [2]; logic [7:0] arlen [2];
  logic [2:0]     arsize [2]; logic [1:0]    arburst [2]; logic arvalid [2], arready [2];
  logic [IDW-1:0] rid [2];    logic [127:0]  rdata [2];   logic [1:0] rresp [2];
  logic           rlast [2], rvalid [2], rready [2];

  logic [IDW-1:0] d_awid, d_bid, d_arid, d_rid;
  logic [AW-1:0]  d_awaddr, d_araddr;
  logic [7:0]     d_awlen, d_arlen;
  logic [2:0]     d_awsize, d_arsize;
  logic [1:0]     d_awburst, d_arburst, d_bresp, d_rresp;
  logic [127:0]   d_wdata, d_rdata;
  logic [15:0]    d_wstrb;
  logic d_awvalid, d_awready, d_wlast, d_wvalid, d_wready, d_bvalid, d_bready;
  logic d_arvalid, d_arready, d_rlast, d_rvalid, d_rready;

  //---------------- SoC
  o6_soc #(.TICK_DIV(TICK_DIV), .N_EXT_IRQ(8), .HAS_FPU(1'b1), .HAS_MMU(1'b1), .HAS_L2(1'b1), .L2_RD_LAT(1)) u_soc (
    .clk_i(clk), .rst_ni(core_rst_n), .test_mode_i(1'b0),
    .m_awid(awid[0]), .m_awaddr(awaddr[0]), .m_awlen(awlen[0]), .m_awsize(awsize[0]), .m_awburst(awburst[0]),
    .m_awvalid(awvalid[0]), .m_awready(awready[0]),
    .m_wdata(wdata[0]), .m_wstrb(wstrb[0]), .m_wlast(wlast[0]), .m_wvalid(wvalid[0]), .m_wready(wready[0]),
    .m_bid(bid[0]), .m_bresp(bresp[0]), .m_bvalid(bvalid[0]), .m_bready(bready[0]),
    .m_arid(arid[0]), .m_araddr(araddr[0]), .m_arlen(arlen[0]), .m_arsize(arsize[0]), .m_arburst(arburst[0]),
    .m_arvalid(arvalid[0]), .m_arready(arready[0]),
    .m_rid(rid[0]), .m_rdata(rdata[0]), .m_rresp(rresp[0]), .m_rlast(rlast[0]), .m_rvalid(rvalid[0]), .m_rready(rready[0]),
    .uart_rx_i(uart_rx), .uart_tx_o(uart_tx), .ext_irq_i(8'b0),
      .jtag_tck_i(rv_tck_g), .jtag_tms_i(rv_tms), .jtag_tdi_i(rv_tdi), .jtag_tdo_o(rv_tdo_q), .jtag_tdo_oe_o(rv_tdo_oe),
    .jtag_trst_ni(locked)
  );

  //---------------- JTAG-to-AXI (IP，由 build.tcl 生成：AXI4，32 位数据，32 位地址)
  logic [31:0] j_awaddr, j_wdata, j_araddr, j_rdata;
  logic [7:0]  j_awlen, j_arlen;
  logic [3:0]  j_wstrb;
  logic [1:0]  j_bresp, j_rresp;
  logic j_awvalid, j_awready, j_wlast, j_wvalid, j_wready, j_bvalid, j_bready;
  logic j_arvalid, j_arready, j_rlast, j_rvalid, j_rready;
  jtag_axi_0 u_jtag (
    .aclk(clk), .aresetn(sys_rst_n),
    .m_axi_awid(), .m_axi_awaddr(j_awaddr), .m_axi_awlen(j_awlen), .m_axi_awsize(), .m_axi_awburst(),
    .m_axi_awlock(), .m_axi_awcache(), .m_axi_awprot(), .m_axi_awqos(),
    .m_axi_awvalid(j_awvalid), .m_axi_awready(j_awready),
    .m_axi_wdata(j_wdata), .m_axi_wstrb(j_wstrb), .m_axi_wlast(j_wlast), .m_axi_wvalid(j_wvalid), .m_axi_wready(j_wready),
    .m_axi_bid(1'b0), .m_axi_bresp(j_bresp), .m_axi_bvalid(j_bvalid), .m_axi_bready(j_bready),
    .m_axi_arid(), .m_axi_araddr(j_araddr), .m_axi_arlen(j_arlen), .m_axi_arsize(), .m_axi_arburst(),
    .m_axi_arlock(), .m_axi_arcache(), .m_axi_arprot(), .m_axi_arqos(),
    .m_axi_arvalid(j_arvalid), .m_axi_arready(j_arready),
    .m_axi_rid(1'b0), .m_axi_rdata(j_rdata), .m_axi_rresp(j_rresp), .m_axi_rlast(j_rlast), .m_axi_rvalid(j_rvalid), .m_axi_rready(j_rready)
  );

  o6_jtag_bridge #(.AW(AW), .IDW(IDW)) u_bridge (
    .clk_i(clk), .rst_ni(sys_rst_n), .locked_i(locked), .core_hold_o(core_hold),
    .s_awaddr(j_awaddr), .s_awlen(j_awlen), .s_awvalid(j_awvalid), .s_awready(j_awready),
    .s_wdata(j_wdata), .s_wstrb(j_wstrb), .s_wlast(j_wlast), .s_wvalid(j_wvalid), .s_wready(j_wready),
    .s_bresp(j_bresp), .s_bvalid(j_bvalid), .s_bready(j_bready),
    .s_araddr(j_araddr), .s_arlen(j_arlen), .s_arvalid(j_arvalid), .s_arready(j_arready),
    .s_rdata(j_rdata), .s_rresp(j_rresp), .s_rlast(j_rlast), .s_rvalid(j_rvalid), .s_rready(j_rready),
    .m_awid(awid[1]), .m_awaddr(awaddr[1]), .m_awlen(awlen[1]), .m_awsize(awsize[1]), .m_awburst(awburst[1]),
    .m_awvalid(awvalid[1]), .m_awready(awready[1]),
    .m_wdata(wdata[1]), .m_wstrb(wstrb[1]), .m_wlast(wlast[1]), .m_wvalid(wvalid[1]), .m_wready(wready[1]),
    .m_bid(bid[1]), .m_bresp(bresp[1]), .m_bvalid(bvalid[1]), .m_bready(bready[1]),
    .m_arid(arid[1]), .m_araddr(araddr[1]), .m_arlen(arlen[1]), .m_arsize(arsize[1]), .m_arburst(arburst[1]),
    .m_arvalid(arvalid[1]), .m_arready(arready[1]),
    .m_rid(rid[1]), .m_rdata(rdata[1]), .m_rresp(rresp[1]), .m_rlast(rlast[1]), .m_rvalid(rvalid[1]), .m_rready(rready[1])
  );

  //---------------- 仲裁 + URAM 主存
  o6_axi_arb2 #(.AW(AW), .IDW(IDW)) u_arb (
    .clk_i(clk), .rst_ni(sys_rst_n),
    .s_awid(awid), .s_awaddr(awaddr), .s_awlen(awlen), .s_awsize(awsize), .s_awburst(awburst),
    .s_awvalid(awvalid), .s_awready(awready),
    .s_wdata(wdata), .s_wstrb(wstrb), .s_wlast(wlast), .s_wvalid(wvalid), .s_wready(wready),
    .s_bid(bid), .s_bresp(bresp), .s_bvalid(bvalid), .s_bready(bready),
    .s_arid(arid), .s_araddr(araddr), .s_arlen(arlen), .s_arsize(arsize), .s_arburst(arburst),
    .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast), .s_rvalid(rvalid), .s_rready(rready),
    .m_awid(d_awid), .m_awaddr(d_awaddr), .m_awlen(d_awlen), .m_awsize(d_awsize), .m_awburst(d_awburst),
    .m_awvalid(d_awvalid), .m_awready(d_awready),
    .m_wdata(d_wdata), .m_wstrb(d_wstrb), .m_wlast(d_wlast), .m_wvalid(d_wvalid), .m_wready(d_wready),
    .m_bid(d_bid), .m_bresp(d_bresp), .m_bvalid(d_bvalid), .m_bready(d_bready),
    .m_arid(d_arid), .m_araddr(d_araddr), .m_arlen(d_arlen), .m_arsize(d_arsize), .m_arburst(d_arburst),
    .m_arvalid(d_arvalid), .m_arready(d_arready),
    .m_rid(d_rid), .m_rdata(d_rdata), .m_rresp(d_rresp), .m_rlast(d_rlast), .m_rvalid(d_rvalid), .m_rready(d_rready)
  );

  o6_axi_ram #(.AW(AW), .IDW(IDW), .BYTES(RAM_BYTES), .RD_PIPE(2)) u_ram (
    .clk_i(clk), .rst_ni(sys_rst_n),
    .s_awid(d_awid), .s_awaddr(d_awaddr), .s_awlen(d_awlen), .s_awsize(d_awsize), .s_awburst(d_awburst),
    .s_awvalid(d_awvalid), .s_awready(d_awready),
    .s_wdata(d_wdata), .s_wstrb(d_wstrb), .s_wlast(d_wlast), .s_wvalid(d_wvalid), .s_wready(d_wready),
    .s_bid(d_bid), .s_bresp(d_bresp), .s_bvalid(d_bvalid), .s_bready(d_bready),
    .s_arid(d_arid), .s_araddr(d_araddr), .s_arlen(d_arlen), .s_arsize(d_arsize), .s_arburst(d_arburst),
    .s_arvalid(d_arvalid), .s_arready(d_arready),
    .s_rid(d_rid), .s_rdata(d_rdata), .s_rresp(d_rresp), .s_rlast(d_rlast), .s_rvalid(d_rvalid), .s_rready(d_rready)
  );

  //---------------- LED：心跳 / MMCM 锁定 / 核运行 / 串口活动
  logic [25:0] hb;
  logic        tx_seen;
  always_ff @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin hb <= '0; tx_seen <= 1'b0; end
    else begin
      hb <= hb + 1'b1;
      if (!uart_tx) tx_seen <= 1'b1;
      else if (&hb[22:0]) tx_seen <= 1'b0;
    end
  end
  assign led = {tx_seen, core_rst_n, locked, hb[25]};
endmodule
