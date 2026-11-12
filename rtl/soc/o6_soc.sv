//==============================================================================
// o6_soc.sv - OOO6 SoC：核 + 行请求分流 (BootROM / DRAM-AXI) + MMIO 外设
//
// 地址映射 (与 Spike / QEMU virt 一致)：
//   0x0000_0000  调试模块存储区 (4 KB，只在调试模式可访问；取指不进 I$)
//   0x0000_1000  BootROM (64 KB 窗口)      行请求 (取指) 与 MMIO 读都可访问
//   0x0200_0000  CLINT
//   0x0C00_0000  PLIC  (源 1 = UART，源 2.. = 外部中断)
//   0x1000_0000  UART (16550)
//   0x8000_0000  DRAM  (AXI4 主口，128 位)
// L2 (512 KB) 接入后替换 o6_line2axi。
//==============================================================================
`default_nettype none
module o6_soc
  import o6_pkg::*;
#(
  parameter int  TICK_DIV = 10,       // mtime 分频
  parameter int  N_EXT_IRQ = 8,
  parameter bit  HAS_FPU = 1'b0,
  parameter bit  HAS_MMU = 1'b1,
  parameter bit  HAS_L2  = 1'b1,
  parameter int  L2_RD_LAT = 1,       // FPGA 1；ASIC 40LP 用 2 (多周期)
  parameter bit  FE_M2   = 1'b1
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,          // 外部异步复位 (片内同步释放)
  input  wire                 test_mode_i,     // 扫描测试：复位同步器旁路，复位由引脚直控 (功能模式接 0)

  // AXI4 主口 (DRAM)
  output logic [3:0]          m_awid,
  output logic [PA_W-1:0]     m_awaddr,
  output logic [7:0]          m_awlen,
  output logic [2:0]          m_awsize,
  output logic [1:0]          m_awburst,
  output logic                m_awvalid,
  input  wire                 m_awready,
  output logic [127:0]        m_wdata,
  output logic [15:0]         m_wstrb,
  output logic                m_wlast,
  output logic                m_wvalid,
  input  wire                 m_wready,
  input  wire  [3:0]          m_bid,
  input  wire  [1:0]          m_bresp,
  input  wire                 m_bvalid,
  output logic                m_bready,
  output logic [3:0]          m_arid,
  output logic [PA_W-1:0]     m_araddr,
  output logic [7:0]          m_arlen,
  output logic [2:0]          m_arsize,
  output logic [1:0]          m_arburst,
  output logic                m_arvalid,
  input  wire                 m_arready,
  input  wire  [3:0]          m_rid,
  input  wire  [127:0]        m_rdata,
  input  wire  [1:0]          m_rresp,
  input  wire                 m_rlast,
  input  wire                 m_rvalid,
  output logic                m_rready,

  // 外设
  input  wire                 uart_rx_i,
  output logic                uart_tx_o,
  input  wire  [N_EXT_IRQ-1:0] ext_irq_i,

  // JTAG (RISC-V 调试)
  input  wire                 jtag_tck_i,
  input  wire                 jtag_tms_i,
  input  wire                 jtag_tdi_i,
  output logic                jtag_tdo_o,
  output logic                jtag_tdo_oe_o,
  input  wire                 jtag_trst_ni
);
  //==========================================================================
  // 复位：异步置位、同步释放 (test_mode 下旁路)
  //==========================================================================
  logic rst_por_n, rst_n, ndmreset;
  o6_rst_sync u_rst_sync (.clk_i, .rst_ni, .test_mode_i, .rst_no(rst_por_n));     // 调试模块用
  o6_rst_sync u_rst_sys  (.clk_i, .rst_ni(rst_por_n && !ndmreset), .test_mode_i, .rst_no(rst_n));  // 其余

  //==========================================================================
  // 调试：JTAG DTM + DM
  //==========================================================================
  logic        dmi_req, dmi_resp;
  logic [6:0]  dmi_addr;
  logic [31:0] dmi_wdata, dmi_rdata;
  logic [1:0]  dmi_op;
  logic        dbg_haltreq;
  logic        dm_req, dm_ack;
  logic [63:0] dm_rdata, dm_rd_q;
  logic [511:0] dm_line;
  // MBIST：核 (I$/D$) + L2
  logic        mbist_start, mbist_done, mbist_fail, core_bd, core_bf, l2_bd, l2_bf;
  assign mbist_done = core_bd && l2_bd;
  assign mbist_fail = core_bf || l2_bf;

  o6_dtm_jtag u_dtm (
    .tck_i(jtag_tck_i), .tms_i(jtag_tms_i), .tdi_i(jtag_tdi_i), .tdo_o(jtag_tdo_o), .tdo_oe_o(jtag_tdo_oe_o),
    .trst_ni(jtag_trst_ni),
    .clk_i, .rst_ni(rst_por_n),
    .dmi_req_o(dmi_req), .dmi_addr_o(dmi_addr), .dmi_wdata_o(dmi_wdata), .dmi_op_o(dmi_op),
    .dmi_resp_i(dmi_resp), .dmi_rdata_i(dmi_rdata),
    .mbist_start_o(mbist_start), .mbist_done_i(mbist_done), .mbist_fail_i(mbist_fail)
  );

  //==========================================================================
  // 核
  //==========================================================================
  logic            mem_req_valid, mem_req_ready, mem_req_we, mem_resp_valid, mem_resp_ready;
  logic [PA_W-1:6] mem_req_addr;
  logic [511:0]    mem_req_data, mem_resp_data;
  logic [3:0]      mem_req_id, mem_resp_id;
  logic            mmio_req, mmio_we, mmio_ack;
  logic [PA_W-1:0] mmio_addr;
  logic [1:0]      mmio_size;
  logic [63:0]     mmio_wdata, mmio_rdata;
  logic [7:0]      mmio_wmask;
  logic            mtip, msip;
  logic [1:0]      eip;
  logic [63:0]     mtime;

  o6_core #(.HAS_FPU(HAS_FPU), .HAS_MMU(HAS_MMU), .FE_M2(FE_M2)) u_core (
    .clk_i, .rst_ni(rst_n),
    .mem_req_valid_o(mem_req_valid), .mem_req_ready_i(mem_req_ready), .mem_req_we_o(mem_req_we),
    .mem_req_addr_o(mem_req_addr), .mem_req_data_o(mem_req_data), .mem_req_id_o(mem_req_id),
    .mem_resp_valid_i(mem_resp_valid), .mem_resp_ready_o(mem_resp_ready),
    .mem_resp_id_i(mem_resp_id), .mem_resp_data_i(mem_resp_data),
    .mmio_req_o(mmio_req), .mmio_we_o(mmio_we), .mmio_addr_o(mmio_addr), .mmio_size_o(mmio_size),
    .mmio_wdata_o(mmio_wdata), .mmio_wmask_o(mmio_wmask), .mmio_ack_i(mmio_ack), .mmio_rdata_i(mmio_rdata),
    .mtip_i(mtip), .msip_i(msip), .meip_i(eip[0]), .seip_i(eip[1]), .mtime_i(mtime),
    .dbg_haltreq_i(dbg_haltreq),
    .bist_start_i(mbist_start), .bist_done_o(core_bd), .bist_fail_o(core_bf)
  );

  //==========================================================================
  // 行请求分流：BootROM 区由 ROM 直接回 64 B，其余走 AXI
  //==========================================================================
  wire to_rom = ({mem_req_addr, 6'b0} < 40'h0001_1000);   // 调试区 + BootROM 窗口 (只读，写回不会落在这里)
  wire to_dm  = (mem_req_addr[PA_W-1:12] == '0);

  logic            ax_req_valid, ax_req_ready, ax_resp_valid, ax_resp_ready;
  logic [3:0]      ax_resp_id;
  logic [511:0]    ax_resp_data;

  // ROM 行响应 (1 项缓冲)
  logic            rl_v;
  logic [3:0]      rl_id;
  logic [511:0]    rl_data;
  logic [63:0]     rom_w [8];
  for (genvar i = 0; i < 8; i++) begin : g_romw
    o6_bootrom_comb u_rw (.addr_i({mem_req_addr[15:6], 3'(i), 3'b000}), .data_o(rom_w[i]));
  end

  assign ax_req_valid  = mem_req_valid && !to_rom;
  assign mem_req_ready = to_rom ? !rl_v : ax_req_ready;

  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) rl_v <= 1'b0;
    else begin
      if (mem_req_valid && to_rom && !rl_v) rl_v <= 1'b1;
      else if (rl_v && mem_resp_ready && !ax_resp_valid) rl_v <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    if (mem_req_valid && to_rom && !rl_v) begin
      rl_id <= mem_req_id;
      for (int i = 0; i < 8; i++) rl_data[64*i +: 64] <= to_dm ? dm_line[64*i +: 64] : rom_w[i];
    end
  end

  // 响应合并：AXI 优先
  assign mem_resp_valid = ax_resp_valid || rl_v;
  assign mem_resp_id    = ax_resp_valid ? ax_resp_id   : rl_id;
  assign mem_resp_data  = ax_resp_valid ? ax_resp_data : rl_data;
  assign ax_resp_ready  = mem_resp_ready;

  // L2 (可选)
  logic            b_req_valid, b_req_ready, b_req_we, b_resp_valid, b_resp_ready;
  logic [PA_W-1:6] b_req_addr;
  logic [511:0]    b_req_data, b_resp_data;
  logic [3:0]      b_req_id, b_resp_id;

  if (HAS_L2) begin : g_l2
    o6_l2 #(.RD_LAT(L2_RD_LAT)) u_l2 (
      .clk_i, .rst_ni(rst_n),
      .up_req_valid_i(ax_req_valid), .up_req_ready_o(ax_req_ready), .up_req_we_i(mem_req_we),
      .up_req_addr_i(mem_req_addr), .up_req_data_i(mem_req_data), .up_req_id_i(mem_req_id),
      .up_resp_valid_o(ax_resp_valid), .up_resp_ready_i(ax_resp_ready),
      .up_resp_id_o(ax_resp_id), .up_resp_data_o(ax_resp_data),
      .dn_req_valid_o(b_req_valid), .dn_req_ready_i(b_req_ready), .dn_req_we_o(b_req_we),
      .dn_req_addr_o(b_req_addr), .dn_req_data_o(b_req_data), .dn_req_id_o(b_req_id),
      .dn_resp_valid_i(b_resp_valid), .dn_resp_ready_o(b_resp_ready),
      .dn_resp_id_i(b_resp_id), .dn_resp_data_i(b_resp_data),
      .bist_start_i(mbist_start), .bist_done_o(l2_bd), .bist_fail_o(l2_bf)
    );
  end else begin : g_nol2
    assign l2_bd = 1'b1; assign l2_bf = 1'b0;
    assign b_req_valid = ax_req_valid;  assign ax_req_ready = b_req_ready;
    assign b_req_we = mem_req_we;       assign b_req_addr = mem_req_addr;
    assign b_req_data = mem_req_data;   assign b_req_id = mem_req_id;
    assign ax_resp_valid = b_resp_valid; assign b_resp_ready = ax_resp_ready;
    assign ax_resp_id = b_resp_id;      assign ax_resp_data = b_resp_data;
  end

  o6_line2axi u_l2a (
    .clk_i, .rst_ni(rst_n),
    .req_valid_i(b_req_valid), .req_ready_o(b_req_ready), .req_we_i(b_req_we),
    .req_addr_i(b_req_addr), .req_data_i(b_req_data), .req_id_i(b_req_id),
    .resp_valid_o(b_resp_valid), .resp_ready_i(b_resp_ready), .resp_id_o(b_resp_id), .resp_data_o(b_resp_data),
    .m_awid, .m_awaddr, .m_awlen, .m_awsize, .m_awburst, .m_awvalid, .m_awready,
    .m_wdata, .m_wstrb, .m_wlast, .m_wvalid, .m_wready,
    .m_bid, .m_bresp, .m_bvalid, .m_bready,
    .m_arid, .m_araddr, .m_arlen, .m_arsize, .m_arburst, .m_arvalid, .m_arready,
    .m_rid, .m_rdata, .m_rresp, .m_rlast, .m_rvalid, .m_rready
  );

  //==========================================================================
  // MMIO 译码
  //==========================================================================
  wire sel_dm    = (mmio_addr < 40'h0000_1000);
  wire sel_rom   = (mmio_addr >= 40'h0000_1000) && (mmio_addr < 40'h0001_1000);
  wire sel_clint = (mmio_addr[PA_W-1:16] == 24'h000200);
  wire sel_plic  = (mmio_addr[PA_W-1:26] == 14'h0003);       // 0x0C00_0000 - 0x0FFF_FFFF
  wire sel_uart  = (mmio_addr[PA_W-1:12] == 28'h0010000);

  logic rom_ack, clint_ack, plic_ack, uart_ack, uart_irq;
  logic [63:0] rom_rd, clint_rd, plic_rd, uart_rd;

  o6_bootrom u_rom (
    .clk_i, .rst_ni(rst_n), .req_i(mmio_req && sel_rom), .addr_i(mmio_addr[15:0]), .ack_o(rom_ack), .rdata_o(rom_rd)
  );
  o6_clint #(.TICK_DIV(TICK_DIV)) u_clint (
    .clk_i, .rst_ni(rst_n), .req_i(mmio_req && sel_clint), .we_i(mmio_we), .addr_i(mmio_addr[15:0]),
    .wdata_i(mmio_wdata), .wmask_i(mmio_wmask), .ack_o(clint_ack), .rdata_o(clint_rd),
    .mtip_o(mtip), .msip_o(msip), .mtime_o(mtime)
  );
  o6_plic #(.NSRC(31)) u_plic (
    .clk_i, .rst_ni(rst_n), .src_i({{(31-1-N_EXT_IRQ){1'b0}}, ext_irq_i, uart_irq}),
    .req_i(mmio_req && sel_plic), .we_i(mmio_we), .addr_i(mmio_addr[25:0]),
    .wdata_i(mmio_wdata), .wmask_i(mmio_wmask), .ack_o(plic_ack), .rdata_o(plic_rd), .eip_o(eip)
  );
  o6_uart u_uart (
    .clk_i, .rst_ni(rst_n), .req_i(mmio_req && sel_uart), .we_i(mmio_we), .addr_i(mmio_addr[11:0]),
    .wdata_i(mmio_wdata), .wmask_i(mmio_wmask), .ack_o(uart_ack), .rdata_o(uart_rd),
    .irq_o(uart_irq), .rx_i(uart_rx_i), .tx_o(uart_tx_o)
  );

  // 调试模块：请求只在第一拍生效 (写不重复)，下一拍应答
  assign dm_req = mmio_req && sel_dm && !dm_ack;
  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) dm_ack <= 1'b0;
    else        dm_ack <= dm_req;
  end
  always_ff @(posedge clk_i) if (dm_req) dm_rd_q <= dm_rdata;

  o6_dm u_dm (
    .clk_i, .rst_ni(rst_por_n),
    .dmi_req_i(dmi_req), .dmi_addr_i(dmi_addr), .dmi_wdata_i(dmi_wdata), .dmi_op_i(dmi_op),
    .dmi_resp_o(dmi_resp), .dmi_rdata_o(dmi_rdata),
    .haltreq_o(dbg_haltreq), .ndmreset_o(ndmreset), .hart_unavail_i(!rst_n),
    .mem_req_i(dm_req), .mem_we_i(mmio_we), .mem_addr_i(mmio_addr[11:0]), .mem_wdata_i(mmio_wdata),
    .mem_wmask_i(mmio_wmask), .mem_rdata_o(dm_rdata),
    .line_addr_i(mem_req_addr[11:6]), .line_data_o(dm_line)
  );

  // 未命中任何设备的 MMIO (理论上 PMA 已挡住)：直接应答 0
  logic none_ack;
  always_ff @(posedge clk_i or negedge rst_n) begin
    if (!rst_n) none_ack <= 1'b0;
    else none_ack <= mmio_req && !(sel_dm || sel_rom || sel_clint || sel_plic || sel_uart) && !none_ack;
  end

  assign mmio_ack   = dm_ack | rom_ack | clint_ack | plic_ack | uart_ack | none_ack;
  assign mmio_rdata = dm_ack ? dm_rd_q : rom_ack ? rom_rd : clint_ack ? clint_rd : plic_ack ? plic_rd :
                      uart_ack ? uart_rd : 64'd0;

  wire unused = ^mmio_size;
endmodule

//==============================================================================
// BootROM 组合读 (给行请求用；与 o6_bootrom 共用同一份镜像)
//==============================================================================
module o6_bootrom_comb (
  input  wire  [15:0] addr_i,
  output logic [63:0] data_o
);
`include "o6_bootrom_img.svh"
  // ROM 镜像从 0x1000 开始
  wire [15:0] off = addr_i - 16'h1000;
  wire [15:3] wi  = off[15:3];
  assign data_o = (wi < ROM_WORDS) ? ROM[wi] : 64'd0;
endmodule
