//==============================================================================
// tb_scan.sv - 扫描链移位仿真 (docs/13 §16 P3「scan 移位仿真」)：插扫描后的网表 out_o6_soc_<run>/o6_soc_scan.v
//   test_mode = 1、scan_en = 1，每条链从 scan_in 灌入随机位流，记录 scan_out；对每条链找延迟 d 使 out[t] == in[t - d]
//   在整段 (去掉前 d 拍) 都成立 -> 链通，d = 链长；链断 / 串错 / 时钟门控没被测试使能打开都会对不上
//   链长与 DFT 报告 (scan_chains.rpt) 比对由 run 脚本做
//   +nchain=N (默认 32)  +maxlen=L (链长上限，默认 20000)
//==============================================================================
`timescale 1ns/1ps
module tb_scan;
  localparam int NC = 32;
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst_n = 1'b0, scan_en = 1'b0;
  logic [NC-1:0] si = '0;
  wire  [NC-1:0] so;

  o6_soc dut (
    .clk_i(clk), .rst_ni(rst_n), .test_mode_i(1'b1),
    .m_awid(), .m_awaddr(), .m_awlen(), .m_awsize(), .m_awburst(), .m_awvalid(), .m_awready(1'b0),
    .m_wdata(), .m_wstrb(), .m_wlast(), .m_wvalid(), .m_wready(1'b0),
    .m_bid(4'b0), .m_bresp(2'b0), .m_bvalid(1'b0), .m_bready(),
    .m_arid(), .m_araddr(), .m_arlen(), .m_arsize(), .m_arburst(), .m_arvalid(), .m_arready(1'b0),
    .m_rid(4'b0), .m_rdata(128'b0), .m_rresp(2'b0), .m_rlast(1'b0), .m_rvalid(1'b0), .m_rready(),
    .uart_rx_i(1'b1), .uart_tx_o(), .ext_irq_i(8'b0),
    .jtag_tck_i(1'b0), .jtag_tms_i(1'b1), .jtag_tdi_i(1'b0), .jtag_tdo_o(), .jtag_tdo_oe_o(), .jtag_trst_ni(1'b0),
    .scan_en_i(scan_en), .scan_in_i(si), .scan_out_o(so));

  int maxlen, nshift;
  bit inb [NC][$];
  bit outb [NC][$];
  initial begin
    if (!$value$plusargs("maxlen=%d", maxlen)) maxlen = 20000;
    nshift = 2 * maxlen + 200;
    // 复位释放 (复位由引脚直控：test_mode 下同步器旁路)，然后整段移位
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    scan_en = 1'b1;
    for (int t = 0; t < nshift; t++) begin
      @(negedge clk);
      for (int c = 0; c < NC; c++) begin
        bit b;
        b = $urandom_range(1);
        si[c] = b; inb[c].push_back(b);
        outb[c].push_back(so[c]);         // negedge 采：上一拍沿后的稳定值
      end
    end
    begin
      int bad;
      bad = 0;
      for (int c = 0; c < NC; c++) begin
        int found;
        found = -1;
        for (int d = 1; d <= maxlen + 2 && found < 0; d++) begin
          bit ok;
          ok = 1;
          for (int t = d; t < nshift && ok; t++) if (outb[c][t] !== inb[c][t - d]) ok = 0;
          if (ok) found = d;
        end
        if (found < 0) begin bad++; $display("链 %0d：找不到使输出复现输入的延迟 (断链 / X)", c); end
        else $display("CHAIN %0d LEN %0d", c, found);   // negedge 灌、posedge 移、negedge 采：延迟 d 拍 = d 个触发器
      end
      if (bad == 0) $display("=== SCAN PASS === %0d 条链移位通过", NC);
      else          $display("=== SCAN FAIL === %0d 条链不通", bad);
    end
    $finish;
  end
endmodule
