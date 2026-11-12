// tb_bridge.sv - JTAG 桥 + 仲裁 + URAM 主存 的定向测试 (xsim / VCS)
//   主口 0：模拟核的 4 拍 128 位突发读写；主口 1：JTAG 桥，上游 32 位突发
`timescale 1ns/1ps
`default_nettype none
module tb_bridge;
  localparam int AW = 40, IDW = 4;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // 上游 32 位
  logic [31:0] j_awaddr = 0, j_wdata = 0, j_araddr = 0, j_rdata;
  logic [7:0]  j_awlen = 0, j_arlen = 0;
  logic [3:0]  j_wstrb = 0;
  logic [1:0]  j_bresp, j_rresp;
  logic j_awvalid = 0, j_awready, j_wlast = 0, j_wvalid = 0, j_wready, j_bvalid, j_bready = 1;
  logic j_arvalid = 0, j_arready, j_rlast, j_rvalid, j_rready = 1;
  logic hold;

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

  o6_jtag_bridge #(.AW(AW), .IDW(IDW)) u_bridge (
    .clk_i(clk), .rst_ni(rst_n), .locked_i(1'b1), .core_hold_o(hold),
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
  o6_axi_arb2 #(.AW(AW), .IDW(IDW)) u_arb (
    .clk_i(clk), .rst_ni(rst_n),
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
  o6_axi_ram #(.AW(AW), .IDW(IDW), .BYTES(64 * 1024), .RD_PIPE(2)) u_ram (
    .clk_i(clk), .rst_ni(rst_n),
    .s_awid(d_awid), .s_awaddr(d_awaddr), .s_awlen(d_awlen), .s_awsize(d_awsize), .s_awburst(d_awburst),
    .s_awvalid(d_awvalid), .s_awready(d_awready),
    .s_wdata(d_wdata), .s_wstrb(d_wstrb), .s_wlast(d_wlast), .s_wvalid(d_wvalid), .s_wready(d_wready),
    .s_bid(d_bid), .s_bresp(d_bresp), .s_bvalid(d_bvalid), .s_bready(d_bready),
    .s_arid(d_arid), .s_araddr(d_araddr), .s_arlen(d_arlen), .s_arsize(d_arsize), .s_arburst(d_arburst),
    .s_arvalid(d_arvalid), .s_arready(d_arready),
    .s_rid(d_rid), .s_rdata(d_rdata), .s_rresp(d_rresp), .s_rlast(d_rlast), .s_rvalid(d_rvalid), .s_rready(d_rready)
  );

  // 参考模型 (字节)
  logic [7:0] ref_mem [bit [15:0]];
  int errors = 0;

  //---------------- 主口 0：核侧 4 拍突发 (64 B 行)
  logic [AW-1:0] c_addr; logic c_we; logic c_go = 0, c_done;
  logic [127:0] c_line [4];
  logic [1:0] c_beat;
  typedef enum logic [2:0] {C_IDLE, C_AW, C_W, C_B, C_AR, C_R} cst_e;
  cst_e cs;
  assign awid[0] = 4'd3; assign awaddr[0] = c_addr; assign awlen[0] = 8'd3; assign awsize[0] = 3'd4;
  assign awburst[0] = 2'b01; assign awvalid[0] = (cs == C_AW);
  assign wdata[0] = c_line[c_beat]; assign wstrb[0] = '1; assign wlast[0] = (c_beat == 2'd3);
  assign wvalid[0] = (cs == C_W); assign bready[0] = (cs == C_B);
  assign arid[0] = 4'd8; assign araddr[0] = c_addr; assign arlen[0] = 8'd3; assign arsize[0] = 3'd4;
  assign arburst[0] = 2'b01; assign arvalid[0] = (cs == C_AR); logic rr_rnd = 1'b1; always @(negedge clk) rr_rnd = ($urandom % 4 != 0);
  assign rready[0] = (cs == C_R) && rr_rnd;
  assign c_done = (cs == C_IDLE);
  always @(posedge clk) begin
    if (!rst_n) begin cs <= C_IDLE; c_beat <= 0; end
    else unique case (cs)
      C_IDLE: if (c_go) begin cs <= c_we ? C_AW : C_AR; c_beat <= 0; end
      C_AW: if (awready[0]) cs <= C_W;
      C_W:  if (wready[0]) begin c_beat <= c_beat + 1; if (c_beat == 3) cs <= C_B; end
      C_B:  if (bvalid[0]) cs <= C_IDLE;
      C_AR: if (arready[0]) cs <= C_R;
      C_R:  if (rvalid[0] && rready[0]) begin
              for (int b = 0; b < 16; b++)
                if (rdata[0][8*b +: 8] !== ref_mem[16'(c_addr) + 16 * c_beat + b]) begin
                  if (errors < 10) $display("核读错 @%h 拍 %0d 字节 %0d: %h != %h", c_addr, c_beat, b,
                                            rdata[0][8*b +: 8], ref_mem[16'(c_addr) + 16 * c_beat + b]);
                  errors++;
                end
              if (rlast[0] != (c_beat == 3)) begin $display("核 rlast 错"); errors++; end
              c_beat <= c_beat + 1; if (rlast[0]) cs <= C_IDLE;
            end
      default: cs <= C_IDLE;
    endcase
  end

  task automatic core_write(input logic [15:0] a);
    @(posedge clk);
    while (!c_done) @(posedge clk);
    c_addr = {24'h80, a[15:6], 6'b0}; c_we = 1;
    for (int k = 0; k < 4; k++) begin
      c_line[k] = {$urandom, $urandom, $urandom, $urandom};
      for (int b = 0; b < 16; b++) ref_mem[{a[15:6], 6'b0} + 16 * k + b] = c_line[k][8*b +: 8];
    end
    @(negedge clk); c_go = 1; @(negedge clk); c_go = 0;
  endtask
  task automatic core_read(input logic [15:0] a);
    @(posedge clk);
    while (!c_done) @(posedge clk);
    c_addr = {24'h80, a[15:6], 6'b0}; c_we = 0;
    @(negedge clk); c_go = 1; @(negedge clk); c_go = 0;
  endtask

  //---------------- 主口 1：JTAG 侧
  task automatic j_write(input logic [31:0] a, input int n, input bit rnd_strb);
    logic [31:0] d; logic [3:0] s;
    @(negedge clk); j_awaddr = a; j_awlen = 8'(n - 1); j_awvalid = 1;
    @(posedge clk); while (!j_awready) @(posedge clk);
    @(negedge clk); j_awvalid = 0;
    for (int i = 0; i < n; i++) begin
      d = $urandom; s = rnd_strb ? 4'($urandom) : 4'hf;
      j_wdata = d; j_wstrb = s; j_wlast = (i == n - 1); j_wvalid = 1;
      @(posedge clk); while (!j_wready) @(posedge clk);
      if (a[31:28] == 4'h8)
        for (int b = 0; b < 4; b++) if (s[b]) ref_mem[16'(a + 4 * i + b)] = d[8*b +: 8];
      @(negedge clk); j_wvalid = 0;
    end
    @(posedge clk); while (!j_bvalid) @(posedge clk);
  endtask
  task automatic j_read(input logic [31:0] a, input int n, output logic [31:0] last);
    logic [31:0] exp;
    @(negedge clk); j_araddr = a; j_arlen = 8'(n - 1); j_arvalid = 1;
    @(posedge clk); while (!j_arready) @(posedge clk);
    @(negedge clk); j_arvalid = 0;
    for (int i = 0; i < n; i++) begin
      @(posedge clk); while (!j_rvalid) @(posedge clk);
      last = j_rdata;
      if (a[31:28] == 4'h8) begin
        for (int b = 0; b < 4; b++) exp[8*b +: 8] = ref_mem[16'(a + 4 * i + b)];
        if (j_rdata !== exp) begin
          if (errors < 10) $display("JTAG 读错 @%h: %h != %h", a + 4 * i, j_rdata, exp);
          errors++;
        end
      end
      if (j_rlast != (i == n - 1)) begin $display("JTAG rlast 错"); errors++; end
    end
  endtask

  initial begin
    logic [31:0] v;
    // 先把参考模型清零的区域写成已知值：用核写覆盖 0..4K
    repeat (5) @(posedge clk);
    rst_n = 1;
    for (int a = 0; a < 4096; a += 64) core_write(16'(a));
    $display("[%0t] 核写完", $time);
    core_read(0);
    $display("[%0t] 核读完", $time);
    // JTAG 控制寄存器
    j_read(32'hF000_0000, 1, v); if (v !== 32'd1) begin $display("CTRL 复位值错 %h", v); errors++; end
    j_write(32'hF000_000C, 1, 0);
    j_read(32'hF000_0000, 4, v);
    j_write(32'hF000_0000, 1, 0);
    j_read(32'hF000_0000, 1, v); if (v !== 32'd0 || hold) begin $display("CTRL 写 0 无效"); errors++; end
    $display("[%0t] 控制寄存器完", $time);
    // 并发：核与 JTAG 交错
    fork
      begin
        for (int i = 0; i < 200; i++) begin
          int a = ($urandom % 2048) & ~3;   // JTAG 用 0..2K，核读 2K..4K，避免同址先后不确定
          int n = 1 + $urandom % 16;
          if (a + 4 * n > 2048) n = (2048 - a) / 4;
          if ($urandom % 2) j_write(32'h8000_0000 + a, n, 1);
          else              j_read(32'h8000_0000 + a, n, v);
        end
      end
      begin
        for (int i = 0; i < 300; i++) begin
          int a = 2048 + $urandom % 2048;
          core_read(16'(a));
        end
      end
    join
    core_read(0);
    @(posedge clk); while (!c_done) @(posedge clk);
    if (errors == 0) $display("=== PASS === tb_bridge");
    else             $display("=== FAIL === tb_bridge errors=%0d", errors);
    $finish;
  end
  initial begin #20ms; $display("=== FAIL === 超时"); $finish; end
endmodule
