//==============================================================================
// o6_axi_ram.sv - AXI4 从机 RAM (FPGA 用：VU7P 上推断 URAM 作主存)
//
// 128 位数据，INCR 突发，读写各一个在途事务 (读写可并行，但同一时刻对 RAM 只有一个口，
// 写优先)。读延迟 RD_PIPE 拍 (URAM 级联深时需要输出寄存器)。
// 地址 = DRAM_BASE 起的偏移，超出容量的高位忽略 (回绕)。
//==============================================================================
`default_nettype none
module o6_axi_ram #(
  parameter int AW      = 40,
  parameter int IDW     = 4,
  parameter int BYTES   = 16 * 1024 * 1024,
  parameter int RD_PIPE = 2
) (
  input  wire              clk_i,
  input  wire              rst_ni,
  input  wire  [IDW-1:0]   s_awid,
  input  wire  [AW-1:0]    s_awaddr,
  input  wire  [7:0]       s_awlen,
  input  wire  [2:0]       s_awsize,
  input  wire  [1:0]       s_awburst,
  input  wire              s_awvalid,
  output logic             s_awready,
  input  wire  [127:0]     s_wdata,
  input  wire  [15:0]      s_wstrb,
  input  wire              s_wlast,
  input  wire              s_wvalid,
  output logic             s_wready,
  output logic [IDW-1:0]   s_bid,
  output logic [1:0]       s_bresp,
  output logic             s_bvalid,
  input  wire              s_bready,
  input  wire  [IDW-1:0]   s_arid,
  input  wire  [AW-1:0]    s_araddr,
  input  wire  [7:0]       s_arlen,
  input  wire  [2:0]       s_arsize,
  input  wire  [1:0]       s_arburst,
  input  wire              s_arvalid,
  output logic             s_arready,
  output logic [IDW-1:0]   s_rid,
  output logic [127:0]     s_rdata,
  output logic [1:0]       s_rresp,
  output logic             s_rlast,
  output logic             s_rvalid,
  input  wire              s_rready
);
  localparam int DEPTH = BYTES / 16;
  localparam int IW    = $clog2(DEPTH);

  (* ram_style = "ultra" *) logic [127:0] mem [DEPTH];

  //---------------- 写
  logic           w_busy;
  logic [IW-1:0]  w_idx;
  logic [IDW-1:0] w_id;
  assign s_awready = !w_busy && !s_bvalid;
  assign s_wready  = w_busy;
  wire   w_fire    = s_wvalid && s_wready;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_busy <= 1'b0; s_bvalid <= 1'b0;
    end else begin
      if (s_awvalid && s_awready) w_busy <= 1'b1;
      if (w_fire && s_wlast) begin w_busy <= 1'b0; s_bvalid <= 1'b1; end
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    if (s_awvalid && s_awready) begin w_idx <= s_awaddr[IW+3:4]; w_id <= s_awid; end
    else if (w_fire) w_idx <= w_idx + 1'b1;
  end
  assign s_bid   = w_id;
  assign s_bresp = 2'b00;

  //---------------- 读 (写占用 RAM 口时读暂停)
  logic           r_busy;
  logic [IW-1:0]  r_idx;
  logic [7:0]     r_left;
  logic [IDW-1:0] r_id;
  logic [RD_PIPE:0] pv;           // 读流水有效
  logic [RD_PIPE:0] pl;           // 读流水 last
  logic [127:0]   pd [RD_PIPE+1];
  // 输出 FIFO 余量：流水里在途 + 已出的未被接收不超过深度
  logic [2:0]     q_n;
  logic [127:0]   q_d [4];
  logic           q_l [4];
  logic [1:0]     q_h, q_t;
  wire  [2:0]     inflight = 3'(pv[0]) + 3'(pv[1]) + ((RD_PIPE > 1) ? 3'(pv[2]) : 3'd0);
  wire            rd_go = r_busy && !w_fire && (q_n + inflight) < 3'd4;

  assign s_arready = !r_busy;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      r_busy <= 1'b0; pv <= '0; q_n <= '0; q_h <= '0; q_t <= '0;
    end else begin
      logic [2:0] n;
      n = q_n;
      if (s_arvalid && s_arready) r_busy <= 1'b1;
      if (rd_go && r_left == 8'd0) r_busy <= 1'b0;
      pv <= {pv[RD_PIPE-1:0], rd_go};
      if (pv[RD_PIPE]) begin q_t <= q_t + 1'b1; n = n + 1'b1; end
      if (s_rvalid && s_rready) begin q_h <= q_h + 1'b1; n = n - 1'b1; end
      q_n <= n;
    end
  end
  always_ff @(posedge clk_i) begin
    if (s_arvalid && s_arready) begin r_idx <= s_araddr[IW+3:4]; r_left <= s_arlen; r_id <= s_arid; end
    else if (rd_go) begin r_idx <= r_idx + 1'b1; r_left <= r_left - 1'b1; end
    // RAM：写优先
    if (w_fire) begin
      for (int b = 0; b < 16; b++) if (s_wstrb[b]) mem[w_idx][8*b +: 8] <= s_wdata[8*b +: 8];
    end
    pd[0] <= mem[r_idx];
    pl    <= {pl[RD_PIPE-1:0], rd_go && r_left == 8'd0};
    for (int i = 1; i <= RD_PIPE; i++) pd[i] <= pd[i-1];
    if (pv[RD_PIPE]) begin q_d[q_t] <= pd[RD_PIPE]; q_l[q_t] <= pl[RD_PIPE]; end
  end

  assign s_rvalid = (q_n != 0);
  assign s_rdata  = q_d[q_h];
  assign s_rlast  = q_l[q_h];
  assign s_rid    = r_id;
  assign s_rresp  = 2'b00;

  wire unused = (|s_awlen) ^ (|s_awsize) ^ (|s_awburst) ^ (|s_arsize) ^ (|s_arburst);
endmodule
