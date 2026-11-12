//==============================================================================
// o6_axi_arb2.sv - AXI4 二选一仲裁 (两个主 -> 一个从)，128 位
//
// 读、写通道各自独立仲裁，按事务加锁：
//   写：AW 握手时锁定主口，直到 B 握手释放 (从机同时只接一个写事务)
//   读：AR 握手时锁定主口，直到 RLAST 握手释放
// 优先级轮转 (上次赢家让位)。ID 原样透传，响应按锁定的主口回送，不改 ID。
// 只适用于同一时刻每个方向只有一个在途事务的从机 (o6_axi_ram 就是这样)。
//==============================================================================
`default_nettype none
module o6_axi_arb2 #(
  parameter int AW  = 40,
  parameter int IDW = 4
) (
  input  wire             clk_i,
  input  wire             rst_ni,
  // 主口 0/1 (数组下标 = 主口号)
  input  wire  [IDW-1:0]  s_awid    [2],
  input  wire  [AW-1:0]   s_awaddr  [2],
  input  wire  [7:0]      s_awlen   [2],
  input  wire  [2:0]      s_awsize  [2],
  input  wire  [1:0]      s_awburst [2],
  input  wire             s_awvalid [2],
  output logic            s_awready [2],
  input  wire  [127:0]    s_wdata   [2],
  input  wire  [15:0]     s_wstrb   [2],
  input  wire             s_wlast   [2],
  input  wire             s_wvalid  [2],
  output logic            s_wready  [2],
  output logic [IDW-1:0]  s_bid     [2],
  output logic [1:0]      s_bresp   [2],
  output logic            s_bvalid  [2],
  input  wire             s_bready  [2],
  input  wire  [IDW-1:0]  s_arid    [2],
  input  wire  [AW-1:0]   s_araddr  [2],
  input  wire  [7:0]      s_arlen   [2],
  input  wire  [2:0]      s_arsize  [2],
  input  wire  [1:0]      s_arburst [2],
  input  wire             s_arvalid [2],
  output logic            s_arready [2],
  output logic [IDW-1:0]  s_rid     [2],
  output logic [127:0]    s_rdata   [2],
  output logic [1:0]      s_rresp   [2],
  output logic            s_rlast   [2],
  output logic            s_rvalid  [2],
  input  wire             s_rready  [2],
  // 从口
  output logic [IDW-1:0]  m_awid,
  output logic [AW-1:0]   m_awaddr,
  output logic [7:0]      m_awlen,
  output logic [2:0]      m_awsize,
  output logic [1:0]      m_awburst,
  output logic            m_awvalid,
  input  wire             m_awready,
  output logic [127:0]    m_wdata,
  output logic [15:0]     m_wstrb,
  output logic            m_wlast,
  output logic            m_wvalid,
  input  wire             m_wready,
  input  wire  [IDW-1:0]  m_bid,
  input  wire  [1:0]      m_bresp,
  input  wire             m_bvalid,
  output logic            m_bready,
  output logic [IDW-1:0]  m_arid,
  output logic [AW-1:0]   m_araddr,
  output logic [7:0]      m_arlen,
  output logic [2:0]      m_arsize,
  output logic [1:0]      m_arburst,
  output logic            m_arvalid,
  input  wire             m_arready,
  input  wire  [IDW-1:0]  m_rid,
  input  wire  [127:0]    m_rdata,
  input  wire  [1:0]      m_rresp,
  input  wire             m_rlast,
  input  wire             m_rvalid,
  output logic            m_rready
);
  //---------------- 写通道
  logic w_lock, w_own, w_last;     // w_lock: 有事务在途；w_own: 在途事务属于哪个主口；w_last: 上次赢家
  logic w_sel;                     // 本拍 AW 选哪个主口
  always_comb begin
    if (w_lock)                                   w_sel = w_own;
    else if (s_awvalid[0] && s_awvalid[1])        w_sel = !w_last;
    else                                          w_sel = s_awvalid[1];
  end
  // AW 只在未加锁时放行；W/B 跟随锁定主口
  assign m_awid    = s_awid[w_sel];
  assign m_awaddr  = s_awaddr[w_sel];
  assign m_awlen   = s_awlen[w_sel];
  assign m_awsize  = s_awsize[w_sel];
  assign m_awburst = s_awburst[w_sel];
  assign m_awvalid = !w_lock && s_awvalid[w_sel];
  assign m_wdata   = s_wdata[w_own];
  assign m_wstrb   = s_wstrb[w_own];
  assign m_wlast   = s_wlast[w_own];
  assign m_wvalid  = w_lock && s_wvalid[w_own];
  assign m_bready  = w_lock && s_bready[w_own];
  for (genvar i = 0; i < 2; i++) begin : g_wresp
    assign s_awready[i] = !w_lock && (w_sel == 1'(i)) && m_awready;
    assign s_wready[i]  = w_lock && (w_own == 1'(i)) && m_wready;
    assign s_bid[i]     = m_bid;
    assign s_bresp[i]   = m_bresp;
    assign s_bvalid[i]  = w_lock && (w_own == 1'(i)) && m_bvalid;
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_lock <= 1'b0; w_own <= 1'b0; w_last <= 1'b0;
    end else begin
      if (m_awvalid && m_awready) begin w_lock <= 1'b1; w_own <= w_sel; w_last <= w_sel; end
      if (m_bvalid && m_bready)   w_lock <= 1'b0;
    end
  end

  //---------------- 读通道
  logic r_lock, r_own, r_last, r_sel;
  always_comb begin
    if (r_lock)                                   r_sel = r_own;
    else if (s_arvalid[0] && s_arvalid[1])        r_sel = !r_last;
    else                                          r_sel = s_arvalid[1];
  end
  assign m_arid    = s_arid[r_sel];
  assign m_araddr  = s_araddr[r_sel];
  assign m_arlen   = s_arlen[r_sel];
  assign m_arsize  = s_arsize[r_sel];
  assign m_arburst = s_arburst[r_sel];
  assign m_arvalid = !r_lock && s_arvalid[r_sel];
  assign m_rready  = r_lock && s_rready[r_own];
  for (genvar i = 0; i < 2; i++) begin : g_rresp
    assign s_arready[i] = !r_lock && (r_sel == 1'(i)) && m_arready;
    assign s_rid[i]     = m_rid;
    assign s_rdata[i]   = m_rdata;
    assign s_rresp[i]   = m_rresp;
    assign s_rlast[i]   = m_rlast;
    assign s_rvalid[i]  = r_lock && (r_own == 1'(i)) && m_rvalid;
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      r_lock <= 1'b0; r_own <= 1'b0; r_last <= 1'b0;
    end else begin
      if (m_arvalid && m_arready)          begin r_lock <= 1'b1; r_own <= r_sel; r_last <= r_sel; end
      if (m_rvalid && m_rready && m_rlast) r_lock <= 1'b0;
    end
  end
endmodule
