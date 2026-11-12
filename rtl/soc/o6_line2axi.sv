//==============================================================================
// o6_line2axi.sv - 64 B 行请求 -> AXI4 主口 (128 位，INCR 4 拍)，多事务在途
//
// 读：AR 带上游请求号 (ARID)，最多 RQ 个在途；R 数据按 RID 回来 (假定同一突发不与别的 ID 交织，
//     仿真里断言)，收齐 4 拍进响应 FIFO。在途读数 + FIFO 占用不超过 RQ，FIFO 永远放得下。
// 写：AW + 4 拍 W，B 回来前可以继续发下一个；记录在途写的行地址 (至多 WQ 个，B 按序返回、ID 固定 0)。
// 顺序：AXI 不保证读、写两个通道之间的先后 —— 读的行若与在途写相同，等那个写的 B 回来再发 AR
//     (L2 / D$ 写回牺牲行后又缺失同一行时，否则会读到旧数据)。
// 防死锁：上游 (D$ / L2) 在有待发写回时会拒收响应，等写回被接受。所以只有读受 FIFO 容量约束，
//     写请求只看写通道是否空闲。
//==============================================================================
`default_nettype none
module o6_line2axi
  import o6_pkg::*;
#(
  parameter int IDW = 4,
  parameter int RQ  = 8,             // 读在途上限 (= 响应 FIFO 深度)
  parameter int WQ  = 4              // 写在途上限 (等 B)
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  input  wire                 req_valid_i,
  output wire                 req_ready_o,
  input  wire                 req_we_i,
  input  wire  [PA_W-1:6]     req_addr_i,
  input  wire  [511:0]        req_data_i,
  input  wire  [3:0]          req_id_i,
  output logic                resp_valid_o,
  input  wire                 resp_ready_i,
  output logic [3:0]          resp_id_o,
  output logic [511:0]        resp_data_o,

  // AXI4 主口
  output logic [IDW-1:0]      m_awid,
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
  input  wire  [IDW-1:0]      m_bid,
  input  wire  [1:0]          m_bresp,
  input  wire                 m_bvalid,
  output logic                m_bready,
  output logic [IDW-1:0]      m_arid,
  output logic [PA_W-1:0]     m_araddr,
  output logic [7:0]          m_arlen,
  output logic [2:0]          m_arsize,
  output logic [1:0]          m_arburst,
  output logic                m_arvalid,
  input  wire                 m_arready,
  input  wire  [IDW-1:0]      m_rid,
  input  wire  [127:0]        m_rdata,
  input  wire  [1:0]          m_rresp,
  input  wire                 m_rlast,
  input  wire                 m_rvalid,
  output logic                m_rready
);
  localparam int RQW = $clog2(RQ);
  localparam int WQW = $clog2(WQ);

  //--------------------------------------------------------------------------
  // 写通道：AW/W 一次一个 (发完 4 拍即可接下一个)，在途写的行地址表等 B
  //--------------------------------------------------------------------------
  logic            w_act, aw_done;        // 正在发 AW/W
  logic [PA_W-1:6] w_addr;
  logic [511:0]    w_data;
  logic [1:0]      w_beat;
  logic [PA_W-1:6] wt_addr [WQ];          // 在途写 (已发 AW、等 B)，按序
  logic [WQW-1:0]  wt_h, wt_t;
  logic [WQW:0]    wt_n;

  // 读请求与在途写 (含正在发的) 同一行
  logic r_conf;
  always_comb begin
    r_conf = w_act && w_addr == req_addr_i;
    for (int i = 0; i < WQ; i++)
      if ((WQW+1)'(i) < wt_n && wt_addr[WQW'(wt_h + WQW'(i))] == req_addr_i) r_conf = 1'b1;
  end

  //--------------------------------------------------------------------------
  // 读通道
  //--------------------------------------------------------------------------
  logic            ar_v;                  // AR 待发
  logic [PA_W-1:6] ar_addr;
  logic [3:0]      ar_id;
  logic [RQW:0]    r_out;                 // 在途读数 (已接收、未进 FIFO)
  logic [511:0]    r_buf;                 // 正在收的突发
  logic [1:0]      r_beat;
  logic [IDW-1:0]  r_cur;

  // 响应 FIFO
  logic [3:0]      fq_id   [RQ];
  logic [511:0]    fq_data [RQ];
  logic [RQW-1:0]  fq_h, fq_t;
  logic [RQW:0]    fq_n;

  wire rd_room = (r_out + fq_n) < (RQW+1)'(RQ);
  // 写也不越过还没发出 AR 的同一行读 (读在前，写在后，AR 后发会读到新数据)
  assign req_ready_o = req_we_i ? (!w_act && wt_n < (WQW+1)'(WQ) && !(ar_v && ar_addr == req_addr_i))
                                : (!ar_v && rd_room && !r_conf);
  wire take = req_valid_i && req_ready_o;

  assign m_awid    = '0;
  assign m_awaddr  = {w_addr, 6'b0};
  assign m_awlen   = 8'd3;
  assign m_awsize  = 3'd4;
  assign m_awburst = 2'b01;
  assign m_awvalid = w_act && !aw_done;
  assign m_wdata   = w_data[128*w_beat +: 128];
  assign m_wstrb   = '1;
  assign m_wlast   = (w_beat == 2'd3);
  assign m_wvalid  = w_act && aw_done;
  assign m_bready  = 1'b1;
  assign m_arid    = IDW'(ar_id);
  assign m_araddr  = {ar_addr, 6'b0};
  assign m_arlen   = 8'd3;
  assign m_arsize  = 3'd4;
  assign m_arburst = 2'b01;
  assign m_arvalid = ar_v;
  assign m_rready  = 1'b1;              // FIFO 容量已按在途读数预留

  assign resp_valid_o = (fq_n != 0);
  assign resp_id_o    = fq_id[fq_h];
  assign resp_data_o  = fq_data[fq_h];

  wire rd_done = m_rvalid && m_rlast;
  wire fq_pop  = resp_valid_o && resp_ready_i;
  wire w_fin   = m_wvalid && m_wready && m_wlast;
  wire b_fin   = m_bvalid && m_bready;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_act <= 1'b0; aw_done <= 1'b0; w_beat <= '0; wt_h <= '0; wt_t <= '0; wt_n <= '0;
      ar_v <= 1'b0; r_out <= '0; r_beat <= '0; fq_h <= '0; fq_t <= '0; fq_n <= '0;
    end else begin
      // 写
      if (take && req_we_i) begin w_act <= 1'b1; aw_done <= 1'b0; w_beat <= '0; end
      if (m_awvalid && m_awready) aw_done <= 1'b1;
      if (m_wvalid && m_wready) begin
        w_beat <= w_beat + 1'b1;
        if (m_wlast) w_act <= 1'b0;
      end
      if (w_fin) wt_t <= wt_t + 1'b1;
      if (b_fin) wt_h <= wt_h + 1'b1;
      wt_n <= wt_n + w_fin - b_fin;
      // 读
      if (take && !req_we_i) ar_v <= 1'b1;
      else if (m_arvalid && m_arready) ar_v <= 1'b0;
      if (m_rvalid) r_beat <= m_rlast ? 2'd0 : r_beat + 1'b1;
      r_out <= r_out + (take && !req_we_i) - rd_done;
      if (rd_done) fq_t <= fq_t + 1'b1;
      if (fq_pop)  fq_h <= fq_h + 1'b1;
      fq_n <= fq_n + rd_done - fq_pop;
    end
  end

  always_ff @(posedge clk_i) begin
    if (take && req_we_i) begin w_addr <= req_addr_i; w_data <= req_data_i; end
    if (w_fin) wt_addr[wt_t] <= w_addr;
    if (take && !req_we_i) begin ar_addr <= req_addr_i; ar_id <= req_id_i; end
    if (m_rvalid) begin r_buf[128*r_beat +: 128] <= m_rdata; r_cur <= m_rid; end
    if (rd_done) begin
      fq_id[fq_t]   <= 4'(m_rid);
      fq_data[fq_t] <= {m_rdata, r_buf[383:0]};
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni) begin
    if (m_rvalid && r_beat != 2'd0)
      assert (m_rid == r_cur) else $error("line2axi: 读突发交织 (RID %0d 插在 %0d 的突发中间)", m_rid, r_cur);
    assert (fq_n <= RQ) else $error("line2axi: 响应 FIFO 溢出");
  end
`endif
  wire unused = (|m_bid) ^ (|m_bresp) ^ (|m_rresp);
endmodule
