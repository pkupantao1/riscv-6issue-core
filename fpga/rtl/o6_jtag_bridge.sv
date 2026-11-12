//==============================================================================
// o6_jtag_bridge.sv - JTAG-to-AXI (AXI4, 32 位) -> 128 位 AXI4 主口 + 控制寄存器
//
// 上游接 Xilinx JTAG-to-AXI Master IP (PROTOCOL=AXI4, DATA_WIDTH=32)，支持 INCR 突发。
// 每个 32 位拍转成下游一次单拍 128 位访问 (按地址[3:2]移位 / 选通)，JTAG 本身是瓶颈，
// 这种做法简单可靠。
//
// 地址空间 (上游地址)：
//   CTRL_BASE + 0x00  CTRL   bit0 = 核复位保持 (上电为 1，写 0 放核运行)
//   CTRL_BASE + 0x04  STATUS bit0 = 核复位状态, bit1 = MMCM 锁定
//   CTRL_BASE + 0x08  CYCLE  核时钟周期计数 (低 32 位)
//   CTRL_BASE + 0x0C  SCRATCH 读写寄存器 (连通性检查)
//   其余地址          转发到下游 (DRAM)
//==============================================================================
`default_nettype none
module o6_jtag_bridge #(
  parameter int          AW        = 40,
  parameter int          IDW       = 4,
  parameter logic [31:0] CTRL_BASE = 32'hF000_0000
) (
  input  wire             clk_i,
  input  wire             rst_ni,
  input  wire             locked_i,
  output logic            core_hold_o,
  // 上游 AXI4 从口 (32 位)
  input  wire  [31:0]     s_awaddr,
  input  wire  [7:0]      s_awlen,
  input  wire             s_awvalid,
  output logic            s_awready,
  input  wire  [31:0]     s_wdata,
  input  wire  [3:0]      s_wstrb,
  input  wire             s_wlast,
  input  wire             s_wvalid,
  output logic            s_wready,
  output logic [1:0]      s_bresp,
  output logic            s_bvalid,
  input  wire             s_bready,
  input  wire  [31:0]     s_araddr,
  input  wire  [7:0]      s_arlen,
  input  wire             s_arvalid,
  output logic            s_arready,
  output logic [31:0]     s_rdata,
  output logic [1:0]      s_rresp,
  output logic            s_rlast,
  output logic            s_rvalid,
  input  wire             s_rready,
  // 下游 AXI4 主口 (128 位，单拍)
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
  //---------------- 控制寄存器
  logic        ctrl_hold;
  logic [31:0] scratch, cycle;
  assign core_hold_o = ctrl_hold;
  function automatic logic is_ctrl(input logic [31:0] a);
    return a[31:12] == CTRL_BASE[31:12];
  endfunction
  function automatic logic [31:0] ctrl_rd(input logic [3:0] off);
    case (off[3:2])
      2'd0:    return {31'b0, ctrl_hold};
      2'd1:    return {30'b0, locked_i, ctrl_hold};
      2'd2:    return cycle;
      default: return scratch;
    endcase
  endfunction

  //---------------- 写 FSM
  typedef enum logic [1:0] {W_IDLE, W_DATA, W_DOWN, W_RESP} wst_e;
  wst_e        ws;
  logic [31:0] wa;
  logic [31:0] wd;
  logic [3:0]  wb;
  logic        wl, w_aw_done, w_w_done;

  assign s_awready = (ws == W_IDLE);
  assign s_wready  = (ws == W_DATA);
  assign s_bresp   = 2'b00;
  assign s_bvalid  = (ws == W_RESP);

  assign m_awid    = '0;
  assign m_awaddr  = {{(AW-32){1'b0}}, wa[31:4], 4'b0};
  assign m_awlen   = 8'd0;
  assign m_awsize  = 3'd4;
  assign m_awburst = 2'b01;
  assign m_awvalid = (ws == W_DOWN) && !w_aw_done;
  assign m_wdata   = {4{wd}};
  assign m_wstrb   = 16'(wb) << (4 * wa[3:2]);
  assign m_wlast   = 1'b1;
  assign m_wvalid  = (ws == W_DOWN) && !w_w_done;
  assign m_bready  = (ws == W_DOWN);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ws <= W_IDLE; ctrl_hold <= 1'b1; scratch <= '0; cycle <= '0;
      wa <= '0; wd <= '0; wb <= '0; wl <= 1'b0; w_aw_done <= 1'b0; w_w_done <= 1'b0;
    end else begin
      cycle <= cycle + 1'b1;
      unique case (ws)
        W_IDLE: if (s_awvalid) begin wa <= s_awaddr; ws <= W_DATA; end
        W_DATA: if (s_wvalid) begin
          wd <= s_wdata; wb <= s_wstrb; wl <= s_wlast;
          if (is_ctrl(wa)) begin
            // 控制寄存器写：只取全字节写
            if (wa[3:2] == 2'd0 && s_wstrb[0]) ctrl_hold <= s_wdata[0];
            if (wa[3:2] == 2'd3)               scratch   <= s_wdata;
            wa <= wa + 32'd4;
            if (s_wlast) ws <= W_RESP;
          end else begin
            w_aw_done <= 1'b0; w_w_done <= 1'b0;
            ws <= W_DOWN;
          end
        end
        W_DOWN: begin
          if (m_awvalid && m_awready) w_aw_done <= 1'b1;
          if (m_wvalid && m_wready)   w_w_done  <= 1'b1;
          if (m_bvalid) begin
            wa <= wa + 32'd4;
            ws <= wl ? W_RESP : W_DATA;
          end
        end
        W_RESP: if (s_bready) ws <= W_IDLE;
        default: ws <= W_IDLE;
      endcase
    end
  end

  //---------------- 读 FSM
  typedef enum logic [1:0] {R_IDLE, R_DOWN, R_WAIT, R_RESP} rst_e;
  rst_e        rs;
  logic [31:0] ra, rd;
  logic [7:0]  rleft;

  assign s_arready = (rs == R_IDLE);
  assign s_rdata   = rd;
  assign s_rresp   = 2'b00;
  assign s_rlast   = (rleft == 8'd0);
  assign s_rvalid  = (rs == R_RESP);

  assign m_arid    = '0;
  assign m_araddr  = {{(AW-32){1'b0}}, ra[31:4], 4'b0};
  assign m_arlen   = 8'd0;
  assign m_arsize  = 3'd4;
  assign m_arburst = 2'b01;
  assign m_arvalid = (rs == R_DOWN);
  assign m_rready  = (rs == R_WAIT);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rs <= R_IDLE; ra <= '0; rd <= '0; rleft <= '0;
    end else begin
      unique case (rs)
        R_IDLE: if (s_arvalid) begin
          ra <= s_araddr; rleft <= s_arlen;
          if (is_ctrl(s_araddr)) begin rd <= ctrl_rd(s_araddr[3:0]); rs <= R_RESP; end
          else                   rs <= R_DOWN;
        end
        R_DOWN: if (m_arready) rs <= R_WAIT;
        R_WAIT: if (m_rvalid) begin rd <= m_rdata[32*ra[3:2] +: 32]; rs <= R_RESP; end
        R_RESP: if (s_rready) begin
          if (rleft == 8'd0) rs <= R_IDLE;
          else begin
            ra <= ra + 32'd4; rleft <= rleft - 1'b1;
            if (is_ctrl(ra + 32'd4)) rd <= ctrl_rd(4'(ra[3:0] + 4'd4));
            rs <= is_ctrl(ra + 32'd4) ? R_RESP : R_DOWN;
          end
        end
        default: rs <= R_IDLE;
      endcase
    end
  end

  wire unused = (|m_bid) ^ (|m_bresp) ^ (|m_rid) ^ (|m_rresp) ^ m_rlast;
endmodule
