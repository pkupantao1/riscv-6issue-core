//==============================================================================
// o6_uart.sv - 16550 兼容 UART 子集 (reg-shift 0，字节寄存器)
//   0 RBR/THR/DLL  1 IER/DLM  2 IIR/FCR  3 LCR  4 MCR  5 LSR  6 MSR  7 SCR
// 8N1；分频 = {DLM, DLL} (波特率 = f_clk / (16 * 分频))；收发 FIFO 各 16 字节。
// 中断：IER[0] 收到数据，IER[1] 发送保持寄存器空。
// Linux 的 8250 驱动与 OpenSBI 的 uart8250 都只用到这些。
//==============================================================================
`default_nettype none
module o6_uart (
  input  wire          clk_i,
  input  wire          rst_ni,
  input  wire          req_i,
  input  wire          we_i,
  input  wire  [11:0]  addr_i,        // 偏移 (双字对齐地址 + 字节掩码定位寄存器)
  input  wire  [63:0]  wdata_i,
  input  wire  [7:0]   wmask_i,
  output logic         ack_o,
  output logic [63:0]  rdata_o,
  output logic         irq_o,
  input  wire          rx_i,
  output logic         tx_o
);
  logic [7:0] ier, lcr, mcr, scr, dll, dlm;
  wire        dlab = lcr[7];

  // 访问的是哪个字节寄存器：取字节掩码里最低的那个
  logic [2:0] reg_a;
  always_comb begin
    reg_a = 3'd0;
    for (int b = 7; b >= 0; b--) if (wmask_i[b] || !we_i) reg_a = 3'(b);
    if (!we_i) reg_a = addr_i[2:0];
  end
  wire [7:0] wbyte = wdata_i[8*reg_a +: 8];

  // FIFO
  logic [7:0] txq [16]; logic [3:0] tx_h, tx_t; logic [4:0] tx_n;
  logic [7:0] rxq [16]; logic [3:0] rx_h, rx_t; logic [4:0] rx_n;

  // 发送器
  logic [15:0] tdiv; logic [3:0] tsub; logic [3:0] tbit; logic [9:0] tsh; logic tbusy;
  // 接收器
  logic [15:0] rdiv; logic [3:0] rsub; logic [3:0] rbit; logic [7:0] rsh; logic rbusy;
  logic [1:0]  rx_s;
  wire  [15:0] divisor = ({dlm, dll} == 16'd0) ? 16'd1 : {dlm, dll};

  wire thre = (tx_n == 0);
  wire temt = thre && !tbusy;
  wire dr   = (rx_n != 0);
  wire [7:0] lsr = {1'b0, temt, thre, 4'b0, dr};
  wire [7:0] iir = (ier[0] && dr) ? 8'hC4 : (ier[1] && thre) ? 8'hC2 : 8'hC1;
  assign irq_o = (ier[0] && dr) || (ier[1] && thre);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ier <= '0; lcr <= 8'h03; mcr <= '0; scr <= '0; dll <= 8'd1; dlm <= '0;
      tx_h <= '0; tx_t <= '0; tx_n <= '0; rx_h <= '0; rx_t <= '0; rx_n <= '0;
      tbusy <= 1'b0; tx_o <= 1'b1; tdiv <= '0; tsub <= '0; tbit <= '0;
      rbusy <= 1'b0; rdiv <= '0; rsub <= '0; rbit <= '0; rx_s <= 2'b11;
      ack_o <= 1'b0; rdata_o <= '0;
      tsh <= '1; rsh <= '0;
      for (int i = 0; i < 16; i++) begin txq[i] <= '0; rxq[i] <= '0; end
    end else begin
      logic [4:0] txn, rxn;
      txn = tx_n; rxn = rx_n;
      ack_o <= req_i && !ack_o;

      // ---- 寄存器访问
      if (req_i && !ack_o) begin
        logic [7:0] r;
        r = 8'h00;
        unique case (reg_a)
          3'd0: if (dlab) begin r = dll; if (we_i) dll <= wbyte; end
                else if (we_i) begin
                  if (txn < 16) begin txq[tx_t] <= wbyte; tx_t <= tx_t + 1'b1; txn = txn + 1'b1; end
`ifndef SYNTHESIS
                  $write("%c", wbyte);
                  if (wbyte == 8'h0a) $fflush;
`endif
                end else begin
                  r = rxq[rx_h];
                  if (rxn != 0) begin rx_h <= rx_h + 1'b1; rxn = rxn - 1'b1; end
                end
          3'd1: if (dlab) begin r = dlm; if (we_i) dlm <= wbyte; end
                else begin r = ier; if (we_i) ier <= wbyte & 8'h0F; end
          3'd2: begin
                  r = iir;
                  if (we_i && wbyte[1]) begin rx_h <= '0; rx_t <= '0; rxn = '0; end
                  if (we_i && wbyte[2]) begin tx_h <= tx_h; tx_t <= tx_h; txn = '0; end
                end
          3'd3: begin r = lcr; if (we_i) lcr <= wbyte; end
          3'd4: begin r = mcr; if (we_i) mcr <= wbyte; end
          3'd5: r = lsr;
          3'd6: r = 8'hB0;               // DCD/DSR/CTS 有效
          default: begin r = scr; if (we_i) scr <= wbyte; end
        endcase
        rdata_o <= {8{r}};
      end

      // ---- 发送：每位 16 个子拍
      if (!tbusy) begin
        if (txn != 0) begin
          tsh <= {1'b1, txq[tx_h], 1'b0};
          tx_h <= tx_h + 1'b1; txn = txn - 1'b1;
          tbusy <= 1'b1; tbit <= '0; tsub <= '0; tdiv <= '0;
        end
      end else begin
        tx_o <= tsh[0];
        if (tdiv == divisor - 1) begin
          tdiv <= '0;
          if (tsub == 4'd15) begin
            tsub <= '0;
            tsh  <= {1'b1, tsh[9:1]};
            if (tbit == 4'd9) tbusy <= 1'b0;
            tbit <= tbit + 1'b1;
          end else tsub <= tsub + 1'b1;
        end else tdiv <= tdiv + 1'b1;
      end
      if (!tbusy) tx_o <= 1'b1;

      // ---- 接收：起始位中点采样
      rx_s <= {rx_s[0], rx_i};
      if (!rbusy) begin
        if (rx_s[1] == 1'b0) begin rbusy <= 1'b1; rdiv <= '0; rsub <= '0; rbit <= '0; end
      end else if (rdiv == divisor - 1) begin
        rdiv <= '0;
        rsub <= rsub + 1'b1;
        if (rsub == 4'd7 && rbit == 4'd0 && rx_s[1]) rbusy <= 1'b0;      // 假起始位
        else if (rsub == 4'd7 && rbit != 4'd0) begin
          if (rbit <= 4'd8) rsh <= {rx_s[1], rsh[7:1]};
          if (rbit == 4'd9) begin
            if (rxn < 16) begin rxq[rx_t] <= rsh; rx_t <= rx_t + 1'b1; rxn = rxn + 1'b1; end
            rbusy <= 1'b0;
          end
        end
        if (rsub == 4'd15) rbit <= rbit + 1'b1;
      end else rdiv <= rdiv + 1'b1;

      tx_n <= txn; rx_n <= rxn;
    end
  end
endmodule
