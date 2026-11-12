//==============================================================================
// o6_busytable.sv - 物理寄存器 "结果未就绪" 位表 (一类寄存器一份)
//
// 置位：重命名分配新 pdst
// 清零：唤醒广播 (结果将在被唤醒者需要时经旁路/PRF 可得)
// 读  ：重命名级读源操作数；本拍的唤醒直接旁路 (表在拍末才更新)
// 冲刷：异常冲刷后所有映射都指向已提交的值 -> 全部清零
//
// 误预测不需要处理：被作废指令的 pdst 退回 free list，再分配时会重新置位；
// 被作废的指令在 br_update 那一拍之后不会再产生唤醒 (各执行单元负责丢弃)。
//==============================================================================
`default_nettype none
module o6_busytable
  import o6_pkg::*;
#(
  parameter int N      = 192,
  parameter int NRD    = 3*W,
  parameter int NWAKE  = 8
) (
  input  wire              clk_i,
  input  wire              rst_ni,
  input  wire              flush_i,
  input  wire  [W-1:0]     set_en_i,
  input  var preg_t [W-1:0]    set_preg_i,
  input  wire  [NWAKE-1:0] clr_en_i,
  input  var preg_t [NWAKE-1:0] clr_preg_i,
  input  var preg_t [NRD-1:0]  rd_preg_i,
  output logic [NRD-1:0]   rd_busy_o
);
  logic [N-1:0] busy;

  always_comb begin
    for (int r = 0; r < NRD; r++) begin
      logic b;
      b = busy[rd_preg_i[r]];
      for (int k = 0; k < NWAKE; k++)
        if (clr_en_i[k] && clr_preg_i[k] == rd_preg_i[r]) b = 1'b0;
      rd_busy_o[r] = b;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy <= '0;
    end else if (flush_i) begin
      busy <= '0;
    end else begin
      logic [N-1:0] nb;
      nb = busy;
      for (int k = 0; k < NWAKE; k++)
        if (clr_en_i[k]) nb[clr_preg_i[k]] = 1'b0;
      // 置位在清零之后：新分配的 pdst 不可能同时被合法唤醒
      for (int k = 0; k < W; k++)
        if (set_en_i[k]) nb[set_preg_i[k]] = 1'b1;
      busy <= nb;
    end
  end
endmodule
