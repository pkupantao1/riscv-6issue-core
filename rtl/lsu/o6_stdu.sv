//==============================================================================
// o6_stdu.sv - 整数 STD 单元：N_STDP 条专用通道，把 store / AMO 的数据 (rs2) 写进 SQ
//
// 每条通道：IS (IQ_D 选择) -> RF (读 PRF + 旁路) -> EX (再旁路一次，送 SQ 数据)
//   时序与原来挤在 P3 上的 STD 相同 (2026-10-13 拆出：dhrystone 里 P3 被 STD 与 ALU/DIV/CSR 抢)。
// 旁路只比较 wbreg (W0..)：wbreg 那一拍既写 PRF 又作为旁路源，RF 与 EX 两处各比一次即可。
//==============================================================================
`default_nettype none
module o6_stdu
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,
  input  wire  [N_LDP-1:0]    ld_fail_i,        // load 投机唤醒失败 (本拍)：RF 级丢弃带 poia 的 uop
  input  wire  [N_LDP-1:0]    ld_fail_q_i,      // load 投机唤醒失败 (寄存)：RF 级丢弃带 pois 的 uop

  input  wire  [N_STDP-1:0]   iss_valid_i,
  input  var uop_t [N_STDP-1:0] iss_uop_i,

  output preg_t [N_STDP-1:0]  prf_raddr_o,
  input  wire  [63:0]         prf_rdata_i [N_STDP],
  input  var wb_t [N_IWB-1:0] wb_i,

  output logic [N_STDP-1:0]   std_valid_o,
  output sq_ptr_t [N_STDP-1:0] std_sq_o,
  output rob_ptr_t [N_STDP-1:0] std_rob_o,
  output logic [63:0]         std_data_o [N_STDP]
);
  function automatic logic [63:0] byp(input preg_t p, input logic [63:0] v, input wb_t [N_IWB-1:0] wb);
    logic [63:0] r;
    r = v;
    for (int k = 0; k < N_IWB; k++)
      if (wb[k].valid && wb[k].wen && wb[k].pdst == p && p != '0) r = wb[k].data;
    return r;
  endfunction

  logic [N_STDP-1:0] rf_v, ex_v;
  uop_t              rf_u [N_STDP], ex_u [N_STDP];
  logic [63:0]       rf_d [N_STDP], ex_d0 [N_STDP];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rf_v <= '0; ex_v <= '0;
    end else if (flush_i) begin
      rf_v <= '0; ex_v <= '0;
    end else begin
      for (int p = 0; p < N_STDP; p++) begin
        rf_v[p] <= iss_valid_i[p] && !br_killed(iss_uop_i[p].br_mask, br_upd_i);
        ex_v[p] <= rf_v[p] && !br_killed(rf_u[p].br_mask, br_upd_i) && !(|(rf_u[p].pois & ld_fail_q_i)) &&
                   !(|(rf_u[p].poia & ld_fail_i));
      end
    end
  end

  always_comb begin
    for (int p = 0; p < N_STDP; p++) begin
      prf_raddr_o[p] = rf_u[p].psrc2;
      rf_d[p] = (rf_u[p].rs2_class == RC_NONE) ? 64'd0 : byp(rf_u[p].psrc2, prf_rdata_i[p], wb_i);
    end
  end

  always_ff @(posedge clk_i) begin
    for (int p = 0; p < N_STDP; p++) begin
      uop_t n;
      n = iss_uop_i[p]; n.br_mask = iss_uop_i[p].br_mask & ~br_upd_i.resolve_mask;
      rf_u[p] <= n;
      n = rf_u[p];      n.br_mask = rf_u[p].br_mask & ~br_upd_i.resolve_mask;
      ex_u[p]  <= n;
      ex_d0[p] <= rf_d[p];
    end
  end

  always_comb begin
    for (int p = 0; p < N_STDP; p++) begin
      std_valid_o[p] = ex_v[p] && !br_killed(ex_u[p].br_mask, br_upd_i);
      std_sq_o[p]    = ex_u[p].sq_idx;
      std_rob_o[p]   = ex_u[p].rob_idx;
      std_data_o[p]  = (ex_u[p].rs2_class == RC_NONE) ? 64'd0 : byp(ex_u[p].psrc2, ex_d0[p], wb_i);
    end
  end
endmodule
