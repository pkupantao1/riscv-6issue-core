// fv_brmask.sv - 形式证明 F5：分支解析 / 误预测杀之后发射队列里不留该标签 (docs/13 §5.4「br_mask 广播后清零」；bug #5 同类)
//   被测 o6_iq (SIZE 8、NP 2：选择逻辑与真实实例同一份代码，只是项数小)，全部输入自由
//   性质 (每项 i)：
//     ast_res ：本拍解析的标签，下一拍任何有效项的 br_mask 里都没有 (含本拍入队的、本拍被改写的槽)
//     ast_kill：本拍误预测杀 (且不冲刷)，下一拍没有有效项带被杀标签 (老项被杀、本拍入队的带杀标签的不入队)
//     ast_fl  ：冲刷后下一拍队列为空
//   cover：被杀项当拍仍被选中发射 (规格允许)、入队与杀同拍
`default_nettype none
module fv_brmask
  import o6_pkg::*;
#(parameter int SIZE = 8, parameter int NP = 2) (
  input  wire                       clk,
  input  wire                       rst_n,
  input  wire                       flush,
  input  var br_update_t            bu,
  input  wire  [IQ_ENQ_MAX-1:0]     enq_v,
  input  var uop_t [IQ_ENQ_MAX-1:0] enq_u,
  input  var wakeup_t [N_IWAKE-1:0] iwake,
  input  var wakeup_t [N_FWAKE-1:0] fwake,
  input  var wakeup_t [N_CWAKE-1:0] cwake,
  input  var wakeup_t [N_LDP-1:0]   ewake,
  input  var lwake_t  [N_LDP-1:0]   lwake,
  input  wire  [N_LDP-1:0]          ld_fail,
  input  var rob_ptr_t              rob_head, head2,
  input  wire                       head_go, head2_go,
  input  wire  [15:0]               fu_ok0, fu_ok1
);
  logic [$clog2(SIZE+1)-1:0] free_cnt;
  logic [NP-1:0] iss_v;
  uop_t [NP-1:0] iss_u;
  logic [15:0]   fu_ok [NP];
  assign fu_ok[0] = fu_ok0;
  assign fu_ok[NP-1] = fu_ok1;

  o6_iq #(.SIZE(SIZE), .NP(NP), .PBASE(2)) dut (
    .clk_i(clk), .rst_ni(rst_n), .flush_i(flush), .br_upd_i(bu),
    .enq_valid_i(enq_v), .enq_uop_i(enq_u), .free_cnt_o(free_cnt),
    .iwake_i(iwake), .fwake_i(fwake), .cwake_i(cwake), .ewake_i(ewake), .lwake_i(lwake), .ld_fail_i(ld_fail),
    .rob_head_i(rob_head), .head_go_i(head_go), .head2_i(head2), .head2_go_i(head2_go),
    .fu_ok_i(fu_ok), .iss_valid_o(iss_v), .iss_uop_o(iss_u));

  // 环境：入队不超过空位 (分派级保证；否则 RTL 断言报「入队时无空项」)
  asm_enq: assume property (@(posedge clk) $countones(enq_v) <= free_cnt);
  // 标签 6 位、MAX_BR = 48：越界的 kill_tag 取 br_mask[48..63] 在形式里是任意值 (RTL 与性质各取各的)，真实硬件不会广播
  asm_tag: assume property (@(posedge clk) bu.kill_tag < MAX_BR);
  asm_pfx: assume property (@(posedge clk) (enq_v & (enq_v + 1'b1)) == '0);   // 已压缩到前几个槽

  for (genvar i = 0; i < SIZE; i++) begin : g_e
    ast_res:  assert property (@(posedge clk) disable iff (!rst_n)
      1'b1 |=> (!dut.vld[i] || (dut.ent[i].br_mask & $past(bu.resolve_mask)) == '0));
    ast_kill: assert property (@(posedge clk) disable iff (!rst_n)
      (bu.kill && !flush) |=> (!dut.vld[i] || !dut.ent[i].br_mask[$past(bu.kill_tag)]));
    ast_fl:   assert property (@(posedge clk) disable iff (!rst_n) flush |=> !dut.vld[i]);
  end
  cov_kill_iss: cover property (@(posedge clk) disable iff (!rst_n)
    bu.kill && iss_v[0] && iss_u[0].br_mask[bu.kill_tag]);
  cov_enq_kill: cover property (@(posedge clk) disable iff (!rst_n)
    bu.kill && enq_v[0] && enq_u[0].br_mask[bu.kill_tag]);
endmodule
