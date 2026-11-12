// 向量单元空壳 (O6_NO_VEC：FPGA 不搭载 RVV 时用，2026-10-22)
// 端口与 o6_vu 相同。任何向量指令 (含 vsetvl / 向量 CSR) 到 ROB 头交过来，下一拍回非法指令异常；
// 不发向量访存，始终空闲。misa.V 在 o6_csr 里同样按 O6_NO_VEC 清零，软件不会用向量。
`default_nettype none
module o6_vu_stub
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 req_v_i,
  input  wire  [31:0]         req_inst_i,
  input  wire  [63:0]         req_a_i,
  input  wire  [63:0]         req_b_i,
  output logic                rsp_v_o,
  output logic [63:0]         rsp_data_o,
  output logic                rsp_excp_o,
  output logic [5:0]          rsp_cause_o,
  output logic [63:0]         rsp_tval_o,
  output logic                vm_req_o,
  input  wire                 vm_ack_i,
  output logic                vm_we_o,
  output logic [63:0]         vm_va_o,
  output logic [511:0]        vm_wdata_o,
  output logic [63:0]         vm_wmask_o,
  input  wire                 vm_done_i,
  input  wire  [511:0]        vm_rdata_i,
  input  wire                 vm_fault_i,
  input  wire  [5:0]          vm_cause_i,
  input  wire  [2:0]          frm_i,
  output logic [4:0]          fflags_o,
  output logic                vs_dirty_o,
  output logic                idle_o
);
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) rsp_v_o <= 1'b0;
    else         rsp_v_o <= req_v_i;
  always_ff @(posedge clk_i) rsp_tval_o <= {32'b0, req_inst_i};
  assign rsp_data_o  = '0;
  assign rsp_excp_o  = 1'b1;
  assign rsp_cause_o = 6'd2;          // 非法指令
  assign vm_req_o    = 1'b0;
  assign vm_we_o     = 1'b0;
  assign vm_va_o     = '0;
  assign vm_wdata_o  = '0;
  assign vm_wmask_o  = '0;
  assign fflags_o    = '0;
  assign vs_dirty_o  = 1'b0;
  assign idle_o      = !rsp_v_o;
endmodule
`default_nettype wire
