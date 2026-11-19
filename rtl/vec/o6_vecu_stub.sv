// 向量单元空壳 (O6_NO_VEC：FPGA 不搭载 RVV 时用，2026-10-22)
// 端口与 o6_vecu 相同。任何向量指令 (含 vsetvl / 向量 CSR) 到 ROB 头交过来，下一拍回非法指令异常；
// 不发向量访存，始终空闲。misa.V 在 o6_csr 里同样按 O6_NO_VEC 清零，软件不会用向量。
`default_nettype none
module o6_vecu_stub
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
  output logic                rsp_flush_o,
  input  wire                 vm_conf_i,
  input  wire                 vm_tce_i,
  output logic                vm_req_o,
  input  wire                 vm_ack_i,
  output logic                vm_we_o,
  output logic                vm_cont_o,
  output logic [63:0]         vm_va_o,
  output logic [VM_NL-1:0]    vm_ln_o,
  output logic [511:0]        vm_wdata_o [VM_NL],
  output logic [63:0]         vm_wmask_o [VM_NL],
  input  wire                 vm_done_i,
  input  wire  [511:0]        vm_rdata_i [VM_NL],
  input  wire                 vm_fault_i,
  input  wire  [1:0]          vm_flane_i,
  input  wire  [5:0]          vm_cause_i,
  input  wire  [2:0]          frm_i,
  output logic [4:0]          fflags_o,
  output logic                vs_dirty_o,
  output logic                idle_o,
  output logic                fast_ok_o,
  output logic [10:0]         vl_o,
  output logic [1:0]          sew_o,
  output logic [2:0]          lmul_o,
  input  var br_update_t      br_upd_i,
  input  wire                 flush_i,
  input  wire  [NVX-1:0]      vx_v_i,
  input  wire  [31:0]         vx_raw_i [NVX],
  input  wire  [1:0]          vx_sew_i [NVX],
  input  wire  [10:0]         vx_vl_i  [NVX],
  input  var vsrc_t           vx_v0_i  [NVX],
  input  var rob_ptr_t        vx_rob_i [NVX],
  input  var br_mask_t        vx_bm_i  [NVX],
  input  wire  [3:0]          vx_ng_i  [NVX],
  input  var vsrc_t [2:0][7:0] vx_sg_i [NVX],
  input  var preg_t [7:0]     vx_pg_i  [NVX],
  input  wire  [63:0]         vx_xs_i  [NVX],
  output logic [NVX-1:0]      vx_rdy_o,
  input  wire  [3:0]          vx_ns_i  [NVX],
  input  wire  [2:0]          vx_lm_i  [NVX],
  input  wire                 vx_alt_i [NVX],
  output logic [NVX-1:0]      vx_dok_o,
  output logic [NVX-1:0]      vxw_v_o,
  output preg_t               vxw_p_o  [NVX],
  output cmpl_t               vx_cmpl_o [NVX],
  input  wire  [N_VPREG-1:0]  vbusy_i,
  input  wire                 vs_v_i,
  input  wire  [31:0]         vs_raw_i,
  input  var vsrc_t [3:0]     vs_src_i,
  input  var rob_ptr_t        vs_rob_i,
  input  var br_mask_t        vs_bm_i,
  input  var sq_ptr_t         vs_sq_i,
  input  wire  [2:0]          vs_vk_i,
  output logic [CL_B-1:0]     vsd_bm_o,
  output logic                vsd_v_o,
  output sq_ptr_t             vsd_sq_o,
  output rob_ptr_t            vsd_rob_o,
  output logic [CL_W-1:0]     vsd_d_o,
  input  wire                 cv_mk_i,
  input  var preg_t           cv_pd_i,
  input  wire  [4:0]          cv_vd_i,
  input  wire  [3:0]          cv_ng_i,
  input  var preg_t [7:0]     cv_pg_i,
  output logic                vmok_o,
  output logic                valt_o,
  output logic [9:0]          vt_o,
  input  wire  [N_LDP-1:0]    vlw_v_i,
  input  var preg_t [N_LDP-1:0] vlw_pd_i,
  input  wire  [CL_W-1:0]     vlw_d_i [N_LDP],
  input  wire  [CL_B-1:0]     vlw_bm_i [N_LDP],
  input  wire  [N_LDP-1:0]    vlw_ov_i,
  input  var vsrc_t [N_LDP-1:0] vlw_old_i,
  input  wire  [N_LDP-1:0]    vlw_mk_i,
  input  var vsrc_t [N_LDP-1:0] vlw_m0_i,
  input  wire  [2:0]          vlw_k_i [N_LDP],
  input  wire  [1:0]          vlw_eb_i [N_LDP],
  output logic [W-1:0]        vfr_v_o,
  output preg_t [W-1:0]       vfr_p_o,
  output logic [31:0]         vwpend_o,
  input  wire                 cv_v_i,
  input  wire  [31:0]         cv_inst_i,
  input  wire  [63:0]         cv_a_i,
  output logic                cv_acc_o,
  output logic                cv_ill_o,
  input  wire                 cv_go_i,
  input  wire  [NVC-1:1]      cvx_v_i,
  input  wire  [31:0]         cvx_inst_i [NVC],
  input  var preg_t           cvx_pd_i   [NVC],
  input  wire  [4:0]          cvx_vd_i   [NVC],
  input  wire  [3:0]          cvx_ng_i   [NVC],
  input  var preg_t [7:0]     cvx_pg_i   [NVC],
  output logic [NVC-1:1]      cvx_acc_o,
  input  wire  [2:0]          cv_gon_i,
  input  var sq_ptr_t         cv_sqb_i,
  output logic                vm_async_o,
  output sq_ptr_t             vm_sqb_o,
  output logic                vbar_v_o,
  output sq_ptr_t             vbar_o,
  output logic                vasync_o
);
  assign cv_acc_o  = 1'b0;            // 提交口 (I1.3b / I2a)：一律非法
  assign cv_ill_o  = cv_v_i;
  assign cvx_acc_o = '0;
  assign vm_async_o = 1'b0;           // 没有异步 load (I2a)
  assign vm_sqb_o   = '0;
  assign vbar_v_o   = 1'b0;
  assign vbar_o     = '0;
  assign vasync_o   = 1'b0;
  assign fast_ok_o = 1'b0;            // 从不分乱序候选 (向量指令一律非法)
  assign vl_o = '0; assign sew_o = '0; assign lmul_o = '0;
  assign vxw_v_o   = '0;              // 没有 VX / 切换标记 / 回收 / 在途写 (O1)
  for (genvar x = 0; x < NVX; x++) begin : g_x assign vxw_p_o[x] = '0; assign vx_cmpl_o[x] = '0; end
  assign vsd_v_o   = 1'b0;
  assign vsd_sq_o  = '0;
  assign vsd_rob_o = '0;
  assign vsd_d_o   = '0;
  assign vsd_bm_o  = '0;
  assign vx_rdy_o  = '1;
  assign vx_dok_o  = '1;
  assign vfr_v_o   = '0;
  assign vfr_p_o   = '0;
  assign vwpend_o  = '0;
  assign vmok_o    = 1'b0;            // M3：不展开向量 load
  assign valt_o     = 1'b0;
  assign vt_o       = 10'h200;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) rsp_v_o <= 1'b0;
    else         rsp_v_o <= req_v_i;
  always_ff @(posedge clk_i) rsp_tval_o <= {32'b0, req_inst_i};
  assign rsp_data_o  = '0;
  assign rsp_excp_o  = 1'b1;
  assign rsp_flush_o = 1'b0;
  assign rsp_cause_o = 6'd2;          // 非法指令
  assign vm_req_o    = 1'b0;
  assign vm_we_o     = 1'b0;
  assign vm_cont_o   = 1'b0;
  assign vm_va_o     = '0;
  assign vm_ln_o     = '0;
  always_comb for (int j = 0; j < VM_NL; j++) begin vm_wdata_o[j] = '0; vm_wmask_o[j] = '0; end
  assign fflags_o    = '0;
  assign vs_dirty_o  = 1'b0;
  assign idle_o      = !rsp_v_o;
endmodule
`default_nettype wire
