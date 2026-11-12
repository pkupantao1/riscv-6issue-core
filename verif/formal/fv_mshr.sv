// fv_mshr.sv - 形式证明 F4：D$ 的 MSHR 无重复行 (docs/13 §5.4；bug #18 同类)
//   被测 o6_dcache 全部输入自由；标签 / 有效 / 脏 / PLRU 阵列在 tcl 里 stopat (每拍任意值)，数据 SRAM 黑盒：
//   MSHR 去重只靠 mshr_match 与同拍合并，不依赖命中结果是否正确，所以这是**放宽**的模型 —— 这里证明成立，真实设计必成立。
//   环境约束只有一条：下游回填只给处于 M_WAIT 的 MSHR (每个读请求恰好一次响应)
//   性质：
//     ast_uniq[i][j]：任意两个非空闲 MSHR 的行地址不同
//     ast_fill      ：回填写入拍 (F_WR) 的 MSHR 处于 M_FILL
//   cover：store 排空缺失与同行 load 缺失同拍 (#18 的场景)、两个以上 MSHR 同时在用
`default_nettype none
module fv_mshr
  import o6_pkg::*;
#(parameter int NMSHR = DC_NMSHR, parameter int NLD = N_LDP) (
  input  wire                 clk,
  input  wire                 rst_n,
  input  wire  [NLD-1:0]      ld_req,
  input  wire  [PA_W-1:0]     ld_addr  [NLD],
  input  wire  [PA_W-1:0]     ld_paddr [NLD],
  input  wire  [NLD-1:0]      ld_kill,
  input  wire                 st_req,
  input  wire  [PA_W-1:0]     st_paddr, st_next_paddr, st_lk_paddr,
  input  wire  [63:0]         st_data,
  input  wire  [7:0]          st_mask,
  input  wire                 st_amo,
  input  wire  [4:0]          st_amo_op,
  input  wire  [1:0]          st_size,
  input  wire                 st_line,
  input  wire  [511:0]        st_ldata,
  input  wire  [63:0]         st_lmask,
  input  wire                 pf_req,
  input  wire  [PA_W-1:0]     pf_paddr,
  input  wire                 flush_req,
  input  wire                 mem_req_ready,
  input  wire                 mem_resp_valid,
  input  wire  [3:0]          mem_resp_id,
  input  wire  [511:0]        mem_resp_data
);
  logic [NLD-1:0] ld_hit, ld_nack;
  logic [63:0]    ld_data [NLD];
  logic [MSHR_W-1:0] ld_mshr [NLD];
  logic refill, st_done, flush_done, mem_req_valid, mem_req_we, mem_resp_ready, bist_done, bist_fail;
  logic [MSHR_W-1:0] refill_mshr;
  logic [63:0]  st_rdata;
  logic [511:0] st_lrdata, mem_req_data;
  logic [PA_W-1:6] mem_req_addr;
  logic [3:0]   mem_req_id;

  o6_dcache #(.NMSHR(NMSHR), .NLD(NLD)) dut (
    .clk_i(clk), .rst_ni(rst_n),
    .ld_req_i(ld_req), .ld_addr_i(ld_addr), .ld_paddr_i(ld_paddr), .ld_kill_i(ld_kill),
    .ld_hit_o(ld_hit), .ld_nack_o(ld_nack), .ld_data_o(ld_data), .ld_mshr_o(ld_mshr),
    .refill_o(refill), .refill_mshr_o(refill_mshr),
    .st_req_i(st_req), .st_paddr_i(st_paddr), .st_next_paddr_i(st_next_paddr), .st_lk_paddr_i(st_lk_paddr),
    .st_data_i(st_data), .st_mask_i(st_mask), .st_amo_i(st_amo), .st_amo_op_i(st_amo_op), .st_size_i(st_size),
    .st_done_o(st_done), .st_rdata_o(st_rdata),
    .st_line_i(st_line), .st_ldata_i(st_ldata), .st_lmask_i(st_lmask), .st_lrdata_o(st_lrdata),
    .pf_req_i(pf_req), .pf_paddr_i(pf_paddr), .flush_req_i(flush_req), .flush_done_o(flush_done),
    .mem_req_valid_o(mem_req_valid), .mem_req_ready_i(mem_req_ready), .mem_req_we_o(mem_req_we),
    .mem_req_addr_o(mem_req_addr), .mem_req_data_o(mem_req_data), .mem_req_id_o(mem_req_id),
    .mem_resp_valid_i(mem_resp_valid), .mem_resp_ready_o(mem_resp_ready), .mem_resp_id_i(mem_resp_id),
    .mem_resp_data_i(mem_resp_data),
    .bist_start_i(1'b0), .bist_done_o(bist_done), .bist_fail_o(bist_fail));

  localparam logic [1:0] M_FREE = 2'd0, M_WAIT = 2'd2, M_FILL = 2'd3;   // 与 o6_dcache::mst_e 一致
  localparam logic [1:0] F_WR = 2'd2;

  // 环境：回填只给在等的 MSHR
  asm_resp: assume property (@(posedge clk) mem_resp_valid |-> (mem_resp_id < NMSHR && dut.m_st[mem_resp_id[MSHR_W-1:0]] == M_WAIT));
  // 环境：store 两拍协议 —— S_LOOK 动作拍 LSU 给的地址就是上拍查表的地址 (o6_dcache 末尾的仿真断言查的是 LSU 这条义务)
  localparam logic [2:0] S_LOOK = 3'd1;                                // 与 o6_dcache::sst_e 一致
  asm_st2: assume property (@(posedge clk) (dut.s_st == S_LOOK && st_req && !dut.fill_all && !dut.s_lkf_q && st_lk_paddr == dut.spa_q)
                            |-> st_paddr == dut.spa_q);

  for (genvar i = 0; i < NMSHR; i++) begin : g_i
    for (genvar j = i + 1; j < NMSHR; j++) begin : g_j
      ast_uniq: assert property (@(posedge clk) disable iff (!rst_n)
        !(dut.m_st[i] != M_FREE && dut.m_st[j] != M_FREE && dut.m_line[i] == dut.m_line[j]));
    end
  end
  ast_fill: assert property (@(posedge clk) disable iff (!rst_n) dut.f_st == F_WR |-> dut.m_st[dut.f_m] == M_FILL);

  // 可达性
  for (genvar k = 0; k < NLD; k++) begin : g_cov
    cov_st_ld_same: cover property (@(posedge clk) disable iff (!rst_n)
      dut.st_miss_alloc && dut.l1_v[k] && dut.l1_go[k] && !ld_kill[k] && ld_paddr[k][PA_W-1:6] == dut.spa_q[PA_W-1:6]);
  end
  cov_two:  cover property (@(posedge clk) disable iff (!rst_n) dut.m_st[0] != M_FREE && dut.m_st[1] != M_FREE);
  cov_pf:   cover property (@(posedge clk) disable iff (!rst_n) dut.pf_alloc);
  cov_fill: cover property (@(posedge clk) disable iff (!rst_n) dut.f_st == F_WR);
endmodule
