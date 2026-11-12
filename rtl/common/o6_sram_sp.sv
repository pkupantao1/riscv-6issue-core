//==============================================================================
// o6_sram_sp.sv - 单口 SRAM 封装 (1RW，同步读，1 拍延迟)
//
// 行为与 TSMC 40LP 编译器宏一致 (见 docs/05_sram_macros.md §3.1)：
//   - ce=1, we=0：下一拍 rdata = mem[addr]
//   - ce=1, we=1：按 wmask 逐位写；rdata 保持上一次读出的值 (不写穿)
//   - ce=0     ：rdata 保持
//
// 实现选择：
//   O6_ASIC：例化 o6_sram_sp_macro (由宏生成脚本按 DEPTH/WIDTH 拼宏，见 syn/sram/)
//   O6_FPGA：推断 BRAM/URAM，写模式 NO_CHANGE (正好对应"写时输出保持")
//   仿真     ：行为模型；未写过的地址读出 X，方便发现读未初始化
//
// MBIST：内置 o6_mbist (March C−)，bist_start_i 拉高后接管端口；done/fail 由外层汇总到 JTAG TDR。
//==============================================================================
`default_nettype none
module o6_sram_sp #(
  parameter int DEPTH = 64,
  parameter int WIDTH = 64,
  parameter     STYLE = "block"        // FPGA：block / ultra / distributed
) (
  input  wire                      clk_i,
  input  wire                      ce_i,
  input  wire                      we_i,
  input  wire  [$clog2(DEPTH)-1:0] addr_i,
  input  wire  [WIDTH-1:0]         wdata_i,
  input  wire  [WIDTH-1:0]         wmask_i,
  output logic [WIDTH-1:0]         rdata_o,
  // MBIST
  input  wire                      bist_start_i,
  output logic                     bist_done_o,
  output logic                     bist_fail_o
);
  // ---- MBIST 引擎与端口切换
  logic                     b_act, b_ce, b_we;
  logic [$clog2(DEPTH)-1:0] b_addr;
  logic [WIDTH-1:0]         b_wdata;
  o6_mbist #(.DEPTH(DEPTH), .WIDTH(WIDTH)) u_bist (
    .clk_i, .start_i(bist_start_i), .active_o(b_act), .done_o(bist_done_o), .fail_o(bist_fail_o),
    .ce_o(b_ce), .we_o(b_we), .addr_o(b_addr), .wdata_o(b_wdata), .rdata_i(rdata_o)
  );
  wire                     m_ce    = b_act ? b_ce    : ce_i;
  wire                     m_we    = b_act ? b_we    : we_i;
  wire [$clog2(DEPTH)-1:0] m_addr  = b_act ? b_addr  : addr_i;
  wire [WIDTH-1:0]         m_wdata = b_act ? b_wdata : wdata_i;
  wire [WIDTH-1:0]         m_wmask = b_act ? '1      : wmask_i;

`ifdef O6_ASIC
  o6_sram_sp_macro #(.DEPTH(DEPTH), .WIDTH(WIDTH)) u_macro (
    .clk_i, .ce_i(m_ce), .we_i(m_we), .addr_i(m_addr), .wdata_i(m_wdata), .wmask_i(m_wmask), .rdata_o
  );
`elsif O6_FPGA
  // Vivado 只认字节写使能：取每字节最低位作为该字节的使能 (缓存数据都是字节粒度)
  localparam int NB = (WIDTH + 7) / 8;
  (* ram_style = STYLE *) logic [NB*8-1:0] mem [DEPTH];
  logic [NB*8-1:0] wd;
  logic [NB-1:0]   be;
  always_comb begin
    wd = '0;
    wd[WIDTH-1:0] = m_wdata;
    for (int i = 0; i < NB; i++) be[i] = m_wmask[(8*i < WIDTH) ? 8*i : WIDTH-1];
  end
  always_ff @(posedge clk_i) begin
    if (m_ce) begin
      for (int i = 0; i < NB; i++)
        if (m_we && be[i]) mem[m_addr][8*i +: 8] <= wd[8*i +: 8];
      if (!m_we) rdata_o <= mem[m_addr][WIDTH-1:0];
    end
  end
`else
  logic [WIDTH-1:0] mem [DEPTH];

  always_ff @(posedge clk_i) begin
    if (m_ce) begin
      if (m_we) begin
        for (int b = 0; b < WIDTH; b++)
          if (m_wmask[b]) mem[m_addr][b] <= m_wdata[b];
      end else begin
        rdata_o <= mem[m_addr];
      end
    end
  end
`endif
endmodule
