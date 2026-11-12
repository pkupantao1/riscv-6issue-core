//==============================================================================
// o6_sram_sp_macro.sv - o6_sram_sp 的 ASIC 实现：按 (DEPTH, WIDTH) 拼 TSMC 40LP 编译器宏
//
// 只进 ASIC 流程 (DC / Formality / Innovus)，不进仿真与 FPGA 文件表。
// 宏的选择与面积 / 时序依据见 docs/05_sram_macros.md 与 docs/06_asic_flow.md：
//   (128, 256)  I$ 数据       2 x ts5n40lphsa128x128m2s   (1PRF)
//   (64, 1024)  D$ 数据 bank  8 x ts5n40lphsa64x128m2s    (1PRF，16 路 x 64 位)
//   (1024, 512) L2 数据       4 x ts1n40lpb1024x128m4m    (sp299，2 拍访问)
//   (1024, 216) L2 元数据     3 x ts1n40lpb1024x72m4m     (sp299，2 拍访问)
//   (512,  44)  TAGE 表一个体 1 x ts5n40lphsa512x44m4s    (1PRF，16 个)
//   (256,  51)  ITTAGE 表     1 x ts5n40lphsa256x52m2s    (1PRF，4 个，最高位不用)
//   (2048, 512) L2 数据 (1 MB) 4 x ts1n40lpb2048x128m4m    (sp299，2 拍访问)
//   (2048, 208) L2 元数据      3 x ts1n40lpb2048x72m4m     (sp299，高 8 位不用)
// 极性：宏的 CEB / WEB / BWEB 低有效。固定脚按数据手册接 (1PRF：TSEL=01 RTSEL=0 TURBO=1；
// sp299：RTSEL=WTSEL=01，BIST=0，AWT=0)。PD 接 0 (粗粒度掉电以后由电源管理接)。
// BIST 口暂时接死；MBIST 插入时 (Tessent) 由工具接管 *M 口。
//==============================================================================
module o6_sram_sp_macro #(
  parameter int DEPTH = 64,
  parameter int WIDTH = 64
) (
  input  wire                      clk_i,
  input  wire                      ce_i,
  input  wire                      we_i,
  input  wire  [$clog2(DEPTH)-1:0] addr_i,
  input  wire  [WIDTH-1:0]         wdata_i,
  input  wire  [WIDTH-1:0]         wmask_i,
  output logic [WIDTH-1:0]         rdata_o
);
  wire ceb = ~ce_i;
  wire web = ~we_i;

  if (DEPTH == 128 && WIDTH == 256) begin : g_128x256
    for (genvar i = 0; i < 2; i++) begin : g_m
      ts5n40lphsa128x128m2s u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wdata_i[128*i +: 128]), .BWEB(~wmask_i[128*i +: 128]), .Q(rdata_o[128*i +: 128]),
        .PD(1'b0), .TSEL(2'b01), .RTSEL(1'b0), .TURBO(1'b1)
      );
    end
  end else if (DEPTH == 64 && WIDTH % 128 == 0) begin : g_64xN
    for (genvar i = 0; i < WIDTH / 128; i++) begin : g_m
      ts5n40lphsa64x128m2s u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wdata_i[128*i +: 128]), .BWEB(~wmask_i[128*i +: 128]), .Q(rdata_o[128*i +: 128]),
        .PD(1'b0), .TSEL(2'b01), .RTSEL(1'b0), .TURBO(1'b1)
      );
    end
  end else if (DEPTH == 1024 && WIDTH == 512) begin : g_1024x512
    for (genvar i = 0; i < 4; i++) begin : g_m
      ts1n40lpb1024x128m4m u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wdata_i[128*i +: 128]), .BWEB(~wmask_i[128*i +: 128]), .Q(rdata_o[128*i +: 128]),
        .PD(1'b0), .RTSEL(2'b01), .WTSEL(2'b01),
        .BIST(1'b0), .CEBM(1'b1), .WEBM(1'b1), .AM('0), .DM('0), .BWEBM('1), .AWT(1'b0)
      );
    end
  end else if (DEPTH == 1024 && WIDTH == 216) begin : g_1024x216
    for (genvar i = 0; i < 3; i++) begin : g_m
      ts1n40lpb1024x72m4m u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wdata_i[72*i +: 72]), .BWEB(~wmask_i[72*i +: 72]), .Q(rdata_o[72*i +: 72]),
        .PD(1'b0), .RTSEL(2'b01), .WTSEL(2'b01),
        .BIST(1'b0), .CEBM(1'b1), .WEBM(1'b1), .AM('0), .DM('0), .BWEBM('1), .AWT(1'b0)
      );
    end
  end else if (DEPTH == 2048 && WIDTH == 512) begin : g_2048x512
    // L2 数据 (1 MB：每路 2048 x 512)
    for (genvar i = 0; i < 4; i++) begin : g_m
      ts1n40lpb2048x128m4m u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wdata_i[128*i +: 128]), .BWEB(~wmask_i[128*i +: 128]), .Q(rdata_o[128*i +: 128]),
        .PD(1'b0), .RTSEL(2'b01), .WTSEL(2'b01),
        .BIST(1'b0), .CEBM(1'b1), .WEBM(1'b1), .AM('0), .DM('0), .BWEBM('1), .AWT(1'b0)
      );
    end
  end else if (DEPTH == 2048 && WIDTH == 208) begin : g_2048x208
    // L2 元数据 (3 x 72 = 216 位，高 8 位不用)
    logic [215:0] wd, wm, rd;
    assign wd = {8'b0, wdata_i};
    assign wm = {8'b0, wmask_i};
    assign rdata_o = rd[207:0];
    for (genvar i = 0; i < 3; i++) begin : g_m
      ts1n40lpb2048x72m4m u_m (
        .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
        .D(wd[72*i +: 72]), .BWEB(~wm[72*i +: 72]), .Q(rd[72*i +: 72]),
        .PD(1'b0), .RTSEL(2'b01), .WTSEL(2'b01),
        .BIST(1'b0), .CEBM(1'b1), .WEBM(1'b1), .AM('0), .DM('0), .BWEBM('1), .AWT(1'b0)
      );
    end
  end else if (DEPTH == 512 && WIDTH == 44) begin : g_512x44
    // TAGE 表的一个体 (o6_tage：8 表 x 2 体)
    ts5n40lphsa512x44m4s u_m (
      .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
      .D(wdata_i), .BWEB(~wmask_i), .Q(rdata_o),
      .PD(1'b0), .TSEL(2'b01), .RTSEL(1'b0), .TURBO(1'b1)
    );
  end else if (DEPTH == 256 && WIDTH == 51) begin : g_256x51
    // ITTAGE 表 (o6_ittage：4 张)，宏 52 位，最高位不用
    logic [51:0] rd;
    ts5n40lphsa256x52m2s u_m (
      .CLK(clk_i), .CEB(ceb), .WEB(web), .A(addr_i),
      .D({1'b0, wdata_i}), .BWEB(~{1'b0, wmask_i}), .Q(rd),
      .PD(1'b0), .TSEL(2'b01), .RTSEL(1'b0), .TURBO(1'b1)
    );
    assign rdata_o = rd[50:0];
  end else begin : g_unmapped
    // 新增尺寸要在这里加映射；elaborate 时直接报错
    initial $fatal(1, "o6_sram_sp_macro: no macro mapping for %0dx%0d", DEPTH, WIDTH);
  end
endmodule
