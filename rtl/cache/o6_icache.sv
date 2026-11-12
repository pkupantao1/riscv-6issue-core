//==============================================================================
// o6_icache.sv - L1 指令缓存：64 KB / 16 路 / 64 B 行 / VIPT / 阻塞式缺失
//
// 取指单位 32 B (半行)。数据 SRAM 每路一块 128 x 256 (64 组 x 2 半行)，16 路并读后按命中路选。
// 标签 / 有效：触发器。请求拍按组号把那一组的 16 个标签 / 有效位读出寄存，下一拍 (与 SRAM 读出同拍) 只做比较。
//
// 时序：t 拍请求 -> SRAM 读 + 比标签 -> t+1 拍 resp_valid + 数据 (命中)
//       缺失：t+1 发下游请求 -> 回填两拍写 SRAM (两个半行) -> 重读 -> 响应
// kill：前端重定向时置 1，丢弃当前在途请求的响应 (回填照常完成)。
// fence.i：invalidate 全部有效位。
//==============================================================================
`default_nettype none
module o6_icache
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,

  input  wire                 req_valid_i,
  output wire                 req_ready_o,
  input  wire  [PA_W-1:0]     req_addr_i,       // 32 B 对齐
  input  wire                 kill_i,
  output logic                resp_valid_o,
  output logic [255:0]        resp_data_o,

  input  wire                 inval_i,          // fence.i

  output logic                mem_req_valid_o,
  input  wire                 mem_req_ready_i,
  output logic [PA_W-1:6]     mem_req_addr_o,
  input  wire                 mem_resp_valid_i,
  input  wire  [511:0]        mem_resp_data_i,
  // MBIST (各 SRAM 的 March C− 引擎；done 全与、fail 全或)
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int NSET = 64;
  localparam int NWAY = L1_NWAY;
  localparam int WB   = L1_WB;
  localparam int TW   = PA_W - 12;

  logic [TW-1:0]   tag   [NSET][NWAY];
  logic [NWAY-1:0] valid [NSET];
  logic [NWAY-2:0] plru  [NSET];

  typedef enum logic [2:0] {I_IDLE, I_LOOK, I_MREQ, I_MWAIT, I_FILL0, I_FILL1, I_REREAD, I_NC} st_e;
  st_e             st;
  logic [PA_W-1:0] a_q;        // 在途请求地址
  logic            dead;       // 在途请求已被 kill
  logic [WB-1:0]   vway;
  logic [511:0]    line;

  // SRAM
  logic [NWAY-1:0] s_ce, s_we;
  logic [6:0]      s_addr;
  logic [255:0]    s_wdata;
  logic [255:0]    s_rdata [NWAY];
  logic [NWAY-1:0] bd, bf;
  for (genvar w = 0; w < NWAY; w++) begin : g_way
    o6_sram_sp #(.DEPTH(128), .WIDTH(256)) u_data (
      .clk_i, .ce_i(s_ce[w]), .we_i(s_we[w]), .addr_i(s_addr), .wdata_i(s_wdata),
      .wmask_i({256{1'b1}}), .rdata_o(s_rdata[w]),
      .bist_start_i, .bist_done_o(bd[w]), .bist_fail_o(bf[w])
    );
  end
  assign bist_done_o = &bd;
  assign bist_fail_o = |bf;


  // 接受新请求：空闲，或命中响应当拍 (流水)
  logic hit_now;
  assign req_ready_o = (st == I_IDLE) || (st == I_LOOK && hit_now);
  wire  take = req_valid_i && req_ready_o && !inval_i;

  // 组的标签 / 有效位预读 (2026-10-25，6 GHz：原来 I_LOOK 拍 "64 选 1 选组 -> 比较 -> 编码路号 -> 16 选 1 数据"
  // 串在一拍，b180 out 189 ps)。进 I_LOOK 只有两条路：take (按请求地址) 与 I_REREAD (按 a_q)，就在那拍读出寄存。
  // 两拍之间标签只在 I_FILL1 改 (不会夹在中间)；有效位还会被 fence.i 清：take 已排除 inval，I_REREAD 拍遇到就寄存全 0。
  logic [TW-1:0]   trow_q [NWAY];
  logic [NWAY-1:0] vrow_q;
  wire  [5:0]      cap_set = (st == I_REREAD) ? a_q[11:6] : req_addr_i[11:6];
  always_ff @(posedge clk_i)
    if (take || st == I_REREAD) begin
      for (int w = 0; w < NWAY; w++) trow_q[w] <= tag[cap_set][w];
      vrow_q <= inval_i ? '0 : valid[cap_set];
    end

  // I_LOOK 拍：寄存的组与 a_q 的标签比较；数据按独热命中直接与或选择
  logic [NWAY-1:0] hit_oh;
  logic [WB-1:0]   hit_way;
  logic [255:0]    hit_data;
  always_comb begin
    hit_oh = '0; hit_way = '0; hit_data = '0;
    for (int w = 0; w < NWAY; w++)
      if (vrow_q[w] && trow_q[w] == a_q[PA_W-1:12]) begin hit_oh[w] = 1'b1; hit_way = WB'(w); end
    for (int w = 0; w < NWAY; w++) hit_data = hit_data | ({256{hit_oh[w]}} & s_rdata[w]);
    hit_now = |hit_oh;
  end
`ifndef SYNTHESIS
  // 与原写法 (I_LOOK 拍现读阵列) 一致；同一组不会有两路同标签
  always_ff @(posedge clk_i) if (rst_ni && st == I_LOOK) begin
    logic [NWAY-1:0] ro;
    for (int w = 0; w < NWAY; w++) ro[w] = valid[a_q[11:6]][w] && tag[a_q[11:6]][w] == a_q[PA_W-1:12];
    assert (ro == hit_oh) else $error("icache: 预读的组命中 %h 与现读 %h 不一致", hit_oh, ro);
    assert ($onehot0(hit_oh)) else $error("icache: 多路命中 %h", hit_oh);
  end
`endif

  always_comb begin
    s_ce = '0; s_we = '0; s_addr = '0; s_wdata = '0;
    if (st == I_FILL0 || st == I_FILL1) begin
      s_ce[vway] = 1'b1; s_we[vway] = 1'b1;
      s_addr  = {a_q[11:6], (st == I_FILL1)};
      s_wdata = (st == I_FILL1) ? line[511:256] : line[255:0];
    end else if (st == I_REREAD) begin
      s_ce = '1; s_addr = {a_q[11:6], a_q[5]};
    end else if (st == I_IDLE || st == I_LOOK) begin
      // 空闲 / 命中拍总按请求地址读 (不看 take：req_valid 来自前端很深的逻辑，别让它进 SRAM 地址/片选)
      s_ce = '1; s_addr = {req_addr_i[11:6], req_addr_i[5]};
    end
  end

  // 不可缓存区 (调试模块 0x000-0xFFF：内容随时会变)：行回来直接给出，不写阵列
  wire   nc = (a_q[PA_W-1:12] == '0);
  assign resp_valid_o = ((st == I_LOOK) && hit_now || (st == I_NC)) && !dead && !kill_i;
  assign resp_data_o  = (st == I_NC) ? (a_q[5] ? line[511:256] : line[255:0]) : hit_data;

  assign mem_req_valid_o = (st == I_MREQ);
  assign mem_req_addr_o  = a_q[PA_W-1:6];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st <= I_IDLE; dead <= 1'b0;
      for (int s = 0; s < NSET; s++) begin valid[s] <= '0; plru[s] <= '0; end
    end else begin
      if (kill_i) dead <= 1'b1;
      unique case (st)
        I_IDLE: if (take) begin st <= I_LOOK; a_q <= req_addr_i; dead <= 1'b0; end
        I_LOOK: begin
          if (hit_now) begin
            plru[a_q[11:6]] <= l1_plru_touch(plru[a_q[11:6]], hit_way);
            if (take) begin a_q <= req_addr_i; dead <= 1'b0; end
            else st <= I_IDLE;
          end else begin
            st <= I_MREQ;
          end
        end
        I_MREQ:  if (mem_req_ready_i) st <= I_MWAIT;
        I_MWAIT: if (mem_resp_valid_i) begin
          line <= mem_resp_data_i;
          vway <= l1_plru_victim(plru[a_q[11:6]]);
          st   <= nc ? I_NC : I_FILL0;
        end
        I_NC: st <= I_IDLE;
        I_FILL0: begin
          valid[a_q[11:6]][vway] <= 1'b0;     // 写数据期间先置无效
          st <= I_FILL1;
        end
        I_FILL1: begin
          tag[a_q[11:6]][vway]   <= a_q[PA_W-1:12];
          valid[a_q[11:6]][vway] <= 1'b1;
          st <= dead ? I_IDLE : I_REREAD;
        end
        I_REREAD: st <= I_LOOK;
        default: st <= I_IDLE;
      endcase
      if (inval_i) for (int s = 0; s < NSET; s++) valid[s] <= '0;
    end
  end
endmodule
