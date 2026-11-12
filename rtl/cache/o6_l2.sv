//==============================================================================
// o6_l2.sv - L2 缓存：1 MB / 8 路 / 2048 组 / 64 B 行 / 写回 + 写分配 / 与 L1 非包含 / 非阻塞
//
// 上游：核的行请求口 (读 = 取行，写 = L1 写回整行)；下游：同格式的行请求口 (接 o6_line2axi)。
//
// 非阻塞 (2026-10-14，原来一次处理一个请求、缺失期间不接新请求)：
//   一个控制 FSM 只负责 SRAM 操作 (查标签、读写数据 / 元数据)，从不等下游：
//     读命中   -> 读数据 -> 推响应
//     读缺失   -> 分配 L2 MSHR (记行地址、上游请求号)，由 MSHR 向下游发读 (请求号 = MSHR 号)，FSM 立即空闲
//     写命中   -> 写数据、置脏
//     写缺失   -> 选牺牲路 (脏则读出进写回缓冲) -> 写新行 (脏)
//     回填任务 -> 下游数据回来后 (MSHR 进 M_FILL)，FSM 优先处理：查该组元数据、选牺牲路 (脏则写回)、
//                写数据与元数据、推响应 (带原上游请求号)、释放 MSHR
//   同一行的规则：
//     读请求的行已有 MSHR (如 I$ 与 D$ 先后缺同一行)：不接收，等那个 MSHR 完成
//     写请求 (L1 写回整行) 的行已有 MSHR：直接写进该 MSHR 的数据缓冲并标脏 (覆盖下游数据)，不走 SRAM。
//       不能让写等 MSHR：MSHR 完成要往响应 FIFO 推数据，而 D$ 有待发写回时不收响应 —— 会互相卡死
//   下游顺序：写回缓冲优先于 MSHR 的读发出；读写同一行时由 o6_line2axi 保证读等写完成
//
// 存储：
//   标签 + 元数据：一块 NSET x MW 单口 SRAM (每路 {有效, 脏, TW 位标签} x 8 + 7 位 PLRU + 1 位保留)
//   数据：每路一块 NSET x 512 单口 SRAM (ASIC 上由 sp299 宏拼成，见 docs/05_sram_macros.md)
// SRAM 读延迟 RD_LAT 拍 (FPGA 1；ASIC 40LP 下 sp299 宏要 2 拍，对应多周期约束)。
//
// 防死锁：响应 FIFO 满时只拒收读请求，写回请求照收 (上游 D$ 在有待发写回时不收响应，必须先让写回进来)。
//==============================================================================
`default_nettype none
module o6_l2
  import o6_pkg::*;
#(
  parameter int RD_LAT = 1,
  parameter int NM     = 8           // L2 MSHR 数 (下游请求号 0..NM-1)
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 上游
  input  wire                 up_req_valid_i,
  output logic                up_req_ready_o,
  input  wire                 up_req_we_i,
  input  wire  [PA_W-1:6]     up_req_addr_i,
  input  wire  [511:0]        up_req_data_i,
  input  wire  [3:0]          up_req_id_i,
  output logic                up_resp_valid_o,
  input  wire                 up_resp_ready_i,
  output logic [3:0]          up_resp_id_o,
  output logic [511:0]        up_resp_data_o,

  // 下游
  output logic                dn_req_valid_o,
  input  wire                 dn_req_ready_i,
  output logic                dn_req_we_o,
  output logic [PA_W-1:6]     dn_req_addr_o,
  output logic [511:0]        dn_req_data_o,
  output logic [3:0]          dn_req_id_o,
  input  wire                 dn_resp_valid_i,
  output logic                dn_resp_ready_o,
  input  wire  [3:0]          dn_resp_id_i,
  input  wire  [511:0]        dn_resp_data_i,
  // MBIST (各 SRAM 的 March C− 引擎；done 全与、fail 全或)
  input  wire                 bist_start_i,
  output logic                bist_done_o,
  output logic                bist_fail_o
);
  localparam int NSET = 2048;
  localparam int NWAY = 8;
  localparam int SW   = $clog2(NSET);  // 组号宽度 (11)
  localparam int TW   = PA_W - 6 - SW; // 标签宽度 (23)
  localparam int EW   = TW + 2;        // 每路元数据
  localparam int MW   = NWAY * EW + 8; // 208
  localparam int MIW  = $clog2(NM);
  localparam int FQD  = 4;             // 响应 FIFO 深度

  //--------------------------------------------------------------------------
  // SRAM
  //--------------------------------------------------------------------------
  logic            m_ce, m_we;
  logic [SW-1:0]   m_addr;
  logic [MW-1:0]   m_wdata, m_rdata;
  logic md, mf;
  o6_sram_sp #(.DEPTH(NSET), .WIDTH(MW), .STYLE("block")) u_meta (
    .clk_i, .ce_i(m_ce), .we_i(m_we), .addr_i(m_addr), .wdata_i(m_wdata), .wmask_i({MW{1'b1}}), .rdata_o(m_rdata),
    .bist_start_i, .bist_done_o(md), .bist_fail_o(mf)
  );

  logic [NWAY-1:0] d_ce, d_we;
  logic [SW-1:0]   d_addr;
  logic [511:0]    d_wdata;
  logic [511:0]    d_rdata [NWAY];
  logic [NWAY-1:0] dd, df;
  for (genvar w = 0; w < NWAY; w++) begin : g_way
    o6_sram_sp #(.DEPTH(NSET), .WIDTH(512), .STYLE("ultra")) u_data (
      .clk_i, .ce_i(d_ce[w]), .we_i(d_we[w]), .addr_i(d_addr), .wdata_i(d_wdata), .wmask_i({512{1'b1}}),
      .rdata_o(d_rdata[w]),
      .bist_start_i, .bist_done_o(dd[w]), .bist_fail_o(df[w])
    );
  end
  assign bist_done_o = md && (&dd);
  assign bist_fail_o = mf || (|df);

  //--------------------------------------------------------------------------
  // 元数据解包
  //--------------------------------------------------------------------------
  function automatic logic [TW-1:0] e_tag(input logic [MW-1:0] m, input int w); return m[EW*w +: TW]; endfunction
  function automatic logic e_v(input logic [MW-1:0] m, input int w); return m[EW*w + TW]; endfunction
  function automatic logic e_d(input logic [MW-1:0] m, input int w); return m[EW*w + TW + 1]; endfunction

  function automatic logic [2:0] plru_victim(input logic [6:0] t);
    logic [2:0] v;
    v[2] = t[0];
    if (!t[0]) begin v[1] = t[1]; v[0] = t[1] ? t[4] : t[3]; end
    else       begin v[1] = t[2]; v[0] = t[2] ? t[6] : t[5]; end
    return v;
  endfunction
  function automatic logic [6:0] plru_touch(input logic [6:0] t, input logic [2:0] w);
    logic [6:0] n;
    n = t;
    n[0] = ~w[2];
    if (!w[2]) begin n[1] = ~w[1]; if (!w[1]) n[3] = ~w[0]; else n[4] = ~w[0]; end
    else       begin n[2] = ~w[1]; if (!w[1]) n[5] = ~w[0]; else n[6] = ~w[0]; end
    return n;
  endfunction

  //--------------------------------------------------------------------------
  // MSHR
  //--------------------------------------------------------------------------
  typedef enum logic [1:0] {M_FREE, M_REQ, M_WAIT, M_FILL} mst_e;
  mst_e            ms_st  [NM];
  logic [PA_W-1:6] ms_line[NM];
  logic [3:0]      ms_id  [NM];       // 上游请求号
  logic [511:0]    ms_data[NM];
  logic [NM-1:0]   ms_ovr;            // 上游写回已覆盖数据 (下游数据作废；回填为脏)

  // 写回缓冲 (1 项：牺牲的脏行)
  logic            wb_v;
  logic [PA_W-1:6] wb_line;
  logic [511:0]    wb_data;

  // 上游请求与 MSHR 的行比较
  logic [NM-1:0] up_hit_m;
  logic          up_conf, ms_free_v;
  logic [MIW-1:0] up_m, free_m;
  always_comb begin
    up_hit_m = '0; up_m = '0; ms_free_v = 1'b0; free_m = '0;
    for (int i = 0; i < NM; i++) begin
      up_hit_m[i] = (ms_st[i] != M_FREE) && ms_line[i] == up_req_addr_i;
      if (up_hit_m[i]) up_m = MIW'(i);
    end
    for (int i = NM - 1; i >= 0; i--) if (ms_st[i] == M_FREE) begin ms_free_v = 1'b1; free_m = MIW'(i); end
    up_conf = |up_hit_m;
  end

  //--------------------------------------------------------------------------
  // 响应 FIFO
  //--------------------------------------------------------------------------
  logic [3:0]   fq_id   [FQD];
  logic [511:0] fq_data [FQD];
  logic [$clog2(FQD)-1:0] fq_h, fq_t;
  logic [$clog2(FQD):0]   fq_n;
  logic         fq_push;
  logic [3:0]   fq_push_id;
  logic [511:0] fq_push_data;
  wire          fq_pop  = up_resp_valid_o && up_resp_ready_i;
  wire          fq_room = (fq_n < FQD);
  assign up_resp_valid_o = (fq_n != 0);
  assign up_resp_id_o    = fq_id[fq_h];
  assign up_resp_data_o  = fq_data[fq_h];

  //--------------------------------------------------------------------------
  // 主状态机
  //--------------------------------------------------------------------------
  typedef enum logic [3:0] {
    S_INIT, S_IDLE, S_TAG, S_DRD, S_VRD, S_DONE, S_GAP, S_FTAG, S_FVRD, S_FDONE
  } st_e;
  st_e             st;
  logic [1:0]      wcnt;          // SRAM 读等待计数
  logic [SW-1:0]   init_set;      // 复位后逐组清元数据 (SRAM 上电内容随机)
  logic            q_we;
  logic [PA_W-1:6] q_addr;
  logic [511:0]    q_data;        // 写请求数据 / 读命中数据 / 回填数据
  logic [3:0]      q_id;
  logic            q_dirty;       // 写入后的脏位 (写 = 1；回填 = MSHR 被覆盖过)
  logic [MIW-1:0]  q_m;           // 回填任务的 MSHR 号
  logic [MW-1:0]   meta;          // 本组元数据 (读出后在寄存器里改)
  logic [2:0]      way;           // 命中路或牺牲路
  logic            hit;

  wire [SW-1:0]  set = q_addr[SW+5:6];
  wire [TW-1:0]  tg  = q_addr[PA_W-1:SW+6];

  // 回填任务：有 MSHR 数据到齐、响应 FIFO 有空、写回缓冲空 (牺牲行可能要写回)
  logic          fill_v;
  logic [MIW-1:0] fill_m;
  always_comb begin
    fill_v = 1'b0; fill_m = '0;
    for (int i = NM - 1; i >= 0; i--) if (ms_st[i] == M_FILL) begin fill_v = 1'b1; fill_m = MIW'(i); end
  end
  wire fill_go = (st == S_IDLE) && fill_v && fq_room && !wb_v;

  // 本拍比较 (S_TAG / S_FTAG 且读等待结束)
  logic [NWAY-1:0] hit_oh;
  logic [2:0]      hit_w;
  always_comb begin
    hit_oh = '0; hit_w = '0;
    for (int w = 0; w < NWAY; w++)
      if (e_v(m_rdata, w) && e_tag(m_rdata, w) == tg) begin hit_oh[w] = 1'b1; hit_w = 3'(w); end
  end
  wire [2:0] vic_w   = plru_victim(m_rdata[NWAY*EW +: 7]);
  wire       vic_dty = e_v(m_rdata, vic_w) && e_d(m_rdata, vic_w);
  wire       rd_end  = (wcnt == RD_LAT - 1);

  // 接收上游请求：
  //   写：写回缓冲空即可 (可能要写回牺牲行)；行已有 MSHR 时直接并进 MSHR (不进 FSM)
  //   读：行没有 MSHR、有空闲 MSHR (可能缺失)、响应 FIFO 有空 (可能命中)
  wire up_absorb = up_req_we_i && up_conf;
  assign up_req_ready_o = (st == S_IDLE) && !fill_go &&
                          (up_req_we_i ? (up_conf || !wb_v) : (!up_conf && ms_free_v && fq_room));
  wire up_take = up_req_valid_i && up_req_ready_o;

  // 写回元数据：读命中只更新 PLRU；写 -> 新标签、有效、脏；回填 -> 新标签、有效、脏 = 是否被覆盖
  logic [MW-1:0] meta_new;
  always_comb begin
    meta_new = meta;
    meta_new[EW*way +: TW]    = tg;
    meta_new[EW*way + TW]     = 1'b1;
    meta_new[EW*way + TW + 1] = (hit && !q_we) ? e_d(meta, way) : q_dirty;
    meta_new[NWAY*EW +: 7]    = plru_touch(meta[NWAY*EW +: 7], way);
  end

  always_comb begin
    m_ce = 1'b0; m_we = 1'b0; m_addr = set; m_wdata = meta;
    d_ce = '0; d_we = '0; d_addr = set; d_wdata = q_data;
    fq_push = 1'b0; fq_push_id = q_id; fq_push_data = q_data;
    unique case (st)
      S_INIT: begin
        m_ce = 1'b1; m_we = 1'b1; m_addr = init_set; m_wdata = '0;
      end
      S_IDLE: begin
        if (fill_go) begin m_ce = 1'b1; m_addr = ms_line[fill_m][SW+5:6]; end
        else if (up_take && !up_absorb) begin m_ce = 1'b1; m_addr = up_req_addr_i[SW+5:6]; end
      end
      S_DONE, S_FDONE: begin
        // 写数据 (写 / 回填；读命中不写) + 写元数据 + 读命中 / 回填推响应
        if (!(hit && !q_we)) begin d_ce[way] = 1'b1; d_we[way] = 1'b1; end
        m_ce = 1'b1; m_we = 1'b1; m_wdata = meta_new;
        if (st == S_FDONE || !q_we) fq_push = 1'b1;
      end
      default: ;
    endcase
    // 查标签结束那拍：命中读 / 牺牲行读 (写回)
    if ((st == S_TAG || st == S_FTAG) && rd_end) begin
      if (st == S_TAG && (|hit_oh)) begin if (!q_we) d_ce[hit_w] = 1'b1; end
      else if (vic_dty && (st == S_FTAG || q_we)) d_ce[vic_w] = 1'b1;     // 读缺失不动牺牲行
    end
  end

  //--------------------------------------------------------------------------
  // 下游：写回缓冲优先，其次 MSHR 的读
  //--------------------------------------------------------------------------
  logic           req_v;
  logic [MIW-1:0] req_m;
  always_comb begin
    req_v = 1'b0; req_m = '0;
    for (int i = NM - 1; i >= 0; i--) if (ms_st[i] == M_REQ) begin req_v = 1'b1; req_m = MIW'(i); end
    dn_req_valid_o = wb_v || req_v;
    dn_req_we_o    = wb_v;
    dn_req_addr_o  = wb_v ? wb_line : ms_line[req_m];
    dn_req_data_o  = wb_data;
    dn_req_id_o    = wb_v ? 4'd0 : 4'(req_m);
  end
  wire wb_sent = dn_req_valid_o && dn_req_ready_i && wb_v;
  wire rd_sent = dn_req_valid_o && dn_req_ready_i && !wb_v;
  assign dn_resp_ready_o = 1'b1;          // 每个在途读都有 MSHR 的数据缓冲

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st <= S_INIT; wcnt <= '0; init_set <= '0;
      fq_h <= '0; fq_t <= '0; fq_n <= '0;
      wb_v <= 1'b0; ms_ovr <= '0;
      for (int i = 0; i < NM; i++) ms_st[i] <= M_FREE;
    end else begin
      if (fq_push) fq_t <= fq_t + 1'b1;
      if (fq_pop)  fq_h <= fq_h + 1'b1;
      fq_n <= fq_n + fq_push - fq_pop;
      if (wb_sent) wb_v <= 1'b0;
      if (rd_sent) ms_st[req_m] <= M_WAIT;
      if (dn_resp_valid_i) ms_st[dn_resp_id_i[MIW-1:0]] <= M_FILL;
      // 上游写回并进已有 MSHR
      if (up_take && up_absorb) ms_ovr[up_m] <= 1'b1;
      unique case (st)
        S_INIT: begin
          init_set <= init_set + 1'b1;
          if (init_set == SW'(NSET - 1)) st <= S_IDLE;
        end
        S_IDLE: begin
          if (fill_go)                       begin st <= S_FTAG; wcnt <= '0; end
          else if (up_take && !up_absorb)    begin st <= S_TAG;  wcnt <= '0; end
        end
        S_TAG: begin
          if (!rd_end) wcnt <= wcnt + 1'b1;
          else begin
            wcnt <= '0;
            if (|hit_oh)        st <= q_we ? S_DONE : S_DRD;
            else if (q_we)      st <= vic_dty ? S_VRD : S_DONE;
            else begin                           // 读缺失：分配 MSHR，FSM 空闲
              ms_st[free_m]   <= M_REQ;
              ms_line[free_m] <= q_addr;
              ms_id[free_m]   <= q_id;
              ms_ovr[free_m]  <= 1'b0;
              st <= (RD_LAT > 1) ? S_GAP : S_IDLE;
            end
          end
        end
        S_DRD: if (!rd_end) wcnt <= wcnt + 1'b1; else begin wcnt <= '0; st <= S_DONE; end
        S_VRD: if (!rd_end) wcnt <= wcnt + 1'b1; else begin wcnt <= '0; st <= S_DONE; wb_v <= 1'b1; end
        S_FTAG: begin
          if (!rd_end) wcnt <= wcnt + 1'b1;
          else begin wcnt <= '0; st <= vic_dty ? S_FVRD : S_FDONE; end
        end
        S_FVRD: if (!rd_end) wcnt <= wcnt + 1'b1; else begin wcnt <= '0; st <= S_FDONE; wb_v <= 1'b1; end
        S_FDONE: begin
          ms_st[q_m]  <= M_FREE;
          ms_ovr[q_m] <= 1'b0;
          st <= (RD_LAT > 1) ? S_GAP : S_IDLE;
        end
        // ASIC (RD_LAT=2)：sp299 宏最小周期 > 1 拍，写完元数据后空一拍再接新任务
        S_DONE: st <= (RD_LAT > 1) ? S_GAP : S_IDLE;
        S_GAP:  st <= S_IDLE;
        default: st <= S_IDLE;
      endcase
    end
  end

  // 数据通路寄存器
  always_ff @(posedge clk_i) begin
    if (fill_go) begin
      q_we <= 1'b0; q_addr <= ms_line[fill_m]; q_data <= ms_data[fill_m]; q_id <= ms_id[fill_m];
      q_m <= fill_m; q_dirty <= ms_ovr[fill_m];
    end else if (up_take && !up_absorb) begin
      q_we <= up_req_we_i; q_addr <= up_req_addr_i; q_data <= up_req_data_i; q_id <= up_req_id_i;
      q_dirty <= up_req_we_i;
    end
    if ((st == S_TAG || st == S_FTAG) && rd_end) begin
      hit  <= (st == S_TAG) && (|hit_oh);
      way  <= (st == S_TAG && (|hit_oh)) ? hit_w : vic_w;
      meta <= m_rdata;
      // 只有写缺失 / 回填要写回牺牲行 (它们开始时写回缓冲必为空)；读缺失查标签时写回缓冲可能还有待发的行
      if (vic_dty && !(st == S_TAG && (|hit_oh)) && (st == S_FTAG || q_we)) wb_line <= {e_tag(m_rdata, vic_w), set};
    end
    if ((st == S_VRD || st == S_FVRD) && rd_end) wb_data <= d_rdata[way];
    if (st == S_DRD && rd_end) q_data <= d_rdata[way];
    // MSHR 数据：下游返回 (未被覆盖时) / 上游写回覆盖
    if (dn_resp_valid_i && !ms_ovr[dn_resp_id_i[MIW-1:0]] && !(up_take && up_absorb && up_m == dn_resp_id_i[MIW-1:0]))
      ms_data[dn_resp_id_i[MIW-1:0]] <= dn_resp_data_i;
    if (up_take && up_absorb) ms_data[up_m] <= up_req_data_i;
    if (fq_push) begin
      fq_id[fq_t]   <= fq_push_id;
      fq_data[fq_t] <= fq_push_data;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i) if (rst_ni) begin
    if (dn_resp_valid_i)
      assert (ms_st[dn_resp_id_i[MIW-1:0]] == M_WAIT) else $error("l2: 下游响应 id=%0d 对应的 MSHR 不在等待", dn_resp_id_i);
    if (st == S_FTAG && rd_end)
      assert (!(|hit_oh)) else $error("l2: 回填行 %h 已在 L2 里", q_addr);
    assert (fq_n <= FQD) else $error("l2: 响应 FIFO 溢出");
  end
`endif
  wire unused = (|dn_resp_id_i[3:MIW]);
endmodule
