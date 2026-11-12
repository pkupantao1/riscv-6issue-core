//==============================================================================
// o6_rat.sv - 寄存器别名表 (一类寄存器一份：整数 / 浮点)
//
// 推测 RAT：重命名级读 (W 组 × 4 个读：rs1/rs2/rs3/旧 rd) 与写 (W 个，组内后者优先)
// 提交 RAT：提交级写，异常冲刷时整体拷回推测 RAT
// 分支快照：每个分支标签一份，保存"组内到该分支为止"的映射；误预测时整体恢复
//
// 组内依赖旁路 (源被同组更早的指令写) 在 o6_rename 里做，这里只给出组开始前的映射。
//==============================================================================
`default_nettype none
module o6_rat
  import o6_pkg::*;
#(
  parameter int NARCH = 32
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 读 (组开始前的映射)
  input  wire  [4:0]          rd_addr_i [W][4],
  output preg_t               rd_data_o [W][4],

  // 推测写
  input  wire  [W-1:0]        wr_en_i,
  input  wire  [4:0]          wr_addr_i [W],
  input  var preg_t [W-1:0]       wr_preg_i,

  // 分支快照：snap_en[b] 时把 "写入槽 0..snap_slot[b] 之后" 的映射存到 snap_tag[b]
  input  wire  [MAX_BR_PER_GRP-1:0] snap_en_i,
  input  var br_tag_t [MAX_BR_PER_GRP-1:0] snap_tag_i,
  input  wire  [2:0]          snap_slot_i [MAX_BR_PER_GRP],

  // 恢复
  input  wire                 br_restore_i,
  input  var br_tag_t             br_restore_tag_i,
  input  wire                 flush_i,          // 异常冲刷：推测 <- 提交

  // 提交写
  input  wire  [W-1:0]        cm_en_i,
  input  wire  [4:0]          cm_addr_i [W],
  input  var preg_t [W-1:0]       cm_preg_i
);
  preg_t spec [NARCH];
  preg_t arch [NARCH];
  preg_t snap [MAX_BR][NARCH];

  // 读
  always_comb begin
    for (int s = 0; s < W; s++)
      for (int p = 0; p < 4; p++)
        rd_data_o[s][p] = spec[rd_addr_i[s][p]];
  end

  // 写入槽 0..k 之后的映射 (k = 0..W-1)
  preg_t after [W][NARCH];
  always_comb begin
    for (int r = 0; r < NARCH; r++) begin
      preg_t v;
      v = spec[r];
      for (int k = 0; k < W; k++) begin
        if (wr_en_i[k] && wr_addr_i[k] == 5'(r)) v = wr_preg_i[k];
        after[k][r] = v;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int r = 0; r < NARCH; r++) begin
        spec[r] <= preg_t'(r);
        arch[r] <= preg_t'(r);
      end
    end else begin
      // 提交 RAT (组内后者优先)
      for (int k = 0; k < W; k++)
        if (cm_en_i[k]) arch[cm_addr_i[k]] <= cm_preg_i[k];

      if (flush_i) begin
        // 注意：用本拍提交之后的 arch
        for (int r = 0; r < NARCH; r++) begin
          preg_t v;
          v = arch[r];
          for (int k = 0; k < W; k++)
            if (cm_en_i[k] && cm_addr_i[k] == 5'(r)) v = cm_preg_i[k];
          spec[r] <= v;
        end
      end else if (br_restore_i) begin
        for (int r = 0; r < NARCH; r++) spec[r] <= snap[br_restore_tag_i][r];
      end else begin
        for (int r = 0; r < NARCH; r++) spec[r] <= after[W-1][r];
      end
    end
  end

  // 快照 (不需要复位：只在分配标签时写，读之前一定写过)
  always_ff @(posedge clk_i) begin
    for (int b = 0; b < MAX_BR_PER_GRP; b++)
      if (snap_en_i[b])
        for (int r = 0; r < NARCH; r++) snap[snap_tag_i[b]][r] <= after[snap_slot_i[b]][r];
  end
endmodule
