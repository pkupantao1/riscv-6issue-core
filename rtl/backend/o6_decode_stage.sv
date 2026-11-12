//==============================================================================
// o6_decode_stage.sv - 译码级 (DE)：6 路译码 + 按资源截组 + DE/RN 流水寄存器
//
// 截组规则 (保证分派级"整组要么全进要么全等"不会因为结构限制永远卡住)：
//   - 组内分支 (需要标签的) <= MAX_BR_PER_GRP
//   - 每个发射队列入队数 <= IQ_ENQ_MAX：
//       IQ_M: LD        IQ_S: STA / AMO 地址      IQ_D: 整数 STD / AMO 数据     IQ_F: 浮点运算 / 浮点 STD
//       IQ_B: MUL / DIV / CSR / I2F
//       ALU 与 BRU 可进 A 或 B：要求 BRU <= IQ_ENQ_MAX、B 固定 <= IQ_ENQ_MAX、BRU + B 固定 + ALU <= 2*IQ_ENQ_MAX
// 剩下的指令留在取指缓冲，下一拍重新译码。
//==============================================================================
`default_nettype none
module o6_decode_stage
  import o6_pkg::*;
  import o6_isa_pkg::*;
(
  input  wire                     clk_i,
  input  wire                     rst_ni,
  input  wire                     flush_i,       // 冲刷 / 误预测 (DE 里都是较新的指令)

  // 取指缓冲
  input  wire  [W-1:0]            ib_valid_i,    // 前缀有效
  input  var fetch_inst_t [W-1:0] ib_inst_i,
  output logic [2:0]              ib_pop_o,      // 本拍取走几条

  // CSR 状态
  input  var priv_e               priv_i,
  input  wire  [1:0]              fs_i,
  input  wire  [1:0]              vs_i,          // mstatus.VS
  input  wire  [2:0]              frm_i,
  input  wire  [2:0]              cbo_ok_i,      // Zicbom / Zicboz 可执行 (o6_csr)
  input  wire  [2:0]              cbo_vi_i,      // 不可执行时报 virtual instruction
  input  wire                     virt_i,        // H：V 位
  input  wire                     hlv_ok_i,      // H：HLV / HSV 可执行
  input  wire                     hlv_vi_i,      // H：不可执行时报 virtual instruction

  // 去重命名
  output logic [W-1:0]            out_valid_o,
  output uop_t [W-1:0]            out_uop_o,
  input  wire                     out_ready_i
);
  uop_t [W-1:0] dec0, dec;
  for (genvar s = 0; s < W; s++) begin : g_dec
    o6_decoder u_dec (
      .fi_i(ib_inst_i[s]), .priv_i, .fs_i, .vs_i, .frm_i, .cbo_ok_i, .cbo_vi_i, .virt_i, .hlv_ok_i, .hlv_vi_i, .uop_o(dec0[s])
    );
  end

  // move 消除：候选的源若由同组更早的另一个候选写，就不消除 (照常走 ALU)。
  // 这样重命名级被消除者的新映射 = RAT 读出或更早普通槽的新 pdst，组内旁路链不会因为 move 串长
  always_comb begin
    for (int s = 0; s < W; s++) begin
      uop_t u;
      logic [4:0] src;
      logic       blk;
      u = dec0[s];
      src = u.mvsrc ? u.lrs2 : u.lrs1;
      blk = 1'b0;
      for (int j = 0; j < s; j++)
        if (dec0[j].mvc && dec0[j].lrd == src && src != 5'd0) blk = 1'b1;
      u.mvel = u.mvc && !blk;
      if (u.mvel) u.fu = FU_NONE;
      dec[s] = u;
    end
  end

  // 分类：用前端 F3 预分好的类别 (随取指缓冲寄存)，截组不必等完整译码
  logic [W-1:0] c_br, c_alu, c_a, c_b, c_m, c_s, c_d, c_f;
  always_comb begin
    for (int s = 0; s < W; s++)
      {c_br[s], c_a[s], c_b[s], c_m[s], c_s[s], c_d[s], c_f[s], c_alu[s]} = ib_inst_i[s].cls;
  end

`ifndef SYNTHESIS
  // 预分类别必须覆盖译码器的实际类别 (只多不少)
  always_ff @(posedge clk_i)
    if (rst_ni)
      for (int s = 0; s < W; s++)
        if (ib_valid_i[s]) begin
          uop_t u;
          logic [7:0] d;
          u = dec0[s];
          d = {u.is_br, u.fu == FU_BRU,
               (u.fu == FU_MUL) || (u.fu == FU_DIV) || (u.fu == FU_CSR) || (u.fu == FU_I2F) || (u.fu == FU_VEC),
               u.fu == FU_LD,
               (u.fu == FU_ST) || (u.fu == FU_AMO),
               (u.fu == FU_ST && u.rs2_class != RC_FP) || (u.fu == FU_AMO && !amo_nodata(u.op)),
               (u.fu == FU_FMA) || (u.fu == FU_FMISC) || (u.fu == FU_F2I) || (u.fu == FU_FDIV) ||
               (u.fu == FU_ST && u.rs2_class == RC_FP),
               u.fu == FU_ALU};
          assert ((d & ~ib_inst_i[s].cls) == '0)
            else $error("o6_decode_stage: 预分类别 %b 未覆盖译码类别 %b (slot %0d inst=%h)", ib_inst_i[s].cls, d, s, ib_inst_i[s].inst);
        end
`endif

  // 截组：每类资源在前缀 0..s 上的计数都不超限的槽才收。计数随 s 单调不减，超限后一直超限，
  // 而 ib_valid 是前缀有效，所以 take 自然是前缀 —— 各槽并行判断，不用逐槽串行累加 (原写法用 int
  // 计数器串 6 级加法比较，是 IB -> 译码 -> 出队 -> ib_cnt 的关键路径)
  function automatic logic [2:0] pcnt(input logic [W-1:0] v, input int s);
    logic [2:0] r;
    r = '0;
    for (int k = 0; k < W; k++) if (k <= s) r = r + 3'(v[k]);
    return r;
  endfunction
  logic [W-1:0] take;
  logic [2:0]   n_take;
  always_comb begin
    n_take = '0;
    for (int s = 0; s < W; s++) begin
      logic [2:0] kbr, ka, kb, km, ks, kd, kf, kx;
      kbr = pcnt(c_br, s); ka = pcnt(c_a, s); kb = pcnt(c_b, s);
      km  = pcnt(c_m, s);  ks = pcnt(c_s, s); kd = pcnt(c_d, s);
      kf  = pcnt(c_f, s);  kx = pcnt(c_a | c_b | c_alu, s);
      take[s] = ib_valid_i[s] && kbr <= 3'(MAX_BR_PER_GRP) && ka <= 3'(IQ_ENQ_MAX) && kb <= 3'(IQ_ENQ_MAX) &&
                km <= 3'(IQ_ENQ_MAX) && ks <= 3'(IQ_ENQ_MAX) && kd <= 3'(IQ_ENQ_MAX) &&
                kf <= 3'(IQ_ENQ_MAX) && 4'(kx) <= 4'(2*IQ_ENQ_MAX);
    end
    for (int s = 0; s < W; s++) n_take = n_take + 3'(take[s]);
  end

  // 输出 2 组队列 (q0 对外可见)：取指缓冲出队只看寄存的"q1 空"，
  // 不再等重命名 / 分派当拍的就绪 (那是 IQ/ROB/LSQ 空位计数的深逻辑)。
  // 队列空时译码结果直接进 q0，不增加延迟；重命名停住时最多再收一组
  logic [W-1:0] q1_v;
  uop_t [W-1:0] q1_u;
  wire load = !(|q1_v) && !flush_i;
  wire push = load && (|take);
  wire pop  = (|out_valid_o) && out_ready_i;
  assign ib_pop_o = load ? n_take : 3'd0;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      out_valid_o <= '0; q1_v <= '0;
    end else if (flush_i) begin
      out_valid_o <= '0; q1_v <= '0;
    end else if (!(|out_valid_o) || pop) begin
      if (|q1_v) begin out_valid_o <= q1_v; q1_v <= push ? take : '0; end
      else            out_valid_o <= push ? take : '0;
    end else if (push) begin
      q1_v <= take;
    end
  end

  always_ff @(posedge clk_i) begin
    if (!(|out_valid_o) || pop) begin
      if (|q1_v) begin out_uop_o <= q1_u; if (push) q1_u <= dec; end
      else if (push) out_uop_o <= dec;
    end else if (push) begin
      q1_u <= dec;
    end
  end
endmodule
