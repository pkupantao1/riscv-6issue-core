//==============================================================================
// o6_dispatch.sv - 分派级 (DS)
//
// 整组一起分派 (组已在译码级按队列容量截过)，任何资源不够就整组等：
//   ROB 一行；LQ / SQ 空项；四个发射队列各自的空项。
//
// 拆分：
//   store  -> STA (IQ_S, 只等 rs1) + STD (整数数据 IQ_D，浮点数据 IQ_F，只等 rs2)
//   AMO/SC -> 地址 (IQ_S, wait_head) + STD (IQ_D)；LR 只有地址半边
// 端口不在这里定：发射队列在发射时按各端口能执行的功能单元动态选 (uop.port 由 IQ 填)。
// 每个 uop 记录分派时的 LQ/SQ 尾 (lq_idx/sq_idx)，分支误预测时它们就是恢复点。
//==============================================================================
`default_nettype none
module o6_dispatch
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,

  // 来自重命名
  input  wire  [W-1:0]        in_valid_i,
  input  var uop_t [W-1:0]    in_uop_i,
  input  var ds_need_t        in_need_i,      // 重命名级算好的本组资源需求
  output wire                 in_ready_o,

  // ROB
  input  wire                 rob_ready_i,
  input  var rob_ptr_t        rob_ptr_i,
  output wire                 rob_fire_o,
  output logic [W-1:0]        rob_valid_o,
  output uop_t [W-1:0]        rob_uop_o,
  output logic [63:0]         rob_tval_o [W],

  // LSU 分配
  input  var lq_ptr_t         lq_tail_i,
  input  var sq_ptr_t         sq_tail_i,
  input  wire  [LQ_W:0]       lq_free_i,
  input  wire  [SQ_W:0]       sq_free_i,
  output logic [W-1:0]        lq_alloc_o,
  output logic [W-1:0]        sq_alloc_o,
  output uop_t [W-1:0]        lsu_uop_o,      // 与 rob_uop_o 相同，给 LSU 记录表项

  // 发射队列
  input  wire  [6:0]          iq_free_i [N_IQ],
  output logic [IQ_ENQ_MAX-1:0] iq_enq_valid_o [N_IQ],
  output uop_t [IQ_ENQ_MAX-1:0] iq_enq_uop_o  [N_IQ]
);
  logic [IQ_ENQ_MAX-1:0] enq_v [N_IQ];     // 未与 fire 相与的入队有效
  logic [W-1:0]          lq_a, sq_a;
  //--------------------------------------------------------------------------
  // 每个 uop 的去向：路由在重命名级按组成分算好 (in_need_i.sel / lqo / sqo)，
  // 这里只剩组内偏移加尾指针、按独热码选
  //--------------------------------------------------------------------------

  //--------------------------------------------------------------------------
  // LQ / SQ 指针
  //--------------------------------------------------------------------------
  function automatic lq_ptr_t lq_add(input lq_ptr_t p, input logic [2:0] k);
    logic [LQ_W:0] s;
    s = {1'b0, p.idx} + k;
    return '{flag: p.flag ^ s[LQ_W], idx: s[LQ_W-1:0]};
  endfunction
  function automatic sq_ptr_t sq_add(input sq_ptr_t p, input logic [2:0] k);
    logic [SQ_W:0] s;
    s = {1'b0, p.idx} + k;
    return '{flag: p.flag ^ s[SQ_W], idx: s[SQ_W-1:0]};
  endfunction

  uop_t [W-1:0] du;     // 分派后的 uop (主项)
  always_comb begin
    for (int s = 0; s < W; s++) begin
      uop_t u;
      u = in_uop_i[s];
      u.rob_idx = '{flag: rob_ptr_i.flag, row: rob_ptr_i.row, col: ROB_COL_W'(s)};
      u.lq_idx  = lq_add(lq_tail_i, in_need_i.lqo[s]);
      u.sq_idx  = sq_add(sq_tail_i, in_need_i.sqo[s]);
      u.port    = '0;
      lq_a[s] = in_valid_i[s] && (u.fu == FU_LD);
      sq_a[s] = in_valid_i[s] && (u.fu == FU_ST || u.fu == FU_AMO);
      du[s] = u;
    end
  end

  //--------------------------------------------------------------------------
  // 入队：每个 uop 先做好两种形态 (主项 / 数据半边)，再按独热路由与或
  //--------------------------------------------------------------------------
  uop_t hu [2*W];
  always_comb begin
    for (int s = 0; s < W; s++) begin
      uop_t u;
      u = du[s];
      if (u.fu == FU_ST || u.fu == FU_AMO) u.src2_busy = 1'b0;   // 地址半边：不等数据
      hu[s] = u;
      u = du[s];
      u.fu        = FU_STD;
      u.src1_busy = 1'b0;          // 数据半边：不等地址
      u.wait_head = 1'b0;          // AMO 的数据可以提前送进 SQ
      u.rd_class  = RC_NONE;       // STD 不写寄存器
      hu[W+s] = u;
    end
  end

  always_comb begin
    for (int q = 0; q < N_IQ; q++)
      for (int k = 0; k < IQ_ENQ_MAX; k++) begin
        uop_t acc;
        acc = '0;
        for (int j = 0; j < 2*W; j++) acc = acc | (hu[j] & {$bits(uop_t){in_need_i.sel[q][k][j]}});
        enq_v[q][k]        = |in_need_i.sel[q][k];
        iq_enq_uop_o[q][k] = acc;
      end
  end

`ifndef SYNTHESIS
  //--------------------------------------------------------------------------
  // 参考模型 (仅仿真)：原来逐条串行的去向 / 压缩计算，用来核对重命名级算好的路由
  //--------------------------------------------------------------------------
  logic [2:0] iq0 [W];   logic has0 [W];
  logic [2:0] iq1 [W];   logic has1 [W];

  always_comb begin
    for (int s = 0; s < W; s++) begin
      uop_t u;
      u = in_uop_i[s];
      has0[s] = 1'b0; has1[s] = 1'b0;
      iq0[s] = '0; iq1[s] = '0;
      if (in_valid_i[s]) begin
        unique case (u.fu)
          FU_ALU, FU_BRU: begin            // 分流结果来自重命名级 (按当时的队列占用)
            has0[s] = 1'b1;
            iq0[s] = (in_need_i.aq[s] == 2'd0) ? 3'(IQ_A) : (in_need_i.aq[s] == 2'd1) ? 3'(IQ_B) : 3'(IQ_C);
          end
          FU_MUL, FU_DIV, FU_CSR, FU_I2F, FU_VEC:
                   begin has0[s] = 1'b1; iq0[s] = 3'(IQ_B); end
          FU_LD:   begin has0[s] = 1'b1; iq0[s] = in_need_i.mq[s] ? 3'(IQ_N) : 3'(IQ_M); end
          FU_ST: begin
            has0[s] = 1'b1; iq0[s] = 3'(IQ_S);
            has1[s] = 1'b1; iq1[s] = (u.rs2_class == RC_FP) ? 3'(IQ_F) : 3'(IQ_D);
          end
          FU_AMO: begin
            has0[s] = 1'b1; iq0[s] = 3'(IQ_S);
            if (!amo_nodata(u.op)) begin has1[s] = 1'b1; iq1[s] = 3'(IQ_D); end
          end
          FU_FMA, FU_FMISC, FU_F2I, FU_FDIV:
                   begin has0[s] = 1'b1; iq0[s] = 3'(IQ_F); end
          default: ;   // FU_NONE：不进队列
        endcase
      end
    end
  end

  uop_t [W-1:0] rdu;
  logic [W-1:0] r_lq_a, r_sq_a;
  logic [3:0]   n_ld, n_st;
  always_comb begin
    logic [2:0] cl, cs;
    cl = '0; cs = '0;
    for (int s = 0; s < W; s++) begin
      uop_t u;
      u = in_uop_i[s];
      u.rob_idx = '{flag: rob_ptr_i.flag, row: rob_ptr_i.row, col: ROB_COL_W'(s)};
      u.lq_idx  = lq_add(lq_tail_i, cl);
      u.sq_idx  = sq_add(sq_tail_i, cs);
      u.port    = '0;
      r_lq_a[s] = in_valid_i[s] && (u.fu == FU_LD);
      r_sq_a[s] = in_valid_i[s] && (u.fu == FU_ST || u.fu == FU_AMO);
      cl += r_lq_a[s];
      cs += r_sq_a[s];
      rdu[s] = u;
    end
    n_ld = {1'b0, cl};
    n_st = {1'b0, cs};
  end

  //--------------------------------------------------------------------------
  // 压缩到各发射队列的入队槽 (程序序：同一 uop 的主项在前)
  //--------------------------------------------------------------------------
  logic [3:0] iq_cnt [N_IQ];
  logic [IQ_ENQ_MAX-1:0] ref_v [N_IQ];
  uop_t [IQ_ENQ_MAX-1:0] ref_u [N_IQ];
  always_comb begin
    for (int q = 0; q < N_IQ; q++) begin
      iq_cnt[q] = '0;
      ref_v[q] = '0;
      ref_u[q] = '0;
    end
    for (int s = 0; s < W; s++) begin
      if (has0[s]) begin
        uop_t u;
        u = rdu[s];
        if (u.fu == FU_ST || u.fu == FU_AMO) begin
          // 地址半边：不等数据
          u.src2_busy = 1'b0;
        end
        if (iq_cnt[iq0[s]] < IQ_ENQ_MAX) begin
          ref_v[iq0[s]][iq_cnt[iq0[s]][1:0]] = 1'b1;
          ref_u[iq0[s]][iq_cnt[iq0[s]][1:0]]   = u;
        end
        iq_cnt[iq0[s]] = iq_cnt[iq0[s]] + 1'b1;
      end
      if (has1[s]) begin
        uop_t u;
        u = rdu[s];
        u.fu        = FU_STD;
        u.src1_busy = 1'b0;          // 数据半边：不等地址
        u.wait_head = 1'b0;          // AMO 的数据可以提前送进 SQ
        u.rd_class  = RC_NONE;       // STD 不写寄存器
        if (iq_cnt[iq1[s]] < IQ_ENQ_MAX) begin
          ref_v[iq1[s]][iq_cnt[iq1[s]][1:0]] = 1'b1;
          ref_u[iq1[s]][iq_cnt[iq1[s]][1:0]]   = u;
        end
        iq_cnt[iq1[s]] = iq_cnt[iq1[s]] + 1'b1;
      end
    end
  end

`endif

  //--------------------------------------------------------------------------
  // 整组资源判定
  //--------------------------------------------------------------------------
  // 只比较寄存的需求与空位 (iq_cnt / n_ld / n_st 的组合计算只用来生成入队数据)
  logic iq_ok;
  always_comb begin
    iq_ok = 1'b1;
    for (int q = 0; q < N_IQ; q++)
      if (7'(in_need_i.iqn[q]) > iq_free_i[q]) iq_ok = 1'b0;
  end

  wire any  = |in_valid_i;
  wire fire = any && rob_ready_i && iq_ok &&
              ((LQ_W+1)'(in_need_i.nld) <= lq_free_i) && ((SQ_W+1)'(in_need_i.nst) <= sq_free_i) &&
              !flush_i && !br_upd_i.kill;

`ifndef SYNTHESIS
  // 寄存的需求必须与本拍按组成分算出的一致
  always_ff @(posedge clk_i)
    if (rst_ni && any) begin
      for (int q = 0; q < N_IQ; q++)
        assert (4'(in_need_i.iqn[q]) == iq_cnt[q]) else $error("o6_dispatch: 需求 iq%0d %0d != %0d", q, in_need_i.iqn[q], iq_cnt[q]);
      assert (4'(in_need_i.nld) == n_ld && 4'(in_need_i.nst) == n_st) else $error("o6_dispatch: LQ/SQ 需求不一致");
      for (int q = 0; q < N_IQ; q++) begin
        assert (enq_v[q] == ref_v[q]) else $error("o6_dispatch: 入队路由 iq%0d %b != 参考 %b", q, enq_v[q], ref_v[q]);
        for (int k = 0; k < IQ_ENQ_MAX; k++)
          assert (!ref_v[q][k] || iq_enq_uop_o[q][k] == ref_u[q][k]) else $error("o6_dispatch: 入队 uop iq%0d[%0d] 与参考不一致", q, k);
      end
      for (int s = 0; s < W; s++)
        assert (!in_valid_i[s] || du[s] == rdu[s]) else $error("o6_dispatch: 第 %0d 条 uop (端口 / LQ / SQ 序号) 与参考不一致", s);
    end
`endif

  assign in_ready_o  = fire;
  assign rob_fire_o  = fire;
  assign rob_valid_o = in_valid_i;
  assign rob_uop_o   = du;
  assign lsu_uop_o   = du;

  always_comb begin
    for (int s = 0; s < W; s++) rob_tval_o[s] = {32'b0, in_uop_i[s].raw};
  end

  // 入队 / 分配只在整组分派时有效
  always_comb begin
    for (int q = 0; q < N_IQ; q++) iq_enq_valid_o[q] = enq_v[q] & {IQ_ENQ_MAX{fire}};
  end
  assign lq_alloc_o = lq_a & {W{fire}};
  assign sq_alloc_o = sq_a & {W{fire}};

endmodule
