//==============================================================================
// o6_viq.sv - 核内向量发射队列 VIQ (O1，docs/11_vector_uarch.md §9f / §9h)
//
// 乱序向量 uop (重命名级定的 vfast) 分派时按程序序入队 (同时也进 VDQ：提交时经 VECU 提交口按序切换映射)。
// 源就绪 = 不用 / 按 crat 读 (重命名时已确认最近的写做完) / 物理寄存器不忙 (rename 的 vbusy：写回那拍清，下一拍可选)。
// 存储是环形缓冲，发射后留空洞，头指针越过空洞。viq_ok_o (头到尾的跨度 <= N - W) 参与分派判定
// M3b：向量 store 数据 uop (vms) 也在这里排队，经单独的 store 数据口 (vs_*) 发出：VX 读 src[2] 送 SQ 项 sq
// V3：每拍至多发 NVX 条运算 (各进一个空闲的执行器，从最老的起按序分给编号小的) + 1 个 store 数据 uop。
//   V7：SEW 32 / 64 的浮点运算只能去 VX_HP (FP64 / FP32 执行器)，其余 (整数、FP16、点积) 只能去 0..NVX_LP-1
//   链式 (按寄存器唤醒)：源组第 0 个寄存器要已写好；第 j >= 1 个可以还在写，只要它的生产者已发射到某个执行器
//   (vxiss：发射时整组置位，该物理寄存器再分配 (入队) 时清)。执行器的序列器第 j 拍发现它还忙就停一拍，
//   生产者都比消费者先发射、不会反过来等，所以不会死锁
// 误预测：br_mask 含误预测标签的作废，队尾退到最后一个存活项之后；冲刷：清空
//==============================================================================
`default_nettype none
module o6_viq
  import o6_pkg::*;
#(
  parameter int N = VDQ_N
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,

  // 分派
  input  wire                 enq_fire_i,
  input  wire  [W-1:0]        enq_valid_i,
  input  var uop_t [W-1:0]    enq_uop_i,

  input  wire  [N_VPREG-1:0]  vbusy_i,

  // 运算发射 (到 VECU 的 NVX 个执行器)；整条发射：组大小、源 0..2 的整组映射、v0、目的组
  output logic [NVX-1:0]      vx_v_o,
  output logic [31:0]         vx_raw_o [NVX],
  output logic [1:0]          vx_sew_o [NVX],
  output logic [10:0]         vx_vl_o  [NVX],
  output vsrc_t               vx_v0_o  [NVX],
  output rob_ptr_t            vx_rob_o [NVX],
  output br_mask_t            vx_bm_o  [NVX],
  output logic [3:0]          vx_ng_o  [NVX],
  output vsrc_t [2:0][7:0]    vx_sg_o  [NVX],
  output preg_t [7:0]         vx_pg_o  [NVX],
  output logic [63:0]         vx_xs_o  [NVX],
  input  wire  [NVX-1:0]      vx_rdy_i,         // 执行器空闲 (序列器没在送上一条)
  input  wire  [NVX-1:0]      vx_dok_i,         // V4：执行器的点积单元有空的 A 缓冲 (点积只发给这样的)
  output logic [3:0]          vx_ns_o  [NVX],   // V4：源组寄存器数 (拍数)
  output logic [2:0]          vx_lm_o  [NVX],   // V4：LMUL 编码
  output logic                vx_alt_o [NVX],   // V4：altfmt
  // store 数据口 (M3b)
  output logic                vs_v_o,
  output logic [31:0]         vs_raw_o,
  output vsrc_t [3:0]         vs_src_o,
  output rob_ptr_t            vs_rob_o,
  output br_mask_t            vs_bm_o,
  output sq_ptr_t             vs_sq_o,
  output logic [2:0]          vs_vk_o,
  // V1.2c：带标量源的 (vxs) 等数据半边 (STD 口) 按 ROB 号送来 x / f[rs1]；发射时带出
  input  wire  [N_STD-1:0]    xsd_v_i,
  input  var rob_ptr_t [N_STD-1:0] xsd_rob_i,
  input  wire  [63:0]         xsd_d_i [N_STD],
  output logic                viq_ok_o         // 能再收一整组
);
  localparam int AW = $clog2(N);

  typedef struct packed {
    logic [31:0] raw;
    logic [1:0]  sew;
    logic [10:0] vl;
    vsrc_t [3:0] src;
    rob_ptr_t    rob;
    logic        vms;
    sq_ptr_t     sq;
    logic [2:0]  vk;          // M4a：store 数据 uop 是组里第几个寄存器 (带掩码时按它取 v0 的位)
    logic [3:0]  ng;          // V1.2b：目的组
    logic [3:0]  ns;          // V4：源组 (点积 ng = 1)
    logic [2:0]  lm;
    logic        alt;
    vsrc_t [2:0][7:0] sg;
    preg_t [7:0] pg;
  } ent_t;

  ent_t            e   [N];
  br_mask_t        bm  [N];
  logic [N-1:0]    val;
  logic [N-1:0]    xs_need, xs_ok;                       // V1.2c
  logic [63:0]     xs_val [N];
  logic [AW:0]     head, tail;                 // {圈, 下标}
  logic [N_VPREG-1:0] vxiss;                   // V3：生产者已发射到执行器 (链式)
  // V4 累加器转发：物理寄存器正由某执行器上的点积写 (vxdv) 及是哪个 (vxdx)。点积的 vd 旧值是它时不必等写回，
  //   只能发到同一个执行器 (点积单元在 S0 才读 vd，按序在生产者之后)
  localparam int XW = (NVX > 1) ? $clog2(NVX) : 1;
  logic [N_VPREG-1:0] vxdv;
  logic [XW-1:0]      vxdx [N_VPREG];
  logic [N-1:0]       fw;                      // 项 i 的 vd 旧值要转发 (只能去 vxdx 那个执行器)
  logic [XW-1:0]      fwx [N];

  function automatic logic [AW-1:0] ix(input logic [AW:0] p, input int k);
    return AW'(int'(p[AW-1:0]) + k);
  endfunction
  wire [AW:0] span = tail - head;

  // 就绪
  logic [N-1:0] rdy;
  always_comb
    for (int i = 0; i < N; i++) begin
      logic r;
      r = val[i] && !br_killed(bm[i], br_upd_i);
      fw[i]  = vdot_op(e[i].raw) && e[i].src[2].use_ && !e[i].src[2].cr && vbusy_i[e[i].src[2].p] && vxdv[e[i].src[2].p];
      fwx[i] = vxdx[e[i].src[2].p];
      for (int k = 0; k < 4; k++)
        if (e[i].src[k].use_ && !e[i].src[k].cr && vbusy_i[e[i].src[k].p] && !(k == 2 && fw[i])) r = 1'b0;
      // 组里第 j >= 1 个：运算可以链式 (生产者已发射)；store 数据 uop 只有 1 个寄存器
      for (int k = 0; k < 3; k++)
        for (int j = 1; j < 8; j++)
          if (j < int'((k < 2) ? e[i].ns : e[i].ng) && e[i].sg[k][j].use_ && !e[i].sg[k][j].cr &&
              vbusy_i[e[i].sg[k][j].p] && !vxiss[e[i].sg[k][j].p]) r = 1'b0;
      if (xs_need[i] && !xs_ok[i]) r = 1'b0;                // V1.2c：标量还没到
      rdy[i] = r;
    end

  // 选择：从队头起按年龄，store 数据 uop 取最老的一个；运算依次分给编号最小的空闲执行器
  logic                vs_sel_v;
  logic [AW-1:0]       vs_sel;
  logic [NVX-1:0]      xv;
  logic [AW-1:0]       xsel [NVX];
  logic [N-1:0]        isd;                    // 本拍发出的项
  always_comb begin
    logic [NVX-1:0] fr;
    vs_sel_v = 1'b0; vs_sel = '0; xv = '0; isd = '0;
    for (int x = 0; x < NVX; x++) xsel[x] = '0;
    fr = vx_rdy_i;
    for (int k = 0; k < N; k++) begin
      logic [AW-1:0] i;
      i = ix(head, k);
      if ((AW+1)'(k) < span && rdy[i]) begin
        if (e[i].vms) begin
          if (!vs_sel_v) begin vs_sel_v = 1'b1; vs_sel = i; isd[i] = 1'b1; end
        end else begin
          logic dn;
          dn = 1'b0;
          for (int x = 0; x < NVX; x++)
            if (!dn && fr[x] && (vx_hp(e[i].raw, e[i].sew) == (x == VX_HP)) &&     // V7：按精度分执行器
                (!vdot_op(e[i].raw) || vx_dok_i[x]) && (!fw[i] || fwx[i] == XW'(x))) begin dn = 1'b1; fr[x] = 1'b0; xv[x] = 1'b1; xsel[x] = i; isd[i] = 1'b1; end
        end
      end
    end
  end
  for (genvar x = 0; x < NVX; x++) begin : g_out
    assign vx_v_o[x]   = xv[x] && !flush_i;
    assign vx_raw_o[x] = e[xsel[x]].raw;
    assign vx_sew_o[x] = e[xsel[x]].sew;
    assign vx_vl_o[x]  = e[xsel[x]].vl;
    assign vx_v0_o[x]  = e[xsel[x]].src[3];
    assign vx_rob_o[x] = e[xsel[x]].rob;
    assign vx_bm_o[x]  = br_mask_upd(bm[xsel[x]], br_upd_i);
    assign vx_ng_o[x]  = e[xsel[x]].ng;
    assign vx_ns_o[x]  = e[xsel[x]].ns;
    assign vx_lm_o[x]  = e[xsel[x]].lm;
    assign vx_alt_o[x] = e[xsel[x]].alt;
    assign vx_sg_o[x]  = e[xsel[x]].sg;
    assign vx_pg_o[x]  = e[xsel[x]].pg;
    assign vx_xs_o[x]  = xs_val[xsel[x]];
  end
  assign vs_v_o   = vs_sel_v && !flush_i;
  assign vs_raw_o = e[vs_sel].raw;
  assign vs_src_o = e[vs_sel].src;
  assign vs_rob_o = e[vs_sel].rob;
  assign vs_bm_o  = br_mask_upd(bm[vs_sel], br_upd_i);
  assign vs_sq_o  = e[vs_sel].sq;
  assign vs_vk_o  = e[vs_sel].vk;
  assign viq_ok_o = span <= (AW+1)'(N - W);

  // 状态更新
  logic [W-1:0] ev;
  logic [3:0]   n_enq;
  always_comb begin
    n_enq = '0;
    for (int s = 0; s < W; s++) begin
      ev[s] = enq_fire_i && enq_valid_i[s] && enq_uop_i[s].vfast && !enq_uop_i[s].vml;   // M3：向量 load 走 LSU
      n_enq = n_enq + 4'(ev[s]);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      val <= '0; head <= '0; tail <= '0;
    end else if (flush_i) begin
      val <= '0; head <= '0; tail <= '0;
    end else begin
      logic [N-1:0] v;
      logic [AW:0]  t, h;
      int           last, first;
      v = val & ~isd;
      t = tail;
      if (br_upd_i.kill) begin
        for (int i = 0; i < N; i++) if (bm[i][br_upd_i.kill_tag]) v[i] = 1'b0;
        last = 0;
        for (int k = 0; k < N; k++) if ((AW+1)'(k) < span && v[ix(head, k)]) last = k + 1;
        t = head + (AW+1)'(last);
      end else begin
        int o;
        o = 0;
        for (int s = 0; s < W; s++)
          if (ev[s]) begin v[ix(tail, o)] = 1'b1; o++; end
        t = tail + (AW+1)'(n_enq);
      end
      // 队头越过空洞
      first = -1;
      for (int k = N - 1; k >= 0; k--) if ((AW+1)'(k) < (t - head) && v[ix(head, k)]) first = k;
      h = (first < 0) ? t : head + (AW+1)'(first);
      val <= v; head <= h; tail <= t;
    end
  end
  // V3 vxiss：发射时目的组置位；新分配 (任何乱序向量 uop 入队，含走 LSU 的 load) 时清；冲刷全清 (V4 vxdv 同)
  always_ff @(posedge clk_i)
    for (int x = 0; x < NVX; x++)
      if (xv[x] && vdot_op(e[xsel[x]].raw)) vxdx[e[xsel[x]].pg[0]] <= XW'(x);
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) begin vxiss <= '0; vxdv <= '0; end
    else if (flush_i) begin vxiss <= '0; vxdv <= '0; end
    else begin
      logic [N_VPREG-1:0] b, d;
      b = vxiss; d = vxdv;
      for (int x = 0; x < NVX; x++)
        if (xv[x] && vdot_op(e[xsel[x]].raw)) d[e[xsel[x]].pg[0]] = 1'b1;
      for (int x = 0; x < NVX; x++)
        if (xv[x])
          for (int j = 0; j < 8; j++) if (j < int'(e[xsel[x]].ng)) b[e[xsel[x]].pg[j]] = 1'b1;
      if (enq_fire_i)
        for (int s = 0; s < W; s++)
          if (enq_valid_i[s] && enq_uop_i[s].vfast && !enq_uop_i[s].vms)
            for (int j = 0; j < 8; j++) if (j < int'(enq_uop_i[s].vng)) begin b[enq_uop_i[s].vpd[j]] = 1'b0; d[enq_uop_i[s].vpd[j]] = 1'b0; end
      vxiss <= b; vxdv <= d;
    end
  always_ff @(posedge clk_i) begin
    for (int i = 0; i < N; i++) bm[i] <= br_mask_upd(bm[i], br_upd_i);
    // V1.2c：数据半边送来的标量按 ROB 号收进等它的项
    for (int i = 0; i < N; i++)
      for (int k = 0; k < N_STD; k++)
        if (xsd_v_i[k] && xs_need[i] && !xs_ok[i] && xsd_rob_i[k] == e[i].rob) begin
          xs_ok[i] <= 1'b1; xs_val[i] <= xsd_d_i[k];
        end
    if (!flush_i && !br_upd_i.kill) begin
      int o;
      o = 0;
      for (int s = 0; s < W; s++)
        if (ev[s]) begin
          e[ix(tail, o)]  <= '{raw: enq_uop_i[s].raw, sew: enq_uop_i[s].vsew, vl: enq_uop_i[s].vvl,
                              src: enq_uop_i[s].vps, rob: enq_uop_i[s].rob_idx,
                              vms: enq_uop_i[s].vms, sq: enq_uop_i[s].sq_idx, vk: enq_uop_i[s].vk,
                              ng: enq_uop_i[s].vms ? 4'd1 : enq_uop_i[s].vng, ns: enq_uop_i[s].vms ? 4'd1 : enq_uop_i[s].vns,
                              lm: enq_uop_i[s].vlm, alt: enq_uop_i[s].valt, sg: enq_uop_i[s].vpg, pg: enq_uop_i[s].vpd};
          bm[ix(tail, o)] <= br_mask_upd(enq_uop_i[s].br_mask, br_upd_i);
          xs_need[ix(tail, o)] <= enq_uop_i[s].vxs; xs_ok[ix(tail, o)] <= 1'b0;
          o++;
        end
    end
  end

`ifndef SYNTHESIS
  initial assert ((1 << AW) == N) else $error("o6_viq: N = %0d 不是 2 的幂", N);
  always_ff @(posedge clk_i)
    if (rst_ni && !flush_i) begin
      assert (int'(span) + int'(n_enq) <= N) else $error("o6_viq: 溢出 (占 %0d + 入 %0d > %0d)", span, n_enq, N);
      for (int i = 0; i < N; i++)
        if (val[i]) begin
          logic [AW-1:0] off;
          off = AW'(i) - head[AW-1:0];                    // 模 N 的偏移 (先在 AW 位上减，再比)
          assert ((AW+1)'(off) < span) else $error("o6_viq: 有效项 %0d 不在 [头, 尾) 内", i);
        end
      // 发出的运算项第 0 拍的源都已写好
      for (int x = 0; x < NVX; x++)
        if (xv[x])
          for (int k = 0; k < 3; k++)
            if (e[xsel[x]].sg[k][0].use_ && !e[xsel[x]].sg[k][0].cr && !(k == 2 && fw[xsel[x]]))
              assert (!vbusy_i[e[xsel[x]].sg[k][0].p]) else $error("o6_viq: 执行器 %0d 第 0 拍源 %0d 还忙", x, k);
    end
`endif
endmodule
