//==============================================================================
// o6_fpu_exu.sv - 浮点执行簇：P6 / P7 + I2F (来自 P3) + 浮点 PRF
//
// 运算单元用 OpenHW cvfpu (fpnew_top)，四个实例：
//   U6 (P6)：ADDMUL + NONCOMP (SGNJ/MINMAX/CMP/CLASS) + CONV (F2F/F2I)，各组都是 5 级流水
//   U7 (P7)：ADDMUL (5 级)
//   (500 MHz@ss：3 级时重定时后 FMA 一级仍 3.8 ns，加到 5 级让 DC 重定时把乘法树切开)
//   UD (P7)：DIVSQRT (THMULTI，可变延迟)，与 U7 共用 FW1，U7 优先，UD 靠 out_ready 反压
//            O6_UD_HALF (流片配置)：UD 跑在 ICG 门控出的半速时钟上 (C910 SRT 一次迭代 2.8 ns，
//            主频下放不下)。请求 / 冲刷先进保持寄存器，在 UD 时钟沿前一拍交付；结果只在那一拍取走
//   UI     ：CONV 只做 I2F (操作数来自整数 P3 的 EX 级)
//   Zfa    ：o6_zfa (fli / fminm / fmaxm / fround / fcvtmod / fleq / fltq) 是 U6 旁边同延迟 (5 拍) 的旁路流水，
//            占 U6 的槽、带同样的 tag，在 U6 输出处合并 (P6 每拍只进一条，延迟相同就不会同拍出)
// 同一实例内各运算组延迟必须相同：fpnew 内部输出仲裁一旦让某组等待，该组 in_ready 会变低，
// 而发射侧不看 in_ready (固定发射)，输入会被丢掉 —— 所以可变延迟的 DIVSQRT 单独成实例。
// 另外 P7 还发射 FSTD (浮点 store 数据，不进 fpnew)，P6 的 FMV.X.* 本地完成。
//
// fpnew 是握手 + 可变延迟 (各运算组输出仲裁)，所以：
//   - 唤醒在结果出来那一拍发 (进入 wbreg)，不做预测唤醒
//   - 分支作废：fpnew 里的流水不能按 br_mask 删，改为"槽位表"——每个在飞运算占一个槽，
//     tag 里只带槽号；槽位的 br_mask 每拍清解析位，被 kill 的槽标成僵尸，结果出来时丢弃
//   - 异常冲刷：fpnew 有 flush_i，槽位表清空
//
// 结果去向：浮点结果写 FW0(U6)/FW1(U7)/FW4(UI)；F2I / CMP / CLASS / FMV.X 结果走整数 W6
// (int_exu 给 F2I 最高优先级，所以这里不用反压)。
//==============================================================================
`default_nettype none
module o6_fpu_exu
  import o6_pkg::*;
(
  input  wire                 clk_i,
  input  wire                 rst_ni,
  input  wire                 flush_i,
  input  var br_update_t      br_upd_i,
  input  wire  [2:0]          frm_i,

  // 发射 (IQ_F：P6/P7)
  input  wire  [1:0]          iss_valid_i,
  input  var uop_t [1:0]      iss_uop_i,
  output logic [15:0]         fu_ok_o [2],

  // I2F (整数 P3 的 EX 级)
  input  wire                 i2f_valid_i,
  input  var uop_t            i2f_uop_i,
  input  wire  [63:0]         i2f_data_i,
  output wire                 i2f_ready_o,

  // 整数 P3 的浮点读口 (向量指令的标量浮点操作数，ROB 头才发射，不旁路)
  input  var preg_t           xrd_addr_i,
  output logic [63:0]         xrd_data_o,

  // LSU 各 load 流水的浮点 load 写回 (FW3..)
  input  var wb_t [N_LDP-1:0] fld_wb_i,

  // 全部浮点写回 (旁路源 / PRF 写口)、唤醒、完成
  output wb_t  [N_FWB-1:0]    fwb_o,
  output wakeup_t [2:0]       fwake_o,          // P6 / P7 / I2F
  output cmpl_t [2:0]         cmpl_o,           // P6 / P7 / I2F

  // F2I -> 整数 W6
  output logic                f2i_valid_o,
  output uop_t                f2i_uop_o,
  output logic [63:0]         f2i_data_o,
  output logic [4:0]          f2i_fflags_o,
  output wire                 f2i_occ_o,       // 本拍 W6 口被 F2I 路径占用 (含本拍刚被杀的，早到)
  output wakeup_t             f2i_wk_o,        // F2I 路径的唤醒 (早到；被杀的多唤醒无害)

  // FSTD -> SQ / ROB
  output logic                std_valid_o,
  output sq_ptr_t             std_sq_o,
  output rob_ptr_t            std_rob_o,
  output logic [63:0]         std_data_o
);
  localparam int NSLOT = 16;      // >= 每实例最多在飞运算数 (5 级 fpnew + RF/EX + 输出寄存)
  localparam int SW    = 4;
  localparam int TW    = 2 + $bits(preg_t) + SW;   // fpnew 的 tag：{rd 是浮点, rd 是整数, pdst, 槽号}

  //==========================================================================
  // 浮点 PRF：7 读 (P6 x3, P7 x3, 整数 P3 x1)，5 写 (FW0..FW4)
  //==========================================================================
  preg_t [6:0]      prf_ra;
  logic [63:0]      prf_rd [7];
  logic [N_FWB-1:0] prf_we;
  preg_t [N_FWB-1:0] prf_wa;
  logic [63:0]      prf_wd [N_FWB];

  always_comb begin
    for (int w = 0; w < N_FWB; w++) begin
      prf_we[w] = fwb_o[w].valid && fwb_o[w].wen;
      prf_wa[w] = fwb_o[w].pdst;
      prf_wd[w] = fwb_o[w].data;
    end
  end

  assign prf_ra[6]  = xrd_addr_i;
  assign xrd_data_o = prf_rd[6];
  o6_prf #(.N(N_FPREG), .NR(7), .NW(N_FWB), .ZERO_P0(1'b0)) u_fprf (
    .clk_i, .raddr_i(prf_ra), .rdata_o(prf_rd), .wen_i(prf_we), .waddr_i(prf_wa), .wdata_i(prf_wd)
  );

  function automatic logic [63:0] byp(input preg_t p, input logic [63:0] v, input wb_t [N_FWB-1:0] wb);
    logic [63:0] r;
    r = v;
    for (int k = 0; k < N_FWB; k++)
      if (wb[k].valid && wb[k].wen && wb[k].pdst == p) r = wb[k].data;
    return r;
  endfunction

  //==========================================================================
  // RF / EX 级 (P6, P7)
  //==========================================================================
  logic [1:0]  rf_v, ex_v;
  uop_t        rf_u [2], ex_u [2];
  logic [63:0] ex_o0 [2][3];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin rf_v <= '0; ex_v <= '0; end
    else if (flush_i) begin rf_v <= '0; ex_v <= '0; end
    else for (int p = 0; p < 2; p++) begin
      rf_v[p] <= iss_valid_i[p] && !br_killed(iss_uop_i[p].br_mask, br_upd_i);
      ex_v[p] <= rf_v[p] && !br_killed(rf_u[p].br_mask, br_upd_i);
    end
  end

  logic [63:0] rf_op [2][3];
  always_comb begin
    for (int p = 0; p < 2; p++) begin
      prf_ra[3*p]   = rf_u[p].psrc1;
      prf_ra[3*p+1] = rf_u[p].psrc2;
      prf_ra[3*p+2] = rf_u[p].psrc3;
      rf_op[p][0] = byp(rf_u[p].psrc1, prf_rd[3*p],   fwb_o);
      rf_op[p][1] = byp(rf_u[p].psrc2, prf_rd[3*p+1], fwb_o);
      rf_op[p][2] = byp(rf_u[p].psrc3, prf_rd[3*p+2], fwb_o);
    end
  end

  always_ff @(posedge clk_i) begin
    for (int p = 0; p < 2; p++) begin
      uop_t n1, n2;
      n1 = iss_uop_i[p]; n1.br_mask = iss_uop_i[p].br_mask & ~br_upd_i.resolve_mask;
      n2 = rf_u[p];      n2.br_mask = rf_u[p].br_mask      & ~br_upd_i.resolve_mask;
      rf_u[p] <= n1;
      ex_u[p] <= n2;
      for (int j = 0; j < 3; j++) ex_o0[p][j] <= rf_op[p][j];
    end
  end

  // ex_ok：在 EX 且未冲刷 (不看分支杀)；ex_kill：本拍被分支杀。进 fpnew / FMV 队列只看 ex_ok，
  // 被杀的照样进、标成僵尸 (槽 live = 0 / 队项 dead = 1)，这样 kill_tag 不经过 fpnew 的握手与队列计数 (6 GHz：b180ud in 组 −31)
  logic [1:0]  ex_ok, ex_kill, ex_live;
  logic [63:0] ex_op [2][3];
  always_comb begin
    for (int p = 0; p < 2; p++) begin
      ex_ok[p]   = ex_v[p] && !flush_i;
      ex_kill[p] = br_killed(ex_u[p].br_mask, br_upd_i);
      ex_live[p] = ex_ok[p] && !ex_kill[p];
      ex_op[p][0] = byp(ex_u[p].psrc1, ex_o0[p][0], fwb_o);
      ex_op[p][1] = byp(ex_u[p].psrc2, ex_o0[p][1], fwb_o);
      ex_op[p][2] = byp(ex_u[p].psrc3, ex_o0[p][2], fwb_o);
    end
  end

  //==========================================================================
  // 槽位表 (每个 fpnew 实例一份)
  //==========================================================================
  // 0 = U6, 1 = U7, 2 = UI, 3 = UD
  localparam int NU = 4;
  typedef struct packed {
    logic     live;     // 结果有效 (没被作废)
    logic     busy;     // 槽被占用 (含僵尸)
    uop_t     u;
  } slot_t;

  slot_t           slot [NU][NSLOT];
  logic [SW-1:0]   free_idx [NU];
  logic            free_ok  [NU];

  always_comb begin
    for (int x = 0; x < NU; x++) begin
      free_idx[x] = '0; free_ok[x] = 1'b0;
      for (int s = NSLOT-1; s >= 0; s--)
        if (!slot[x][s].busy) begin free_idx[x] = SW'(s); free_ok[x] = 1'b1; end
    end
  end

  //==========================================================================
  // 运算映射
  //==========================================================================
  function automatic fpnew_pkg::roundmode_e rm_of(input logic [2:0] rm, input logic [2:0] frm);
    return fpnew_pkg::roundmode_e'((rm == 3'b111) ? frm : rm);
  endfunction

  typedef struct packed {
    fpnew_pkg::operation_e  op;
    logic                   mod;
    fpnew_pkg::roundmode_e  rnd;
    fpnew_pkg::fp_format_e  src_fmt;
    fpnew_pkg::fp_format_e  dst_fmt;
    fpnew_pkg::int_format_e int_fmt;
    logic [2:0][63:0]       opnd;
  } fpreq_t;

  function automatic fpreq_t map(input uop_t u, input logic [63:0] a, input logic [63:0] b,
                                 input logic [63:0] c, input logic [2:0] frm);
    fpreq_t r;
    fpnew_pkg::fp_format_e f;
    f = u.fp_dbl ? fpnew_pkg::FP64 : fpnew_pkg::FP32;
    r.op = fpnew_pkg::FMADD; r.mod = 1'b0; r.rnd = rm_of(u.rm, frm);
    r.src_fmt = f; r.dst_fmt = f; r.int_fmt = fpnew_pkg::INT64;
    r.opnd = {c, b, a};
    unique case (u.fu)
      FU_FMA: begin
        unique case (u.op)
          FMA_ADD:   begin r.op = fpnew_pkg::ADD;    r.opnd = {b, a, 64'd0}; end
          FMA_SUB:   begin r.op = fpnew_pkg::ADD;    r.mod = 1'b1; r.opnd = {b, a, 64'd0}; end
          FMA_MUL:   begin r.op = fpnew_pkg::MUL;    r.opnd = {64'd0, b, a}; end
          FMA_MADD:  r.op = fpnew_pkg::FMADD;
          FMA_MSUB:  begin r.op = fpnew_pkg::FMADD;  r.mod = 1'b1; end
          FMA_NMSUB: r.op = fpnew_pkg::FNMSUB;
          default:   begin r.op = fpnew_pkg::FNMSUB; r.mod = 1'b1; end   // FNMADD
        endcase
      end
      FU_FDIV: begin
        r.op = (u.op == FDIV_SQRT) ? fpnew_pkg::SQRT : fpnew_pkg::DIV;
        r.opnd = {64'd0, b, a};
      end
      FU_FMISC: begin
        r.opnd = {64'd0, b, a};
        unique case (u.op)
          FMISC_SGNJ:  begin r.op = fpnew_pkg::SGNJ;   r.rnd = fpnew_pkg::RNE; end
          FMISC_SGNJN: begin r.op = fpnew_pkg::SGNJ;   r.rnd = fpnew_pkg::RTZ; end
          FMISC_SGNJX: begin r.op = fpnew_pkg::SGNJ;   r.rnd = fpnew_pkg::RDN; end
          FMISC_MIN:   begin r.op = fpnew_pkg::MINMAX; r.rnd = fpnew_pkg::RNE; end
          FMISC_MAX:   begin r.op = fpnew_pkg::MINMAX; r.rnd = fpnew_pkg::RTZ; end
          FMISC_CVTSH: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP16;    r.dst_fmt = fpnew_pkg::FP32; end
          FMISC_CVTHS: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP32;    r.dst_fmt = fpnew_pkg::FP16; end
          FMISC_CVTDH: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP16;    r.dst_fmt = fpnew_pkg::FP64; end
          FMISC_CVTHD: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP64;    r.dst_fmt = fpnew_pkg::FP16; end
          FMISC_CVTSB: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP16ALT; r.dst_fmt = fpnew_pkg::FP32; end
          FMISC_CVTBS: begin r.op = fpnew_pkg::F2F; r.src_fmt = fpnew_pkg::FP32;    r.dst_fmt = fpnew_pkg::FP16ALT; end
          default: begin   // FCVT.S.D / FCVT.D.S：fp_dbl 是目的格式
            r.op = fpnew_pkg::F2F;
            r.src_fmt = u.fp_dbl ? fpnew_pkg::FP32 : fpnew_pkg::FP64;
          end
        endcase
      end
      FU_F2I: begin
        r.opnd = {64'd0, b, a};
        unique case (u.op)
          F2I_EQ:    begin r.op = fpnew_pkg::CMP; r.rnd = fpnew_pkg::RDN; end
          F2I_LT:    begin r.op = fpnew_pkg::CMP; r.rnd = fpnew_pkg::RTZ; end
          F2I_LE:    begin r.op = fpnew_pkg::CMP; r.rnd = fpnew_pkg::RNE; end
          F2I_CLASS: r.op = fpnew_pkg::CLASSIFY;
          default: begin
            r.op = fpnew_pkg::F2I;
            r.mod = (u.op == F2I_CVTWU) || (u.op == F2I_CVTLU);
            r.int_fmt = (u.op == F2I_CVTW || u.op == F2I_CVTWU) ? fpnew_pkg::INT32 : fpnew_pkg::INT64;
          end
        endcase
      end
      FU_I2F: begin
        r.op = fpnew_pkg::I2F;
        r.opnd = {64'd0, 64'd0, a};
        r.mod = (u.op == I2F_CVTWU) || (u.op == I2F_CVTLU);
        r.int_fmt = (u.op == I2F_CVTW || u.op == I2F_CVTWU) ? fpnew_pkg::INT32 : fpnew_pkg::INT64;
      end
      default: ;
    endcase
    return r;
  endfunction

  //==========================================================================
  // 进入各单元
  //==========================================================================
  // P6：FMV.X.* 本地完成 (不进 fpnew)，其余进 U6
  wire        e6_fmv  = ex_u[0].fu == FU_F2I && (ex_u[0].op == F2I_MV || ex_u[0].op == F2I_MVH);
  wire        p6_fmv  = ex_ok[0] && e6_fmv;      // 进 FMV 队列 (含本拍被杀的：dead = 1)
  wire        e6_zfa  = (ex_u[0].fu == FU_FMISC && ex_u[0].op >= FMISC_FLI) ||
                        (ex_u[0].fu == FU_F2I   && ex_u[0].op >= F2I_CVTMOD);
  wire        zfa_in_v = ex_ok[0] && e6_zfa;
  wire        u6_in_v = ex_ok[0] && !e6_fmv && !e6_zfa;
  wire        p7_std  = ex_live[1] && ex_u[1].fu == FU_STD;
  wire        p7_div  = ex_live[1] && ex_u[1].fu == FU_FDIV;
  wire        u7_in_v = ex_ok[1] && ex_u[1].fu != FU_STD && ex_u[1].fu != FU_FDIV;
  // I2F：FMV.*.X 本地完成
  wire        i2f_kill = br_killed(i2f_uop_i.br_mask, br_upd_i);
  wire        i2f_mv  = i2f_valid_i && (i2f_uop_i.op == I2F_MV || i2f_uop_i.op == I2F_MVH);
  wire        ui_in_v = i2f_valid_i && !i2f_mv;
  // 各实例本拍进入的 uop 是否刚被杀 (UD 由 p7_div 进，已排除被杀的)
  logic       in_kill [4];
  assign in_kill[0] = ex_kill[0];
  assign in_kill[1] = ex_kill[1];
  assign in_kill[2] = i2f_kill;
  assign in_kill[3] = 1'b0;

  fpreq_t     rq6, rq7, rqi;
  assign rq6 = map(ex_u[0], ex_op[0][0], ex_op[0][1], ex_op[0][2], frm_i);
  assign rq7 = map(ex_u[1], ex_op[1][0], ex_op[1][1], ex_op[1][2], frm_i);
  assign rqi = map(i2f_uop_i, i2f_data_i, 64'd0, 64'd0, frm_i);

  logic        u_in_v  [NU];
  logic        u_in_rdy[NU];
  logic        u_out_v [NU];
  // UD 时钟域：ud_en = 1 表示本拍末是 UD 的时钟沿
  logic        ud_en, clk_ud;
`ifdef O6_UD_HALF
  // 分频比 UD_DIV (缺省 3)：2nm 6 GHz 下 C910 radix-16 SRT 一次迭代 ~350 ps，2T (扣不确定度 ~323 ps) 放不下 (b180ud)，
  // 改 3T。请求保持 / 冲刷保持 / 结果只在 ud_en 拍取走这套握手与分频比无关；综合侧生成时钟沿 {1 2 2N+1} (UD_DIV 环境变量)
`ifdef O6_UD_DIV
  localparam int UD_DIV = `O6_UD_DIV;
`else
  localparam int UD_DIV = 3;
`endif
  logic [1:0] ud_cnt;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) ud_cnt <= '0;
    else         ud_cnt <= (ud_cnt == 2'(UD_DIV - 1)) ? 2'd0 : ud_cnt + 2'd1;
  assign ud_en = ud_cnt == 2'(UD_DIV - 1);
  o6_icg u_ud_icg (.clk_i, .en_i(ud_en), .te_i(1'b0), .clk_o(clk_ud));
`else
  assign ud_en  = 1'b1;
  assign clk_ud = clk_i;
`endif
  // UD 请求保持：发射那拍记下 (操作数 / 槽号)，在 UD 时钟沿收下后清掉；冲刷也保持到下一个 UD 沿
  logic          udh_v, udh_fl;
  fpreq_t        udh_rq;
  logic [TW-1:0] udh_tag;
  logic        u_out_rdy[NU];
  logic        u_fire  [NU];       // 结果真正离开 fpnew (out_valid && out_ready)
  logic [63:0] u_res   [NU];
  fpnew_pkg::status_t u_st [NU];
  logic [TW-1:0] u_tag [NU];      // {rd 是浮点, rd 是整数, pdst, 槽号}
  logic [SW-1:0] u_sl  [NU];      // 槽号
  uop_t          in_u  [NU];      // 各实例本拍输入的 uop
  logic        u_busy  [NU];
  // fpnew 本身的输出 (U6 的再与 Zfa 旁路合并成 u_*)
  logic          f_out_v [NU];
  logic [63:0]   f_res   [NU];
  fpnew_pkg::status_t f_st [NU];
  logic [TW-1:0] f_tag   [NU];

  assign u_in_v[0] = u6_in_v;
  assign u_in_v[1] = u7_in_v;
  assign u_in_v[2] = ui_in_v;
  assign u_in_v[3] = udh_v;
  // FW1 上 U7 优先：U7 有结果出来的那拍 UD 等待
  assign u_out_rdy[0] = 1'b1;
  assign u_out_rdy[1] = 1'b1;
  assign u_out_rdy[2] = 1'b1;
  assign u_out_rdy[3] = !u_out_v[1] && ud_en;      // 只在 UD 时钟沿前一拍取走 (UD 同沿出队)
  for (genvar x = 0; x < NU; x++) begin : g_fire
    assign u_fire[x] = u_out_v[x] && u_out_rdy[x];
  end
  // FMV.*.X 本地结果队列 (与 UI 转换结果争 FW4)；到 2 项就停发 FU_I2F
  // 2026-10-17：整数侧加 R1 后，发射 -> EX 共 3 拍，ready 判定时最多 4 条在路上 (原 3 条)：
  //   4 项、ready = n<=1 会溢出，改 8 项、ready = n<=3 (最坏 3 + 4 + 向量 1 = 8)
  uop_t        ivq_u [8];
  logic [63:0] ivq_d [8];
  logic [2:0]  ivq_h, ivq_t;
  logic [3:0]  ivq_n;
  logic [7:0]  ivq_dead;      // 已被分支杀、还没走到队头的项 (见 mvq_dead)
  assign i2f_ready_o = (ivq_n <= 4'd3);

  localparam fpnew_pkg::fpu_features_t FEAT_U6 = '{
    Width:         64,
    EnableVectors: 1'b0,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b11101,                    // FP32 FP64 FP16 - FP16ALT (FP16 / BF16 只给 Zfhmin / Zfbfmin 的转换)
    IntFmtMask:    4'b0011
  };
  localparam fpnew_pkg::fpu_implementation_t IMPL_U6 = '{
    PipeRegs:   '{'{5, 5, 0, 0, 0},     // ADDMUL
                  '{0, 0, 0, 0, 0},     // DIVSQRT
                  '{5, 5, 0, 0, 0},     // NONCOMP (与 ADDMUL 同延迟，避免输出仲裁反压)
                  '{5, 5, 0, 0, 0}},    // CONV
    UnitTypes:  '{'{fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::MERGED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  localparam fpnew_pkg::fpu_implementation_t IMPL_U7 = '{
    PipeRegs:   '{'{5, 5, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}},
    UnitTypes:  '{'{fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  localparam fpnew_pkg::fpu_implementation_t IMPL_UD = '{
    PipeRegs:   '{'{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}},
    UnitTypes:  '{'{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::MERGED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  localparam fpnew_pkg::fpu_implementation_t IMPL_UI = '{
    // CONV 3 -> 4 级 (2026-10-28，6 GHz：p155u59 r2r −15.4，重定时后 舍入 -> 输出 -> fw4 选择 167 ps)；
    // 唤醒取 fpnew 输出的 tag，与级数无关；I2F 转换 (FCVT.*.W/L) 少见，多 1 拍无关紧要
    PipeRegs:   '{'{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{4, 4, 0, 0, 0}},
    UnitTypes:  '{'{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::MERGED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };

  fpreq_t rq [NU];
  assign rq[0] = rq6;
  assign rq[1] = rq7;
  assign rq[2] = rqi;
  assign rq[3] = udh_rq;

  for (genvar x = 0; x < NU; x++) begin : g_fpu
    localparam fpnew_pkg::fpu_implementation_t IMPL = (x == 0) ? IMPL_U6 : (x == 1) ? IMPL_U7 :
                                                      (x == 2) ? IMPL_UI : IMPL_UD;
    fpnew_top #(
      .Features       ((x == 0) ? FEAT_U6 : fpnew_pkg::RV64D),
      .Implementation (IMPL),
      .DivSqrtSel     (fpnew_pkg::THMULTI),    // PULP 的 mvp 除法器特殊值不合 IEEE (负数开方返回 0)，不能用
      .TagType        (logic [TW-1:0])
    ) u_fpnew (
      .clk_i         ((x == 3) ? clk_ud : clk_i),
      .rst_ni,
      .operands_i    (rq[x].opnd),
      .rnd_mode_i    (rq[x].rnd),
      .op_i          (rq[x].op),
      .op_mod_i      (rq[x].mod),
      .src_fmt_i     (rq[x].src_fmt),
      .dst_fmt_i     (rq[x].dst_fmt),
      .int_fmt_i     (rq[x].int_fmt),
      .vectorial_op_i(1'b0),
      // U6 (x = 0) 的整数侧位 = 「不是浮点目的」：rd = x0 的 FCMP / FCLASS / F2I (rd_class = RC_NONE) 也要经 W6 完成，
      //   否则结果两边都不收、ROB 永远等不到 (#31，riscv-arch-test 的 feq/flt/fle/fclass/fcvt.* rd = x0 用例)；
      //   此时 pdst 填 0 (p0 是 x0 的固定映射，从不分配)：F2I 早到唤醒 p0 无害，W6 写回的 wen 仍按 RC_INT 门控
      .tag_i         ((x == 3) ? udh_tag :
                      {in_u[x].rd_class == RC_FP,
                       (x == 0) ? (in_u[x].rd_class != RC_FP) : (in_u[x].rd_class == RC_INT),
                       (in_u[x].rd_class == RC_NONE) ? preg_t'(0) : in_u[x].pdst, free_idx[x]}),
      .simd_mask_i   (1'b1),
      .in_valid_i    (u_in_v[x]),
      .in_ready_o    (u_in_rdy[x]),
      .flush_i       ((x == 3) ? (flush_i || udh_fl) : flush_i),
      .result_o      (f_res[x]),
      .status_o      (f_st[x]),
      .tag_o         (f_tag[x]),
      .out_valid_o   (f_out_v[x]),
      .out_ready_i   (u_out_rdy[x]),
      .busy_o        (u_busy[x]),
      .early_valid_o ()
    );
  end

  //==========================================================================
  // Zfa 旁路：入口寄存 (同 fpnew 第一级) -> 运算 -> 再 4 级 = 5 拍，与 IMPL_U6 一致
  //==========================================================================
  localparam int ZD = 5;
  logic [ZD-1:0] zq_v;
  logic [TW-1:0] zq_tag [ZD];
  logic [63:0]   z_res;               // 入口后第 ZD 拍
  logic [4:0]    z_fl;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)      zq_v <= '0;
    else if (flush_i) zq_v <= '0;
    else              zq_v <= {zq_v[ZD-2:0], zfa_in_v};
  end
  always_ff @(posedge clk_i) begin
    if (zfa_in_v)
      zq_tag[0] <= {ex_u[0].rd_class == RC_FP, ex_u[0].rd_class != RC_FP,
                    (ex_u[0].rd_class == RC_NONE) ? preg_t'(0) : ex_u[0].pdst, free_idx[0]};
    for (int k = 1; k < ZD; k++) zq_tag[k] <= zq_tag[k-1];
  end
  o6_zfa_pipe #(.D(ZD)) u_zfa (.clk_i, .op_i(ex_u[0].op), .f2i_i(ex_u[0].fu == FU_F2I), .fmt_i(ex_u[0].raw[26:25]),
                .rm_i((ex_u[0].rm == 3'b111) ? frm_i : ex_u[0].rm), .idx_i(ex_u[0].raw[19:15]),
                .a_i(ex_op[0][0]), .b_i(ex_op[0][1]), .res_o(z_res), .flags_o(z_fl));

  // U6 输出合并
  wire z_out = zq_v[ZD-1];
  always_comb begin
    for (int x = 0; x < NU; x++) begin
      u_out_v[x] = f_out_v[x]; u_res[x] = f_res[x]; u_st[x] = f_st[x]; u_tag[x] = f_tag[x];
    end
    u_out_v[0] = f_out_v[0] || z_out;
    if (z_out) begin
      u_res[0] = z_res; u_st[0] = fpnew_pkg::status_t'(z_fl); u_tag[0] = zq_tag[ZD-1];
    end
  end
`ifndef SYNTHESIS
  always @(posedge clk_i) if (rst_ni && f_out_v[0] && z_out)
    $error("o6_fpu_exu: U6 与 Zfa 旁路同拍出结果 (延迟不一致)");
`endif

  // FMV.X 本地结果队列 (定义见 F2I 路径)
  uop_t        mvq_u [4];
  logic [63:0] mvq_d [4];
  logic [1:0]  mvq_h, mvq_t;
  logic [2:0]  mvq_n;
  // 已被分支杀、还没走到队头的项：误预测广播只有一拍，队头之外的项错过就再也认不出
  // (同拍 resolve 还把 kill 位从 br_mask 清掉)，之后会照常出队，写回已回收的 pdst、给 ROB 报完成 (R2 r2fp20331)
  logic [3:0]  mvq_dead;

  //==========================================================================
  // 发射许可：有空槽；除法/开方单元一次只放一条
  //==========================================================================
  logic div_inflight;
  logic [1:0] fok_q;           // 各口空槽数够 (寄存，见 fu_ok)
  always_comb begin
    fu_ok_o[0] = '0; fu_ok_o[1] = '0;
    // 留余量：RF/EX 两级里可能各有一条还没占槽；空槽数寄存一拍 (16 项计数 -> fu_ok -> IQ 选择太长)，
    // 期间每口最多再发一条，所以阈值再加 1
    if (fok_q[0]) begin
      fu_ok_o[0][FU_FMA] = 1'b1; fu_ok_o[0][FU_FMISC] = 1'b1;
      fu_ok_o[0][FU_F2I] = (mvq_n <= 3'd1);
    end
    fu_ok_o[1][FU_STD] = 1'b1;
    if (fok_q[1]) fu_ok_o[1][FU_FMA] = 1'b1;
    // UD 一次只放一条 (RF/EX 级在途的也算)
    fu_ok_o[1][FU_FDIV] = !div_inflight && !(rf_v[1] && rf_u[1].fu == FU_FDIV) && !(ex_v[1] && ex_u[1].fu == FU_FDIV);
  end

  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) fok_q <= 2'b11;
    else         fok_q <= {free_cnt(1) > 3, free_cnt(0) > 3};

  function automatic int free_cnt(input int x);
    int n;
    n = 0;
    for (int s = 0; s < NSLOT; s++) n += !slot[x][s].busy;
    return n;
  endfunction

  //==========================================================================
  // 槽位表更新
  //==========================================================================
  assign in_u[0] = ex_u[0];
  assign in_u[1] = ex_u[1];
  assign in_u[2] = i2f_uop_i;
  assign in_u[3] = ex_u[1];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      udh_v <= 1'b0; udh_fl <= 1'b0;
    end else begin
      if (flush_i)                      udh_v <= 1'b0;
      else if (p7_div)                  udh_v <= 1'b1;
      else if (ud_en && u_in_rdy[3])    udh_v <= 1'b0;
      udh_fl <= (flush_i || udh_fl) && !ud_en;
    end
  end
  always_ff @(posedge clk_i)
    if (p7_div) begin udh_rq <= rq7; udh_tag <= {ex_u[1].rd_class == RC_FP, ex_u[1].rd_class == RC_INT, ex_u[1].pdst, free_idx[3]}; end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int x = 0; x < NU; x++) for (int s = 0; s < NSLOT; s++) begin slot[x][s].busy <= 1'b0; slot[x][s].live <= 1'b0; end
      div_inflight <= 1'b0;
    end else if (flush_i) begin
      for (int x = 0; x < NU; x++) for (int s = 0; s < NSLOT; s++) begin slot[x][s].busy <= 1'b0; slot[x][s].live <= 1'b0; end
      div_inflight <= 1'b0;
    end else begin
      for (int x = 0; x < NU; x++) begin
        for (int s = 0; s < NSLOT; s++) begin
          // 分支广播
          if (slot[x][s].busy && br_killed(slot[x][s].u.br_mask, br_upd_i)) slot[x][s].live <= 1'b0;
          slot[x][s].u.br_mask <= slot[x][s].u.br_mask & ~br_upd_i.resolve_mask;
        end
        // 出
        if (u_fire[x]) begin slot[x][u_sl[x]].busy <= 1'b0; slot[x][u_sl[x]].live <= 1'b0; end
        // 进 (同拍出进同一槽不会发生：进用的是当前空槽)
        if ((x == 3) ? p7_div : ((u_in_v[x] && u_in_rdy[x]) || (x == 0 && zfa_in_v))) begin
          uop_t n;
          n = in_u[x]; n.br_mask = in_u[x].br_mask & ~br_upd_i.resolve_mask;
          slot[x][free_idx[x]].busy <= 1'b1;
          slot[x][free_idx[x]].live <= !in_kill[x];
          slot[x][free_idx[x]].u    <= n;
        end
      end
      if (p7_div)                   div_inflight <= 1'b1;
      if (u_fire[3])                div_inflight <= 1'b0;
    end
  end

  //==========================================================================
  // 输出
  //==========================================================================
  // 浮点结果有效 (结果出来且槽未被作废，且本拍广播也没把它作废)
  logic  o_live [NU];
  uop_t  o_u    [NU];
  always_comb begin
    for (int x = 0; x < NU; x++) begin
      o_u[x]    = slot[x][u_sl[x]].u;
      o_live[x] = u_fire[x] && slot[x][u_sl[x]].live && !br_killed(o_u[x].br_mask, br_upd_i) && !flush_i;
    end
  end

  // FW4：UI 结果出来 (槽位 live，不论是否刚被杀) 就由它占用，FMV 队列让一拍 —— 与唤醒的选择一致
  wire ui_go    = u_fire[2] && slot[2][u_sl[2]].live;
  // 队头前进只看寄存的 dead：本拍刚被杀的队头若 W 口空着就当场丢 (pop 无效)，被 UI 占着就等下一拍按 dead 丢。
  // 进队也不看分支杀 (被杀的以 dead = 1 进)。这样 ivq_n / ivq_h 与 br_upd_i 无关
  wire ivq_hdead = ivq_dead[ivq_h] || br_killed(ivq_u[ivq_h].br_mask, br_upd_i);
  wire ivq_pop  = (ivq_n != 0) && !ui_go && !ivq_hdead;           // 写回有效
  wire ivq_adv  = (ivq_n != 0) && (!ui_go || ivq_dead[ivq_h]);     // 队头前进 (出队或丢弃)
  wire ivq_push = i2f_mv;

  // F2I 路径：U6 的 F2I/CMP/CLASS 结果优先；本地 FMV.X 结果先进 4 项队列，W6 口空时再出
  //   (两者同拍到达时 FMV 只能等；队列到 2 项就停发 FU_F2I，留出 RF/EX 两级在途的余量)
  // u6_go：U6 出了整数目的的结果 (槽位 live，不论本拍是否刚被杀) —— 它占住 W6 口，FMV 队列与
  // 整数侧的除法 / CSR 都让一拍。只看 tag 与槽位 live 位，不经槽位表的 uop 查找和分支杀判断，
  // 这样 W6 的占用与唤醒 pdst 很早就定 (f2i_occ_o / f2i_wk_o)
  wire         u6_go  = u_fire[0] && slot[0][u_sl[0]].live && u_tag[0][TW-2];
  wire         u6_int = u6_go && o_live[0];
  wire         mvq_hdead = mvq_dead[mvq_h] || br_killed(mvq_u[mvq_h].br_mask, br_upd_i);
  wire         mvq_pop = (mvq_n != 0) && !u6_go && !mvq_hdead;
  wire         mvq_adv = (mvq_n != 0) && (!u6_go || mvq_dead[mvq_h]);   // 同 ivq_adv：不看本拍分支杀

  assign f2i_occ_o = u6_go || mvq_pop;
  assign f2i_wk_o  = u6_go ? '{valid: 1'b1, is_fp: 1'b0, pdst: u_tag[0][SW +: $bits(preg_t)], sa: '0, sn: '0, sc: '0}
                           : '{valid: mvq_pop && mvq_u[mvq_h].rd_class == RC_INT, is_fp: 1'b0, pdst: mvq_u[mvq_h].pdst, sa: '0, sn: '0, sc: '0};

  // 有效位含本拍分支杀 (u6_int / mvq_pop)；uop / 数据 / 标志只在有效时有意义，按不含杀判断的 u6_go 选
  // (u6_go 时 mvq_pop 必为 0)，kill_tag 就不经选择直通到 f2i_uop_o (6 GHz：b180ud3k in 组 −40)。
  // CMP / CLASS 结果是 0/1 或类别位，已在低位；F2I 已按 RISC-V 符号扩展
  assign f2i_valid_o  = u6_int || mvq_pop;
  assign f2i_uop_o    = u6_go ? o_u[0]   : mvq_u[mvq_h];
  assign f2i_data_o   = u6_go ? u_res[0] : mvq_d[mvq_h];
  assign f2i_fflags_o = u6_go ? u_st[0]  : '0;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mvq_h <= '0; mvq_t <= '0; mvq_n <= '0; mvq_dead <= '0;
    end else if (flush_i) begin
      mvq_h <= '0; mvq_t <= '0; mvq_n <= '0; mvq_dead <= '0;
    end else begin
      logic [2:0] n;
      n = mvq_n;
      if (mvq_adv) begin mvq_h <= mvq_h + 1'b1; n = n - 1'b1; end
      if (p6_fmv) begin mvq_t <= mvq_t + 1'b1; n = n + 1'b1; end
      mvq_n <= n;
      for (int i = 0; i < 4; i++) if (br_killed(mvq_u[i].br_mask, br_upd_i)) mvq_dead[i] <= 1'b1;
      if (p6_fmv) mvq_dead[mvq_t] <= ex_kill[0];   // 本拍被杀的也进队、直接标死；队不会满，t 不是在队的项
    end
  end
  always_ff @(posedge clk_i) begin
    for (int i = 0; i < 4; i++) mvq_u[i].br_mask <= mvq_u[i].br_mask & ~br_upd_i.resolve_mask;
    if (p6_fmv) begin
      uop_t n;
      n = ex_u[0]; n.br_mask = ex_u[0].br_mask & ~br_upd_i.resolve_mask;
      mvq_u[mvq_t] <= n;
      mvq_d[mvq_t] <= (ex_u[0].op == F2I_MVH) ? {{48{ex_op[0][0][15]}}, ex_op[0][0][15:0]} :
                      ex_u[0].fp_dbl ? ex_op[0][0] : {{32{ex_op[0][0][31]}}, ex_op[0][0][31:0]};
    end
  end

  // FSTD
  assign std_valid_o = p7_std;
  assign std_sq_o    = ex_u[1].sq_idx;
  assign std_rob_o   = ex_u[1].rob_idx;
  // FSW 只存低 32 位 (SQ 按 size 截取)；不检查 NaN-boxing (RISC-V 规定原样存)
  assign std_data_o  = ex_op[1][1];

  // 写回寄存器：FW0 = U6，FW1 = U7，FW2/3 = LSU，FW4 = I2F
  wb_t fw0_n, fw1_n, fw4_n;
  always_comb begin
    fw0_n = '{valid: o_live[0] && o_u[0].rd_class == RC_FP, pdst: o_u[0].pdst, wen: 1'b1, data: u_res[0]};
    if (u_out_v[1])
      fw1_n = '{valid: o_live[1] && o_u[1].rd_class == RC_FP, pdst: o_u[1].pdst, wen: 1'b1, data: u_res[1]};
    else
      fw1_n = '{valid: o_live[3], pdst: o_u[3].pdst, wen: 1'b1, data: u_res[3]};
    if (ui_go)
      fw4_n = '{valid: o_live[2], pdst: o_u[2].pdst, wen: 1'b1, data: u_res[2]};
    else
      fw4_n = '{valid: ivq_pop, pdst: ivq_u[ivq_h].pdst, wen: 1'b1, data: ivq_d[ivq_h]};
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ivq_h <= '0; ivq_t <= '0; ivq_n <= '0; ivq_dead <= '0;
    end else if (flush_i) begin
      ivq_h <= '0; ivq_t <= '0; ivq_n <= '0; ivq_dead <= '0;
    end else begin
      logic [3:0] n;                         // 最多 8 项：3 位会把 8 回绕成 0
      n = ivq_n;
      if (ivq_adv) begin ivq_h <= ivq_h + 1'b1; n = n - 1'b1; end
      if (ivq_push) begin ivq_t <= ivq_t + 1'b1; n = n + 1'b1; end
      ivq_n <= n;
      for (int i = 0; i < 8; i++) if (br_killed(ivq_u[i].br_mask, br_upd_i)) ivq_dead[i] <= 1'b1;
      if (ivq_push) ivq_dead[ivq_t] <= i2f_kill;
    end
  end
  always_ff @(posedge clk_i) begin
    for (int i = 0; i < 8; i++) ivq_u[i].br_mask <= ivq_u[i].br_mask & ~br_upd_i.resolve_mask;
    if (ivq_push) begin
      uop_t n;
      n = i2f_uop_i; n.br_mask = i2f_uop_i.br_mask & ~br_upd_i.resolve_mask;
      ivq_u[ivq_t] <= n;
      ivq_d[ivq_t] <= (i2f_uop_i.op == I2F_MVH) ? {48'hFFFF_FFFF_FFFF, i2f_data_i[15:0]} :
                      i2f_uop_i.fp_dbl ? i2f_data_i : {32'hFFFF_FFFF, i2f_data_i[31:0]};
    end
  end

  wb_t fw0, fw1, fw4;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin fw0.valid <= 1'b0; fw1.valid <= 1'b0; fw4.valid <= 1'b0; end
    else if (flush_i) begin fw0.valid <= 1'b0; fw1.valid <= 1'b0; fw4.valid <= 1'b0; end
    else begin fw0.valid <= fw0_n.valid; fw1.valid <= fw1_n.valid; fw4.valid <= fw4_n.valid; end
  end
  always_ff @(posedge clk_i) begin
    fw0.pdst <= fw0_n.pdst; fw0.wen <= 1'b1; fw0.data <= fw0_n.data;
    fw1.pdst <= fw1_n.pdst; fw1.wen <= 1'b1; fw1.data <= fw1_n.data;
    fw4.pdst <= fw4_n.pdst; fw4.wen <= 1'b1; fw4.data <= fw4_n.data;
  end

  // FW0/FW1 = 两个浮点口，FW2 = I2F，FW3.. = FLD (旁路与 PRF 写口不分先后)
  assign fwb_o[0] = fw0;
  assign fwb_o[1] = fw1;
  assign fwb_o[2] = fw4;
  for (genvar k = 0; k < N_LDP; k++) begin : g_fld
    assign fwb_o[3+k] = fld_wb_i[k];
  end

  // 唤醒：结果进入 wbreg 的那一拍
  // 唤醒：pdst 直接取 fpnew 带出来的 tag；有效只看"结果出来 + 槽位仍 live"，不再查分支杀 / 冲刷
  // (多出来的唤醒只可能是本拍刚被杀的指令：它的消费者同样被杀，它的 pdst 下一拍才会被重新分配)。
  // 写回 (fw*_n) 仍做完整检查
  for (genvar x = 0; x < NU; x++) begin : g_sl
    assign u_sl[x] = u_tag[x][SW-1:0];
  end
  wire [NU-1:0] wk_v;
  for (genvar x = 0; x < NU; x++) begin : g_wk
    assign wk_v[x] = u_fire[x] && slot[x][u_sl[x]].live && u_tag[x][TW-1];
  end
  assign fwake_o[0] = '{valid: wk_v[0], is_fp: 1'b1, pdst: u_tag[0][SW +: $bits(preg_t)], sa: '0, sn: '0, sc: '0};
  assign fwake_o[1] = u_out_v[1] ? '{valid: wk_v[1], is_fp: 1'b1, pdst: u_tag[1][SW +: $bits(preg_t)], sa: '0, sn: '0, sc: '0}
                                 : '{valid: wk_v[3], is_fp: 1'b1, pdst: u_tag[3][SW +: $bits(preg_t)], sa: '0, sn: '0, sc: '0};
  assign fwake_o[2] = ui_go ? '{valid: 1'b1, is_fp: 1'b1, pdst: u_tag[2][SW +: $bits(preg_t)], sa: '0, sn: '0, sc: '0}
                              : '{valid: ivq_n != 4'd0 && !ivq_dead[ivq_h], is_fp: 1'b1, pdst: ivq_u[ivq_h].pdst, sa: '0, sn: '0, sc: '0};

  // 完成 (与 wbreg 同拍)：浮点结果、浮点 STD 由 std_done 报；F2I 的完成由 int_exu 的 W6 报
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) for (int x = 0; x < 3; x++) cmpl_o[x].valid <= 1'b0;
    else if (flush_i) for (int x = 0; x < 3; x++) cmpl_o[x].valid <= 1'b0;
    else begin
      cmpl_o[0].valid <= fw0_n.valid;
      cmpl_o[1].valid <= fw1_n.valid;
      cmpl_o[2].valid <= fw4_n.valid;
    end
  end
  always_ff @(posedge clk_i) begin
    cmpl_o[0].rob_idx <= o_u[0].rob_idx; cmpl_o[0].fflags <= u_st[0];
    cmpl_o[1].rob_idx <= u_out_v[1] ? o_u[1].rob_idx : o_u[3].rob_idx;
    cmpl_o[1].fflags  <= u_out_v[1] ? u_st[1] : u_st[3];
    cmpl_o[2].rob_idx <= o_live[2] ? o_u[2].rob_idx : ivq_u[ivq_h].rob_idx;
    cmpl_o[2].fflags  <= o_live[2] ? u_st[2] : 5'd0;
    for (int x = 0; x < 3; x++) begin
      cmpl_o[x].excp <= 1'b0; cmpl_o[x].cause <= '0; cmpl_o[x].tval <= '0; cmpl_o[x].flush_after <= 1'b0; cmpl_o[x].replay <= 1'b0;
    end
  end
`ifndef SYNTHESIS
  // 发射侧不看 in_ready：任何实例拒收都是设计错误 (见文件头说明)
  always @(posedge clk_i) if (rst_ni)
    for (int x = 0; x < NU; x++)
      if (u_in_v[x] && !u_in_rdy[x] && x != 3) $error("o6_fpu_exu: fpnew 实例 %0d 拒收输入 (运算被丢)", x);
`endif
endmodule
