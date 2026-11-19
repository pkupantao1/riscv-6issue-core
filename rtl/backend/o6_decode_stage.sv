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

  input  wire                     vfast_ok_i,    // 向量单元：vtype 合法、LMUL <= 1、vstart = 0 (快路径可用，docs/11 §9a)
  // O1 (docs/11 §9f)：VECU 当前的 vl / SEW / LMUL (= 提交态：vsetvl 在 ROB 头执行，改了就冲刷比它年轻的)
  input  wire  [10:0]             vvl_i,
  input  wire  [1:0]              vsew_i,
  input  wire  [2:0]              vlmul_i,
  input  wire                     vmok_i,        // M3：vtype 合法且 vstart = 0 (向量 load 可展开进共用 LSU)
  input  wire                     valt_i,        // V1.2b：vtype.altfmt = 1 时单宽度浮点运算非法 (不当候选，原路径报非法)
  input  wire  [31:0]             vpend_i,       // M3：有在途的不在核内改名的写的寄存器 (碰到目的组就不展开，走原路径)
  // I2a (docs/11 §9e)：上一条预查不过 replay 的向量 load 重取后要走原路径 (ROB 头同步)。vslow 时本拍译码的第一个候选不标 vel，
  //   被取走 (进 q0 / q1) 的那拍 vslow_clr 清掉
  input  wire                     vslow_i,
  output logic                    vslow_clr_o,
  // V5 快 vset：VECU 当前 {vill, vtype}；上一条快 vset replay 后 (vss) 本拍译码的 vset 走原路径，被取走那拍 vss_clr
  input  wire  [9:0]              vt_i,
  input  wire                     vss_i,
  output logic                    vss_clr_o,

  // 去重命名
  output logic [W-1:0]            out_valid_o,
  output uop_t [W-1:0]            out_uop_o,
  input  wire                     out_ready_i
);
  uop_t [W-1:0] dec0, dec, dec_n;
  logic [W-1:0] visv;               // V5：是 vset (vss 时都走原路径)
  logic [W-1:0] velc;               // vel 候选 (不看 vslow)
  logic [W-1:0] take, take_n;
  for (genvar s = 0; s < W; s++) begin : g_dec
    o6_decoder u_dec (
      .fi_i(ib_inst_i[s]), .priv_i, .fs_i, .vs_i, .frm_i, .cbo_ok_i, .cbo_vi_i, .virt_i, .hlv_ok_i, .hlv_vi_i, .uop_o(dec0[s])
    );
  end

  // move 消除：候选的源若由同组更早的另一个候选写，就不消除 (照常走 ALU)。
  // 这样重命名级被消除者的新映射 = RAT 读出或更早普通槽的新 pdst，组内旁路链不会因为 move 串长
  always_comb begin
    logic first;
    first = vslow_i;                  // 还没遇到第一个候选 (vslow 时它走原路径)
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
      // O1 核内乱序 (docs/11 §9f)：候选按译码当拍的 vtype / vl / vstart 分 (之后更老的 vsetvl / 写 vstart / ff 截 vl
      //   改了它们时，那条指令提交后冲刷，这里分的结果随之作废)；是否真走乱序由重命名级按在途写定 (vfast)。
      //   I1.2 的 ROB 头快路径不再使用 (vfast 只由重命名置)
      u.vfast = 1'b0;
      u.vsetf = 1'b0;
      // V5 快 vset：vsetvli / vsetivli 的 vtype 与当前相同 (vill = 0、vstart = 0)、VS 已脏、结果 vl 预计不变 -> 当 ALU 指令乱序做，
      //   不再在 ROB 头经 VECU 同步执行 (分段循环每轮省约 7 拍)。avl 是寄存器时 rd = min(avl, VLMAX) (ALU_MINU)，执行时 avl < VLMAX
      //   (vl 要变) 就 replay，重取后 (vss) 走原路径。更老的在途 vsetvl 改了 vtype / vl 时它提交后冲刷，这里按旧值判的也一起作废
      begin
        logic        isv, ivi;
        logic [10:0] z;
        logic [10:0] vlm, nvl;
        isv = u.raw[6:0] == 7'b1010111 && u.raw[14:12] == 3'b111 && !u.raw[31];
        ivi = u.raw[6:0] == 7'b1010111 && u.raw[14:12] == 3'b111 && u.raw[31:30] == 2'b11;
        visv[s] = isv || ivi || (u.raw[6:0] == 7'b1010111 && u.raw[14:12] == 3'b111);
        z   = isv ? u.raw[30:20] : {1'b0, u.raw[29:20]};
        vlm = vlmul_i[2] ? 11'((CL_B >> vsew_i) >> (4'd8 - 4'(vlmul_i))) : 11'((CL_B >> vsew_i) << vlmul_i[1:0]);
        if ((isv || ivi) && !u.excp && !vss_i && !vt_i[9] && z[10:9] == 2'b00 && z[8:0] == vt_i[8:0] && vs_i == 2'b11 && vmok_i) begin
          logic ok;
          logic [4:0] rd, rs1;
          rd = u.raw[11:7]; rs1 = u.raw[19:15];
          ok = 1'b1;
          u.op = ALU_ADD; u.vsetf = 1'b0; u.imm = 32'(vvl_i);
          if (ivi) begin
            nvl = (11'(rs1) < vlm) ? 11'(rs1) : vlm;
            ok  = (nvl == vvl_i);                              // rd = 常数 vl
            rs1 = 5'd0;
          end else if (rs1 == 5'd0) begin
            ok  = (rd == 5'd0) || (vvl_i == vlm);              // rd != 0：vl = VLMAX；rd = 0：保持 vl
          end else begin
            ok  = (vvl_i == vlm);                              // avl >= VLMAX 时 vl 不变
            u.op = ALU_MINU; u.vsetf = 1'b1; u.imm = 32'(vlm);
          end
          if (ok) begin
            u.fu = FU_ALU; u.use_imm = 1'b1; u.is_w = 1'b0; u.auw = 1'b0; u.ash = 2'd0;
            u.lrd = rd; u.rd_class = (rd != 5'd0) ? RC_INT : RC_NONE;
            u.lrs1 = rs1; u.rs1_class = RC_INT; u.rs2_class = RC_NONE;
            u.wait_head = 1'b0; u.flush_after = 1'b0; u.mvc = 1'b0; u.mvel = 1'b0;
          end else u.vsetf = 1'b0;
        end
      end
      // V1.2b：不再限 LMUL <= 1 —— 整条发射，组 vng 个寄存器 (小数 LMUL 按 1)；vd / vs2 / (.vv 的) vs1 要按组对齐
      begin
        logic [3:0] ng;
        logic       al;
        logic       dt, dok;
        ng = (vlmul_i[2] || vlmul_i == 3'd0) ? 4'd1 : (4'd1 << vlmul_i[1:0]);
        dt = vdot_op(u.raw);
        al = (dt || (u.raw[11:7] & 5'(ng - 4'd1)) == '0) && (u.raw[24:20] & 5'(ng - 4'd1)) == '0 &&
             (!(u.raw[14:12] == 3'b000 || u.raw[14:12] == 3'b001) || (u.raw[19:15] & 5'(ng - 4'd1)) == '0);
        // V4 点积：SEW = 8，vd (1 个) 不与 vs1 / vs2 组重叠 (否则 Spike 判非法，留给原路径)；浮点的另要 frm 合法、FS 不是 Off
        dok = vsew_i == 2'd0 &&
              (u.raw[11:7] < u.raw[19:15] || {1'b0, u.raw[11:7]} >= {1'b0, u.raw[19:15]} + 6'(ng)) &&
              (u.raw[11:7] < u.raw[24:20] || {1'b0, u.raw[11:7]} >= {1'b0, u.raw[24:20]} + 6'(ng)) &&
              (u.raw[14:12] == 3'b000 || (frm_i <= 3'd4 && fs_i != 2'b00));
        u.vng = dt ? 4'd1 : ng;
        u.vns = ng;
        u.vlm = vlmul_i;
        u.valt = valt_i;
        u.vfc = u.fu == FU_VEC && !u.excp && vmok_i && al && vooo_op(u.raw) && (!dt || dok) &&
                // V1.2a：浮点向量运算要 SEW != 8、frm 合法、FS 不是 Off (否则仍按原路径报非法指令)
                //   altfmt = 1 时 (SEW 16 按 BF16) 单宽度浮点运算非法 (v_cache 发现：vsetvl 设 altfmt 后的 vfadd 应陷入)
                (!vexf_op(u.raw) || (vsew_i != 2'd0 && frm_i <= 3'd4 && fs_i != 2'b00 && !valt_i));
        // V1.2c：.vx / .vf 的候选另派数据半边读 x / f[rs1] (rs2 的位置登记成 rs1；原路径仍按 rs1 读)
        u.vxs = u.vfc && (u.raw[14:12] == 3'b100 || u.raw[14:12] == 3'b101);
        if (u.vxs) begin u.rs2_class = u.rs1_class; u.lrs2 = u.lrs1; end
        // V3：不读 vd 旧值 —— 不带掩码、LMUL >= 1 且 vl = VLMAX (没有尾部 / 被掩掉的元素)，又不是浮点乘加 (OPFVV / OPFVF 101xxx)
        u.vnod = u.vfc && !dt && u.raw[25] && !vlmul_i[2] &&
                 (32'(vvl_i) == ((32'(CL_B) >> vsew_i) << vlmul_i[1:0])) &&
                 !((u.raw[14:12] == 3'b001 || u.raw[14:12] == 3'b101) && u.raw[31:29] == 3'b101);
      end
      u.vvl   = vvl_i; u.vsew = vsew_i;
      u.vwd   = u.raw[11:7];
      begin
        logic [4:0] n;
        n = vwr_n(u.raw, vsew_i, vlmul_i);
        u.vnw = VOOO && u.fu == FU_VEC && !u.excp && (n != 5'd0);
        u.vwn = (n > 5'd8) ? 4'd8 : n[3:0];
      end
      // I1.3b：纯向量运算提交时送 VECU (docs/11 §9d)，不进发射队列、分派即完成 (乱序的由重命名改成等 VX 完成)
      u.vea = VEARLY && u.fu == FU_VEC && !u.excp && vea_op(u.raw);
      if (u.vea) begin u.fu = FU_NONE; u.wait_head = 1'b0; end
      // I2a：单位步长类向量 load 预查后提前提交 (docs/11 §9e)。fu 仍是 FU_VEC (P3 源就绪即发、送 VPC 预查)，同时进 VDQ
      velc[s] = VEL && u.fu == FU_VEC && !u.excp && vel_op(u.raw);
      if (velc[s]) begin
        if (first) first = 1'b0;
        else begin u.vel = 1'b1; u.vea = 1'b1; u.wait_head = 1'b0; end
      end
      dec_n[s] = u;
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
  logic [2:0]   n_take;
  always_comb begin
    for (int s = 0; s < W; s++) begin
      logic [2:0] kbr, ka, kb, km, ks, kd, kf, kx;
      logic [6:0] kv;
      kbr = pcnt(c_br, s); ka = pcnt(c_a, s); kb = pcnt(c_b, s);
      km  = pcnt(c_m, s);  ks = pcnt(c_s, s); kd = pcnt(c_d, s);
      kf  = pcnt(c_f, s);  kx = pcnt(c_a | c_b | c_alu, s);
      // V1.2b：本组乱序向量候选的目的寄存器合计不超过 8 (向量空闲表每拍至多出队 8 个)
      kv = '0;
      for (int k = 0; k < W; k++) if (k <= s && dec_n[k].vfc) kv = kv + 7'(dec_n[k].vng);
      // V1.2c：带标量源的候选的数据半边也占 IQ_D / IQ_F 的入队口
      begin
        logic [3:0] kd2, kf2;
        kd2 = 4'(kd); kf2 = 4'(kf);
        for (int k = 0; k < W; k++)
          if (k <= s && dec_n[k].vxs) begin
            if (dec_n[k].rs2_class == RC_FP) kf2 = kf2 + 4'd1;
            else                             kd2 = kd2 + 4'd1;
          end
        take_n[s] = ib_valid_i[s] && kbr <= 3'(MAX_BR_PER_GRP) && ka <= 3'(IQ_ENQ_MAX) && kb <= 3'(IQ_ENQ_MAX) &&
                    km <= 3'(IQ_ENQ_MAX) && ks <= 3'(IQ_ENQ_MAX) && kd2 <= 4'(IQ_ENQ_MAX) &&
                    kf2 <= 4'(IQ_ENQ_MAX) && 4'(kx) <= 4'(2*IQ_ENQ_MAX) && kv <= 7'd8;
      end
    end
  end

  //--------------------------------------------------------------------------
  // M3 (docs/11 §9g)：向量 load 展开进共用 LSU。
  //   候选 vmc：vml_kind != 0、VECU 的 vtype 合法且 vstart = 0、寄存器组合法 (EMUL 在 1/8..8、vd 按组对齐、不越过 v31)。
  //   只在第 0 槽时展开 (vslow 时第 0 槽的候选按原路径，见 velc)；在后面的槽时截组到它之前。
  //   展开：第一段 = 检查 uop (同 vel：P3 发 VPC 预查，vea = 0 不送 VECU) + 至多 4 个数据 uop，之后每段至多 4 个数据 uop；
  //   最后一段才让取指缓冲出队。数据 uop k：寄存器 vd + k，地址 = 基址 + 128k，有效字节 nb_k = min(128, 总字节 - 128k)，
  //   总字节按译码时的 vl 算 (整寄存器 = 组大小 × 128，vlm = ceil(vl / 8))；nb_k = 0 的寄存器不发 (保持原映射 = 尾部不变)。
  //   vl = 0：只发一个空 uop (不访存、不预查)
  //   M3b：store (vms_kind) 同样展开：检查 uop 的 VPC 按写权限查；数据 uop 是 FU_ST + vms (STA 进 IQ_S，数据半边进 VIQ)
  //--------------------------------------------------------------------------
  localparam int XC = 4;                       // 每段至多几个数据 uop (load 入队上限)
  logic [W-1:0] vmc;
  logic [3:0]   vm_na [W];                     // 各槽候选的数据 uop 数
  logic [10:0]  vm_tot [W];                    // 总字节
  always_comb
    for (int s = 0; s < W; s++) begin
      logic [31:0] i;
      logic [1:0]  kd;
      int          eew, el, ng, tot, na, lm;
      logic        ok;
      i  = dec0[s].raw;
      kd = vml_kind(i) | vms_kind(i);        // M3b：store 同样展开 (两者编码不重叠)
      eew = (i[14:12] == 3'b000) ? 0 : int'(i[13:12]);
      lm  = vlmul_i[2] ? int'(vlmul_i) - 8 : int'(vlmul_i);
      ok = 1'b1; ng = 1; tot = 0;
      unique case (kd)
        2'd1: begin
          el = eew - int'(vsew_i) + lm;
          ok = (el >= -3) && (el <= 3);
          ng = (el > 0) ? (1 << el) : 1;
          tot = int'(vvl_i) << eew;
        end
        2'd2: begin ng = int'(i[31:29]) + 1; tot = ng * CL_B; end
        2'd3: begin ng = 1; tot = (int'(vvl_i) + 7) / 8; end
        default: ok = 1'b0;
      endcase
      ok = ok && ((int'(i[11:7]) % ng) == 0) && (int'(i[11:7]) + ng <= 32);
      // 目的组有在途的不在核内改名的写 (如 VECU 里还没做的 vfmacc)：展开的话重命名要停住等它，不如走原路径 (VECU 内改名)
      // (M3b store：数据源组同理，展开的话要在重命名等那条写做完)
      ok = ok && ((vpend_i & (32'(((64'(1) << ng) - 64'(1)) << i[11:7]) | {31'b0, !i[25]})) == '0);   // M4a：带掩码时 v0 也算
      na = (tot + CL_B - 1) / CL_B;
      if (na > ng) na = ng;
      vmc[s]    = VOOO && vmok_i && dec0[s].fu == FU_VEC && !dec0[s].excp && kd != 2'd0 && ok;
      vm_na[s]  = 4'(na);
      vm_tot[s] = 11'(tot);
    end

  logic        x_on, x_last;
  logic [3:0]  x_k;                            // 本条已发出的数据 uop 数 (检查 uop 在第一段，x_st = 1 表示已发)
  logic        x_st;
  always_comb begin
    logic [W-1:0] cut;
    int           f;
    // 已经开始展开 (x_st：第一段已发) 的指令一直按展开走完：候选条件里的 vpend 是提示，会随更老的指令改名而变，
    //   中途变成 "不展开" 会让这条再按原路径整条发一次，且残留的 x_st / x_k 把下一条可展开的指令当成已发过而整条吞掉
    //   (vrand 种子 9703：vl8re16 第二段时 vpend 变了，随后的 vle16 没有任何 uop)。展开的参数只取决于 vl / vtype，
    //   它们变了会冲刷 (清 x_st)；在途写由重命名停住等
    x_on   = ib_valid_i[0] && (x_st || (vmc[0] && !vslow_i));
    dec    = dec_n;
    take   = take_n;
    x_last = 1'b0;
    if (x_on) begin
      int n, j, na;
      uop_t c;
      na = int'(vm_na[0]);
      take = '0;
      n = 0;
      if (!x_st) begin
        c = dec0[0];
        c.vel = (na != 0); c.vea = 1'b0; c.wait_head = 1'b0; c.vnw = 1'b0; c.vfc = 1'b0; c.vfast = 1'b0;
        if (na == 0) c.fu = FU_NONE;               // vl = 0：空 uop
        c.vgrp = 1'b1; c.vmid = 1'b0; c.vlast = (na == 0); c.vml = 1'b0; c.vms = 1'b0; c.vmk = 1'b0;
        dec[0] = c; take[0] = 1'b1; n = 1;
      end
      for (j = 0; j < XC; j++)
        if (int'(x_k) + j < na) begin
          int   k, nb;
          uop_t u;
          k  = int'(x_k) + j;
          nb = int'(vm_tot[0]) - k * CL_B;
          if (nb > CL_B) nb = CL_B;
          u = dec0[0];
          if (dec0[0].raw[5]) begin
            // M3b 向量 store 数据 uop：不进 VDQ (vea = 0，提交时 VECU 不参与)，数据半边进 VIQ
            u.fu = FU_ST; u.vms = 1'b1; u.vml = 1'b0; u.vfast = 1'b1; u.vea = 1'b0;
          end else begin
            u.fu = FU_LD; u.vml = 1'b1; u.vms = 1'b0; u.vfast = 1'b1; u.vea = 1'b1;
          end
          u.vng = 4'd1; u.vns = 4'd1;                // V1.2b：访存 uop 各管一个寄存器
          u.vel = 1'b0; u.wait_head = 1'b0; u.flush_after = 1'b0;
          u.imm = 32'(k * CL_B); u.use_imm = 1'b1; u.mem_sz = 2'd3; u.mem_unsigned = 1'b1;
          u.rd_class = RC_NONE; u.lrd = 5'd0; u.rs2_class = RC_NONE;
          u.vwd = dec0[0].raw[11:7] + 5'(k); u.vnb = 8'(nb); u.vnw = 1'b0; u.vfc = 1'b0;
          // M4a：带掩码时的 {第几个寄存器, 元素宽度}；整寄存器 / vlm / vsm 都是 vm = 1
          u.vmk = !dec0[0].raw[25]; u.vk = 3'(k);
          u.veb = (dec0[0].raw[14:12] == 3'b000) ? 2'd0 : dec0[0].raw[13:12];
          u.vgrp = 1'b1; u.vmid = 1'b1; u.vlast = (k == na - 1);
          dec[n] = u; take[n] = 1'b1; n++;
        end
      x_last = (int'(x_k) + XC >= na);
    end else begin
      // 后面的槽有候选：截组到它之前 (下一拍它到第 0 槽再展开)
      f = W;
      for (int s = W - 1; s >= 1; s--) if (vmc[s]) f = s;
      for (int s = 0; s < W; s++) if (s >= f) take[s] = 1'b0;
    end
    n_take = '0;
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
  // M3 展开时只在最后一段让取指缓冲出队 (一条)
  assign ib_pop_o = !load ? 3'd0 : x_on ? 3'(x_last) : n_take;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)       begin x_k <= '0; x_st <= 1'b0; end
    else if (flush_i)  begin x_k <= '0; x_st <= 1'b0; end
    else if (load && x_on) begin
      if (x_last) begin x_k <= '0; x_st <= 1'b0; end
      else        begin x_k <= x_k + 4'(XC); x_st <= 1'b1; end
    end
  end
`ifndef SYNTHESIS
  // 展开到一半时取指缓冲第 0 槽仍是那条可展开的向量访存 (只有最后一段才出队)
  always_ff @(posedge clk_i)
    if (rst_ni && !flush_i && x_st)
      assert (ib_valid_i[0] && (vml_kind(dec0[0].raw) != 2'd0 || vms_kind(dec0[0].raw) != 2'd0))
        else $error("o6_decode_stage: 展开到一半时第 0 槽不是那条向量访存 (inst=%h)", dec0[0].raw);
`endif

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

  // I2a：vslow 被本拍取走的第一个候选消耗 (取走 = push 里 take 的槽)
  assign vslow_clr_o = vslow_i && push && (|(take & velc));
  assign vss_clr_o   = vss_i && push && (|(take & visv));
endmodule
