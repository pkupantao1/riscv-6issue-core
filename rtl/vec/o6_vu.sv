//==============================================================================
// o6_vu.sv - 向量单元 (RVV 1.0，VLEN = 1024)，与标量乱序核解耦、按序执行 (docs/11_vector_uarch.md)
//
// 请求：ROB 头的 FU_VEC 指令由 P3 送来 (原始编码 + rs1 / rs2 值)，一次一条，直到回 rsp。
//   异步指令 (纯向量运算)：合法性检查通过、队列有空即回完成，之后在后台按序执行。
//   同步指令 (vsetvl*、向量 CSR、vmv.x.s、访存 …)：等队列与执行流水排空后执行，做完回完成 (带标量结果)。
//   合法性 (依赖 vtype) 在收指令时判：比它老的 vsetvl 都是同步的，当前 vtype 就是它的 vtype。
// 执行：一条按序流水，每拍处理 DLEN 位；寄存器组 (LMUL) 与元素按拍展开，读写在同一拍完成。
// 尾部 / 被掩掉的元素一律不改 (undisturbed，与 Spike 一致)。
//
// 阶段 (见文档)：V0 CSR / vsetvl；V1 整数基本运算、单位步长 / 跨步访存；V2 乘除 / 定点 / 归约 / 掩码 / 置换；
// V3 浮点 (FP64 / FP32 / FP16 / BF16 转换与乘加)：每 64 位一条 fpnew 通道，本拍元素轮流送入。
// 未实现的编码按非法指令报 (对拍会指出)。
//==============================================================================
`default_nettype none
//------------------------------------------------------------------------------
// o6_vu_pkg：向量单元与其子模块共用的类型 (放在同一个文件里，各流程的文件列表不用改)
//------------------------------------------------------------------------------
package o6_vu_pkg;
  // 元素运算的操作码：a = vs2 元素，b = vs1 元素 / x[rs1] / 立即数，c = vd 旧值 (都已扩展到计算宽度)
  typedef enum logic [7:0] {
    A_ADD, A_SUB, A_RSUB, A_MINU, A_MIN, A_MAXU, A_MAX, A_AND, A_OR, A_XOR,
    A_SLL, A_SRL, A_SRA, A_MERGE, A_MV,
    C_EQ, C_NE, C_LTU, C_LT, C_LEU, C_LE, C_GTU, C_GT,
    A_ADC, A_SBC, A_MADC, A_MSBC,                     // 进位 / 借位 (cin = v0 的位)
    A_SADDU, A_SADD, A_SSUBU, A_SSUB,                 // 饱和加减
    A_AADDU, A_AADD, A_ASUBU, A_ASUB,                 // 平均加减 (vxrm 舍入)
    A_SMUL, A_SSRL, A_SSRA,                           // vsmul / 舍入移位
    A_NSRL, A_NSRA, A_NCLIPU, A_NCLIP,                // 收窄移位 (在 2*SEW 宽度算，取低半 / 饱和)
    A_MUL, A_MULH, A_MULHU, A_MULHSU,
    A_MACC, A_NMSAC, A_MADD, A_NMSUB,
    A_WMACC,                                          // 加宽乘加：c + a*b (a / b 已按符号扩展)
    A_EXT,                                            // vzext / vsext：结果 = a
    A_DIVU, A_DIV, A_REMU, A_REM,                     // 除法 (逐拍迭代)
    R_SUM, R_AND, R_OR, R_XOR, R_MINU, R_MIN, R_MAXU, R_MAX,      // 归约
    M_AND, M_NAND, M_ANDN, M_XOR, M_OR, M_NOR, M_ORN, M_XNOR,     // 掩码逻辑
    S_SBF, S_SIF, S_SOF,                              // 置位到第一个 1 之前 / 含 / 仅
    P_IOTA, P_ID,                                     // viota / vid
    P_UP, P_DOWN, P_1UP, P_1DOWN, P_GATH, P_GATH16, P_COMP,       // 置换
    A_SGNJ, A_SGNJN, A_SGNJX,                         // 浮点符号注入 (按位，走整数通路)
    F_ADD, F_SUB, F_RSUB, F_MUL, F_DIV, F_RDIV, F_SQRT, F_MIN, F_MAX,
    F_MACC, F_NMACC, F_MSAC, F_NMSAC, F_MADD, F_NMADD, F_MSUB, F_NMSUB,
    F_CEQ, F_CNE, F_CLT, F_CLE, F_CGT, F_CGE,
    F_CLASS, F_REC7, F_RSQ7, F_CVT,
    F_RUSUM, F_ROSUM, F_RMIN, F_RMAX,                 // 浮点归约
    D_IU, D_IS, D_F8, D_F8A, D_BF,                    // 点积：vqwdotau / vqwdotas / vfqwdota / vfqwdota.alt / vfwdota
    // Zvbb (RVA23 R2c)：vandn / vrol / vror (Zvkb) 与一元的 vbrev / vbrev8 / vrev8 / vclz / vctz / vcpop.v (按 SEW 位宽)
    A_ANDN, A_ROL, A_ROR, A_BREV, A_BREV8, A_REV8, A_CLZ, A_CTZ, A_CPOP
  } aop_e;
endpackage

module o6_vu
  import o6_pkg::*;
  import o6_isa_pkg::*;
#(
  parameter int DLEN = VDLEN
) (
  input  wire                 clk_i,
  input  wire                 rst_ni,

  // 请求 (ROB 头，来自 P3)：req_v 单拍脉冲，之后等 rsp_v
  input  wire                 req_v_i,
  input  wire  [31:0]         req_inst_i,
  input  wire  [63:0]         req_a_i,          // x[rs1]
  input  wire  [63:0]         req_b_i,          // x[rs2]
  output logic                rsp_v_o,          // 单拍脉冲
  output logic [63:0]         rsp_data_o,       // 标量结果 (vsetvl 的 vl、csrr、vmv.x.s)
  output logic                rsp_excp_o,
  output logic [5:0]          rsp_cause_o,
  output logic [63:0]         rsp_tval_o,

  // 访存口 (经 LSU，见 o6_lsu 的向量访存口)：每次一整行 64 B，请求保持到 done
  output logic                vm_req_o,         // 保持到 vm_ack (收下)；结果按序回 vm_done，最多 2 个在途
  input  wire                 vm_ack_i,
  output logic                vm_we_o,
  output logic [63:0]         vm_va_o,          // 本次第一个活动字节的地址
  output logic [511:0]        vm_wdata_o,       // 按行内字节位置
  output logic [63:0]         vm_wmask_o,
  input  wire                 vm_done_i,
  input  wire  [511:0]        vm_rdata_i,
  input  wire                 vm_fault_i,
  input  wire  [5:0]          vm_cause_i,

  // 浮点：frm 在收指令时锁存 (随指令进队列)；异常标志在元素运算结果出来时随到随报 (CSR 里并入 fflags)
  input  wire  [2:0]          frm_i,
  output logic [4:0]          fflags_o,

  output logic                vs_dirty_o,       // 收下会改向量状态的指令：mstatus.VS 置 Dirty
  output logic                idle_o            // 队列空、执行流水空、没有在处理的请求
);
  localparam int LVLEN = $clog2(VLEN);          // 10
  localparam int LDLEN = $clog2(DLEN);
  localparam int VLW   = LVLEN + 1;             // vl 0..1024
  localparam int NEB   = DLEN / 8;              // 每拍最多元素数 (SEW = 8)
  localparam int QD    = 8;                     // 指令队列深度

  //==========================================================================
  // 向量 CSR
  //==========================================================================
  logic [VLW-1:0]   vl;
  logic             vill;
  logic [8:0]       vt;                         // {altfmt, vma, vta, vsew[2:0], vlmul[2:0]}
  logic [LVLEN-1:0] vstart;
  logic             vxsat;
  logic [1:0]       vxrm;
  wire  [1:0]       sew  = vt[4:3];             // 0..3 = 8 / 16 / 32 / 64 (vill 时无意义)
  wire  [2:0]       lmul = vt[2:0];
  wire  [63:0]      vtype_rd = vill ? {1'b1, 63'b0} : {55'b0, vt};
  // altfmt：SEW = 8 时 FP8 选 E5M2 (否则 E4M3)、整数点积 vs1 按有符号；SEW = 16 时 BF16 点积要求它为 1。
  // 只在 SEW = 8 / 16 合法 (本核实现的扩展：Zvfofp8min、Zvqwdota8i / 16i、Zvfqwdota8f、Zvfwdota16bf)
  wire              altf = vt[8];

  // LMUL 的 log2 (-3..3)；寄存器组大小 (LMUL >= 1 时为 LMUL，小数时为 1)
  function automatic int lmul_log2(input logic [2:0] l);
    return (l[2]) ? int'(l) - 8 : int'(l);
  endfunction
  function automatic int grp(input logic [2:0] l);
    return l[2] ? 1 : (1 << l);
  endfunction
  // 寄存器号按寄存器组大小对齐
  function automatic logic al(input logic [4:0] r, input int g);
    return (r & 5'(g - 1)) == 5'd0;
  endfunction
  // 两个寄存器区间 [a, a+as) 与 [b, b+bs) 重叠 (Spike is_overlapped)
  function automatic logic ovl(input int a, input int as_, input int b, input int bs);
    int ae, be;
    ae = a + as_; be = b + bs;
    return ((ae > be ? ae : be) - (a < b ? a : b)) < (as_ + bs);
  endfunction

  //==========================================================================
  // 向量寄存器堆 (32 x VLEN，触发器)
  //==========================================================================
  logic [VLEN-1:0] vrf [32];
  logic [2:0]      rrm;                         // 请求的 frm (当拍的请求 / 锁存的；合法性要用，定义见请求处理)

  //==========================================================================
  // 指令分类 (按原始编码)
  //==========================================================================
  typedef enum logic [4:0] {
    K_ILL,        // 非法 / 未实现
    K_VSET,       // vsetvli / vsetivli / vsetvl          (同步)
    K_CSR,        // 向量 CSR 读写                         (同步)
    K_VMVXS,      // vmv.x.s                              (同步，写整数 rd)
    K_EW,         // 元素运算，向量目的                     (异步)
    K_EWM,        // 元素运算，掩码目的 (比较 / vmadc / vmsbc) (异步)
    K_VMVSX,      // vmv.s.x                              (异步)
    K_VMEM,       // 向量 load / store (单位步长、跨步)      (同步)
    K_RED,        // 归约 (含加宽求和)                       (异步)
    K_MLOG,       // 掩码逻辑 vmand.mm ...                   (异步)
    K_MSET,       // vmsbf / vmsif / vmsof                  (异步)
    K_VIOTA,      // viota / vid                            (异步)
    K_PERM,       // slide / gather / compress (逐元素)      (异步)
    K_VMVNR,      // vmv<nr>r.v 整寄存器复制                 (异步)
    K_VCPOP,      // vcpop.m                                (同步，写整数 rd)
    K_VFIRST,     // vfirst.m                               (同步，写整数 rd)
    K_FEW,        // 浮点元素运算 (fpnew 通道)，向量目的        (异步)
    K_FEWM,       // 浮点比较，掩码目的                       (异步)
    K_FRED,       // 浮点归约 (逐元素串行，与 Spike 同序)       (异步)
    K_VFMVFS,     // vfmv.f.s                               (同步，写浮点 rd)
    K_DOT         // 点积 vd[0] += vs1 . vs2 (OP-VE)          (异步)
  } kind_e;

  // 元素运算的操作码 aop_e 定义在本文件开头的 o6_vu_pkg (子模块 o6_vu_alu 也要用)
  import o6_vu_pkg::*;

  // 宽度类 (相对 SEW)：0 同宽 SSS，1 加宽 .vv/.vx (DSS)，2 加宽 .wv/.wx (DDS)，3 收窄 (SDS)
  typedef struct packed {
    kind_e      k;
    aop_e       op;
    logic [1:0] src;       // 第二操作数：0 = vs1，1 = x[rs1]，2 = simm5，3 = uimm5 (一元运算也用 3：vs1 字段是操作码，不查对齐)
    logic       u6;        // vror.vi：立即数 6 位 (bit 5 = inst[26])
    logic       vm;
    logic [4:0] vd, vs1, vs2;
    logic [1:0] wcls;
    logic [1:0] ext;       // vzext / vsext：1 = vf2，2 = vf4，3 = vf8
    logic       a_sx;      // 加宽 / 扩展：vs2 按符号扩展
    logic       b_sx;      // 加宽：第二操作数按符号扩展
    logic       allb;      // 写全部体内元素 (merge / 进位类，不按掩码跳过)
    logic       cin;       // v0 的位作进位 / 借位输入
    logic       st;        // 访存：store
    logic [2:0] mm;        // 访存方式 (M_*)
    logic [2:0] nf;        // 访存：段数 - 1 (整寄存器：寄存器数 - 1)
    logic [1:0] eb;        // 访存：EEW = 8 << eb
    logic       fp;        // 浮点指令：要求 SEW 在 sewm 里、frm 合法 (FS 在标量译码器查)
    logic [3:0] sewm;      // 允许的 SEW (位 i = 8 << i)
    logic       bf;        // BF16 (vfwcvtbf16 / vfncvtbf16 / vfwmaccbf16)
    logic [1:0] cvk;       // 转换：0 浮点 -> 浮点，1 整数 -> 浮点，2 浮点 -> 整数
    logic       cvu;       // 转换：整数无符号
    logic       rtz;       // 转换：固定 RTZ
    logic       rod;       // 转换：round-to-odd (vfncvt.rod)
    logic       sat;       // OFP8 收窄：饱和
    logic       q;         // 四倍收窄 (vfncvt.f.f.q：FP32 -> OFP8)
    logic [3:0] afm;       // altfmt = 1 时仍合法的 SEW (浮点指令；默认都不合法，与 Spike 一致)
    logic       afreq;     // 要求 altfmt = 1 (vfwdota)
  } vdec_t;

  // 访存方式
  localparam logic [2:0] M_UNIT = 3'd0, M_STRD = 3'd1, M_IDX = 3'd2, M_WHOLE = 3'd3, M_MASK = 3'd4, M_FF = 3'd5;

  function automatic vdec_t vdec(input logic [31:0] x);
    vdec_t d;
    logic [2:0] f3;
    logic [5:0] f6;
    logic vv, vx, vi, mv, mx, fv, ff;
    d = '0;
    d.k = K_ILL;
    f3 = x[14:12]; f6 = x[31:26];
    d.vm = x[25]; d.vs2 = x[24:20]; d.vs1 = x[19:15]; d.vd = x[11:7];
    vv = (f3 == 3'b000); vx = (f3 == 3'b100); vi = (f3 == 3'b011);
    mv = (f3 == 3'b010); mx = (f3 == 3'b110);
    fv = (f3 == 3'b001); ff = (f3 == 3'b101);
    if (x[6:0] == OPC_SYSTEM) d.k = K_CSR;
    else if (x[6:0] == OPC_LOAD_FP || x[6:0] == OPC_STORE_FP) begin
      d.st   = (x[6:0] == OPC_STORE_FP);
      d.nf   = x[31:29];
      d.eb   = (f3 == 3'b000) ? 2'd0 : (f3 == 3'b101) ? 2'd1 : (f3 == 3'b110) ? 2'd2 : 2'd3;
      // mew = 0；mop：00 单位步长 (lumop / sumop 再细分)，01 / 11 索引 (无序 / 有序，这里都按序做)，10 跨步
      if (!x[28]) begin
        d.k = K_VMEM;
        unique case (x[27:26])
          2'b00: unique case (x[24:20])
            5'b00000: d.mm = M_UNIT;
            5'b01000: d.mm = M_WHOLE;                  // vl<nr>re<eew> / vs<nr>r
            5'b01011: d.mm = M_MASK;                   // vlm / vsm
            5'b10000: begin d.mm = M_FF; if (d.st) d.k = K_ILL; end   // fault-only-first (只有 load)
            default:  d.k = K_ILL;
          endcase
          2'b10:   d.mm = M_STRD;
          default: d.mm = M_IDX;
        endcase
      end
    end
    else if (x[6:0] == OPC_OP_VE) begin                        // 点积
      d.k = K_DOT;
      unique case ({f3, f6})
        {3'b000, 6'b100110}: begin d.op = D_IU; d.sewm = 4'b0011; end
        {3'b000, 6'b100111}: begin d.op = D_IS; d.sewm = 4'b0011; end
        {3'b001, 6'b100110}: begin d.op = D_F8;  d.fp = 1'b1; d.sewm = 4'b0001; d.afm = 4'b1111; end
        {3'b001, 6'b100111}: begin d.op = D_F8A; d.fp = 1'b1; d.sewm = 4'b0001; d.afm = 4'b1111; end
        {3'b001, 6'b100100}: begin d.op = D_BF;  d.fp = 1'b1; d.sewm = 4'b0010; d.afm = 4'b1111; d.afreq = 1'b1; end
        default: d.k = K_ILL;
      endcase
    end
    else if (x[6:0] == OPC_OP_V) begin
      if (f3 == 3'b111) d.k = K_VSET;
      else if (vv || vx || vi) begin                           // OPIVV / OPIVX / OPIVI
        d.src = vv ? 2'd0 : vx ? 2'd1 : 2'd2;
        d.k = K_EW;
        unique case (f6)
          6'b000000: d.op = A_ADD;
          6'b000001: begin d.op = A_ANDN; if (vi) d.k = K_ILL; end                  // Zvbb vandn
          6'b010100: begin d.op = A_ROR; if (vi) begin d.src = 2'd3; d.u6 = 1'b1; end end   // vror (.vi 立即数 bit 5 = 0)
          6'b010101: begin                                                            // vrol (.vv / .vx)；.vi = vror 立即数 bit 5 = 1
            if (vi) begin d.op = A_ROR; d.src = 2'd3; d.u6 = 1'b1; end
            else d.op = A_ROL;
          end
          6'b110101: begin d.op = A_SLL; d.wcls = 2'd1; if (vi) d.src = 2'd3; end   // vwsll：2*SEW = zext(vs2) << (低 log2(2*SEW) 位)
          6'b000010: begin d.op = A_SUB;  if (vi) d.k = K_ILL; end
          6'b000011: begin d.op = A_RSUB; if (vv) d.k = K_ILL; end
          6'b000100: begin d.op = A_MINU; if (vi) d.k = K_ILL; end
          6'b000101: begin d.op = A_MIN;  if (vi) d.k = K_ILL; end
          6'b000110: begin d.op = A_MAXU; if (vi) d.k = K_ILL; end
          6'b000111: begin d.op = A_MAX;  if (vi) d.k = K_ILL; end
          6'b001001: d.op = A_AND;
          6'b001010: d.op = A_OR;
          6'b001011: d.op = A_XOR;
          6'b001100: begin d.k = K_PERM; d.op = P_GATH; if (vi) d.src = 2'd3; end
          6'b001110: begin d.k = K_PERM; d.op = vv ? P_GATH16 : P_UP; if (vi) d.src = 2'd3; end
          6'b001111: begin d.k = vv ? K_ILL : K_PERM; d.op = P_DOWN; if (vi) d.src = 2'd3; end
          6'b010000: begin d.op = A_ADC; d.cin = 1'b1; d.allb = 1'b1; if (d.vm) d.k = K_ILL; end
          6'b010001: begin d.op = A_MADC; d.cin = !d.vm; d.allb = 1'b1; d.k = K_EWM; end
          6'b010010: begin d.op = A_SBC; d.cin = 1'b1; d.allb = 1'b1; if (d.vm || vi) d.k = K_ILL; end
          6'b010011: begin d.op = A_MSBC; d.cin = !d.vm; d.allb = 1'b1; d.k = vi ? K_ILL : K_EWM; end
          6'b010111: begin                                     // vmerge (vm = 0) / vmv.v.* (vm = 1, vs2 = 0)
            d.op = d.vm ? A_MV : A_MERGE; d.allb = 1'b1;
            if (d.vm && d.vs2 != 5'd0) d.k = K_ILL;
          end
          6'b011000: begin d.op = C_EQ;  d.k = K_EWM; end
          6'b011001: begin d.op = C_NE;  d.k = K_EWM; end
          6'b011010: begin d.op = C_LTU; d.k = vi ? K_ILL : K_EWM; end
          6'b011011: begin d.op = C_LT;  d.k = vi ? K_ILL : K_EWM; end
          6'b011100: begin d.op = C_LEU; d.k = K_EWM; end
          6'b011101: begin d.op = C_LE;  d.k = K_EWM; end
          6'b011110: begin d.op = C_GTU; d.k = vv ? K_ILL : K_EWM; end
          6'b011111: begin d.op = C_GT;  d.k = vv ? K_ILL : K_EWM; end
          6'b100000: d.op = A_SADDU;
          6'b100001: d.op = A_SADD;
          6'b100010: begin d.op = A_SSUBU; if (vi) d.k = K_ILL; end
          6'b100011: begin d.op = A_SSUB;  if (vi) d.k = K_ILL; end
          6'b100101: begin d.op = A_SLL; if (vi) d.src = 2'd3; end
          6'b100111: begin                                     // vsmul / vmv<nr>r.v (vi，nr - 1 在 simm 字段)
            d.op = A_SMUL;
            if (vi) d.k = (d.vm && (d.vs1 == 5'd0 || d.vs1 == 5'd1 || d.vs1 == 5'd3 || d.vs1 == 5'd7)) ? K_VMVNR : K_ILL;
          end
          6'b101000: begin d.op = A_SRL;  if (vi) d.src = 2'd3; end
          6'b101001: begin d.op = A_SRA;  if (vi) d.src = 2'd3; end
          6'b101010: begin d.op = A_SSRL; if (vi) d.src = 2'd3; end
          6'b101011: begin d.op = A_SSRA; if (vi) d.src = 2'd3; end
          6'b101100: begin d.op = A_NSRL;   d.wcls = 2'd3; if (vi) d.src = 2'd3; end
          6'b101101: begin d.op = A_NSRA;   d.wcls = 2'd3; if (vi) d.src = 2'd3; end
          6'b101110: begin d.op = A_NCLIPU; d.wcls = 2'd3; if (vi) d.src = 2'd3; end
          6'b101111: begin d.op = A_NCLIP;  d.wcls = 2'd3; if (vi) d.src = 2'd3; end
          6'b110000: begin d.op = R_SUM; d.k = vv ? K_RED : K_ILL; d.wcls = 2'd1; end              // vwredsumu
          6'b110001: begin d.op = R_SUM; d.k = vv ? K_RED : K_ILL; d.wcls = 2'd1; d.a_sx = 1; end  // vwredsum
          default:   d.k = K_ILL;
        endcase
      end else if (fv || ff) begin                             // OPFVV / OPFVF
        d.src = fv ? 2'd0 : 2'd1;
        d.k = K_FEW; d.fp = 1'b1; d.sewm = 4'b1110;
        unique case (f6)
          6'b000000: d.op = F_ADD;
          6'b000001: begin d.op = F_RUSUM; d.k = fv ? K_FRED : K_ILL; end
          6'b000010: d.op = F_SUB;
          6'b000011: begin d.op = F_ROSUM; d.k = fv ? K_FRED : K_ILL; end
          6'b000100: d.op = F_MIN;
          6'b000101: begin d.op = F_RMIN;  d.k = fv ? K_FRED : K_ILL; end
          6'b000110: d.op = F_MAX;
          6'b000111: begin d.op = F_RMAX;  d.k = fv ? K_FRED : K_ILL; end
          6'b001000: begin d.op = A_SGNJ;  d.k = K_EW; end
          6'b001001: begin d.op = A_SGNJN; d.k = K_EW; end
          6'b001010: begin d.op = A_SGNJX; d.k = K_EW; end
          6'b001110: begin d.op = P_1UP;   d.k = ff ? K_PERM : K_ILL; end
          6'b001111: begin d.op = P_1DOWN; d.k = ff ? K_PERM : K_ILL; end
          6'b010000: begin                                     // vfmv.f.s / vfmv.s.f
            if (fv && d.vs1 == 5'd0 && d.vm)      d.k = K_VFMVFS;
            else if (ff && d.vs2 == 5'd0 && d.vm) d.k = K_VMVSX;
            else                                  d.k = K_ILL;
          end
          6'b010010: begin                                     // VFUNARY0：转换
            d.op = F_CVT; d.src = 2'd3;
            unique case (d.vs1)
              5'b00000: begin d.cvk = 2'd2; d.cvu = 1'b1; end
              5'b00001: begin d.cvk = 2'd2; end
              5'b00010: begin d.cvk = 2'd1; d.cvu = 1'b1; end
              5'b00011: begin d.cvk = 2'd1; end
              5'b00110: begin d.cvk = 2'd2; d.cvu = 1'b1; d.rtz = 1'b1; end
              5'b00111: begin d.cvk = 2'd2; d.rtz = 1'b1; end
              5'b01000: begin d.cvk = 2'd2; d.cvu = 1'b1; d.wcls = 2'd1; d.sewm = 4'b0110; end
              5'b01001: begin d.cvk = 2'd2; d.wcls = 2'd1; d.sewm = 4'b0110; end
              5'b01010: begin d.cvk = 2'd1; d.cvu = 1'b1; d.wcls = 2'd1; d.sewm = 4'b0111; d.afm = 4'b0010; end
              5'b01011: begin d.cvk = 2'd1; d.wcls = 2'd1; d.sewm = 4'b0111; d.afm = 4'b0010; end
              5'b01100: begin d.cvk = 2'd0; d.wcls = 2'd1; d.sewm = 4'b0110; end
              // vfwcvtbf16：SEW = 16 是 BF16 -> FP32；SEW = 8 是 OFP8 -> BF16 (Zvfofp8min，altfmt 选 E5M2)
              5'b01101: begin d.cvk = 2'd0; d.wcls = 2'd1; d.sewm = 4'b0011; d.bf = 1'b1; d.afm = 4'b0011; end
              5'b01110: begin d.cvk = 2'd2; d.cvu = 1'b1; d.rtz = 1'b1; d.wcls = 2'd1; d.sewm = 4'b0110; end
              5'b01111: begin d.cvk = 2'd2; d.rtz = 1'b1; d.wcls = 2'd1; d.sewm = 4'b0110; end
              5'b10000: begin d.cvk = 2'd2; d.cvu = 1'b1; d.wcls = 2'd3; d.sewm = 4'b0111; d.afm = 4'b0010; end
              5'b10001: begin d.cvk = 2'd2; d.wcls = 2'd3; d.sewm = 4'b0111; d.afm = 4'b0010; end
              5'b10010: begin d.cvk = 2'd1; d.cvu = 1'b1; d.wcls = 2'd3; d.sewm = 4'b0110; end
              5'b10011: begin d.cvk = 2'd1; d.wcls = 2'd3; d.sewm = 4'b0110; end
              5'b10100: begin d.cvk = 2'd0; d.wcls = 2'd3; d.sewm = 4'b0110; end
              5'b10101: begin d.cvk = 2'd0; d.rod = 1'b1; d.wcls = 2'd3; d.sewm = 4'b0110; end
              5'b10110: begin d.cvk = 2'd2; d.cvu = 1'b1; d.rtz = 1'b1; d.wcls = 2'd3; d.sewm = 4'b0111; d.afm = 4'b0010; end
              5'b10111: begin d.cvk = 2'd2; d.rtz = 1'b1; d.wcls = 2'd3; d.sewm = 4'b0111; d.afm = 4'b0010; end
              // vfncvtbf16 (.sat)：SEW = 16 是 FP32 -> BF16；SEW = 8 是 BF16 -> OFP8
              5'b11101: begin d.cvk = 2'd0; d.wcls = 2'd3; d.sewm = 4'b0011; d.bf = 1'b1; d.afm = 4'b0011; end
              5'b11111: begin d.cvk = 2'd0; d.wcls = 2'd3; d.sewm = 4'b0001; d.bf = 1'b1; d.afm = 4'b0001; d.sat = 1'b1; end
              // vfncvt (.sat).f.f.q：FP32 -> OFP8 (四倍收窄)
              5'b11001: begin d.cvk = 2'd0; d.wcls = 2'd3; d.q = 1'b1; d.sewm = 4'b0001; d.afm = 4'b0001; end
              5'b11011: begin d.cvk = 2'd0; d.wcls = 2'd3; d.q = 1'b1; d.sewm = 4'b0001; d.afm = 4'b0001; d.sat = 1'b1; end
              default:  d.k = K_ILL;
            endcase
            if (!fv) d.k = K_ILL;
          end
          6'b010011: begin                                     // VFUNARY1
            d.src = 2'd3;
            unique case (d.vs1)
              5'b00000: d.op = F_SQRT;
              5'b00100: d.op = F_RSQ7;
              5'b00101: d.op = F_REC7;
              5'b10000: d.op = F_CLASS;
              default:  d.k = K_ILL;
            endcase
            if (!fv) d.k = K_ILL;
          end
          6'b010111: begin                                     // vfmerge.vfm (vm = 0) / vfmv.v.f (vm = 1, vs2 = 0)
            d.k = ff ? K_EW : K_ILL; d.op = d.vm ? A_MV : A_MERGE; d.allb = 1'b1;
            if (d.vm && d.vs2 != 5'd0) d.k = K_ILL;
          end
          6'b011000: begin d.op = F_CEQ; d.k = K_FEWM; end
          6'b011001: begin d.op = F_CLE; d.k = K_FEWM; end
          6'b011011: begin d.op = F_CLT; d.k = K_FEWM; end
          6'b011100: begin d.op = F_CNE; d.k = K_FEWM; end
          6'b011101: begin d.op = F_CGT; d.k = ff ? K_FEWM : K_ILL; end
          6'b011111: begin d.op = F_CGE; d.k = ff ? K_FEWM : K_ILL; end
          6'b100000: d.op = F_DIV;
          6'b100001: begin d.op = F_RDIV; if (fv) d.k = K_ILL; end
          6'b100100: d.op = F_MUL;
          6'b100111: begin d.op = F_RSUB; if (fv) d.k = K_ILL; end
          6'b101000: d.op = F_MADD;
          6'b101001: d.op = F_NMADD;
          6'b101010: d.op = F_MSUB;
          6'b101011: d.op = F_NMSUB;
          6'b101100: d.op = F_MACC;
          6'b101101: d.op = F_NMACC;
          6'b101110: d.op = F_MSAC;
          6'b101111: d.op = F_NMSAC;
          // 加宽 (源先精确升到 2*SEW 再算，与 Spike 的 f16_to_f32 / f32_to_f64 相同)
          6'b110000: begin d.op = F_ADD;   d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b110001: begin d.op = F_RUSUM; d.wcls = 2'd1; d.sewm = 4'b0110; d.k = fv ? K_FRED : K_ILL; end
          6'b110010: begin d.op = F_SUB;   d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b110011: begin d.op = F_ROSUM; d.wcls = 2'd1; d.sewm = 4'b0110; d.k = fv ? K_FRED : K_ILL; end
          6'b110100: begin d.op = F_ADD;   d.wcls = 2'd2; d.sewm = 4'b0110; end
          6'b110110: begin d.op = F_SUB;   d.wcls = 2'd2; d.sewm = 4'b0110; end
          6'b111000: begin d.op = F_MUL;   d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b111011: begin d.op = F_MACC;  d.wcls = 2'd1; d.sewm = 4'b0010; d.bf = 1'b1; d.afm = 4'b0010; end   // vfwmaccbf16
          6'b111100: begin d.op = F_MACC;  d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b111101: begin d.op = F_NMACC; d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b111110: begin d.op = F_MSAC;  d.wcls = 2'd1; d.sewm = 4'b0110; end
          6'b111111: begin d.op = F_NMSAC; d.wcls = 2'd1; d.sewm = 4'b0110; end
          default:   d.k = K_ILL;
        endcase
      end else if (mv || mx) begin                             // OPMVV / OPMVX
        d.src = mv ? 2'd0 : 2'd1;
        d.k = K_EW;
        unique case (f6)
          6'b000000: begin d.op = R_SUM;  d.k = mv ? K_RED : K_ILL; end
          6'b000001: begin d.op = R_AND;  d.k = mv ? K_RED : K_ILL; end
          6'b000010: begin d.op = R_OR;   d.k = mv ? K_RED : K_ILL; end
          6'b000011: begin d.op = R_XOR;  d.k = mv ? K_RED : K_ILL; end
          6'b000100: begin d.op = R_MINU; d.k = mv ? K_RED : K_ILL; end
          6'b000101: begin d.op = R_MIN;  d.k = mv ? K_RED : K_ILL; end
          6'b000110: begin d.op = R_MAXU; d.k = mv ? K_RED : K_ILL; end
          6'b000111: begin d.op = R_MAX;  d.k = mv ? K_RED : K_ILL; end
          6'b001110: begin d.op = P_1UP;   d.k = mx ? K_PERM : K_ILL; end
          6'b001111: begin d.op = P_1DOWN; d.k = mx ? K_PERM : K_ILL; end
          6'b010100: begin                                     // VMUNARY0
            unique case (d.vs1)
              5'b00001: begin d.k = K_MSET; d.op = S_SBF; end
              5'b00010: begin d.k = K_MSET; d.op = S_SOF; end
              5'b00011: begin d.k = K_MSET; d.op = S_SIF; end
              5'b10000: begin d.k = K_VIOTA; d.op = P_IOTA; end
              5'b10001: begin d.k = (d.vs2 == 5'd0) ? K_VIOTA : K_ILL; d.op = P_ID; end
              default:  d.k = K_ILL;
            endcase
            if (!mv) d.k = K_ILL;
          end
          6'b010111: begin d.op = P_COMP; d.k = (mv && d.vm) ? K_PERM : K_ILL; end
          6'b011000: begin d.op = M_ANDN; d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011001: begin d.op = M_AND;  d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011010: begin d.op = M_OR;   d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011011: begin d.op = M_XOR;  d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011100: begin d.op = M_ORN;  d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011101: begin d.op = M_NAND; d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011110: begin d.op = M_NOR;  d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b011111: begin d.op = M_XNOR; d.k = (mv && d.vm) ? K_MLOG : K_ILL; end
          6'b100000: d.op = A_DIVU;
          6'b100001: d.op = A_DIV;
          6'b100010: d.op = A_REMU;
          6'b100011: d.op = A_REM;
          6'b001000: d.op = A_AADDU;
          6'b001001: d.op = A_AADD;
          6'b001010: d.op = A_ASUBU;
          6'b001011: d.op = A_ASUB;
          6'b010000: begin
            if (mv && d.vs1 == 5'd0 && d.vm)      d.k = K_VMVXS;
            else if (mv && d.vs1 == 5'b10000)     d.k = K_VCPOP;
            else if (mv && d.vs1 == 5'b10001)     d.k = K_VFIRST;
            else if (mx && d.vs2 == 5'd0 && d.vm) d.k = K_VMVSX;
            else                                  d.k = K_ILL;
          end
          6'b010010: begin                                     // VXUNARY0：vzext / vsext；Zvbb 的一元运算
            d.op = A_EXT;
            unique case (d.vs1)
              5'b01000: begin d.op = A_BREV8; d.src = 2'd3; end
              5'b01001: begin d.op = A_REV8;  d.src = 2'd3; end
              5'b01010: begin d.op = A_BREV;  d.src = 2'd3; end
              5'b01100: begin d.op = A_CLZ;   d.src = 2'd3; end
              5'b01101: begin d.op = A_CTZ;   d.src = 2'd3; end
              5'b01110: begin d.op = A_CPOP;  d.src = 2'd3; end
              5'b00010: begin d.ext = 2'd3; d.a_sx = 1'b0; end
              5'b00011: begin d.ext = 2'd3; d.a_sx = 1'b1; end
              5'b00100: begin d.ext = 2'd2; d.a_sx = 1'b0; end
              5'b00101: begin d.ext = 2'd2; d.a_sx = 1'b1; end
              5'b00110: begin d.ext = 2'd1; d.a_sx = 1'b0; end
              5'b00111: begin d.ext = 2'd1; d.a_sx = 1'b1; end
              default:  d.k = K_ILL;
            endcase
            if (!mv) d.k = K_ILL;
          end
          6'b100100: d.op = A_MULHU;
          6'b100101: d.op = A_MUL;
          6'b100110: d.op = A_MULHSU;
          6'b100111: d.op = A_MULH;
          6'b101001: d.op = A_MADD;
          6'b101011: d.op = A_NMSUB;
          6'b101101: d.op = A_MACC;
          6'b101111: d.op = A_NMSAC;
          // 加宽加减 (DSS) / .w 形式 (DDS)：在 2*SEW 宽度做加减
          6'b110000: begin d.op = A_ADD; d.wcls = 2'd1; end                      // vwaddu
          6'b110001: begin d.op = A_ADD; d.wcls = 2'd1; d.a_sx = 1; d.b_sx = 1; end   // vwadd
          6'b110010: begin d.op = A_SUB; d.wcls = 2'd1; end                      // vwsubu
          6'b110011: begin d.op = A_SUB; d.wcls = 2'd1; d.a_sx = 1; d.b_sx = 1; end   // vwsub
          6'b110100: begin d.op = A_ADD; d.wcls = 2'd2; end                      // vwaddu.w
          6'b110101: begin d.op = A_ADD; d.wcls = 2'd2; d.b_sx = 1; end          // vwadd.w
          6'b110110: begin d.op = A_SUB; d.wcls = 2'd2; end                      // vwsubu.w
          6'b110111: begin d.op = A_SUB; d.wcls = 2'd2; d.b_sx = 1; end          // vwsub.w
          6'b111000: begin d.op = A_MUL; d.wcls = 2'd1; end                      // vwmulu
          6'b111010: begin d.op = A_MUL; d.wcls = 2'd1; d.b_sx = 0; d.a_sx = 1; end  // vwmulsu：vs2 有符号
          6'b111011: begin d.op = A_MUL; d.wcls = 2'd1; d.a_sx = 1; d.b_sx = 1; end   // vwmul
          6'b111100: begin d.op = A_WMACC; d.wcls = 2'd1; end                    // vwmaccu
          6'b111101: begin d.op = A_WMACC; d.wcls = 2'd1; d.a_sx = 1; d.b_sx = 1; end // vwmacc
          6'b111110: begin d.op = A_WMACC; d.wcls = 2'd1; d.a_sx = 1; if (mv) d.k = K_ILL; end  // vwmaccus：x 无符号、vs2 有符号
          6'b111111: begin d.op = A_WMACC; d.wcls = 2'd1; d.b_sx = 1; end         // vwmaccsu：vs1/x 有符号、vs2 无符号
          default:   d.k = K_ILL;
        endcase
      end
    end
    return d;
  endfunction

  // 计算宽度 / 目的宽度 / vs2 宽度 (log2(EEW / 8))
  function automatic logic [1:0] cwid(input vdec_t d, input logic [1:0] s);
    return (d.wcls != 2'd0) ? s + 2'd1 : s;
  endfunction
  function automatic logic [1:0] dwid(input vdec_t d, input logic [1:0] s);
    return (d.wcls == 2'd1 || d.wcls == 2'd2) ? s + 2'd1 : s;
  endfunction
  function automatic logic [1:0] s2wid(input vdec_t d, input logic [1:0] s);
    return (d.wcls == 2'd2 || d.wcls == 2'd3) ? s + 2'd1 : s - d.ext;
  endfunction

  // Spike 的寄存器大小 (浮点 LMUL 转 int：小数为 0，重叠判断时按 1 算)
  function automatic int lsz(input int lg);
    return (lg >= 0) ? (1 << lg) : 0;
  endfunction
  function automatic logic ovl0(input int a, input int as_, input int b, input int bs);
    return ovl(a, (as_ == 0) ? 1 : as_, b, (bs == 0) ? 1 : bs);
  endfunction
  // is_overlapped_widen：源在目的组的最高部分时允许
  function automatic logic ovlw(input int a, input int as_, input int b, input int bs);
    int aa, bb;
    aa = (as_ == 0) ? 1 : as_; bb = (bs == 0) ? 1 : bs;
    if (a < b && ovl(a, aa, b, bb) && !ovl(a, aa, b + bb, bb)) return 1'b0;
    return ovl(a, aa, b, bb);
  endfunction
  function automatic logic alg(input logic [4:0] r, input int lg);   // require_align (小数时恒真)
    return (lg <= 0) ? 1'b1 : al(r, 1 << lg);
  endfunction

  // 按当前 vtype 的合法性 (Spike 的 require_vector / VI_CHECK_*)
  function automatic logic vlegal(input vdec_t d);
    int  ll;
    logic ok, v1;
    ll = lmul_log2(lmul);
    v1 = (d.src == 2'd0);
    ok = !vill && (vstart == '0);                // 运算类指令要求 vstart = 0
    unique case (d.k)
      K_EWM, K_FEWM: begin                       // VI_CHECK_MSS
        if (d.vd != d.vs2 && ovl0(d.vd, 1, d.vs2, lsz(ll))) ok = 1'b0;
        if (!alg(d.vs2, ll)) ok = 1'b0;
        if (v1) begin
          if (d.vd != d.vs1 && ovl0(d.vd, 1, d.vs1, lsz(ll))) ok = 1'b0;
          if (!alg(d.vs1, ll)) ok = 1'b0;
        end
      end
      K_EW, K_FEW: begin
        if (!d.vm && d.vd == 5'd0) ok = 1'b0;    // require_vm (含 vmerge / vadc / vsbc)
        if (d.ext != 2'd0) begin                 // VI_EXT_CHECK
          int sl;
          sl = ll - int'(d.ext);
          if (int'(sew) < int'(d.ext)) ok = 1'b0;            // from = SEW / div >= 8
          if (d.vd == d.vs2) ok = 1'b0;
          if (sl < -3 || sl > 3) ok = 1'b0;
          if (!alg(d.vd, ll) || !alg(d.vs2, sl)) ok = 1'b0;
          if (sl < 0 ? ovl0(d.vd, lsz(ll), d.vs2, lsz(sl)) : ovlw(d.vd, lsz(ll), d.vs2, lsz(sl))) ok = 1'b0;
        end else unique case (d.wcls)
          2'd0: begin                            // VI_CHECK_SSS
            if (!alg(d.vd, ll) || !alg(d.vs2, ll)) ok = 1'b0;
            if (v1 && !alg(d.vs1, ll)) ok = 1'b0;
          end
          2'd1, 2'd2: begin                      // VI_CHECK_DSS / DDS
            if (ll > 2 || sew > 2'd2) ok = 1'b0;
            if (!alg(d.vd, ll + 1)) ok = 1'b0;
            if (d.wcls == 2'd1) begin
              if (!alg(d.vs2, ll)) ok = 1'b0;
              if (ll < 0 ? ovl0(d.vd, lsz(ll + 1), d.vs2, lsz(ll)) : ovlw(d.vd, lsz(ll + 1), d.vs2, lsz(ll))) ok = 1'b0;
            end else if (!alg(d.vs2, ll + 1)) ok = 1'b0;
            if (v1) begin
              if (!alg(d.vs1, ll)) ok = 1'b0;
              if (ll < 0 ? ovl0(d.vd, lsz(ll + 1), d.vs1, lsz(ll)) : ovlw(d.vd, lsz(ll + 1), d.vs1, lsz(ll))) ok = 1'b0;
            end
          end
          default: if (d.q) begin               // VI_CHECK_SQS (四倍收窄)
            if (ll > 1 || sew > 2'd1) ok = 1'b0;
            if (!alg(d.vs2, ll + 2) || !alg(d.vd, ll)) ok = 1'b0;
            if (d.vd != d.vs2 && ovl0(d.vd, lsz(ll), d.vs2, lsz(ll + 2))) ok = 1'b0;
          end else begin                         // VI_CHECK_SDS
            if (ll > 2 || sew > 2'd2) ok = 1'b0;
            if (!alg(d.vs2, ll + 1) || !alg(d.vd, ll)) ok = 1'b0;
            if (d.vd != d.vs2 && ovl0(d.vd, lsz(ll), d.vs2, lsz(ll + 1))) ok = 1'b0;
            if (v1 && !alg(d.vs1, ll)) ok = 1'b0;
          end
        endcase
      end
      K_VMVXS, K_VMVSX, K_MLOG, K_VCPOP, K_VFIRST: ;
      K_RED, K_FRED: begin                       // VI_CHECK_REDUCTION
        if (!alg(d.vs2, ll)) ok = 1'b0;
        if (d.wcls != 2'd0 && sew > 2'd2) ok = 1'b0;
      end
      K_MSET: begin
        if (!d.vm && d.vd == 5'd0) ok = 1'b0;
        if (d.vd == d.vs2) ok = 1'b0;
      end
      K_VIOTA: begin
        if (!d.vm && d.vd == 5'd0) ok = 1'b0;
        if (!alg(d.vd, ll)) ok = 1'b0;
        if (d.op == P_IOTA && ovl0(d.vd, lsz(ll), d.vs2, 1)) ok = 1'b0;
      end
      K_VMVNR: begin
        int nr;
        nr = int'(d.vs1) + 1;
        if (!al(d.vd, nr) || !al(d.vs2, nr)) ok = 1'b0;
      end
      K_PERM: begin
        if (!alg(d.vd, ll) || !alg(d.vs2, ll)) ok = 1'b0;
        unique case (d.op)
          P_UP, P_1UP: begin if (!d.vm && d.vd == 5'd0) ok = 1'b0; if (d.vd == d.vs2) ok = 1'b0; end
          P_DOWN, P_1DOWN: if (!d.vm && d.vd == 5'd0) ok = 1'b0;
          P_GATH: begin
            if (!d.vm && d.vd == 5'd0) ok = 1'b0;
            if (d.vd == d.vs2) ok = 1'b0;
            if (v1 && (d.vd == d.vs1 || !alg(d.vs1, ll))) ok = 1'b0;
          end
          P_GATH16: begin
            int l16;
            l16 = 1 - int'(sew) + ll;                  // log2 EMUL (EEW = 16)
            if (!d.vm && d.vd == 5'd0) ok = 1'b0;
            if (d.vd == d.vs2) ok = 1'b0;
            if (l16 < -3 || l16 > 3) ok = 1'b0;
            if (!alg(d.vs1, l16)) ok = 1'b0;
            if (ovl0(d.vd, lsz(ll), d.vs1, lsz(l16))) ok = 1'b0;
          end
          default: begin                           // vcompress
            if (d.vd == d.vs2) ok = 1'b0;
            if (ovl0(d.vd, lsz(ll), d.vs1, 1)) ok = 1'b0;
          end
        endcase
      end
      K_DOT: begin                               // ZVLDOT_INIT
        if (!alg(d.vs1, ll) || !alg(d.vs2, ll)) ok = 1'b0;
        if (!d.vm && d.vd == 5'd0) ok = 1'b0;
        if (ovl0(d.vd, 1, d.vs1, lsz(ll)) || ovl0(d.vd, 1, d.vs2, lsz(ll))) ok = 1'b0;
        if (!d.sewm[sew]) ok = 1'b0;
      end
      default: ok = 1'b0;
    endcase
    // 浮点 (VI_VFP_COMMON 及各转换的 CHECK)：SEW 受限、frm 必须合法 (validate_rm)
    if (d.fp && (!d.sewm[sew] || rrm > 3'd4)) ok = 1'b0;
    // altfmt：浮点指令只有少数在 altfmt = 1 时合法 (BF16 / OFP8 转换、vfwmaccbf16、部分整数 <-> 浮点转换、点积)
    if (d.fp && altf && !d.afm[sew]) ok = 1'b0;
    if (d.afreq && !altf) ok = 1'b0;
    return ok;
  endfunction
  // 访存的合法性 (Spike 的 VI_CHECK_LOAD / STORE / LD_INDEX / ST_INDEX、VI_LD_WHOLE)：vstart 可以非零 (可恢复)
  function automatic logic vmlegal(input vdec_t d);
    int le, ll, nf, em;
    logic ok;
    ll = lmul_log2(lmul);
    nf = int'(d.nf) + 1;
    unique case (d.mm)
      M_WHOLE: begin                             // 不看 vtype (require_vector_novtype)；vs<nr>r 只有 EEW = 8
        ok = d.vm && (nf == 1 || nf == 2 || nf == 4 || nf == 8) && al(d.vd, nf) && !(d.st && d.eb != 2'd0);
      end
      M_MASK: ok = !vill && d.vm && d.nf == 3'd0 && d.eb == 2'd0;
      M_IDX: begin                               // eb 是索引 EEW；数据 EEW = SEW
        int li, fl;
        li = int'(d.eb) - int'(sew) + ll;        // log2 索引 EMUL
        fl = (ll > 0) ? (1 << ll) : 1;
        ok = !vill && li >= -3 && li <= 3;
        if (!alg(d.vd, ll) || !alg(d.vs2, li)) ok = 1'b0;
        if (nf * fl > 8 || int'(d.vd) + nf * fl > 32) ok = 1'b0;
        if (!d.st) begin
          for (int f = 0; f < 8; f++)
            if (f < nf) begin
              int sv;
              sv = int'(d.vd) + fl * f;
              if (d.eb > sew) begin
                if (sv != int'(d.vs2) && ovl0(sv, lsz(ll), d.vs2, lsz(li))) ok = 1'b0;
              end else if (d.eb < sew) begin
                if (li < 0 ? ovl0(sv, lsz(ll), d.vs2, lsz(li)) : ovlw(sv, lsz(ll), d.vs2, lsz(li))) ok = 1'b0;
              end
              if (nf >= 2 && ovl0(sv, lsz(ll), d.vs2, lsz(li))) ok = 1'b0;
            end
          if (!d.vm && d.vd == 5'd0) ok = 1'b0;
        end
      end
      default: begin                             // 单位步长 / 跨步 / fault-only-first (含分段)
        le = int'(d.eb) - int'(sew) + ll;        // log2 EMUL
        em = (le > 0) ? (1 << le) : 1;
        ok = !vill && le >= -3 && le <= 3;
        if (le > 0 && !al(d.vd, 1 << le)) ok = 1'b0;
        if (nf * em > 8 || int'(d.vd) + nf * em > 32) ok = 1'b0;
        if (!d.st && !d.vm && d.vd == 5'd0) ok = 1'b0;      // load：require_vm
      end
    endcase
    return ok;
  endfunction

  //==========================================================================
  // vsetvl
  //==========================================================================
  typedef struct packed {
    logic           vill;
    logic [8:0]     vt;
    logic [VLW-1:0] vl;
  } vset_t;
  function automatic int vlmax_log2(input logic [1:0] s, input logic [2:0] l);
    return LVLEN - (int'(s) + 3) + lmul_log2(l);
  endfunction
  function automatic vset_t vsetvl(input logic [63:0] nt, input logic [63:0] avl, input logic rd0, input logic rs10);
    vset_t r;
    int    lg, lgold;
    logic  ill;
    lg  = vlmax_log2(nt[4:3], nt[2:0]);
    ill = (nt[63:9] != '0) || nt[5] || (nt[2:0] == 3'b100) || (nt[8] && nt[4:3] > 2'd1) ||
          // SEW > min(LMUL, 1) * ELEN：小数 LMUL 时 SEW <= LMUL * 64
          (nt[2] && (int'(nt[4:3]) + 3 > 6 + lmul_log2(nt[2:0])));
    lgold = vill ? -1 : vlmax_log2(sew, lmul);
    if (rd0 && rs10 && (ill || lgold != lg)) ill = 1'b1;     // x0, x0 不能改 VLMAX
    r.vill = ill;
    r.vt   = ill ? 9'd0 : nt[8:0];
    if (ill)               r.vl = '0;
    else if (rd0 && rs10)  r.vl = vl;
    else if (rs10)         r.vl = VLW'(1 << lg);
    else                   r.vl = (avl < 64'(1 << lg)) ? VLW'(avl) : VLW'(1 << lg);
    return r;
  endfunction

  //==========================================================================
  // 请求处理
  //==========================================================================
  typedef enum logic [1:0] {R_IDLE, R_DEC, R_SYNC, R_MEM} rst_e;
  rst_e        rs;
  // 请求：空闲时来的请求当拍就判定 (异步指令 / 非法当拍应答)，同步指令再用锁存的副本 (*_q)
  logic [31:0] ri_q;
  logic [63:0] ra_q, rb_q;
  logic [2:0]  rrm_q;
  wire         r_new = (rs == R_IDLE) && req_v_i;
  logic [31:0] ri;               // 在处理的请求
  logic [63:0] ra;
  assign ri  = r_new ? req_inst_i : ri_q;
  assign ra  = r_new ? req_a_i    : ra_q;
  assign rrm = r_new ? frm_i      : rrm_q;
  vdec_t       rd_;
  vdec_t       rq;               // 锁存副本的译码：同步执行 / 访存 / 应答数据用 (rd_ 经 r_new 选择器连着请求端口，
                                 //   只给空闲 / R_DEC 当拍判定；其余都用 rq，免得端口到后面各级的假路径)
  logic        rlegal;

  // 指令队列 (异步指令)
  logic [31:0] q_inst [QD];
  logic [63:0] q_a    [QD];
  logic [2:0]  q_rm   [QD];
  logic [$clog2(QD)-1:0] q_h, q_t;
  logic [$clog2(QD):0]   q_n;

  // 访存引擎状态 (逻辑在后面)
  logic        m_fin, m_flt;
  logic [5:0]  m_cause;
  logic [63:0] m_tval;
  // 执行
  logic        x_busy;
  logic        s2_v, s3_v;       // 拍流水 S2 (计算) / S3 (写回) 有效 (见执行流水)
  localparam int ANP = 4;        // S2 之后的运算流水级数 (= 元素运算槽 o6_vu_alu 的流水级数，>= 4)
  logic [ANP-1:0] s2p_v;         // 运算流水各级有效 (S2 -> P1 .. P<ANP> -> S3)
  // 写回上下文：运算流水 / S3 里的拍可能属于已经离开 S1、S2 的前一条指令 (计分板放行后新指令在执行)，
  //   写回要用的指令信息随拍走，不再看在执行的 xd / xcw / xdw
  typedef struct packed {
    kind_e       k;
    logic [4:0]  vd;
    logic [1:0]  cw, dw;          // 计算宽度 / 目的宽度
    logic [31:0] wm;              // 本条写的寄存器 (目的组，保守)
  } wc_t;
  wc_t         p_wc [ANP];
  logic [31:0] p_wr;              // 运算流水里各拍要写的寄存器之并 (计分板)
  wire         drained = (q_n == '0) && !x_busy && !s2_v && (s2p_v == '0) && !s3_v;
  assign idle_o = drained && (rs == R_IDLE);

  // CSR 访问
  logic [11:0] ca;
  logic [63:0] crv, cwv;
  logic        cwr, cill;
  // 读值 / 写值只在同步执行时用 (按锁存副本 ri_q / ra_q)；非法 cill 要在请求那拍判定 (按 ri)
  always_comb begin
    logic [2:0] f3;
    f3 = ri_q[14:12];
    ca = ri_q[31:20];
    unique case (ca)
      CSR_VSTART: crv = 64'(vstart);
      CSR_VXSAT:  crv = 64'(vxsat);
      CSR_VXRM:   crv = 64'(vxrm);
      CSR_VCSR:   crv = {61'b0, vxrm, vxsat};
      CSR_VL:     crv = 64'(vl);
      CSR_VTYPE:  crv = vtype_rd;
      CSR_VLENB:  crv = 64'(VLEN / 8);
      default:    crv = '0;
    endcase
    // 写：RW 总写，RS / RC 在 rs1 (或 zimm) 非零时写
    cwr = (f3[1:0] == 2'b01) || (ri_q[19:15] != 5'd0);
    unique case (f3[1:0])
      2'b01:   cwv = f3[2] ? 64'(ri_q[19:15]) : ra_q;
      2'b10:   cwv = crv | (f3[2] ? 64'(ri_q[19:15]) : ra_q);
      default: cwv = crv & ~(f3[2] ? 64'(ri_q[19:15]) : ra_q);
    endcase
  end
  always_comb begin
    logic [11:0] a;
    logic        w;
    a = ri[31:20];
    w = (ri[13:12] == 2'b01) || (ri[19:15] != 5'd0);
    cill = !(a == CSR_VSTART || a == CSR_VXSAT || a == CSR_VXRM || a == CSR_VCSR ||
             a == CSR_VL || a == CSR_VTYPE || a == CSR_VLENB);
    if (w && (a == CSR_VL || a == CSR_VTYPE || a == CSR_VLENB)) cill = 1'b1;   // 只读
  end

  // vsetvl 的新值
  vset_t vs_n;
  always_comb begin
    logic [63:0] nt, avl;
    logic        rd0, rs10;
    rd0 = (ri_q[11:7] == 5'd0);                                     // 同步执行时用：按锁存副本
    if (!ri_q[31])                 begin nt = {53'b0, ri_q[30:20]}; avl = ra_q; rs10 = (ri_q[19:15] == 5'd0); end   // vsetvli
    else if (ri_q[31:30] == 2'b11) begin nt = {54'b0, ri_q[29:20]}; avl = 64'(ri_q[19:15]); rs10 = 1'b0; end      // vsetivli
    else                           begin nt = rb_q;                 avl = ra_q; rs10 = (ri_q[19:15] == 5'd0); end   // vsetvl
    vs_n = vsetvl(nt, avl, rd0, rs10);
  end

  logic [63:0] ra_x;              // 进队列的标量操作数
  always_comb begin
    ra_x = ra;
    if (rd_.fp && rd_.src == 2'd1)
      unique case (sew)
        2'd1: ra_x = (&ra[63:16]) ? {48'b0, ra[15:0]} : (rd_.bf ? 64'h7FC0 : 64'h7E00);
        2'd2: ra_x = (&ra[63:32]) ? {32'b0, ra[31:0]} : 64'h7FC0_0000;
        default: ;
      endcase
  end

  //--------------------------------------------------------------------------
  // 读寄存器堆的同步指令 (vmv.x.s / vfmv.f.s / vcpop.m / vfirst.m)：排空后分两拍 (P5)，结果寄存后应答
  //   第 1 拍 (sy_ph 0 -> 1)：vs2 元素 0；vs2 & (vm | v0) & (i < vl) 按 64 位分块的 1 的个数、块内第一个 1、块非零
  //   第 2 拍 (1 -> 2)：块计数求和、取第一个非零块，结果寄存；sy_ph = 2 那拍应答。
  //   原先一拍做完：32 选 1 读寄存器堆 + 1024 位逐位计数 / 找第一个 1 (串行) 直接到应答端口
  //--------------------------------------------------------------------------
  localparam int NCH = VLEN / 64;
  wire  sy_k = (rq.k == K_VMVXS) || (rq.k == K_VFMVFS) || (rq.k == K_VCPOP) || (rq.k == K_VFIRST);
  logic [1:0]      sy_ph;
  logic [63:0]     sy_e0;                                  // vs2 元素 0 的原值
  logic [6:0]      sy_cnt [NCH];
  logic [5:0]      sy_fi  [NCH];
  logic [NCH-1:0]  sy_nz;
  logic [63:0]     vmvxs, vfmvfs, vpop, vfst;              // 寄存后的应答数据
  function automatic logic [6:0] pop64(input logic [63:0] x);   // 6 级二叉加法树
    logic [6:0] t [64];
    for (int i = 0; i < 64; i++) t[i] = 7'(x[i]);
    for (int w = 32; w >= 1; w = w / 2)
      for (int i = 0; i < w; i++) t[i] = t[2*i] + t[2*i+1];
    return t[0];
  endfunction
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni)                                  sy_ph <= '0;
    else if (rs != R_SYNC || !sy_k)               sy_ph <= '0;
    else if (drained && sy_ph != 2'd2)            sy_ph <= sy_ph + 2'd1;
  always_ff @(posedge clk_i) begin
    if (sy_ph == 2'd0) begin
      sy_e0 <= vrf[rq.vs2][63:0];
      for (int c = 0; c < NCH; c++) begin
        logic [63:0] b, bm;
        // 体内 (i < vl) 的位
        if (int'(vl) >= 64 * (c + 1)) bm = '1;
        else if (int'(vl) <= 64 * c)  bm = '0;
        else                          bm = ~(64'(-1) << (int'(vl) - 64 * c));
        b = vrf[rq.vs2][64*c +: 64] & (rq.vm ? '1 : vrf[0][64*c +: 64]) & bm;
        sy_cnt[c] <= pop64(b);
        sy_fi[c]  <= 6'(clz64(brev64(b)));                // 最低的 1 的位置 (b = 0 时无意义，看 sy_nz)
        sy_nz[c]  <= |b;
      end
    end
    if (sy_ph == 2'd1) begin
      logic [VLW:0] c;
      logic         f;
      c = '0;
      for (int k = 0; k < NCH; k++) c = c + (VLW+1)'(sy_cnt[k]);
      vpop <= 64'(c);
      f = 1'b0; vfst <= '1;
      for (int k = 0; k < NCH; k++)                       // NCH (16) 块的优先选择
        if (sy_nz[k] && !f) begin f = 1'b1; vfst <= 64'(64 * k + int'(sy_fi[k])); end
      unique case (sew)                                   // vmv.x.s：按 SEW 符号扩展；vfmv.f.s：窄于 64 位上半补 1
        2'd0:    begin vmvxs <= {{56{sy_e0[7]}},  sy_e0[7:0]};  vfmvfs <= sy_e0; end
        2'd1:    begin vmvxs <= {{48{sy_e0[15]}}, sy_e0[15:0]}; vfmvfs <= {48'hFFFF_FFFF_FFFF, sy_e0[15:0]}; end
        2'd2:    begin vmvxs <= {{32{sy_e0[31]}}, sy_e0[31:0]}; vfmvfs <= {32'hFFFF_FFFF, sy_e0[31:0]}; end
        default: begin vmvxs <= sy_e0; vfmvfs <= sy_e0; end
      endcase
    end
  end

  wire q_full = (q_n == ($clog2(QD)+1)'(QD));
  wire async_k = (rd_.k == K_EW) || (rd_.k == K_EWM) || (rd_.k == K_VMVSX) || (rd_.k == K_RED) ||
                 (rd_.k == K_MLOG) || (rd_.k == K_MSET) || (rd_.k == K_VIOTA) || (rd_.k == K_PERM) ||
                 (rd_.k == K_VMVNR) || (rd_.k == K_FEW) || (rd_.k == K_FEWM) || (rd_.k == K_FRED) ||
                 (rd_.k == K_DOT);
  // 本拍完成 (R_DEC：非法 / 异步入队；R_SYNC：排空后执行)
  logic do_rsp, do_enq, do_sync;
  always_comb begin
    do_rsp = 1'b0; do_enq = 1'b0; do_sync = 1'b0;
    if (rs == R_DEC || r_new) begin
      if (!rlegal || (rd_.k == K_CSR && cill)) do_rsp = 1'b1;
      else if (async_k) begin if (!q_full) begin do_rsp = 1'b1; do_enq = 1'b1; end end
    end else if (rs == R_SYNC && drained && rq.k != K_VMEM && (!sy_k || sy_ph == 2'd2)) begin
      do_rsp = 1'b1; do_sync = 1'b1;
    end else if (rs == R_MEM && m_fin) begin
      do_rsp = 1'b1;
    end
  end

  always_comb begin
    rsp_v_o     = do_rsp;
    rsp_excp_o  = ((rs == R_DEC || r_new) && (!rlegal || (rd_.k == K_CSR && cill))) || (rs == R_MEM && m_flt);
    rsp_cause_o = (rs == R_MEM) ? m_cause : EXC_ILLEGAL_INST;
    rsp_tval_o  = (rs == R_MEM) ? m_tval : {32'b0, ri};
    unique case (rq.k)                                  // 只有同步执行的应答带数据 (空闲当拍应答的是非法 / 异步)
      K_VSET:  rsp_data_o = 64'(vs_n.vl);
      K_CSR:   rsp_data_o = crv;
      K_VMVXS: rsp_data_o = vmvxs;
      K_VCPOP: rsp_data_o = vpop;
      K_VFIRST: rsp_data_o = vfst;
      K_VFMVFS: rsp_data_o = vfmvfs;
      default: rsp_data_o = '0;
    endcase
    // VS 变脏：收下 (不含只读的 CSR 访问)
    // 访存无论成败都会写 vstart (Spike 出错时写出错元素号)，同样置脏
    vs_dirty_o = do_rsp && ((rs == R_MEM) || (!rsp_excp_o && !(rs == R_SYNC && rq.k == K_CSR && !cwr)));
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rs <= R_IDLE;
    end else begin
      unique case (rs)
        R_IDLE: if (req_v_i) begin                      // 当拍判定：应答了就留在空闲；同步指令去排空；队列满则等
          if (!do_rsp) rs <= (rlegal && !async_k && !(rd_.k == K_CSR && cill)) ? R_SYNC : R_DEC;
        end
        R_DEC: begin
          if (do_rsp)                                     rs <= R_IDLE;
          else if (rlegal && !async_k && !(rd_.k == K_CSR && cill)) rs <= R_SYNC;
        end
        R_SYNC: if (drained && (!sy_k || sy_ph == 2'd2)) rs <= (rq.k == K_VMEM) ? R_MEM : R_IDLE;
        R_MEM:  if (m_fin) rs <= R_IDLE;
        default: rs <= R_IDLE;
      endcase
    end
  end
  always_ff @(posedge clk_i) begin
    if (r_new) begin ri_q <= req_inst_i; ra_q <= req_a_i; rb_q <= req_b_i; rrm_q <= frm_i; end
  end
  assign rq = vdec(ri_q);
  always_comb begin
    rd_ = vdec(ri);
    unique case (rd_.k)
      K_VSET:  rlegal = 1'b1;
      K_CSR:   rlegal = 1'b1;
      K_VMVXS: rlegal = !vill && (vstart == '0);
      K_VFMVFS: rlegal = !vill && rd_.sewm[sew] && (rrm <= 3'd4);     // VI_VFP_COMMON (不查 vstart)
      K_VMEM:  rlegal = vmlegal(rd_);
      default: rlegal = vlegal(rd_);
    endcase
  end

  //==========================================================================
  // 执行流水 (异步指令)：队列头 -> 分拍
  //   每拍处理 E = DLEN / 计算宽度 个元素 (计算宽度 = max(目的 EEW, vs2 EEW))；
  //   源 / 目的按各自 EEW 取同样 E 个元素的切片 (切片按自身大小对齐，不跨寄存器)
  //==========================================================================
  logic [31:0]    xi;            // 在执行的指令
  logic [63:0]    xa;
  logic [2:0]     xrm;           // 本条的 frm
  vdec_t          xd;
  logic [1:0]     xcw, xdw, xs2w;
  logic [1:0]     xsew;          // 本条的 SEW (vmv<nr>r 按字节)
  logic [VLW-1:0] xvl;           // 本条的元素数 (vmv<nr>r = 整组字节数)
  logic [VLW-1:0] xb;            // 当前拍号 (逐元素类：元素号)
  logic [VLW-1:0] xnb;           // 总拍数
  logic [VLW-1:0] xpn;           // 点积：每遍的拍数
  wire  [31:0]    qi = q_inst[q_h];
  logic           x_start;
  logic           x_bdone;       // 本拍完成 (除法要迭代 W 拍)
  // VLMAX (当前 vtype)
  wire  [VLW-1:0] vlmax = VLW'(((VLEN >> (int'(sew) + 3)) << 3) >> (3 - lmul_log2(lmul)));

  // 计算宽度为 cw 时每拍元素数 E = DLEN >> (cw + 3)；总拍数 = ceil(n / E)
  function automatic logic [VLW-1:0] nbeats(input logic [1:0] cw, input int n);
    int le;
    le = LDLEN - 3 - int'(cw);
    return VLW'((n + (1 << le) - 1) >> le);
  endfunction
  // 一条指令的执行参数：{计算宽度, 目的宽度, vs2 宽度, SEW, 元素数, 拍数}
  typedef struct packed {
    logic [1:0]     cw, dw, s2w, se;
    logic [VLW-1:0] n, nb;
    logic [VLW-1:0] pn;                                    // 点积：每遍的拍数
  } xp_t;
  function automatic xp_t xparam(input vdec_t d);
    xp_t p;
    p.se = sew; p.n = vl;
    p.cw = cwid(d, sew); p.dw = dwid(d, sew); p.s2w = s2wid(d, sew);
    p.pn = nbeats(sew, int'(vl));
    if (d.q) begin p.cw = sew + 2'd2; p.s2w = sew + 2'd2; p.dw = sew; end
    unique case (d.k)
      K_VMVNR: begin                                       // 整组按字节复制，不看 vl
        p.se = 2'd0; p.cw = 2'd0; p.dw = 2'd0; p.s2w = 2'd0;
        p.n  = VLW'((int'(d.vs1) + 1) * (VLEN / 8));
      end
      K_RED, K_FRED: begin p.cw = sew; p.s2w = sew; p.dw = (d.wcls != 2'd0) ? sew + 2'd1 : sew; end
      K_DOT: begin p.cw = sew; p.s2w = sew; p.dw = (d.op == D_BF) ? sew + 2'd1 : sew + 2'd2; end   // vfwdota 是 2*SEW
      default: ;
    endcase
    unique case (d.k)
      K_PERM:          p.nb = vl;                           // 逐元素
      K_MLOG, K_MSET:  p.nb = (vl != '0) ? VLW'(1) : '0;     // 整寄存器一拍
      K_VMVSX:         p.nb = (vl != '0) ? VLW'(1) : '0;
      // 点积：整数一遍、浮点两遍，再加一拍写回 (vl = 0 时不做)
      K_DOT:           p.nb = (vl == '0) ? '0 : (d.fp ? (p.pn << 1) + 1'b1 : p.pn + 1'b1);
      default:         p.nb = nbeats(p.cw, int'(p.n));
    endcase
    return p;
  endfunction

  //--------------------------------------------------------------------------
  // 计分板 (P3)：走 S2 的指令只等 S2 空 (S2 的逻辑看在执行的 xd)，不等运算流水排空 ——
  //   条件是它要读的寄存器与运算流水里在途的写不相交 (S3 那拍本沿写完，下一拍 S1 才读，不用比)。
  //   读 / 写集合按寄存器组保守估计 (宽度 w 的操作数占 LMUL << (w - SEW) 个，至多 8；掩码寄存器也按组算)。
  //   其余类 (浮点通道、掩码逻辑、置换、vmv.s.x：S1 直接进 S3) 照旧要运算流水排空，免得与流水出口抢 S3
  //--------------------------------------------------------------------------
  function automatic logic [31:0] rgm(input logic [4:0] r, input int n);   // 寄存器 r .. r+n-1
    return 32'(((64'(1) << n) - 64'(1)) << r);
  endfunction
  function automatic int rspan(input logic [1:0] w);                        // 宽度 w 的操作数占几个寄存器
    int n;
    n = grp(lmul) << ((w > sew) ? int'(w) - int'(sew) : 0);
    return (n > 8) ? 8 : n;
  endfunction
  function automatic logic [31:0] xrds(input vdec_t d, input xp_t p);       // 读集合
    logic [31:0] m;
    if (d.k == K_VMVNR) return rgm(d.vs2, int'(d.vs1) + 1);
    m = rgm(d.vs2, (d.k == K_VIOTA) ? 1 : rspan(p.s2w)) | rgm(d.vd, rspan(p.dw));   // vd：乘加读旧值
    if (d.src == 2'd0) m = m | rgm(d.vs1, rspan(sew));
    if (!d.vm || d.cin) m[0] = 1'b1;                                        // v0：掩码 / 进位
    return m;
  endfunction
  function automatic logic [31:0] xwrs(input vdec_t d, input xp_t p);       // 写集合
    return (d.k == K_VMVNR) ? rgm(d.vd, int'(d.vs1) + 1) : rgm(d.vd, rspan(p.dw));
  endfunction
  vdec_t       qd;
  xp_t         qp;
  logic        q_s2k;
  logic [31:0] x_wm;             // 在执行的指令的写集合 (进运算流水时随拍带上)
  always_comb begin
    qd = vdec(qi);
    qp = xparam(qd);
    q_s2k = (qd.k == K_EW) || (qd.k == K_EWM) || (qd.k == K_VIOTA) || (qd.k == K_VMVNR) ||
            (qd.k == K_RED) || (qd.k == K_DOT);
    x_start = !x_busy && (q_n != '0) && !s2_v &&
              ((s2p_v == '0) || (q_s2k && (xrds(qd, qp) & p_wr) == '0));
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      x_busy <= 1'b0; q_h <= '0; q_t <= '0; q_n <= '0;
    end else begin
      if (do_enq) q_t <= q_t + 1'b1;
      if (x_start) q_h <= q_h + 1'b1;
      q_n <= q_n + do_enq - x_start;
      if (x_start) begin
        xp_t p;
        p = xparam(vdec(qi));
        x_busy <= (p.nb != '0);
      end else if (x_busy && x_bdone && xb + 1'b1 >= xnb) x_busy <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    if (do_enq) begin q_inst[q_t] <= ri; q_a[q_t] <= ra_x; q_rm[q_t] <= rrm; end
    if (x_start) begin
      vdec_t d;
      xp_t   p;
      d = vdec(qi);
      p = xparam(d);
      if (d.k == K_VMVNR) begin d.op = A_EXT; d.allb = 1'b1; d.vm = 1'b1; end
      xi <= qi; xa <= q_a[q_h]; xd <= d; xb <= '0; xrm <= q_rm[q_h];
      xcw <= p.cw; xdw <= p.dw; xs2w <= p.s2w; xsew <= p.se; xvl <= p.n; xnb <= p.nb; xpn <= p.pn;
      x_wm <= xwrs(d, p);
    end else if (x_busy && x_bdone) xb <= xb + 1'b1;
  end

  //--------------------------------------------------------------------------
  // 定宽取 / 放元素：宽度 w 是变量时按 case 展开成 4 种定宽切片 (综合成选择器 / 移位器)。
  // 不要写成 "for k: x[(j * w + k) % N]" 这种变量宽度乘变量下标的逐位循环 —— DC 会为每一位造乘法器和
  // 上万输入的选择器 (VU 曾因此 elaborate 吃到 46 GB 内存)
  //--------------------------------------------------------------------------
  function automatic logic [63:0] eget(input logic [DLEN-1:0] s, input int j, input logic [1:0] w);
    unique case (w)
      2'd0:    return 64'(s[8  * (j % (DLEN / 8))  +: 8]);
      2'd1:    return 64'(s[16 * (j % (DLEN / 16)) +: 16]);
      2'd2:    return 64'(s[32 * (j % (DLEN / 32)) +: 32]);
      default: return s[64 * (j % (DLEN / 64)) +: 64];
    endcase
  endfunction
  function automatic logic [DLEN-1:0] eput(input logic [63:0] v, input int j, input logic [1:0] w);
    logic [DLEN-1:0] r;
    r = '0;
    unique case (w)
      2'd0:    r[8  * (j % (DLEN / 8))  +: 8]  = v[7:0];
      2'd1:    r[16 * (j % (DLEN / 16)) +: 16] = v[15:0];
      2'd2:    r[32 * (j % (DLEN / 32)) +: 32] = v[31:0];
      default: r[64 * (j % (DLEN / 64)) +: 64] = v;
    endcase
    return r;
  endfunction
  // 前导零个数 (x = 0 时 64)：6 级 "高半为零则左移" (log 深度；不要写成逐位的 "if (x[k]) n = k" 循环，
  // 那是 64 级串联的选择器)
  function automatic logic [6:0] clz64(input logic [63:0] x);
    logic [63:0] v;
    logic [6:0]  n;
    v = x; n = '0;
    if (x == '0) return 7'd64;
    if (v[63:32] == '0) begin n[5] = 1'b1; v = v << 32; end
    if (v[63:48] == '0) begin n[4] = 1'b1; v = v << 16; end
    if (v[63:56] == '0) begin n[3] = 1'b1; v = v << 8;  end
    if (v[63:60] == '0) begin n[2] = 1'b1; v = v << 4;  end
    if (v[63:62] == '0) begin n[1] = 1'b1; v = v << 2;  end
    if (!v[63])           n[0] = 1'b1;
    return n;
  endfunction
  function automatic logic [63:0] brev64(input logic [63:0] x);
    logic [63:0] r;
    for (int k = 0; k < 64; k++) r[k] = x[63 - k];
    return r;
  endfunction
  // 低 8 << w 位按 sx 符号 / 零扩展到 64 位
  function automatic logic [63:0] xtnd(input logic [63:0] v, input logic [1:0] w, input logic sx);
    unique case (w)
      2'd0:    return sx ? {{56{v[7]}},  v[7:0]}  : {56'b0, v[7:0]};
      2'd1:    return sx ? {{48{v[15]}}, v[15:0]} : {48'b0, v[15:0]};
      2'd2:    return sx ? {{32{v[31]}}, v[31:0]} : {32'b0, v[31:0]};
      default: return v;
    endcase
  endfunction
  // 宽度 w 的元素的最高位
  function automatic logic emsb(input logic [63:0] v, input logic [1:0] w);
    unique case (w)
      2'd0:    return v[7];
      2'd1:    return v[15];
      2'd2:    return v[31];
      default: return v[63];
    endcase
  endfunction

  //--------------------------------------------------------------------------
  // 切片：寄存器组 base 里、宽度 w (log2(EEW/8)) 的元素 e0.. 共 E 个 -> 放在低位
  //   (切片按自身大小对齐，必在一个 DLEN 块内：先选寄存器，再选块，再在块内移位)
  //--------------------------------------------------------------------------
  function automatic logic [DLEN-1:0] vslice(input logic [4:0] base, input logic [1:0] w, input int e0);
    int bo;
    logic [VLEN-1:0] rg;
    logic [DLEN-1:0] c;
    bo = e0 << (int'(w) + 3);
    rg = vrf[base + 5'(bo >> LVLEN)];
    c  = rg[DLEN * ((bo % VLEN) / DLEN) +: DLEN];
    return c >> ((bo % VLEN) % DLEN);
  endfunction

  wire  [5:0]      le  = 6'(LDLEN - 3 - int'(xcw));          // log2 E
  // 本拍的元素起点；浮点点积的第二遍从头再扫一遍
  wire  [VLW-1:0]  xbx = (xd.k == K_DOT && xd.fp && xb >= xpn) ? xb - xpn : xb;
  //--------------------------------------------------------------------------
  // 拍流水：S1 发射 (按拍号 xb 读寄存器堆切片) -> S2 计算 -> S3 格式化 + 写寄存器堆
  //   计算类 (元素运算 / 掩码目的 / viota / vmv<nr>r / 归约 / 点积) 走 S1 -> S2 -> S3；
  //   浮点 (fpnew 通道自带流水，完成时)、掩码逻辑、vmsbf 类、置换、vmv.s.x 由 S1 直接进 S3。
  //   同一条指令里读总在写之前 (后面的拍读的区域不会是前面的拍写过的)，写回晚几拍不影响结果；
  //   新指令要 S2 空；走 S2 的指令经计分板可不等运算流水排空 (见 x_start)，同步指令要全部排空
  //--------------------------------------------------------------------------
  wire  [VLW-1:0]  e0_1  = VLW'(int'(xbx) << le);
  wire  [DLEN-1:0] s1_vs2 = vslice(xd.vs2, xs2w, int'(e0_1));
  wire  [DLEN-1:0] s1_vs1 = vslice(xd.vs1, xsew, int'(e0_1));
  wire  [DLEN-1:0] s1_vd  = vslice(xd.vd,  xdw,  int'(e0_1));

  // v0 里本拍的掩码位；体内元素 (e0 + j < vl)
  logic [NEB-1:0]  c_mask1, c_body1;
  always_comb begin
    for (int j = 0; j < NEB; j++) begin
      c_body1[j] = (j < (1 << le)) && (int'(e0_1) + j < int'(xvl));
    end
    c_mask1 = NEB'(vrf[0] >> e0_1);
  end

  // 本拍要写的元素：体内且 (不掩码 / 写全部体内 / 掩码位为 1)
  logic [NEB-1:0] c_act1;
  always_comb
    for (int j = 0; j < NEB; j++)
      c_act1[j] = c_body1[j] && (xd.vm || xd.allb || c_mask1[j]);

  // 走 S2 的类；S1 本拍能发射 (S2 类要 S2 空或本拍前进；浮点 = 写回拍的结果都回来了；其余每拍一个)
  wire x_s2k = (xd.k == K_EW) || (xd.k == K_EWM) || (xd.k == K_VIOTA) || (xd.k == K_VMVNR) ||
               (xd.k == K_RED) || (xd.k == K_DOT);
  logic            s2_adv;                                  // S2 的拍本拍进 S3
  logic            s2_first, s2_last;
  logic [VLW-1:0]  s2_xb;
  logic [VLW-1:0]  e0;                                      // S2 拍的元素起点
  logic [DLEN-1:0] s_vs2, s_vs1, s_vd;                      // S2 拍的切片
  logic [NEB-1:0]  c_mask, c_act;
  logic [NEB-1:0]  io_win;                                  // viota：vs2 (掩码) 里本拍的位
  logic [63:0]     s2_v0d;                                  // 点积最后一拍：vd 元素 0 的旧值
  wire             s1_s2 = x_busy && x_s2k && x_bdone;      // 本拍发射进 S2
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) s2_v <= 1'b0;
    else if (s1_s2) s2_v <= 1'b1;
    else if (s2_adv) s2_v <= 1'b0;
  end
  always_ff @(posedge clk_i)
    if (s1_s2) begin
      e0 <= e0_1; s2_xb <= xb; s2_first <= (xb == '0); s2_last <= (xb + 1'b1 >= xnb);
      s_vs2 <= s1_vs2; s_vs1 <= s1_vs1; s_vd <= s1_vd;
      c_mask <= c_mask1; c_act <= c_act1;
      io_win <= NEB'(vrf[xd.vs2] >> e0_1);
      s2_v0d <= vrf[xd.vd][63:0];
    end

  // 把宽度 fw 的切片展开到计算宽度 tw (每个槽 8 << tw 位)，按 sx 符号扩展
  function automatic logic [DLEN-1:0] vexpand(input logic [DLEN-1:0] s, input logic [1:0] fw, input logic [1:0] tw,
                                              input logic sx);
    logic [DLEN-1:0] r;
    unique case (tw)
      2'd0:    r = s;                                     // 8 -> 8
      2'd1:    for (int j = 0; j < DLEN / 16; j++) r[16*j +: 16] = 16'(xtnd(eget(s, j, fw), fw, sx));
      2'd2:    for (int j = 0; j < DLEN / 32; j++) r[32*j +: 32] = 32'(xtnd(eget(s, j, fw), fw, sx));
      default: for (int j = 0; j < DLEN / 64; j++) r[64*j +: 64] = xtnd(eget(s, j, fw), fw, sx);
    endcase
    return r;
  endfunction

  // 操作数 (都按计算宽度 xcw 排布)
  logic [DLEN-1:0] o_a, o_b, o_c;
  always_comb begin
    logic [63:0] sc;
    o_a = vexpand(s_vs2, xs2w, xcw, xd.a_sx);
    o_c = s_vd;
    // 第二操作数：vs1 (SEW) 或标量 / 立即数 (先截到 SEW 再扩展)
    unique case (xd.src)
      2'd1:    sc = xa;
      2'd2:    sc = {{59{xi[19]}}, xi[19:15]};
      default: sc = {58'b0, xd.u6 & xi[26], xi[19:15]};
    endcase
    if (xd.src == 2'd0) o_b = vexpand(s_vs1, xsew, xcw, xd.b_sx);
    else begin
      logic [DLEN-1:0] rep;
      for (int k = 0; k < DLEN / 64; k++) rep[64*k +: 64] = sc;
      // 复制成 SEW 宽的整拍后按 SEW -> 计算宽度展开 (立即数 simm5 已符号扩展到 64 位)
      unique case (xsew)
        2'd0: for (int k = 0; k < DLEN / 8;  k++) rep[8*k +: 8]   = sc[7:0];
        2'd1: for (int k = 0; k < DLEN / 16; k++) rep[16*k +: 16] = sc[15:0];
        2'd2: for (int k = 0; k < DLEN / 32; k++) rep[32*k +: 32] = sc[31:0];
        default: ;
      endcase
      o_b = vexpand(rep, xsew, xcw, xd.b_sx);
    end
  end

  //--------------------------------------------------------------------------
  // 元素运算 (每种计算宽度一份)：返回 {饱和, 掩码位, 结果}
  //--------------------------------------------------------------------------
  // 每个槽一个 o6_vu_alu (定义在本文件末尾)，按计算宽度 xcw 选出结果

  // 进位输入：merge 用掩码位做选择；进位类 (vadc / vsbc / 带 v0 的 vmadc / vmsbc) 用它作进位，其余为 0
  logic [NEB-1:0] c_cin;
  always_comb
    for (int j = 0; j < NEB; j++) c_cin[j] = (xd.op == A_MERGE || xd.cin) && c_mask[j];
  logic [DLEN-1:0] c_res;     // 结果 (按计算宽度排布)
  logic [NEB-1:0]  c_cmp;     // 掩码结果 (按元素)
  logic [NEB-1:0]  c_sat;     // 饱和 (按元素)
  logic [DLEN-1:0] c_r [4];   // 各计算宽度的结果
  logic [NEB-1:0]  c_m [4], c_s [4];
  for (genvar w = 0; w < 4; w++) begin : g_alu
    localparam int W  = 8 << w;
    localparam int NS = DLEN / W;
    for (genvar e = 0; e < NS; e++) begin : g_s
      o6_vu_alu #(.W(W), .NP(ANP)) u_alu (
        .clk_i, .op_i(xd.op), .a_i(o_a[W*e +: W]), .b_i(o_b[W*e +: W]), .c_i(o_c[W*e +: W]),
        .cin_i(c_cin[e]), .xrm_i(vxrm), .r_o(c_r[w][W*e +: W]), .m_o(c_m[w][e]), .s_o(c_s[w][e])
      );
    end
    if (NS < NEB) begin : g_z
      assign c_m[w][NEB-1:NS] = '0;
      assign c_s[w][NEB-1:NS] = '0;
    end
  end
  always_comb begin                       // 槽的输出在运算流水最后一级：按那一拍自己的计算宽度选
    c_res = c_r[p_wc[ANP-1].cw]; c_cmp = c_m[p_wc[ANP-1].cw]; c_sat = c_s[p_wc[ANP-1].cw];
  end

  //--------------------------------------------------------------------------
  // 除法：每拍所有槽并行做恢复余数除法，W 拍迭代 (每拍 1 位商)；无符号做模，有符号最后修正符号
  //   除 0：商全 1、余数 = 被除数；有符号溢出 (最小值 / -1)：商 = 被除数、余数 = 0 (与 Spike 一致)
  //--------------------------------------------------------------------------
  wire x_fp  = (xd.k == K_FEW) || (xd.k == K_FEWM) || (xd.k == K_FRED);   // 浮点引擎 (见后)
  logic            f_bdone;                // 浮点：写回拍 xb 的全部元素完成
  // 浮点各拍重叠：发射拍 xbi 可比写回拍 xb 超前一拍；结果按拍号奇偶放两个缓冲
  logic [VLW-1:0]  xbi;
  logic [DLEN-1:0] f_res2 [2];             // 结果 (按目的宽度排布)
  logic [NEB-1:0]  f_cmp2 [2];             // 比较结果位
  wire x_div = (xd.k == K_EW) && (xd.op == A_DIVU || xd.op == A_DIV || xd.op == A_REMU || xd.op == A_REM);
  wire x_sgn = (xd.op == A_DIV || xd.op == A_REM);
  logic           dv_ld;                   // 本拍的操作数已装入
  logic [6:0]     dv_cnt;                  // 剩余迭代
  wire  [6:0]     dv_w = 7'(8 << xcw);
  logic           df_go, df_done;          // 点积最后一拍在 S2 / 最后一步已算完 (见点积)
  assign s2_adv  = s2_v && (!x_div || (dv_ld && dv_cnt == '0)) && !(df_go && !df_done);   // 除法迭代 / 点积最后一步时停
  assign x_bdone = x_fp ? f_bdone : x_s2k ? (!s2_v || s2_adv) : 1'b1;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) dv_ld <= 1'b0;
    else if (s2_v && x_div) begin
      if (!dv_ld) begin dv_ld <= 1'b1; dv_cnt <= dv_w; end
      else if (dv_cnt != '0) dv_cnt <= dv_cnt - 1'b1;
      else dv_ld <= 1'b0;
    end
  end
  // 每槽一个除法器 o6_vu_dvs (定义在本文件末尾)；槽 j 的位宽 = 它能遇到的最大元素宽度 (j < DLEN / W)
  logic [63:0] dv_y [NEB];
  for (genvar j = 0; j < NEB; j++) begin : g_dv
    localparam int DW = (j < DLEN / 64) ? 64 : (j < DLEN / 32) ? 32 : (j < DLEN / 16) ? 16 : 8;
    logic [DW-1:0] y;
    o6_vu_dvs #(.W(DW)) u_dv (
      .clk_i,
      .ld_i(s2_v && x_div && !dv_ld), .step_i(s2_v && x_div && dv_ld && dv_cnt != '0),
      .cw_i(xcw), .sgn_i(x_sgn), .div_i(xd.op == A_DIVU || xd.op == A_DIV),
      .a_i(DW'(eget(o_a, j, xcw))), .b_i(DW'(eget(o_b, j, xcw))), .y_o(y)
    );
    assign dv_y[j] = 64'(y);
  end
  // 除法结果 (按计算宽度排布)
  logic [DLEN-1:0] c_div;
  always_comb begin
    c_div = '0;
    for (int j = 0; j < NEB; j++)
      if (j < (DLEN >> (int'(xcw) + 3))) c_div = c_div | eput(dv_y[j], j, xcw);
  end

  // viota / vid：元素号 / 之前被选中元素的 1 的个数 (逐拍累计)
  logic [VLW-1:0] io_cnt;
  logic [DLEN-1:0] c_iota;
  logic [VLW-1:0]  io_add;                // 本拍新增的 1
  // 元素 j 的 viota = 之前选中的个数：前缀和 (Kogge-Stone，log2(NEB) 级；原先逐元素串行加 1)
  localparam int PCW = $clog2(NEB + 1);
  always_comb begin
    logic [PCW-1:0] pc [NEB];
    for (int j = 0; j < NEB; j++) pc[j] = PCW'(c_act[j] && io_win[j]);
    for (int d = 1; d < NEB; d = d * 2)
      for (int j = NEB - 1; j >= d; j--) pc[j] = pc[j] + pc[j - d];     // 倒序：读到的 pc[j-d] 还是上一级的
    c_iota = '0;
    for (int j = 0; j < NEB; j++) begin
      logic [63:0] y;
      logic [PCW-1:0] ex;
      ex = (j == 0) ? '0 : pc[j - 1];                                      // 不含自己
      y  = (xd.op == P_ID) ? 64'(int'(e0) + j) : 64'(io_cnt + VLW'(ex));
      if (j < (1 << le)) c_iota = c_iota | eput(y, j, xsew);
    end
    io_add = VLW'(pc[NEB - 1]);
  end
  always_ff @(posedge clk_i)
    if (x_start) io_cnt <= '0;
    else if (s2_v && xd.k == K_VIOTA) io_cnt <= io_cnt + io_add;

  // 运算流水 (S2 -> P1 .. P<ANP>)：元素运算槽自带 ANP 级输出寄存器；S2 算出的其余结果 (除法 / viota、
  //   归约 / 点积的最终值) 与写回要的拍信息同样延迟 ANP 拍，在最后一级与槽的结果汇合后进 S3。
  //   流水寄存器无使能：S2 停顿 (除法迭代) 时后面照常前进、只是插气泡
  logic [DLEN-1:0] p_alt [ANP];            // 除法 / viota 的结果
  logic [63:0]     p_red [ANP];            // 归约 / 点积 (最后一拍有用)
  logic [VLW-1:0]  p_e0  [ANP];
  logic [NEB-1:0]  p_act [ANP];
  logic [ANP-1:0]  p_last, p_alts;         // 本拍最后一拍 / 取 p_alt
  wire p_out = s2p_v[ANP-1];               // 最后一级有效：本拍进 S3

  // 目的切片 (最后一级)：收窄时把每个元素的低半打包 (宽度用这一拍自己的)
  logic [DLEN-1:0] c_dst;
  always_comb begin
    logic [1:0] cw, dw;
    cw = p_wc[ANP-1].cw; dw = p_wc[ANP-1].dw;
    c_dst = p_alts[ANP-1] ? p_alt[ANP-1] : c_res;
    if (dw != cw) begin
      c_dst = '0;
      for (int j = 0; j < NEB; j++)
        if (j < (DLEN >> (int'(cw) + 3))) c_dst = c_dst | eput(eget(c_res, j, cw), j, dw);
    end
  end


  //--------------------------------------------------------------------------
  // 归约：累加器从 vs1[0] 开始，每拍并入本拍的活动元素；最后一拍写 vd[0]
  //--------------------------------------------------------------------------
  logic [63:0] rd_acc, rd_nacc;
  function automatic logic [63:0] sext_w(input logic [63:0] v, input logic [1:0] w, input logic sx);
    return xtnd(v, w, sx);
  endfunction
  //   本拍活动元素做平衡树 (堆式：叶子在 [NEB, 2NEB)，log2(NEB) 级)，再并入累加器；不活动的元素取单位元。
  //   整数加 / 与 / 或 / 异或 / 最值都满足结合、交换律：写回的低 dw 位与原先逐元素串行的结果逐位相同
  //   (累加器高位是相邻元素的残留，只有低 dw 位写回；最值相等时取谁都一样)
  //   每个节点一个比较器：有符号比较 = 最高位取反后无符号比较；最值相等时两边 (已扩展的 64 位) 本来就一样
  function automatic logic [63:0] rop(input aop_e op, input logic [63:0] x, input logic [63:0] y);
    logic sx, lt;
    sx = (op == R_MIN || op == R_MAX);
    lt = {x[63] ^ sx, x[62:0]} < {y[63] ^ sx, y[62:0]};
    unique case (op)
      R_SUM:         return x + y;
      R_AND:         return x & y;
      R_OR:          return x | y;
      R_XOR:         return x ^ y;
      R_MINU, R_MIN: return lt ? x : y;
      default:       return lt ? y : x;
    endcase
  endfunction
  //   分级 (P2c；原先一拍 5 级 64 位运算 + 并入累加器，QUICK 剖面 444 级)：
  //     S2：叶子 (取元素、扩展、不活动取单位元) + 第 1 级 -> P1：RC2 级 -> P2：其余各级 -> P3：并入累加器 rd_acc，
  //     结果从 P3 起随运算流水到 S3。计分板放行后流水里可能还有上一条归约的拍，所以运算 / 首拍 / 目的宽度 / vs1[0]
  //     都随拍带 (rc)，不看在执行的 xd
  localparam int RL  = $clog2(NEB);                         // 树的级数 (NEB = 32：5)
  localparam int RC2 = RL / 2;                              // P1 做的级数；P2 做 RL - 1 - RC2 级
  typedef logic [NEB-1:0][63:0] rv_t;
  function automatic rv_t rlev(input aop_e op, input rv_t a, input int n, input int lv);   // 前 n 个两两并 lv 级
    for (int l = 0; l < lv; l++) begin
      for (int j = 0; j < n / 2; j++) a[j] = rop(op, a[2*j], a[2*j+1]);   // 升序就地：写 a[j] 时 a[2j..] 还没被覆盖
      n = n / 2;
    end
    return a;
  endfunction
  typedef struct packed {
    aop_e        op;
    logic        red, first;                               // 本拍是归约的拍 / 首拍
    logic [1:0]  dw;
    logic [63:0] init;                                     // vs1[0] (首拍用)
  } rc_t;
  rc_t         rc [3];                                      // P1 / P2 / P3 的控制
  rv_t         rt1, rt2;
  logic [63:0] rt3;
  always_ff @(posedge clk_i) begin
    rv_t lv;
    logic [63:0] id;
    logic        smm;
    smm = xd.op == R_MIN || xd.op == R_MAX;
    unique case (xd.op)
      R_AND, R_MINU: id = '1;
      R_MIN:         id = {1'b0, {63{1'b1}}};
      R_MAX:         id = {1'b1, 63'b0};
      default:       id = '0;
    endcase
    for (int j = 0; j < NEB; j++)
      // 有符号比较 / 加宽有符号求和：按 SEW 符号扩展
      lv[j] = c_act[j] ? sext_w(eget(s_vs2, j, xsew), xsew, smm || xd.a_sx) : id;
    rt1   <= rlev(xd.op, lv, NEB, 1);
    rc[0] <= '{op: xd.op, red: s2_adv && xd.k == K_RED, first: s2_first, dw: xdw, init: s_vs1[63:0]};
    rt2   <= rlev(rc[0].op, rt1, NEB / 2, RC2);
    rc[1] <= rc[0];
    begin
      rv_t t;
      t = rlev(rc[1].op, rt2, NEB >> (1 + RC2), RL - 1 - RC2);
      rt3 <= t[0];
    end
    rc[2] <= rc[1];
  end
  always_comb begin                                         // P3：并入累加器
    logic [63:0] acc;
    logic        smm;
    smm = rc[2].op == R_MIN || rc[2].op == R_MAX;
    // 最值按目的宽度扩展后比较；加 / 逻辑运算的累加器原样
    acc = rc[2].first ? rc[2].init : rd_acc;
    if (rc[2].op == R_MINU || rc[2].op == R_MAXU || smm) acc = sext_w(acc, rc[2].dw, smm);
    rd_nacc = rop(rc[2].op, acc, rt3);
  end
  // 浮点归约的初值 vs1[0]：在开始后的那一拍按在执行的指令读 (x_first，浮点归约这拍不发射)。
  //   原先在 x_start 那拍按队头读 —— 上一条的最后一拍可能还在 S3 (本沿才写)，会读到旧值 (vx9627：
  //   vmflt.vf v14 紧接 vfwredosum v6, v29, v14)；且 "队头译码 -> 寄存器堆 32 选 1 -> 切片" 是一条长路径
  logic            x_first;                                    // x_start 的下一拍
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) x_first <= 1'b0;
    else         x_first <= x_start;
  wire  [DLEN-1:0] rd_init = vslice(xd.vs1, xdw, 0);
  always_ff @(posedge clk_i)
    if (rc[2].red) rd_acc <= rd_nacc;


  //==========================================================================
  // 点积 (OP-VE)：vd[0] (4*SEW) += sum(vs1[i] * vs2[i])，被掩掉 / 尾部的元素按 0
  //   结果宽度：vqwdota* / vfqwdota 是 4*SEW，vfwdota 是 2*SEW (都写 vd 的元素 0)
  //   整数 (vqwdotau / vqwdotas)：一遍，按 4*SEW 回绕累加；altfmt 时 vs1 按有符号，vqwdotas 的 vs2 按有符号
  //   浮点 (Spike bulknormdot.h 的 bulk normalization，组大小 = VLMAX，保护位 g = log2 VLMAX)：
  //     第一遍：乘积近似指数 (两个指数之和) 的最大值 + 无穷 / NaN / 无效等特殊情况
  //     第二遍：乘积尾数 << (23 - ma - mb + g)，按 (最大指数 - 本指数) 带粘滞位右移，有符号累加 (64 位)
  //     最后一拍：规格化成 FP32 (截断 + 粘滞，即 round-to-odd；含 Spike 的 (mag << sig_bits) 64 位截断)，
  //     再以 round-to-odd 加到 vd[0] (溢出时 Spike 把最大有限值 +1 成无穷；只保留 OF / NV 标志)
  //==========================================================================
  wire x_dot = (xd.k == K_DOT);
  wire x_dfp = x_dot && xd.fp;
  wire dt_p1 = x_dfp && (s2_xb < xpn);                       // 浮点第一遍 (S2 的拍)
  // 点积元素格式：0 = E4M3，1 = E5M2，2 = BF16 (a = vs1，b = vs2)
  logic [1:0] dt_fa, dt_fb;
  always_comb begin
    unique case (xd.op)
      D_F8:    begin dt_fa = altf ? 2'd1 : 2'd0; dt_fb = 2'd0; end
      D_F8A:   begin dt_fa = altf ? 2'd1 : 2'd0; dt_fb = 2'd1; end
      default: begin dt_fa = 2'd2; dt_fb = 2'd2; end
    endcase
  end
  function automatic int dbias(input logic [1:0] f);
    return (f == 2'd0) ? 7 : (f == 2'd1) ? 15 : 127;
  endfunction
  function automatic int dmw(input logic [1:0] f);
    return (f == 2'd0) ? 3 : (f == 2'd1) ? 2 : 7;
  endfunction
  // 元素解码：{符号, 修正后的指数 (次正规按 1), 尾数 (含隐含位), 零, 无穷, NaN, sNaN}
  typedef struct packed {
    logic       sg;
    logic [8:0] ex;
    logic [7:0] sig;
    logic       zero, inf, nan, snan;
  } dfe_t;
  function automatic dfe_t ddec(input logic [15:0] v, input logic [1:0] f);
    dfe_t r;
    logic [7:0] e, emax;
    logic [6:0] m;
    logic       sub;
    unique case (f)
      2'd0:    begin r.sg = v[7];  e = {4'b0, v[6:3]}; m = {4'b0, v[2:0]}; emax = 8'h0F; end
      2'd1:    begin r.sg = v[7];  e = {3'b0, v[6:2]}; m = {5'b0, v[1:0]}; emax = 8'h1F; end
      default: begin r.sg = v[15]; e = v[14:7];        m = v[6:0];         emax = 8'hFF; end
    endcase
    sub    = (e == '0);
    r.ex   = 9'(e) + 9'(sub);
    r.sig  = 8'(m) | (8'(!sub) << dmw(f));
    r.zero = sub && (m == '0);
    r.inf  = (f != 2'd0) && (e == emax) && (m == '0);
    r.nan  = (f == 2'd0) ? (e == emax && m == 7'h07) : (e == emax && m != '0);
    r.snan = (f == 2'd2) && r.nan && !m[6];             // OFP8 没有 sNaN
    return r;
  endfunction
  function automatic logic [63:0] jam64(input logic [63:0] n, input int amt);
    logic [63:0] r;
    if (amt >= 64) r = {63'b0, n != '0};
    else begin
      r = n >> amt;
      if ((n & ((64'd1 << amt) - 1)) != '0) r[0] = 1'b1;
    end
    return r;
  endfunction

  logic signed [10:0] dt_max;                                // 乘积近似指数的最大值
  logic [4:0]         dt_fl;                                 // {+inf, -inf, NaN, 无效 (inf*0), sNaN}
  logic [63:0]        dt_acc;                                // 累加 (整数：4*SEW 回绕；浮点：有符号定点)
  logic signed [10:0] dt_bmax;
  logic [4:0]         dt_bfl;
  //   第一遍 (浮点，只看指数与特殊情况，便宜) 留在 S2：各元素的乘积近似指数取最大、特殊情况取或，并入 dt_max / dt_fl。
  //   第二遍 (浮点) / 整数的累加分级 (P2e；原先一拍做完 "译码 -> 乘 -> 对齐移位 -> 取负 -> 32 个 64 位加数的树 -> 累加")：
  //     S2：各元素的积 (整数 17x17 有符号；浮点尾数 8x8、乘积近似指数、符号) -> P1：浮点按 dt_max 对齐 (jam) 并取符号 ->
  //     P2：32 -> 4 -> P3：4 -> 1 并入 dt_acc。
  //   这几级做的时候 xd 一定还是这条点积 (最后一拍停在 S2 等它们做完，见 df_st)；dt_max 在第二遍开始前已定。
  //   64 位补码加法满足结合律，结果与串行逐位相同
  always_comb begin
    logic signed [10:0] mv [2*NEB];
    logic [4:0]         fv [2*NEB];
    for (int j = 0; j < NEB; j++) begin
      logic [63:0] a, b;
      a = eget(s_vs1, j, xsew); b = eget(s_vs2, j, xsew);
      mv[NEB + j] = -11'sd1024; fv[NEB + j] = '0;
      if (c_act[j] && x_dfp) begin
        dfe_t ea, eb;
        int pe, ez;
        ea = ddec(a[15:0], dt_fa); eb = ddec(b[15:0], dt_fb);
        ez = 127 - (dbias(dt_fa) + dbias(dt_fb));
        pe = (ea.zero || eb.zero) ? ez : int'(ea.ex) + int'(eb.ex) + ez;
        mv[NEB + j] = 11'(pe);
        if ((ea.inf || eb.inf) && !(ea.nan || eb.nan) && !(ea.zero || eb.zero)) begin
          if (ea.sg == eb.sg) fv[NEB + j][4] = 1'b1; else fv[NEB + j][3] = 1'b1;
        end
        if ((ea.inf && eb.zero) || (eb.inf && ea.zero)) fv[NEB + j][1] = 1'b1;
        if (ea.nan || eb.nan) fv[NEB + j][2] = 1'b1;
        if (ea.snan || eb.snan) fv[NEB + j][0] = 1'b1;
      end
    end
    for (int k = NEB - 1; k >= 1; k--) begin
      mv[k] = (mv[2*k] > mv[2*k+1]) ? mv[2*k] : mv[2*k+1];
      fv[k] = fv[2*k] | fv[2*k+1];
    end
    dt_bmax = (mv[1] > dt_max) ? mv[1] : dt_max;
    dt_bfl  = fv[1];
  end
  always_ff @(posedge clk_i)
    if (x_start) begin dt_max <= -11'sd1024; dt_fl <= '0; end
    else if (s2_v && x_dot && dt_p1) begin dt_max <= dt_bmax; dt_fl <= dt_fl | dt_bfl; end

  // 第二遍 / 整数的各级 (dq_v[k] = P(k+1) 是要累加的拍)
  logic [2:0]         dq_v;
  logic [33:0]        dp_pr  [NEB];                            // S2 -> P1：积 (整数 34 位有符号；浮点低 16 位尾数积)
  logic signed [10:0] dp_pe  [NEB];                            //   浮点：乘积近似指数
  logic [NEB-1:0]     dp_ng;                                   //   浮点：积为负
  logic               dp_fp;
  logic [63:0]        dq_t1  [NEB];                            // P1 -> P2：对齐后的加数
  logic [63:0]        dq_t2  [4];                              // P2 -> P3
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) dq_v <= '0;
    else         dq_v <= {dq_v[1:0], s2_adv && x_dot && !dt_p1 && !s2_last};
  always_ff @(posedge clk_i) begin
    // S2：各元素的积
    for (int j = 0; j < NEB; j++) begin
      logic [63:0] a, b;
      a = eget(s_vs1, j, xsew); b = eget(s_vs2, j, xsew);
      dp_pr[j] <= '0; dp_pe[j] <= -11'sd1024; dp_ng[j] <= 1'b0;
      if (c_act[j]) begin
        if (!x_dfp) begin
          // SEW 只有 8 / 16：17 位有符号乘 (结果同为按符号扩展后的精确积)
          logic [16:0] ia, ib;
          ia = (xsew == 2'd0) ? {{9{altf && a[7]}}, a[7:0]} : {altf && a[15], a[15:0]};
          ib = (xsew == 2'd0) ? {{9{(xd.op == D_IS) && b[7]}}, b[7:0]} : {(xd.op == D_IS) && b[15], b[15:0]};
          dp_pr[j] <= 34'($signed(ia) * $signed(ib));
        end else begin
          dfe_t ea, eb;
          int ez;
          ea = ddec(a[15:0], dt_fa); eb = ddec(b[15:0], dt_fb);
          ez = 127 - (dbias(dt_fa) + dbias(dt_fb));
          dp_pr[j] <= 34'(ea.sig * eb.sig);
          dp_pe[j] <= 11'((ea.zero || eb.zero) ? ez : int'(ea.ex) + int'(eb.ex) + ez);
          dp_ng[j] <= ea.sg ^ eb.sg;
        end
      end
    end
    dp_fp <= x_dfp;
    // P1：浮点对齐到最大指数 (带粘滞) 并取符号；整数按符号扩展
    begin
      int lsh;
      lsh = 23 - dmw(dt_fa) - dmw(dt_fb) + vlmax_log2(sew, lmul);
      for (int j = 0; j < NEB; j++)
        if (dp_fp) begin
          logic [63:0] ps;
          ps = jam64(64'(dp_pr[j][15:0]) << lsh, int'(dt_max) - int'(dp_pe[j]));
          dq_t1[j] <= dp_ng[j] ? -ps : ps;
        end else dq_t1[j] <= 64'($signed(dp_pr[j]));
    end
    // P2：32 -> 4
    for (int q = 0; q < 4; q++) begin
      logic [63:0] t;
      t = '0;
      for (int j = 0; j < NEB / 4; j++) t = t + dq_t1[q * (NEB / 4) + j];
      dq_t2[q] <= t;
    end
    // P3：4 -> 1，并入累加器
    if (x_start) dt_acc <= '0;
    else if (dq_v[2]) dt_acc <= dt_acc + ((dq_t2[0] + dq_t2[1]) + (dq_t2[2] + dq_t2[3]));
  end

  // FP32 round-to-odd 加法 (Spike f32_add_odd)，分两半 (点积最后一步各一拍)：
  //   前半：特殊值、对齐 (jam)、加 / 减；后半：规格化、溢出、打包。后半返回 {OF, NV, 结果}
  typedef struct packed {
    logic        sp;                                         // 结果已定 (NaN / 无穷 / 抵消为 0)
    logic [33:0] spr;
    logic        sg;
    logic [8:0]  e;
    logic [27:0] sum;
  } fao_t;
  function automatic fao_t fadd_odd_a(input logic [31:0] a, input logic [31:0] b);
    fao_t r;
    logic an, bn, ai, bi, sg;
    logic [7:0]  ea, eb, xa, xb, xg, xl;
    logic [23:0] ma, mb, mg, ml;
    logic [27:0] bg, sm;
    int d;
    r = '0;
    an = (a[30:23] == 8'hFF) && (a[22:0] != '0); bn = (b[30:23] == 8'hFF) && (b[22:0] != '0);
    ai = (a[30:23] == 8'hFF) && (a[22:0] == '0); bi = (b[30:23] == 8'hFF) && (b[22:0] == '0);
    if (an || bn) begin r.sp = 1'b1; r.spr = {1'b0, (an && !a[22]) || (bn && !b[22]), 32'h7FC0_0000}; return r; end
    if (ai && bi && a[31] != b[31]) begin r.sp = 1'b1; r.spr = {2'b01, 32'h7FC0_0000}; return r; end
    if (ai) begin r.sp = 1'b1; r.spr = {2'b00, a}; return r; end
    if (bi) begin r.sp = 1'b1; r.spr = {2'b00, b}; return r; end
    ea = a[30:23]; eb = b[30:23];
    ma = {ea != '0, a[22:0]}; mb = {eb != '0, b[22:0]};
    xa = (ea == '0) ? 8'd1 : ea; xb = (eb == '0) ? 8'd1 : eb;
    if ({xa, ma} >= {xb, mb}) begin sg = a[31]; xg = xa; mg = ma; xl = xb; ml = mb; end
    else                      begin sg = b[31]; xg = xb; mg = mb; xl = xa; ml = ma; end
    d     = int'(xg) - int'(xl);
    bg    = {1'b0, mg, 3'b0};
    sm    = 28'(jam64(64'({1'b0, ml, 3'b0}), d));
    r.sum = (a[31] == b[31]) ? bg + sm : bg - sm;
    if (r.sum == '0) begin r.sp = 1'b1; r.spr = {2'b00, (a[31] == b[31]) ? {a[31], 31'b0} : 32'h0}; return r; end   // 抵消为 0：+0
    r.sg = sg; r.e = 9'(xg);
    return r;
  endfunction
  function automatic logic [33:0] fadd_odd_b(input fao_t r);
    logic [27:0] sum;
    int e;
    if (r.sp) return r.spr;
    sum = r.sum; e = int'(r.e);
    if (sum[27]) begin sum = {1'b0, sum[27:2], sum[1] | sum[0]}; e = e + 1; end
    else begin
      // 左移到 sum[26] 为 1，但 e 不低于 1：移位量 = min(前导零, e - 1)
      int lz, n;
      lz = int'(clz64({sum[26:0], 37'b0}));
      n  = (lz < e - 1) ? lz : e - 1;
      sum = sum << n; e = e - n;
    end
    if (e >= 255) return {2'b10, r.sg, 8'hFF, 23'b0};                              // 溢出：最大有限值 +1 = 无穷
    return {2'b00, r.sg, sum[26] ? 8'(e) : 8'd0, sum[25:3] | {22'b0, sum[2:0] != '0}};
  endfunction

  // 最后一拍的结果：最后一拍停在 S2 (同除法迭代)，先等运算流水里第二遍的拍都并入 dt_acc (dq_v 空)，再分拍算
  //   (P2e；原先一拍从 dt_acc 一路做到结果与 fflags 输出，QUICK 剖面 391 级)。df_st：
  //   0 等累加完 (整数这一拍算 v0 + dt_acc，直接到 5) -> 1 绝对值已寄存 -> 2 最高的 1 -> 3 截断 + jam、打包成 FP32
  //   (round-to-odd) -> 4 与 vd[0] 相加的前半 -> 5 后半，结果 dt_res / 标志 dt_ff 已寄存，本拍前进
  logic [2:0]  df_st;
  assign df_go   = s2_v && x_dot && s2_last;                 // (声明在 s2_adv 前)
  assign df_done = (df_st == 3'd5);
  logic [63:0] dt_res;
  logic [4:0]  dt_ff;
  logic        df_sg;
  logic [63:0] df_mag;
  logic [6:0]  df_nd;
  logic [31:0] df_out;
  logic [4:0]  df_ff;
  fao_t        df_a;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni)                 df_st <= '0;
    else if (!df_go)             df_st <= '0;
    else if (df_st == 3'd0)      begin if (dq_v == '0) df_st <= x_dfp ? 3'd1 : 3'd5; end
    else if (df_st != 3'd5)      df_st <= df_st + 3'd1;
  always_ff @(posedge clk_i) begin
    if (df_go && df_st == 3'd0 && dq_v == '0) begin
      dt_res <= s2_v0d + dt_acc; dt_ff <= '0;               // 整数：只写低 4*SEW 位
      df_sg  <= dt_acc[63];
      df_mag <= dt_acc[63] ? -dt_acc : dt_acc;
    end
    if (df_st == 3'd1) df_nd <= (df_mag == '0) ? 7'd0 : 7'(63 - int'(clz64(df_mag)));   // 最高的 1
    if (df_st == 3'd2) begin
      logic        nanout, ovf, anyinf;
      logic [63:0] t, rs;
      int g, ex, sb;
      g   = vlmax_log2(sew, lmul);
      ex  = int'(dt_max) - 23 - g + int'(df_nd);
      sb  = (ex <= 0) ? 23 - (1 - ex) : 23;
      if (sb < 0) sb = 0;
      t   = df_mag << sb;                                   // 与 Spike 一样按 64 位截断
      rs  = jam64(t, int'(df_nd));
      anyinf = dt_fl[4] || dt_fl[3];
      ovf    = (ex >= 255 && df_mag != '0) || anyinf;
      nanout = dt_fl[2] || dt_fl[1] || (dt_fl[4] && dt_fl[3]);
      if (nanout)              df_out <= 32'h7FC0_0000;
      else if (ovf)            df_out <= {anyinf ? dt_fl[3] : df_sg, 8'hFF, 23'b0};
      else if (df_mag == '0)   df_out <= {df_sg, 8'b0, rs[22:0]};
      else if (ex <= 0)        df_out <= {df_sg, 8'b0, rs[22:0]};
      else                     df_out <= {df_sg, 8'(ex), rs[22:0]};
      df_ff <= '0;
      df_ff[4] <= dt_fl[0] || dt_fl[1] || (dt_fl[4] && dt_fl[3]);
      df_ff[2] <= (ex >= 255 && df_mag != '0) && !anyinf && !nanout;
    end
    if (df_st == 3'd3) df_a <= fadd_odd_a(df_out, s2_v0d[31:0]);
    if (df_st == 3'd4) begin
      logic [33:0] ad;
      ad = fadd_odd_b(df_a);
      dt_res <= {32'b0, ad[31:0]};
      dt_ff  <= df_ff | {ad[32], 1'b0, ad[33], 2'b0};
    end
  end

  //==========================================================================
  // 浮点 (V3)：NFL 条 fpnew 通道 (标量模式：FP64 / FP32 / FP16 / BF16，INT8..INT64)
  //   每拍 E 个元素，通道 l 依次处理元素 l, l + NFL, ...；每条通道自己握手 (fpnew 的 in_ready 依赖 in_valid)，
  //   不活动的元素直接跳过。结果按元素号 (tag) 写进本拍结果缓冲，全部回来后整拍写寄存器堆。
  //   加宽：源先精确升到 2*SEW (sNaN 报 NV，NaN 成默认 NaN，与 Spike 的 f16_to_f32 / f32_to_f64 / bf16_to_f32 相同)。
  //   归约：通道 0 逐元素串行 (结果回来再发下一个)，与 Spike 的求和顺序一致。
  //   vfrec7 / vfrsqrt7：本地查表 (组合)，不进 fpnew。
  //==========================================================================
  localparam int NFL = DLEN / 64;
  localparam int EW  = $clog2(NEB) + 1;          // 拍内元素号 / 计数位宽
  localparam int KW  = EW;

  function automatic fpnew_pkg::fp_format_e ffmt(input logic [1:0] w, input logic bf);
    unique case (w)
      2'd1:    return bf ? fpnew_pkg::FP16ALT : fpnew_pkg::FP16;
      2'd2:    return fpnew_pkg::FP32;
      default: return fpnew_pkg::FP64;
    endcase
  endfunction
  function automatic fpnew_pkg::int_format_e ifmt(input logic [1:0] w);
    unique case (w)
      2'd0:    return fpnew_pkg::INT8;
      2'd1:    return fpnew_pkg::INT16;
      2'd2:    return fpnew_pkg::INT32;
      default: return fpnew_pkg::INT64;
    endcase
  endfunction
  // NaN-boxing：fpnew 要求窄格式输入的上半全 1
  function automatic logic [63:0] fbox(input logic [63:0] v, input logic [1:0] w);
    unique case (w)
      2'd1:    return {48'hFFFF_FFFF_FFFF, v[15:0]};
      2'd2:    return {32'hFFFF_FFFF, v[31:0]};
      default: return v;
    endcase
  endfunction
  // 低 n = 8 << w 位
  function automatic logic [63:0] lowb(input logic [63:0] v, input logic [1:0] w);
    unique case (w)
      2'd0:    return {56'b0, v[7:0]};
      2'd1:    return {48'b0, v[15:0]};
      2'd2:    return {32'b0, v[31:0]};
      default: return v;
    endcase
  endfunction
  // 默认 NaN
  function automatic logic [63:0] fdnan(input logic [1:0] w);
    unique case (w)
      2'd1:    return 64'h7E00;
      2'd2:    return 64'h7FC0_0000;
      default: return 64'h7FF8_0000_0000_0000;
    endcase
  endfunction
  // 是否 NaN / sNaN (w = 1/2/3：FP16 / FP32 / FP64)
  function automatic logic [1:0] fnan(input logic [63:0] v, input logic [1:0] w);
    logic n, q;
    unique case (w)
      2'd1:    begin n = (v[14:10] == 5'h1F) && (v[9:0] != '0);  q = v[9];  end
      2'd2:    begin n = (v[30:23] == 8'hFF) && (v[22:0] != '0); q = v[22]; end
      default: begin n = (v[62:52] == 11'h7FF) && (v[51:0] != '0); q = v[51]; end
    endcase
    return {n, n && !q};
  endfunction

  // 精确升格式：w = 源宽度 (1：FP16 / BF16 -> FP32，2：FP32 -> FP64)；返回 {NV, 结果}
  function automatic logic [64:0] fwiden(input logic [63:0] v, input logic [1:0] w, input logic bf);
    logic [63:0] r;
    logic        nv;
    nv = 1'b0;
    if (w == 2'd1 && bf) begin
      if (v[14:7] == 8'hFF && v[6:0] != '0) begin r = 64'h7FC0_0000; nv = !v[6]; end
      else r = {32'b0, v[15:0], 16'b0};
    end else if (w == 2'd1) begin
      logic [4:0] e;
      logic [9:0] m;
      e = v[14:10]; m = v[9:0];
      if (e == 5'h1F) begin
        if (m == '0) r = {32'b0, v[15], 8'hFF, 23'b0};
        else begin r = 64'h7FC0_0000; nv = !m[9]; end
      end else if (e == '0) begin
        if (m == '0) r = {32'b0, v[15], 31'b0};
        else begin
          int pm;
          logic [31:0] f;
          pm = 0;
          for (int k = 0; k < 10; k++) if (m[k]) pm = k;
          f = ((32'(m) << (10 - pm)) & 32'h3FF) << 13;
          r = {32'b0, v[15], 8'(pm + 103), f[22:0]};
        end
      end else r = {32'b0, v[15], 8'(int'(e) + 112), m, 13'b0};
    end else begin
      logic [7:0]  e;
      logic [22:0] m;
      e = v[30:23]; m = v[22:0];
      if (e == 8'hFF) begin
        if (m == '0) r = {v[31], 11'h7FF, 52'b0};
        else begin r = 64'h7FF8_0000_0000_0000; nv = !m[22]; end
      end else if (e == '0) begin
        if (m == '0) r = {v[31], 63'b0};
        else begin
          int pm;
          logic [63:0] f;
          pm = 0;
          for (int k = 0; k < 23; k++) if (m[k]) pm = k;
          f = ((64'(m) << (23 - pm)) & 64'h7F_FFFF) << 29;
          r = {v[31], 11'(pm + 874), f[51:0]};
        end
      end else r = {v[31], 11'(int'(e) + 896), m, 29'b0};
    end
    return {nv, r};
  endfunction

  // vfrec7 / vfrsqrt7 (Spike softfloat/fall_reciprocal.c)：ne / ns = 指数 / 尾数位数；返回 {fflags, 结果}
  localparam logic [6:0] RSQ_T [128] = '{
    52, 51, 50, 48, 47, 46, 44, 43, 42, 41, 40, 39, 38, 36, 35, 34,
    33, 32, 31, 30, 30, 29, 28, 27, 26, 25, 24, 23, 23, 22, 21, 20,
    19, 19, 18, 17, 16, 16, 15, 14, 14, 13, 12, 12, 11, 10, 10, 9,
    9, 8, 7, 7, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0,
    127, 125, 123, 121, 119, 118, 116, 114, 113, 111, 109, 108, 106, 105, 103, 102,
    100, 99, 97, 96, 95, 93, 92, 91, 90, 88, 87, 86, 85, 84, 83, 82,
    80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 70, 69, 68, 67, 66,
    65, 64, 63, 63, 62, 61, 60, 59, 59, 58, 57, 56, 56, 55, 54, 53};
  localparam logic [6:0] REC_T [128] = '{
    127, 125, 123, 121, 119, 117, 116, 114, 112, 110, 109, 107, 105, 104, 102, 100,
    99, 97, 96, 94, 93, 91, 90, 88, 87, 85, 84, 83, 81, 80, 79, 77,
    76, 75, 74, 72, 71, 70, 69, 68, 66, 65, 64, 63, 62, 61, 60, 59,
    58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43,
    42, 41, 40, 40, 39, 38, 37, 36, 35, 35, 34, 33, 32, 31, 31, 30,
    29, 28, 28, 27, 26, 25, 25, 24, 23, 23, 22, 21, 21, 20, 19, 19,
    18, 17, 17, 16, 15, 15, 14, 14, 13, 12, 12, 11, 11, 10, 9, 9,
    8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0};
  function automatic logic [68:0] fest(input logic [63:0] v, input int ne, input int ns, input logic rsq,
                                       input logic [2:0] rm);
    logic        sg, isnan, snan, isinf, iszero, issub;
    longint      ex, oe;
    logic [63:0] sig, emax, smask, r, osig;
    logic [4:0]  fl;                             // {NV, DZ, OF, UF, NX}
    int          bias;
    smask = (64'd1 << ns) - 1;
    emax  = (64'd1 << ne) - 1;
    sg    = v[ne + ns];
    ex    = longint'((v >> ns) & emax);
    sig   = v & smask;
    bias  = (1 << (ne - 1)) - 1;
    isnan = (ex == longint'(emax)) && (sig != '0);
    snan  = isnan && !sig[ns - 1];
    isinf = (ex == longint'(emax)) && (sig == '0);
    iszero = (ex == 0) && (sig == '0);
    issub = (ex == 0) && (sig != '0);
    fl = '0;
    r  = '0;
    if (issub) begin                             // 规格化：数前导 0，指数随之减
      for (int k = 0; k < 64; k++)
        if (k < ns && !sig[ns - 1]) begin ex = ex - 1; sig = sig << 1; end
      sig = (sig << 1) & smask;
    end
    if (rsq) begin
      if (isnan || (sg && !iszero)) begin
        r = (emax << ns) | (64'd1 << (ns - 1));
        fl[4] = !(isnan && !snan);
      end else if (iszero) begin
        r = (64'(sg) << (ne + ns)) | (emax << ns); fl[3] = 1'b1;
      end else if (isinf) r = '0;
      else begin
        int idx;
        idx  = (int'(ex & 1) << 6) | int'(sig >> (ns - 6));
        osig = 64'(RSQ_T[idx]) << (ns - 7);
        oe   = (3 * longint'(bias) - 1 - ex) / 2;
        r    = (64'(oe) << ns) | osig;
      end
    end else begin
      if (isinf) r = 64'(sg) << (ne + ns);
      else if (iszero) begin r = (64'(sg) << (ne + ns)) | (emax << ns); fl[3] = 1'b1; end
      else if (isnan) begin r = (emax << ns) | (64'd1 << (ns - 1)); fl[4] = snan; end
      else if (issub && ex != 0 && ex != -1) begin                // 太小：倒数溢出
        fl[2] = 1'b1; fl[0] = 1'b1;
        if (rm == 3'd1 || (rm == 3'd2 && !sg) || (rm == 3'd3 && sg)) r = ((64'(sg) << (ne + ns)) | (emax << ns)) - 1;
        else r = (64'(sg) << (ne + ns)) | (emax << ns);
      end else begin
        int idx;
        idx  = int'(sig >> (ns - 7));
        osig = 64'(REC_T[idx]) << (ns - 7);
        oe   = 2 * longint'(bias) - 1 - ex;
        if (oe == 0 || oe == -1) begin
          osig = (osig >> 1) | (64'd1 << (ns - 1));
          if (oe == -1) begin osig = osig >> 1; oe = 0; end
        end
        r = (64'(sg) << (ne + ns)) | ((64'(oe) & emax) << ns) | osig;
      end
    end
    return {fl, r};
  endfunction
  function automatic logic [68:0] festw(input logic [63:0] v, input logic [1:0] w, input logic rsq, input logic [2:0] rm);
    unique case (w)
      2'd1:    return fest(v, 5, 10, rsq, rm);
      2'd2:    return fest(v, 8, 23, rsq, rm);
      default: return fest(v, 11, 52, rsq, rm);
    endcase
  endfunction

  // OFP8 -> BF16 (精确；E4M3 的 S.1111.111 与 E5M2 的 NaN 都成默认 NaN，不报标志)
  function automatic logic [15:0] o8tob(input logic [7:0] v, input logic e5);
    logic [4:0] e;
    logic [2:0] m;
    int pm;
    if (!e5) begin                                           // E4M3：偏置 7
      e = {1'b0, v[6:3]}; m = v[2:0];
      if (e == 5'd15 && m == 3'd7) return 16'h7FC0;
      if (e == '0) begin
        if (m == '0) return {v[7], 15'b0};
        pm = 0;
        for (int k = 0; k < 3; k++) if (m[k]) pm = k;        // 值 = m * 2^-9
        return {v[7], 8'(pm + 118), 7'((8'(m) << (7 - pm)) & 8'h7F)};
      end
      return {v[7], 8'(int'(e) + 120), m, 4'b0};
    end else begin                                           // E5M2：偏置 15
      e = v[6:2]; m = {1'b0, v[1:0]};
      if (e == 5'h1F) return (m != '0) ? 16'h7FC0 : {v[7], 8'hFF, 7'b0};
      if (e == '0) begin
        if (m == '0) return {v[7], 15'b0};
        pm = m[1] ? 1 : 0;                                   // 值 = m * 2^-16
        return {v[7], 8'(pm + 111), 7'((8'(m) << (7 - pm)) & 8'h7F)};
      end
      return {v[7], 8'(int'(e) + 112), m[1:0], 5'b0};
    end
  endfunction
  // FP32 -> OFP8 (softfloat f32_to_e4m3 / f32_to_e5m2 + roundPack，逐行照做)；返回 {fflags, 结果}
  function automatic logic [12:0] f2o8(input logic [31:0] a, input logic e5, input logic sat, input logic [2:0] rm);
    logic        sg, rne;
    logic [7:0]  ex;
    logic [22:0] fr;
    logic [4:0]  fl;
    int          e, sig, inc, rb;
    sg = a[31]; ex = a[30:23]; fr = a[22:0];
    fl = '0;
    if (ex == 8'hFF) begin
      if (fr != '0) return {!fr[22], 4'b0, 8'h7F};          // NaN (sNaN 报 NV)
      if (e5) return {5'b0, sg, sat ? 7'h7B : 7'h7C};
      return {5'b0, sat ? {sg, 7'h7E} : 8'h7F};
    end
    rne = (rm == 3'd0);
    inc = 8;
    if (!rne && rm != 3'd4) inc = (rm == (sg ? 3'd2 : 3'd3)) ? 15 : 0;
    if (!e5) begin
      sig = int'(fr[22:16]) | int'(fr[15:0] != '0);
      if (ex == '0 && sig == 0) return {5'b0, sg, 7'b0};
      sig = sig | 8'h80; e = int'(ex) - 121;
    end else begin
      sig = int'(fr[22:17]) | int'(fr[16:0] != '0);
      if (ex == '0 && sig == 0) return {5'b0, sg, 7'b0};
      sig = sig | 8'h40; e = int'(ex) - 113;
    end
    rb = sig & 15;
    if (e < 0) begin
      logic tiny;
      tiny = (e < -1) || (sig + inc < (e5 ? 128 : 256));
      sig  = (-e >= 31) ? int'(sig != 0) : ((sig >> (-e)) | int'((sig & ((1 << (-e)) - 1)) != 0));
      e    = 0;
      rb   = sig & 15;
      if (tiny && rb != 0) fl[1] = 1'b1;
    end else if (!e5 && (e > 14 || (e == 14 && sig + inc >= 240 + int'(rne)))) begin
      return {5'b00101, (sat || inc == 0) ? {sg, 7'h7E} : 8'h7F};
    end else if (e5 && (e > 29 || (e == 29 && sig + inc >= 128))) begin
      return {5'b00101, sg, sat ? 7'h7B : 7'(7'h7C - 7'(inc == 0))};
    end
    sig = (sig + inc) >> 4;
    if (rb != 0) fl[0] = 1'b1;
    if (rb == 8 && rne) sig = sig & ~1;
    if (sig == 0) e = 0;
    return {fl, 8'((sg ? 128 : 0) + (e << (e5 ? 2 : 3)) + sig)};
  endfunction

  //--------------------------------------------------------------------------
  // 通道调度
  //--------------------------------------------------------------------------
  wire             x_red = (xd.k == K_FRED);
  // 本地组合计算 (不进 fpnew)：vfrec7 / vfrsqrt7、OFP8 转换 (SEW = 8 的浮点 <-> 浮点转换都是)
  wire             x_o8  = (xd.op == F_CVT) && (xd.cvk == 2'd0) && (xsew == 2'd0);
  wire             x_loc = (xd.op == F_REC7) || (xd.op == F_RSQ7) || x_o8;
  wire  [EW-1:0]   f_ne  = EW'(1 << le);                 // 每拍元素数
  logic [KW-1:0]   f_k   [NFL];                          // 发射拍里各通道已处理 (发出或跳过) 的元素数
  logic [EW-1:0]   f_onb [2];                            // 各缓冲在飞的元素数
  wire  [EW-1:0]   f_on  = f_onb[0] + f_onb[1];
  logic [1:0]      f_idn;                                // 缓冲对应的那一拍已全部发出 / 跳过
  // 发射拍的操作数切片与活动元素 (写回拍的在 s_vs2 / c_act)：寄存器 (P4d)。开始后那一拍装第 0 拍的
  //   (这一拍不发射)，一拍发完 (i_bend) 时装下一拍的 —— 原先当拍按 xbi 读寄存器堆切片，接着取元素 / 升格式 /
  //   装箱进 fpnew 或算本地结果写缓冲 (QUICK 剖面浮点组 304 级)。同一条指令后面的拍读的区域不会是前面的拍
  //   写过的，提前读没有冒险
  logic [DLEN-1:0] si_vs2, si_vs1, si_vd;
  logic [NEB-1:0]  i_act;
  logic            i_bend;
  always_ff @(posedge clk_i)
    if (x_first || i_bend) begin
      logic [VLW-1:0] e0n, bn;
      logic [NEB-1:0] mk;
      bn  = x_first ? '0 : xbi + 1'b1;
      e0n = VLW'(int'(bn) << le);
      si_vs2 <= vslice(xd.vs2, xs2w, int'(e0n));
      si_vs1 <= vslice(xd.vs1, xsew, int'(e0n));
      si_vd  <= vslice(xd.vd,  xdw,  int'(e0n));
      mk = NEB'(vrf[0] >> e0n);
      for (int j = 0; j < NEB; j++)
        i_act[j] <= (j < (1 << le)) && (int'(e0n) + j < int'(xvl)) && (xd.vm || xd.allb || mk[j]);
    end
  // 可以发射：还有拍、没超前写回拍两拍 (缓冲)、切片已装好 (开始后那一拍不发)；归约不超前 (逐个串行，结果回来再发下一个)
  wire             i_ok   = x_busy && x_fp && !x_first && (xbi < xnb) && (xbi - xb < VLW'(2)) && !(x_red && xbi != xb);
  logic [63:0]     f_acc;                                // 归约累加器 (目的宽度)
  logic            f_any;                                // 归约：有过活动元素

  // 各通道本拍的元素号、是否完成
  logic [EW-1:0]   fl_j   [NFL];
  logic [NFL-1:0]  fl_end, fl_act, fl_go;               // 完成 / 当前元素活动 / 要发 (fpnew)
  always_comb begin
    for (int l = 0; l < NFL; l++) begin
      fl_j[l]   = x_red ? EW'(f_k[0]) : EW'(l + NFL * int'(f_k[l]));
      fl_end[l] = x_red ? (l != 0 || f_k[0] >= f_ne) : (fl_j[l] >= f_ne);
      fl_act[l] = !fl_end[l] && i_act[fl_j[l] % NEB];
      fl_go[l]  = i_ok && fl_act[l] && !x_loc && !(x_red && f_on != '0);
    end
  end

  // 各通道的请求
  typedef struct packed {
    fpnew_pkg::operation_e  op;
    logic                   mod;
    fpnew_pkg::roundmode_e  rnd;
    fpnew_pkg::fp_format_e  src_fmt;
    fpnew_pkg::fp_format_e  dst_fmt;
    fpnew_pkg::int_format_e int_fmt;
    logic [2:0][63:0]       opnd;
  } vfreq_t;
  vfreq_t          fq    [NFL];
  logic [NFL-1:0]  fq_nv;                                // 升格式的 NV (发出时报)
  logic [68:0]     fq_est [NFL];                         // vfrec7 / vfrsqrt7 的本地结果
  always_comb begin
    for (int l = 0; l < NFL; l++) begin
      logic [63:0] r2, r1, rd, a, b, c;
      logic [64:0] wa, wb;
      logic [1:0]  ow;                                   // 运算宽度
      int j;
      fpnew_pkg::roundmode_e rm;
      j  = int'(fl_j[l]);
      r2 = eget(si_vs2, j, xs2w);
      r1 = eget(si_vs1, j, xsew);
      rd = eget(si_vd,  j, xdw);
      if (xd.src != 2'd0) r1 = xa;                       // .vf：已拆盒的 f[rs1] (SEW 宽)
      rm = fpnew_pkg::roundmode_e'(xrm);
      fq_nv[l] = 1'b0;
      // 操作数：a = vs2，b = vs1 / f[rs1]，c = vd；加宽类把窄源升到 2*SEW
      a = r2; b = r1; c = rd; ow = xcw;
      if (x_red) begin
        ow = xdw;
        a = f_acc;
        if (xd.wcls != 2'd0) begin wa = fwiden(r2, xsew, 1'b0); b = wa[63:0]; fq_nv[l] = wa[64]; end
        else b = r2;
      end else if (xd.op != F_CVT && xd.wcls != 2'd0) begin
        wb = fwiden(r1, xsew, xd.bf); b = wb[63:0];
        wa = fwiden(r2, xsew, xd.bf);
        if (xd.wcls == 2'd1) begin a = wa[63:0]; fq_nv[l] = wa[64] | wb[64]; end
        else fq_nv[l] = wb[64];
      end
      a = fbox(a, ow); b = fbox(b, ow); c = fbox(c, ow);
      fq[l].op = fpnew_pkg::ADD; fq[l].mod = 1'b0; fq[l].rnd = rm;
      fq[l].src_fmt = ffmt(ow, 1'b0); fq[l].dst_fmt = ffmt(ow, 1'b0); fq[l].int_fmt = fpnew_pkg::INT64;
      fq[l].opnd = {b, a, 64'd0};
      unique case (xd.op)
        F_ADD:   ;
        F_SUB:   fq[l].mod = 1'b1;
        F_RSUB:  begin fq[l].mod = 1'b1; fq[l].opnd = {a, b, 64'd0}; end
        F_MUL:   begin fq[l].op = fpnew_pkg::MUL; fq[l].opnd = {64'd0, b, a}; end
        F_DIV:   begin fq[l].op = fpnew_pkg::DIV; fq[l].opnd = {64'd0, b, a}; end
        F_RDIV:  begin fq[l].op = fpnew_pkg::DIV; fq[l].opnd = {64'd0, a, b}; end
        F_SQRT:  begin fq[l].op = fpnew_pkg::SQRT; fq[l].opnd = {64'd0, 64'd0, a}; end
        F_MIN, F_RMIN: begin fq[l].op = fpnew_pkg::MINMAX; fq[l].rnd = fpnew_pkg::RNE; fq[l].opnd = {64'd0, b, a}; end
        F_MAX, F_RMAX: begin fq[l].op = fpnew_pkg::MINMAX; fq[l].rnd = fpnew_pkg::RTZ; fq[l].opnd = {64'd0, b, a}; end
        // vd = ±(vs1 * vs2) ± vd
        F_MACC:  begin fq[l].op = fpnew_pkg::FMADD;  fq[l].opnd = {c, a, b}; end
        F_NMACC: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].mod = 1'b1; fq[l].opnd = {c, a, b}; end
        F_MSAC:  begin fq[l].op = fpnew_pkg::FMADD;  fq[l].mod = 1'b1; fq[l].opnd = {c, a, b}; end
        F_NMSAC: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].opnd = {c, a, b}; end
        // vd = ±(vd * vs1) ± vs2
        F_MADD:  begin fq[l].op = fpnew_pkg::FMADD;  fq[l].opnd = {a, b, c}; end
        F_NMADD: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].mod = 1'b1; fq[l].opnd = {a, b, c}; end
        F_MSUB:  begin fq[l].op = fpnew_pkg::FMADD;  fq[l].mod = 1'b1; fq[l].opnd = {a, b, c}; end
        F_NMSUB: begin fq[l].op = fpnew_pkg::FNMSUB; fq[l].opnd = {a, b, c}; end
        // 比较 (CMP：RNE = LE，RTZ = LT，RDN = EQ)
        F_CEQ, F_CNE: begin fq[l].op = fpnew_pkg::CMP; fq[l].rnd = fpnew_pkg::RDN; fq[l].opnd = {64'd0, b, a}; end
        F_CLT:   begin fq[l].op = fpnew_pkg::CMP; fq[l].rnd = fpnew_pkg::RTZ; fq[l].opnd = {64'd0, b, a}; end
        F_CLE:   begin fq[l].op = fpnew_pkg::CMP; fq[l].rnd = fpnew_pkg::RNE; fq[l].opnd = {64'd0, b, a}; end
        F_CGT:   begin fq[l].op = fpnew_pkg::CMP; fq[l].rnd = fpnew_pkg::RTZ; fq[l].opnd = {64'd0, a, b}; end
        F_CGE:   begin fq[l].op = fpnew_pkg::CMP; fq[l].rnd = fpnew_pkg::RNE; fq[l].opnd = {64'd0, a, b}; end
        F_CLASS: begin fq[l].op = fpnew_pkg::CLASSIFY; fq[l].opnd = {64'd0, 64'd0, a}; end
        F_CVT: begin
          fq[l].opnd = {64'd0, 64'd0, r2};
          unique case (xd.cvk)
            2'd0: begin                                  // 浮点 -> 浮点
              fq[l].op = fpnew_pkg::F2F;
              fq[l].src_fmt = ffmt(xs2w, xd.bf && xd.wcls == 2'd1);
              fq[l].dst_fmt = ffmt(xdw,  xd.bf && xd.wcls == 2'd3);
              fq[l].opnd = {64'd0, 64'd0, fbox(r2, xs2w)};
              if (xd.rod) fq[l].rnd = fpnew_pkg::ROD;
            end
            2'd1: begin                                  // 整数 -> 浮点
              fq[l].op = fpnew_pkg::I2F; fq[l].mod = xd.cvu;
              fq[l].int_fmt = ifmt(xs2w);
              fq[l].src_fmt = ffmt(xdw, 1'b0); fq[l].dst_fmt = ffmt(xdw, 1'b0);
            end
            default: begin                               // 浮点 -> 整数
              fq[l].op = fpnew_pkg::F2I; fq[l].mod = xd.cvu;
              fq[l].int_fmt = ifmt(xdw);
              fq[l].src_fmt = ffmt(xs2w, 1'b0); fq[l].dst_fmt = ffmt(xs2w, 1'b0);
              fq[l].opnd = {64'd0, 64'd0, fbox(r2, xs2w)};
              if (xd.rtz) fq[l].rnd = fpnew_pkg::RTZ;
            end
          endcase
        end
        F_RUSUM, F_ROSUM: ;                              // acc + vs2[i] (a = acc，b = 元素)
        default: ;
      endcase
      fq_est[l] = festw(r2, xsew, xd.op == F_RSQ7, xrm);
      if (x_o8) begin
        if (xd.wcls == 2'd1) fq_est[l] = {5'b0, 48'b0, o8tob(r2[7:0], altf)};      // OFP8 -> BF16
        else begin                                                                  // BF16 / FP32 -> OFP8
          logic [12:0] t;
          t = f2o8(xd.q ? r2[31:0] : {r2[15:0], 16'b0}, altf, xd.sat, xrm);
          fq_est[l] = {t[12:8], 56'b0, t[7:0]};
        end
      end
    end
  end

  // fpnew 通道
  localparam fpnew_pkg::fpu_features_t VFEAT = '{
    Width:         64,
    EnableVectors: 1'b0,
    EnableNanBox:  1'b1,
    // FP8 只为转换单元打开：fpnew 只在有同宽浮点格式时才启用该宽度的整数格式 (INT8 要 FP8，
    // 否则 FP16 <-> INT8 的转换结果被静音成 0)；运算单元里 FP8 关掉
    FpFmtMask:     5'b11111,                             // FP32 FP64 FP16 FP8 FP16ALT(BF16)
    IntFmtMask:    4'b1111
  };
  // 每通道两个 fpnew：主实例 (ADDMUL / NONCOMP / CONV，全流水) + UD 实例 (DIVSQRT，可变延迟)。
  //   UD 拆出来是为了跑慢时钟：C910 radix-16 SRT 一次迭代 3.1 ns (2 ns 下 -1.1，单独综合)，O6_UD_HALF 时
  //   UD 跑 ICG 门控出的 1/2 速时钟 (同标量 FPU 的 UD，SDC 里 *u_ud_icg/u_cg/Q 的生成时钟 -edges {1 2 5})
  localparam fpnew_pkg::fpu_implementation_t VIMPL = '{
    // 级数按 2 ns 单独综合定 (syn/blk_fpw.sv)：ADDMUL 3 级 + fpnew_fma_multi 内显式 3 级 (共 6 拍)，-0.06 ns；
    //   NONCOMP 1 级时到输出 -0.11 ns，加到 2；CONV 3 级
    PipeRegs:   '{'{3, 3, 3, 3, 3},                      // ADDMUL
                  '{0, 0, 0, 0, 0},                      // DIVSQRT (在 UD 实例)
                  '{2, 2, 2, 2, 2},                      // NONCOMP
                  '{3, 3, 3, 3, 3}},                     // CONV
    UnitTypes:  '{'{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::DISABLED, fpnew_pkg::MERGED},
                  '{default: fpnew_pkg::DISABLED},
                  '{fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::DISABLED,
                    fpnew_pkg::PARALLEL},
                  '{default: fpnew_pkg::MERGED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  localparam fpnew_pkg::fpu_implementation_t VIMPL_UD = '{
    PipeRegs:   '{'{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}, '{0, 0, 0, 0, 0}},
    UnitTypes:  '{'{default: fpnew_pkg::DISABLED},
                  '{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::DISABLED, fpnew_pkg::MERGED},
                  '{default: fpnew_pkg::DISABLED},
                  '{default: fpnew_pkg::DISABLED}},
    PipeConfig: fpnew_pkg::DISTRIBUTED
  };
  // UD 时钟：ud_en = 1 表示本拍末是 UD 的时钟沿
  logic ud_en, clk_ud;
`ifdef O6_UD_HALF
  logic ud_ph;
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) ud_ph <= 1'b0;
    else         ud_ph <= !ud_ph;
  assign ud_en = ud_ph;
  o6_icg u_ud_icg (.clk_i, .en_i(ud_en), .te_i(1'b0), .clk_o(clk_ud));
`else
  assign ud_en  = 1'b1;
  assign clk_ud = clk_i;
`endif
  logic [NFL-1:0]  fo_v, fi_rdy;
  logic [63:0]     fo_r   [NFL];
  fpnew_pkg::status_t fo_st [NFL];
  logic [EW:0]     fo_tag [NFL];                       // {拍号奇偶, 元素号}
  for (genvar l = 0; l < NFL; l++) begin : g_fl
`ifdef O6_VU_FPBB
    // 时序摸底 (综合 VU_FPBB=1)：fpnew 换成寄存器桩，只看 o6_vu 自身的路径
    logic [4:0] st;
    o6_vu_fpbb #(.TW(EW + 1), .CW($bits(vfreq_t) - 192)) u_fpnew (
      .clk_i, .opnd_i(fq[l].opnd), .ctl_i(fq[l][$bits(vfreq_t)-1:192]),
      .tag_i({xbi[0], fl_j[l]}), .v_i(fl_go[l]),
      .r_o(fo_r[l]), .st_o(st), .tag_o(fo_tag[l]), .v_o(fo_v[l])
    );
    assign fi_rdy[l] = 1'b1;
    assign fo_st[l]  = fpnew_pkg::status_t'(st);
`else
    wire is_ud = (fq[l].op == fpnew_pkg::DIV) || (fq[l].op == fpnew_pkg::SQRT);
    logic               m_rdy, m_v, d_rdy, d_v;
    logic [63:0]        m_r, d_r;
    fpnew_pkg::status_t m_st, d_st;
    logic [EW:0]        m_tag, d_tag;
    fpnew_top #(
      .Features       (VFEAT),
      .Implementation (VIMPL),
      .DivSqrtSel     (fpnew_pkg::THMULTI),
      .TagType        (logic [EW:0])
    ) u_fpnew (
      .clk_i,
      .rst_ni,
      .operands_i    (fq[l].opnd),
      .rnd_mode_i    (fq[l].rnd),
      .op_i          (fq[l].op),
      .op_mod_i      (fq[l].mod),
      .src_fmt_i     (fq[l].src_fmt),
      .dst_fmt_i     (fq[l].dst_fmt),
      .int_fmt_i     (fq[l].int_fmt),
      .vectorial_op_i(1'b0),
      .tag_i         ({xbi[0], fl_j[l]}),
      .simd_mask_i   (1'b1),
      .in_valid_i    (fl_go[l] && !is_ud),
      .in_ready_o    (m_rdy),
      .flush_i       (1'b0),
      .result_o      (m_r),
      .status_o      (m_st),
      .tag_o         (m_tag),
      .out_valid_o   (m_v),
      .out_ready_i   (1'b1),
      .busy_o        (),
      .early_valid_o ()
    );
    // UD 请求保持：发出那拍记下，UD 时钟沿收下后清掉；结果只在 UD 时钟沿前一拍取走 (UD 同沿出队)
    logic        udh_v;
    vfreq_t      udh_rq;
    logic [EW:0] udh_tag;
    always_ff @(posedge clk_i or negedge rst_ni)
      if (!rst_ni) udh_v <= 1'b0;
      else if (fl_go[l] && is_ud && !udh_v) udh_v <= 1'b1;
      else if (udh_v && ud_en && d_rdy) udh_v <= 1'b0;
    always_ff @(posedge clk_i)
      if (fl_go[l] && is_ud && !udh_v) begin udh_rq <= fq[l]; udh_tag <= {xbi[0], fl_j[l]}; end
    fpnew_top #(
      .Features       (VFEAT),
      .Implementation (VIMPL_UD),
      .DivSqrtSel     (fpnew_pkg::THMULTI),
      .TagType        (logic [EW:0])
    ) u_ud (
      .clk_i         (clk_ud),
      .rst_ni,
      .operands_i    (udh_rq.opnd),
      .rnd_mode_i    (udh_rq.rnd),
      .op_i          (udh_rq.op),
      .op_mod_i      (udh_rq.mod),
      .src_fmt_i     (udh_rq.src_fmt),
      .dst_fmt_i     (udh_rq.dst_fmt),
      .int_fmt_i     (udh_rq.int_fmt),
      .vectorial_op_i(1'b0),
      .tag_i         (udh_tag),
      .simd_mask_i   (1'b1),
      .in_valid_i    (udh_v),
      .in_ready_o    (d_rdy),
      .flush_i       (1'b0),
      .result_o      (d_r),
      .status_o      (d_st),
      .tag_o         (d_tag),
      .out_valid_o   (d_v),
      .out_ready_i   (ud_en),
      .busy_o        (),
      .early_valid_o ()
    );
    // 一条指令只用一个实例 (除 / 开方整条都走 UD)，两边结果不会同拍
    wire d_fire = d_v && ud_en;
    assign fi_rdy[l] = is_ud ? !udh_v : m_rdy;
    assign fo_v[l]   = m_v || d_fire;
    assign fo_r[l]   = d_fire ? d_r   : m_r;
    assign fo_st[l]  = d_fire ? d_st  : m_st;
    assign fo_tag[l] = d_fire ? d_tag : m_tag;
`ifndef SYNTHESIS
    always_ff @(posedge clk_i)
      if (rst_ni) assert (!(m_v && d_fire)) else $error("vu: fpnew 主实例与 UD 同拍出结果");
`endif
`endif
  end

  // 发出 / 跳过 / 本地完成
  logic [NFL-1:0] fl_adv, fl_fire, fl_loc;
  always_comb begin
    for (int l = 0; l < NFL; l++) begin
      fl_fire[l] = fl_go[l] && fi_rdy[l];
      fl_loc[l]  = i_ok && x_loc && fl_act[l];
      // 前进：发出了、本地算完、或者当前元素不活动 (归约在飞时不动)
      fl_adv[l]  = i_ok && !fl_end[l] && !(x_red && f_on != '0) &&
                   (fl_fire[l] || fl_loc[l] || !fl_act[l]);
    end
  end
  assign i_bend = i_ok && (&fl_end);                     // 发射拍全部发出 / 跳过
  // 写回拍完成：它已全部发出，且它那个缓冲的结果都回来了
  assign f_bdone = x_fp && f_idn[xb[0]] && (f_onb[xb[0]] == '0);

  // 归约的最后结果：Spike 的 vfredusum / vfwredusum 在没有活动元素时，vs1[0] 若是 NaN 改成默认 NaN (sNaN 报 NV)
  logic [63:0] f_fin;
  logic        f_fnv;
  always_comb begin
    logic [1:0] nn;
    nn = fnan(f_acc, xdw);
    f_fin = f_acc; f_fnv = 1'b0;
    if (xd.op == F_RUSUM && !f_any && nn[1]) begin f_fin = fdnan(xdw); f_fnv = nn[0]; end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      f_onb[0] <= '0; f_onb[1] <= '0; f_idn <= '0; xbi <= '0;
      for (int l = 0; l < NFL; l++) f_k[l] <= '0;
    end else begin
      logic [EW-1:0] n [2];
      n[0] = f_onb[0]; n[1] = f_onb[1];
      for (int l = 0; l < NFL; l++) begin
        if (fl_fire[l]) n[xbi[0]] = n[xbi[0]] + 1'b1;
        if (fo_v[l])    n[fo_tag[l][EW]] = n[fo_tag[l][EW]] - 1'b1;
      end
      f_onb[0] <= n[0]; f_onb[1] <= n[1];
      if (x_start) begin
        xbi <= '0; f_idn <= '0;
        for (int l = 0; l < NFL; l++) f_k[l] <= '0;
      end else begin
        if (x_busy && x_fp && x_bdone) f_idn[xb[0]] <= 1'b0;
        if (i_bend) begin
          f_idn[xbi[0]] <= 1'b1;
          xbi <= xbi + 1'b1;
          for (int l = 0; l < NFL; l++) f_k[l] <= '0;
        end else
          for (int l = 0; l < NFL; l++) if (fl_adv[l]) f_k[l] <= f_k[l] + 1'b1;
      end
    end
  end
  always_ff @(posedge clk_i) begin
    if (x_start) f_any <= 1'b0;
    if (x_first) f_acc <= rd_init[63:0];
    begin
      logic [DLEN-1:0] nr [2];
      nr[0] = f_res2[0]; nr[1] = f_res2[1];
      for (int l = 0; l < NFL; l++) begin
        if (x_red && fl_fire[l]) f_any <= 1'b1;
        if (fo_v[l]) begin
          int j;
          logic bp;
          j  = int'(fo_tag[l][EW-1:0]);
          bp = fo_tag[l][EW];
          if (x_red) f_acc <= lowb(fo_r[l], xdw);
          else if (xd.k == K_FEWM) f_cmp2[bp][j % NEB] <= (xd.op == F_CNE) ? !fo_r[l][0] : fo_r[l][0];
          else nr[bp] = (nr[bp] & ~eput('1, j, xdw)) | eput(fo_r[l], j, xdw);
        end
        if (fl_loc[l]) begin
          int j;
          j = int'(fl_j[l]);
          nr[xbi[0]] = (nr[xbi[0]] & ~eput('1, j, xdw)) | eput(fq_est[l][63:0], j, xdw);
        end
      end
      f_res2[0] <= nr[0]; f_res2[1] <= nr[1];
    end
  end

  // 异常标志：结果出来时 (fpnew 状态)、发出时 (升格式的 NV)、本地估算、归约收尾
  always_comb begin
    fflags_o = '0;
    for (int l = 0; l < NFL; l++) begin
      if (fo_v[l])    fflags_o |= fo_st[l];
      if (fl_fire[l]) fflags_o[4] |= fq_nv[l];
      if (fl_loc[l])  fflags_o |= fq_est[l][68:64];
    end
    if (x_busy && x_bdone && x_red && xb + 1'b1 >= xnb) fflags_o[4] |= f_fnv;
    if (df_go && df_done && x_dfp) fflags_o |= dt_ff;
  end

  //--------------------------------------------------------------------------
  // 掩码逻辑 / vmsbf / vmsif / vmsof：整寄存器一拍 (位 < vl)。S1 算好寄存进 S3 (s3_mres / s3_mwe) 再写
  //   (原先 S3 那拍按寄存器堆现读现算直接写回，且第一个 1 用 1024 位的 x & (~x + 1) / first - 1 两条进位链；
  //    QUICK 剖面里寄存器堆写这一组 1052 级)。现在用前缀或 (Kogge-Stone，10 级)：
  //    pre[i] = x[0..i] 有 1，pex[i] = x[0..i-1] 有 1；vmsbf = ~pre，vmsif = ~pex，vmsof = pre & ~pex
  //--------------------------------------------------------------------------
  logic [VLEN-1:0] c_mres, c_mwe;           // 结果位、写使能 (S1)
  always_comb begin
    logic [VLEN-1:0] a, b, body, act, x, pre, pex;
    a = vrf[xd.vs2]; b = vrf[xd.vs1];
    for (int i = 0; i < VLEN; i++) begin
      body[i] = (i < int'(vl));
      act[i]  = body[i] && (xd.vm || vrf[0][i]);
    end
    x   = a & act;                        // 参与的 1
    pre = x;
    for (int d = 1; d < VLEN; d = d * 2)
      for (int i = VLEN - 1; i >= d; i--) pre[i] = pre[i] | pre[i - d];   // 倒序：读到的 pre[i-d] 还是上一级的
    pex = {pre[VLEN-2:0], 1'b0};
    if (xd.k == K_MLOG) begin
      c_mwe = body;
      unique case (xd.op)
        M_AND:  c_mres = a & b;
        M_NAND: c_mres = ~(a & b);
        M_ANDN: c_mres = a & ~b;
        M_XOR:  c_mres = a ^ b;
        M_OR:   c_mres = a | b;
        M_NOR:  c_mres = ~(a | b);
        M_ORN:  c_mres = a | ~b;
        default: c_mres = ~(a ^ b);
      endcase
    end else begin                                // vmsbf / vmsif / vmsof
      c_mwe = act;
      unique case (xd.op)
        S_SBF:   c_mres = ~pre;
        S_SIF:   c_mres = ~pex;
        default: c_mres = pre & ~pex;
      endcase
    end
  end

  //--------------------------------------------------------------------------
  // 置换 (逐元素，元素号 i = xb)：slide / slide1 / gather / compress
  //--------------------------------------------------------------------------
  // 寄存器组 base 里宽度 w 的第 idx 个元素 (先选寄存器，再定宽选元素)
  function automatic logic [63:0] velem(input logic [4:0] base, input logic [1:0] w, input logic [63:0] idx);
    int bo, ei;
    logic [VLEN-1:0] rg;
    bo = int'(idx[12:0]) << (int'(w) + 3);
    rg = vrf[base + 5'(bo >> LVLEN)];
    ei = int'(idx[12:0]);
    unique case (w)
      2'd0:    return 64'(rg[8  * (ei % (VLEN / 8))  +: 8]);
      2'd1:    return 64'(rg[16 * (ei % (VLEN / 16)) +: 16]);
      2'd2:    return 64'(rg[32 * (ei % (VLEN / 32)) +: 32]);
      default: return rg[64 * (ei % (VLEN / 64)) +: 64];
    endcase
  endfunction
  logic [VLW-1:0] pm_pos;                   // vcompress 的写位置
  logic           p_we;                     // 本元素写
  logic [63:0]    p_idx;                    // 写入的元素号
  logic [63:0]    p_val;
  // 两个读口：vs1 (gather 的下标) 与 vs2 (源元素)；每种置换只是源下标与"是否有源"不同
  always_comb begin
    logic [63:0] i, off, gi, si;
    logic        act, sv, sx;             // 源下标有效 / 用标量
    i   = 64'(xb);
    off = (xd.src == 2'd1) ? xa : 64'(xi[19:15]);
    act = xd.vm || vrf[0][xb % VLEN];
    gi  = (xd.src == 2'd0) ? velem(xd.vs1, (xd.op == P_GATH16) ? 2'd1 : xsew, i) : off;
    p_we = act; p_idx = i; sv = 1'b1; sx = 1'b0;
    unique case (xd.op)
      P_UP:    begin si = i - off; p_we = act && (i >= off); end
      P_DOWN:  begin si = i + off; sv = (off < 64'(vlmax)) && (i < 64'(vlmax) - off); end
      P_1UP:   begin si = i - 1;   sx = (i == 0); end
      P_1DOWN: begin si = i + 1;   sx = (i == 64'(vl) - 1); end
      P_GATH, P_GATH16: begin si = gi; sv = (gi < 64'(vlmax)); end
      default: begin                              // vcompress：vs1 的掩码位选中的元素依次写到 pos
        si = i;
        p_we = vrf[xd.vs1][xb % VLEN];
        p_idx = 64'(pm_pos);
      end
    endcase
    p_val = sx ? sext_w(xa, xsew, 1'b0) : sv ? velem(xd.vs2, xsew, si) : '0;
  end
  always_ff @(posedge clk_i)
    if (x_start) pm_pos <= '0;
    else if (x_busy && xd.k == K_PERM && xd.op == P_COMP && p_we) pm_pos <= pm_pos + 1'b1;

  //==========================================================================
  // 访存引擎 (R_MEM)：从 vstart 起按元素顺序访问，每次一整行 (64 B，经 D$ 的整行模式)
  //   合并方式 (单位步长 / 整寄存器 / 掩码 / fault-only-first，且不分段)：一次覆盖该双字里剩下的元素；
  //   逐个方式 (分段 / 跨步 / 索引)：一次一个 (元素, 段)，同一元素的各段依次访问 (与 Spike 同序)。
  //   双字的第 k 字节 <-> 本段寄存器组字节偏移 i*W + (k - a[2:0])，属于元素 i + (k - a[2:0]) / W；
  //   段 f 的寄存器组从 vd + f * (段间隔) 起 (段间隔：单位步长 / 跨步按 EMUL，索引按 LMUL，都至少 1)。
  //   全部被掩掉的不访存；元素地址不对齐报地址不对齐；出错时 vstart = 出错的 (第一个活动) 元素，
  //   之前的元素与本元素之前的段已写；fault-only-first 在第 0 个元素之后出错时不报异常，vl 截到该元素
  //==========================================================================
  wire  [1:0]     m_dw  = (rq.mm == M_IDX) ? sew : rq.eb;                     // 数据元素宽度 (log2 字节)
  wire  [3:0]     m_nfe = (rq.mm == M_WHOLE) ? 4'd1 : 4'(rq.nf) + 4'd1;       // 段数
  wire            m_grp = (rq.mm != M_STRD) && (rq.mm != M_IDX) && (m_nfe == 4'd1);
  logic [VLW-1:0] m_evl;                         // 元素数
  logic [3:0]     m_fs;                          // 段间寄存器间隔
  always_comb begin
    int le, ll;
    ll = lmul_log2(lmul);
    le = int'(rq.eb) - int'(sew) + ll;
    unique case (rq.mm)
      M_WHOLE: m_evl = VLW'(((int'(rq.nf) + 1) * (VLEN / 8)) >> rq.eb);
      M_MASK:  m_evl = VLW'((int'(vl) + 7) >> 3);
      default: m_evl = vl;
    endcase
    if (rq.mm == M_IDX) m_fs = (ll > 0) ? 4'(1 << ll) : 4'd1;
    else                 m_fs = (le > 0) ? 4'(1 << le) : 4'd1;
  end
  // 相邻元素的地址间隔：跨步 = rs2，其余 = 段数 * W (索引方式不用)
  wire  [63:0]    m_es = (rq.mm == M_STRD) ? rb_q : (64'(m_nfe) << m_dw);

  logic [VLW-1:0] m_i;                           // 当前元素
  logic [2:0]     m_f;                           // 当前段
  logic [63:0]    m_a;                           // 当前元素 (第 0 段) 的地址 (索引方式不用)
  wire  [63:0]    m_idx = velem(rq.vs2, rq.eb, 64'(m_i));                      // 索引 (零扩展)
  wire  [63:0]    m_ax  = ((rq.mm == M_IDX) ? ra_q + m_idx : m_a) + (64'(m_f) << m_dw);   // 本次访问的地址
  wire  [3:0]     m_ebytes = 4'd1 << m_dw;
  wire  [5:0]     m_off = m_ax[5:0];                      // 行内偏移
  wire            m_mis = (m_ax & 64'(m_ebytes - 1)) != '0;
  // 本次的元素数：合并方式 = 本行里剩下的元素 (最多 64 个 e8)，逐个方式 / 不对齐 = 1
  logic [6:0]     m_n;
  always_comb begin
    int nn;
    nn = (!m_grp || m_mis) ? 1 : ((64 - int'(m_off)) >> m_dw);
    if (int'(m_i) + nn > int'(m_evl)) nn = int'(m_evl) - int'(m_i);
    m_n = 7'(nn);
  end
  // 本次 = 行里 [off, off + len) 这一段连续字节 <-> 段 m_f 的寄存器组里从 gb = i*W 起的连续字节；
  // 一段最多 64 字节，最多跨两个寄存器 r0 / r1 (r0 内从 p0 起)
  localparam int VLB = VLEN / 8;                          // 每个寄存器的字节数
  wire  [10:0]    m_gb  = 11'(int'(m_i) << m_dw);
  wire  [4:0]     m_r0  = rq.vd + 5'(int'(m_f) * int'(m_fs)) + 5'(int'(m_gb) / VLB);
  wire  [4:0]     m_r1  = m_r0 + 5'd1;
  wire  [6:0]     m_p0  = 7'(int'(m_gb) % VLB);
  wire  [6:0]     m_len = 7'(int'(m_n) << m_dw);
  // 段内第 t 字节是否活动 (t < len 且所属元素没被掩掉)；第一个活动元素及其地址 (出错时的 vstart / tval)
  logic [63:0]    m_tact;
  logic           m_any;
  logic [VLW-1:0] m_fe;
  logic [63:0]    m_fa;
  always_comb begin
    logic [63:0] win;
    win = 64'(vrf[0] >> m_i);                             // v0 里元素 m_i.. 的位
    for (int t = 0; t < 64; t++) begin
      logic ea;
      unique case (m_dw)
        2'd0:    ea = win[t];
        2'd1:    ea = win[t >> 1];
        2'd2:    ea = win[t >> 2];
        default: ea = win[t >> 3];
      endcase
      m_tact[t] = (t < int'(m_len)) && (rq.vm || ea);
    end
    m_any = |m_tact;
    m_fe = m_i; m_fa = m_ax;
    if (m_any) begin                                        // 最低的活动字节 (尾零个数；原先 64 级优先链)
      logic [6:0] tz;
      tz   = clz64(brev64(m_tact));
      m_fe = m_i + VLW'(tz >> m_dw);
      m_fa = m_ax + 64'(tz);
    end
  end
  // store：读 r0 / r1 拼成 128 字节，右旋 p0 字节成段顺序，再放到行内 off 处
  logic [VLEN-1:0] m_z, m_zr;
  always_comb begin
    for (int p = 0; p < VLB; p++) m_z[8*p +: 8] = (p >= int'(m_p0)) ? vrf[m_r0][8*p +: 8] : vrf[m_r1][8*p +: 8];
    m_zr       = (m_z >> (8 * int'(m_p0))) | (m_z << (8 * (VLB - int'(m_p0))));
    vm_wdata_o = 512'(m_zr) << (8 * int'(m_off));
    vm_wmask_o = m_tact << m_off;
  end
  // 在途的行 (最多 2 个，按序回)：load 写回与出错时要用的 {r0, p0, 行内偏移, 字节使能, 第一个活动元素及地址}
  typedef struct packed {
    logic [4:0]     r0;
    logic [6:0]     p0;
    logic [5:0]     off;
    logic [63:0]    tact;
    logic [VLW-1:0] fe;
    logic [63:0]    fa;
  } mq_t;
  mq_t        mq [2];
  logic       mq_h;
  logic [1:0] mq_n;
  mq_t        mqh;
  assign mqh = mq[mq_h];
  // load：回来的行先寄存一拍 (P5：vm_rdata_i 是端口输入，原先当拍移位 / 旋转后直接写寄存器堆)，
  //   下一拍把数据移到段顺序，左旋 p0 字节到寄存器位置；字节使能分给 r0 (p >= p0) / r1 (p < p0，绕回部分)。
  //   在途计数 / 出错仍按 vm_done_i 当拍算；写回晚一拍，访存结束 (m_fin) 要等这一拍写完
  logic            mr_v;                                 // 寄存的那行本拍写寄存器堆
  logic [511:0]    mr_d;
  mq_t             mr_q;
  logic [VLEN-1:0] m_x;
  logic [VLB-1:0]  m_we0, m_we1;
  wire  [4:0]      m_wr0 = mr_q.r0;
  wire  [4:0]      m_wr1 = mr_q.r0 + 5'd1;
  always_comb begin
    logic [VLEN-1:0] y;
    logic [VLB-1:0]  ey, er;
    y   = VLEN'(mr_d >> (8 * int'(mr_q.off)));
    ey  = VLB'(mr_q.tact);
    m_x = (y << (8 * int'(mr_q.p0))) | (y >> (8 * (VLB - int'(mr_q.p0))));
    er  = (ey << mr_q.p0) | (ey >> (VLB - int'(mr_q.p0)));
    for (int p = 0; p < VLB; p++) begin
      m_we0[p] = er[p] && (p >= int'(mr_q.p0));
      m_we1[p] = er[p] && (p <  int'(mr_q.p0));
    end
  end
  wire m_run = (rs == R_MEM) && !m_fin;
  wire m_end = (int'(m_i) >= int'(m_evl));       // 全部发完
  // 队头本拍完成也可补发 (LSU 向量口 T/C/A 三级，2 个在途要背靠背补上；满时同拍一进一出，新项写进弹出的那格)
  assign vm_req_o   = m_run && !m_end && m_any && !m_mis && (mq_n < 2'd2 || vm_done_i);
  assign vm_we_o    = rq.st;
  assign vm_va_o    = m_fa;
  wire m_push = vm_req_o && vm_ack_i;
  // 前进：全部被掩掉 (跳过整个元素 / 组，不访存) 或请求被收下；不对齐只对有活动元素的判 (Spike 也不查被掩掉的)
  wire m_adv  = m_run && !m_end && (!m_any || m_push);
  wire m_nxe  = !m_any || (4'(m_f) + 4'd1 >= m_nfe);   // 本次之后换到下一个元素 (组)
  // 出错：LSU 报的 (在途队头，此时它是唯一在途的) / 不对齐 (等在途的都回来再报)
  wire m_lerr = m_run && vm_done_i && vm_fault_i;
  wire m_merr = m_run && !m_end && m_any && m_mis && (mq_n == 2'd0);
  wire m_err  = m_lerr || m_merr;
  wire [VLW-1:0] m_efe = m_lerr ? mqh.fe : m_fe;
  logic           m_trim;                        // fault-only-first：截短 vl
  logic [VLW-1:0] m_fv;                          // 出错的元素

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      m_fin <= 1'b0; m_flt <= 1'b0; m_trim <= 1'b0; mq_h <= 1'b0; mq_n <= '0;
    end else begin
      if (rs == R_SYNC && drained && rq.k == K_VMEM) begin
        m_fin <= 1'b0; m_flt <= 1'b0; m_trim <= 1'b0; mq_h <= 1'b0; mq_n <= '0;
      end else if (m_run) begin
        if (m_err) begin
          m_fin <= 1'b1;
          if (rq.mm == M_FF && m_efe != '0) m_trim <= 1'b1;  // 第 0 个元素之后出错：不陷入，vl = 该元素
          else m_flt <= 1'b1;
        end else if (m_end && mq_n == 2'd0 && !mr_v) m_fin <= 1'b1;
        mq_n <= mq_n + 2'(m_push) - 2'(vm_done_i);
        if (vm_done_i) mq_h <= !mq_h;
      end else if (rs != R_MEM) m_fin <= 1'b0;
    end
  end
  always_ff @(posedge clk_i) begin
    if (rs == R_SYNC && drained && rq.k == K_VMEM) begin
      m_i <= VLW'(vstart); m_f <= '0;
      m_a <= ra_q + 64'(vstart) * m_es;
    end else if (m_adv) begin
      if (m_nxe) begin
        m_f <= '0;
        m_i <= m_i + VLW'(m_n);
        m_a <= m_a + (m_grp ? (64'(m_n) << m_dw) : m_es);
      end else m_f <= m_f + 1'b1;
    end
    if (m_push) mq[mq_h ^ mq_n[0]] <= '{r0: m_r0, p0: m_p0, off: m_off, tact: m_tact, fe: m_fe, fa: m_fa};
    if (m_err) begin
      m_cause <= m_lerr ? vm_cause_i : (rq.st ? EXC_STORE_MISALIGN : EXC_LOAD_MISALIGN);
      m_tval  <= m_lerr ? mqh.fa : m_fa;
      m_fv    <= m_efe;
    end
  end
  always_ff @(posedge clk_i or negedge rst_ni)
    if (!rst_ni) mr_v <= 1'b0;
    else         mr_v <= m_run && !rq.st && vm_done_i && !vm_fault_i;
  always_ff @(posedge clk_i)
    if (vm_done_i) begin mr_d <= vm_rdata_i; mr_q <= mqh; end
  wire m_wr = mr_v;                                              // load 数据写回 (上一拍回来的那行)

  // 运算流水寄存器 (声明与说明见 c_dst 前)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) s2p_v <= '0;
    else s2p_v <= ANP'({s2p_v, s2_adv});
  end
  always_ff @(posedge clk_i) begin
    p_alt[0] <= x_div ? c_div : c_iota;
    p_red[0] <= dt_res;                                    // 点积 (S2 出)；归约在 P3 并入 (见 rc)
    p_e0[0] <= e0; p_act[0] <= c_act; p_last[0] <= s2_last; p_alts[0] <= x_div || (xd.k == K_VIOTA);
    p_wc[0] <= '{k: xd.k, vd: xd.vd, cw: xcw, dw: xdw, wm: x_wm};
    for (int k = 1; k < ANP; k++) begin
      p_alt[k] <= p_alt[k-1];
      p_red[k] <= (k == 3 && rc[2].red) ? rd_nacc : p_red[k-1];
      p_e0[k] <= p_e0[k-1]; p_act[k] <= p_act[k-1]; p_last[k] <= p_last[k-1]; p_alts[k] <= p_alts[k-1];
      p_wc[k] <= p_wc[k-1];
    end
  end
  always_comb begin
    p_wr = '0;
    for (int k = 0; k < ANP; k++) if (s2p_v[k]) p_wr = p_wr | p_wc[k].wm;
  end

  // ---- S3：写回的拍 (来自 S2，或浮点 / 掩码逻辑 / 置换 / vmv.s.x 由 S1 直接来)
  logic            s3_last, s3_sat;
  logic [VLW-1:0]  s3_e0;
  logic [NEB-1:0]  s3_act, s3_cmp;
  logic [DLEN-1:0] s3_dst;
  logic [63:0]     s3_red;                 // 归约 / 点积 / 浮点归约的最终结果 (最后一拍)
  logic            s3_pwe;                 // 置换：本元素写、写入的元素号与值
  logic [12:0]     s3_pidx;
  logic [63:0]     s3_pval;
  logic [VLEN-1:0] s3_mres, s3_mwe;        // 掩码逻辑 / vmsbf 类的结果与写使能 (S1 算好)
  kind_e           s3_k;                   // 写回上下文 (这一拍所属指令的类 / vd / 目的宽度)
  logic [4:0]      s3_vd;
  logic [1:0]      s3_dw;
  wire             s1_s3 = x_busy && !x_s2k && x_bdone;      // S1 直接进 S3
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) s3_v <= 1'b0;
    else s3_v <= p_out || s1_s3;
  end
  always_ff @(posedge clk_i) begin
    if (p_out) begin
      s3_e0 <= p_e0[ANP-1]; s3_act <= p_act[ANP-1]; s3_dst <= c_dst; s3_cmp <= c_cmp;
      s3_sat <= |(c_sat & p_act[ANP-1]); s3_last <= p_last[ANP-1]; s3_red <= p_red[ANP-1];
      s3_k <= p_wc[ANP-1].k; s3_vd <= p_wc[ANP-1].vd; s3_dw <= p_wc[ANP-1].dw;
    end else if (s1_s3) begin
      s3_k <= xd.k; s3_vd <= xd.vd; s3_dw <= xdw;
      s3_e0 <= e0_1; s3_act <= c_act1; s3_dst <= f_res2[xb[0]]; s3_cmp <= f_cmp2[xb[0]]; s3_sat <= 1'b0;
      s3_last <= (xb + 1'b1 >= xnb); s3_red <= f_fin;
      s3_pwe <= p_we; s3_pidx <= p_idx[12:0]; s3_pval <= p_val;
      s3_mres <= c_mres; s3_mwe <= c_mwe;
    end
  end

  // 目的切片在寄存器里的位置与字节写使能
  wire  [13:0]      d_bo  = 14'(int'(s3_e0) << (int'(s3_dw) + 3));
  wire  [4:0]       d_reg = s3_vd + 5'(d_bo >> LVLEN);
  wire  [LVLEN-1:0] d_pos = LVLEN'(d_bo);
  logic [DLEN/8-1:0] c_be;
  always_comb
    for (int bt = 0; bt < DLEN / 8; bt++) c_be[bt] = s3_act[(bt >> s3_dw) % NEB];
  // 写口：切片在块内的字节偏移 (切片按自身大小对齐，不跨块)
  wire  [3:0]             d_blk = 4'(d_pos >> LDLEN);
  wire  [LDLEN-1:0]       d_off = LDLEN'(d_pos);
  wire  [DLEN/8-1:0]      d_bes = c_be << (d_off >> 3);
  wire  [DLEN-1:0]        d_dts = s3_dst << d_off;
  // 掩码目的 (比较 / vmadc / 浮点比较)：本拍 NEB 位移到 e0
  logic [VLEN-1:0]        m_ben, m_bdt;
  always_comb begin
    m_ben = VLEN'(s3_act) << s3_e0;
    m_bdt = VLEN'(s3_cmp) << s3_e0;
  end

  //==========================================================================
  // 寄存器堆写 + CSR 更新
  //==========================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vl <= '0; vill <= 1'b1; vt <= '0; vstart <= '0; vxsat <= 1'b0; vxrm <= '0;
    end else begin
      // 饱和类运算按元素累积 (执行期间；CSR 写只在排空后，不会同拍)
      if (s3_v && s3_k == K_EW && s3_sat) vxsat <= 1'b1;
      if (rs == R_MEM && m_fin) begin
      vstart <= m_flt ? LVLEN'(m_fv) : '0;             // 出错：从出错元素恢复；完成：清零
      if (m_trim) vl <= m_fv;                          // fault-only-first：vl 截到出错元素
    end else if (do_sync) begin
      unique case (rq.k)
        K_VSET: begin vl <= vs_n.vl; vill <= vs_n.vill; vt <= vs_n.vt; vstart <= '0; end
        K_CSR: if (cwr)
          unique case (ca)
            CSR_VSTART: vstart <= cwv[LVLEN-1:0];
            CSR_VXSAT:  vxsat  <= cwv[0];
            CSR_VXRM:   vxrm   <= cwv[1:0];
            CSR_VCSR:   begin vxrm <= cwv[2:1]; vxsat <= cwv[0]; end
            default: ;
          endcase
        default: ;
      endcase
      end
    end
  end

  // 仅仿真：第一个时钟沿清零寄存器堆 (Spike 复位为 0；硬件不复位，软件不能假定初值)
`ifndef SYNTHESIS
  logic sim_zeroed;
  initial sim_zeroed = 1'b0;
  always @(posedge clk_i) sim_zeroed <= 1'b1;
`endif
  // 写口：每拍最多写两个寄存器 (访存 load 一行跨 r0 / r1；其余都只写一个)，各给 {寄存器号, 数据, 按位写使能}
  //   A 口 = 执行流水的各类目的 / load 的 r0，B 口 = load 的 r1。各类写先排成整寄存器的数据与写使能
  //   (数据按宽度复制到所有位置，写使能只开目标位置)，寄存器堆每位只剩 "A 命中 / B 命中 / 保持" —— 原先在一个 always_ff
  //   里按类别用可变下标分别写 (寄存器号、块号、元素号都是变量)，DC 给 32K 位每位展开一串选择器
  //   访存只在同步执行 (队列与执行流水已排空) 时进行，与执行流水不会同拍写 (断言)
  function automatic logic [VLEN-1:0] b2bit(input logic [VLB-1:0] be);   // 字节使能 -> 位使能
    logic [VLEN-1:0] m;
    for (int k = 0; k < VLB; k++) m[8*k +: 8] = {8{be[k]}};
    return m;
  endfunction
  function automatic logic [VLEN-1:0] lowm(input logic [1:0] w);          // 低 8 << w 位 (4 个常数选一)
    unique case (w)
      2'd0:    return VLEN'({8{1'b1}});
      2'd1:    return VLEN'({16{1'b1}});
      2'd2:    return VLEN'({32{1'b1}});
      default: return VLEN'({64{1'b1}});
    endcase
  endfunction
  function automatic logic [VLEN-1:0] erep(input logic [63:0] v, input logic [1:0] w);   // 宽度 w 的元素复制满一个寄存器
    logic [VLEN-1:0] r;
    unique case (w)
      2'd0:    for (int k = 0; k < VLEN / 8;  k++) r[8*k +: 8]   = v[7:0];
      2'd1:    for (int k = 0; k < VLEN / 16; k++) r[16*k +: 16] = v[15:0];
      2'd2:    for (int k = 0; k < VLEN / 32; k++) r[32*k +: 32] = v[31:0];
      default: for (int k = 0; k < VLEN / 64; k++) r[64*k +: 64] = v;
    endcase
    return r;
  endfunction
  logic            wa_v, wb_v;
  logic [4:0]      wa_r, wb_r;
  logic [VLEN-1:0] wa_d, wa_m, wb_d, wb_m;
  wire             x_wr = s3_v;
  logic [31:0]     wa_hit, wb_hit;                          // 各寄存器被 A / B 口写 (译码一次，位循环里共用)
  always_comb
    for (int r = 0; r < 32; r++) begin
      wa_hit[r] = wa_v && wa_r == 5'(r);
      wb_hit[r] = wb_v && wb_r == 5'(r);
    end
  wire             x_last = s3_last;
  always_comb begin
    wa_v = 1'b0; wa_r = s3_vd; wa_d = '0; wa_m = '0;
    wb_v = m_wr; wb_r = m_wr1; wb_d = m_x; wb_m = b2bit(m_we1);
    if (m_wr) begin
      wa_v = 1'b1; wa_r = m_wr0; wa_d = m_x; wa_m = b2bit(m_we0);
    end
    if (x_wr) begin
      unique case (s3_k)
        K_EW, K_VIOTA, K_VMVNR, K_FEW: begin                 // 目的切片在 d_reg 的第 d_blk 个 DLEN 块内
          logic [DLEN-1:0] dm;
          for (int bt = 0; bt < DLEN / 8; bt++) dm[8*bt +: 8] = {8{d_bes[bt]}};
          wa_v = 1'b1; wa_r = d_reg;
          for (int k = 0; k < VLEN / DLEN; k++) begin
            wa_d[DLEN*k +: DLEN] = d_dts;
            wa_m[DLEN*k +: DLEN] = (k == int'(d_blk)) ? dm : '0;
          end
        end
        K_EWM, K_FEWM: begin wa_v = 1'b1; wa_d = m_bdt; wa_m = m_ben; end     // 掩码目的：位 e0..
        K_VMVSX: begin wa_v = 1'b1; wa_d = VLEN'(xa); wa_m = lowm(sew); end
        K_DOT, K_FRED, K_RED: if (x_last) begin wa_v = 1'b1; wa_d = VLEN'(s3_red); wa_m = lowm(s3_dw); end
        K_MLOG, K_MSET: begin wa_v = 1'b1; wa_d = s3_mres; wa_m = s3_mwe; end
        K_PERM:
          if (s3_pwe) begin
            int bo, ei;
            bo = int'(s3_pidx) << (int'(xsew) + 3);
            ei = int'(s3_pidx);
            wa_v = 1'b1;
            wa_r = s3_vd + 5'(bo >> LVLEN);
            wa_d = erep(s3_pval, xsew);
            unique case (xsew)
              2'd0:    wa_m = VLEN'({8{1'b1}})  << (8  * (ei % (VLEN / 8)));
              2'd1:    wa_m = VLEN'({16{1'b1}}) << (16 * (ei % (VLEN / 16)));
              2'd2:    wa_m = VLEN'({32{1'b1}}) << (32 * (ei % (VLEN / 32)));
              default: wa_m = VLEN'({64{1'b1}}) << (64 * (ei % (VLEN / 64)));
            endcase
          end
        default: ;
      endcase
    end
  end
  always_ff @(posedge clk_i) begin
`ifndef SYNTHESIS
    if (!sim_zeroed) for (int r = 0; r < 32; r++) vrf[r] <= '0;
`endif
    for (int r = 0; r < 32; r++)
      for (int i = 0; i < VLEN; i++)
        if (wa_hit[r] && wa_m[i]) vrf[r][i] <= wa_d[i];
        else if (wb_hit[r] && wb_m[i]) vrf[r][i] <= wb_d[i];
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk_i)
    if (rst_ni) assert (!(req_v_i && rs != R_IDLE)) else $error("vu: 上一条请求未完成又来新请求");
  always_ff @(posedge clk_i)
    if (rst_ni) assert (!(m_wr && x_wr)) else $error("vu: 访存 load 写回与执行流水同拍写寄存器堆");
  always_ff @(posedge clk_i)
    if (rst_ni) assert (!(p_out && s1_s3)) else $error("vu: 运算流水出口与 S1 直通同拍进 S3");
  // +vutrace：每条异步指令开始执行 / 每条请求的响应
  bit vutrace;
  initial vutrace = $test$plusargs("vutrace");
  always_ff @(posedge clk_i)
    if (vutrace && rst_ni) begin
      if (x_start) $display("[vu %0t] 执行 inst=%08h a=%016h vl=%0d sew=%0d", $time, qi, q_a[q_h], vl, sew);
      if (do_rsp)  $display("[vu %0t] 响应 inst=%08h k=%0d excp=%0d data=%016h", $time, ri, rd_.k, rsp_excp_o, rsp_data_o);
    end
`endif
endmodule

//------------------------------------------------------------------------------
// o6_vu_alu：一个计算宽度为 W 的元素槽 (每拍 DLEN / W 个)，结果 r、掩码位 m、饱和 s
//   a = vs2 元素，b = vs1 元素 / 标量 / 立即数，c = vd 旧值 (都已扩展到 W)；cin = v0 的位
//   运算器各一个、按操作码复用 (原先每种操作各写一个，DC 展开成每槽十来个加法器 / 比较器 / 移位器与 9 个乘法器)：
//   W+1 位加法器 t：按操作选零 / 符号扩展、加 / 减 / 反减、进位输入，t 是精确的和 / 差 ——
//     进位借位 = t[W]，有符号溢出 = t[W] != t[W-1]，比较 a < b = 差的符号 t[W]，平均 = t[W:1] 再舍入；
//   右移器：{a, 0} 右移 sh 位，高 W 位是移位结果、最低位是舍入位 (第 sh-1 位)；粘滞 = 更低位非零；
//   乘法器：(W+1) x (W+1) 有符号 (见 pm)
// 流水 (NP >= 4；NP = 0 为纯组合，单元测试用；1–3 不支持)：
//   E0：操作码译码，选好乘法器的两个操作数 (含符号位) 与第三个加数 zz (乘加的 vd / vs2，vsmul 的舍入半位) -> R1
//   E1：乘数按 8 位分段的小乘法器各出两个部分积向量 (DW02_multp)；非乘法的运算算到舍入加 1 之前 -> R2
//   E2：全部部分积向量 + zz 压成两个 (DW02_tree)；非乘法的舍入加 1、收窄饱和 -> R3
//   E3：进位传播相加 (乘加 / 舍入已在 zz 里，不再串第二个加法器)，取低 / 高半 / vsmul 结果，与非乘法结果合并 -> R4 .. R<NP> (输出)
//   (2026-10-27 单独综合 64 位槽：组合 3.9 ns；只在输出放 2 / 3 级寄存器靠重定时，最差级 -0.24 / -0.04 ns ——
//    DC 把乘法树与末级加法器聚成一个数据通路块，寄存器推不匀；显式切 3 级时整个 65x65 乘法树一级 2.53 ns，再把树拆成两级)
//   综合 (O6_ASIC) / O6_DW_SIM 用 DW02_multp / DW02_tree；否则行为模型 (out0 = 和，out1 = 0)
//------------------------------------------------------------------------------
`ifdef O6_ASIC
`define O6_VALU_DW
`elsif O6_DW_SIM
`define O6_VALU_DW
`endif
module o6_vu_alu
  import o6_vu_pkg::*;
#(
  parameter int W  = 8,
  parameter int NP = 0
) (
  input  wire          clk_i,
  input  aop_e         op_i,
  input  wire [W-1:0]  a_i, b_i, c_i,
  input  wire          cin_i,
  input  wire [1:0]    xrm_i,
  output logic [W-1:0] r_o,
  output logic         m_o, s_o
);
  localparam int LW = $clog2(W);
  localparam int PW = 2 * W + 4;          // DW02_multp 输出宽度 (>= 两个操作数宽度之和 + 2)
  localparam logic [W-1:0] SMIN = {1'b1, {(W-1){1'b0}}};
  localparam logic [W-1:0] SMAX = {1'b0, {(W-1){1'b1}}};

  //---------------------------------------------------------------- E0 -> R1
  // 乘法类在 E2 取哪段结果
  typedef enum logic [1:0] { M_NONE, M_LO, M_HI, M_SMUL } msel_e;
  typedef struct packed {
    aop_e         op;
    logic [W-1:0] a, b, c;
    logic         cin;
    logic [1:0]   xrm;
    logic [W:0]   ma, mb;                 // 乘法器的两个 W+1 位有符号操作数 (已按操作选好 / 扩展好符号)
    logic [W-1:0] zz;                     // E2 与部分积一起相加的第三个数 (乘加的加数 / vsmul 的舍入半位)
    logic         neg;                    // vnmsac / vnmsub：zz 取反码、结果低 W 位再取反 (~(~c + p) = c - p)
    msel_e        ms;
    logic [W:0]   xo, yo;                 // 加法器两个输入 (已按操作扩展符号、交换、取反)，E1 只做 t = xo + yo + ci
    logic         ci;
  } s1_t;
  s1_t e0, q1;
  always_comb begin
    logic         mxc, mxs, mys;          // 乘数取 c (vmadd / vnmsub) / 乘数按有符号 / 被乘数 (b) 按有符号
    logic [W-1:0] mx;
    e0.op  = op_i; e0.a = a_i; e0.b = b_i; e0.c = c_i; e0.cin = cin_i; e0.xrm = xrm_i;
    mxc = (op_i == A_MADD || op_i == A_NMSUB);
    mxs = (op_i == A_MULH || op_i == A_MULHSU || op_i == A_SMUL);
    mys = (op_i == A_MULH || op_i == A_SMUL);
    mx  = mxc ? c_i : a_i;
    e0.ma = {mxs && mx[W-1], mx};
    e0.mb = {mys && b_i[W-1], b_i};
    e0.neg = (op_i == A_NMSAC || op_i == A_NMSUB);
    unique case (op_i)
      A_MACC, A_WMACC, A_NMSAC: e0.zz = c_i;
      A_MADD, A_NMSUB:          e0.zz = a_i;
      // vsmul：积右移 W-1 位舍入；rnu / rne 先加半位 (2^(W-2))，rne 的「正好一半」在 E2 清最低位
      A_SMUL:                   e0.zz = xrm_i[1] ? '0 : W'(1) << (W - 2);
      default:                  e0.zz = '0;
    endcase
    if (e0.neg) e0.zz = ~e0.zz;
    // 加法器的用法 (译码与操作数选择放在 E0，E1 从寄存器直接进加法器)
    begin
      logic         is_add, sg, rsub, sub;
      logic [W:0]   xa, xb, y;
      is_add = op_i == A_ADD || op_i == A_ADC || op_i == A_MADC || op_i == A_SADDU || op_i == A_SADD ||
               op_i == A_AADDU || op_i == A_AADD;
      sg     = op_i == A_SADD || op_i == A_SSUB || op_i == A_AADD || op_i == A_ASUB ||
               op_i == A_MIN || op_i == A_MAX || op_i == C_LT || op_i == C_LE || op_i == C_GT;   // 按有符号扩展
      rsub   = op_i == A_RSUB;
      sub    = !is_add;                                                                        // 减 (含反减、比较、最值)
      e0.ci  = (op_i == A_ADC || op_i == A_MADC) ? cin_i : (op_i == A_SBC || op_i == A_MSBC) ? !cin_i : sub;
      xa = {sg && a_i[W-1], a_i};
      xb = {sg && b_i[W-1], b_i};
      e0.xo = rsub ? xb : xa;
      y     = rsub ? xa : xb;
      e0.yo = sub ? ~y : y;
    end
    unique case (op_i)
      A_SMUL: e0.ms = M_SMUL;
      A_MUL, A_MACC, A_WMACC, A_NMSAC, A_MADD, A_NMSUB: e0.ms = M_LO;
      A_MULH, A_MULHU, A_MULHSU: e0.ms = M_HI;
      default: e0.ms = M_NONE;
    endcase
  end

  //---------------------------------------------------------------- E1 -> R2
  aop_e         op;
  logic [W-1:0] a, b, c;
  logic         cin;
  logic [1:0]   xrm;
  assign op = q1.op;
  assign {a, b, c, cin, xrm} = {q1.a, q1.b, q1.c, q1.cin, q1.xrm};
  wire [W:0] t  = q1.xo + q1.yo + (W+1)'(q1.ci);
  wire       eq = (a == b);
  wire       lt = t[W];                // a < b (按 sg)：a - b 的符号
  wire       gt = !lt && !eq;
  wire       ovf = t[W] != t[W-1];     // 有符号加 / 减溢出
  // 平均加减：t 右移 1 位再按 vxrm 舍入；舍入的加 1 放到 E2 (这里只出 aq 与 ainc)
  logic [W-1:0] aq;
  logic         ainc;
  always_comb begin
    aq = t[W:1];
    unique case (xrm)
      2'd0:    ainc = t[0];
      2'd1:    ainc = t[0] && t[1];
      default: ainc = 1'b0;
    endcase
    if (xrm == 2'd3) aq[0] = aq[0] | t[0];
  end

  // 右移器 (逻辑 / 算术) + 舍入：vsrl / vsra / vnsrl / vnsra 取移位结果，vssrl / vssra / vnclip(u) 取舍入后的 W+1 位
  wire [LW-1:0] sh  = b[LW-1:0];
  wire          sra = op == A_SRA || op == A_NSRA || op == A_SSRA || op == A_NCLIP;
  wire [W+1:0]  rxw = $signed({sra && a[W-1], a, 1'b0}) >>> sh;
  wire [W-1:0]  qs  = rxw[W:1];        // a >> sh
  wire          rb  = rxw[0];          // 舍入位：a[sh-1] (sh = 0 时为 0)
  logic         st;                    // 粘滞：a[sh-2:0] 非零
  always_comb begin
    st = 1'b0;
    for (int k = 0; k < W - 1; k++) if (k + 1 < int'(sh)) st = st | a[k];
  end
  logic [W:0] sq;                      // 舍入前 (W+1 位)；E2 加 sinc 得舍入后的 qr (收窄饱和按 qr 比较)
  logic       sinc;
  always_comb begin
    logic [W-1:0] q;
    q = qs;
    unique case (xrm)
      2'd0:    sinc = rb;
      2'd1:    sinc = rb && (st || qs[0]);
      default: sinc = 1'b0;
    endcase
    if (xrm == 2'd3 && (rb || st)) q[0] = 1'b1;
    sq = {sra && q[W-1], q};
  end
  localparam logic [W:0] NUMAX = (W+1)'({(W/2){1'b1}});           // 收窄：无符号最大值
  localparam logic [W:0] NSMAX = (W+1)'({(W/2-1){1'b1}});         // 有符号最大值
  localparam logic [W:0] NSMIN = -(W+1)'({1'b1, {(W/2-1){1'b0}}}); // 有符号最小值

  // 唯一的乘法器：(W+1) x (W+1) 有符号。乘数 x = vd (vmadd / vnmsub 是 vd * vs1) 或 vs2；
  // 符号位按操作扩展 (MULH / vsmul 两边有符号，MULHSU 只有 x)，低 2W 位就是各种符号组合的精确积。
  // 操作数在 E0 选好。一个 (W+1) x (W+1) 的乘法树一级放不下 (64 位槽单独综合：2.53 ns)，按 mb 切成 NCK 段
  // 每段 CB 位 (最高段 CB+1 位、带符号；其余段补 0 当正数)：E1 每段一个 (W+1) x (CB+1) 小乘法器出两个部分积向量，
  // 第 k 段的积左移 CB*k；输出宽度取 PW - CB*k，左移后正好模 2^PW (各段之和 = 积 mod 2^PW)
  localparam int CB  = 8;
  localparam int NCK = W / CB;
  logic [2*NCK-1:0][PW-1:0] pv;
  for (genvar k = 0; k < NCK; k++) begin : g_ck
    localparam int OW = PW - CB * k;
    wire [CB:0]   bk;
    wire [OW-1:0] o0, o1;
    if (k == NCK - 1) begin : g_top
      assign bk = q1.mb[W:CB*k];
    end else begin : g_lo
      assign bk = {1'b0, q1.mb[CB*k+CB-1:CB*k]};
    end
`ifdef O6_VALU_DW
    DW02_multp #(.a_width(W + 1), .b_width(CB + 1), .out_width(OW)) u_pp (
      .a(q1.ma), .b(bk), .tc(1'b1), .out0(o0), .out1(o1));
`else
    assign o0 = OW'($signed(q1.ma) * $signed(bk));
    assign o1 = '0;
`endif
    assign pv[2*k]   = PW'(o0) << (CB * k);
    assign pv[2*k+1] = PW'(o1) << (CB * k);
  end

  // Zvbb：按槽宽 W (= SEW) 的位操作
  function automatic logic [W-1:0] f_brev(input logic [W-1:0] x);
    for (int k = 0; k < W; k++) f_brev[k] = x[W-1-k];
  endfunction
  function automatic logic [W-1:0] f_brev8(input logic [W-1:0] x);
    for (int k = 0; k < W; k++) f_brev8[k] = x[(k & ~7) + 7 - (k & 7)];
  endfunction
  function automatic logic [W-1:0] f_rev8(input logic [W-1:0] x);
    for (int k = 0; k < W / 8; k++) f_rev8[8*k +: 8] = x[W - 8 - 8*k +: 8];
  endfunction
  function automatic logic [W-1:0] f_clz(input logic [W-1:0] x);
    logic f;
    f_clz = '0; f = 1'b0;
    for (int k = W - 1; k >= 0; k--) begin f = f | x[k]; f_clz = f_clz + W'(!f); end
  endfunction
  function automatic logic [W-1:0] f_ctz(input logic [W-1:0] x);
    logic f;
    f_ctz = '0; f = 1'b0;
    for (int k = 0; k < W; k++) begin f = f | x[k]; f_ctz = f_ctz + W'(!f); end
  endfunction
  function automatic logic [W-1:0] f_cpop(input logic [W-1:0] x);
    f_cpop = '0;
    for (int k = 0; k < W; k++) f_cpop = f_cpop + W'(x[k]);
  endfunction
  wire [2*W-1:0] rol2 = {a, a} << sh;
  wire [2*W-1:0] ror2 = {a, a} >> sh;

  // 非乘法的结果 (乘法类在 E2 覆盖)
  // 要在 E2 收尾的：rk = 舍入加 1 之后取低 W 位 / 无符号收窄饱和 / 有符号收窄饱和，值 = rv + ri
  typedef enum logic [1:0] { RK_NONE, RK_W, RK_NU, RK_NS } rk_e;
  logic [W-1:0] r_n;
  logic         m_n, s_n;
  rk_e          rk_n;
  logic [W:0]   rv_n;
  logic         ri_n;
  always_comb begin
    r_n = '0; m_n = 1'b0; s_n = 1'b0;
    rk_n = RK_NONE; rv_n = sq; ri_n = sinc;
    unique case (op)
      A_ADD, A_SUB, A_RSUB, A_ADC, A_SBC: r_n = t[W-1:0];
      A_MINU, A_MIN: r_n = lt ? a : b;
      A_MAXU, A_MAX: r_n = gt ? a : b;
      A_AND:  r_n = a & b;
      A_OR:   r_n = a | b;
      A_XOR:  r_n = a ^ b;
      A_SLL:  r_n = a << sh;
      A_SRL, A_NSRL, A_SRA, A_NSRA: r_n = qs;
      A_MERGE: r_n = cin ? b : a;
      A_MV:   r_n = b;
      A_SGNJ:  r_n = {b[W-1], a[W-2:0]};
      A_SGNJN: r_n = {!b[W-1], a[W-2:0]};
      A_SGNJX: r_n = {a[W-1] ^ b[W-1], a[W-2:0]};
      C_EQ:   m_n = eq;
      C_NE:   m_n = !eq;
      C_LTU, C_LT: m_n = lt;
      C_LEU, C_LE: m_n = lt || eq;
      C_GTU, C_GT: m_n = gt;
      A_MADC, A_MSBC: m_n = t[W];
      A_SADDU: begin s_n = t[W]; r_n = s_n ? '1 : t[W-1:0]; end
      A_SSUBU: begin s_n = t[W]; r_n = s_n ? '0 : t[W-1:0]; end
      A_SADD, A_SSUB: begin s_n = ovf; r_n = ovf ? (t[W] ? SMIN : SMAX) : t[W-1:0]; end
      A_AADDU, A_AADD, A_ASUBU, A_ASUB: begin rk_n = RK_W; rv_n = {1'b0, aq}; ri_n = ainc; end
      A_SSRL, A_SSRA: rk_n = RK_W;
      A_NCLIPU: rk_n = RK_NU;
      A_NCLIP:  rk_n = RK_NS;
      A_EXT:   r_n = a;
      A_ANDN:  r_n = a & ~b;
      A_ROL:   r_n = rol2[2*W-1:W];
      A_ROR:   r_n = ror2[W-1:0];
      A_BREV:  r_n = f_brev(a);
      A_BREV8: r_n = f_brev8(a);
      A_REV8:  r_n = f_rev8(a);
      A_CLZ:   r_n = f_clz(a);
      A_CTZ:   r_n = f_ctz(a);
      A_CPOP:  r_n = f_cpop(a);
      default: r_n = b;
    endcase
  end

  typedef struct packed {
    msel_e        ms;
    logic [1:0]   xrm;
    logic         neg;
    logic         smm;                    // vsmul 的 最小负数 * 最小负数 (饱和)
    logic [2*NCK-1:0][PW-1:0] pv;
    logic [W-1:0] zz;
    logic [W-1:0] r;
    logic         m, s;
    rk_e          rk;
    logic [W:0]   rv;
    logic         ri;
  } s2_t;
  s2_t e1, q2;
  always_comb begin
    e1.ms = q1.ms; e1.xrm = xrm; e1.neg = q1.neg; e1.zz = q1.zz;
    e1.smm = (a == SMIN) && (b == SMIN);
    e1.pv = pv;
    e1.r = r_n; e1.m = m_n; e1.s = s_n;
    e1.rk = rk_n; e1.rv = rv_n; e1.ri = ri_n;
  end

  //---------------------------------------------------------------- E2 -> R3
  // 2*NCK 个部分积向量 + zz 压成进位保存的两个 (DW02_tree，华莱士树，不做进位传播)
  logic [PW-1:0] t0, t1;
`ifdef O6_VALU_DW
  DW02_tree #(.num_inputs(2 * NCK + 1), .input_width(PW)) u_tree (
    .INPUT({q2.pv, PW'(q2.zz)}), .OUT0(t0), .OUT1(t1));
`else
  always_comb begin
    logic [PW-1:0] sm;
    sm = PW'(q2.zz);
    for (int k = 0; k < 2 * NCK; k++) sm += q2.pv[k];
    t0 = sm;
  end
  assign t1 = '0;
`endif
  typedef struct packed {
    msel_e        ms;
    logic [1:0]   xrm;
    logic         neg;
    logic         smm;
    logic [PW-1:0] t0, t1;
    logic [W-1:0] r;
    logic         m, s;
  } s3_t;
  s3_t e2, q3;
  // 非乘法结果的收尾：舍入加 1，收窄饱和
  wire [W:0] qr = q2.rv + (W+1)'(q2.ri);
  always_comb begin
    e2.ms = q2.ms; e2.xrm = q2.xrm; e2.neg = q2.neg; e2.smm = q2.smm;
    e2.t0 = t0; e2.t1 = t1;
    e2.r = q2.r; e2.m = q2.m; e2.s = q2.s;
    unique case (q2.rk)
      RK_W:  e2.r = qr[W-1:0];
      RK_NU: if (qr > NUMAX) begin e2.s = 1'b1; e2.r = W'(NUMAX); end else e2.r = qr[W-1:0];
      RK_NS: begin
        if ($signed(qr) > $signed(NSMAX)) begin e2.s = 1'b1; e2.r = W'(NSMAX); end
        else if ($signed(qr) < $signed(NSMIN)) begin e2.s = 1'b1; e2.r = W'({1'b1, {(W/2-1){1'b0}}}); end   // 只用低 W/2 位 (收窄)
        else e2.r = qr[W-1:0];
      end
      default: ;
    endcase
  end

  //---------------------------------------------------------------- E3 -> R4 .. R<NP>
  // 进位传播相加：pf = 积 + zz (vsmul：积 + 舍入半位)
  wire [PW-1:0] pf = q3.t0 + q3.t1;
  wire          lz = pf[W-2:0] == '0;     // vsmul：右移掉的 W-1 位全零 (rne 加半位后 = 原来正好一半；rod 不加 = 恰好整除)
  logic [W-1:0] r_c;
  logic         m_c, s_c;
  always_comb begin
    logic [W-1:0] q;
    r_c = q3.r; m_c = q3.m; s_c = q3.s;
    q = pf[2*W-2:W-1];
    if (q3.xrm == 2'd1 && lz) q[0] = 1'b0;          // rne 正好一半：取偶
    if (q3.xrm == 2'd3 && !lz) q[0] = 1'b1;         // rod：有丢弃位就置最低位
    unique case (q3.ms)
      M_SMUL: if (q3.smm) begin s_c = 1'b1; r_c = SMAX; end else r_c = q;
      M_LO:   r_c = pf[W-1:0] ^ {W{q3.neg}};
      M_HI:   r_c = pf[2*W-1:W];
      default: ;
    endcase
  end

  if (NP == 0) begin : g_comb
    assign q1 = e0;
    assign q2 = e1;
    assign q3 = e2;
    assign {s_o, m_o, r_o} = {s_c, m_c, r_c};
  end else begin : g_pipe
    logic [W+1:0] pr [NP-3];
    always_ff @(posedge clk_i) begin
      q1 <= e0;
      q2 <= e1;
      q3 <= e2;
      pr[0] <= {s_c, m_c, r_c};
      for (int k = 1; k < NP - 3; k++) pr[k] <= pr[k-1];
    end
    assign {s_o, m_o, r_o} = pr[NP-4];
  end
endmodule
`undef O6_VALU_DW

//------------------------------------------------------------------------------
// o6_vu_dvs：一个除法槽，恢复余数除法每拍 1 位商 (w = 8 << cw 拍)；W = 本槽最大元素宽度 (w <= W)
//   无符号做模，有符号先取绝对值、最后修正符号；除 0：商全 1、余数 = 被除数；
//   有符号溢出 (最小值 / -1)：商 = 被除数、余数 = 0 (与 Spike 一致)
//------------------------------------------------------------------------------
module o6_vu_dvs #(
  parameter int W = 64
) (
  input  wire          clk_i,
  input  wire          ld_i,             // 装入本拍操作数
  input  wire          step_i,           // 迭代一位
  input  wire [1:0]    cw_i,             // log2(w / 8)
  input  wire          sgn_i,            // 有符号 (vdiv / vrem)
  input  wire          div_i,            // 取商 (否则取余数)
  input  wire [W-1:0]  a_i, b_i,         // 被除数 / 除数 (本槽元素，w 位零扩展)
  output logic [W-1:0] y_o
);
  logic [W-1:0] m, top;                  // 低 w 位全 1 / 第 w-1 位
  always_comb
    for (int k = 0; k < W; k++) begin
      m[k]   = (k < (8 << cw_i));
      top[k] = (k == (8 << cw_i) - 1);
    end
  function automatic logic msb(input logic [W-1:0] v, input logic [W-1:0] t);
    return |(v & t);
  endfunction

  logic [W-1:0] q, d;                    // 被除数左移 / 商；除数 (绝对值)
  logic [W:0]   r;                       // 余数
  logic         nq, nr, z, ov;
  always_ff @(posedge clk_i) begin
    logic sa, sb;
    sa = sgn_i && msb(a_i, top);
    sb = sgn_i && msb(b_i, top);
    if (ld_i) begin
      q  <= sa ? (~a_i + 1'b1) & m : a_i;
      d  <= sb ? (~b_i + 1'b1) & m : b_i;
      r  <= '0;
      nq <= sa ^ sb; nr <= sa;
      z  <= (b_i == '0);
      ov <= sgn_i && a_i == top && b_i == m;
    end else if (step_i) begin
      logic [W:0] r2;
      r2 = {r[W-1:0], msb(q, top)};
      if (r2 >= {1'b0, d}) begin r <= r2 - {1'b0, d}; q <= {q[W-2:0], 1'b1}; end
      else begin r <= r2; q <= {q[W-2:0], 1'b0}; end
    end
  end
  always_comb begin
    logic [W-1:0] qq, rr;
    qq = nq ? -q : q;
    rr = nr ? -r[W-1:0] : r[W-1:0];
    if (div_i) y_o = z ? '1 : ov ? a_i : qq;
    else       y_o = z ? a_i : ov ? '0 : rr;
  end
endmodule

`ifdef O6_VU_FPBB
//------------------------------------------------------------------------------
// o6_vu_fpbb：时序摸底用的 fpnew 桩 (输出寄存；不是功能模型，只在综合 VU_FPBB=1 时用)
//------------------------------------------------------------------------------
module o6_vu_fpbb #(
  parameter int TW = 1,
  parameter int CW = 16
) (
  input  wire           clk_i,
  input  wire  [191:0]  opnd_i,
  input  wire  [CW-1:0] ctl_i,
  input  wire  [TW-1:0] tag_i,
  input  wire           v_i,
  output logic [63:0]   r_o,
  output logic [4:0]    st_o,
  output logic [TW-1:0] tag_o,
  output logic          v_o
);
  always_ff @(posedge clk_i) begin
    r_o   <= opnd_i[63:0] ^ opnd_i[127:64] ^ opnd_i[191:128] ^ 64'(ctl_i);
    st_o  <= opnd_i[4:0] ^ opnd_i[68:64] ^ ctl_i[4:0];
    tag_o <= tag_i;
    v_o   <= v_i;
  end
endmodule
`endif
