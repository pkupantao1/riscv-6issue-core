//==============================================================================
// o6_pkg.sv - OOO6 微架构参数、uop 格式、全局广播格式
//
// 参数取值依据见 docs/01_microarch_spec.md。改这里之前先改文档。
//==============================================================================
`ifndef O6_PKG_SV
`define O6_PKG_SV
`default_nettype none

package o6_pkg;

  //--------------------------------------------------------------------------
  // 宽度
  //--------------------------------------------------------------------------
  localparam int XLEN       = 64;
  localparam int VLEN_ADDR  = 64;   // PC / VA 在前端与重定向路径上保持 64 位，合法性在取指时查
  // PMP 粒度 G：2^(G+2) 字节 = 64 B (#34)：取指块不跨 64 B 行，每块必落在同一个 PMP 粒度单元里，只查块首即精确；
  //   自然对齐的标量访存 (<= 8 B) 与向量逐行访存也不会跨粒度单元。G >= 1 时 NA4 不可选 (写入按 NAPOT)
  localparam int PMP_G      = 4;
  localparam int PA_W       = 40;   // 物理地址实现宽度 (PTE 给出的高位非零 -> access fault)
  // 虚地址：Sv39 / Sv48 / Sv57 (RVA23 R4，docs/14)。TLB / PTW 按 57 位 (VPN 45 位) 做，规范地址检查按 satp.MODE；
  //   vam = satp.MODE[1:0]：0 = Sv39，1 = Sv48，2 = Sv57 (MODE 只取 0 / 8 / 9 / 10，翻译开 = MODE[3])
  localparam int VA_W       = 59;   // 57 位 VA + 2：H 的 vsatp = Bare 时 GPA = VA，Sv57x4 的 GPA 有 59 位
  localparam int VPN_W      = VA_W - 12;
  localparam int W          = 6;    // 译码 / 重命名 / 分派 / 提交宽度
  // 向量单元 (RVV 1.0，docs/11_vector_uarch.md)：VLEN 固定 1024；DLEN = 每拍处理的位数 (FPGA 256，ASIC 满配 1024)
  localparam int VLEN       = 1024;
`ifdef O6_VDLEN
  localparam int VDLEN      = `O6_VDLEN;
`else
  localparam int VDLEN      = 256;
`endif
  localparam int FETCH_BYTES = 32;  // 取指块
  localparam int FETCH_HW   = FETCH_BYTES / 2;   // 一个取指块里的 16 位槽数 (16)

  //--------------------------------------------------------------------------
  // 重命名资源
  //--------------------------------------------------------------------------
  // ROB 扩到 258 项后在途写寄存器的指令可达 ~180 条：整数 256 / 浮点 192 (2026-10-13，原 192 / 160)
  localparam int N_IPREG    = 256;
  localparam int N_FPREG    = 192;
  localparam int PREG_W     = 8;
  // 在途未决分支 (条件分支 + JALR)。16 -> 32 (2026-10-13)：t50 CoreMark 重命名停顿中"分支标签用完"占全程 36%
`ifdef O6_MAX_BR
  localparam int MAX_BR     = `O6_MAX_BR;   // 仅供性能对比实验 (+define+O6_MAX_BR=n)
`else
  localparam int MAX_BR     = 48;   // 32 -> 48 (2026-10-14)：IQ 加大后分支标签又成为限制 (exp6)
`endif
  localparam int BR_TAG_W   = $clog2(MAX_BR);
  localparam int MAX_BR_PER_GRP = 2;

  //--------------------------------------------------------------------------
  // ROB：43 行 x 6 列 = 258 项 (用户要求 256 项；一个分派组占一整行，行数不必是 2 的幂，row_inc 显式回绕)
  //--------------------------------------------------------------------------
`ifdef O6_ROB_ROWS
  localparam int ROB_ROWS   = `O6_ROB_ROWS;
`else
  localparam int ROB_ROWS   = 43;
`endif
  localparam int ROB_ROW_W  = 6;
  localparam int ROB_COL_W  = 3;
  localparam int ROB_IDX_W  = ROB_ROW_W + ROB_COL_W;   // {row, col}

  //--------------------------------------------------------------------------
  // 访存队列
  //--------------------------------------------------------------------------
`ifdef O6_LQ_SIZE
  localparam int LQ_SIZE    = `O6_LQ_SIZE;   // 2 的幂 (指针自然回绕)
`else
  localparam int LQ_SIZE    = 128;  // 64 -> 128 (2026-10-14，exp7：分派停顿里 LQ 满 11 万拍)
`endif
  localparam int LQ_W       = $clog2(LQ_SIZE);
`ifdef O6_SQ_SIZE
  localparam int SQ_SIZE    = `O6_SQ_SIZE;   // 2 的幂 (指针自然回绕)
`else
  // 32 -> 64 (2026-10-13)：t53 dhrystone 分派停顿里 SQ 满 1.75 万拍，加大后 mcycle 63561 -> 55708
  localparam int SQ_SIZE    = 64;
`endif
  localparam int SQ_W       = $clog2(SQ_SIZE);

  //--------------------------------------------------------------------------
  // 前端
  //--------------------------------------------------------------------------
`ifdef O6_FTQ_SIZE
  localparam int FTQ_SIZE   = `O6_FTQ_SIZE;  // 2 的幂
`else
  // 32 -> 64 (2026-10-13)：ROB 扩到 258 后在途块超过 32，FTQ 满让前端停顿 (dhrystone 慢 9%)
  localparam int FTQ_SIZE   = 64;
`endif
  localparam int FTQ_W      = $clog2(FTQ_SIZE);
  localparam int FTQ_OFF_W  = 5;    // 取指组内 2 B 粒度偏移 (跨块 RVI 并入下一组，最大 16)
  localparam int IBUF_SIZE  = 32;

  //--------------------------------------------------------------------------
  // D$ MSHR：15 个 (2026-10-14 由 8 个加到 15 个)。下游请求号 4 位：0..14 = D$ 各 MSHR，15 = I$
  // (150 拍内存延迟下 8 个 MSHR 把带宽卡在 8 行 / 150 拍，预取只会和正常缺失抢 MSHR)
  //--------------------------------------------------------------------------
  localparam int DC_NMSHR   = 15;
  localparam int MSHR_W     = 4;
  localparam logic [3:0] MEM_ID_IC = 4'hF;

  //--------------------------------------------------------------------------
  // 发射队列与端口
  //--------------------------------------------------------------------------
  // 访存流水 (2026-10-13 起)：N_LDP 条 load 流水 + 1 条 STA 流水 (STA / AMO 地址)；
  // 整数 STD 有 N_STDP 个专用口 (原来 STA 挤在第 2 条 load 流水、STD 挤在 P3)
`ifdef O6_N_LDP
  localparam int N_LDP  = `O6_N_LDP;      // 性能对比实验用
`else
  localparam int N_LDP  = 3;              // 2026-10-13 由 2 条加到 3 条 (与香山昆明湖相同)
`endif
  localparam int N_STA  = 2;             // STA 流水条数 (2026-10-14 由 1 条加到 2 条，同香山 / Neoverse V3)
  localparam int LSU_NP = N_LDP + N_STA; // LSU 内流水条数
  localparam int LSU_KS = N_LDP;         // 第一条 STA 流水在 LSU 内的编号
  localparam int N_STDP = 2;

  // 端口编号
  // 整数 N_ALU 个口 (2026-10-14 由 4 个加到 6 个)：P0..P5 都有 ALU
  localparam int N_ALU  = 6;
  localparam int P_ALU0 = 0;   // ALU + BRU                     (IQ_A)
  localparam int P_ALU1 = 1;   // ALU + BRU                     (IQ_A)
  localparam int P_ALU2 = 2;   // ALU + BRU + MUL               (IQ_B)
  localparam int P_ALU3 = 3;   // ALU + MUL + DIV + CSR + I2F   (IQ_B)
  localparam int P_ALU4 = 4;   // ALU                           (IQ_C)
  localparam int P_ALU5 = 5;   // ALU                           (IQ_C)
  localparam int P_LD0  = N_ALU;             // Load × N_LDP
  localparam int P_STA  = P_LD0 + N_LDP;     // STA / AMO 地址 × N_STA
  localparam int P_STD0 = P_STA + N_STA;     // 整数 STD × N_STDP
  localparam int P_FP0  = P_STD0 + N_STDP;   // FMA + FMISC + F2I
  localparam int P_FP1  = P_FP0 + 1;         // FMA + FDIV + FSTD
  localparam int N_PORTS = P_FP1 + 1;
  localparam int PORT_W  = 4;
  localparam int N_BRU  = 3;   // 分支单元：P0 / P1 / P2 (2026-10-13 由 2 个加到 3 个)
  localparam int N_FTQ_RD = N_BRU + 1;   // FTQ 读口：[0..N_BRU-1] 各 BRU 的 RF 级，[N_BRU] 提交控制

  // 发射队列：A (P0/P1)、B (P2/P3)、C (P4/P5 纯 ALU)、M (load)、F (浮点)、S (STA)、D (整数 STD)
  localparam int N_IQ       = 8;
  // IQ_N (2026-10-17，6 GHz)：原 64 项 IQ_M 拆成 IQ_M (32 项，load 口 0/1) + IQ_N (32 项，load 口 2)，
  // 唤醒-选择环在 2nm 下 64 项单拍放不下 (DC 不计连线 WNS -152 ps)
  localparam int IQ_A = 0, IQ_B = 1, IQ_M = 2, IQ_F = 3, IQ_S = 4, IQ_D = 5, IQ_C = 6, IQ_N = 7;
`ifdef O6_IQ_C
  localparam int IQ_C_SIZE  = `O6_IQ_C;
`else
  localparam int IQ_C_SIZE  = 48;      // 2026-10-14 32 -> 48
`endif
`ifdef O6_IQ_AB
  localparam int IQ_A_SIZE  = `O6_IQ_AB;
  localparam int IQ_B_SIZE  = `O6_IQ_AB;
`else
  localparam int IQ_A_SIZE  = 48;   // 24 -> 48 (2026-10-14，exp6/exp7：CoreMark 7.37 -> 7.74 合计)
  localparam int IQ_B_SIZE  = 48;
`endif
`ifdef O6_IQ_M
  localparam int IQ_M_SIZE  = `O6_IQ_M;
`else
  localparam int IQ_M_SIZE  = 32;      // 2026-10-14 48 -> 64；2026-10-17 拆成 IQ_M + IQ_N 各 32
`endif
  localparam int IQ_N_SIZE  = IQ_M_SIZE;
  localparam int IQ_F_SIZE  = 32;
`ifdef O6_IQ_SD
  localparam int IQ_S_SIZE  = `O6_IQ_SD;
  localparam int IQ_D_SIZE  = `O6_IQ_SD;
`else
  localparam int IQ_S_SIZE  = 32;   // 16 -> 32：CoreMark 里 STD 等长依赖链的数据，IQ_D 满 20 万拍
  localparam int IQ_D_SIZE  = 32;
`endif
  localparam int IQ_ENQ_MAX = 4;    // 每个队列每拍最多入队数

  // 整数 PRF 写口：W0..W(N_ALU-1) = P0..，W(N_ALU) = 长延迟共享口 (DIV / F2I / CSR / FP->INT)，其后各 load 流水
  localparam int IWB_LONG   = N_ALU;
  localparam int IWB_LD0    = N_ALU + 1;
  localparam int N_IWB      = N_ALU + 1 + N_LDP;
  // 浮点 PRF 写口：FW0/FW1 = 浮点两口，FW2 = I2F，FW3.. = 各 load 流水的 FLD
  localparam int N_FWB      = 3 + N_LDP;
  // 整数 PRF 读口：整数各口 2 个，LSU 各流水 1 个 (基址)，STD 口各 1 个
  localparam int N_IRD      = 2 * N_ALU + LSU_NP + N_STDP;
  localparam int N_FRD      = 7;
  // 唤醒：整数 [0..N_ALU-1] 各整数口 (含 MUL)，[N_ALU] 长延迟口，其后 load；浮点 [0..2] 浮点簇，[3..] load
  localparam int N_IWAKE    = N_ALU + 1 + N_LDP;
  localparam int N_FWAKE    = 3 + N_LDP;
  // 确认唤醒 (只含不会被取消的)：给重命名的 busy 表 / 队列，以及 IQ 入队口。
  //   整数各口的发射当拍唤醒若带"下一拍才判定"的毒化 (uop.poia)，不进确认流，晚一拍判定通过后由
  //   追加的 N_ALU 个口补发；load 的确认唤醒在 L2
  localparam int N_IWAKE_C  = N_IWAKE + N_ALU;
  localparam int N_CWAKE    = N_LDP + N_ALU;
  // ROB 完成口：[0..N_ALU-1] 整数各口，[N_ALU] 长延迟口，其后 LSU 各流水、3 个浮点簇；STD 完成另走 std_done
  localparam int N_CMPL     = N_ALU + 1 + LSU_NP + 3;
  localparam int N_STD      = N_STDP + 1;

  //--------------------------------------------------------------------------
  // 地址映射 (与 Spike / QEMU virt 一致)
  //--------------------------------------------------------------------------
  localparam logic [63:0] RESET_PC   = 64'h0000_0000_0000_1000;
  localparam logic [63:0] BOOTROM_BASE = 64'h0000_0000_0000_1000;
  localparam logic [63:0] CLINT_BASE = 64'h0000_0000_0200_0000;
  localparam logic [63:0] PLIC_BASE  = 64'h0000_0000_0C00_0000;
  localparam logic [63:0] UART_BASE  = 64'h0000_0000_1000_0000;
  localparam logic [63:0] DRAM_BASE  = 64'h0000_0000_8000_0000;

  //==========================================================================
  // 类型
  //==========================================================================
  typedef logic [PREG_W-1:0]   preg_t;
  typedef logic [MAX_BR-1:0]   br_mask_t;
  typedef logic [BR_TAG_W-1:0] br_tag_t;

  // ROB 指针：{翻转位, 行, 列}。比较新旧只看 {翻转位, 行, 列} 的环形顺序。
  typedef struct packed {
    logic                 flag;
    logic [ROB_ROW_W-1:0] row;
    logic [ROB_COL_W-1:0] col;
  } rob_ptr_t;

  typedef struct packed {
    logic             flag;
    logic [LQ_W-1:0]  idx;
  } lq_ptr_t;

  typedef struct packed {
    logic             flag;
    logic [SQ_W-1:0]  idx;
  } sq_ptr_t;

  typedef struct packed {
    logic             flag;
    logic [FTQ_W-1:0] idx;
  } ftq_ptr_t;

  // PMP 一项的预译码 (o6_pmp_dec 由 pmpcfg / pmpaddr 算出，o6_core 寄存一拍后分给各检查点)：
  //   TOR：区间 [lo, hi)；NAPOT (含 NA4)：((a ^ lo) & hi) == 0，hi 是掩码。地址都按 PA[55:2] 计
  typedef struct packed {
    logic        lk;          // L
    logic [2:0]  xwr;
    logic        tor, nap;
    logic [53:0] lo, hi;
  } pmp_dec_t;

  // 寄存器类
  typedef enum logic [1:0] {
    RC_NONE = 2'd0,
    RC_INT  = 2'd1,
    RC_FP   = 2'd2
  } rclass_e;

  // 功能单元类型
  typedef enum logic [3:0] {
    FU_ALU  = 4'd0,
    FU_BRU  = 4'd1,    // 条件分支 / JAL / JALR / AUIPC (需要 PC)
    FU_MUL  = 4'd2,
    FU_DIV  = 4'd3,
    FU_CSR  = 4'd4,    // CSR 读写与系统指令 (ECALL/EBREAK/xRET/WFI/FENCE*/SFENCE)，ROB 头执行
    FU_LD   = 4'd5,
    FU_ST   = 4'd6,    // 分派时拆 STA (IQ_M) + STD (IQ_B 或 IQ_F)
    FU_AMO  = 4'd7,    // LR / SC / AMO*，ROB 头执行
    FU_FMA  = 4'd8,    // FADD/FSUB/FMUL/FMADD 系
    FU_FMISC= 4'd9,    // FSGNJ / FMINMAX / FCMP / FCLASS / F2F 转换
    FU_F2I  = 4'd10,   // FCVT.[W|L].* / FMV.X.*，写整数寄存器
    FU_FDIV = 4'd11,   // FDIV / FSQRT
    FU_I2F  = 4'd12,   // FCVT.*.[W|L] / FMV.*.X，读整数写浮点
    FU_STD  = 4'd13,   // store 数据半边 (分派时由 FU_ST 拆出；FU_ST 在 IQ 里只表示 STA)
    FU_VEC  = 4'd14,   // 向量指令 (RVV)：ROB 头经 P3 交给向量单元 (o6_vu)，见 docs/11_vector_uarch.md
    FU_NONE = 4'd15    // 无需执行 (NOP 化的指令，或译码即异常)
  } fu_e;

  //--------------------------------------------------------------------------
  // 各 FU 的 op 编码 (5 位，按 fu 解释)
  //--------------------------------------------------------------------------
  // ALU
  localparam logic [4:0] ALU_ADD  = 5'd0;
  localparam logic [4:0] ALU_SUB  = 5'd1;
  localparam logic [4:0] ALU_SLL  = 5'd2;
  localparam logic [4:0] ALU_SLT  = 5'd3;
  localparam logic [4:0] ALU_SLTU = 5'd4;
  localparam logic [4:0] ALU_XOR  = 5'd5;
  localparam logic [4:0] ALU_SRL  = 5'd6;
  localparam logic [4:0] ALU_SRA  = 5'd7;
  localparam logic [4:0] ALU_OR   = 5'd8;
  localparam logic [4:0] ALU_AND  = 5'd9;
  localparam logic [4:0] ALU_LUI  = 5'd10;   // 结果 = imm
  // B 扩展 (Zba / Zbb / Zbs，2026-10-14)。Zba 的 shNadd / *.uw / slli.uw 不另占操作码：
  // 用 ALU_ADD / ALU_SLL 加操作数 A 预处理 (uop.auw = 先零扩展低 32 位，uop.ash = 再左移 0..3 位)
  localparam logic [4:0] ALU_ANDN = 5'd11;
  localparam logic [4:0] ALU_ORN  = 5'd12;
  localparam logic [4:0] ALU_XNOR = 5'd13;
  // 14 / 15 原为 ALU_CLZ / CTZ (clz / ctz / cpop 2026-10-26 挪到 o6_mul，见 MUL_CLZ)，2026-10-29 改给 Zicond；16 空
  localparam logic [4:0] ALU_CZEQZ = 5'd14;   // Zicond czero.eqz：rs2 == 0 ? 0 : rs1
  localparam logic [4:0] ALU_CZNEZ = 5'd15;   // Zicond czero.nez：rs2 != 0 ? 0 : rs1
  localparam logic [4:0] ALU_MAX  = 5'd17;
  localparam logic [4:0] ALU_MAXU = 5'd18;
  localparam logic [4:0] ALU_MIN  = 5'd19;
  localparam logic [4:0] ALU_MINU = 5'd20;
  localparam logic [4:0] ALU_SEXTB = 5'd21;
  localparam logic [4:0] ALU_SEXTH = 5'd22;
  localparam logic [4:0] ALU_ZEXTH = 5'd23;
  localparam logic [4:0] ALU_ROL  = 5'd24;    // is_w：rolw
  localparam logic [4:0] ALU_ROR  = 5'd25;    // is_w：rorw / roriw；立即数时 b = imm
  localparam logic [4:0] ALU_ORCB = 5'd26;
  localparam logic [4:0] ALU_REV8 = 5'd27;
  localparam logic [4:0] ALU_BCLR = 5'd28;    // 位号 = b[5:0] (立即数形式 b = imm)
  localparam logic [4:0] ALU_BEXT = 5'd29;
  localparam logic [4:0] ALU_BINV = 5'd30;
  localparam logic [4:0] ALU_BSET = 5'd31;
  // BRU
  localparam logic [4:0] BRU_BEQ  = 5'd0;
  localparam logic [4:0] BRU_BNE  = 5'd1;
  localparam logic [4:0] BRU_BLT  = 5'd4;
  localparam logic [4:0] BRU_BGE  = 5'd5;
  localparam logic [4:0] BRU_BLTU = 5'd6;
  localparam logic [4:0] BRU_BGEU = 5'd7;
  localparam logic [4:0] BRU_JAL  = 5'd8;
  localparam logic [4:0] BRU_JALR = 5'd9;
  localparam logic [4:0] BRU_AUIPC= 5'd10;
  // MUL
  localparam logic [4:0] MUL_MUL    = 5'd0;
  localparam logic [4:0] MUL_MULH   = 5'd1;
  localparam logic [4:0] MUL_MULHSU = 5'd2;
  localparam logic [4:0] MUL_MULHU  = 5'd3;
  // Zbb 计数 (clz / ctz / cpop，is_w 为 *w 形式)：2026-10-26 从 ALU 挪到 MUL 流水 (4 拍)，
  // 6 GHz 下 ALU 带计数 155 ps、去掉后 150 ps (旁路 57 ps 到达)；13 个基准里静态出现 0 次
  localparam logic [4:0] MUL_CLZ    = 5'd4;
  localparam logic [4:0] MUL_CTZ    = 5'd5;
  localparam logic [4:0] MUL_CPOP   = 5'd6;
  // DIV
  localparam logic [4:0] DIV_DIV  = 5'd0;
  localparam logic [4:0] DIV_DIVU = 5'd1;
  localparam logic [4:0] DIV_REM  = 5'd2;
  localparam logic [4:0] DIV_REMU = 5'd3;
  // CSR / 系统
  localparam logic [4:0] CSR_RW     = 5'd1;
  localparam logic [4:0] CSR_RS     = 5'd2;
  localparam logic [4:0] CSR_RC     = 5'd3;
  localparam logic [4:0] CSR_RWI    = 5'd5;
  localparam logic [4:0] CSR_RSI    = 5'd6;
  localparam logic [4:0] CSR_RCI    = 5'd7;
  localparam logic [4:0] SYS_ECALL  = 5'd8;
  localparam logic [4:0] SYS_EBREAK = 5'd9;
  localparam logic [4:0] SYS_MRET   = 5'd10;
  localparam logic [4:0] SYS_SRET   = 5'd11;
  localparam logic [4:0] SYS_WFI    = 5'd12;
  localparam logic [4:0] SYS_FENCE  = 5'd13;
  localparam logic [4:0] SYS_FENCEI = 5'd14;
  localparam logic [4:0] SYS_SFENCE = 5'd15;
  localparam logic [4:0] SYS_DRET   = 5'd16;
  localparam logic [4:0] SYS_WRS    = 5'd17;   // Zawrs wrs.nto：按"立即结束"实现，只做 TW 检查 (TW=1 且非 M 态 -> 非法)
  localparam logic [4:0] SYS_HFENCEV = 5'd19;  // H：hfence.vvma / hinval.vvma (清 TLB，同 SFENCE 的流程)
  localparam logic [4:0] SYS_HFENCEG = 5'd20;  // H：hfence.gvma / hinval.gvma
  localparam logic [4:0] SYS_INVF   = 5'd18;   // Svinval sfence.w.inval / sfence.inval.ir：sinval.vma 已按 sfence.vma 做，这两条只剩特权检查 (U 态非法)

  // 调试 (Debug 0.13.2，执行式)：调试模块存储区 0x000-0xFFF，只在调试模式可取指 / 访问
  localparam logic [63:0] DBG_ENTRY  = 64'h800;      // 进入调试模式 / 调试模式下 ebreak
  localparam logic [63:0] DBG_EXCP   = 64'h808;      // 调试模式下发生异常
  localparam logic [5:0]  IRQ_DBG_HALT = 6'h3F;      // 借中断通路进入调试模式的伪原因号
  localparam logic [5:0]  IRQ_DBG_STEP = 6'h3E;
  // LD / ST：op[1:0] = log2(字节数)，op[2] = 无符号 (LD)
  // AMO：op = 下面编号，size 放在 mem_sz
  localparam logic [4:0] AMO_LR   = 5'd0;
  localparam logic [4:0] AMO_SC   = 5'd1;
  localparam logic [4:0] AMO_SWAP = 5'd2;
  localparam logic [4:0] AMO_ADD  = 5'd3;
  localparam logic [4:0] AMO_XOR  = 5'd4;
  localparam logic [4:0] AMO_AND  = 5'd5;
  localparam logic [4:0] AMO_OR   = 5'd6;
  localparam logic [4:0] AMO_MIN  = 5'd7;
  localparam logic [4:0] AMO_MAX  = 5'd8;
  localparam logic [4:0] AMO_MINU = 5'd9;
  localparam logic [4:0] AMO_MAXU = 5'd10;
  // Zicbom / Zicboz (RVA23 R2，docs/14)：缓存块操作按 AMO 走 (STA 翻译 / 权限，ROB 头 + SQ 排空后由 LSU 头状态机发 D$)
  //   CBOZ：cbo.zero，store 权限，D$ 整行写全 0；CBOC：cbo.clean；CBOF：cbo.flush 与 cbo.inval (按 flush 做：
  //   Spike 没有缓存，真丢弃脏数据会与它的存储不一致；inval 做成 flush 只是多一次写回)。CBOC / CBOF 按 load 查权限、报 store 类异常
  localparam logic [4:0] AMO_CBOZ = 5'd11;
  localparam logic [4:0] AMO_CBOC = 5'd12;
  localparam logic [4:0] AMO_CBOF = 5'd13;
  function automatic logic amo_is_cbo(input logic [4:0] op);
    return op == AMO_CBOZ || op == AMO_CBOC || op == AMO_CBOF;
  endfunction
  function automatic logic amo_nodata(input logic [4:0] op);     // 没有 store 数据 (不进 IQ_D)
    return op == AMO_LR || amo_is_cbo(op);
  endfunction
  // H 扩展：FU_LD / FU_ST 的 op 位 (普通访存 op = 0)：HLV / HSV 用 V = 1 的翻译 (hstatus.SPVP 的特权级)，HLVX 要执行权限
  localparam logic [4:0] LS_HV   = 5'd1;
  localparam logic [4:0] LS_HLVX = 5'd3;

  // 翻译上下文 (o6_csr 算好：取指一份、普通数据访存一份、HLV / HSV 一份；LSU 按 uop 选)
  typedef struct packed {
    logic       v;       // 有效 V：TLB 项标记、PTW 用 vsatp / hgatp
    logic       xl;      // 需要翻译 (V = 1 时 vsatp 或 hgatp 任一非 Bare)
    logic       cn;      // 一级 (satp / vsatp) 翻译开：规范地址检查 + 一级权限检查 (vsatp = Bare 只有 G-stage 时为 0)
    logic       gmxr;    // G-stage 的 MXR (只看 mstatus.MXR)
    logic [1:0] priv;    // 页权限 / PMP 用的特权级 (U / S；M 时 xl = 0)
    logic       sum;     // V = 1 时是 vsstatus.SUM
    logic       mxr;     // V = 1 时 = mstatus.MXR | vsstatus.MXR (VS-stage)
    logic [1:0] vam;     // 规范地址检查的模式 (satp / vsatp 的 MODE[1:0])
    logic [1:0] pmm;     // 指针掩码 (已按 Spike get_pmlen 选好)
    logic       zx;      // 指针掩码零扩展 (有效特权 M 或 (vs)satp = Bare)
  } xctx_t;

  // 规范虚地址：va[63:VA-1] 全同 (VA = 39 / 48 / 57，按 vam)
  function automatic logic va_canon(input logic [63:0] va, input logic [1:0] vam);
    unique case (vam)
      2'd1:    return (&va[63:47]) || !(|va[63:47]);
      2'd2:    return (&va[63:56]) || !(|va[63:56]);
      default: return (&va[63:38]) || !(|va[63:38]);
    endcase
  endfunction

  // 指针掩码 (Smnpm / Ssnpm / Supm，RVA23 R4c)：数据访存的有效地址高 PMLEN 位换成 bit (63 - PMLEN) 的符号扩展
  //   (翻译开) 或零扩展 (翻译关)；pmm 同 xenvcfg.PMM：2 = PMLEN 7，3 = PMLEN 16，其余不掩
  function automatic logic [63:0] pm_x(input logic [63:0] va, input logic [1:0] pmm, input logic zx);
    unique case (pmm)
      2'd2:    return {{7{va[56] & !zx}}, va[56:0]};
      2'd3:    return {{16{va[47] & !zx}}, va[47:0]};
      default: return va;
    endcase
  endfunction

  function automatic logic amo_rdperm(input logic [4:0] op);     // 按 load 查页 / PMP 权限
    return op == AMO_LR || op == AMO_CBOC || op == AMO_CBOF;
  endfunction
  // FMA
  localparam logic [4:0] FMA_ADD   = 5'd0;
  localparam logic [4:0] FMA_SUB   = 5'd1;
  localparam logic [4:0] FMA_MUL   = 5'd2;
  localparam logic [4:0] FMA_MADD  = 5'd3;
  localparam logic [4:0] FMA_MSUB  = 5'd4;
  localparam logic [4:0] FMA_NMSUB = 5'd5;
  localparam logic [4:0] FMA_NMADD = 5'd6;
  // FMISC
  localparam logic [4:0] FMISC_SGNJ  = 5'd0;
  localparam logic [4:0] FMISC_SGNJN = 5'd1;
  localparam logic [4:0] FMISC_SGNJX = 5'd2;
  localparam logic [4:0] FMISC_MIN   = 5'd3;
  localparam logic [4:0] FMISC_MAX   = 5'd4;
  localparam logic [4:0] FMISC_CVTFF = 5'd5;   // FCVT.S.D / FCVT.D.S
  // Zfhmin / Zfbfmin：半精度 / BF16 只有转换 (目的 <- 源)
  localparam logic [4:0] FMISC_CVTSH = 5'd6;   // FCVT.S.H
  localparam logic [4:0] FMISC_CVTHS = 5'd7;   // FCVT.H.S
  localparam logic [4:0] FMISC_CVTDH = 5'd8;   // FCVT.D.H
  localparam logic [4:0] FMISC_CVTHD = 5'd9;   // FCVT.H.D
  localparam logic [4:0] FMISC_CVTSB = 5'd10;  // FCVT.S.BF16
  localparam logic [4:0] FMISC_CVTBS = 5'd11;  // FCVT.BF16.S
  // Zfa (不进 fpnew，o6_zfa 旁路；fmt 取 raw[26:25])
  localparam logic [4:0] FMISC_FLI     = 5'd12;  // fli.{h,s,d}：常数下标 = raw[19:15]
  localparam logic [4:0] FMISC_MINM    = 5'd13;
  localparam logic [4:0] FMISC_MAXM    = 5'd14;
  localparam logic [4:0] FMISC_ROUND   = 5'd15;
  localparam logic [4:0] FMISC_ROUNDNX = 5'd16;
  // F2I (写整数)
  localparam logic [4:0] F2I_EQ    = 5'd0;
  localparam logic [4:0] F2I_LT    = 5'd1;
  localparam logic [4:0] F2I_LE    = 5'd2;
  localparam logic [4:0] F2I_CLASS = 5'd3;
  localparam logic [4:0] F2I_MV    = 5'd4;
  localparam logic [4:0] F2I_CVTW  = 5'd5;
  localparam logic [4:0] F2I_CVTWU = 5'd6;
  localparam logic [4:0] F2I_CVTL  = 5'd7;
  localparam logic [4:0] F2I_CVTLU = 5'd8;
  localparam logic [4:0] F2I_MVH   = 5'd9;    // FMV.X.H (低 16 位符号扩展)
  localparam logic [4:0] F2I_CVTMOD = 5'd10;  // Zfa fcvtmod.w.d
  localparam logic [4:0] F2I_LEQ    = 5'd11;  // Zfa fleq
  localparam logic [4:0] F2I_LTQ    = 5'd12;  // Zfa fltq
  // FDIV
  localparam logic [4:0] FDIV_DIV  = 5'd0;
  localparam logic [4:0] FDIV_SQRT = 5'd1;
  // I2F
  localparam logic [4:0] I2F_MV    = 5'd0;
  localparam logic [4:0] I2F_CVTW  = 5'd1;
  localparam logic [4:0] I2F_CVTWU = 5'd2;
  localparam logic [4:0] I2F_CVTL  = 5'd3;
  localparam logic [4:0] I2F_CVTLU = 5'd4;
  localparam logic [4:0] I2F_MVH   = 5'd5;    // FMV.H.X (上 48 位补 1)

  //--------------------------------------------------------------------------
  // 取指 -> 译码 的单条指令
  //--------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0]          inst;      // 已展开的 32 位指令 (RVC 已解压)
    logic [31:0]          raw;       // 原始编码 (RVC 只有低 16 位)，非法指令的 tval 用它
    logic                 is_rvc;
    logic                 rvc_ill;   // RVC 解压发现非法编码
    logic [FTQ_W-1:0]     ftq_idx;
    logic [FTQ_OFF_W-1:0] ftq_off;
    logic                 pred_taken;   // 前端预测本条是 taken 的控制流 (取指块终点)
    logic                 ld_wait;      // 访存相关预测：load 曾越过未知地址的 store 违例，要等它们地址确定
    logic                 fetch_excp;   // 取指异常 (页错误 / 访问错误)
    logic [5:0]           fetch_cause;
    logic                 fetch_hi;     // 异常发生在跨页 RVI 的高半部分 (tval = pc+2)
    logic [7:0]           cls;          // 截组用的资源类别 {br, A, B, M, S, D, F, ALU} (F3 按操作码预分，只多不少)
    logic [11:0]          pch;          // PC[12:1] (数据预取按 load 的 PC 建表)
  } fetch_inst_t;

  //--------------------------------------------------------------------------
  // 译码后 / 重命名后的 uop
  //--------------------------------------------------------------------------
  typedef struct packed {
    // 译码
    fu_e                  fu;
    logic [4:0]           op;
    logic                 is_w;        // RV64 的 *W 运算 (32 位结果符号扩展)
    logic                 auw;         // Zba：ALU 操作数 A 先零扩展低 32 位 (add.uw / shNadd.uw / slli.uw)
    logic [1:0]           ash;         // Zba：ALU 操作数 A 再左移 0..3 位 (shNadd)
    logic [1:0]           mem_sz;      // 访存 / AMO 字节数 log2
    logic                 mem_unsigned;
    logic                 aq, rl;
    logic [2:0]           rm;          // 浮点舍入 (7 = 动态)
    logic                 fp_dbl;      // 浮点格式：1 = D，0 = S
    logic                 use_imm;     // 第二操作数用 imm
    logic [31:0]          imm;         // 已符号扩展到 32 位 (LUI/AUIPC 已左移 12)
    logic [11:0]          csr_addr;
    rclass_e              rd_class;
    rclass_e              rs1_class;
    rclass_e              rs2_class;
    logic                 rs3_en;      // FMA 第三源 (总是浮点)
    logic [4:0]           lrd, lrs1, lrs2, lrs3;
    logic                 wait_head;   // 只在 ROB 头执行 (CSR / AMO / FENCE / 系统指令)
    logic                 flush_after; // 提交后冲刷重取 (CSR 写 / FENCE.I / SFENCE.VMA / xRET)
    logic                 is_br;       // 需要分支标签 (条件分支 / JALR)
    logic                 excp;        // 译码 / 取指期异常
    logic [5:0]           cause;
    logic                 tval_hi;     // 取指异常发生在跨页 RVI 的高半部分 (tval = pc+2)
    logic [31:0]          raw;
    logic                 is_rvc;
    logic [FTQ_W-1:0]     ftq_idx;
    logic [FTQ_OFF_W-1:0] ftq_off;
    logic                 pred_taken;
    logic                 ld_wait;     // 见 fetch_inst_t
    logic [11:0]          pch;         // PC[12:1] (见 fetch_inst_t)
    // move 消除 (2026-10-14)：mv rd, rs (ADDI rd, rs, 0 / ADD rd, rs, x0 / ADD rd, x0, rs，含 c.mv / c.li rd, 0)
    //   mvc：译码器认出的候选，mvsrc：源是 rs2 (否则 rs1)；mvel：译码级确定消除 (fu 已改成 FU_NONE)，
    //   重命名把 rd 直接映射到源的物理寄存器，不分配新的，不进发射队列，分派即完成
    logic                 mvc, mvsrc, mvel;
    // 重命名
    preg_t                pdst, psrc1, psrc2, psrc3, old_pdst;
    logic                 src1_busy, src2_busy, src3_busy;
    br_tag_t              br_tag;
    br_mask_t             br_mask;
    // 分派
    rob_ptr_t             rob_idx;
    lq_ptr_t              lq_idx;      // load：自己的 LQ 项；store：分派时的 LQ 尾 (比它新的 load 从这里开始)
    sq_ptr_t              sq_idx;      // store：自己的 SQ 项；load：分派时的 SQ 尾 (比它老的 store 在这之前)
    logic [PORT_W-1:0]    port;
    // 发射时：源操作数取决于 load 流水 k 本拍 (pois) / 下一拍 (poia) 的 L2 判定。
    // 判定失败则执行流水在 RF -> 下一级处丢弃它 (pois 用寄存的失败信号，poia 用当拍的)，IQ 保留该项重发
    logic [N_LDP-1:0]     pois;
    logic [N_LDP-1:0]     poia;
    logic [N_LDP-1:0]     poib;        // 判定在两拍后 (load -> load 快速路径的消费者，只在 IQ_M / LSU)
  } uop_t;

  //--------------------------------------------------------------------------
  // 分支解析广播 (EX 解析后的下一拍发出，所有持有 uop 的结构都要听)
  //--------------------------------------------------------------------------
  typedef struct packed {
    br_mask_t   resolve_mask;   // 本拍解析完成的标签 (含误预测的那个)：所有 br_mask 清掉这些位
    logic       kill;           // 有误预测
    br_tag_t    kill_tag;       // 误预测分支的标签：br_mask 含此位的 uop 一律作废
    rob_ptr_t   kill_rob;       // 误预测分支的 ROB 位置 (ROB 尾回滚到它之后)
    lq_ptr_t    kill_lq;        // 误预测分支分派时的 LQ 尾 (= 分支 uop 的 lq_idx)，LQ 尾恢复到这里
    sq_ptr_t    kill_sq;        // 同上，SQ
  } br_update_t;

  //--------------------------------------------------------------------------
  // 唤醒广播
  //--------------------------------------------------------------------------
  // load 投机唤醒：load 在 L0 就按"会命中"唤醒，L2 判定失败 (缺失 / 等 store / nack ...) 时取消。
  // (2026-10-13 在 L1 唤醒、窗口 1 拍；2026-10-14 提前到 L0，load-to-use 5 -> 4 拍，窗口 2 拍)
  //   sa[k]：本唤醒取决于 load 流水 k 两拍后的 L2 判定 (load 自己在 L0 发的)
  //   sn[k]：本唤醒取决于 load 流水 k 下一拍的 L2 判定 (带 poia 发射的 uop 的发射当拍唤醒)
  //   sc[k]：本唤醒取决于 load 流水 k 本拍的 L2 判定 (带 pois 发射的 uop 的发射当拍唤醒)：本拍失败则作废
  // 判定之后毒化标记就清掉，不会一级级传下去
  typedef struct packed {
    logic   valid;
    logic   is_fp;
    preg_t  pdst;
    logic [N_LDP-1:0] sa;
    logic [N_LDP-1:0] sn;
    logic [N_LDP-1:0] sc;
  } wakeup_t;

  // load -> load 快速唤醒 (2026-10-14)：对齐的 64 位 ld 在 AG 级只唤醒 IQ_M (消费者以它为基址)，
  // 消费者 AG 级直接从生产者的 L2 数据取基址：指针追逐 4 -> 3 拍。
  //   sb[k]：判定在三拍后 (load 流水 k 的 L2)；sc[k]：生产者自己带 poib、本拍判定，失败则作废
  typedef struct packed {
    logic             valid;
    preg_t            pdst;
    logic [N_LDP-1:0] sb;
    logic [N_LDP-1:0] sc;
  } lwake_t;

  //--------------------------------------------------------------------------
  // 写回 (写 PRF + 标记 ROB 完成)
  //--------------------------------------------------------------------------
  typedef struct packed {
    logic             valid;
    preg_t            pdst;
    logic             wen;      // 是否写 PRF (store/分支等不写)
    logic [XLEN-1:0]  data;
  } wb_t;

  // 完成 (写 ROB)
  typedef struct packed {
    logic             valid;
    rob_ptr_t         rob_idx;
    logic             excp;
    logic [5:0]       cause;
    logic [63:0]      tval;
    logic             flush_after;   // 提交后冲刷 (从下一条重取)
    logic             replay;        // 到 ROB 头时不提交，冲刷后从它自己重取 (TLB 缺失)
    logic [4:0]       fflags;
  } cmpl_t;

  //==========================================================================
  // 函数
  //==========================================================================

  // a 是否比 b 老 (a 在程序序上先于 b)
  function automatic logic rob_older(input rob_ptr_t a, input rob_ptr_t b);
    logic [ROB_ROW_W+ROB_COL_W-1:0] ia, ib;
    ia = {a.row, a.col};
    ib = {b.row, b.col};
    return (a.flag == b.flag) ? (ia < ib) : (ia > ib);
  endfunction

  // 标志位不同、下标相同 = 正好差一整圈 (队列满)，a 比 b 老一整圈：必须算"更老"。
  // 原来写成 a.idx > b.idx，队列正好满时判错：冲刷把已提交未排空的最老 store 作废 (Linux 启动时
  // memset 让 SQ 被 32 条已提交 store 占满，随即死锁)；SQ 满时发射的 load 转发搜索也会漏掉最老的 store
  function automatic logic lq_older(input lq_ptr_t a, input lq_ptr_t b);
    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx >= b.idx);
  endfunction

  function automatic logic sq_older(input sq_ptr_t a, input sq_ptr_t b);
    return (a.flag == b.flag) ? (a.idx < b.idx) : (a.idx >= b.idx);
  endfunction

  // 分支掩码更新
  function automatic logic br_killed(input br_mask_t m, input br_update_t bu);
    return bu.kill && m[bu.kill_tag];
  endfunction

  function automatic br_mask_t br_mask_upd(input br_mask_t m, input br_update_t bu);
    return m & ~bu.resolve_mask;
  endfunction

  //--------------------------------------------------------------------------
  // 分派一组需要的资源：各发射队列入队数、LQ / SQ 项数。只取决于组里各条的类型，
  // 在重命名级提前算好随组寄存，分派级就只剩"寄存值 <= 空位"的比较
  //--------------------------------------------------------------------------
  // 入队路由也只取决于组成分：sel[q][k] 是第 q 个队列第 k 个入队口取哪一半 uop 的独热码
  // (位 s = 第 s 条的主项，位 W+s = 第 s 条的数据半边 STD)；lqo / sqo 是组内 LQ / SQ 偏移。
  // 端口不在分派时定 (发射队列发射时动态选端口)，uop.port 分派时填 0
  typedef struct packed {
    logic [N_IQ-1:0][2:0] iqn;
    logic [2:0]           nld, nst;
    logic [N_IQ-1:0][IQ_ENQ_MAX-1:0][2*W-1:0] sel;
    logic [W-1:0][2:0]    lqo, sqo;
    logic [W-1:0][1:0]    aq;            // 第 s 条 ALU / BRU 分到哪个整数队列 (0 = A，1 = B，2 = C)
    logic [W-1:0]         mq;            // 第 s 条 load 分到哪个 load 队列 (0 = IQ_M，1 = IQ_N)
  } ds_need_t;

  // 与 o6_dispatch 的参考模型一致：MUL/DIV/CSR/I2F 固定进 B；
  // ALU 在 A / B / C 三个整数队列间、BRU 在 A / B 间按占用分流：进"已有占用 + 本组已分到的"最小的一侧
  // (相等时 A 先于 B 先于 C)，每队每组不超过 IQ_ENQ_MAX。
  // (原来 ALU 先填满 A 才分到 B：整数代码几乎全挤进 IQ_A 的两个端口，CoreMark 有 38% 的拍分派卡在 IQ_A 满。
  //  BRU 原来固定进 A；P2 加了分支单元后 B 也能执行，2026-10-13 起一起分流；2026-10-14 加 IQ_C 两个纯 ALU 口)
  // fa / fb / fc：IQ_A / IQ_B / IQ_C 的寄存空位数 (保守值，晚一拍无妨：分派时还会再比较一次)
  function automatic logic fixed_b(input uop_t u);
    return u.fu == FU_MUL || u.fu == FU_DIV || u.fu == FU_CSR || u.fu == FU_I2F || u.fu == FU_VEC;
  endfunction

  // 组内计数 (DC 2018 不支持 $countones)
  function automatic logic [2:0] popc_w(input logic [W-1:0] x);
    logic [2:0] c;
    c = '0;
    for (int i = 0; i < W; i++) c = c + 3'(x[i]);
    return c;
  endfunction

  // grp_need 的单条决策：状态 (a, b, c) 下一条 ALU / BRU 去哪个队列，返回 {C, B, A} 独热
  function automatic logic [2:0] gn_dec(input logic is_alu, input int a, input int b, input int c,
                                         input logic [6:0] lt_b, input logic [12:0] le_ab,
                                         input logic [12:0] le_ac, input logic [12:0] le_bc);
    logic ka, kb, kc, ga, gb;
    ka = a < IQ_ENQ_MAX; kb = lt_b[b]; kc = is_alu && c < IQ_ENQ_MAX;
    ga = ka && (!kb || le_ab[b - a + 6]) && (!kc || le_ac[c - a + 6]);
    gb = !ga && kb && (!kc || le_bc[c - b + 6]);
    return {!ga && !gb, gb, ga};
  endfunction
  // load 的单条决策：状态 (m, n) 下去 IQ_M 为 1
  function automatic logic gn_ldec(input int m, input int n, input logic [6:0] lt_m, input logic [6:0] lt_n,
                                   input logic [18:0] le_mn);
    logic km, kn;
    km = m < IQ_ENQ_MAX && lt_m[m]; kn = n < IQ_ENQ_MAX && lt_n[n];
    return km && (!kn || le_mn[2 * n - m + 6]);
  endfunction

  // 一组的分类 (2026-10-25，6 GHz)：只取决于各条的有效位与 fu 等，rename 在 uop 写进 rq0 时就算好寄存，
  // grp_need 不再在本拍做 "fu 译码 -> 固定进 B 的条数 nbf" (b180 rename：rq0_u.fu -> ... -> out_need 212 ps)
  typedef struct packed {
    logic [W-1:0] ab, alu, ld, fxb, sto, dq, fq, f0, f1;   // f0：主项进 IQ_F；f1：数据半边进 IQ_F (浮点 store)
    logic [2:0]   nbf;
  } gn_cls_t;
  function automatic gn_cls_t gn_cls(input logic [W-1:0] v, input uop_t [W-1:0] u);
    gn_cls_t c;
    for (int s = 0; s < W; s++) begin
      c.ab[s]  = v[s] && (u[s].fu == FU_ALU || u[s].fu == FU_BRU);
      c.alu[s] = v[s] && u[s].fu == FU_ALU;
      c.ld[s]  = v[s] && u[s].fu == FU_LD;
      c.fxb[s] = v[s] && fixed_b(u[s]);
      c.sto[s] = v[s] && (u[s].fu == FU_ST || u[s].fu == FU_AMO);
      c.dq[s]  = v[s] && ((u[s].fu == FU_ST && u[s].rs2_class != RC_FP) || (u[s].fu == FU_AMO && !amo_nodata(u[s].op)));
      c.f1[s]  = v[s] && u[s].fu == FU_ST && u[s].rs2_class == RC_FP;
      c.f0[s]  = v[s] && (u[s].fu == FU_FMA || u[s].fu == FU_FMISC || u[s].fu == FU_F2I || u[s].fu == FU_FDIV);
      c.fq[s]  = c.f0[s] || c.f1[s];
    end
    c.nbf = popc_w(c.fxb);
    return c;
  endfunction

  // 在途组需求的「可选择」形式 (2026-10-28，6 GHz)：grp_need 下一拍的比较位要用在途组的 ta - tb 等差值，
  // 原来把 ta / tb 当数值并进多操作数加法再比较 (out_need.iqn -> 比较位 ~85 ps，整个环 177 ps)。
  // 改成在上一拍末尾由最终独热状态直接或出各差值的独热码 (与 iqn 编码同深度)，下一拍只做"常数比较向量按独热选位"。
  //   各差值都是"在途组对比较式的贡献" (该组固定进 B 的 nbf 已并进去，nbf 早到，按它选不同的或树)：
  //   ab：ta - tb = a - b - nbf (下标 d + 12，d ∈ -12..6)；ac：ta - tc = a - c (下标 d + 6，d ∈ -6..6)；
  //   bc：tb - tc = b - c + nbf (下标 d + 6，d ∈ -6..12)；mn：tm - 2tn = m - 2n (下标 d + 12，d ∈ -12..6)；m / n：tm、tn 的独热
  typedef struct packed {
    logic [18:0] ab;
    logic [12:0] ac;
    logic [18:0] bc;
    logic [18:0] mn;
    logic [6:0]  m, n;
  } gn_inf_t;
  localparam gn_inf_t GN_INF_NONE = '{ab: 19'd1 << 12, ac: 13'd1 << 6, bc: 19'd1 << 6, mn: 19'd1 << 12,
                                      m: 7'd1, n: 7'd1};   // 没有在途组 (各差值为 0)
  typedef struct packed {
    ds_need_t need;
    gn_inf_t  inf;
  } gn_res_t;

  // grp_need 的核心：给定比较位后的分流、计数与入队路由 (need)，以及本组作为下一拍在途组时的独热差值 (inf)
  function automatic gn_res_t grp_need_core(input gn_cls_t cl,
                                             input logic [12:0] le_ab, input logic [12:0] le_ac,
                                             input logic [12:0] le_bc, input logic [18:0] le_mn,
                                             input logic [6:0] lt_m, input logic [6:0] lt_n);
    localparam int NS = 7;
    ds_need_t r;
    gn_inf_t  inf;
    logic [W-1:0] ab, alu, ld, fxb, sto, dq, fq, toa, tob, toc, ton;
    logic [2:0]   nbf;
    logic [NS-1:0] lt_b;
    logic         st  [NS][NS][NS];
    logic         stl [NS][NS];
    r = '0;
    ab = cl.ab; alu = cl.alu; ld = cl.ld; fxb = cl.fxb; sto = cl.sto; dq = cl.dq; fq = cl.fq; nbf = cl.nbf;
    for (int k = 0; k < NS; k++) lt_b[k] = k + int'(nbf) < IQ_ENQ_MAX;
    // 2. 整数分流：两条一步 (radix-2)。第二条在"第一条去 A / B / C / 不分流"四种情况下的去向都按静态下标
    //    预先算好，再按第一条的结果四选一；状态链 6 步缩为 3 步
    for (int a = 0; a < NS; a++) for (int b = 0; b < NS; b++) for (int c = 0; c < NS; c++) st[a][b][c] = 1'b0;
    st[0][0][0] = 1'b1;
    toa = '0; tob = '0; toc = '0;
    for (int s0 = 0; s0 < W; s0 += 2) begin
      logic nx [NS][NS][NS];
      int   s1;
      s1 = s0 + 1;
      for (int a = 0; a < NS; a++) for (int b = 0; b < NS; b++) for (int c = 0; c < NS; c++) nx[a][b][c] = 1'b0;
      for (int a = 0; a < NS; a++) for (int b = 0; b < NS; b++) for (int c = 0; c < NS; c++) begin
        logic [2:0] g0;                 // 第一条的去向 {C, B, A} (独热；不分流时全 0)
        logic [3:0] o0;                 // 第一条的结果 {不分流, C, B, A}
        g0 = ab[s0] ? gn_dec(alu[s0], a, b, c, lt_b, le_ab, le_ac, le_bc) : 3'b000;
        o0 = {!ab[s0], g0};
        toa[s0] = toa[s0] | (st[a][b][c] & g0[0]);
        tob[s0] = tob[s0] | (st[a][b][c] & g0[1]);
        toc[s0] = toc[s0] | (st[a][b][c] & g0[2]);
        for (int k = 0; k < 4; k++) begin
          int a1, b1, c1;
          logic [2:0] g1;
          a1 = a + (k == 0); b1 = b + (k == 1); c1 = c + (k == 2);
          if (a1 < NS && b1 < NS && c1 < NS) begin
            logic w;
            w  = st[a][b][c] & o0[k];
            g1 = ab[s1] ? gn_dec(alu[s1], a1, b1, c1, lt_b, le_ab, le_ac, le_bc) : 3'b000;
            toa[s1] = toa[s1] | (w & g1[0]);
            tob[s1] = tob[s1] | (w & g1[1]);
            toc[s1] = toc[s1] | (w & g1[2]);
            if (!ab[s1])                                      nx[a1][b1][c1]     = nx[a1][b1][c1]     | w;
            if (a1 < NS - 1)                                  nx[a1 + 1][b1][c1] = nx[a1 + 1][b1][c1] | (w & g1[0]);
            if (b1 < NS - 1)                                  nx[a1][b1 + 1][c1] = nx[a1][b1 + 1][c1] | (w & g1[1]);
            if (c1 < NS - 1)                                  nx[a1][b1][c1 + 1] = nx[a1][b1][c1 + 1] | (w & g1[2]);
          end
        end
      end
      st = nx;
    end
    for (int s = 0; s < W; s++) r.aq[s] = {toc[s], tob[s]};
    // 2'. load 分流 (同样两条一步)
    for (int m = 0; m < NS; m++) for (int n = 0; n < NS; n++) stl[m][n] = 1'b0;
    stl[0][0] = 1'b1;
    ton = '0;
    for (int s0 = 0; s0 < W; s0 += 2) begin
      logic nx [NS][NS];
      int   s1;
      s1 = s0 + 1;
      for (int m = 0; m < NS; m++) for (int n = 0; n < NS; n++) nx[m][n] = 1'b0;
      for (int m = 0; m < NS; m++) for (int n = 0; n < NS; n++) begin
        logic [2:0] o0;                 // {不分流, N, M}
        logic       gm0;
        gm0 = gn_ldec(m, n, lt_m, lt_n, le_mn);
        o0  = {!ld[s0], ld[s0] & !gm0, ld[s0] & gm0};
        ton[s0] = ton[s0] | (stl[m][n] & o0[1]);
        for (int k = 0; k < 3; k++) begin
          int m1, n1;
          m1 = m + (k == 0); n1 = n + (k == 1);
          if (m1 < NS && n1 < NS) begin
            logic w, gm1;
            w   = stl[m][n] & o0[k];
            gm1 = gn_ldec(m1, n1, lt_m, lt_n, le_mn);
            ton[s1] = ton[s1] | (w & ld[s1] & !gm1);
            if (!ld[s1])     nx[m1][n1]     = nx[m1][n1]     | w;
            if (m1 < NS - 1) nx[m1 + 1][n1] = nx[m1 + 1][n1] | (w & ld[s1] & gm1);
            if (n1 < NS - 1) nx[m1][n1 + 1] = nx[m1][n1 + 1] | (w & ld[s1] & !gm1);
          end
        end
      end
      stl = nx;
    end
    for (int s = 0; s < W; s++) r.mq[s] = ton[s];
    // 3. 计数：分流的四个队列直接从最终独热状态编码读出 (不再对每条的去向做加法)；
    //    IQ_B 的 nbf + b 按 b 预先加好 (nbf 来自寄存的分类，早到)，编码后不再串加法器
    begin
      logic [2:0] ea, eb, ec, em, en;
      ea = '0; eb = '0; ec = '0; em = '0; en = '0;
      for (int a = 0; a < NS; a++) for (int b = 0; b < NS; b++) for (int c = 0; c < NS; c++)
        if (a + b + c <= W) begin
          ea = ea | ({3{st[a][b][c]}} & 3'(a));
          eb = eb | ({3{st[a][b][c]}} & (nbf + 3'(b)));
          ec = ec | ({3{st[a][b][c]}} & 3'(c));
        end
      for (int m = 0; m < NS; m++) for (int n = 0; n < NS; n++)
        if (m + n <= W) begin
          em = em | ({3{stl[m][n]}} & 3'(m));
          en = en | ({3{stl[m][n]}} & 3'(n));
        end
      r.iqn[IQ_A] = ea; r.iqn[IQ_B] = eb; r.iqn[IQ_C] = ec;
      r.iqn[IQ_M] = em; r.iqn[IQ_N] = en;
      // 下一拍在途组用的独热差值：同样直接从最终独热状态或出
      inf = '0;
      for (int a = 0; a < NS; a++) for (int b = 0; b < NS; b++) for (int c = 0; c < NS; c++)
        if (a + b + c <= W) begin
          inf.ac[a - c + 6] = inf.ac[a - c + 6] | st[a][b][c];
          for (int v = 0; v < NS; v++)
            if (nbf == 3'(v)) begin
              inf.ab[a - b - v + 12] = inf.ab[a - b - v + 12] | st[a][b][c];
              inf.bc[b - c + v + 6]  = inf.bc[b - c + v + 6]  | st[a][b][c];
            end
        end
      for (int m = 0; m < NS; m++) for (int n = 0; n < NS; n++)
        if (m + n <= W) begin
          inf.mn[m - 2 * n + 12] = inf.mn[m - 2 * n + 12] | stl[m][n];
          inf.m[m]               = inf.m[m] | stl[m][n];
          inf.n[n]               = inf.n[n] | stl[m][n];
        end
    end
    r.iqn[IQ_F] = popc_w(fq); r.iqn[IQ_S] = popc_w(sto); r.iqn[IQ_D] = popc_w(dq);
    r.nld = popc_w(ld); r.nst = popc_w(sto);
    for (int s = 0; s < W; s++) begin
      logic [W-1:0] pre;
      pre = W'((1 << s) - 1);
      r.lqo[s] = popc_w(ld & pre); r.sqo[s] = popc_w(sto & pre);
    end
    // 入队路由：每条的主项 / 数据半边各去哪个队列，再按队列做前缀计数得入队口序号
    begin
      logic [N_IQ-1:0] h0q [W], h1q [W];
      for (int s = 0; s < W; s++) begin
        h0q[s] = '0; h1q[s] = '0;
        if (ab[s])  h0q[s][toc[s] ? IQ_C : tob[s] ? IQ_B : IQ_A] = 1'b1;
        if (fxb[s]) h0q[s][IQ_B] = 1'b1;
        if (ld[s])  h0q[s][ton[s] ? IQ_N : IQ_M] = 1'b1;
        if (sto[s]) h0q[s][IQ_S] = 1'b1;
        if (dq[s])  h1q[s][IQ_D] = 1'b1;
        if (cl.f1[s]) h1q[s][IQ_F] = 1'b1;
        if (cl.f0[s]) h0q[s][IQ_F] = 1'b1;
      end
      for (int q = 0; q < N_IQ; q++)
        for (int s = 0; s < W; s++) begin
          logic [3:0] p0, p1;
          p0 = '0;
          for (int j = 0; j < s; j++) p0 = p0 + 4'(h0q[j][q]) + 4'(h1q[j][q]);
          p1 = p0 + 4'(h0q[s][q]);
          if (h0q[s][q] && p0 < IQ_ENQ_MAX) r.sel[q][p0][s]     = 1'b1;
          if (h1q[s][q] && p1 < IQ_ENQ_MAX) r.sel[q][p1][W + s] = 1'b1;
        end
    end
    return '{need: r, inf: inf};
  endfunction

  // 并行版 (2026-10-24，6 GHz)：与 grp_need_ref (原逐条累加的写法) 逐位等价，rename 里有仿真断言核对。
  // 原写法每条要 "占用 + 已分数 -> 三方比较 -> 选队列 -> 计数 +1"，6 条串成一链 (2nm 综合约 364 ps)。
  // 改成"进位选择"：
  //   1. oa <= ob 等价于 (A - fa) - (B - fb) <= nb - na：左边每组算一次，右边只有 -6..6 共 13 种，
  //      于是所有比较位 (le_ab / le_ac / le_bc / le_mn) 与指令无关、一次并行算完；
  //   2. 每条指令对"之前已分到的计数 (na, nb, nc)"的每一种取值预先算出去向，再用独热状态逐条传递，
  //      每一步只剩"按状态选去向 + 状态转移" (与或两级)；
  //   3. 各总数、LQ / SQ 偏移、入队口序号都改成按位前缀计数。
  //   4. 空位数直接取 IQ 寄存的 fx 与在途组需求 tx (调用方已按输出级有效门控)，不先做饱和减：
  //      比较位写成一个多操作数式子 (加法树 + 符号)，不再是"饱和减 -> 相减 -> 比较"三层串联。
  //      与参考模型 (用饱和后的 free_adj) 只在 tx > fx 时可能不同；而结果只在 o_mv 时使用，
  //      那时在途组正在分派走、需求必然装得下 (tx <= fx)，rename 的断言也只在 o_mv 时核对。
  function automatic ds_need_t grp_need(input gn_cls_t cl,
                                        input logic [6:0] fa, input logic [6:0] fb, input logic [6:0] fc,
                                        input logic [6:0] fm, input logic [6:0] fn,
                                        input logic [3:0] ta, input logic [3:0] tb, input logic [3:0] tc,
                                        input logic [3:0] tm, input logic [3:0] tn);   // 4 位：可为两组在途之和 (参考用)
    localparam int NS = 7;                         // 每个计数 0..6
    ds_need_t r;
    gn_res_t  g;
    logic [12:0]  le_ab, le_ac, le_bc;             // 下标 d + 6，d = 右边的计数差
    logic [18:0]  le_mn;                           // 下标 2nn - nm + 6
    logic [NS-1:0] lt_m, lt_n;                     // k < fm / k < fn
    logic [NS-1:0] lt_b;                           // k + nbf < IQ_ENQ_MAX (B 队列本组还能收)
    int           dab, dac, dbc, dmn;
    logic [2:0]   nbf;
    nbf = cl.nbf;
    // 1. 比较位。状态里的 nb 只记本组 ALU / BRU 分到 B 的条数 (不含固定进 B 的 nbf 条)：
    //    nbf 并进比较常数与 B 的容量位，不再作为状态链的初值 (否则 6 步状态链要等 nbf 的计数先算完)
    // 占用 = SIZE - (fx - tx)；各比较位整体写成一个式子
    dab = (IQ_A_SIZE - IQ_B_SIZE) + int'(fb) - int'(fa) + int'(ta) - int'(tb) - int'(nbf);
    dac = (IQ_A_SIZE - IQ_C_SIZE) + int'(fc) - int'(fa) + int'(ta) - int'(tc);
    dbc = (IQ_B_SIZE - IQ_C_SIZE) + int'(fc) - int'(fb) + int'(tb) - int'(tc) + int'(nbf);
    for (int k = 0; k < NS; k++) lt_b[k] = k + int'(nbf) < IQ_ENQ_MAX;
    dmn = (IQ_M_SIZE - 2 * IQ_N_SIZE) - int'(fm) + int'(tm) + 2 * int'(fn) - 2 * int'(tn);
    for (int i = 0; i < 13; i++) begin le_ab[i] = dab <= i - 6; le_ac[i] = dac <= i - 6; le_bc[i] = dbc <= i - 6; end
    for (int i = 0; i < 19; i++) le_mn[i] = dmn <= i - 6;
    // k < (fx - tx)_饱和 与 k + tx < fx 等价 (饱和到 0 时两边都为假)
    for (int k = 0; k < NS; k++) begin lt_m[k] = k + int'(tm) < int'(fm); lt_n[k] = k + int'(tn) < int'(fn); end
    g = grp_need_core(cl, le_ab, le_ac, le_bc, le_mn, lt_m, lt_n);
    r = g.need;
    return r;
  endfunction

  // 比较位预计算一拍 (2026-10-28，6 GHz)。grp_need 一拍内"比较位 (多操作数加法 + 比较，~85 ps) -> 状态链 -> 计数 / sel"
  // 单独综合 179 ps (gn_loop：起点是 IQ 空位 fa，不是在途反馈)，所以把比较位挪到上一拍：
  //   上一拍 (gn_pre)：用当时的 IQ 空位、当时输出级的在途组 (数值) 与下一拍 rq0 那组的 nbf，
  //     把比较式里除"刚进输出级那组"以外的部分 base 与所有常数比完，寄存向量 (有整拍时间)；
  //   本拍 (grp_need_pp)：只补"上一拍还没算进去、本拍刚进输出级的那组" —— 它的贡献以独热 pv 寄存 (见 gn_inf_t，
  //     无此组时 = GN_INF_NONE)，按 pv 与或选位即得比较位。本拍从寄存器出来只有一级与或，不再有加法 / 移位。
  // 与"用上一拍空位、两组在途之和"的数值版 grp_need 逐位等价 (rename 断言)。比真实空位少算了上一拍发射走的项 ——
  // 只影响分流均衡 (保守)，分派时仍按实际空位检查
  typedef struct packed {
    logic [30:0] vab;              // vab[k + 12] = bab <= k，k ∈ -12..18   (bab 已含 -nbf_本组)
    logic [36:0] vbc;              // vbc[k + 24] = bbc <= k，k ∈ -24..12   (bbc 已含 +nbf_本组)
    logic [24:0] vac;              // vac[k + 12] = bac <= k，k ∈ -12..12
    logic [36:0] vmn;              // vmn[k + 12] = bmn <= k，k ∈ -12..24
    logic [12:0] tm, tn;           // tm[x] = x + 在途 m < fm，x ∈ 0..12
  } gn_pre_t;
  function automatic gn_pre_t gn_pre(input logic [6:0] fa, input logic [6:0] fb, input logic [6:0] fc,
                                     input logic [6:0] fm, input logic [6:0] fn,
                                     input logic [2:0] ta, input logic [2:0] tb, input logic [2:0] tc,
                                     input logic [2:0] tm, input logic [2:0] tn, input logic [2:0] nbf);
    gn_pre_t p;
    int bab, bac, bbc, bmn;
    bab = (IQ_A_SIZE - IQ_B_SIZE) + int'(fb) - int'(fa) + int'(ta) - int'(tb) - int'(nbf);
    bac = (IQ_A_SIZE - IQ_C_SIZE) + int'(fc) - int'(fa) + int'(ta) - int'(tc);
    bbc = (IQ_B_SIZE - IQ_C_SIZE) + int'(fc) - int'(fb) + int'(tb) - int'(tc) + int'(nbf);
    bmn = (IQ_M_SIZE - 2 * IQ_N_SIZE) - int'(fm) + int'(tm) + 2 * int'(fn) - 2 * int'(tn);
    for (int k = 0; k < 31; k++) p.vab[k] = bab <= k - 12;
    for (int k = 0; k < 37; k++) begin p.vbc[k] = bbc <= k - 24; p.vmn[k] = bmn <= k - 12; end
    for (int k = 0; k < 25; k++) p.vac[k] = bac <= k - 12;
    for (int x = 0; x < 13; x++) begin p.tm[x] = x + int'(tm) < int'(fm); p.tn[x] = x + int'(tn) < int'(fn); end
    return p;
  endfunction

  function automatic gn_res_t grp_need_pp(input gn_cls_t cl, input gn_pre_t p, input gn_inf_t pv);
    localparam int NS = 7;
    logic [12:0]  le_ab, le_ac, le_bc;
    logic [18:0]  le_mn;
    logic [NS-1:0] lt_m, lt_n;
    // le[i] = (base + d <= i - 6) = v[(i - 6 - d)]，d 取在途组贡献 (pv 的独热)
    for (int i = 0; i < 13; i++) begin
      le_ab[i] = 1'b0; le_ac[i] = 1'b0; le_bc[i] = 1'b0;
      for (int d = -12; d <= 6; d++) le_ab[i] = le_ab[i] | (pv.ab[d + 12] & p.vab[i - 6 - d + 12]);
      for (int d = -6; d <= 6; d++)  le_ac[i] = le_ac[i] | (pv.ac[d + 6] & p.vac[i - 6 - d + 12]);
      for (int d = -6; d <= 12; d++) le_bc[i] = le_bc[i] | (pv.bc[d + 6] & p.vbc[i - 6 - d + 24]);
    end
    for (int i = 0; i < 19; i++) begin
      le_mn[i] = 1'b0;
      for (int d = -12; d <= 6; d++) le_mn[i] = le_mn[i] | (pv.mn[d + 12] & p.vmn[i - 6 - d + 12]);
    end
    for (int k = 0; k < NS; k++) begin
      lt_m[k] = 1'b0; lt_n[k] = 1'b0;
      for (int j = 0; j < NS; j++) begin
        lt_m[k] = lt_m[k] | (pv.m[j] & p.tm[k + j]);
        lt_n[k] = lt_n[k] | (pv.n[j] & p.tn[k + j]);
      end
    end
    return grp_need_core(cl, le_ab, le_ac, le_bc, le_mn, lt_m, lt_n);
  endfunction


  // 参考模型：原逐条累加的写法 (仿真断言用)
  // load 在 IQ_M / IQ_N 间同样按占用分流 (IQ_M 两个口、腾空快，自然多分到)
  function automatic ds_need_t grp_need_ref(input logic [W-1:0] v, input uop_t [W-1:0] u,
                                        input logic [6:0] fa, input logic [6:0] fb, input logic [6:0] fc,
                                        input logic [6:0] fm, input logic [6:0] fn);
    ds_need_t r;
    int na, nb, nc, nm, nn, nf, nl, ns, nsa, nd;
    logic h0, h1;
    logic [2:0] q0, q1;
    int cnt [N_IQ];
    r = '0;
    na = 0; nb = 0; nc = 0; nm = 0; nn = 0; nf = 0; nl = 0; ns = 0; nsa = 0; nd = 0;
    for (int s = 0; s < W; s++)
      if (v[s] && fixed_b(u[s])) nb++;
    for (int s = 0; s < W; s++)
      if (v[s]) begin
        if (u[s].fu == FU_ALU || u[s].fu == FU_BRU) begin
          int oa, ob, oc;
          logic ka, kb, kc;
          oa = IQ_A_SIZE - int'(fa) + na; ob = IQ_B_SIZE - int'(fb) + nb; oc = IQ_C_SIZE - int'(fc) + nc;
          ka = na < IQ_ENQ_MAX; kb = nb < IQ_ENQ_MAX; kc = (u[s].fu == FU_ALU) && nc < IQ_ENQ_MAX;
          if (ka && (!kb || oa <= ob) && (!kc || oa <= oc))  begin r.aq[s] = 2'd0; na++; end
          else if (kb && (!kc || ob <= oc))                   begin r.aq[s] = 2'd1; nb++; end
          else                                                begin r.aq[s] = 2'd2; nc++; end
        end
        if (u[s].fu == FU_LD) begin
          int om, on;
          logic km, kn;
          om = IQ_M_SIZE - int'(fm) + nm; on = IQ_N_SIZE - int'(fn) + nn;
          // 还要看剩余空位：只按占用分时，一边快满了也照样分过去，分派因这一边不够而整组停
          // (s2 实测：CoreMark 分派停顿 IQ_M 10.5 万 + IQ_N 6.9 万拍，拆分前 IQ_M 12.1 万)
          km = nm < IQ_ENQ_MAX && nm < int'(fm); kn = nn < IQ_ENQ_MAX && nn < int'(fn);
          // 按服务速率加权：IQ_M 两个口、IQ_N 一个口，IQ_M 占用到 IQ_N 的 2 倍才分给 IQ_N
          // (s2/s3 按占用五五分：两边各约 10 项，IQ_N 单口里的 load 等得更久，CoreMark -1.8 %)
          if (km && (!kn || om <= 2 * on)) begin r.mq[s] = 1'b0; nm++; end
          else                         begin r.mq[s] = 1'b1; nn++; end
        end
        if (u[s].fu == FU_ST || u[s].fu == FU_AMO) nsa++;
        if ((u[s].fu == FU_ST && u[s].rs2_class != RC_FP) || (u[s].fu == FU_AMO && !amo_nodata(u[s].op))) nd++;   // LR 与 cbo.* 无数据 (同 gn_cls / o6_dispatch)
        if (u[s].fu == FU_FMA || u[s].fu == FU_FMISC || u[s].fu == FU_F2I || u[s].fu == FU_FDIV ||
            (u[s].fu == FU_ST && u[s].rs2_class == RC_FP)) nf++;
        if (u[s].fu == FU_LD) nl++;
        if (u[s].fu == FU_ST || u[s].fu == FU_AMO) ns++;
      end
    r.iqn[IQ_A] = 3'(na); r.iqn[IQ_B] = 3'(nb); r.iqn[IQ_M] = 3'(nm); r.iqn[IQ_F] = 3'(nf);
    r.iqn[IQ_S] = 3'(nsa); r.iqn[IQ_D] = 3'(nd); r.iqn[IQ_C] = 3'(nc); r.iqn[IQ_N] = 3'(nn);
    r.nld = 3'(nl); r.nst = 3'(ns);
    // ---- 入队路由 (与 o6_dispatch 的参考模型逐条对应，分派级有断言核对)
    for (int q = 0; q < N_IQ; q++) cnt[q] = 0;
    nl = 0; ns = 0;
    for (int s = 0; s < W; s++) begin
      h0 = 1'b0; h1 = 1'b0; q0 = '0; q1 = '0;
      r.lqo[s] = 3'(nl); r.sqo[s] = 3'(ns);
      if (v[s]) begin
        if (u[s].fu == FU_LD) nl++;
        if (u[s].fu == FU_ST || u[s].fu == FU_AMO) ns++;
        unique case (u[s].fu)
          FU_ALU, FU_BRU:                 begin h0 = 1'b1; q0 = (r.aq[s] == 2'd0) ? 3'(IQ_A) : (r.aq[s] == 2'd1) ? 3'(IQ_B) : 3'(IQ_C); end
          FU_MUL, FU_DIV, FU_CSR, FU_I2F, FU_VEC: begin h0 = 1'b1; q0 = 3'(IQ_B); end
          FU_LD:                          begin h0 = 1'b1; q0 = r.mq[s] ? 3'(IQ_N) : 3'(IQ_M); end
          FU_ST: begin
            h0 = 1'b1; q0 = 3'(IQ_S);
            h1 = 1'b1; q1 = (u[s].rs2_class == RC_FP) ? 3'(IQ_F) : 3'(IQ_D);
          end
          FU_AMO: begin
            h0 = 1'b1; q0 = 3'(IQ_S);
            if (!amo_nodata(u[s].op)) begin h1 = 1'b1; q1 = 3'(IQ_D); end   // LR / cbo.* 无数据
          end
          FU_FMA, FU_FMISC, FU_F2I, FU_FDIV: begin h0 = 1'b1; q0 = 3'(IQ_F); end
          default: ;
        endcase
      end
      if (h0) begin
        if (cnt[q0] < IQ_ENQ_MAX) r.sel[q0][cnt[q0]][s] = 1'b1;
        cnt[q0]++;
      end
      if (h1) begin
        if (cnt[q1] < IQ_ENQ_MAX) r.sel[q1][cnt[q1]][W+s] = 1'b1;
        cnt[q1]++;
      end
    end
    return r;
  endfunction
  //--------------------------------------------------------------------------
  // 截组用的资源类别 (按展开后的 32 位编码，只看操作码 / funct)：
  //   {br: 需要分支标签, A: BRU, B: MUL/DIV/CSR/I2F, M: LD, S: STA/AMO 地址, D: 整数 STD/AMO 数据,
  //    F: 浮点/浮点 STD, ALU}
  // 只能比译码器多算、不能少算 (译码器只会因非法指令把 fu 置 NONE 而少用资源)，
  // 否则一组的入队数可能超过 IQ_ENQ_MAX；译码级有仿真断言核对
  //--------------------------------------------------------------------------
  // LOAD-FP / STORE-FP 的 width 字段是向量访存 (EEW 8 / 16 / 32 / 64)
  function automatic logic vmem_w(input logic [2:0] w);
    return w == 3'b000 || w == 3'b101 || w == 3'b110 || w == 3'b111;
  endfunction
  function automatic logic [7:0] fe_cls(input logic [31:0] x);
    logic br, a, b, m, sa, sd, f, alu;
    logic [6:0] opc;
    br = 1'b0; a = 1'b0; b = 1'b0; m = 1'b0; sa = 1'b0; sd = 1'b0; f = 1'b0; alu = 1'b0;
    opc = x[6:0];
    unique case (opc)
      o6_isa_pkg::OPC_LUI: alu = 1'b1;
      // Zbb 的 clz / ctz / cpop(w) (imm = 0x600..0x602，f3 = 001) 走 MUL 流水 (B 类)
      o6_isa_pkg::OPC_OP_IMM, o6_isa_pkg::OPC_OP_IMM32:
        if (x[14:12] == 3'b001 && (x[31:20] == 12'h600 || x[31:20] == 12'h601 || x[31:20] == 12'h602)) b = 1'b1;
        else alu = 1'b1;
      o6_isa_pkg::OPC_OP, o6_isa_pkg::OPC_OP32: if (x[31:25] == 7'b0000001) b = 1'b1; else alu = 1'b1;
      o6_isa_pkg::OPC_AUIPC, o6_isa_pkg::OPC_JAL: a = 1'b1;
      o6_isa_pkg::OPC_JALR, o6_isa_pkg::OPC_BRANCH: begin a = 1'b1; br = 1'b1; end
      o6_isa_pkg::OPC_LOAD:     m = 1'b1;
      // LOAD-FP / STORE-FP 的宽度 000 / 101 / 110 / 111 是向量访存 (FU_VEC，进 IQ_B)
      o6_isa_pkg::OPC_LOAD_FP:  if (vmem_w(x[14:12])) b = 1'b1; else m = 1'b1;
      o6_isa_pkg::OPC_STORE:    begin sa = 1'b1; sd = 1'b1; end
      o6_isa_pkg::OPC_STORE_FP: if (vmem_w(x[14:12])) b = 1'b1; else begin sa = 1'b1; f = 1'b1; end
      o6_isa_pkg::OPC_OP_V, o6_isa_pkg::OPC_OP_VE: b = 1'b1;
      o6_isa_pkg::OPC_AMO:      begin sa = 1'b1; sd = 1'b1; end
      // SYSTEM：f3 = 100 是 Zimop (mop.r / rr -> ALU) 与 H 的 HLV (-> load) / HSV (-> store)；wrs.sto -> ALU (只多不少：B 照留)
      o6_isa_pkg::OPC_SYSTEM: begin
        b = 1'b1;
        if (x[14:12] == 3'b100) begin
          if (x[31]) alu = 1'b1; else if (x[25]) begin sa = 1'b1; sd = 1'b1; end else m = 1'b1;
        end
        if (x == 32'h01d0_0073) alu = 1'b1;
      end
      // MISC-MEM：f3 = 010 是 cbo.* (FU_AMO，无数据)
      o6_isa_pkg::OPC_MISC_MEM: if (x[14:12] == 3'b010) sa = 1'b1; else b = 1'b1;
      o6_isa_pkg::OPC_MADD, o6_isa_pkg::OPC_MSUB, o6_isa_pkg::OPC_NMSUB, o6_isa_pkg::OPC_NMADD: f = 1'b1;
      // FMV.*.X / FCVT.*.[W|L] 走整数侧 (I2F)；同在 11110 的 Zfa fli (rs2 = 1) 是 FMISC
      o6_isa_pkg::OPC_OP_FP: if (x[31:27] == 5'b11010 || (x[31:27] == 5'b11110 && x[24:20] != 5'd1)) b = 1'b1; else f = 1'b1;
      default: ;
    endcase
    return {br, a, b, m, sa, sd, f, alu};
  endfunction

  //--------------------------------------------------------------------------
  // 分支预测器 v2 (TAGE，见 docs/08_branch_predictor.md)
  //   8 张带标签表，每张 1024 行 x 2 路 (路以"标签 + 槽号"区分)，12 位标签，历史 6..300 (几何级数)
  //   全局历史 = 跳转路径历史：每次跳转推入 2 位 (分支槽号 ^ 目标地址 的散列)，不跳的分支不推
  //--------------------------------------------------------------------------
  localparam int BP_NT  = 8;                  // 带标签表个数
  localparam int BP_LG  = 10;                 // 每表行数 log2 (两体交织，每体 512 行)
  localparam int BP_TW  = 12;                 // 标签宽度
  localparam int BP_HMAX = 300;               // 最长历史 (位)
  // 原始历史寄存器长度：最长历史 + 在途推测推入的余量 (FTQ 全部项 + F0..F3，每次 2 位)
  localparam int BP_GHL = BP_HMAX + 2 * (FTQ_SIZE + 8);
  localparam int BP_KW  = 7;                  // 推入次数计数器宽度 (恢复时差值 <= FTQ_SIZE + 8)

  function automatic int bp_hl(input int t);
    unique case (t)
      0: return 6;   1: return 10;  2: return 18;  3: return 32;
      4: return 56;  5: return 98;  6: return 172; default: return 300;
    endcase
  endfunction

  // 折叠历史 (每表：索引 BP_LG 位、标签两段 BP_TW / BP_TW-1 位)
  typedef struct packed {
    logic [BP_NT-1:0][BP_LG-1:0] fi;
    logic [BP_NT-1:0][BP_TW-1:0] g1;
    logic [BP_NT-1:0][BP_TW-2:0] g2;
  } bp_fold_t;

  // 推入的 2 位：只散列目标地址 (bpsim：hr=1 + ht=1)。不用槽号：F0 按 uBTB 推入时槽号可能与最终预测不同
  // 而目标相同 (F3 不纠正)，同一路径会产生两种历史、把同一分支拆进多个 TAGE 项；只看去向则推测与实际一致。
  // 模型：只用目标的信息损失约 1%
  function automatic logic [1:0] bp_phash(input logic [3:0] slot, input logic [63:0] tgt);
    return {tgt[2] ^ tgt[6] ^ tgt[10], tgt[1] ^ tgt[5] ^ tgt[9]};
  endfunction

  // 每槽预测信息 (F2 算出，随 F3 / FTQ 带到训练)
  typedef struct packed {
    logic       phit;     // 有提供者
    logic [2:0] pt;       // 提供者表号
    logic       pw;       // 提供者路
    logic [2:0] pctr;     // 提供者计数器 (>= 4 预测跳)
    logic [1:0] pu;       // 提供者有用位
    logic       adir;     // 替补方向 (次长命中表，否则 bimodal)
    logic       tdir;     // TAGE 方向 (含"新项弱时用替补")；TAGE 自己的训练按它判对错
    logic       scd;      // SC 求和的方向 (门控前)
    logic       scl;      // SC 求和的绝对值 < 阈值 (低置信：不推翻 TAGE，训练时照样更新)
    logic       fdir;     // 最终方向 (TAGE 经 SC 修正)
  } bp_slot_t;

  //--------------------------------------------------------------------------
  // 统计校正器 SC (2026-10-14，o6_sc)：GEHL，偏置表 + 3 张历史表 (复用 TAGE 折叠历史 0/1/2，即 6/10/18)，
  // 每表 2^SC_LG 行；每行按块内 4 个 8 B 槽组 x TAGE 方向 2 种各一个 SC_CW 位有符号计数器 (触发器)
  //--------------------------------------------------------------------------
  localparam int SC_NT = 4;
  localparam int SC_LG = 7;
  localparam int SC_CW = 5;

  //--------------------------------------------------------------------------
  // 间接跳转目标预测 ITTAGE (2026-10-14，o6_ittage)：IT_NT 张带标签表 x 2^IT_LG 项，
  // 复用 TAGE 第 0/2/4/6 表的折叠历史 (6/18/56/172)；基础预测仍是 uBTB / BTB 记的上次目标
  //--------------------------------------------------------------------------
  localparam int IT_NT = 4;
  localparam int IT_LG = 8;
  localparam int IT_TW = 9;
  // 预测时的信息 (随 FTQ 带到训练)
  typedef struct packed {
    logic                   phit;     // 有提供者
    logic [1:0]             pt;       // 提供者表号
    logic [1:0]             pctr;     // 提供者置信计数器
    logic                   pu;       // 提供者有用位
    logic [38:1]            ptgt;     // 提供者目标
    logic [IT_NT-1:0][1:0]  uv;       // 各表 {有效, 有用位}
  } it_info_t;

  //==========================================================================
  // L1 缓存几何：64 组 x 16 路 x 64 B = 64 KB (组号 = 虚地址 [11:6]，VIPT 无别名要求每路 <= 4 KB)
  //==========================================================================
`ifdef O6_L1_NWAY
  localparam int L1_NWAY = `O6_L1_NWAY;
`else
  localparam int L1_NWAY = 16;
`endif
  localparam int L1_WB   = $clog2(L1_NWAY);

  // 树形伪 LRU (L1_NWAY-1 个节点，堆式编号：节点 i 的子节点 2i+1 / 2i+2)
  // 节点值 = 下次替换往哪边走 (0 左 1 右)；访问某路时把路径上每个节点指向另一边
  function automatic logic [L1_WB-1:0] l1_plru_victim(input logic [L1_NWAY-2:0] t);
    logic [L1_WB-1:0] v;
    int n;
    n = 0;
    for (int l = 0; l < L1_WB; l++) begin
      v[L1_WB-1-l] = t[n];
      n = 2*n + 1 + int'(t[n]);
    end
    return v;
  endfunction
  function automatic logic [L1_NWAY-2:0] l1_plru_touch(input logic [L1_NWAY-2:0] t, input logic [L1_WB-1:0] w);
    logic [L1_NWAY-2:0] r;
    int n;
    r = t;
    n = 0;
    for (int l = 0; l < L1_WB; l++) begin
      r[n] = ~w[L1_WB-1-l];
      n = 2*n + 1 + int'(w[L1_WB-1-l]);
    end
    return r;
  endfunction
endpackage

`endif
