#!/usr/bin/env python3
"""rvgen.py - OOO6 随机指令程序生成器 (M1：RV64IMAC，M 态，裸机)

生成一个汇编文件，用 riscv64-unknown-elf-gcc 编成 ELF 后在 tb_top 里与 Spike 逐条对拍。
结束时向 tohost 写 1。

保留寄存器 (随机指令不写)：
  x0、ra(x1, 调用)、sp(x2)、gp(x3)、tp(x4)、s0(x8, 数据区基址)、
  t4/t5(x29/x30, 陷入处理)、t6(x31, 循环计数)
随机指令可写的：其余 22 个整数寄存器。

覆盖的类别 (权重可调)：
  alu / alu_imm / lui_auipc / mul / div / load / store / store_load_pair (强制转发)
  branch_fwd / loop / call / indirect / amo / lrsc / csr / trap (ecall / 未对齐 / 非法)
  --fp 打开浮点 (RV64FD)：fp_arith / fp_fma / fp_cmp / fp_cvt / fp_mem / fp_csr
  --csr 打开 CSR 随机访问：可写 CSR 随机读写 (屏蔽 MIE/SIE/MPRV、PMP 锁定位等会失控的位)，
        外加随机 CSR 地址的读 (未实现的地址触发非法指令陷入，与 Spike 对拍)
  --vm  主体改在 S 态、Sv39 / Sv48 / Sv57 分页下跑 (模式随机；随机页表 + 随机 PMP)：
        Sv48 / Sv57 在 Sv39 的三级表上面再加一 / 两级：项 0 指下一级，项 1 也指下一级 (VA + 2^39 / 2^48 是别名，
        Sv39 / Sv48 下却是非规范地址 -> 缺页，探测时随机加这两个偏移)；menvcfg.PBMTE 随机；
        探测区 VA 0xC001_0000 起 64 KB 是 Svnapot 页 (16 个相同 PTE，PPN[3:0] = 1000，映射到 64 KB 对齐的 pbuf)；
        menvcfg.PMM 随机 (关 / PMLEN 7 / PMLEN 16，Smnpm)，探测地址随机在高 7 / 16 位带标签 (掩码关或 MXR = 1 时是非规范地址)；
        代码区恒等映射，粒度随机 (1 GB 吉页 / 2 MB 大页 / 4 KB 小页)；数据区经虚拟别名 VA 0x4000_0000 访问 (粒度随机)；
        探测区 VA 0xC000_0000：8 个 4 KB 页 PTE 属性随机 (正常 / 只读 / 缺 A / 缺 D / 无效 / W 无 R / 只执行 / U 页 / 保留位)，
        另有一个 PPN 不对齐的 2 MB 超级页 (必缺页)；主体里随机改写探测页 PTE (随后 sfence.vma)、切换 SUM / MXR；
        PMP 前 1~4 项随机落在探测缓冲区 (TOR / NAPOT x 合法权限，粒度 64 B)，第 7 项放行全部。页表本身不受随机 PMP 覆盖。
        出错的访存由 M 态陷入处理跳过 (mepc += 指令长度)，缺页 / 访问错误与 Spike 对拍
  --hyp H 扩展 (RVA23 R5a)：M 态开场随机选主体模式 HS / VS / VU / U (mret，MPV / MPP)，随机 medeleg / hedeleg /
        hstatus (VTW / VTVM / HU / SPVP / VTSR)、hcounteren / scounteren、henvcfg；vsatp = Bare；hgatp 随机 (Bare / Sv39x4 / 48x4 / 57x4：
        DRAM 恒等 1 GB 叶子 + GPA 0x1_0000_0000 起 8 个 4 KB 的 G 探测页，G-PTE 属性随机 -> guest page fault)；
        三套陷入处理：M (mepc)、HS 与 VS 共用一段 (V = 1 时 sepc 即 vsepc)，都是跳过出错指令；不开中断使能。
        主体里随机：H / VS CSR 读写 (MODE、vstvec、各中断使能不写)、HLV / HLVX / HSV (基址 s0)、hfence.vvma / gvma、
        sfence.vma、wfi、wrs.nto、ecall、sret (先把 sepc 设成下一条：在 HS 里会按 SPP / SPV 换模式)；VTSR = 1 时不往 VS 委托；
        G 探测 (普通访存 / HLV / HSV 打 G 探测页)
  --hyp --vm  两级翻译：主体在 VS 态，vsatp 用 --vm 的页表 (satp 那套改写 vsatp)，hgatp 同上 (非 Bare)；VS 探测页之外再有
        VA PVA + 128 KB 起 8 页 -> G 探测页，VA 0x1_0000_0000 的 VS 页表页落在 G 探测页 8 上 (G 属性随机：隐式 guest page fault)；
        陷入都到 M；不发 H CSR 写与 sret (VS 页表的代码页 U = 0，进 VU 会卡死)
  浮点寄存器全部可写；初值来自含特殊值 (±0/±inf/NaN/非规格化/未 NaN-box) 的数据区。
用法：
  rvgen.py --seed 1 --n 2000 -o test.S [--fp]   (as 需 -march=rv64gc)
"""
import argparse
import random

RESERVED = {0, 1, 2, 3, 4, 8, 29, 30, 31}
WREGS = [r for r in range(32) if r not in RESERVED]
ALLREGS = list(range(32))
ABI = ["zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
       "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"]

DATA_SIZE = 2048     # 数据区字节数 (小一点才会频繁地址碰撞)


def x(r):
    return ABI[r]


class Gen:
    def __init__(self, seed, n, weights):
        self.r = random.Random(seed)
        self.n = n
        self.w = weights
        self.lines = []
        self.lbl = 0
        self.funcs = []
        self.recent = []          # 最近写过的寄存器 (制造依赖)

    def label(self, p="L"):
        self.lbl += 1
        return f"{p}{self.lbl}"

    def emit(self, s):
        self.lines.append("    " + s)

    def rd(self):
        r = self.r.choice(WREGS)
        self.recent = (self.recent + [r])[-6:]
        return r

    def rs(self):
        # 70% 取最近写过的寄存器，制造真相关
        if self.recent and self.r.random() < 0.7:
            return self.r.choice(self.recent)
        return self.r.choice(ALLREGS)

    def imm12(self):
        c = self.r.random()
        if c < 0.2:
            return self.r.choice([0, 1, -1, 2047, -2048])
        return self.r.randint(-2048, 2047)

    # ------------------------------------------------------------------ 类别
    def g_alu(self):
        op = self.r.choice(["add", "sub", "sll", "slt", "sltu", "xor", "srl", "sra", "or", "and",
                            "addw", "subw", "sllw", "srlw", "sraw"])
        self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, {x(self.rs())}")

    def g_alu_imm(self):
        op = self.r.choice(["addi", "slti", "sltiu", "xori", "ori", "andi", "slli", "srli", "srai",
                            "addiw", "slliw", "srliw", "sraiw"])
        if op in ("slli", "srli", "srai"):
            imm = self.r.randint(0, 63)
        elif op in ("slliw", "srliw", "sraiw"):
            imm = self.r.randint(0, 31)
        else:
            imm = self.imm12()
        self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, {imm}")

    # RVA23 R1 (docs/14)：Zicond / Zimop / Zawrs / Zcb / Zcmop。压缩形式只能用 x8..x15，x8 (s0) 是数据区基址，
    # 目的寄存器只取 s1、a0..a5；wrs.nto 只在 M 态主体里出 (S 态下 TW 可能被置，会陷入)
    CREGS = [9, 10, 11, 12, 13, 14, 15]

    def g_rva23(self):
        k = self.r.randrange(11)
        cr = lambda: self.r.choice(self.CREGS)
        if k == 0:
            op = self.r.choice(["czero.eqz", "czero.nez"])
            b = 0 if self.r.random() < 0.3 else self.rs()           # rs2 = x0 / 常为 0 的情形
            self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, {x(b)}")
        elif k == 1:
            self.emit(f"mop.r.{self.r.randrange(32)} {x(self.rd())}, {x(self.rs())}")
        elif k == 2:
            self.emit(f"mop.rr.{self.r.randrange(8)} {x(self.rd())}, {x(self.rs())}, {x(self.rs())}")
        elif k == 3:
            self.emit("wrs.sto" if (getattr(self, "vm", False) or self.r.random() < 0.5) else "wrs.nto")
        elif k == 4:
            op, sz = self.r.choice([("c.lbu", 1), ("c.lhu", 2), ("c.lh", 2)])
            off = self.r.randrange(0, 4, sz)
            self.emit(f"{op} {x(cr())}, {off}(s0)")
        elif k == 5:
            op, sz = self.r.choice([("c.sb", 1), ("c.sh", 2)])
            off = self.r.randrange(0, 4, sz)
            self.emit(f"{op} {x(cr())}, {off}(s0)")
        elif k == 6:
            self.emit(f"c.mul {x(cr())}, {x(cr())}")
        elif k == 7:
            op = self.r.choice(["c.zext.b", "c.sext.b", "c.zext.h", "c.sext.h", "c.zext.w", "c.not"])
            self.emit(f"{op} {x(cr())}")
        elif k == 8:
            self.emit(f"c.mop.{self.r.choice(range(1, 16, 2))}")
        elif not getattr(self, "vm", False):
            # Zicbom / Zicboz：M 态主体里才出 (S 态默认 menvcfg 全关会陷入)；地址不要求对齐，作用于所在 64 B 块
            op = self.r.choice(["cbo.zero", "cbo.clean", "cbo.flush", "cbo.inval"])
            self.emit(f"addi t5, s0, {self.r.randrange(0, DATA_SIZE - 64)}")
            self.emit(f"{op} (t5)")
        else:
            self.emit("wrs.sto")

    def g_lui_auipc(self):
        op = self.r.choice(["lui", "auipc"])
        self.emit(f"{op} {x(self.rd())}, {self.r.randint(0, 0xfffff)}")

    def g_mul(self):
        op = self.r.choice(["mul", "mulh", "mulhsu", "mulhu", "mulw"])
        self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, {x(self.rs())}")

    def g_div(self):
        op = self.r.choice(["div", "divu", "rem", "remu", "divw", "divuw", "remw", "remuw"])
        b = self.rs()
        if self.r.random() < 0.1:
            b = 0                                          # 除零
        self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, {x(b)}")

    def mem_off(self, size):
        return self.r.randrange(0, DATA_SIZE - 8, size)

    def g_load(self):
        op, sz = self.r.choice([("lb", 1), ("lbu", 1), ("lh", 2), ("lhu", 2), ("lw", 4), ("lwu", 4), ("ld", 8)])
        self.emit(f"{op} {x(self.rd())}, {self.mem_off(sz)}(s0)")

    def g_store(self):
        op, sz = self.r.choice([("sb", 1), ("sh", 2), ("sw", 4), ("sd", 8)])
        self.emit(f"{op} {x(self.rs())}, {self.mem_off(sz)}(s0)")

    def g_store_load_pair(self):
        # store 后紧跟重叠的 load：测转发 (全覆盖 / 部分覆盖 / 多个 store 拼字节)
        sz_s = self.r.choice([1, 2, 4, 8])
        base = self.r.randrange(0, DATA_SIZE - 16, 8)
        so = base + self.r.randrange(0, 8, sz_s)
        sop = {1: "sb", 2: "sh", 4: "sw", 8: "sd"}[sz_s]
        self.emit(f"{sop} {x(self.rs())}, {so}(s0)")
        if self.r.random() < 0.4:
            sz2 = self.r.choice([1, 2, 4])
            so2 = base + self.r.randrange(0, 8, sz2)
            self.emit(f"{ {1: 'sb', 2: 'sh', 4: 'sw'}[sz2]} {x(self.rs())}, {so2}(s0)")
        lop, lsz = self.r.choice([("lb", 1), ("lhu", 2), ("lw", 4), ("ld", 8)])
        lo = base + self.r.randrange(0, 8, lsz)
        self.emit(f"{lop} {x(self.rd())}, {lo}(s0)")

    def g_branch_fwd(self):
        op = self.r.choice(["beq", "bne", "blt", "bge", "bltu", "bgeu"])
        l = self.label()
        self.emit(f"{op} {x(self.rs())}, {x(self.rs())}, {l}")
        for _ in range(self.r.randint(0, 4)):
            self.g_simple()
        self.lines.append(f"{l}:")

    def g_loop(self):
        # 有界循环：t6 计数
        l = self.label()
        self.emit(f"li t6, {self.r.randint(1, 6)}")
        self.lines.append(f"{l}:")
        for _ in range(self.r.randint(1, 6)):
            self.g_simple()
        self.emit("addi t6, t6, -1")
        self.emit(f"bnez t6, {l}")

    def g_call(self):
        f = self.label("F")
        self.funcs.append(f)
        self.emit(f"call {f}")

    def g_indirect(self):
        l = self.label()
        r = self.r.choice([5, 6, 7, 28])            # t0-t2 / t3
        self.emit(f"la {x(r)}, {l}")
        self.emit(f"jalr x0, 0({x(r)})")
        for _ in range(self.r.randint(0, 2)):
            self.g_simple()
        self.lines.append(f"{l}:")

    def g_amo(self):
        op = self.r.choice(["amoswap", "amoadd", "amoxor", "amoand", "amoor", "amomin", "amomax",
                            "amominu", "amomaxu"])
        w = self.r.choice(["w", "d"])
        off = self.mem_off(8 if w == "d" else 4)
        self.emit(f"addi t5, s0, {off}")
        self.emit(f"{op}.{w} {x(self.rd())}, {x(self.rs())}, (t5)")

    def g_lrsc(self):
        w = self.r.choice(["w", "d"])
        off = self.mem_off(8 if w == "d" else 4)
        self.emit(f"addi t5, s0, {off}")
        self.emit(f"lr.{w} {x(self.rd())}, (t5)")
        if self.r.random() < 0.7:
            self.g_alu()
        if self.r.random() < 0.2:
            off = self.mem_off(8 if w == "d" else 4)          # SC 到别的地址：必失败
            self.emit(f"addi t5, s0, {off}")
        self.emit(f"sc.{w} {x(self.rd())}, {x(self.rs())}, (t5)")

    def g_csr(self):
        op = self.r.choice(["csrrw", "csrrs", "csrrc"])
        s_mode = getattr(self, "vm", False) or getattr(self, "hyp", False)   # --vm / --hyp：主体不在 M 态
        self.emit(f"{op} {x(self.rd())}, {'sscratch' if s_mode else 'mscratch'}, {x(self.rs())}")
        if self.r.random() < 0.3:
            self.emit(f"csrr {x(self.rd())}, {'instret' if s_mode else 'minstret'}")

    def g_trap(self):
        c = self.r.random()
        if c < 0.4:
            self.emit("ecall")
        elif c < 0.8:
            op, sz = self.r.choice([("lh", 2), ("lw", 4), ("ld", 8), ("sh", 2), ("sw", 4), ("sd", 8)])
            off = self.r.randrange(1, DATA_SIZE - 16)
            if off % sz == 0:
                off += 1
            if op[0] == "l":
                self.emit(f"{op} {x(self.rd())}, {off}(s0)")
            else:
                self.emit(f"{op} {x(self.rs())}, {off}(s0)")
        else:
            self.emit(".word 0x0000707b")        # 非法编码 (custom-3 空间未实现)

    # ------------------------------------------------------------------ H 扩展 (--hyp)
    # (地址, 写入前清掉的位, 是否只读)：MODE 字段 (vsatp / hgatp 保持 Bare)、各中断使能、vstvec 不改
    HCSR = [
        (0x600, 0, False), (0x602, 0, False), (0x603, 0, False), (0x604, -1, True), (0x605, 0, False),
        (0x606, 0, False), (0x607, 0, False), (0x60A, 0, False), (0x60C, 0, False), (0x60D, 0, False),
        (0x643, 0, False), (0x644, 0, False), (0x645, 0, False), (0x64A, 0, False), (0x680, 0, True),
        (0x6A8, 0, False), (0xE12, 0, True),
        (0x200, 0, False), (0x204, -1, True), (0x205, -1, True), (0x240, 0, False), (0x241, 0, False),
        (0x242, 0, False), (0x243, 0, False), (0x244, 0, False), (0x24D, 0, False), (0x280, 0xF << 60, False),
        (0x34A, 0, False), (0x34B, 0, False),
        (0x100, 0, False), (0x104, -1, True), (0x140, 0, False), (0x141, 0, False), (0x142, 0, False),
        (0x143, 0, False), (0x144, 0, False), (0x180, 0xF << 60, False), (0x14D, 0, False), (0x10A, 0, False),
        (0x106, 0, False), (0x10C, 0, False), (0xC00, 0, True), (0xC01, 0, True), (0xC02, 0, True), (0xDA0, 0, True),
    ]

    def g_hyp_csr(self):
        a, clr, ro = self.r.choice(self.HCSR)
        if a == 0x602 and getattr(self, "vtsr", False):
            clr = -1                                           # VTSR = 1：不往 VS 委托 (VS 处理程序的 sret 会陷入)
        if a == 0x600:
            clr = 1 << 22                                      # hstatus.VTSR 只在开场定
        if ro or self.r.random() < 0.4:
            self.emit(f"csrr {x(self.rd())}, {a:#x}")
            return
        op = self.r.choice(["csrrw", "csrrs", "csrrc"])
        src = self.rs()
        if clr:
            self.emit(f"li t5, {(~clr) & ((1 << 64) - 1):#x}")
            self.emit(f"and t5, t5, {x(src)}")
            src = 30                                           # t5
        self.emit(f"{op} {x(self.rd())}, {a:#x}, {x(src)}")

    def g_hyp_ls(self):
        if self.r.random() < 0.6:
            op, sz = self.r.choice([("hlv.b", 1), ("hlv.bu", 1), ("hlv.h", 2), ("hlv.hu", 2), ("hlv.w", 4),
                                    ("hlv.wu", 4), ("hlv.d", 8), ("hlvx.hu", 2), ("hlvx.wu", 4)])
            st = False
        else:
            op, sz = self.r.choice([("hsv.b", 1), ("hsv.h", 2), ("hsv.w", 4), ("hsv.d", 8)])
            st = True
        off = self.r.randrange(0, DATA_SIZE - 16, sz)
        if self.r.random() < 0.08 and sz > 1:
            off += 1                                           # 未对齐
        self.emit(f"addi t5, s0, {off}")
        if st:
            self.emit(f"{op} {x(self.rs())}, (t5)")
        else:
            self.emit(f"{op} {x(self.rd())}, (t5)")

    GALIAS = 0x100000000

    def g_gprobe(self):
        if getattr(self, "hvm", False):                        # 两级：VA -> VS 页表 -> G 探测页 / 隐式
            if self.r.random() < 0.8:
                va = self.PVA + 0x20000 + self.r.randrange(8) * 4096
            else:
                va = 0x100000000
        else:                                                  # vsatp = Bare：GPA 直接落在 G 探测页
            va = self.GALIAS + self.r.randrange(8) * 4096
        sz = self.r.choice([1, 2, 4, 8])
        va += self.r.randrange(0, 4096 - 8, sz)
        self.emit(f"li t5, {va:#x}")
        c = self.r.random()
        hv = getattr(self, "hyp", False) and not getattr(self, "hvm", False) and c < 0.5
        ld = {1: "b", 2: "h", 4: "w", 8: "d"}[sz]
        if self.r.random() < 0.6:
            if hv:
                op = self.r.choice([f"hlv.{ld}", "hlvx.hu" if sz == 2 else f"hlv.{ld}", "hlvx.wu" if sz == 4 else f"hlv.{ld}"])
                self.emit(f"{op} {x(self.rd())}, (t5)")
            else:
                self.emit(f"l{ld} {x(self.rd())}, 0(t5)")
        else:
            if hv:
                self.emit(f"hsv.{ld} {x(self.rs())}, (t5)")
            elif self.r.random() < 0.15:
                self.emit(f"amoadd.d {x(self.rd())}, {x(self.rs())}, (t5)" if sz == 8 else f"s{ld} {x(self.rs())}, 0(t5)")
            else:
                self.emit(f"s{ld} {x(self.rs())}, 0(t5)")

    def g_hyp_misc(self):
        c = self.r.randrange(9)
        if c == 0:
            self.emit(f"hfence.vvma {x(self.r.choice([0, self.rs()]))}, {x(self.r.choice([0, self.rs()]))}")
        elif c == 1:
            self.emit(f"hfence.gvma {x(self.r.choice([0, self.rs()]))}, {x(self.r.choice([0, self.rs()]))}")
        elif c == 2:
            self.emit("sfence.vma")
        elif c == 3:
            self.emit("wfi")
        elif c == 4:
            self.emit("wrs.nto")
        elif c == 5:
            self.emit("ecall")
        elif c == 6:
            self.emit(".word 0x26000073" if self.r.random() < 0.5 else ".word 0x66000073")   # hinval.vvma / hinval.gvma x0, x0
        elif getattr(self, "hvm", False):
            self.emit("sfence.vma")                            # 两级 (VS 页表) 时不发 sret
        else:
            # sret 到下一条：HS 里按 sstatus.SPP / hstatus.SPV 换模式，VS 里按 vsstatus.SPP；U / VU 里陷入 (跳过)
            self.emit("la t5, 1f")
            self.emit("csrw sepc, t5")
            self.emit("sret")
            self.emit("1:")

    # ------------------------------------------------------------------ 分页 (--vm)
    PVA = 0xC0000000                  # 探测区虚拟地址：8 个 4 KB 页 + PVA + 2 MB 处的不对齐超级页

    def pte_flags(self):
        """随机 PTE 属性：返回 (低 8 位, 保留位 [63:54] 的值)"""
        V, R, W, X, U, G, A, D = 1, 2, 4, 8, 16, 32, 64, 128
        c = self.r.random()
        hi = 0
        if c < 0.40:
            f = V | R | W | A | D | (X if self.r.random() < 0.3 else 0) | (G if self.r.random() < 0.2 else 0)
        elif c < 0.50:
            f = V | R | A                                      # 只读
        elif c < 0.57:
            f = V | R | W | D                                  # A = 0 (Svade：缺页)
        elif c < 0.64:
            f = V | R | W | A                                  # D = 0：store 缺页
        elif c < 0.71:
            f = R | W | A | D                                  # 无效
        elif c < 0.76:
            f = V | W | A | D                                  # W 无 R：保留组合
        elif c < 0.83:
            f = V | X | A                                      # 只执行 (MXR = 1 时可读)
        elif c < 0.93:
            f = V | R | W | A | D | U                          # U 页 (S 态访问看 SUM)
        else:
            f = V | R | W | A | D
            # 保留位 54 / 60、PBMT = 3 (保留)、N (4 KB 页 PPN[3:0] = 页号 0..7，不是 1000：缺页)、
            # PBMT = NC / IO (PBMTE = 1 时正常，= 0 时缺页)
            hi = self.r.choice([1 << 0, 1 << 6, 3 << 7, 1 << 9, 1 << 7, 2 << 7])
        return f, hi

    def g_vmprobe(self):
        # 访问探测区：可能正常、缺页或 PMP 访问错误
        c = self.r.random()
        if c < 0.75:
            va = self.PVA + self.r.randrange(8) * 4096
        elif c < 0.88:
            va = self.PVA + 0x10000 + self.r.randrange(16) * 4096   # NAPOT 64 KB 页
        else:
            va = self.PVA + (2 << 20)                          # 不对齐超级页
        if self.r.random() < 0.25:                             # 高层别名：只在 Sv48 / Sv57 (2^39)、Sv57 (2^48) 下规范
            va += self.r.choice([1 << 39, 1 << 48])
        if self.r.random() < 0.3:                              # 指针掩码标签：高 7 位 / 高 16 位
            va |= self.r.choice([self.r.getrandbits(7) << 57, self.r.getrandbits(16) << 48])
        op, sz, kind = self.r.choice([("ld", 8, "l"), ("lw", 4, "l"), ("lbu", 1, "l"), ("sd", 8, "s"), ("sw", 4, "s"),
                                      ("sb", 1, "s"), ("amoadd.d", 8, "a"), ("lr.d", 8, "r")])
        va += self.r.randrange(0, 4096 - 8, sz)
        self.emit(f"li t5, {va:#x}")
        if kind == "l":
            self.emit(f"{op} {x(self.rd())}, 0(t5)")
        elif kind == "s":
            self.emit(f"{op} {x(self.rs())}, 0(t5)")
        elif kind == "a":
            self.emit(f"{op} {x(self.rd())}, {x(self.rs())}, (t5)")
        else:
            self.emit(f"{op} {x(self.rd())}, (t5)")

    def g_pte(self):
        # 改写探测页 j 的 PTE (物理页不变，属性重新随机)，随后 sfence.vma (全部或只这一页)
        j = self.r.randrange(8)
        f, hi = self.pte_flags()
        self.emit("la t4, pbuf")
        self.emit(f"li t5, {j * 4096}")
        self.emit("add t4, t4, t5")
        self.emit("srli t4, t4, 12")
        self.emit("slli t4, t4, 10")
        self.emit(f"ori t4, t4, {f}")
        if hi:
            self.emit(f"li t5, {hi << 54:#x}")
            self.emit("or t4, t4, t5")
        self.emit("la t5, pt_probe_l0")
        self.emit(f"sd t4, {8 * j}(t5)")
        if self.r.random() < 0.5:
            self.emit("sfence.vma")
        else:
            self.emit(f"li t5, {self.PVA + j * 4096:#x}")
            self.emit("sfence.vma t5, x0")

    def g_sum(self):
        bit = self.r.choice([18, 19])                          # SUM / MXR
        self.emit(f"li t5, {1 << bit:#x}")
        self.emit(f"{self.r.choice(['csrs', 'csrc'])} sstatus, t5")

    # ------------------------------------------------------------------ 浮点
    RM = ["rne", "rtz", "rdn", "rup", "rmm", "dyn", "dyn", "dyn"]

    def frd(self):
        r = self.r.randrange(32)
        self.frecent = (getattr(self, "frecent", []) + [r])[-6:]
        return r

    def frs(self):
        fr = getattr(self, "frecent", [])
        if fr and self.r.random() < 0.7:
            return self.r.choice(fr)
        return self.r.randrange(32)

    def fmt(self):
        return self.r.choice(["s", "d", "d"])

    def g_fp_arith(self):
        f = self.fmt()
        op = self.r.choice(["fadd", "fsub", "fmul", "fdiv", "fsqrt", "fmin", "fmax", "fsgnj", "fsgnjn", "fsgnjx"])
        if op == "fsqrt":
            self.emit(f"fsqrt.{f} f{self.frd()}, f{self.frs()}, {self.r.choice(self.RM)}")
        elif op in ("fmin", "fmax") or op.startswith("fsgnj"):
            self.emit(f"{op}.{f} f{self.frd()}, f{self.frs()}, f{self.frs()}")
        else:
            self.emit(f"{op}.{f} f{self.frd()}, f{self.frs()}, f{self.frs()}, {self.r.choice(self.RM)}")

    def g_fp_fma(self):
        f = self.fmt()
        op = self.r.choice(["fmadd", "fmsub", "fnmadd", "fnmsub"])
        self.emit(f"{op}.{f} f{self.frd()}, f{self.frs()}, f{self.frs()}, f{self.frs()}, {self.r.choice(self.RM)}")

    def g_fp_cmp(self):
        f = self.fmt()
        op = self.r.choice(["feq", "flt", "fle", "fclass"])
        if op == "fclass":
            self.emit(f"fclass.{f} {x(self.rd())}, f{self.frs()}")
        else:
            self.emit(f"{op}.{f} {x(self.rd())}, f{self.frs()}, f{self.frs()}")

    def g_fp_cvt(self):
        f = self.fmt()
        c = self.r.random()
        rm = self.r.choice(self.RM)
        if c < 0.3:
            it = self.r.choice(["w", "wu", "l", "lu"])
            self.emit(f"fcvt.{it}.{f} {x(self.rd())}, f{self.frs()}, {rm}")
        elif c < 0.55:
            it = self.r.choice(["w", "wu", "l", "lu"])
            if f == "d" and it in ("w", "wu"):          # 精确转换，汇编器不接受舍入模式
                self.emit(f"fcvt.d.{it} f{self.frd()}, {x(self.rs())}")
            else:
                self.emit(f"fcvt.{f}.{it} f{self.frd()}, {x(self.rs())}, {rm}")
        elif c < 0.7:
            if f == "s":
                self.emit(f"fcvt.s.d f{self.frd()}, f{self.frs()}, {rm}")
            else:
                self.emit(f"fcvt.d.s f{self.frd()}, f{self.frs()}")
        elif c < 0.85:
            self.emit(f"fmv.x.{'w' if f == 's' else 'd'} {x(self.rd())}, f{self.frs()}")
        else:
            self.emit(f"fmv.{'w' if f == 's' else 'd'}.x f{self.frd()}, {x(self.rs())}")

    def g_fp_mem(self):
        c = self.r.random()
        if c < 0.3:
            self.emit(f"fld f{self.frd()}, {self.mem_off(8)}(s0)")
        elif c < 0.5:
            self.emit(f"flw f{self.frd()}, {self.mem_off(4)}(s0)")
        elif c < 0.8:
            self.emit(f"fsd f{self.frs()}, {self.mem_off(8)}(s0)")
        else:
            self.emit(f"fsw f{self.frs()}, {self.mem_off(4)}(s0)")
        if self.r.random() < 0.3:                 # 紧跟整数读同地址：测浮点 store 转发
            off = self.mem_off(8)
            self.emit(f"fsd f{self.frs()}, {off}(s0)")
            self.emit(f"ld {x(self.rd())}, {off}(s0)")

    def g_fp_csr(self):
        c = self.r.random()
        if c < 0.4:
            self.emit(f"frflags {x(self.rd())}")
        elif c < 0.6:
            self.emit(f"fsflags {x(self.rd())}, {x(self.rs())}")
        elif c < 0.8:
            self.emit(f"fsrmi {x(self.rd())}, {self.r.randrange(5)}")
        else:
            self.emit(f"frcsr {x(self.rd())}")

    # Zfa (RVA23)：fli.h 汇编器要 Zfh / Zvfh，直接给编码
    def g_fp_zfa(self):
        f = self.fmt()
        c = self.r.randrange(8)
        if c == 0:
            k = self.r.randrange(32)
            if self.r.random() < 0.2:
                rd = self.frd()
                self.emit(f".word {(0b1111010 << 25) | (1 << 20) | (k << 15) | (rd << 7) | 0x53:#010x}   # fli.h f{rd}, {k}")
            else:
                self.emit(f".word {(0b11110 << 27) | ((f == 'd') << 25) | (1 << 20) | (k << 15) | (self.frd() << 7) | 0x53:#010x}")
        elif c == 1:
            self.emit(f"{self.r.choice(['fminm', 'fmaxm'])}.{f} f{self.frd()}, f{self.frs()}, f{self.frs()}")
        elif c in (2, 3):
            self.emit(f"{self.r.choice(['fround', 'froundnx'])}.{f} f{self.frd()}, f{self.frs()}, {self.r.choice(self.RM)}")
        elif c == 4:
            self.emit(f"fcvtmod.w.d {x(self.rd())}, f{self.frs()}, rtz")
        else:
            self.emit(f"{self.r.choice(['fleq', 'fltq'])}.{f} {x(self.rd())}, f{self.frs()}, f{self.frs()}")

    # ------------------------------------------------------------------ CSR 随机访问
    # (地址, 写入前要清掉的位)：M 态运行，屏蔽会改变控制流/特权的位
    CSR_RW = [
        (0x340, 0), (0x140, 0),                    # mscratch sscratch
        (0x141, 0), (0x142, 0), (0x143, 0),        # sepc scause stval
        (0x341, 0), (0x342, 0), (0x343, 0),        # mepc mcause mtval
        (0x306, 0), (0x106, 0),                    # mcounteren scounteren
        (0x320, 0),                                # mcountinhibit
        (0x323, 0), (0x324, 0), (0x325, 0), (0x326, 0),   # mhpmevent3-6
        (0x302, 0), (0x303, 0),                    # medeleg mideleg (M 态陷入不受委托影响)
        (0x304, 0), (0x104, 0),                    # mie sie (MIE=0，不会真进中断)
        (0x144, 0),                                # sip (只能写 SSIP)
        (0x30A, 0), (0x10A, 0),                    # menvcfg senvcfg
        (0x180, 0),                                # satp (M 态取指/访存不翻译)
        (0x3A0, 0x8080808080808080),               # pmpcfg0：不置 L 位 (否则 M 态也受限)
        (0x3B0, 0), (0x3B1, 0), (0x3B2, 0), (0x3B3, 0),
        (0x3B4, 0), (0x3B5, 0), (0x3B6, 0), (0x3B7, 0),   # pmpaddr0-7
        (0x300, (1 << 3) | (1 << 1) | (1 << 17)),  # mstatus：不开 MIE/SIE，不开 MPRV
        (0x100, (1 << 1)),                         # sstatus：不开 SIE
        (0x7A0, 0), (0x7A1, 0), (0x7A8, 0),        # tselect tdata1 mcontext
        # RVA23 R3：Sstc / Smstateen / Sscofpmf (mhpmevent 的 OF / xINH 位在上面 0x323..0x326)；mip 的 STIP 在 STCE = 1 时不可写
        (0x14D, 0),                                # stimecmp
        (0x30C, 0), (0x30D, 0), (0x30E, 0), (0x30F, 0),   # mstateen0-3
        (0x10C, 0), (0x10D, 0), (0x10E, 0), (0x10F, 0),   # sstateen0-3
        (0x344, 0),                                # mip (MIE = 0，置了也不会进中断)
    ]
    CSR_RO = [0x301, 0xF11, 0xF12, 0xF13, 0xF14, 0xF15, 0xC02, 0xB02, 0x7A4, 0xDA0]

    def g_csr_rand(self):
        c = self.r.random()
        if c < 0.55:
            a, clr = self.r.choice(self.CSR_RW)
            op = self.r.choice(["csrrw", "csrrs", "csrrc"])
            src = x(self.rs())
            if clr:
                self.emit(f"li t5, {clr:#x}")
                self.emit(f"not t5, t5")
                self.emit(f"and t5, t5, {src}")
                src = "t5"
                if op == "csrrc":
                    op = "csrrs"           # 清位操作对 csrrc 无害，这里统一成只置允许的位
            self.emit(f"{op} {x(self.rd())}, {a:#x}, {src}")
        elif c < 0.75:
            self.emit(f"csrr {x(self.rd())}, {self.r.choice(self.CSR_RO):#x}")
        else:
            # 随机地址：多数未实现 -> 非法指令陷入 (陷入处理跳过)；读不写，避免改到真实 CSR
            a = self.r.randrange(0x1000)
            if a in (0x7B0, 0x7B1, 0x7B2, 0x7B3):  # 调试 CSR：两边都非法，留着也行
                pass
            self.emit(f"csrrs {x(self.rd())}, {a:#x}, x0")

    SIMPLE = ["alu", "alu_imm", "lui_auipc", "mul", "load", "store"]

    def g_simple(self):
        getattr(self, "g_" + self.r.choice(self.SIMPLE))()

    # ------------------------------------------------------------------ 输出
    def run(self):
        cats = list(self.w.keys())
        wts = [self.w[c] for c in cats]
        for _ in range(self.n):
            getattr(self, "g_" + self.r.choices(cats, wts)[0])()
        body = self.lines
        self.lines = []
        for f in self.funcs:
            self.lines.append(f"{f}:")
            for _ in range(self.r.randint(1, 5)):
                self.g_simple()
            self.emit("ret")
        funcs = self.lines
        return body, funcs


CSR_W = {"csr_rand": 6}

VM_W = {"vmprobe": 6, "pte": 2, "sum": 1}

RVA23_W = {"rva23": 8}

HYP_W = {"hyp_csr": 8, "hyp_ls": 5, "hyp_misc": 3, "gprobe": 4}
HVM_W = {"hyp_ls": 3, "hyp_misc": 2, "gprobe": 6}         # --hyp --vm：VS 主体，不写 H CSR

GALIAS = 0x100000000
# G-PTE 属性 (V R W X U A D = 0xDF 正常)：U = 0、W 无 R、只 X、无 W、无 X、A = 0、D = 0、无效；高位：保留位 / PBMT / N
GFLAGS = [0xDF, 0xDF, 0xDF, 0xDF, 0xCF, 0xDD, 0xD9, 0xDB, 0xD7, 0x9F, 0x5F, 0xDE]
GHI = [0, 0, 0, 0, 0, 0, 1 << 54, 3 << 61, 1 << 63, 1 << 61]


def g_tables(e, gmode, r):
    """hgatp (Sv39x4 / 48x4 / 57x4) 页表：DRAM 恒等 1 GB 叶子，GPA 0x1_0000_0000 起 9 个 4 KB 探测页 (0..7 -> pbuf，8 -> vpt_g)"""
    def gptr(tbl, idx, nxt):
        e(f"    la    t5, {nxt}")
        e("    srli  t5, t5, 12")
        e("    slli  t5, t5, 10")
        e("    ori   t5, t5, 1")
        e(f"    la    t4, {tbl}")
        e(f"    li    t6, {idx * 8}")
        e("    add   t4, t4, t6")
        e("    sd    t5, 0(t4)")
    dram = "g_root"
    if gmode >= 9:
        gptr("g_root", 0, "g_lv3" if gmode == 10 else "g_lv2")
        dram = "g_lv2"
    if gmode == 10:
        gptr("g_lv3", 0, "g_lv2")
    for idx, pa in ((2, 0x80000000), (3, 0xC0000000)):
        e(f"    li    t5, {((pa >> 12) << 10) | 0xDF:#x}")
        e(f"    la    t4, {dram}")
        e(f"    sd    t5, {idx * 8}(t4)")
    gptr(dram, 4, "g_lv1")
    gptr("g_lv1", 0, "g_lv0")
    desc = []
    for j in range(9):
        f = r.choice(GFLAGS)
        hi = r.choice(GHI)
        desc.append(f"{f:#x}" + (f"|{hi:#x}" if hi else ""))
        if j < 8:
            e("    la    t5, pbuf")
            e(f"    li    t4, {j * 4096}")
            e("    add   t5, t5, t4")
        else:
            e("    la    t5, vpt_g")
        e("    srli  t5, t5, 12")
        e("    slli  t5, t5, 10")
        e(f"    ori   t5, t5, {f}")
        if hi:
            e(f"    li    t4, {hi:#x}")
            e("    or    t5, t5, t4")
        e("    la    t4, g_lv0")
        e(f"    sd    t5, {8 * j}(t4)")
    e("    la    t5, g_root")
    e("    srli  t5, t5, 12")
    e(f"    li    t4, {(gmode << 60) | (r.randrange(1 << 14) << 44):#x}")
    e("    or    t5, t5, t4")
    e("    csrw  0x680, t5")
    e("    .word 0x62000073")                                  # hfence.gvma x0, x0
    return "G-PTE " + ",".join(desc)

FP_W = {"fp_arith": 10, "fp_fma": 5, "fp_cmp": 4, "fp_cvt": 6, "fp_mem": 5, "fp_csr": 1}

DEFAULT_W = {
    "alu": 20, "alu_imm": 20, "lui_auipc": 3, "mul": 6, "div": 2, "load": 12, "store": 8,
    "store_load_pair": 6, "branch_fwd": 8, "loop": 2, "call": 2, "indirect": 1, "amo": 2,
    "lrsc": 1, "csr": 1, "trap": 1,
}

HEADER = """\
    .section .text.init
    .globl _start
_start:
    la    t5, trap_handler
    csrw  mtvec, t5
    la    s0, data_area
{init}
{finit}
    j     body

    .align 2
trap_handler:
    # 跳过出错指令：mepc += 指令长度 (按 mepc 处的编码判断 2 / 4 字节)
    csrr  t4, mepc
    lhu   t5, 0(t4)
    andi  t5, t5, 3
    addi  t4, t4, 2
    addi  t5, t5, -3
    bnez  t5, 1f
    addi  t4, t4, 2
1:  csrw  mepc, t4
    mret

body:
"""

FOOTER = """\
    li    t5, 1
    la    t4, tohost
    sd    t5, 0(t4)
1:  j     1b

{funcs}

    .data
    .align 6
data_area:
{data}

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
"""

def vm_prologue(r, init, finit, vmode=8, gmode=0):
    """--vm 的 M 态开场：随机 PMP、页表、satp (MODE = vmode：8 / 9 / 10)，然后 mret 进 S 态主体"""
    L = []
    e = L.append
    e("    la    t5, trap_handler")
    e("    csrw  mtvec, t5")
    e("    li    t5, 7")
    e("    csrw  mcounteren, t5")
    # ---- 随机 PMP (项 0..n-1 落在 pbuf 内，按地址递增；项 7 放行全部)
    n = r.randint(1, 4)
    cfg = 0
    spots = sorted((r.randrange(8), r.randrange(4096 // 8)) for _ in range(n))
    desc = []
    for i, (j, w) in enumerate(spots):
        # PMP 粒度 64 B (o6_pkg::PMP_G = 4)：不用 NA4 (会被转成 NAPOT，尾部 1 可能把区域扩到 pbuf 外面)，
        #   NAPOT >= 64 B，TOR 边界 64 B 对齐。项 0 不能用 TOR (下界为 0，会把代码与页表也包进去)
        mode = r.choice(["napot", "napot"] + (["tor"] if i > 0 else []))
        perm = r.choice([0, 1, 3, 4, 5, 7])                   # 无 / R / RW / X / RX / RWX (W 无 R 是保留组合)
        if mode == "napot":
            k = r.randint(6, 12)
            size = 1 << k
            off = j * 4096 + r.randrange(0, 4096, size)
            e(f"    la    t5, pbuf")
            e(f"    li    t4, {off + size // 2 - 1}")
            e("    add   t5, t5, t4")
            e("    srli  t5, t5, 2")
            a = 3
            desc.append(f"pmp{i} NAPOT pbuf+{off:#x} size {size} perm {perm}")
        else:
            off = j * 4096 + 64 * (w % 64)
            e(f"    la    t5, pbuf")
            e(f"    li    t4, {off}")
            e("    add   t5, t5, t4")
            e("    srli  t5, t5, 2")
            a = 2 if mode == "na4" else 1
            desc.append(f"pmp{i} {mode.upper()} pbuf+{off:#x} perm {perm}")
        e(f"    csrw  pmpaddr{i}, t5")
        cfg |= ((a << 3) | perm) << (8 * i)
    cfg |= 0x1F << 56
    e("    li    t5, -1")
    e("    csrw  pmpaddr7, t5")
    e(f"    li    t5, {cfg:#x}")
    e("    csrw  pmpcfg0, t5")
    # ---- 页表：根表第 2 项 = 代码区 (恒等)，第 1 项 = 数据别名，第 3 项 = 探测区
    def ptr(tbl, idx, nxt):                                    # tbl[idx] = 指向 nxt 的非叶 PTE
        e(f"    la    t5, {nxt}")
        e("    srli  t5, t5, 12")
        e("    slli  t5, t5, 10")
        e("    ori   t5, t5, 1")
        e(f"    la    t4, {tbl}")
        e(f"    sd    t5, {8 * idx}(t4)")
    cmap = r.choice(["giga", "mega", "page", "page"])
    if cmap == "giga":
        e(f"    li    t5, {(0x80000 << 10) | 0xCF:#x}")
        e("    la    t4, pt_root")
        e("    sd    t5, 16(t4)")
    else:
        ptr("pt_root", 2, "pt_code_l1")
        if cmap == "mega":
            e(f"    li    t5, {(0x80000 << 10) | 0xCF:#x}")
            e("    la    t4, pt_code_l1")
            e("    sd    t5, 0(t4)")
        else:
            ptr("pt_code_l1", 0, "pt_code_l0")
            e("    la    t4, pt_code_l0")                    # 512 个 4 KB 页恒等映射 0x8000_0000 起 2 MB
            e(f"    li    t5, {(0x80000 << 10) | 0xCF:#x}")
            e("    li    t6, 512")
            e("    li    a0, 1024")
            e("1:  sd    t5, 0(t4)")
            e("    addi  t4, t4, 8")
            e("    add   t5, t5, a0")
            e("    addi  t6, t6, -1")
            e("    bnez  t6, 1b")
    dmap = r.choice(["giga", "mega", "page"])
    if dmap == "giga":
        e(f"    li    t5, {(0x80000 << 10) | 0xC7:#x}")
        e("    la    t4, pt_root")
        e("    sd    t5, 8(t4)")
    else:
        ptr("pt_root", 1, "pt_data_l1")
        if dmap == "mega":
            e("    la    t5, data_area")
            e("    srli  t5, t5, 21")
            e("    slli  t5, t5, 19")
            e("    ori   t5, t5, 0xC7")
            e("    la    t4, pt_data_l1")
            e("    sd    t5, 0(t4)")
        else:
            ptr("pt_data_l1", 0, "pt_data_l0")
            e("    la    t5, data_area")
            e("    srli  t5, t5, 12")
            e("    slli  t5, t5, 10")
            e("    ori   t5, t5, 0xC7")
            e("    la    t4, pt_data_l0")
            e("    sd    t5, 0(t4)")
    ptr("pt_root", 3, "pt_probe_l1")
    ptr("pt_probe_l1", 0, "pt_probe_l0")
    # NAPOT 64 KB：pt_probe_l0[16..31] 同一个 PTE，PPN = pbuf 页号 | 8 (pbuf 64 KB 对齐)
    e("    la    t5, pbuf")
    e("    srli  t5, t5, 12")
    e("    ori   t5, t5, 8")
    e("    slli  t5, t5, 10")
    e("    ori   t5, t5, 0xC7")
    e("    li    t4, 1")
    e("    slli  t4, t4, 63")
    e("    or    t5, t5, t4")
    e("    la    t4, pt_probe_l0")
    e("    addi  t4, t4, 128")
    e("    li    t6, 16")
    e("2:  sd    t5, 0(t4)")
    e("    addi  t4, t4, 8")
    e("    addi  t6, t6, -1")
    e("    bnez  t6, 2b")
    # Sv48 / Sv57 的上层表：项 0 与项 1 都指下一级 (VA + 2^39 / 2^48 是别名)
    top = "pt_root"
    if vmode >= 9:
        ptr("pt_l3", 0, "pt_root")
        ptr("pt_l3", 1, "pt_root")
        top = "pt_l3"
    if vmode == 10:
        ptr("pt_l4", 0, "pt_l3")
        ptr("pt_l4", 1, "pt_l3")
        top = "pt_l4"
    pbmte = r.random() < 0.5
    pmm = r.choice([0, 2, 2, 3])
    if pbmte or pmm:
        e(f"    li    t5, {(int(pbmte) << 62) | (pmm << 32):#x}")
        e("    csrs  menvcfg, t5")
        if gmode:
            e("    csrs  0x60a, t5")                         # henvcfg：VS-stage 的 PBMTE / PMM
    gdesc = ""
    if gmode:
        # VS 映射到 G 探测页：PVA + 128 KB 起 8 页 -> GPA 0x1_0000_0000 + j * 4 KB；VA 0x1_0000_0000 的下一级表在 G 探测页 8
        for j in range(8):
            e(f"    li    t5, {(((GALIAS + j * 4096) >> 12) << 10) | 0xCF:#x}")
            e("    la    t4, pt_probe_l0")
            e(f"    sd    t5, {8 * (32 + j)}(t4)")
        e(f"    li    t5, {(((GALIAS + 8 * 4096) >> 12) << 10) | 0x1:#x}")
        e("    la    t4, pt_root")
        e("    sd    t5, 32(t4)")
        gdesc = g_tables(e, gmode, r)
    g = Gen(r.getrandbits(32), 0, {})
    for j in range(8):
        f, hi = g.pte_flags()
        e("    la    t5, pbuf")
        e(f"    li    t4, {j * 4096}")
        e("    add   t5, t5, t4")
        e("    srli  t5, t5, 12")
        e("    slli  t5, t5, 10")
        e(f"    ori   t5, t5, {f}")
        if hi:
            e(f"    li    t4, {hi << 54:#x}")
            e("    or    t5, t5, t4")
        e("    la    t4, pt_probe_l0")
        e(f"    sd    t5, {8 * j}(t4)")
    # PVA + 2 MB：PPN 不对齐的 2 MB 叶 (pbuf 不在 2 MB 边界上，必缺页)
    e("    la    t5, pbuf")
    e("    srli  t5, t5, 12")
    e("    ori   t5, t5, 1")                                    # 保证 PPN[0] = 1 (不对齐)
    e("    slli  t5, t5, 10")
    e("    ori   t5, t5, 0xC7")
    e("    la    t4, pt_probe_l1")
    e("    sd    t5, 8(t4)")
    # ---- satp (ASID 随机)、SUM / MXR 初值
    e(f"    la    t5, {top}")
    e("    srli  t5, t5, 12")
    e(f"    li    t4, {(vmode << 60) | (r.randrange(1 << 16) << 44):#x}")
    e("    or    t5, t5, t4")
    e("    csrw  0x280, t5" if gmode else "    csrw  satp, t5")   # 两级：改写 vsatp
    e("    sfence.vma")
    sm = (1 << 18 if r.random() < 0.5 else 0) | (1 << 19 if r.random() < 0.3 else 0)
    if sm:
        e(f"    li    t5, {sm:#x}")
        e("    csrs  0x200, t5" if gmode else "    csrs  mstatus, t5")   # 两级：vsstatus 的 SUM / MXR
    if gmode and r.random() < 0.3:
        e("    li    t5, 1 << 19")
        e("    csrs  mstatus, t5")                           # mstatus.MXR：两级都生效
    # ---- 浮点初值 (物理地址取)、通用寄存器初值、s0 = 数据区的虚拟别名
    e("    la    s0, data_area")
    if finit:
        e(finit)
    e(init)
    e("    la    t5, data_area")
    if dmap == "giga":
        e("    li    t4, 0x80000000")
        e("    sub   t5, t5, t4")
    elif dmap == "mega":
        e("    li    t4, 0x1fffff")
        e("    and   t5, t5, t4")
    else:
        e("    li    t5, 0")
    e("    li    s0, 0x40000000")
    e("    add   s0, s0, t5")
    # ---- 进 S 态主体
    e("    la    t5, body")
    e("    csrw  mepc, t5")
    e("    li    t5, 3 << 11")
    e("    csrc  mstatus, t5")
    e("    li    t5, 1 << 11")
    e("    csrs  mstatus, t5")
    if gmode:
        e("    li    t5, 1")
        e("    slli  t5, t5, 39")
        e("    csrs  mstatus, t5")                           # MPV = 1：进 VS
    e("    mret")
    hdr = ["    .section .text.init", "    .globl _start", "_start:",
           f"    # {'VS 两级 hgatp.MODE ' + str(gmode) + '，' + gdesc + '，' if gmode else ''}satp.MODE {vmode}，PBMTE {int(pbmte)}，PMM {pmm}，代码映射 {cmap}，数据别名 {dmap}，SUM/MXR = {sm:#x}；" + "；".join(desc)] + L
    return "\n".join(hdr) + "\n" + (TRAP_BODY_H if gmode else TRAP_BODY)


def hyp_prologue(r, init, finit, fp, gmode=0):
    """--hyp 的 M 态开场：PMP 全开、随机委托 / hstatus / 计数器使能，mret 进主体模式。返回 (代码, 主体模式, vtsr)"""
    L = []
    e = L.append
    mode = r.choice(["HS", "HS", "VS", "VS", "VU", "U"])
    vtsr = r.random() < 0.2
    e("    la    t5, trap_handler")
    e("    csrw  mtvec, t5")
    e("    la    t5, s_handler")
    e("    csrw  stvec, t5")
    e("    csrw  0x205, t5")                                 # HS 与 VS 共用 (V = 1 时 sepc / sret 自动走 vs* 那套)
    e("    li    t5, -1")
    e("    csrw  pmpaddr0, t5")
    e("    li    t5, 0x1f")
    e("    csrw  pmpcfg0, t5")
    md = sum(1 << b for b in [2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 20, 21, 22, 23] if r.random() < 0.5)
    hd = 0 if vtsr else sum(1 << b for b in [0, 2, 3, 4, 5, 6, 7, 8, 12, 13, 15] if r.random() < 0.5)
    hs = (int(vtsr) << 22) | (int(r.random() < 0.3) << 21) | (int(r.random() < 0.3) << 20) | \
         (int(r.random() < 0.5) << 9) | (int(r.random() < 0.5) << 8) | (int(r.random() < 0.3) << 7)
    e(f"    li    t5, {md:#x}")
    e("    csrw  medeleg, t5")
    e(f"    li    t5, {hd:#x}")
    e("    csrw  0x602, t5")
    e(f"    li    t5, {hs:#x}")
    e("    csrw  0x600, t5")
    e("    li    t5, 7")
    e("    csrw  mcounteren, t5")
    e(f"    li    t5, {r.choice([0, 7, 5, 2]):#x}")
    e("    csrw  0x606, t5")
    e(f"    li    t5, {r.choice([0, 7, 1]):#x}")
    e("    csrw  scounteren, t5")
    # menvcfg / henvcfg 的 CBO 位与 mstateen0 (SE0 / ENVCFG) 随机：VS 下 cbo / senvcfg / sstateen 走 virtual 判定
    e(f"    li    t5, {(r.getrandbits(1) << 7) | (r.getrandbits(1) << 6) | (r.choice([0, 1, 3]) << 4):#x}")
    e("    csrw  menvcfg, t5")
    e(f"    li    t5, {(r.getrandbits(1) << 7) | (r.getrandbits(1) << 6) | (r.choice([0, 1, 3]) << 4):#x}")
    e("    csrw  0x60a, t5")
    e(f"    li    t5, {(r.getrandbits(1) << 63) | (r.getrandbits(1) << 62):#x}")
    e("    csrw  0x30c, t5")
    e(f"    li    t5, {(r.getrandbits(1) << 63) | (r.getrandbits(1) << 62):#x}")
    e("    csrw  0x60c, t5")
    gdesc = g_tables(e, gmode, r) if gmode else ""
    e("    la    s0, data_area")
    if finit:
        e(finit)
        e("    li    t5, 0x2000")
        e("    csrs  0x200, t5")                           # vsstatus.FS = Initial (否则 VS 里浮点非法)
    e(init)
    # 进主体：MPP / MPV
    mpp, mpv = {"HS": (1, 0), "VS": (1, 1), "VU": (0, 1), "U": (0, 0)}[mode]
    e("    la    t5, body")
    e("    csrw  mepc, t5")
    e("    li    t5, 3 << 11")
    e("    csrc  mstatus, t5")
    if mpp:
        e("    li    t5, 1 << 11")
        e("    csrs  mstatus, t5")
    if mpv:
        e("    li    t5, 1")
        e("    slli  t5, t5, 39")
        e("    csrs  mstatus, t5")
    e("    mret")
    hdr = ["    .section .text.init", "    .globl _start", "_start:",
           f"    # --hyp：主体模式 {mode}，medeleg {md:#x}，hedeleg {hd:#x}，hstatus {hs:#x}，hgatp.MODE {gmode} {gdesc}"] + L
    return "\n".join(hdr) + "\n" + S_HANDLER + TRAP_BODY_H, mode, vtsr


S_HANDLER = """
    .align 2
s_handler:
    # HS / VS 的陷入处理：sepc += 指令长度 (V = 1 时即 vsepc / vsstatus；vsatp / hgatp = Bare，按地址直接读编码)
    csrr  t4, sepc
    lhu   t5, 0(t4)
    andi  t5, t5, 3
    addi  t4, t4, 2
    addi  t5, t5, -3
    bnez  t5, 1f
    addi  t4, t4, 2
1:  csrw  sepc, t4
    sret
"""

TRAP_BODY = """
    .align 2
trap_handler:
    # 跳过出错指令：mepc += 指令长度 (按 mepc 处的编码判断 2 / 4 字节；代码区恒等映射，M 态按物理地址读即可)
    csrr  t4, mepc
    lhu   t5, 0(t4)
    andi  t5, t5, 3
    addi  t4, t4, 2
    addi  t5, t5, -3
    bnez  t5, 1f
    addi  t4, t4, 2
1:  csrw  mepc, t4
    mret

body:
"""

# H 模式的 M 态陷入处理：先读 mtval2 / mtinst / mstatus (GVA、MPV) —— 对拍按 CSR 读值逐次比对 (每次陷入都核对这些字段)
TRAP_BODY_H = TRAP_BODY.replace("trap_handler:\n",
                                "trap_handler:\n    csrr  t5, 0x34b\n    csrr  t5, 0x34a\n    csrr  t5, mstatus\n", 1)

VM_TABLES = """
    .align 12
pt_root:     .zero 4096
pt_code_l1:  .zero 4096
pt_code_l0:  .zero 4096
pt_data_l1:  .zero 4096
pt_data_l0:  .zero 4096
pt_probe_l1: .zero 4096
pt_probe_l0: .zero 4096
pt_l3:       .zero 4096
pt_l4:       .zero 4096
    .align 14
g_root:      .zero 16384
g_lv3:       .zero 4096
g_lv2:       .zero 4096
g_lv1:       .zero 4096
g_lv0:       .zero 4096
vpt_g:       .zero 4096
    .align 16
pbuf:        .zero 65536
    .align 12
"""

LINKER = """\
OUTPUT_ARCH("riscv")
ENTRY(_start)
SECTIONS
{
  . = 0x80000000;
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .tohost : { *(.tohost) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
  .data : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--ld", help="同时写出链接脚本")
    ap.add_argument("--only", help="只用这些类别 (逗号分隔)")
    ap.add_argument("--fp", action="store_true", help="加入浮点类别")
    ap.add_argument("--csr", action="store_true", help="加入 CSR 随机访问类别")
    ap.add_argument("--vm", action="store_true", help="主体在 S 态 Sv39 下跑：随机页表 + 随机 PMP")
    ap.add_argument("--hyp", action="store_true", help="H 扩展：主体在 HS / VS / VU / U 下跑 (随机委托)，加 H 指令 / CSR (as 需 _h)")
    ap.add_argument("--rva23", action="store_true", help="加入 RVA23 新增指令 (Zicond / Zimop / Zawrs / Zcb / Zcmop；as 需对应 -march)")
    a = ap.parse_args()
    w = dict(DEFAULT_W)
    if a.fp:
        w.update(FP_W)
    if a.csr:
        w.update(CSR_W)
    if a.vm:
        w.update(VM_W)
    if a.hyp:
        w.update(HVM_W if a.vm else HYP_W)
    if a.rva23:
        w.update(RVA23_W)
        if a.fp:
            w["fp_zfa"] = 5
    if a.only:
        keep = set(a.only.split(","))
        w = {k: v for k, v in w.items() if k in keep}
    g = Gen(a.seed, a.n, w)
    g.vm = a.vm
    g.hyp = a.hyp
    g.hvm = a.hyp and a.vm
    # hgatp.MODE：--hyp --vm 必开 (两级)，单 --hyp 随机
    gmode = random.Random(a.seed ^ 0x6767).choice([8, 9, 10] if a.vm else [0, 0, 8, 9, 10])
    g.gmode = gmode
    if not a.hyp:
        gmode = 0
    hrr = random.Random(a.seed ^ 0x4848)
    if a.hyp and not a.vm:
        hpro, hmode, g.vtsr = hyp_prologue(hrr, "", "", False, gmode)   # 先定 vtsr (主体生成要用)；代码下面按真 init 再生成一次
    vmode = random.Random(a.seed ^ 0x77).choice([8, 9, 10])   # --vm：Sv39 / Sv48 / Sv57
    body, funcs = g.run()
    rr = random.Random(a.seed ^ 0x5a5a)
    init = "\n".join(f"    li    {x(r)}, {rr.getrandbits(64):#x}" for r in WREGS)
    SPECIAL = [0x0, 0x8000000000000000, 0x7ff0000000000000, 0xfff0000000000000, 0x7ff8000000000000,
               0x7ff4000000000000, 0x0000000000000001, 0x000fffffffffffff, 0x3ff0000000000000,
               0xbff0000000000000, 0x4340000000000000, 0x43e0000000000000, 0xc3e0000000000000,
               0xffffffff7f800000, 0xffffffff00000000, 0xffffffff7fc00000, 0xffffffff00000001,
               0xffffffff3f800000, 0xffffffffcf000000, 0x000000003f800000, 0xffffffff4f800000]

    def dval():
        c = rr.random()
        if c < 0.3:
            return rr.choice(SPECIAL)
        if c < 0.55:                              # 有限双精度，指数在常用范围
            return (rr.getrandbits(1) << 63) | (rr.randint(0x3c0, 0x440) << 52) | rr.getrandbits(52)
        if c < 0.8:                               # NaN-box 的单精度
            return 0xffffffff00000000 | (rr.getrandbits(1) << 31) | (rr.randint(0x60, 0xa0) << 23) | rr.getrandbits(23)
        return rr.getrandbits(64)
    data = "\n".join(f"    .dword {dval():#x}" for _ in range(DATA_SIZE // 8))
    finit = ""
    if a.fp:
        finit = "    li    t5, 0x6000\n    csrs  mstatus, t5\n" + \
                "\n".join(f"    fld   f{i}, {8 * rr.randrange(DATA_SIZE // 8)}(s0)" for i in range(32))
    with open(a.out, "w", encoding="utf-8") as f:
        if a.hyp and not a.vm:
            f.write(hyp_prologue(random.Random(a.seed ^ 0x4848), init, finit, a.fp, gmode)[0])
        elif a.vm:
            f.write(vm_prologue(random.Random(a.seed ^ 0x1234), init, finit, vmode, gmode))
        else:
            f.write(HEADER.format(init=init, finit=finit))
        f.write("\n".join(body) + "\n")
        foot = FOOTER.format(funcs="\n".join(funcs), data=data)
        if a.vm or a.hyp:                                    # 页表 / 探测缓冲区放在 .data 开头 (4 KB 对齐)
            foot = foot.replace("    .data\n    .align 6\ndata_area:", "    .data" + VM_TABLES + "data_area:")
        f.write(foot)
    if a.ld:
        with open(a.ld, "w") as f:
            f.write(LINKER)


if __name__ == "__main__":
    main()
