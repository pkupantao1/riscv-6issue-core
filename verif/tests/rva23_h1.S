# rva23_h1.S - RVA23 R5 (H 扩展，docs/14、docs/15) 的定向自检：随机测试 (rvgen --hyp) 不开中断使能，这里补
#   1. VSSI 委托到 VS：hideleg = 0x444，M 态先置 hvip.VSSIP，进 VS (vsstatus.SIE、vsie.SSIE)，第一条就该进 VS 处理，
#      vscause (经 scause 读) = 中断 | 1；处理里 csrc sip.SSIP (即 mip.VSSIP)
#   2. VSTI (Sstc)：menvcfg / henvcfg.STCE、mcounteren / hcounteren.TM，VS 里经 stimecmp 地址写 vstimecmp = time + 100，
#      等 VS 定时器中断 (vscause = 中断 | 5)，处理里 vstimecmp = -1
#   3. VSEI 不委托 (hideleg = 0)：V = 1 时 HS 组中断总使能，进 HS 处理，scause = 中断 | 10；处理里清 hvip.VSEIP
#   4. MPRV + MPV：M 态 MPRV = 1、MPP = S、MPV = 1，hgatp = Sv39x4 (DRAM 恒等 1 GB)：普通 load 走两级翻译读到数据；
#      读 GPA 0x1_0000_0000 (G 根表项无效) -> load guest page fault (21)：mtval = GVA，mtval2 = GPA >> 2，mstatus.GVA = 1，mtinst = 0
#      另验 hgatp 写非法 MODE (11) 时 MODE 保持、PPN 照写 (Spike 行为)
#   中断时刻由 DUT 决定 (对拍时注入 Spike)；自检只用 DUT 自己读到的值。处理程序只用 t5 / t6
    .section .text.init
    .globl _start
_start:
    la    t0, m_trap
    csrw  mtvec, t0
    la    t0, hs_trap
    csrw  stvec, t0
    la    t0, vs_trap
    csrw  0x205, t0                # vstvec
    li    t0, -1                   # PMP 项 0：全地址 RWX
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    s2, 0                    # VSSI 到了
    li    s3, 0                    # VSTI 到了
    li    s4, 0                    # VSEI (HS) 到了
    li    s5, 0                    # guest page fault 次数

    # ---------------- 1. VSSI -> VS
    li    t0, 0x444
    csrw  0x603, t0                # hideleg：VS 三种中断都委托给 VS
    csrsi 0x200, 2                 # vsstatus.SIE
    li    t0, 2
    csrw  0x204, t0                # vsie.SSIE (= mie.VSSIE)
    li    t0, 4
    csrs  0x645, t0                # hvip.VSSIP
    la    s6, m_cont1
    la    t0, vs1
    jal   ra, to_vs
vs1:
    li    t2, 100000
1:  bnez  s2, 2f
    addi  t2, t2, -1
    bnez  t2, 1b
    j     fail
2:  ecall                          # VS ecall (cause 10) -> M (m_cont1)
m_cont1:
    csrr  t0, 0x645
    andi  t0, t0, 4
    bnez  t0, fail                 # VS 处理里清过 VSSIP

    # ---------------- 2. VSTI -> VS (Sstc)
    li    t0, -1
    csrw  0x24d, t0                # vstimecmp = 最大
    li    t0, 1
    slli  t0, t0, 63
    csrs  0x30a, t0                # menvcfg.STCE
    csrs  0x60a, t0                # henvcfg.STCE
    li    t0, 7
    csrw  mcounteren, t0
    csrw  0x606, t0                # hcounteren
    li    t0, 0x20
    csrw  0x204, t0                # vsie.STIE (= mie.VSTIE)
    la    s6, m_cont2
    la    t0, vs2
    jal   ra, to_vs
vs2:
    rdtime t1                      # V = 1：time + htimedelta
    addi  t1, t1, 100
    csrw  0x14d, t1                # V = 1：stimecmp 地址即 vstimecmp
    li    t2, 200000
1:  bnez  s3, 2f
    addi  t2, t2, -1
    bnez  t2, 1b
    j     fail
2:  ecall
m_cont2:
    csrw  0x204, zero

    # ---------------- 3. VSEI 不委托 -> HS
    csrw  0x603, zero              # hideleg = 0
    li    t0, 0x400
    csrs  0x604, t0                # hie.VSEIE
    csrs  0x645, t0                # hvip.VSEIP
    la    s6, m_cont3
    la    t0, vs3
    jal   ra, to_vs
vs3:
    li    t2, 100000
1:  bnez  s4, 2f
    addi  t2, t2, -1
    bnez  t2, 1b
    j     fail
2:  ecall
m_cont3:
    li    t0, 0x400
    csrc  0x604, t0

    # ---------------- 4. MPRV + MPV，G-stage (Sv39x4)
    li    t0, 0x200000df           # G 根表项 2：GPA 0x8000_0000 起 1 GB -> PA 恒等 (V R W X U A D)
    la    t1, g_root
    sd    t0, 16(t1)
    srli  t1, t1, 12
    li    t0, 8
    slli  t0, t0, 60
    or    t0, t0, t1
    csrw  0x680, t0                # hgatp = Sv39x4
    .word 0x62000073               # hfence.gvma
    csrr  t2, 0x680
    bne   t2, t0, fail
    # hgatp 写非法 MODE：MODE 保持，PPN 照写
    li    t3, 11
    slli  t3, t3, 60
    or    t3, t3, t1
    addi  t3, t3, 4                # PPN 改一下 (低 2 位恒 0：加 4)
    csrw  0x680, t3
    csrr  t4, 0x680
    srli  t5, t4, 60
    li    t6, 8
    bne   t5, t6, fail
    addi  t1, t1, 4
    slli  t4, t4, 20
    srli  t4, t4, 20
    bne   t4, t1, fail
    csrw  0x680, t0                # 恢复
    .word 0x62000073
    # MPRV = 1、MPP = S、MPV = 1：M 态的数据访存按 VS 两级翻译
    la    s7, gdata
    li    t0, 0x5a5a1234
    sd    t0, 0(s7)
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, 1 << 11
    csrs  mstatus, t0              # MPP = S
    li    t0, 1
    slli  t0, t0, 39
    csrs  mstatus, t0              # MPV
    li    t0, 1 << 17
    csrs  mstatus, t0              # MPRV
    ld    t1, 0(s7)                # 两级翻译 (GPA = PA)
    li    t0, 0x5a5a1234
    bne   t1, t0, fail_mprv
    li    s8, 1
    slli  s8, s8, 32               # GPA 0x1_0000_0000：G 根表项 4 无效
    ld    t1, 0(s8)                # -> load guest page fault (m_trap 跳过)
    li    t0, 1 << 17
    csrc  mstatus, t0              # MPRV = 0
    li    t0, 1
    bne   s5, t0, fail
    j     pass
fail_mprv:
    li    t0, 1 << 17
    csrc  mstatus, t0
    j     fail

    # 进 VS：mepc = t0，MPP = S，MPV = 1
to_vs:
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    li    t1, 1
    slli  t1, t1, 39
    csrs  mstatus, t1
    mret

    .align 2
m_trap:
    csrr  t5, mcause
    bltz  t5, fail                 # 本测试不该有 M 态中断
    li    t6, 10                   # VS ecall：回到 s6 (M 态)
    beq   t5, t6, m_ecall
    li    t6, 21                   # load guest page fault
    beq   t5, t6, m_gpf
    j     fail
m_ecall:
    csrw  mepc, s6
    li    t6, 3 << 11
    csrs  mstatus, t6              # MPP = M
    li    t6, 1
    slli  t6, t6, 39
    csrc  mstatus, t6              # MPV = 0
    mret
m_gpf:
    csrr  t6, mtval
    bne   t6, s8, fail             # mtval = GVA
    csrr  t6, 0x34b                # mtval2 = GPA >> 2
    srli  t5, s8, 2
    bne   t6, t5, fail
    csrr  t6, 0x34a                # mtinst = 0 (显式访存)
    bnez  t6, fail
    csrr  t6, mstatus
    srli  t6, t6, 38
    andi  t6, t6, 1                # GVA
    beqz  t6, fail
    addi  s5, s5, 1
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret

    .align 2
hs_trap:
    csrr  t5, scause
    bgez  t5, fail
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 10                   # VSEI (不委托到 VS 时 HS 看到 10)
    bne   t5, t6, fail
    li    t6, 0x400
    csrc  0x645, t6                # 清 hvip.VSEIP
    li    s4, 1
    sret                           # SPV = 1：回 VS

    .align 2
vs_trap:
    csrr  t5, scause               # V = 1：即 vscause
    bgez  t5, fail
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 1                    # VSSI -> 1
    beq   t5, t6, vs_ssi
    li    t6, 5                    # VSTI -> 5
    beq   t5, t6, vs_sti
    j     fail
vs_ssi:
    csrci 0x144, 2                 # sip.SSIP (V = 1：vsip，即 mip.VSSIP)
    li    s2, 1
    sret
vs_sti:
    li    t6, -1
    csrw  0x14d, t6                # vstimecmp = 最大
    li    s3, 1
    sret

pass:
    la    a0, tohost
    li    a1, 1
    sd    a1, 0(a0)
6:  j     6b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
7:  j     7b

    .data
    .align 14
g_root: .zero 16384
gdata:  .dword 0

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
