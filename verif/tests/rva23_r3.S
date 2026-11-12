# rva23_r3.S - RVA23 R3 (docs/14)：Sstc / Sscofpmf / Smstateen 的定向自检
#   1. Sstc：先把 stimecmp 设成最大再开 menvcfg.STCE (复位值 0 时一开 STCE 就挂起 STIP)；STCE = 1 时 csrs mip.STIP 写不进；
#      stimecmp = time + 50，开 mie.STIE + mstatus.MIE (不委托，M 态收)，等 S 定时器中断 (cause 5)，处理里 stimecmp = -1 清掉
#   2. Sscofpmf：mhpmevent3 = 事件 2 (提交了分支)，mhpmcounter3 = -20，开 mie.LCOFIE，等计数器溢出中断 (cause 13)；
#      处理里核对 scountovf 第 3 位、mhpmevent3 = 0 清 OF 并停表、清 mip.LCOFIP。再验 MINH：M 态里计数不动
#   3. Smstateen / Sstc 的访问权限：mstateen0 = 0、mcounteren.TM = 0，进 S 态读 sstateen0 / senvcfg / stimecmp，应各报一次非法指令
#   中断时刻由 DUT 决定 (对拍时注入 Spike)；自检只用 DUT 自己读到的值。M 态处理只用 t5 / t6
    .section .text.init
    .globl _start
_start:
    la    t0, m_trap
    csrw  mtvec, t0
    li    t0, -1                   # PMP 项 0：全地址 RWX (S 态要用)
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    s2, 0                    # STI 已到
    li    s4, 0                    # 非法指令陷入次数
    li    s5, 0                    # LCOFI 已到

    # ---------------- 1. Sstc
    li    t0, -1
    csrw  0x14d, t0             # M 态访问不受 STCE 限制
    li    t0, 1
    slli  t0, t0, 63
    csrs  0x30a, t0              # STCE
    li    t0, 1 << 5
    csrs  mip, t0                  # STCE = 1：STIP 写不进
    csrr  t1, mip
    and   t1, t1, t0
    bnez  t1, fail
    csrs  mie, t0                  # STIE (不委托：M 态收)
    rdtime t1
    addi  t1, t1, 50
    csrw  0x14d, t1
    csrsi mstatus, 8               # MIE
    li    t2, 200000               # 超时
1:  bnez  s2, 2f
    addi  t2, t2, -1
    bnez  t2, 1b
    j     fail
2:  csrci mstatus, 8
    li    t0, 1 << 5
    csrc  mie, t0

    # ---------------- 2. Sscofpmf
    li    t0, -20
    csrw  0xb03, t0
    li    t0, 2                    # 事件 2：提交了分支
    csrw  0x323, t0
    li    t0, 1 << 13
    csrs  mie, t0                  # LCOFIE
    csrsi mstatus, 8
    li    t2, 200000
3:  bnez  s5, 4f                   # 本循环本身就在提交分支
    addi  t2, t2, -1
    bnez  t2, 3b
    j     fail
4:  csrci mstatus, 8
    li    t0, 1 << 13
    csrc  mie, t0
    # MINH：M 态不计
    li    t0, 1
    slli  t0, t0, 62
    ori   t0, t0, 2
    csrw  0x323, t0
    csrr  t3, 0xb03
    li    t2, 50
5:  addi  t2, t2, -1
    bnez  t2, 5b
    csrr  t4, 0xb03
    bne   t3, t4, fail
    csrw  0x323, zero

    # ---------------- 3. 访问权限 (S 态)
    csrw  0x30c, zero          # SE0 = 0 / ENVCFG = 0
    csrw  mcounteren, zero         # TM = 0
    la    t0, s_code
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11              # MPP = S
    csrs  mstatus, t1
    mret
s_code:
    csrr  t0, 0x10c            # SE0 = 0：非法
    csrr  t0, 0x10a              # ENVCFG = 0：非法
    csrr  t0, 0x14d             # TM = 0：非法
    ecall                          # 回 M 态 (m_cont)
m_cont:
    li    t0, 3
    bne   s4, t0, fail
    # SE0 / ENVCFG 打开后 S 态可读 (再进一次 S 态)
    li    t0, 3
    slli  t0, t0, 62
    csrw  0x30c, t0
    li    s4, 0
    la    t0, s_code2
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret
s_code2:
    csrr  t0, 0x10c
    csrr  t0, 0x10a
    ecall
m_cont2:
    bnez  s4, fail
    j     pass

m_trap:
    csrr  t5, mcause
    bltz  t5, irq
    li    t6, 2
    beq   t5, t6, ill
    li    t6, 9                    # S 态 ecall
    beq   t5, t6, ecall_s
    j     fail
ill:
    addi  s4, s4, 1
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret
ecall_s:                           # 回 M 态：按 ecall 所在位置决定去 m_cont / m_cont2
    csrr  t5, mepc
    la    t6, m_cont
    addi  t6, t6, -4
    beq   t5, t6, 1f
    la    t6, m_cont2
    j     2f
1:  la    t6, m_cont
2:  csrw  mepc, t6
    li    t6, 3 << 11
    csrs  mstatus, t6              # MPP = M
    mret
irq:
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 5
    beq   t5, t6, sti
    li    t6, 13
    beq   t5, t6, cof
    j     fail
sti:
    li    t6, -1
    csrw  0x14d, t6             # 清 STIP
    li    s2, 1
    mret
cof:
    csrr  t6, 0xda0
    andi  t6, t6, 8
    beqz  t6, fail
    csrw  0x323, zero         # 清 OF、停表
    li    t6, 1 << 13
    csrc  mip, t6                  # 清 LCOFIP
    li    s5, 1
    mret

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

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
