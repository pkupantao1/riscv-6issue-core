# clint_soc.S - SoC 模式：真实 o6_clint 的定时器中断 (MTI) 与软件中断 (MSI)，以及 mtime 读写
#   CLINT 0x0200_0000：msip @ +0、mtimecmp @ +0x4000、mtime @ +0xBFF8 (每 TICK_DIV 拍加 1)
#   5 轮：mtimecmp = mtime + 30 -> 工作循环等 MTI (处理程序把 mtimecmp 写成全 1 撤中断、计数)；
#         写 msip = 1 -> MSI (处理程序写 msip = 0、计数)
#   另查：mtime 单调递增；写 mtime 后读回从写入值继续走
#   自检：MTI / MSI 各 5 次 + 对拍 (mip.MTIP / MSIP 由 DUT 注入 Spike)
    .equ  CLINT,   0x02000000
    .equ  MTCMP,   0x02004000
    .equ  MTIME,   0x0200BFF8
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    s10, 0                     # MTI 次数
    li    s11, 0                     # MSI 次数
    li    a0, MTCMP
    li    a1, MTIME
    li    a2, CLINT
    li    t0, -1
    sd    t0, 0(a0)                  # 先关定时器
    # mtime 可写：写一个值，之后读回 >= 它
    li    t0, 0x100000
    sd    t0, 0(a1)
    ld    t1, 0(a1)
    bltu  t1, t0, fail
    li    t0, (1 << 7) | (1 << 3)    # MTIE | MSIE
    csrs  mie, t0
    csrsi mstatus, 8
    li    s0, 5
round:
    # 定时器
    ld    t0, 0(a1)
    addi  t0, t0, 30
    sd    t0, 0(a0)
    mv    t2, s10
1:  addi  t3, t3, 1                  # 等 MTI
    beq   s10, t2, 1b
    # 软件中断
    mv    t2, s11
    li    t0, 1
    sw    t0, 0(a2)
2:  addi  t3, t3, 1                  # 等 MSI
    beq   s11, t2, 2b
    # mtime 单调
    ld    t0, 0(a1)
    ld    t1, 0(a1)
    bltu  t1, t0, fail
    addi  s0, s0, -1
    bnez  s0, round
    csrci mstatus, 8
    li    t0, 5
    bne   s10, t0, fail
    bne   s11, t0, fail
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
3:  j     3b

    .align 2
trap:
    csrr  t4, mcause
    bgez  t4, fail
    slli  t4, t4, 1
    srli  t4, t4, 1
    li    t5, 7
    beq   t4, t5, mti
    li    t5, 3
    beq   t4, t5, msi
    j     fail
mti:
    li    t5, -1
    sd    t5, 0(a0)                  # 撤定时器中断
    addi  s10, s10, 1
    mret
msi:
    sw    zero, 0(a2)                # 撤软件中断
    addi  s11, s11, 1
    mret
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
4:  j     4b

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
