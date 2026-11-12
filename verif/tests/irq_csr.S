# irq_csr.S - CSR 密集的外部中断定向测试 (#28 类：ROB 头的 CSR / xRET 被 IQ 选中后、在 EX 启动前中断到达)
#   配 UVM 的 o6_irq_burst_test (连发 / 短间隔注入)；不注入中断时也能通过
#   工作循环几乎全是「重复执行就会出错」的头部 CSR：
#     csrrw 链式读改写 xscratch (读回值 = 上一次写入值，重复执行读回的是本次写入值)、
#     csrrs / csrrc 交替置清 bit 62 (重复执行读回值差 bit 62)、
#     xRET 跳床 (csrrs xstatus 置 xPP / xPIE -> csrw xepc -> xRET 跳到下一条)，中断在 xRET 时是开的
#   每轮再穿插一次 ecall -> M 态 -> mret 往返
#   第 1 段 M 态 (mstatus.MIE = 1，用 mscratch / mret 跳床)；第 2 段 S 态 (sstatus.SIE = 1，SEI 委托，用 sscratch / sret 跳床)，
#     MEI 不委托，从 S 态打进 M 态
#   跳床里被中断：处理程序的陷入 / 返回会改掉 xepc 和 xPP，所以中断点在 [csrw xepc, xRET] 时处理程序把返回地址改成跳床目标
#     (与跳床正常走完后的状态完全相同：xIE = 1，xPIE = 1，xPP = U，xepc = 目标)
#   M 态处理用 s8 / s9，S 态处理用 t5 / t6；计数器 s10 (MEI) / s11 (SEI)；a7 = ecall 的含义 (0 往返，1 通过)；主程序不用这些寄存器
#   不读 mip / sip (Spike 的 mip 只在陷入那一步被后门置位，读出来会和 DUT 不一致)
    .equ  IRQ_DEV, 0x0200C000
    .equ  NITER, 200
    .section .text.init
    .globl _start
_start:
    la    t0, m_trap
    csrw  mtvec, t0
    la    t0, s_trap
    csrw  stvec, t0
    li    s10, 0
    li    s11, 0
    li    a7, 0
    li    t0, -1                   # PMP 项 0：全地址 RWX (否则 S 态无权访问)
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    t0, 1 << 9               # SEI 委托给 S 态
    csrw  mideleg, t0
    li    t0, (1 << 11) | (1 << 9) # MEIE | SEIE
    csrs  mie, t0
    li    a1, 1
    slli  a1, a1, 62               # csrrs / csrrc 的位
    csrw  mscratch, zero           # 不变式：每组开头 xscratch == t1 - 1
    li    t1, 1
    li    a2, (3 << 11) | (1 << 7) # mstatus.MPP = M | MPIE
    la    a3, m_tgt
    csrsi mstatus, 8               # MIE
    li    s0, NITER
1:  call  m_work
    addi  s0, s0, -1
    bnez  s0, 1b
    csrci mstatus, 8
    # 进 S 态
    la    t0, s_entry
    csrw  mepc, t0
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, 1 << 11
    csrs  mstatus, t0
    mret

s_entry:
    csrw  sscratch, zero
    li    t1, 1
    li    a2, (1 << 8) | (1 << 5)  # sstatus.SPP = S | SPIE
    la    a3, s_tgt
    csrsi sstatus, 2               # SIE
    li    s0, NITER
2:  call  s_work
    addi  s0, s0, -1
    bnez  s0, 2b
    li    a7, 1
    ecall                          # -> M 态：a7 = 1 = 通过

# 一组 = csrrw 链 + csrrs + csrrc；组末检查读回值 (Spike 对拍逐条比 rd，这里再自查一遍)
    .macro CSR_GRP csr
    csrrw t0, \csr, t1             # t0 = t1 - 1，写 t1
    csrrs t2, \csr, a1             # t2 = t1，置 bit 62
    csrrc t3, \csr, a1             # t3 = t1 | bit 62，清 bit 62
    addi  t1, t1, 1
    .endm
    .macro CSR_CHK
    sub   t4, t3, t2
    bne   t4, a1, fail
    addi  t4, t0, 2
    bne   t4, t1, fail
    .endm

m_work:
    CSR_GRP mscratch
    CSR_GRP mscratch
    CSR_CHK
    csrrs a4, mstatus, a2          # mret 跳床：MPP = M，MPIE = 1 (上一次 mret 后 MPP = U，重复执行读回值不同)
m_tr0:
    csrrw a5, mepc, a3             # 读回上一次跳床的目标 (重复执行读回 a3 本身，值相同；靠 mret)
    mret
m_tgt:
    CSR_GRP mscratch
    CSR_GRP mscratch
    CSR_CHK
    ecall                          # -> M 态往返 (原因 11)
    CSR_GRP mscratch
    CSR_CHK
    ret

s_work:
    CSR_GRP sscratch
    CSR_GRP sscratch
    CSR_CHK
    csrrs a4, sstatus, a2          # sret 跳床：SPP = S，SPIE = 1
s_tr0:
    csrrw a5, sepc, a3
    sret
s_tgt:
    CSR_GRP sscratch
    CSR_GRP sscratch
    CSR_CHK
    ecall                          # -> M 态往返 (原因 9)
    CSR_GRP sscratch
    CSR_CHK
    ret

    .align 2
m_trap:
    csrr  s8, mcause
    bltz  s8, m_irq
    li    s9, 9                    # ecall (S 态 9 / M 态 11)
    beq   s8, s9, m_ecall
    li    s9, 11
    bne   s8, s9, fail
m_ecall:
    bnez  a7, pass
    csrr  s8, mepc
    addi  s8, s8, 4
    csrw  mepc, s8
    mret
m_irq:
    slli  s8, s8, 1
    srli  s8, s8, 1
    li    s9, 11                   # 只应收到 MEI
    bne   s8, s9, fail
    addi  s10, s10, 1
    csrr  s8, mepc                 # 中断点在 mret 跳床 [m_tr0, m_tgt) 内：直接返回跳床目标
    la    s9, m_tr0
    bltu  s8, s9, 3f
    la    s9, m_tgt
    bgeu  s8, s9, 3f
    csrw  mepc, s9
3:  li    s9, IRQ_DEV
    li    s8, 1
    sw    s8, 0(s9)                # 清 MEIP
    lw    s8, 0(s9)                # 读回：MMIO load 要等前面的 store 排空，保证 mret 前已清掉
    mret

    .align 2
s_trap:
    csrr  t5, scause
    bgez  t5, fail
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 9                    # 只应收到 SEI
    bne   t5, t6, fail
    addi  s11, s11, 1
    csrr  t5, sepc                 # 中断点在 sret 跳床 [s_tr0, s_tgt) 内：直接返回跳床目标
    la    t6, s_tr0
    bltu  t5, t6, 4f
    la    t6, s_tgt
    bgeu  t5, t6, 4f
    csrw  sepc, t6
4:  li    t6, IRQ_DEV
    li    t5, 2
    sw    t5, 0(t6)                # 清 SEIP
    lw    t5, 0(t6)
    sret

pass:
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
5:  j     5b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
6:  j     6b

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
