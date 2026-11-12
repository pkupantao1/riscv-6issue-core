# irq_plic.S - SoC 模式外部中断 (经 PLIC)：配 UVM o6_irq_test (SOC=1 UVM=1)；不注入中断时也能通过
#   TB：中断代理的 MEI / SEI 脉冲拉高 PLIC 源 2 / 源 3 的电平，源被认领时 TB 撤电平
#   PLIC：源 2 -> 上下文 0 (M)，源 3 -> 上下文 1 (S)，优先级 1，阈值 0；UART (源 1) 不使能
#   M 态处理：认领 (0x0C20_0004) 必须是 2，完成后 mret；S 态处理：认领 (0x0C20_1004) 必须是 3，完成后 sret
#   第 1 段 M 态工作循环 (MIE)；第 2 段 S 态 (SIE，mideleg 委托 SEI；MEI 从 S 态打进 M 态)
#   M 态处理用 s8 / s9，S 态处理用 t5 / t6
    .equ  PLIC,      0x0C000000
    .equ  PLIC_EN0,  0x0C002000
    .equ  PLIC_EN1,  0x0C002080
    .equ  PLIC_THR0, 0x0C200000
    .equ  PLIC_CLM0, 0x0C200004
    .equ  PLIC_THR1, 0x0C201000
    .equ  PLIC_CLM1, 0x0C201004
    .section .text.init
    .globl _start
_start:
    la    t0, m_trap
    csrw  mtvec, t0
    la    t0, s_trap
    csrw  stvec, t0
    li    t0, -1                   # PMP 项 0：全地址 RWX
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    # PLIC
    li    t0, PLIC
    li    t1, 1
    sw    t1, 8(t0)                # 源 2 优先级 1
    sw    t1, 12(t0)               # 源 3 优先级 1
    li    t0, PLIC_EN0
    li    t1, 1 << 2
    sw    t1, 0(t0)
    li    t0, PLIC_EN1
    li    t1, 1 << 3
    sw    t1, 0(t0)
    li    t0, PLIC_THR0
    sw    zero, 0(t0)
    li    t0, PLIC_THR1
    sw    zero, 0(t0)
    li    t0, 1 << 9               # SEI 委托给 S 态
    csrw  mideleg, t0
    li    t0, (1 << 11) | (1 << 9)
    csrs  mie, t0
    li    t0, 1 << 13              # FS = Initial
    csrs  mstatus, t0
    csrsi mstatus, 8               # MIE
    li    s0, 600
1:  call  work
    addi  s0, s0, -1
    bnez  s0, 1b
    csrci mstatus, 8
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    csrsi sstatus, 2               # SIE
    li    s0, 600
2:  call  work
    addi  s0, s0, -1
    bnez  s0, 2b
    ecall                          # -> M 态：原因 9 = 通过

work:
    la    t0, buf
    andi  t1, s0, 0x38
    add   t0, t0, t1
    ld    t1, 0(t0)
    addi  t1, t1, 1
    sd    t1, 0(t0)
    mul   t2, t1, t1
    add   t3, t2, s0
    xor   t4, t3, t1
    divu  t3, t3, t1
    csrr  t2, sscratch
    andn  t4, t4, t2
    amoadd.d t2, t1, (t0)
    fld   ft0, 0(t0)
    fcvt.d.l ft1, t1
    fadd.d ft2, ft1, ft0
    fdiv.d ft3, ft2, ft1
    fsd   ft3, 64(t0)
    andi  t4, t4, 1
    beqz  t4, 3f
    sub   t3, t3, t2
3:  ret

    .align 2
m_trap:
    csrr  s8, mcause
    bltz  s8, m_irq
    li    s9, 9
    beq   s8, s9, pass
    j     fail
m_irq:
    slli  s8, s8, 1
    srli  s8, s8, 1
    li    s9, 11
    bne   s8, s9, fail
    li    s9, PLIC_CLM0
    lw    s8, 0(s9)                # 认领
    addi  s8, s8, -2
    bnez  s8, fail                 # 只应是源 2
    li    s8, 2
    sw    s8, 0(s9)                # 完成 (不能再读认领寄存器确认：读认领有副作用，会认领下一个挂起的中断)
    mret

    .align 2
s_trap:
    csrr  t5, scause
    bgez  t5, fail
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 9
    bne   t5, t6, fail
    li    t6, PLIC_CLM1
    lw    t5, 0(t6)
    addi  t5, t5, -3
    bnez  t5, fail                 # 只应是源 3
    li    t5, 3
    sw    t5, 0(t6)
    sret

pass:
    la    a0, tohost
    li    a1, 1
    sd    a1, 0(a0)
4:  j     4b
fail:
    la    a0, tohost
    li    a1, 3
    sd    a1, 0(a0)
5:  j     5b

    .data
    .align 6
buf:
    .zero 128

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
