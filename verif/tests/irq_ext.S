# irq_ext.S - 外部中断 (MEIP / SEIP) 随机注入测试：配 UVM 的 o6_irq_test (tb/uvm)；不注入中断时也能通过
#   TB 中断设备 (0x0200_C000)：UVM 驱动置位 MEIP / SEIP 并保持；程序写 bit0 清 MEIP、bit1 清 SEIP，读回当前挂起位
#   第 1 段 M 态工作循环 (mstatus.MIE = 1)：MEI 进 M 态处理
#   第 2 段 S 态工作循环 (sstatus.SIE = 1，mideleg 委托 SEI)：SEI 进 S 态处理；MEI 不委托，从 S 态打进 M 态
#   M 态处理用 s8 / s9，S 态处理用 t5 / t6 (S 态处理中可被 MEI 打断，两层不能共用)；计数器 s10 (MEI) / s11 (SEI)；主程序不用这些寄存器
#   不读 mip / sip (Spike 的 mip 只在陷入那一步被后门置位，读出来会和 DUT 不一致)
    .equ  IRQ_DEV, 0x0200C000
    .section .text.init
    .globl _start
_start:
    la    t0, m_trap
    csrw  mtvec, t0
    li    s10, 0
    li    s11, 0
    li    t0, -1                   # PMP 项 0：全地址 RWX (否则 S 态无权访问)
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    t0, 1 << 9               # SEI 委托给 S 态
    csrw  mideleg, t0
    li    t0, (1 << 11) | (1 << 9) # MEIE | SEIE
    csrs  mie, t0
    li    t0, 1 << 13              # mstatus.FS = Initial (工作负载里有浮点)
    csrs  mstatus, t0
    csrsi mstatus, 8               # MIE
    li    s0, 800
1:  call  work
    addi  s0, s0, -1
    bnez  s0, 1b
    csrci mstatus, 8
    # 进 S 态
    la    t0, s_trap
    csrw  stvec, t0
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    csrsi sstatus, 2               # SIE
    li    s0, 800
2:  call  work
    addi  s0, s0, -1
    bnez  s0, 2b
    ecall                          # -> M 态：原因 9 = 通过

# 被中断的工作负载：覆盖 cg_trap.cx_irq_last 要求的各类指令 (ALU / B / MUL / DIV / LD / ST / AMO / BR / JMP / CSR /
#   FPLD / FPST / FPU / FDIV / FCVT)，中断可以打在任何一条之后
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
    csrr  t2, sscratch             # 在 ROB 头执行 (wait_head)：提交组在除法处断开，中断才能恰好落在除法之后
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
    li    s9, 11                   # 只应收到 MEI
    bne   s8, s9, fail
    addi  s10, s10, 1
    li    s9, IRQ_DEV
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
    li    t6, IRQ_DEV
    li    t5, 2
    sw    t5, 0(t6)                # 清 SEIP
    lw    t5, 0(t6)
    sret

pass:
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
4:  j     4b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
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
