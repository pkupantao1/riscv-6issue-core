# irq_tmr.S - 定时器 / 软件中断 (MTI / MSI / STI / SSI) + 外部中断 (MEI / SEI) + WFI：配 UVM o6_tmr_test；不注入时也能通过
#   UVM CLINT 代理：随机置 msip (MSI)、把 mtimecmp 设为 mtime + δ (MTI)；外部中断代理同 irq_ext (设备 0x0200_C000)
#   M 态：MTI 处理把 mtimecmp 设回最大值，再置 mip.STIP 转给 S 态 (同 OpenSBI 的做法)；MSI 处理清 msip；MEI 清设备位
#   S 态 (mideleg 委托 SEI / STI / SSI)：STI 处理 ecall 让 M 态清 STIP；SSI 由 S 态工作循环自己 csrsi sip 置位、处理里清；SEI 清设备位
#   M 态段每 64 次迭代自己设一个 20 个 mtime 周期后的定时器再 wfi (本核 WFI 按空操作实现，规范允许)
#   M 态处理用 s8 / s9，S 态处理用 t5 / t6；不读 mip / sip (Spike 的 mip 由 DUT 注入，读出来会不一致)
#   自检：进 MEI / SEI / MSI 处理时，先读设备确认对应挂起位为 1 —— 本测试从不用软件置 SEIP，「设备没挂却进了中断」
#   就是缺陷 (#30：csrs mip 把外部 SEIP 锁进软件位后中断风暴；对拍看不出，因为中断时刻本来就由 DUT 注入给 Spike)
    .equ  MSIP,     0x02000000
    .equ  MTIMECMP, 0x02004000
    .equ  MTIME,    0x0200BFF8
    .equ  IRQ_DEV,  0x0200C000
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
    li    t0, (1 << 9) | (1 << 5) | (1 << 1)    # SEI / STI / SSI 委托给 S 态
    csrw  mideleg, t0
    li    t0, (1 << 11) | (1 << 7) | (1 << 3) | (1 << 9) | (1 << 5) | (1 << 1)
    csrs  mie, t0
    li    t0, MTIMECMP             # 定时器先关
    li    t1, -1
    sd    t1, 0(t0)
    li    t0, 1 << 13              # mstatus.FS = Initial
    csrs  mstatus, t0
    csrsi mstatus, 8               # MIE
    li    s0, 600
1:  call  work
    andi  t0, s0, 63
    bnez  t0, 2f
    li    t0, MTIME                # 自己设 20 个 mtime 周期后的定时器，再 wfi
    ld    t1, 0(t0)
    addi  t1, t1, 20
    li    t0, MTIMECMP
    sd    t1, 0(t0)
    wfi
2:  addi  s0, s0, -1
    bnez  s0, 1b
    csrci mstatus, 8
    # 进 S 态
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
3:  call  work
    andi  t0, s0, 15
    bnez  t0, 4f
    csrsi sip, 2                   # 自己发一个 S 态软件中断
4:  addi  s0, s0, -1
    bnez  s0, 3b
    j     pass                     # S 态直接写 tohost (tb 监视 D$ store 口，与特权级无关)

# 工作负载：与 irq_ext 相同的 15 类指令
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
    beqz  t4, 5f
    sub   t3, t3, t2
5:  ret

    .align 2
m_trap:
    csrr  s8, mcause
    bltz  s8, m_irq
    li    s9, 9                    # 唯一的异常：S 态处理 STI 时 ecall 请 M 态清 STIP
    bne   s8, s9, fail
    li    s9, 1 << 5
    csrc  mip, s9
    csrr  s9, mepc
    addi  s9, s9, 4
    csrw  mepc, s9
    mret
m_irq:
    slli  s8, s8, 1
    srli  s8, s8, 1
    li    s9, 11
    beq   s8, s9, m_mei
    li    s9, 7
    beq   s8, s9, m_mti
    li    s9, 3
    beq   s8, s9, m_msi
    j     fail
m_mei:
    li    s9, IRQ_DEV
    lw    s8, 0(s9)
    andi  s8, s8, 1
    beqz  s8, fail                 # 设备没挂 MEIP
    li    s8, 1
    sw    s8, 0(s9)                # 清 MEIP
    lw    s8, 0(s9)                # 读回：保证 mret 前已清
    mret
m_mti:
    li    s9, MTIMECMP
    li    s8, -1
    sd    s8, 0(s9)                # 定时器关 (MTIP 落)
    ld    s8, 0(s9)
    li    s9, 1 << 5
    csrs  mip, s9                  # 转给 S 态：STIP
    mret
m_msi:
    li    s9, MSIP
    lw    s8, 0(s9)
    beqz  s8, fail                 # msip 没置位
    sw    zero, 0(s9)              # 清 MSIP
    lw    s8, 0(s9)
    mret

    .align 2
s_trap:
    csrr  t5, scause
    bgez  t5, fail                 # S 态不应有异常
    slli  t5, t5, 1
    srli  t5, t5, 1
    li    t6, 9
    beq   t5, t6, s_sei
    li    t6, 5
    beq   t5, t6, s_sti
    li    t6, 1
    beq   t5, t6, s_ssi
    j     fail
s_sei:
    li    t6, IRQ_DEV
    lw    t5, 0(t6)
    andi  t5, t5, 2
    beqz  t5, fail                 # 设备没挂 SEIP (#30)
    li    t5, 2
    sw    t5, 0(t6)                # 清 SEIP
    lw    t5, 0(t6)
    sret
s_sti:
    ecall                          # M 态清 STIP
    sret
s_ssi:
    csrci sip, 2
    sret

pass:                              # 在 S 态主流程里执行，不能用处理程序的 t5 / t6
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
