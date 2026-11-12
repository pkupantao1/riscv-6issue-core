# priv_exc.S - 特权级 x 异常 定向 (cg_trap.cx_exc_priv 的合法缺项)：所有陷入进 M 态 (不委托)，
#   M 态处理核对 mcause 与来源特权级 (mstatus.MPP)，清 MPRV / satp 后在 M 态跳到下一个用例 (不 mret)
#   S 态缺页 / M 态 MPRV 用例用 Sv39：VA 0x8000_0000 起 1 GB 吉页恒等映射 (U=0)，VA 0x4000_0000 无映射
#   访问错误用不存在的物理地址 0x4000_0000 (PMA 无此区域)；未对齐用 buf+1 (本核与 Spike 都对未对齐报异常)
#   寄存器：s1 期望 mcause，s2 处理后继续的地址，s3 期望来源特权级，s4 已完成用例数
    .equ  BAD, 0x40000000
    .section .text.init
    .globl _start

# 进入 prv 态执行 target，期望异常 cause；处理后从 resume 继续
.macro ENTER cause, prv, target, resume
    li    s1, \cause
    li    s3, \prv
    la    s2, \resume
    la    t0, \target
    csrw  mepc, t0
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, \prv << 11
    csrs  mstatus, t0
    mret
.endm
# 在 M 态直接执行 target
.macro MCASE cause, target, resume
    li    s1, \cause
    li    s3, 3
    la    s2, \resume
    j     \target
.endm
.macro VM_ON
    la    t0, pt_root
    srli  t0, t0, 12
    li    t1, 8 << 60
    or    t0, t0, t1
    csrw  satp, t0
    sfence.vma
.endm

_start:
    la    t0, m_trap
    csrw  mtvec, t0
    li    t0, -1                   # PMP 项 0：全地址 RWX (S / U 态可访问；访问错误来自 PMA)
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    csrw  medeleg, zero
    csrw  mideleg, zero
    li    s4, 0
    # 根页表第 2 项：VA 0x8000_0000 吉页 -> PA 0x8000_0000，D A X W R V (U=0)
    la    t0, pt_root
    li    t1, (0x80000 << 10) | 0xCF
    sd    t1, 16(t0)

    # ---- S 态 (分页)：load / store / 取指缺页
    VM_ON
    ENTER 13, 1, s_ld, c2
c2: VM_ON
    ENTER 15, 1, s_st, c3
c3: VM_ON
    ENTER 12, 1, s_if, c4
    # ---- M 态 MPRV=1 (按 MPP=S 翻译)：load / store 缺页
c4: VM_ON
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, (1 << 11) | (1 << 17)    # MPP = S，MPRV = 1
    csrs  mstatus, t0
    MCASE 13, m_ld, c5
c5: VM_ON
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, (1 << 11) | (1 << 17)
    csrs  mstatus, t0
    MCASE 15, m_st, c6
    # ---- M 态 (bare)：store / 取指访问错误
c6: MCASE 7, m_stacc, c7
c7: MCASE 1, m_ifacc, c8
    # ---- U 态 (bare)：load / store / 取指访问错误、断点、未对齐
c8: ENTER 5, 0, u_ldacc, c9
c9: ENTER 7, 0, u_stacc, c10
c10: ENTER 1, 0, u_ifacc, c11
c11: ENTER 3, 0, u_bkpt, c12
c12: ENTER 4, 0, u_ldmis, c13
c13: ENTER 6, 0, u_stmis, c14
    # ---- S 态 (bare)：未对齐
c14: ENTER 4, 1, u_ldmis, c15
c15: ENTER 6, 1, u_stmis, c16
c16: li    t0, 15
    bne   s4, t0, fail             # 15 个用例都进过处理程序
    j     pass

# 各用例的出错指令 (后面的 j fail：没出异常就失败)
s_ld:   li t1, BAD
        ld t2, 0(t1)
        j  fail
s_st:   li t1, BAD
        sd t2, 0(t1)
        j  fail
s_if:   li t1, BAD
        jr t1
m_ld:   li t1, BAD
        ld t2, 0(t1)
        j  fail
m_st:   li t1, BAD
        sd t2, 0(t1)
        j  fail
m_stacc: li t1, BAD
        sd t2, 0(t1)
        j  fail
m_ifacc: li t1, BAD
        jr t1
u_ldacc: li t1, BAD
        ld t2, 0(t1)
        j  fail
u_stacc: li t1, BAD
        sd t2, 0(t1)
        j  fail
u_ifacc: li t1, BAD
        jr t1
u_bkpt: ebreak
        j  fail
u_ldmis: la t1, buf
        ld t2, 1(t1)
        j  fail
u_stmis: la t1, buf
        sd t2, 3(t1)
        j  fail

    .align 2
m_trap:
    li    t0, 1 << 17              # 先清 MPRV (之后的访存不再按 MPP 翻译)
    csrc  mstatus, t0
    csrw  satp, zero
    sfence.vma
    csrr  t0, mcause
    bne   t0, s1, fail
    csrr  t0, mstatus
    srli  t0, t0, 11
    andi  t0, t0, 3
    bne   t0, s3, fail             # 来源特权级
    addi  s4, s4, 1
    jr    s2

pass:
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
1:  j     1b
fail:
    csrw  satp, zero
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
2:  j     2b

    .data
    .align 12
pt_root:
    .zero 4096
    .align 6
buf:
    .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
