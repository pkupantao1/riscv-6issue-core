# sfence_sq.S - SFENCE.VMA 必须等 SQ 里已提交的 store (含改 PTE 的那条) 排空 (bug #14，变异 M11)
#   页表：根[2] = VA 0x8000_0000 起 1 GB 恒等 (代码 / 数据)；根[3] -> L1 -> L0，L0[0] 把 VA 0xC000_0000 映射到 A 页或 B 页
#   S 态每轮：60 个写冷行的 store 把 SQ 堵住 (缺失逐条串行) -> 改 L0[0] (A <-> B) -> sfence.vma -> 立即经 VA 读
#   读到的必须是新页的值 (A 页 0x1111、B 页 0x2222)。#14：SFENCE 不等排空，页表遍历读到还在 SQ 里的旧 PTE -> 读到旧页
    .equ  VA_WIN, 0xC0000000
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, -1                   # PMP 项 0：全地址 RWX
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    # 根[2]：1 GB 恒等叶子
    la    t0, pt_root
    li    t1, (0x80000000 >> 12) << 10
    ori   t1, t1, 0xCF
    sd    t1, 16(t0)
    # 根[3] -> L1，L1[0] -> L0 (非叶：只有 V)
    la    t1, pt_l1
    srli  t1, t1, 12
    slli  t1, t1, 10
    ori   t1, t1, 1
    sd    t1, 24(t0)
    la    t0, pt_l1
    la    t1, pt_l0
    srli  t1, t1, 12
    slli  t1, t1, 10
    ori   t1, t1, 1
    sd    t1, 0(t0)
    # L0[0] -> A 页
    la    t1, page_a
    srli  t1, t1, 12
    slli  t1, t1, 10
    ori   t1, t1, 0xCF
    la    t0, pt_l0
    sd    t1, 0(t0)
    # satp
    la    t0, pt_root
    srli  t0, t0, 12
    li    t1, 8 << 60
    or    t0, t0, t1
    csrw  satp, t0
    sfence.vma
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    li    s9, VA_WIN
    la    s10, pt_l0
    la    t0, page_a
    srli  t0, t0, 12
    slli  t0, t0, 10
    ori   s4, t0, 0xCF             # A 页的 PTE
    la    t0, page_b
    srli  t0, t0, 12
    slli  t0, t0, 10
    ori   s5, t0, 0xCF             # B 页的 PTE
    li    a4, 0x1111
    li    a5, 0x2222
    ld    t1, 0(s9)                # 先把 A 的翻译装进 TLB
    bne   t1, a4, fail
    la    s7, sbuf
    li    s0, 0
    li    s6, 8
1:  mv    s3, s7
    .rept 60
    sd    s0, 0(s3)                # 冷行 store：SQ 里排成长队
    addi  s3, s3, 64
    .endr
    li    t0, 60 * 64
    add   s7, s7, t0
    andi  t0, s0, 1                # 第 0 轮 -> B，第 1 轮 -> A，...
    mv    t2, s5
    mv    t3, a5
    beqz  t0, 2f
    mv    t2, s4
    mv    t3, a4
2:  sd    t2, 0(s10)               # 改 PTE (排在 60 个慢 store 后面)
    sfence.vma
    ld    t1, 0(s9)                # 必须经新映射
    bne   t1, t3, fail
    addi  s0, s0, 1
    blt   s0, s6, 1b
    ecall                          # -> M 态：原因 9 = 通过

    .align 2
trap:
    csrr  t0, mcause
    li    t1, 9
    bne   t0, t1, fail
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
3:  j     3b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
4:  j     4b

    .data
    .align 12
pt_root: .zero 4096
pt_l1:   .zero 4096
pt_l0:   .zero 4096
page_a:  .dword 0x1111
    .align 12
page_b:  .dword 0x2222
    .align 12
sbuf:    .zero 8 * 60 * 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
