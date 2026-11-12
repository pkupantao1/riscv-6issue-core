# ptw_pmp.S - 页表遍历的 PMP 检查
#   PMP0：页表所在 4 KB，NAPOT，无 RWX (L=0，不影响 M 态)
#   PMP1：全部地址 NAPOT RWX
#   satp = Sv39，根表把 VA 0x8000_0000 起 1 GB 恒等映射 (RWXAD)
#   mret 进 S 态执行 0x8000_xxxx 处代码 -> 取指要遍历页表 -> 读根表 PTE 被 PMP0 拒绝
#   期望：取指访问错误 (mcause = 1)，mtval = S 态入口地址
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    # 根页表：第 2 项 (VA[38:30] = 2) -> PA 0x8000_0000 的 1 GB 叶子
    la    t0, pt_root
    li    t1, (0x80000000 >> 12) << 10
    ori   t1, t1, 0xCF              # V R W X A D
    sd    t1, 16(t0)
    # PMP0：pt_root 所在 4 KB，NAPOT：pmpaddr = (base >> 2) | (size/8 - 1)
    srli  t1, t0, 2
    ori   t1, t1, 0x1FF
    csrw  pmpaddr0, t1
    # PMP1：全部
    li    t1, -1
    csrw  pmpaddr1, t1
    # pmpcfg0：项 0 = NAPOT(0x18) 无权限；项 1 = NAPOT | RWX (0x1F)
    li    t1, (0x1F << 8) | 0x18
    csrw  pmpcfg0, t1
    # satp
    la    t0, pt_root
    srli  t0, t0, 12
    li    t1, 8 << 60
    or    t0, t0, t1
    csrw  satp, t0
    sfence.vma
    # 进 S 态
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    li    a0, 0x123                 # 不应执行到
    j     fail

    .align 2
trap:
    csrr  t0, mcause
    li    t1, 1
    bne   t0, t1, fail
    csrr  t0, mtval
    la    t1, s_entry
    bne   t0, t1, fail
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
1:  j     1b
fail:
    li    t0, 3
    la    t1, tohost
    sd    t0, 0(t1)
2:  j     2b

    .data
    .align 12
pt_root:
    .zero 4096

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
