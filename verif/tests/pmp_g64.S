# pmp_g64.S - PMP 粒度 64 B (o6_pkg::PMP_G = 4) 下 NAPOT 的低位按 1 处理 (#34 后的规格；变异 M19)
#   PMP0：NAPOT，pmpaddr0 = X >> 2 (低位全 0：按原始编码是 8 B 区域)，只读；G = 64 B 时低 3 位视为 1 -> 区域 [X, X + 64)
#   PMP1：全地址 RWX
#   S 态 (经 MPRV 不行：M 态 PMP 对非锁定项不生效，所以真进 S 态，satp = Bare)：
#     sw X + 40 -> 必须 store 访问错误 (mcause 7、mtval = X + 40)；lw X + 40 -> 正常 (可读)；sw X + 64 -> 正常 (区域外)
#   另查 pmpaddr0 读回：NAPOT 模式下低 G - 1 = 3 位读作 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    la    s0, region               # X (64 B 对齐)
    srli  t0, s0, 2
    csrw  pmpaddr0, t0
    li    t1, -1
    csrw  pmpaddr1, t1
    li    t1, (0x1F << 8) | 0x19   # 项 0 = NAPOT | R；项 1 = NAPOT | RWX
    csrw  pmpcfg0, t1
    csrr  t1, pmpaddr0             # 读回低 3 位应为 1
    ori   t2, t0, 7
    bne   t1, t2, fail
    csrw  satp, zero
    li    s1, 0                    # 阶段：0 = 等 store 错误，1 = 之后的访问都不该陷入
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    li    t3, 0x77
    sw    t3, 40(s0)               # 应陷入 (区域内、不可写)
    li    s1, 1
    lw    t4, 40(s0)               # 可读
    sw    t3, 64(s0)               # 区域外
    ecall                          # -> M 态：原因 9 = 通过

    .align 2
trap:
    csrr  t0, mcause
    li    t1, 9
    beq   t0, t1, done
    bnez  s1, fail                 # 第二阶段不该有别的陷入
    li    t1, 7
    bne   t0, t1, fail
    csrr  t0, mtval
    addi  t1, s0, 40
    bne   t0, t1, fail
    csrr  t0, mepc
    addi  t0, t0, 4
    csrw  mepc, t0
    li    s1, 1
    mret
done:
    beqz  s1, fail                 # store 错误没出现
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
1:  j     1b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
2:  j     2b

    .data
    .align 12
region:
    .zero 256

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
