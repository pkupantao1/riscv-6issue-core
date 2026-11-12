# dpmp.S - 数据访问的 PMP 检查 (S 态，裸地址)
#   PMP0：buf_ro 所在 4 KB，NAPOT，只读 (R)
#   PMP1：buf_no 所在 4 KB，NAPOT，无权限
#   PMP2：全部地址 NAPOT RWX
#   S 态依次：读 buf_ro (应成功) / 写 buf_ro (store 访问错误 7) / 读 buf_no (load 访问错误 5) /
#            amoadd buf_ro (store 访问错误 7) / 紧跟在被拒的 store 后面读同一地址 (store 先陷入，load 被冲掉)
#   陷入处理：核对原因与 mtval，mepc += 4 返回；最后 S 态 ecall (9) 时核对次数
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    csrw  mscratch, zero
    la    t0, buf_ro
    srli  t1, t0, 2
    ori   t1, t1, 0x1FF
    csrw  pmpaddr0, t1
    la    t0, buf_no
    srli  t1, t0, 2
    ori   t1, t1, 0x1FF
    csrw  pmpaddr1, t1
    li    t1, -1
    csrw  pmpaddr2, t1
    # 项 0 = NAPOT|R (0x19)，项 1 = NAPOT (0x18)，项 2 = NAPOT|RWX (0x1F)
    li    t1, (0x1F << 16) | (0x18 << 8) | 0x19
    csrw  pmpcfg0, t1
    # 进 S 态 (satp = 0，裸地址)
    la    t0, s_entry
    csrw  mepc, t0
    li    t1, 3 << 11
    csrc  mstatus, t1
    li    t1, 1 << 11
    csrs  mstatus, t1
    mret

s_entry:
    la    s0, buf_ro
    la    s1, buf_no
    ld    a0, 0(s0)                 # 允许
    li    a1, 0x5a5a
    bne   a0, a1, fail_s
    sd    a1, 8(s0)                 # 第 1 次：store 访问错误
    ld    a2, 8(s1)                 # 第 2 次：load 访问错误
    li    a3, 1
    addi  s2, s0, 16
    amoadd.d a4, a3, (s2)           # 第 3 次：store/AMO 访问错误
    sd    a1, 24(s0)                # 第 4 次：store 访问错误 (后面紧跟的 load 不能拿到它的数据)
    ld    a5, 24(s0)
    li    a6, 0x77
    bne   a5, a6, fail_s            # buf_ro+24 原值 0x77
    ecall

fail_s:
    ecall                           # mscratch 次数不对 -> 失败

    .align 2
trap:
    csrr  t0, mcause
    csrr  t2, mscratch
    li    t1, 9
    beq   t0, t1, done
    addi  t2, t2, 1
    csrw  mscratch, t2
    # 期望序列：7, 5, 7, 7
    li    t1, 1
    beq   t2, t1, exp_st
    li    t1, 2
    beq   t2, t1, exp_ld
    j     exp_st
exp_st:
    li    t1, 7
    bne   t0, t1, fail
    j     chk_tval
exp_ld:
    li    t1, 5
    bne   t0, t1, fail
chk_tval:
    csrr  t3, mtval
    la    t4, buf_ro
    la    t5, buf_no
    srli  t3, t3, 12
    srli  t4, t4, 12
    srli  t5, t5, 12
    beq   t3, t4, 1f
    bne   t3, t5, fail
1:  csrr  t0, mepc                  # 按指令长度跳过 (被拒的可能是压缩指令)
    lhu   t1, 0(t0)
    andi  t1, t1, 3
    li    t3, 3
    addi  t0, t0, 2
    bne   t1, t3, 4f
    addi  t0, t0, 2
4:  csrw  mepc, t0
    mret
done:
    # ecall 来自 S：要正好 4 次访问错误，且是从正常路径 (s_entry 末尾) 来的
    li    t1, 4
    bne   t2, t1, fail
    csrr  t0, mepc
    la    t1, fail_s
    beq   t0, t1, fail
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
2:  j     2b
fail:
    li    t0, 3
    la    t1, tohost
    sd    t0, 0(t1)
3:  j     3b

    .data
    .align 12
buf_ro:
    .dword 0x5a5a, 0, 0, 0x77
    .zero 4096 - 32
buf_no:
    .zero 4096

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
