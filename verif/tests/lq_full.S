# lq_full.S - 把 LQ (128 项) 塞满：一次发 256 条互不相关、全是冷缺失的 load (ROB 258 项 > LQ 128)，重复 4 轮
#   必达 bin cg_uarch.cp_DS_LQ / cp_LQ_FULL。突发里只有 load (结果都写 t0，无依赖、不累加，免得 ROB 被别的指令占掉一半)；
#   每 32 条换一次基址 (12 位偏移只够 2 KB)；最后读回各轮写在段首的标记做自检
    .section .text.init
    .globl _start
_start:
    li    s1, 4                    # 轮数 (也作标记值)
    la    s0, lbuf
1:  sd    s1, 0(s0)                # 本段 (16 KB) 段首写标记
    mv    s3, s0
    .rept 8                        # 8 组 x 32 条 = 256 条，步长 64 B (每条一个新行)
    .set  k, 0
    .rept 32
    ld    t0, k(s3)
    .set  k, k + 64
    .endr
    addi  s3, s3, 2047
    addi  s3, s3, 1
    .endr
    li    t1, 16384
    add   s0, s0, t1
    addi  s1, s1, -1
    bnez  s1, 1b
    # 自检：4 段段首的标记依次是 4 3 2 1
    la    s0, lbuf
    li    s1, 4
2:  ld    t0, 0(s0)
    bne   t0, s1, fail
    li    t1, 16384
    add   s0, s0, t1
    addi  s1, s1, -1
    bnez  s1, 2b
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
lbuf:
    .zero 65536

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
