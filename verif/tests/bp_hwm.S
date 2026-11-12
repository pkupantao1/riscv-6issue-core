# bp_hwm.S - 分支预测：要靠历史预测的分支都放在「以跨块 RVI 开头」的取指块里 (变异 M09 / bug #25)
#   取指块 32 B。每个块：.balign 32 后 15 个 c.nop (30 B) + 一条 4 字节 RVI (跨进下一块)，
#   下一块的第一条就是这条 RVI 的后半 (ftq_hwm = 1)，块里接着放模式分支。
#   #25：这种块训练 TAGE 时 PC 没加 2，与预测时的索引错位 -> 训练落到别的项上，这些分支只剩基础预测器
#   三个分支：周期 3 (取 2 / 3)、周期 5 (取 1 / 5)、与前两个都相关的 (s1 == 0 且 s2 == 0)；基础预测器各错 1/3、1/5、~1/15
#   自检：最后比较三个计数器 (与 RTL 预测无关，只防跑飞)
    .section .text.init
    .globl _start
_start:
    li    s0, 6000                 # 迭代数
    li    s1, 0                    # mod 3
    li    s2, 0                    # mod 5
    li    s3, 0                    # 分支 1 取的次数
    li    s4, 0                    # 分支 2
    li    s5, 0                    # 分支 3
    li    t3, 3
    li    t4, 5
    j     loop

    .balign 32
loop:
    .rept 15
    c.nop
    .endr
    .option push
    .option norvc
    addi  s1, s1, 1                # 跨块 RVI (30..33)
    .option pop
    bne   s1, t3, 1f               # 分支 1：s1 = 1, 2 取，3 不取 (周期 3)
    li    s1, 0
    j     2f
1:  addi  s3, s3, 1
2:  j     blk2

    .balign 32
blk2:
    .rept 15
    c.nop
    .endr
    .option push
    .option norvc
    addi  s2, s2, 1
    .option pop
    bne   s2, t4, 3f               # 分支 2：周期 5，4 次取 1 次不取
    li    s2, 0
    addi  s4, s4, 1
3:  j     blk3

    .balign 32
blk3:
    .rept 15
    c.nop
    .endr
    .option push
    .option norvc
    or    t0, s1, s2
    .option pop
    bnez  t0, 4f                   # 分支 3：s1 == 0 且 s2 == 0 时不取 (周期 15)
    addi  s5, s5, 1
4:  addi  s0, s0, -1
    bnez  s0, loop

    # 自检：6000 次里 分支 1 取 4000 次、分支 2 不取 1200 次、分支 3 不取 400 次
    li    t0, 4000
    bne   s3, t0, fail
    li    t0, 1200
    bne   s4, t0, fail
    li    t0, 400
    bne   s5, t0, fail
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
