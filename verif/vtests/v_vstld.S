# v_vstld.S - 向量 store 与更年轻的标量 load (P2.4a，docs/11 §7)：向量 store 不再无条件冲刷，
#   只在写的行与已执行 / 正在执行的更年轻 load 冲突时冲刷。这里让标量 load 紧跟向量 store、地址早就算好
#   (可以在 store 到 ROB 头之前推测执行)，分别读：刚写的同一行 / 同一次 store 的后几行 / 不相干的行。
#   读到旧值会与 Spike 对拍不一致。每轮改写入的数据，结果累加进 s2 / s3 并写回内存。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)  # mstatus.VS / FS = Initial
    csrs  mstatus, t0
    la    a0, src
    li    t1, 0
1:  add   t2, a0, t1
    sb    t1, 0(t2)
    addi  t1, t1, 3
    li    t3, 1024
    blt   t1, t3, 1b

    li    s2, 0
    li    s3, 0
    li    s4, 40                      # 轮数
    la    a0, src
    la    a2, dst
    la    a3, other
2:
    # e32 m4：一次 store 写 512 B (8 行)；也换 m1 / m8 让突发长度不同
    andi  t6, s4, 3
    li    a1, 128
    beqz  t6, 3f
    vsetvli t0, a1, e32, m4, ta, ma
    j     4f
3:  li    a1, 40
    vsetvli t0, a1, e64, m8, ta, ma
4:  vle32.v v8, (a0)
    vadd.vx v8, v8, s4
    vse32.v v8, (a2)
    # 紧跟的标量 load：同一行、后面的行、行尾、不相干的行
    lw    t1, 0(a2)
    lw    t2, 132(a2)
    ld    t3, 448(a2)
    lw    t4, 60(a2)
    ld    t5, 0(a3)
    add   s2, s2, t1
    add   s2, s2, t2
    add   s3, s3, t3
    add   s3, s3, t4
    add   s3, s3, t5
    # 下一轮写 dst 的另一段 (同一批行会被反复写)，也改 other
    sd    s2, 0(a3)
    addi  a2, a2, 64
    la    t0, dst+1024
    blt   a2, t0, 5f
    la    a2, dst
5:  addi  s4, s4, -1
    bnez  s4, 2b

    la    t0, res
    sd    s2, 0(t0)
    sd    s3, 8(t0)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
    .align 6
src:   .zero 1024
dst:   .zero 2048
other: .zero 64
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
