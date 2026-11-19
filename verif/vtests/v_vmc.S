# v_vmc.S - M3b 压力：大块冷数据的向量拷贝 (vle8 / vse8，LMUL 8，每条 8 个 load uop + 8 个 store uop)，
#   源 / 目的偏移各轮不同 (窗口大量跨行)，每轮拷 12000 B，拷完标量读回整个目的段累加 (每条 load 都与 Spike 对拍)。
#   第二、三轮的目的段与上一轮重叠 (同一行上先后有不同轮的向量 store 与标量 load)。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    # src (32 KB)：字 i = i * 0x9E3779B1 + 13
    la    a0, src
    li    t1, 0
    li    t3, 8192
    li    t5, 0x9E3779B1
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    addi  t4, t4, 13
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s6, 0
    la    s1, offs
    li    s2, 4                        # 轮数
2:  lw    t1, 0(s1)                    # 源偏移
    lw    t2, 4(s1)                    # 目的偏移
    la    a1, src
    add   a1, a1, t1
    la    a2, dst
    add   a2, a2, t2
    mv    s3, a2
    li    a3, 12000
3:  vsetvli t0, a3, e8, m8, ta, ma
    vle8.v  v8, (a1)
    vse8.v  v8, (a2)
    add   a1, a1, t0
    add   a2, a2, t0
    sub   a3, a3, t0
    bnez  a3, 3b
    # 读回 (字节累加到 8 B 一组：用 ld，目的段起点按 8 对齐向下取)
    andi  a0, s3, -8
    li    t1, 1504
4:  ld    t4, 0(a0)
    add   s6, s6, t4
    xor   s6, s6, t1
    addi  a0, a0, 8
    addi  t1, t1, -1
    bnez  t1, 4b
    addi  s1, s1, 8
    addi  s2, s2, -1
    bnez  s2, 2b

    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
offs:  .word 0, 0,  4, 60,  100, 3,  1, 2050
    .align 12
src:   .zero 32768
    .align 12
dst:   .zero 16384
    .zero 2048
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
