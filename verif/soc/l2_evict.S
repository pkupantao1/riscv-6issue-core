# l2_evict.S - SoC 模式：L2 换出脏行 (写缺失换出 S_VRD、回填换出 S_FVRD)
#   L2 1 MB / 8 路 / 2048 组：相距 128 KB 的行落在 L2 同一组，也落在 D$ 同一组 (D$ 组号是地址 [11:6])。
#   每轮对 24 行 (同组，远超 D$ 16 路、L2 8 路) 各写两个双字 -> D$ 换出脏行写回 L2 -> L2 组里脏行超过 8 路换出；
#   再逐行读回核对 -> 读缺失回填时牺牲行是脏的。两个组 (+0、+64) 各做 3 轮，每轮写入值不同
#   自检 + 对拍
    .equ  BASE,  0x80400000
    .equ  STR,   0x20000              # 128 KB
    .equ  NL,    24
    .section .text.init
    .globl _start
_start:
    li    s0, BASE
    li    s1, STR
    li    s5, 0                       # 组偏移 0 / 64
    li    s6, 2
grp:
    li    s7, 0                       # 轮
    li    s8, 3
round:
    # 写：第 i 行 [0] = (轮 << 16) | i + 1，[8] = ~那个值
    add   t0, s0, s5
    li    t1, 0
1:  slli  t2, s7, 16
    or    t2, t2, t1
    addi  t2, t2, 1
    sd    t2, 0(t0)
    not   t3, t2
    sd    t3, 8(t0)
    add   t0, t0, s1
    addi  t1, t1, 1
    li    t4, NL
    blt   t1, t4, 1b
    # 读回核对
    add   t0, s0, s5
    li    t1, 0
2:  slli  t2, s7, 16
    or    t2, t2, t1
    addi  t2, t2, 1
    ld    t3, 0(t0)
    bne   t3, t2, fail
    ld    t3, 8(t0)
    not   t2, t2
    bne   t3, t2, fail
    add   t0, t0, s1
    addi  t1, t1, 1
    li    t4, NL
    blt   t1, t4, 2b
    addi  s7, s7, 1
    blt   s7, s8, round
    addi  s5, s5, 64
    addi  s6, s6, -1
    bnez  s6, grp
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
3:  j     3b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
4:  j     4b

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
