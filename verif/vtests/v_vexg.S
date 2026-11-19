# v_vexg.S - V1.2b (docs/11 §9h)：LMUL 2 / 4 / 8 的向量运算整条发射，执行器逐拍做 (改名时整组分配物理寄存器)。
#   每轮：vl 随轮变 (含只覆盖组里前几个寄存器的)，SEW 8 / 16 / 32 / 64，带不带掩码；
#   整数 (加减 / 逻辑 / 移位 / 最值 / merge / mv) 与浮点 (加减乘、乘加) 的组与组依赖链，与 vle / vse 交错；
#   结果寄存器与 Spike 整体比对，fflags 与读回的标量累加进 s6。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    la    a0, dat
    li    t1, 0
    li    t3, 2048
    li    t5, 0x3B9ACA07
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    srli  t6, t4, 9
    xor   t4, t4, t6
    li    t6, 0x3FFFFFFF                   # FP32 指数压到不溢出的范围
    and   t4, t4, t6
    li    t6, 0x3E000000
    or    t4, t4, t6
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s6, 0
    li    s4, 20
2:  la    a0, dat
    andi  t1, s4, 3
    slli  t1, t1, 9
    add   a0, a0, t1
    csrw  fflags, x0
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a0)                       # 掩码
    # ---- e32 m8：vl = 轮数 × 13 (覆盖 1..8 个寄存器)
    li    t3, 13
    mul   t3, t3, s4
    vsetvli t0, t3, e32, m8, tu, mu
    vle32.v v8, (a0)
    addi  a1, a0, 1024
    vle32.v v16, (a1)
    vadd.vv  v24, v8, v16
    vxor.vv  v24, v24, v8, v0.t
    vsll.vi  v24, v24, 3
    vfadd.vv v16, v8, v16
    vfmacc.vv v16, v8, v8
    vfmul.vv v8, v16, v8, v0.t
    vfnmsub.vv v8, v16, v24
    li    a2, 4096
    add   a2, a2, a0
    vse32.v  v8, (a2)
    # ---- e16 m4
    slli  t3, t3, 1
    vsetvli t0, t3, e16, m4, tu, mu
    vle16.v v4, (a0)
    vmaxu.vv v12, v4, v20
    vmin.vv  v12, v12, v28, v0.t
    vmerge.vvm v20, v4, v12, v0
    vfsub.vv v28, v4, v20
    vfmadd.vv v28, v4, v12, v0.t
    # ---- e64 m2
    vsetvli t0, t3, e64, m2, tu, mu
    vle64.v v2, (a1)
    vsub.vv  v6, v2, v8
    vsra.vi  v6, v6, 5
    vrsub.vi v10, v6, 7
    vfmul.vv v14, v2, v2
    vfmsac.vv v14, v2, v6, v0.t
    # ---- e8 m8 / LMUL 8 的 vmv.v.v
    vsetvli t0, t3, e8, m8, tu, mu
    vmv.v.v  v16, v24
    vand.vv  v16, v16, v8
    vor.vi   v16, v16, 5, v0.t
    # 汇总
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v1, v16, v24
    vadd.vv v1, v1, v28
    vadd.vv v1, v1, v14
    vadd.vv v1, v1, v8
    vmv.x.s t4, v1
    add   s6, s6, t4
    csrr  t4, fflags
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b

    la    a0, dat+4096
    li    t1, 512
5:  ld    t4, 0(a0)
    add   s6, s6, t4
    addi  a0, a0, 8
    addi  t1, t1, -1
    bnez  t1, 5b
    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
    .align 12
dat:   .zero 8192
    .zero 1024
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
