# v_vexf.S - V1.2a (docs/11 §9h)：浮点向量运算 (.vv 加 / 减 / 乘 / 8 种乘加) 在核内乱序、进 o6_vex。
#   每轮：SEW 16 / 32 / 64 (LMUL 1 或 1/2)、vl 随轮变、带不带掩码、frm 随轮变 (RNE / RTZ / RDN / RUP / RMM)；
#   依赖链 (连续 vfmacc 累加)、与整数运算 / load / store 交错；操作数里混有大数 / 小数 / 0 / 无穷 / NaN，
#   每轮读 fflags 累加进 s6 (与 Spike 比对：fflags 只来自活动元素)。结果寄存器与 Spike 整体比对。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    # 数据区 4 KB：伪随机字 + 每 16 字插几个特殊值
    la    a0, dat
    li    t1, 0
    li    t3, 1024
    li    t5, 0x2C1B3C6D
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    srli  t6, t4, 7
    xor   t4, t4, t6
    andi  t6, t1, 15
    li    a1, 3
    bne   t6, a1, 2f
    li    t4, 0x7F800000                   # +inf (FP32)
2:  li    a1, 7
    bne   t6, a1, 3f
    li    t4, 0x7FC00001                   # NaN
3:  li    a1, 11
    bne   t6, a1, 4f
    li    t4, 0
4:  sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s6, 0
    li    s4, 30
5:  la    a0, dat
    andi  t1, s4, 7
    slli  t1, t1, 7
    add   a0, a0, t1
    # frm 随轮变 (0..4)
    li    t2, 5
    remu  t2, s4, t2
    csrw  frm, t2
    csrw  fflags, x0
    # v0 掩码图案
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a0)
    # ---- SEW 32, LMUL 1
    andi  t3, s4, 31
    addi  t3, t3, 1
    vsetvli t0, t3, e32, m1, tu, mu
    vle32.v v1, (a0)
    addi  a1, a0, 128
    vle32.v v2, (a1)
    addi  a1, a0, 256
    vle32.v v3, (a1)
    vfadd.vv  v4, v1, v2
    vfsub.vv  v5, v1, v3
    vfmul.vv  v6, v2, v3
    vfmacc.vv v3, v1, v2
    vfmacc.vv v3, v4, v5
    vfnmacc.vv v3, v6, v1, v0.t
    vfmsac.vv v4, v2, v6
    vfnmsac.vv v5, v1, v1
    vfmadd.vv v6, v2, v3
    vfnmadd.vv v7, v3, v4, v0.t
    vfmsub.vv v8, v4, v5
    vfnmsub.vv v9, v5, v6
    vadd.vv   v10, v8, v9                  # 整数运算交错 (按位)
    vfadd.vv  v11, v10, v7, v0.t
    addi  a1, a0, 384
    vse32.v   v11, (a1)
    csrr  t4, fflags
    add   s6, s6, t4
    # ---- SEW 64, LMUL 1
    vsetvli t0, t3, e64, m1, tu, mu
    vle64.v v12, (a0)
    vle64.v v13, (a1)
    vfmul.vv  v14, v12, v13
    vfmacc.vv v14, v12, v13
    vfsub.vv  v15, v14, v12, v0.t
    vfmsub.vv v15, v12, v13
    csrr  t4, fflags
    add   s6, s6, t4
    # ---- SEW 16, LMUL 1/2
    slli  t3, t3, 1
    vsetvli t0, t3, e16, mf2, tu, mu
    vle16.v v16, (a0)
    vle16.v v17, (a1)
    vfadd.vv  v18, v16, v17
    vfmul.vv  v19, v16, v17, v0.t
    vfmacc.vv v18, v19, v16
    vfnmsub.vv v19, v18, v17
    csrr  t4, fflags
    add   s6, s6, t4
    # 结果混进 s6
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v20, v3, v11
    vadd.vv v20, v20, v14
    vadd.vv v20, v20, v18
    vmv.x.s t4, v20
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 5b

    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
    .align 12
dat:   .zero 4096
    .zero 1024
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
