# v_vexr.S - 复现 vrand 种子 9808：FP16、LMUL 2、带掩码的 vfnmadd.vv (vl = 20，第二个寄存器全是尾部)，
#   之后原路径的 vfncvtbf16 读它、vnmsub.vx 改它，再接核内乱序的 vfnmacc.vf (带掩码)。结果与 Spike 整体比对
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    la    a0, dat
    li    t1, 0
    li    t3, 1024
    li    t5, 0x5851F42D
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    srli  t6, t4, 13
    xor   t4, t4, t6
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s4, 8
2:  la    a0, dat
    slli  t1, s4, 6
    add   a0, a0, t1
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a0)
    li    t3, 20
    vsetvli t0, t3, e16, m2, tu, mu
    addi  a1, a0, 256
    vle16.v v12, (a1)
    addi  a1, a0, 512
    vle16.v v16, (a1)
    addi  a1, a0, 768
    vle16.v v22, (a1)
    addi  a1, a0, 1024
    vle16.v v26, (a1)
    vle16.v v2, (a0)
    flh   fs2, 2(a0)
    li    a2, 77
    vfnmadd.vv v12, v16, v22, v0.t
    vfncvtbf16.f.f.w v2, v12
    vnmsub.vx v12, a2, v28
    vfnmacc.vf v2, fs2, v26, v0.t
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v1, v2, v12
    vmv.x.s t4, v1
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b
    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
    .align 12
dat:   .zero 4096
    .zero 2048
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
