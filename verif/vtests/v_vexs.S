# v_vexs.S - V1.2c (docs/11 §9h)：带标量源的向量运算 (.vx 整数 / .vf 浮点) 在核内乱序：
#   标量由数据半边 (STD) 读出按 ROB 号送进 VIQ。每轮：标量由紧前面的标量指令算出 (依赖)，
#   LMUL 1 / 2 / 4 / 8，带不带掩码；SEW 16 的 .vf 用 flw 读进来的 f 寄存器 (对 FP16 不是合法 NaN-boxing -> 规范 NaN)。
#   结果寄存器与 Spike 整体比对，fflags / 读回的标量累加进 s6。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    la    a0, dat
    li    t1, 0
    li    t3, 1024
    li    t5, 0x1B873593
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    srli  t6, t4, 11
    xor   t4, t4, t6
    li    t6, 0x3FFFFFFF
    and   t4, t4, t6
    li    t6, 0x3E000000
    or    t4, t4, t6
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s6, 0
    li    s4, 24
2:  la    a0, dat
    andi  t1, s4, 3
    slli  t1, t1, 8
    add   a0, a0, t1
    csrw  fflags, x0
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a0)
    # 标量：由前面的标量运算得到 (依赖)
    mul   a3, s4, s4
    addi  a3, a3, -77
    slli  a4, s4, 3
    flw   fa0, 16(a0)
    fadd.s fa1, fa0, fa0
    fld   fa2, 32(a0)
    # ---- e32 m4：整数 .vx
    li    t3, 37
    mul   t3, t3, s4
    vsetvli t0, t3, e32, m4, tu, mu
    vle32.v v4, (a0)
    vadd.vx  v8, v4, a3
    vsub.vx  v8, v8, a4, v0.t
    vrsub.vx v12, v8, a3
    vand.vx  v12, v12, a4
    vsll.vx  v16, v4, a4
    vmin.vx  v16, v16, a3, v0.t
    vmerge.vxm v20, v4, a3, v0
    vmv.v.x  v24, a4
    # ---- e32 m8：浮点 .vf (fa1 = FP32 合法 NaN-boxing)
    vsetvli t0, t3, e32, m8, tu, mu
    vle32.v v8, (a0)
    vfadd.vf  v16, v8, fa1
    vfmul.vf  v24, v16, fa0, v0.t
    vfmacc.vf v24, fa1, v8
    vfrsub.vf v16, v24, fa0
    vfnmsac.vf v16, fa0, v8, v0.t
    vfmadd.vf v8, fa1, v16
    # ---- e64 m2：fa2 (FP64)
    vsetvli t0, t3, e64, m2, tu, mu
    vle64.v v2, (a0)
    vfsub.vf v6, v2, fa2
    vfmacc.vf v6, fa2, v2, v0.t
    vadd.vx  v10, v2, a3
    # ---- e16 m1：fa0 是 flw 读的 (高 32 位全 1，对 FP16 不是合法 NaN-boxing -> 规范 NaN)
    vsetvli t0, t3, e16, m1, tu, mu
    vle16.v v1, (a0)
    vfadd.vf v3, v1, fa0
    vfmul.vf v5, v1, fa1, v0.t
    # 汇总
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v1, v12, v16
    vadd.vv v1, v1, v20
    vadd.vv v1, v1, v24
    vadd.vv v1, v1, v8
    vadd.vv v1, v1, v6
    vadd.vv v1, v1, v3
    vadd.vv v1, v1, v5
    vadd.vv v1, v1, v10
    vmv.x.s t4, v1
    add   s6, s6, t4
    csrr  t4, fflags
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
    .zero 1024
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
