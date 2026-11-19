# v_vdot.S - V4 点积单元 (docs/11 §9h)：vqwdotau / vqwdotas (INT8) 与 vfqwdota / vfqwdota_alt (OFP8 -> FP32) 在核内乱序。
#   每轮：按表取 vtype (LMUL m1..m8 / mf2 / mf8、altfmt 0 / 1) 与 avl，vs2 = v8 组、vs1 = v16 组 (各自从数据区 load)，
#   v0 = 掩码；4 个累加器 v24..v27 的 [0] 先从 c 表 load (含 NaN / Inf / 大数 / 次正规)，再各做一条点积 (两条带掩码)。
#   偶数轮的 FP8 数据清掉最高指数位 (没有 NaN / Inf)，奇数轮用原始随机字节。结果 / fflags 读回累加进 s6，寄存器与 Spike 整体比对
    .section .text.init
    .globl _start

# OP-VE 点积：f6 (6 位) f3 (3 位) vd vs2 vs1 vm
.macro VDOT f6, f3, vd, vs2, vs1, vm
    .word (((\f6) << 26) | ((\vm) << 25) | ((\vs2) << 20) | ((\vs1) << 15) | ((\f3) << 12) | ((\vd) << 7) | 0x77)
.endm

_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    # 数据：8 KB 随机字节
    la    a0, dat
    li    t1, 0
    li    t3, 2048
    li    t5, 0x2545F491
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    srli  t6, t4, 15
    xor   t4, t4, t6
    mul   t4, t4, t5
    srli  t6, t4, 13
    xor   t4, t4, t6
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    # 驯服区：同样的数据，每字节清 bit 6 (E4M3 / E5M2 都不会是 NaN / Inf)
    la    a1, tame
    li    t1, 0
    li    t3, 1024
    li    t6, 0xBFBFBFBFBFBFBFBF
2:  slli  t2, t1, 3
    add   t4, a0, t2
    ld    t4, 0(t4)
    and   t4, t4, t6
    add   t2, a1, t2
    sd    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 2b

    li    s6, 0
    li    s4, 0                      # 轮次
3:  csrw  fflags, x0
    # 掩码 v0
    la    a0, dat
    andi  t1, s4, 15
    slli  t1, t1, 7
    add   a3, a0, t1
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a3)
    # 累加器初值 c：v24..v27 的 [0]
    la    a4, ctab
    andi  t1, s4, 7
    slli  t1, t1, 4
    add   a4, a4, t1
    vsetivli t0, 1, e32, m1, tu, mu
    vle32.v v24, (a4)
    addi  t2, a4, 4
    vle32.v v25, (t2)
    addi  t2, a4, 8
    vle32.v v26, (t2)
    addi  t2, a4, 12
    vle32.v v27, (t2)
    # vtype / avl
    la    t2, vtab
    li    t3, 24
    remu  t1, s4, t3
    slli  t1, t1, 3
    add   t2, t2, t1
    lw    t1, 0(t2)                  # vtype
    lw    a5, 4(t2)                  # avl
    # 数据区：偶数轮驯服，奇数轮原始
    andi  t3, s4, 1
    la    a1, tame
    beqz  t3, 4f
    la    a1, dat
4:  srli  t3, s4, 1
    andi  t3, t3, 3
    slli  t3, t3, 9
    add   a1, a1, t3                 # vs2 源
    addi  a2, a1, 1536              # vs1 源 (与 vs2 错开)
    vsetvl  t0, a5, t1
    vle8.v  v8, (a1)
    vle8.v  v16, (a2)
    VDOT  0x26, 0, 24, 8, 16, 1      # vqwdotau.vv v24, v8, v16
    VDOT  0x27, 0, 25, 8, 16, 0      # vqwdotas.vv v25, v8, v16, v0.t
    VDOT  0x26, 1, 26, 8, 16, 1      # vfqwdota.vv v26, v8, v16
    VDOT  0x27, 1, 27, 8, 16, 0      # vfqwdota_alt.vv v27, v8, v16, v0.t
    # 背靠背再来一组 (依赖上一组的累加器)
    VDOT  0x26, 1, 26, 16, 8, 1
    VDOT  0x26, 0, 24, 16, 8, 0
    csrr  t4, fflags
    add   s6, s6, t4
    vsetivli t0, 1, e32, m1, ta, ma
    vmv.x.s t4, v24
    add   s6, s6, t4
    vmv.x.s t4, v25
    xor   s6, s6, t4
    vmv.x.s t4, v26
    add   s6, s6, t4
    vmv.x.s t4, v27
    xor   s6, s6, t4
    addi  s4, s4, 1
    li    t3, 96
    blt   s4, t3, 3b

    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    .data
    .align 6
# vtype (altfmt << 8 | vma << 7 | vta << 6 | sew << 3 | lmul)，avl
vtab:
    .word 0x000, 128
    .word 0x100, 77
    .word 0x001, 256
    .word 0x101, 200
    .word 0x002, 512
    .word 0x102, 301
    .word 0x003, 1024
    .word 0x103, 999
    .word 0x007, 64
    .word 0x107, 33
    .word 0x005, 16
    .word 0x105, 5
    .word 0x0C3, 1024
    .word 0x1C2, 512
    .word 0x041, 1
    .word 0x141, 255
    .word 0x006, 32
    .word 0x106, 17
    .word 0x003, 700
    .word 0x103, 1024
    .word 0x000, 3
    .word 0x100, 128
    .word 0x002, 64
    .word 0x102, 450
# c 初值 (4 个一组：v24 v25 整数，v26 v27 浮点)
    .align 4
ctab:
    .word 0x00000000, 0xFFFFFF00, 0x00000000, 0x80000000
    .word 0x12345678, 0x7FFFFFF0, 0x3F800000, 0xC2C80000
    .word 0xDEADBEEF, 0x80000001, 0x7F800000, 0x7FC00000
    .word 0x00000001, 0x00000003, 0x00000005, 0x80400000
    .word 0x7FFFFFFF, 0x0000FFFF, 0x7F7FFFFF, 0xFF7FFFFF
    .word 0xFFFFFFFF, 0x00010000, 0xFF800000, 0x7FA00000
    .word 0x00000080, 0x55555555, 0x4B000000, 0xCB000001
    .word 0x80000000, 0x00000000, 0x00800000, 0x80800000
    .align 12
dat:   .zero 8192
tame:  .zero 8192
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
