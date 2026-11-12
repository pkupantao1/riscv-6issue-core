# v_mem.S - 向量访存 (V1)：单位步长 / 跨步，各 EEW / LMUL，掩码，vstart 恢复，访问错误 (vstart = 出错元素)
#   正确性由对拍判定；通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)  # mstatus.VS / FS = Initial
    csrs  mstatus, t0

    # 源数据：src 填 0..255 的字节模式
    la    a0, src
    li    t1, 0
1:  add   t2, a0, t1
    sb    t1, 0(t2)
    addi  t1, t1, 1
    li    t3, 1024
    blt   t1, t3, 1b

    # ---------------- 单位步长
    li    a1, 100
    vsetvli t0, a1, e8, m1, ta, ma
    la    a0, src+3                       # 不对齐的基址 (EEW = 8 无所谓)
    vle8.v v1, (a0)
    la    a2, dst
    vse8.v v1, (a2)
    vsetvli t0, a1, e16, m2, ta, ma
    la    a0, src+2
    vle16.v v2, (a0)
    vadd.vi v2, v2, 1
    la    a2, dst+256
    vse16.v v2, (a2)
    vsetvli t0, a1, e32, m4, ta, ma
    la    a0, src+4
    vle32.v v4, (a0)
    la    a2, dst+512
    vse32.v v4, (a2)
    li    a1, 37
    vsetvli t0, a1, e64, m8, ta, ma
    la    a0, src+8
    vle64.v v8, (a0)
    la    a2, dst+1024
    vse64.v v8, (a2)
    # EEW 与 SEW 不同 (EMUL = EEW / SEW * LMUL)
    li    a1, 50
    vsetvli t0, a1, e32, m2, ta, ma
    la    a0, src+1
    vle8.v v16, (a0)                      # EMUL = 1/2
    vle64.v v24, (a0)                     # 地址不对齐 -> 异常 (EEW 64)，陷入后跳过
    la    a0, src+16
    vle64.v v24, (a0)                     # EMUL = 4
    # 标量 load 紧跟向量 store (向量 store 后冲刷，标量要读到新值)
    la    a2, dst+2048
    vse32.v v4, (a2)
    lw    t4, 4(a2)
    ld    t5, 8(a2)

    # ---------------- 掩码
    vsetvli t0, a1, e8, m1, ta, ma
    li    a3, 0x6db6db6db6db6db6
    vmv.s.x v0, a3
    vsetvli t0, a1, e16, m1, tu, mu
    la    a0, src+64
    vle16.v v3, (a0), v0.t
    la    a2, dst+3000
    vse16.v v3, (a2), v0.t

    # ---------------- 跨步
    li    a1, 20
    vsetvli t0, a1, e32, m1, ta, ma
    la    a0, src
    li    a4, 12
    vlse32.v v5, (a0), a4
    li    a4, -8
    la    a0, src+512
    vlse32.v v6, (a0), a4
    vlse32.v v7, (a0), zero               # 步长 0
    la    a2, dst+3600
    li    a4, 20
    vsse32.v v5, (a2), a4
    vsetvli t0, a1, e64, m2, ta, ma
    la    a0, src+8
    li    a4, 24
    vlse64.v v10, (a0), a4

    # ---------------- vstart 恢复：从第 5 个元素开始
    vsetvli t0, a1, e8, m1, tu, mu
    la    a0, src+100
    csrwi vstart, 5
    vle8.v v12, (a0)

    # ---------------- 访问错误：第 2 个元素落在主存之外 -> mcause = 5，vstart = 2
    li    a1, 8
    vsetvli t0, a1, e64, m1, ta, ma
    la    a0, src
    li    a4, -0x40000000
    vlse64.v v13, (a0), a4
    csrr  t6, vstart
    csrw  vstart, zero
    # 写回最终数据再读一遍 (对拍会比较读出值)
    la    a2, dst
    ld    s2, 0(a2)
    ld    s3, 256(a2)
    ld    s4, 1024(a2)

    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

    # 陷入：记下 mcause，跳过出错的指令继续
    .align 2
trap:
    csrr  s0, mcause
    csrr  s1, mepc
    addi  s1, s1, 4
    csrw  mepc, s1
    mret

    .data
    .align 6
src: .zero 1024
dst: .zero 4096

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
