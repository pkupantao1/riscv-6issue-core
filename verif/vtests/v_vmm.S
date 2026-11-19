# v_vmm.S - M4a (docs/11 §9g)：带掩码 (vm = 0) 的单位步长向量 load / store 进共用 LSU。
#   每轮换一个 v0 图案 (由轮号算出的伪随机字节) 与基址偏移 (窗口跨行 / 不跨行)：
#   1) 带掩码 vle8 / 16 / 32 / 64 (LMUL 1 / 2 / 4 / 8，vl 随轮变，mu)：被掩掉的元素保持旧值
#   2) 带掩码 vse 写一段后：标量 load 读其中各处、vle (不带掩码) 读整段 —— 被掩掉的字节保持内存旧值 (含转发)
#   3) 带掩码 vse 后再带掩码 vle 读同一段
#   结果在向量寄存器里与 Spike 整体比对，标量读回累加 s6，最后标量读回整个缓冲区。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    la    a0, buf
    li    t1, 0
    li    t3, 2048
    li    t5, 0x41C64E6D
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    addi  t4, t4, 1234
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    # 寄存器初值：v8..v15 = buf 前 1 KB，v16..v23 = buf 第二个 1 KB
    vsetvli t0, x0, e8, m8, ta, ma
    vle8.v  v8, (a0)
    addi  a1, a0, 1024
    vle8.v  v16, (a1)
    li    s6, 0
    li    s4, 24
2:  # v0 = 伪随机字节：mask 区第 (s4 & 7) × 128 起
    la    a1, mpat
    andi  t1, s4, 7
    slli  t1, t1, 7
    add   a1, a1, t1
    vsetvli t0, x0, e8, m1, ta, ma
    vle8.v  v0, (a1)
    # 基址
    la    a0, buf
    andi  t1, s4, 7
    la    t2, offs
    slli  t1, t1, 2
    add   t2, t2, t1
    lw    t1, 0(t2)
    add   a0, a0, t1
    slli  t2, s4, 7
    andi  t2, t2, 2047
    add   a0, a0, t2
    andi  a2, a0, -8                   # 标量访存用按 8 对齐的基址
    # 1) 带掩码 load
    andi  t3, s4, 31
    addi  t3, t3, 100
    vsetvli t0, t3, e8, m1, tu, mu
    vmv.v.v v1, v9
    vle8.v  v1, (a0), v0.t
    vsetvli t0, t3, e16, m2, tu, mu
    vmv.v.v v2, v10                    # (LMUL 2：整组 v2..v3)
    vle16.v v2, (a2), v0.t
    vsetvli t0, t3, e32, m4, tu, mu
    vmv.v.v v4, v12
    vle32.v v4, (a2), v0.t
    vsetvli t0, x0, e64, m8, tu, mu
    vmv.v.v v24, v16
    vle64.v v24, (a2), v0.t
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v5, v1, v2
    vadd.vv v5, v5, v4
    vadd.vv v5, v5, v31
    vmv.x.s t4, v5
    add   s6, s6, t4
    # 2) 带掩码 vse -> 标量 load / vle
    vsetvli t0, t3, e8, m1, tu, mu
    vadd.vx v6, v8, s4
    vse8.v  v6, (a0), v0.t
    ld    t4, 0(a2)
    add   s6, s6, t4
    ld    t4, 64(a2)
    add   s6, s6, t4
    vle8.v  v7, (a0)
    vsetvli t0, t3, e32, m2, tu, mu
    vadd.vx v12, v18, s6
    vse32.v v12, (a2), v0.t
    lw    t4, 4(a2)
    add   s6, s6, t4
    vsetvli t0, t3, e32, m2, ta, ma
    vle32.v v14, (a2)
    # 3) 带掩码 vse 后带掩码 vle
    vsetvli t0, t3, e16, m1, tu, mu
    vse16.v v13, (a2), v0.t
    vmv.v.v v15, v9
    vle16.v v15, (a2), v0.t
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v5, v7, v14
    vadd.vv v5, v5, v15
    vmv.x.s t4, v5
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b

    la    a0, buf
    li    t1, 1024
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
offs:  .word 0, 3, 61, 64, 100, 127, 8, 90
    .align 7
mpat:
    .rept 256
    .word 0xA5C3965A, 0x0FF07E81, 0x3C69D2B4, 0xF0E1C3B7
    .endr
    .align 12
buf:   .zero 8192
    .zero 1024
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
