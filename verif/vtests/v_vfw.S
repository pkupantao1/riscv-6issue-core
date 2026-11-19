# v_vfw.S - M3c (docs/11 §9g)：store -> load 按字节转发，任意宽度。
#   每轮 (基址偏移随轮变，窗口跨行 / 不跨行)，store 都还在 SQ 里时紧接着读：
#   1) 标量 sb / sh / sw / sd 写同一行的几处 -> vle 读整段 (标量 store -> 向量 load，与 D$ 旧值逐字节合并)
#   2) vse 写一段 -> 标量 lbu / lhu / lw / ld 读其中各处 (向量 store -> 标量 load，含跨窗口边界的 ld)
#   3) vse A，sb 改其中一字节，vse B (vl 短、地址错开 5 B) 部分覆盖 -> vle 读 A 的整段 (每字节取最年轻的 store)
#   4) vse (vl 短) 后 vle 读得更长：窗口之外的字节来自更老的 store / D$
#   5) 两条 vse 写相邻两行后 vle 读跨这两行的窗口 (两遍都转发)
#   结果在向量寄存器里与 Spike 整体比对，标量读回累加 s6。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    la    a0, buf
    li    t1, 0
    li    t3, 1024
    li    t5, 0x6C078965
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    addi  t4, t4, 3
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    la    a0, src
    vsetvli t0, x0, e8, m8, ta, ma
    vle8.v  v8, (a0)
    li    s6, 0
    li    s4, 24
2:  la    a0, buf
    andi  t1, s4, 7
    la    t2, offs
    slli  t1, t1, 2
    add   t2, t2, t1
    lw    t1, 0(t2)
    add   a0, a0, t1
    slli  t2, s4, 7
    andi  t2, t2, 1023
    add   a0, a0, t2                   # a0 = buf + off + (轮 × 128) mod 1024
    andi  a1, a0, -8                   # 标量访存用按 8 对齐的基址 (本核不支持不对齐)
    # 1) 标量 store -> vle
    sb    s4, 1(a1)
    sh    s4, 38(a1)
    sw    s6, 64(a1)
    sd    s6, 120(a1)
    vsetvli t0, x0, e8, m1, tu, mu
    vle8.v  v1, (a0)
    vadd.vv v2, v1, v1
    # 2) vse -> 标量 load
    vadd.vx v3, v8, s4
    vse8.v  v3, (a0)
    lbu   t4, 7(a1)
    add   s6, s6, t4
    lhu   t4, 62(a1)
    add   s6, s6, t4
    lw    t4, 124(a1)
    add   s6, s6, t4
    ld    t4, 128(a1)                  # 跨窗口末尾 (窗口之外的字节来自 D$)
    add   s6, s6, t4
    # 3) vse A，sb，vse B (短、错开 5 B) -> vle 读 A 整段
    vadd.vx v4, v9, s6
    vse8.v  v4, (a0)
    sb    s4, 70(a1)
    li    t3, 40
    vsetvli t0, t3, e8, m1, tu, mu
    addi  t2, a0, 5
    vadd.vx v5, v10, s4
    vse8.v  v5, (t2)
    vsetvli t0, x0, e8, m1, tu, mu
    vle8.v  v6, (a0)
    vmv.x.s t4, v6
    add   s6, s6, t4
    # 4) 短 vse 后 vle 读得更长
    li    t3, 17
    vsetvli t0, t3, e16, m1, tu, mu
    vse16.v v11, (a1)
    vsetvli t0, x0, e16, m1, tu, mu
    vle16.v v7, (a1)
    # 5) 相邻两行各一条 vse，vle 读跨两行的窗口
    vsetvli t0, x0, e32, m1, ta, ma
    andi  t2, a0, -128
    vse32.v v12, (t2)
    addi  t3, t2, 128
    vse32.v v13, (t3)
    addi  t3, t2, 72
    vle32.v v14, (t3)
    vadd.vv v15, v14, v7
    vmv.x.s t4, v15
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b

    la    a0, buf
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
offs:  .word 0, 3, 61, 64, 100, 127, 8, 90
    .align 12
src:   .zero 1024
    .align 12
buf:   .zero 4096
    .zero 512
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
