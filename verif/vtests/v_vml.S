# v_vml.S - M3a (docs/11 §9g)：向量 load 在译码展开成 uop，进共用 LSU (load 流水整行读、两遍跨行、VECU 写口补尾部)。
#   每轮 (基址偏移 off 随轮变：0 / 4 / 60 / 64 / 120 / 124 / 8 / 72，使窗口跨行或不跨)：
#   1) vle8 / 16 / 32 / 64，LMUL 1 / 2 / 4 / 8、vl 随轮变 (尾部部分有效、整寄存器无效)，后面紧跟读它的乱序 vadd
#   2) 标量 store 写同一行后紧跟 vle (要等 store 排空后再读)
#   3) vl1re32 / vl2re64 / vl4re8 / vl8re16 整寄存器、vlm 掩码
#   4) 误预测分支影子里的 vle (作废)；vle 后接 ecall (陷入冲刷)
#   5) 跨 4 KB 页的窗口 (第二遍跨页 -> replay -> 走原路径)
#   结果在向量寄存器里与 Spike 整体比对；vmv.x.s 读回的标量累加 s6。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    # src (8 KB) 填充：字 i = i * 0x2545F491 + 7
    la    a0, src
    li    t1, 0
    li    t3, 2048
    li    t5, 0x2545F491
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    addi  t4, t4, 7
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    li    s6, 0
    li    s4, 32                       # 轮数
2:
    la    a0, src
    andi  t1, s4, 7
    la    t2, offs
    slli  t1, t1, 2
    add   t2, t2, t1
    lw    t1, 0(t2)
    add   a0, a0, t1                   # a0 = src + off
    slli  t2, s4, 6
    andi  t2, t2, 1023
    add   a0, a0, t2                   # 再按轮挪 (64 的倍数)
    # 1) 各 EEW / LMUL，vl = 轮数相关
    andi  t3, s4, 31
    addi  t3, t3, 1
    vsetvli t0, t3, e8, m1, tu, mu
    vle8.v  v1, (a0)
    vadd.vv v2, v1, v1
    slli  t3, t3, 2
    vsetvli t0, t3, e16, m2, tu, mu
    vle16.v v4, (a0)
    vadd.vv v6, v4, v4
    vsetvli t0, t3, e32, m4, ta, ma
    vle32.v v8, (a0)
    vsetvli t0, x0, e64, m8, ta, ma
    vle64.v v16, (a0)
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v24, v8, v9
    vmv.x.s t4, v24
    add   s6, s6, t4
    vmv.x.s t4, v19
    add   s6, s6, t4
    # 2) 标量 store 后紧跟同一行的 vle
    sw    s4, 8(a0)
    sd    s6, 16(a0)
    vle32.v v25, (a0)
    vmv.x.s t4, v25
    add   s6, s6, t4
    # 3) 整寄存器 / 掩码
    vl1re32.v v26, (a0)
    vl2re64.v v28, (a0)
    vl4re8.v  v12, (a0)
    vl8re16.v v16, (a0)
    vsetvli t0, x0, e8, m1, ta, ma
    vlm.v     v0, (a0)
    vsetvli t0, x0, e32, m1, ta, mu
    vadd.vv   v27, v26, v12, v0.t
    # 4) 误预测影子里的 vle、ecall
    andi  t2, s4, 3
    beqz  t2, 3f
    vle32.v v30, (a0)
3:
    vle32.v v31, (a0)
    ecall
    vadd.vv v30, v31, v30
    # 5) 跨页：窗口从页尾前 40 B 起
    la    t2, src+4096-40
    vsetvli t0, x0, e32, m2, ta, ma
    vle32.v v10, (t2)
    vsetvli t0, x0, e32, m1, ta, ma
    vmv.x.s t4, v11
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b

    la    t1, res
    sd    s6, 0(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

trap:
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret

    .data
offs:  .word 0, 4, 60, 64, 120, 124, 8, 72
    .align 12
src:   .zero 8192
    .zero 2048
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
