# v_vms.S - M3b (docs/11 §9g)：向量 store 在译码展开成 uop，STA 进共用 LSU、数据经 VIQ / VX 写进 SQ (128 B)，
#   排空按行写 D$ (跨行分两次)。
#   每轮 (目的偏移 off 随轮变：0 / 4 / 60 / 64 / 120 / 124 / 8 / 72，使窗口跨行或不跨)：
#   1) vse8 / 16 / 32 / 64，LMUL 1 / 2 / 4 / 8、vl 随轮变 (尾部只写 vl 个元素)，之后标量 load 读回 (等向量 store 排空)
#   2) vse 后紧跟 vle 读同一段 (向量 load 等向量 store 排空)；vse 后紧跟标量 store 覆盖其中几个字节再读
#   3) vs1r / vs2r / vs4r / vs8r 整寄存器、vsm 掩码
#   4) 误预测分支影子里的 vse (作废，不能写内存)；vse 后接 ecall (陷入冲刷)
#   5) 跨 4 KB 页的窗口 (STA 跨页 -> replay -> 走原路径)
#   6) store 基址由除法算出 (STA 晚)，后面同地址的标量 load 先执行 -> 违例 replay
#   7) vle / vse 的拷贝循环
#   最后标量读整个 dst 区累加进 s6，与 Spike 比对 (每条 load 的写回都对拍)。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)
    csrs  mstatus, t0
    # src (8 KB) 填充：字 i = i * 0x2545F491 + 7；dst 清零 (已在 .zero 里)
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
    # 把源读进 v0..v31 (各寄存器内容不同)
    la    a0, src
    vsetvli t0, x0, e8, m8, ta, ma
    vle8.v  v8, (a0)
    addi  a1, a0, 1024
    vle8.v  v16, (a1)
    addi  a1, a1, 1024
    vle8.v  v24, (a1)
    li    s6, 0
    li    s4, 32                       # 轮数
2:
    la    a0, dst
    andi  t1, s4, 7
    la    t2, offs
    slli  t1, t1, 2
    add   t2, t2, t1
    lw    t1, 0(t2)
    add   a0, a0, t1                   # a0 = dst + off
    slli  t2, s4, 6
    andi  t2, t2, 1023
    add   a0, a0, t2                   # 再按轮挪 (64 的倍数)
    # 1) 各 EEW / LMUL，vl = 轮数相关；后面标量读回
    andi  t3, s4, 31
    addi  t3, t3, 1
    vsetvli t0, t3, e8, m1, tu, mu
    vadd.vx v1, v8, s4
    vse8.v  v1, (a0)
    lbu   t4, 0(a0)
    add   s6, s6, t4
    lw    t4, 4(a0)
    add   s6, s6, t4
    slli  t3, t3, 2
    vsetvli t0, t3, e16, m2, tu, mu
    vse16.v v16, (a0)
    ld    t4, 120(a0)
    add   s6, s6, t4
    vsetvli t0, t3, e32, m4, ta, ma
    vse32.v v24, (a0)
    lw    t4, 200(a0)
    add   s6, s6, t4
    vsetvli t0, x0, e64, m8, ta, ma
    vse64.v v8, (a0)
    ld    t4, 1016(a0)
    add   s6, s6, t4
    # 2) vse 后紧跟同段 vle；vse 后标量 store 改几个字节再读
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vx v2, v9, s4
    vse32.v v2, (a0)
    vle32.v v3, (a0)
    vmv.x.s t4, v3
    add   s6, s6, t4
    vse32.v v3, (a0)
    sb    s4, 3(a0)
    sh    s4, 126(a0)
    ld    t4, 0(a0)
    add   s6, s6, t4
    ld    t4, 120(a0)
    add   s6, s6, t4
    # 3) 整寄存器 / 掩码
    vs1r.v  v10, (a0)
    lw    t4, 64(a0)
    add   s6, s6, t4
    vs2r.v  v12, (a0)
    vs4r.v  v20, (a0)
    vs8r.v  v24, (a0)
    lw    t4, 900(a0)
    add   s6, s6, t4
    vsetvli t0, t3, e8, m1, ta, ma
    vmseq.vx v4, v8, s4
    vsm.v   v4, (a0)
    lw    t4, 0(a0)
    add   s6, s6, t4
    # 4) 误预测影子里的 vse、ecall
    vsetvli t0, x0, e32, m1, ta, ma
    andi  t2, s4, 3
    beqz  t2, 3f
    la    t5, dst+6144
    vse32.v v9, (t5)
3:
    vse32.v v11, (a0)
    ecall
    lw    t4, 8(a0)
    add   s6, s6, t4
    # 5) 跨页：窗口从页尾前 40 B 起
    la    t2, dst+4096-40
    vsetvli t0, x0, e32, m2, ta, ma
    vse32.v v12, (t2)
    lw    t4, 36(t2)
    add   s6, s6, t4
    lw    t4, 44(t2)
    add   s6, s6, t4
    # 6) 基址晚到 (除法) 的 vse，后面同地址的标量 load 先执行 -> 违例
    li    t2, 1
    li    t5, 7
    div   t2, t5, t2                   # t2 = 7 (慢)
    sub   t2, t2, t5                   # 0
    add   t2, t2, a0
    vsetvli t0, x0, e64, m1, ta, ma
    vse64.v v13, (t2)
    ld    t4, 16(a0)
    add   s6, s6, t4
    addi  s4, s4, -1
    bnez  s4, 2b

    # 7) 拷贝循环：src -> dst+4096+512 共 2048 B (每次 LMUL 4)
    la    a1, src
    la    a2, dst+4096+512
    li    a3, 2048
    vsetvli t0, x0, e8, m4, ta, ma
4:  vle8.v  v4, (a1)
    vse8.v  v4, (a2)
    add   a1, a1, t0
    add   a2, a2, t0
    sub   a3, a3, t0
    bgtz  a3, 4b

    # 整个 dst (8 KB) 标量读回累加
    la    a0, dst
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

trap:
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret

    .data
offs:  .word 0, 4, 60, 64, 120, 124, 8, 72
    .align 12
src:   .zero 8192
    .align 12
dst:   .zero 8192
    .zero 2048
res:   .zero 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
