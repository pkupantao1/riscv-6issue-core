# v_vel.S - I2 (docs/11 §9e)：预查后提前提交的向量 load (vel)、提交后异步访存、VECU 内改名。
#   每轮依次覆盖：
#   1) WAR：更老的长延迟运算 (vdivu) 读 v0，紧跟的 vle 改写 v0 (改名后不等它读完)，再下一条读新的 v0
#   2) 访存序：异步 vle 之后紧跟写同一行的标量 store (store 要等 vle 读完才排空)，再 vle 读到新值
#   3) vstart != 0 的 vle (改名切换时 vstart 之前的元素从旧的物理寄存器补回)
#   4) 尾部不变 (vl < VLMAX、tu)：同上，补回尾部
#   5) 带掩码的 vle (不改名的异步路径)
#   6) 整寄存器 / 掩码 / 分段 / fault-only-first 的 load；跨 4 KB 页的 vle (预查两页)
#   7) 异步 load 在途时 ecall (陷入要等它做完)
#   8) 基址不对齐的 vle (预查不过 -> replay -> ROB 头同步执行 -> 报地址不对齐)
#   陷入处理：mepc += 4、vstart 清零后返回。结果都在向量寄存器里，与 Spike 整体比对。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)  # mstatus.VS / FS = Initial
    csrs  mstatus, t0
    # 禁区页 vbad：PMP 第 0 项锁定、无权限 (M 态访问也报访问错误)
    la    t0, vbad
    srli  t0, t0, 2
    ori   t0, t0, 0x1FF
    csrw  pmpaddr0, t0
    li    t0, 0x98                    # L | NAPOT | ---
    csrw  pmpcfg0, t0
    # src (8 KB) 填充：字节 i = i * 7 + 3
    la    a0, src
    li    t1, 0
    li    t3, 8192
1:  add   t2, a0, t1
    slli  t4, t1, 3
    sub   t4, t4, t1
    addi  t4, t4, 3
    sb    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b

    li    s4, 24                      # 轮数
    li    s5, 0                       # 本轮偏移 (0, 192, 384, ... 模 2048)
2:
    la    a0, src
    add   a0, a0, s5                  # a0：本轮的源
    la    a2, mut                     # a2：会被标量 store 改写的一行
    la    a3, src+4096-64             # a3：跨 4 KB 页 (第 0 行在页尾)
    la    a4, dst

    # 1) WAR + 改名
    vsetvli t0, x0, e32, m1, ta, ma   # vl = 32
    vmv.v.i v1, 3
    vle32.v v0, (a0)
    addi  t2, a0, 128
    vdivu.vv v4, v0, v1               # 长延迟，读旧的 v0
    vle32.v v0, (t2)                  # 改名：不等 vdivu 读完
    vadd.vv v5, v0, v4                # 读新的 v0
    # 2) 访存序
    vle32.v v6, (a2)
    sw    s4, 0(a2)
    sd    s5, 64(a2)
    vle32.v v7, (a2)
    # 3) vstart != 0
    csrwi vstart, 3
    vle32.v v9, (a3)
    # 4) 尾部不变
    li    t1, 5
    vsetvli t0, t1, e32, m1, tu, mu
    vle32.v v10, (a0)
    # 5) 带掩码 (v0 = 刚 load 的新值的位)
    vsetvli t0, x0, e32, m1, ta, mu
    vle32.v v11, (a3), v0.t
    # 6) 整寄存器 / 掩码 / 分段 / fault-only-first / 跨页
    vl2re32.v v12, (a0)
    vlm.v   v14, (a2)
    vlseg2e32.v v16, (a3)
    vle32ff.v v18, (a0)
    vsetvli t0, x0, e8, m4, ta, ma    # 512 B，从页尾 64 B 处起跨页
    vle8.v  v20, (a3)
    vsetvli t0, x0, e32, m1, ta, ma
    # 7) 异步 load 在途时 ecall
    vle32.v v19, (a0)
    ecall
    vadd.vv v19, v19, v4
    # 8) 不对齐 / 禁区页 (预查不过，原路径报异常)；禁区页前 64 B 起的 vle (预查范围碰到禁区，实际访问不碰)
    addi  t2, a0, 2
    vle32.v v24, (t2)
    la    t2, vbad
    vle32.v v25, (t2)
    addi  t2, t2, -128
    vle32.v v26, (t2)
    # 结果写回 dst (下轮的 vle 会再读到)
    vse32.v v5, (a4)
    addi  t2, a4, 128
    vse32.v v19, (t2)

    addi  s5, s5, 192
    andi  s5, s5, 2047
    addi  s4, s4, -1
    bnez  s4, 2b

    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

# 陷入：ecall / 不对齐的 vle。跳过那条指令，vstart 清零
trap:
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    csrw  vstart, zero
    mret

    .data
    .align 12
src:   .zero 8192
mut:   .zero 128
dst:   .zero 512
    .align 12
vbad:  .space 4096

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
