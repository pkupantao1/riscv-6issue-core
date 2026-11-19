# v_vm.S - I2 (docs/11 §9e)：开 Sv39 时的向量 load 预查 (VPC 的翻译路径)。
#   0x8000_0000 起 1 GB 大页恒等映射 (D A X W R V)，S 态执行：
#   - 刚开翻译时 DTLB 缺失：预查不过 -> replay -> ROB 头同步执行 (页表遍历) -> 之后的预查命中、提前提交
#   - 跨 4 KB 页的 vle (同一大页，预查查两页)
#   - 访问没有映射的地址 (VPN2 = 1 无效)：预查不过 -> 原路径报 load page fault -> M 态处理：跳过、vstart 清零
#   - sfence.vma 之后再 load (DTLB 清空，再走一遍缺失)
#   结果在向量寄存器里与 Spike 整体比对。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, mtrap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)  # mstatus.VS / FS = Initial
    csrs  mstatus, t0
    # PMP：第 0 项 NAPOT 全地址 RWX (S 态访存要有匹配项)
    li    t0, -1
    csrw  pmpaddr0, t0
    li    t0, 0x1F
    csrw  pmpcfg0, t0
    # 页表：根页 pt，第 2 项映射 VA 0x8000_0000 -> PA 0x8000_0000 (1 GB)
    la    t0, pt
    li    t1, ((0x80000000 >> 12) << 10) | 0xCF
    sd    t1, 16(t0)
    srli  t1, t0, 12
    li    t2, 8
    slli  t2, t2, 60
    or    t1, t1, t2
    csrw  satp, t1
    sfence.vma
    # 进 S 态
    li    t0, 3 << 11
    csrc  mstatus, t0
    li    t0, 1 << 11
    csrs  mstatus, t0
    la    t0, s_main
    csrw  mepc, t0
    mret

s_main:
    # src (8 KB) 填充
    la    a0, src
    li    t1, 0
    li    t3, 8192
1:  add   t2, a0, t1
    slli  t4, t1, 2
    add   t4, t4, t1
    addi  t4, t4, 11
    sb    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b

    li    s4, 16                      # 轮数
    li    s5, 0
2:
    la    a0, src
    add   a0, a0, s5
    vsetvli t0, x0, e32, m1, ta, ma
    vle32.v v1, (a0)
    vadd.vv v2, v2, v1
    addi  t2, a0, 1024
    vle32.v v3, (t2)
    vadd.vv v2, v2, v3
    # 跨 4 KB 页 (src + 4096 - 64 起 512 B)
    la    t2, src+4096-64
    vsetvli t0, x0, e8, m4, ta, ma
    vle8.v  v8, (t2)
    vsetvli t0, x0, e32, m1, ta, ma
    vadd.vv v2, v2, v8
    # 没映射的地址 -> load page fault
    li    t2, 0x40000000
    vle32.v v4, (t2)
    # 每 4 轮 sfence 一次
    andi  t3, s4, 3
    bnez  t3, 3f
    sfence.vma
3:  vle32.v v5, (a0)
    vadd.vv v2, v2, v5
    addi  s5, s5, 320
    andi  s5, s5, 2047
    addi  s4, s4, -1
    bnez  s4, 2b

    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

# M 态陷入：S 态的 load page fault (未委托)。跳过那条指令、vstart 清零，返回 S 态 (MPP 已是 S)
mtrap:
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    csrw  vstart, zero
    mret

    .data
    .align 12
pt:    .zero 4096
src:   .zero 8192

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
