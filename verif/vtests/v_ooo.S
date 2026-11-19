# v_ooo.S - O1 (docs/11 §9f)：核内乱序向量 uop (单宽度整数逐元素 .vv / .vi，LMUL <= 1)。
#   每轮覆盖：
#   1) vl / vtype 每轮变 (vsetvli 改了 vl 要冲刷带旧 vl 的乱序 uop)；tu / mu (尾部、被掩掉的元素保持 vd 旧值)
#   2) 乱序 uop 之间的相关链 (VX 写回 -> 清忙 -> VIQ 选消费者)、.vi、移位、vmerge、vmv.v.v / vmv.v.i、带掩码
#   3) 误预测分支影子里的乱序 uop (作废：不写回、不提交、目的物理寄存器回收)
#   4) 读 / 写在途 load 目的的候选 (有在途写 -> 降级为提交后按序执行)；load 做完很久以后再读 (按 crat 读)
#   5) 归约 (不在核内改名的写) 后紧跟的候选；vmv.x.s 把向量结果读成标量 (每条提交对拍)
#   6) e8 mf2 / e64 的乱序 uop；vse + 标量 load 读回
#   7) 乱序 uop 在途时 ecall (陷入冲刷)；写 v0 的乱序 uop (之后的带掩码 uop 读新的 v0)
#   8) 64 条互不相关的 vadd (VIQ / VDQ 占满)
#   结果在向量寄存器里与 Spike 整体比对，标量累加 s6 每条提交对拍。通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9) | (1 << 13)  # mstatus.VS / FS = Initial
    csrs  mstatus, t0
    # src (4 KB) 填充：字 i = i * 0x9E3779B1 + 5
    la    a0, src
    li    t1, 0
    li    t3, 1024
    li    t5, 0x9E3779B1
1:  slli  t2, t1, 2
    add   t2, a0, t2
    mul   t4, t1, t5
    addi  t4, t4, 5
    sw    t4, 0(t2)
    addi  t1, t1, 1
    blt   t1, t3, 1b
    # 初值：v0..v31 从 src 读 (e32 m8 四组)
    vsetvli t0, x0, e32, m8, ta, ma
    vle32.v v0, (a0)
    addi  t2, a0, 1024
    vle32.v v8, (t2)
    addi  t2, t2, 1024
    vle32.v v16, (t2)
    addi  t2, t2, 1024
    vle32.v v24, (t2)
    li    s6, 0
    li    s4, 40                      # 轮数
2:
    la    a0, src
    andi  t1, s4, 7
    slli  t1, t1, 7
    add   a0, a0, t1                  # a0：本轮的源 (随轮移动)
    addi  a1, a0, 512
    la    a2, dst
    # 1) vl 每轮变，tu / mu
    andi  t1, s4, 15
    addi  t1, t1, 3                   # vl = 3..18
    vsetvli t0, t1, e32, m1, tu, mu
    # 2) 相关链
    vadd.vv  v1, v2, v3
    vsub.vv  v4, v1, v2
    vxor.vv  v5, v4, v1
    vsll.vi  v6, v5, 3
    vsra.vi  v7, v6, 1
    vmerge.vvm v8, v7, v5, v0
    vmv.v.v  v9, v8
    vmv.v.i  v10, -5
    vadd.vv  v11, v10, v9, v0.t
    vmin.vv  v12, v11, v6
    vmaxu.vv v13, v12, v1
    vand.vi  v14, v13, 15
    vor.vv   v15, v14, v4
    vrsub.vi v16, v15, 7
    vsrl.vi  v17, v16, 2, v0.t
    # 3) 误预测：约四分之一的轮跳过 (分支预测器会猜错一些)
    andi  t2, s4, 3
    beqz  t2, 3f
    vadd.vv  v1, v1, v16
    vsub.vv  v2, v2, v15
    vxor.vv  v3, v3, v17
3:
    # 4) 与 load 的相关
    vle32.v  v3, (a0)
    vadd.vv  v18, v3, v1              # 读在途 load 的目的 -> 降级
    vadd.vv  v19, v18, v2             # 读降级者的目的 -> 降级
    vle32.v  v1, (a1)                 # 写 v1 (更老的乱序 uop 读过旧的 v1)
    vadd.vv  v20, v1, v19
    # 等 load 做完 (标量延时)，再读 v3 (按 crat 读)
    li    t2, 60
4:  addi  t2, t2, -1
    bnez  t2, 4b
    vadd.vv  v21, v3, v4
    vsub.vv  v22, v21, v3
    # 5) 归约之后紧跟的候选、vmv.x.s
    vredsum.vs v23, v22, v2
    vadd.vv  v24, v23, v22
    vmv.x.s  t3, v24
    add   s6, s6, t3
    vmv.x.s  t3, v17
    add   s6, s6, t3
    # 6) e8 mf2 / e64
    vsetvli t0, x0, e8, mf2, tu, ma
    vadd.vi  v25, v25, 1
    vadd.vv  v26, v25, v26
    vsetvli t0, x0, e64, m1, ta, ma
    vsrl.vi  v27, v26, 2
    vadd.vv  v28, v27, v28
    vsetvli t0, x0, e32, m1, ta, ma
    vse32.v  v16, (a2)
    lw    t4, 4(a2)
    add   s6, s6, t4
    vmv.x.s  t3, v28
    add   s6, s6, t3
    # 7) ecall 时有乱序 uop 在途；写 v0 后带掩码的读新 v0
    vadd.vv  v29, v29, v16
    vxor.vv  v30, v29, v15
    ecall
    vadd.vv  v31, v30, v31
    vadd.vi  v0, v0, 1
    vsub.vv  v5, v5, v6, v0.t
    addi  s4, s4, -1
    bnez  s4, 2b

    # 8) 64 条互不相关的 vadd，循环 4 次
    vsetvli t0, x0, e32, m1, ta, ma
    li    s4, 4
5:
    .irp i, 0,1,2,3,4,5,6,7
    vadd.vv v1, v9, v17
    vadd.vv v2, v10, v18
    vadd.vv v3, v11, v19
    vadd.vv v4, v12, v20
    vadd.vv v5, v13, v21
    vadd.vv v6, v14, v22
    vadd.vv v7, v15, v23
    vadd.vv v8, v16, v24
    .endr
    vadd.vv v9, v1, v8
    addi  s4, s4, -1
    bnez  s4, 5b

    vmv.x.s  t3, v9
    add   s6, s6, t3
    la    t1, dst
    sd    s6, 64(t1)
    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
9:  j     9b

# 陷入：ecall。跳过那条指令
trap:
    csrr  t6, mepc
    addi  t6, t6, 4
    csrw  mepc, t6
    mret

    .data
    .align 12
src:   .zero 4096
    .zero 1024
dst:   .zero 512

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
