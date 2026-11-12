# v_basic.S - 向量 V0 / V1a 定向测试：vsetvl 系列、向量 CSR、整数运算 / 比较 / merge / vmv
#   正确性由对拍判定 (逐条比标量结果；向量单元每次跑空时整体比向量寄存器与向量 CSR)
#   通过：tohost = 1
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, (1 << 9)              # mstatus.VS = Initial
    csrs  mstatus, t0

    # ---------------- vsetvl 系列与向量 CSR
    li    a0, 37
    vsetvli t1, a0, e8, m1, ta, ma        # vl = 37
    vsetvli t1, a0, e64, m8, tu, mu       # vl = 37
    vsetvli t1, zero, e32, m2, ta, ma     # rs1 = x0, rd != x0：vl = VLMAX = 64
    vsetvli zero, zero, e16, m1, ta, ma   # x0, x0 且 VLMAX 不变 (32 / 2 = 64 / 1)：保持 vl
    vsetvli zero, zero, e16, m2, ta, ma   # x0, x0 改了 VLMAX：vill
    csrr  t2, vtype
    csrr  t3, vl
    vsetivli t1, 17, e16, mf2, ta, ma     # vsetivli，小数 LMUL
    vsetivli t1, 31, e64, mf8, ta, ma     # SEW > LMUL * ELEN：vill
    li    a1, 0x18                        # vsetvl：e64 m1
    li    a0, 1000
    vsetvl t1, a0, a1                     # vl = min(1000, 16) = 16
    li    a1, 0x100                       # 保留位 (altfmt) 置位：vill
    vsetvl t1, a0, a1
    csrr  t2, vlenb
    li    t4, 3
    csrw  vxrm, t4
    csrwi vxsat, 1
    csrr  t5, vcsr
    csrw  vcsr, zero
    csrwi vstart, 5
    csrr  t6, vstart
    csrw  vstart, zero

    # ---------------- 整数运算：各 SEW / LMUL，vv / vx / vi，带掩码与不带掩码
    li    a0, 200
    vsetvli t1, a0, e8, m2, ta, ma        # vl = 200 (VLMAX 256)
    vmv.v.i v2, 5
    li    a2, 0x7f
    vmv.v.x v4, a2
    vadd.vv v6, v2, v4
    vsub.vx v8, v6, a2
    vrsub.vi v10, v8, -3
    vand.vv v12, v6, v10
    vor.vi  v14, v12, 9
    vxor.vx v16, v14, a0
    vsll.vi v18, v16, 3
    vsrl.vx v20, v18, a2
    vsra.vv v22, v18, v2
    vminu.vv v24, v6, v10
    vmax.vx v26, v10, a2
    vmseq.vi v1, v2, 5
    vmsltu.vx v3, v6, a2
    vmv.v.i v0, 0
    li    a3, 0x5555
    vsetivli zero, 4, e64, m1, ta, ma
    vmv.s.x v0, a3
    vsetvli t1, a0, e8, m2, tu, mu
    vadd.vi v6, v2, 1, v0.t               # 掩码
    vmerge.vxm v8, v4, a3, v0
    vmsle.vv v5, v6, v8, v0.t
    vmsgt.vx v9, v6, a2

    li    a0, 100
    vsetvli t1, a0, e16, m4, ta, ma       # vl = 100 (VLMAX 256)
    vmv.v.x v8, a0
    vadd.vx v12, v8, a2
    vsrl.vi v16, v12, 2
    vmslt.vx v1, v16, a2
    vmin.vv v20, v16, v8, v0.t
    vmv.x.s t2, v20

    li    a0, 13
    vsetvli t1, a0, e32, m1, tu, mu
    li    a4, -7
    vmv.v.x v9, a4
    vsra.vi v10, v9, 1
    vmsgtu.vi v11, v10, 4
    vmv.x.s t3, v10
    vsetvli t1, a0, e64, m1, tu, mu
    li    a5, 0x123456789abcdef0
    vmv.s.x v12, a5
    vadd.vx v13, v12, a5
    vmv.x.s t4, v13
    vsetivli zero, 0, e8, m1, ta, ma      # vl = 0：运算不写任何元素
    vadd.vi v13, v13, 1

    li    t0, 1
    la    t1, tohost
    sd    t0, 0(t1)
1:  j     1b

    .align 2
trap:
    li    t0, 3
    la    t1, tohost
    sd    t0, 0(t1)
2:  j     2b

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
