# zfhmin.S - 标量 Zfhmin / Zfbfmin (Zvfh / Zvfbfwma 的前提)：FLH / FSH、FMV.X.H / FMV.H.X、
#   FCVT.S.H / H.S / D.H / H.D、FCVT.S.BF16 / BF16.S，各舍入模式；结果与 fflags 由逐条对拍 (cosim) 检查。
#   输入表里有正确装箱的半精度 / 单精度、没装箱的值 (应按默认 NaN 处理)、NaN / 无穷 / 次正规 / 舍入边界。
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    t0, 1 << 13                   # FS = Initial
    csrs  mstatus, t0
    la    s0, vals
    la    s1, vals_end
    la    s2, buf
    li    s3, 0                         # 动态舍入模式 (每个值换一种)
1:
    ld    t0, 0(s0)
    fmv.d.x f1, t0
    fsrm  s3
    # 升格式 (精确，只有 sNaN 报 NV)
    fcvt.s.h    f2, f1
    fcvt.d.h    f3, f1
    fcvt.s.bf16 f4, f1
    # 降格式 (各舍入模式)
    fcvt.h.s    f5, f1
    fcvt.h.s    f5, f1, rne
    fcvt.h.s    f5, f1, rtz
    fcvt.h.s    f5, f1, rdn
    fcvt.h.s    f5, f1, rup
    fcvt.h.s    f5, f1, rmm
    fcvt.h.d    f6, f1
    fcvt.h.d    f6, f1, rtz
    fcvt.h.d    f6, f1, rup
    fcvt.bf16.s f7, f1
    fcvt.bf16.s f7, f1, rdn
    fcvt.bf16.s f7, f1, rmm
    # 往返：降了再升
    fcvt.s.h    f8, f5
    fcvt.d.h    f9, f6
    fcvt.s.bf16 f10, f7
    # 搬移 / 访存
    fmv.x.h  a0, f1
    fmv.h.x  f11, t0
    fmv.x.d  a1, f11
    fsh      f1, 0(s2)
    flh      f12, 0(s2)
    fmv.x.d  a2, f12
    lhu      a3, 0(s2)
    fsh      f5, 2(s2)
    flh      f13, 2(s2)
    fmv.x.d  a4, f13
    frflags  a5
    fsflags  zero
    # 下一个
    addi  s3, s3, 1
    li    t1, 5
    bltu  s3, t1, 2f
    li    s3, 0
2:
    addi  s0, s0, 8
    bltu  s0, s1, 1b
    # 结束
    li    t1, 1
    la    t2, tohost
    sd    t1, 0(t2)
3:  j     3b

    .align 2
trap:                                   # 不应有任何陷入
    csrr  t1, mcause
    slli  t1, t1, 1
    ori   t1, t1, 3
    la    t2, tohost
    sd    t1, 0(t2)
4:  j     4b

    .data
    .align 3
vals:
    # 装箱的半精度
    .dword 0xFFFFFFFFFFFF3C00           # 1.0
    .dword 0xFFFFFFFFFFFFBC00           # -1.0
    .dword 0xFFFFFFFFFFFF7BFF           # 最大有限值
    .dword 0xFFFFFFFFFFFF0001           # 最小次正规
    .dword 0xFFFFFFFFFFFF03FF           # 最大次正规
    .dword 0xFFFFFFFFFFFF8000           # -0
    .dword 0xFFFFFFFFFFFF7C00           # +inf
    .dword 0xFFFFFFFFFFFFFC00           # -inf
    .dword 0xFFFFFFFFFFFF7E00           # qNaN
    .dword 0xFFFFFFFFFFFF7D01           # sNaN
    .dword 0xFFFFFFFFFFFF3555           # 1/3 附近
    # 装箱的单精度
    .dword 0xFFFFFFFF3F800000           # 1.0
    .dword 0xFFFFFFFF477FF000           # 65520 (半精度舍入到 inf 的边界)
    .dword 0xFFFFFFFF477FE000           # 65504
    .dword 0xFFFFFFFF33800000           # 2^-24 (半精度最小次正规)
    .dword 0xFFFFFFFF33000000           # 2^-25 (舍入到 0 / 次正规的边界)
    .dword 0xFFFFFFFF3F808000           # BF16 恰在中点 (偶数)
    .dword 0xFFFFFFFF3F818000           # BF16 恰在中点 (奇数)
    .dword 0xFFFFFFFF3F80C001           # BF16 中点以上
    .dword 0xFFFFFFFF7F7FFFFF           # 单精度最大 (BF16 舍入到 inf)
    .dword 0xFFFFFFFF00000001           # 单精度最小次正规
    .dword 0xFFFFFFFF7FA00000           # sNaN
    .dword 0xFFFFFFFFC0490FDB           # -pi
    # 双精度
    .dword 0x3FF0000000000000           # 1.0
    .dword 0x40EFFE0000000000           # 65520
    .dword 0x3E70000000000000           # 2^-24
    .dword 0x7FE0000000000000           # 很大
    .dword 0x0000000000000001           # 双精度最小次正规
    .dword 0x7FF4000000000000           # sNaN
    .dword 0xBFB999999999999A           # -0.1
    # 没装箱 (半精度 / 单精度视角下应当是默认 NaN)
    .dword 0x123456789ABC3C00
    .dword 0x00000000FFFF3C00
vals_end:
    .align 3
buf:
    .dword 0

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
