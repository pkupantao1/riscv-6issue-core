# order_fence.S - 「store 刚提交、紧跟的指令就依赖它已排空」的顺序测试 (#27 同类窗口，程序自检)
#   1) 自修改代码：先发 8 个缺失的 store 把 SQ 堵住，再改写目标指令 (addi a0, x0, k)，紧跟 fence.i 后执行它，a0 必须 = k
#   2) MMIO 写后读：同样先堵 SQ，再写 CLINT mtimecmp、紧跟着读回，值必须一致 (读不能越过还在 SQ 里的写)
#   3) SQ 为空时的 MMIO 写后读：#27 的窗口只在 store 提交前 SQ 已排空时出现 (SQ 里还有更老的已提交 store 时
#      sq_head != sq_cmt，排空判断本来就是假)；先做一次 MMIO 读等 SQ 排空，再 sd / ld 背靠背
#   变异 M01 (撤销 #27 修复) 上：1) 2) 通过、3) 失败 —— 堵住 SQ 反而把窗口藏起来了
#   Spike 的 MMIO 读值由 DUT 注入，对拍查不出 2)；这里由程序自己判
    .equ  MTIMECMP, 0x02004000
    .section .text.init
    .globl _start
_start:
    li    s0, 1                    # k
    li    s1, 65
    li    s2, 0                    # clog 调用计数
loop:
    call  clog                     # 8 个 store 缺失排在前面
    # 1) 改写 target 处的 addi a0, x0, k  (编码 = k << 20 | 0x513)
    slli  t0, s0, 20
    li    t1, 0x513
    or    t0, t0, t1
    la    t1, target
    sw    t0, 0(t1)
    fence.i
    li    a0, -1
    jal   ra, target
    bne   a0, s0, fail
    # 2) MMIO 写后立即读回
    call  clog
    li    t0, MTIMECMP
    li    t1, 0x7000000000000000
    add   t1, t1, s0
    sd    t1, 0(t0)
    ld    t2, 0(t0)
    bne   t1, t2, fail
    # 3) SQ 为空时的 MMIO 写后读
    li    t0, MTIMECMP
    ld    t2, 0(t0)                # MMIO 读在 ROB 头等 SQ 排空
    li    t1, 0x6000000000000000
    add   t1, t1, s0
    sd    t1, 0(t0)
    ld    t2, 0(t0)
    bne   t1, t2, fail
    addi  s0, s0, 1
    bne   s0, s1, loop
    li    t0, MTIMECMP             # 恢复成最大值 (定时器中断不会被误触发)
    li    t1, -1
    sd    t1, 0(t0)
    j     pass

# 向 8 个不同的 64 B 行各写一次：每次调用换一组 (s2 计数，256 组 x 512 B = 128 KB > D$ 64 KB，128 次调用全是冷缺失)
clog:
    la    t3, cbuf
    andi  t4, s2, 255
    addi  s2, s2, 1
    slli  t4, t4, 9                # 每组 8 行 = 512 B
    add   t3, t3, t4
    sd    s0, 0(t3)
    sd    s0, 64(t3)
    sd    s0, 128(t3)
    sd    s0, 192(t3)
    sd    s0, 256(t3)
    sd    s0, 320(t3)
    sd    s0, 384(t3)
    sd    s0, 448(t3)
    ret

    .align 3
    .option push
    .option norvc
target:
    addi  a0, x0, 0
    ret
    .option pop

pass:
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
1:  j     1b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
2:  j     2b

    .data
    .align 12
cbuf:
    .zero 131072

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
