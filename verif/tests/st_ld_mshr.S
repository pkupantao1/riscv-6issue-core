# st_ld_mshr.S - store 排空缺失与同一行 load 缺失同拍分配 MSHR (bug #18，变异 M07)
#   st_ld_line.S 碰不到：STA 地址一定就发 store 预取，排空时行已在 / MSHR 已有，排空不再缺失。
#   这里每组先发 13 个冷行 load 占住 13 / 15 个 MSHR (预取要求分配后还剩 >= 2 个空的，于是被丢掉)，
#   再 sd 写冷行 L 的第 0 个双字 (预取被丢 -> 提交后排空时缺失)，再 ld 读 L 的第 1 个双字 (无转发)：
#   ld 的地址取决于最后一个冷 load 的值 (= 0) 再经 d 条 addi，d 从 0 扫到 40 —— 冷 load 回来后 sd 才能提交、排空，
#   总有某个 d 让 ld 的缺失与 sd 的排空缺失同拍分配
#   自检：最后读回每个 L 写入的值
    .section .text.init
    .globl _start
_start:
    la    s0, lbuf                 # 各组的 L
    la    s1, cbuf                 # 冷 load 区
    li    s7, 0x5a5a
    .set  d, 0
    .rept 41
    ld    t0, 0(s1)
    ld    t1, 64(s1)
    ld    t2, 128(s1)
    ld    t3, 192(s1)
    ld    t4, 256(s1)
    ld    t5, 320(s1)
    ld    t6, 384(s1)
    ld    a2, 448(s1)
    ld    a3, 512(s1)
    ld    a4, 576(s1)
    ld    a5, 640(s1)
    ld    a6, 704(s1)
    ld    a7, 768(s1)
    sd    s7, 0(s0)
    andi  a7, a7, 0
    add   a1, s0, a7
    .rept d
    addi  a1, a1, 0
    .endr
    ld    t0, 8(a1)
    addi  s0, s0, 64
    addi  s1, s1, 13 * 64
    .set  d, d + 1
    .endr
    # 自检
    la    s0, lbuf
    li    s1, 41
1:  ld    t1, 0(s0)
    bne   t1, s7, fail
    addi  s0, s0, 64
    addi  s1, s1, -1
    bnez  s1, 1b
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
2:  j     2b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
3:  j     3b

    .data
    .align 12
lbuf:
    .zero 41 * 64
    .align 12
cbuf:
    .zero 41 * 13 * 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
