# st_ld_line.S - 同一行的 store 缺失与 load 缺失同拍分配 MSHR (#18 类；变异 M07)
#   对每个冷行：先 sd 写第 0 个双字，再 ld 读第 1 个双字 (同行不同字，无转发)；ld 的地址经 d 条相关的 addi 才算出，
#   d 从 0 扫到 47 —— store 提交后排空时缺失、load 推测执行时缺失，总有某个 d 让两者同拍分配，触发 #18 的路径
#   (修复：load 合并到 store 本拍新分配的 MSHR；o6_dcache 另有「同一行两个 MSHR」断言)
#   自检：最后读回每行写入的值
    .section .text.init
    .globl _start
_start:
    la    s0, lbuf
    li    s7, 0x5a5a
    .set  d, 0
    .rept 48
    sd    s7, 0(s0)
    mv    a1, s0
    .rept d
    addi  a1, a1, 0                # 推迟 load 的地址就绪 d 拍
    .endr
    ld    t0, 8(a1)
    addi  s0, s0, 64
    .set  d, d + 1
    .endr
    # 自检
    la    s0, lbuf
    li    s1, 48
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
    .zero 48 * 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
