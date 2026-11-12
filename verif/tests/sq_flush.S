# sq_flush.S - SQ 被已提交未排空的 store 正好占满 (#26 类；必达 bin cp_SQ_FULL / cp_SQFULL_FLUSH，变异 M02)
#   store 缺失逐条串行 (每条要等整行回填)，连续 80 个写不同冷行的 store 提交后在 SQ 里排队，SQ (64 项) 正好被占满。
#   #26 (sq_older 在「标志不同、下标相同」时判错) 的两个后果都在这里压：
#     1) SQ 满时进来的 load：转发搜索漏掉最老的 store —— 紧跟 64 条读回后 64 个 store 地址的 load (都在 SQ 满时分派)，
#        读到最老那条的 load 会拿到内存旧值；对拍 / 自检查出
#     2) SQ 满时冲刷 (csrw mscratch 是 flush_after)：作废判断把最老的已提交 store 当成未提交
#   每轮写入值 = 轮号 + 100 (与内存初值 0 不同)；读回值累加后与期望比较，最后再逐行读回
    .section .text.init
    .globl _start
_start:
    la    s0, sbuf
    li    s1, 100                  # 本轮写入值
    li    s2, 106
    li    s8, 0                    # 读回累加
    li    s9, 0                    # 期望
1:  mv    s3, s0
    .rept 80
    sd    s1, 0(s3)                # 每条一个新行 (64 B 步长)
    addi  s3, s3, 64
    .endr
    addi  a2, s0, 1024             # 第 16 个 store 的地址 (16 x 64 B)
    .rept 64                       # 读回后 64 个 store 的地址 (此时 SQ 满)
    ld    t1, 0(a2)
    add   s8, s8, t1
    addi  a2, a2, 64
    .endr
    li    t0, 64
    mul   t0, t0, s1
    add   s9, s9, t0
    csrw  mscratch, s1             # flush_after：SQ 里还排着几十条已提交 store
    li    t0, 80 * 64
    add   s0, s0, t0
    addi  s1, s1, 1
    bne   s1, s2, 1b
    bne   s8, s9, fail
    # 逐行读回：第 r 轮写的 80 行都等于 r + 100
    la    s0, sbuf
    li    s1, 100
2:  mv    s3, s0
    li    s4, 80
3:  ld    t1, 0(s3)
    bne   t1, s1, fail
    addi  s3, s3, 64
    addi  s4, s4, -1
    bnez  s4, 3b
    li    t0, 80 * 64
    add   s0, s0, t0
    addi  s1, s1, 1
    bne   s1, s2, 2b
    la    t5, tohost
    li    t6, 1
    sd    t6, 0(t5)
4:  j     4b
fail:
    la    t5, tohost
    li    t6, 3
    sd    t6, 0(t5)
5:  j     5b

    .data
    .align 12
sbuf:
    .zero 6 * 80 * 64

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
