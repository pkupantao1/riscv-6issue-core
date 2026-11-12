# viol_mshr.S - 访存违例重放与 MSHR 全忙同拍 (必达 bin cg_uarch.cx_viol_mshr)
#   每组：先让 s4 所在行进 D$；store 的地址经 div 拖慢；紧跟的同址 load 先执行 -> STA 出来时判违例，load 到 ROB 头重放冲刷；
#   这条 load 之后 (程序顺序更年轻) 的 32 条冷缺失 load 无依赖、早早发出，把 8 个 MSHR 持续占满 —— 冲刷那拍 MSHR 仍全忙
#   (在途回填不随冲刷撤销)。每组开头 fence：否则违例 load 要等上一组的冷缺失全部完成才到 ROB 头，那时 MSHR 已空。
#   16 组展开、各组 PC 不同，store-wait 预测表学会一条也压不住其余的。
#   自检：每组 store 写入的值被随后的 load (重放后) 读到，累加后与期望比较
    .section .text.init
    .globl _start
_start:
    la    s3, cold                 # 冷缺失区：每组 32 行 (2 KB)
    la    s4, hot                  # 违例地址：每组一个双字
    li    s5, 1000003
    li    s6, 7
    li    s8, 0                    # 累加 load 读到的值
    li    s9, 0                    # 期望值
    .set  g, 0
    .rept 16
    fence rw, rw                   # 上一组的缺失全部完成：本组违例 load 到 ROB 头时只需等 div，不必等上一组的冷缺失
    ld    t0, 8*g(s4)              # 让 hot 所在行进 D$ (之后的违例 load 命中，能先于 store 执行)
    li    s7, 100 + g
    add   s9, s9, s7
    div   t2, s5, s6               # 长延迟
    and   t2, t2, zero             # t2 = 0，但依赖 div
    add   t3, s4, t2
    sd    s7, 8*g(t3)              # STA 等 div
    ld    t4, 8*g(s4)              # 同址 load：先执行 -> 违例 -> 重放后读到 s7
    add   s8, s8, t4
    .set  k, 0
    .rept 32                       # 32 条冷缺失 (每条一个新行)
    ld    t5, k(s3)
    .set  k, k + 64
    .endr
    addi  s3, s3, 2047
    addi  s3, s3, 1
    .set  g, g + 1
    .endr
    bne   s8, s9, fail
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
hot:
    .zero 4096
cold:
    .zero 32768

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
