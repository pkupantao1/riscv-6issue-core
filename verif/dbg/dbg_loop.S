# dbg_loop.S - 调试模块自检用的被测程序：a0 不停加 1 并写到 counter (不会结束，由 TB 判定)
    .section .text.init
    .globl _start
_start:
    la    t0, trap
    csrw  mtvec, t0
    li    a0, 0
    la    t1, counter
loop:
    addi  a0, a0, 1
    sd    a0, 0(t1)
    j     loop
trap:
    j     trap

    .data
    .align 3
counter: .dword 0

    .section .tohost, "aw", @progbits
    .align 6
    .globl tohost
tohost: .dword 0
    .align 6
    .globl fromhost
fromhost: .dword 0
