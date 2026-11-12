# 调试 ROM (o6_dm 里 0x800 起)：执行式调试的停机循环
# 汇编：riscv64-unknown-elf-as -march=rv64gc_zicsr_zifencei debug_rom.S -o debug_rom.o
#       riscv64-unknown-elf-objcopy -O binary debug_rom.o debug_rom.bin  (.text 从 0x800 偏移算起)
    .option norvc
    .text
    .org 0
    j     _entry            # 0x800：进入调试模式
    nop
    j     _exception        # 0x808：调试模式下发生异常
_entry:
    fence
    csrw  dscratch0, s0
loop:
    csrr  s0, mhartid
    sw    s0, 0x100(zero)   # HALTED
    lbu   s0, 0x400(zero)   # FLAGS
    andi  s0, s0, 1
    bnez  s0, going
    lbu   s0, 0x400(zero)
    andi  s0, s0, 2
    bnez  s0, resume
    j     loop
    nop
_exception:
    sw    zero, 0x10C(zero) # EXCEPTION
    ebreak
going:
    csrr  s0, dscratch0
    sw    zero, 0x104(zero) # GOING
    jalr  zero, 0x300(zero) # WHERETO
    nop
resume:
    csrr  s0, mhartid
    sw    s0, 0x108(zero)   # RESUMING
    csrr  s0, dscratch0
    dret
