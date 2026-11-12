// OOO6 (core 模式 tb_top，UVM 构建) 的 riscv-arch-test 宏
//   结束：写 tohost (1 = 通过，3 = 失败；tb_top 监视 D$ store 口)；打印：16550 UART (0x1000_0000，LSR 恒为发送空)
//   CLINT：msip 0x0200_0000、mtimecmp 0x0200_4000、mtime 0x0200_BFF8 (mtime 每 8 拍加 1)
//   外部中断：TB 中断设备 0x0200_C000 (+define+O6_UVM)：写 bit0 / bit1 清 MEIP / SEIP，bit2 / bit3 置 MEIP / SEIP
#ifndef _RVMODEL_MACROS_H
#define _RVMODEL_MACROS_H

#define RVMODEL_DATA_SECTION \
        .pushsection .tohost,"aw",@progbits;                \
        .balign 8; .global tohost; tohost: .dword 0;         \
        .balign 8; .global fromhost; fromhost: .dword 0;     \
        .popsection;

#define STANDARD_SM_SUPPORTED

#define RVMODEL_HALT_PASS  \
  li x1, 1                ;\
  la t0, tohost           ;\
  write_tohost_pass:      ;\
    sw x1, 0(t0)          ;\
    sw x0, 4(t0)          ;\
  self_loop_pass:         ;\
    j self_loop_pass      ;\

#define RVMODEL_HALT_FAIL \
  li x1, 3                ;\
  la t0, tohost           ;\
  write_tohost_fail:      ;\
    sw x1, 0(t0)          ;\
    sw x0, 4(t0)          ;\
  self_loop_fail:         ;\
    j self_loop_fail      ;\

.EQU UART_BASE_ADDR, 0x10000000
.EQU UART_THR, (UART_BASE_ADDR + 0)
.EQU UART_LSR, (UART_BASE_ADDR + 5)

#define RVMODEL_IO_WRITE_STR(_R1, _R2, _R3, _STR_PTR)               \
1:                           ;                       \
  lbu _R1, 0(_STR_PTR)        ;                       \
  beqz _R1, 3f                ;                       \
2:                            ;                       \
  li _R2, UART_LSR            ;                       \
  4:                          ;                       \
    lbu _R3, 0(_R2)           ;                       \
    andi _R3, _R3, 0x20       ;                       \
    beqz _R3, 4b              ;                       \
    li _R2, UART_THR          ;                       \
    sb _R1, 0(_R2)            ;                       \
  addi _STR_PTR, _STR_PTR, 1  ;                       \
  j 1b                        ;                       \
3:

// 不存在的物理地址 (PMA 无此区域) -> 访问错误
#define RVMODEL_ACCESS_FAULT_ADDRESS 0x00000000

#define RVMODEL_INTERRUPT_LATENCY 10

// mtime 每 8 拍加 1：1000 个 mtime 周期 = 8000 拍，够从设定定时器走到低特权级
#define RVMODEL_TIMER_INT_SOON_DELAY 1000
#define RVMODEL_MTIME_ADDRESS    0x0200BFF8
#define RVMODEL_MTIMECMP_ADDRESS 0x02004000
#define RVMODEL_MSIP_ADDRESS     0x02000000

#define O6_IRQ_DEV 0x0200C000

#define RVMODEL_SET_MEXT_INT(_R1, _R2) \
  li _R1, O6_IRQ_DEV;                  \
  li _R2, 4;                           \
  sw _R2, 0(_R1);

#define RVMODEL_CLR_MEXT_INT(_R1, _R2) \
  li _R1, O6_IRQ_DEV;                  \
  li _R2, 1;                           \
  sw _R2, 0(_R1);                      \
  lw _R2, 0(_R1);

#define RVMODEL_SET_SEXT_INT(_R1, _R2) \
  li _R1, O6_IRQ_DEV;                  \
  li _R2, 8;                           \
  sw _R2, 0(_R1);

#define RVMODEL_CLR_SEXT_INT(_R1, _R2) \
  li _R1, O6_IRQ_DEV;                  \
  li _R2, 2;                           \
  sw _R2, 0(_R1);                      \
  lw _R2, 0(_R1);

// SSI / MSI 由测试环境经 mip.SSIP / msip 自己置清

#endif // _RVMODEL_MACROS_H
