/*
 * core_portme.h - CoreMark 的 OOO6 裸机移植
 *   启动 / 链接用 riscv-tests 基准的 crt.S + test.ld；输出用打过补丁的 syscalls.c 的 printf (写 UART THR)
 *   计时用 rdcycle：EE_TICKS_PER_SEC 取 1e6，即"1 MHz 时钟下的秒数"，CoreMark 打印的
 *   Iterations/Sec 直接就是 CoreMark/MHz (整数部分；精确值由 Iterations 与 Total ticks 算)
 */
#ifndef CORE_PORTME_H
#define CORE_PORTME_H

#include <stddef.h>
#include <stdint.h>

#define HAS_FLOAT   0          /* riscv-tests 的 printf 不支持 %f */
#define HAS_TIME_H  0
#define USE_CLOCK   0
#define HAS_STDIO   0
#define HAS_PRINTF  1

#ifndef COMPILER_VERSION
#define COMPILER_VERSION "GCC" __VERSION__
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS FLAGS_STR
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION "STATIC"
#endif

typedef signed short   ee_s16;
typedef unsigned short ee_u16;
typedef signed int     ee_s32;
typedef double         ee_f32;
typedef unsigned char  ee_u8;
typedef unsigned int   ee_u32;
typedef uintptr_t      ee_ptr_int;      /* RV64：指针 64 位 */
typedef size_t         ee_size_t;

#define align_mem(x) (void *)(4 + (((ee_ptr_int)(x)-1) & ~3))

#define CORETIMETYPE uint64_t
typedef uint64_t CORE_TICKS;

#define SEED_METHOD   SEED_VOLATILE
#define MEM_METHOD    MEM_STATIC
#define MULTITHREAD   1
#define USE_PTHREAD   0
#define USE_FORK      0
#define USE_SOCKET    0
#define MAIN_HAS_NOARGC   1
#define MAIN_HAS_NORETURN 0

extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S {
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) && !defined(VALIDATION_RUN)
#if (TOTAL_DATA_SIZE == 1200)
#define PROFILE_RUN 1
#elif (TOTAL_DATA_SIZE == 2000)
#define PERFORMANCE_RUN 1
#else
#define VALIDATION_RUN 1
#endif
#endif

int printf(const char *fmt, ...);
#define ee_printf printf

#endif /* CORE_PORTME_H */
