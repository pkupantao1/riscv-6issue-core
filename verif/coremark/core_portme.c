/*
 * core_portme.c - CoreMark 的 OOO6 裸机移植 (见 core_portme.h)
 */
#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

static inline uint64_t rdcycle(void)
{
    uint64_t c;
    __asm__ volatile("rdcycle %0" : "=r"(c));
    return c;
}

#define EE_TICKS_PER_SEC 1000000u     /* 按 1 MHz 折算：Iterations/Sec = CoreMark/MHz */

static CORETIMETYPE start_time_val, stop_time_val;

void start_time(void) { start_time_val = rdcycle(); }
void stop_time(void)  { stop_time_val  = rdcycle(); }

CORE_TICKS get_time(void) { return (CORE_TICKS)(stop_time_val - start_time_val); }

secs_ret time_in_secs(CORE_TICKS ticks) { return (secs_ret)(ticks / EE_TICKS_PER_SEC); }

ee_u32 default_num_contexts = 1;

void portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;
    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
        ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
    if (sizeof(ee_u32) != 4)
        ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
    p->portable_id = 1;
}

void portable_fini(core_portable *p)
{
    /* 精确的 CoreMark/MHz (×1000，整数打印)：迭代数 × 1e6 / 周期数 */
    uint64_t t = get_time();
    if (t)
        ee_printf("OOO6 CoreMark/MHz x1000 : %lu (iterations=%d cycles=%lu)\n",
                  (unsigned long)((uint64_t)ITERATIONS * 1000000000ull / t), (int)ITERATIONS, (unsigned long)t);
    p->portable_id = 0;
}
