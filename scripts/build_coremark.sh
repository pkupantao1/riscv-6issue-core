#!/bin/bash
# 编 CoreMark (EEMBC，源码在 /export/master/src/sw/coremark) 给 OOO6 跑：
#   移植层 verif/coremark/core_portme.{c,h}；启动 / 链接 / printf 复用 riscv-tests 基准环境
#   (先跑过 scripts/build_bench.sh，它生成打过 UART 补丁的 syscalls.c)
#   ITER=迭代数 (默认 10；仿真里跑不到官方要求的 10 s，结果只作 CoreMark/MHz 估算)
#   MARCH=-march (默认 rv64gc_zba_zbb_zbs：核已实现 B 扩展；对比纯 rv64gc 时 MARCH=rv64gc NAME=coremark_gc)
#   NAME=输出文件名 (默认 coremark)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CM=/export/master/src/sw/coremark
RT=/export/master/src/riscv-tests/benchmarks
BIN=/export/master/rvtools/bin
OUT=$ROOT/work/tests/bench
ITER=${ITER:-10}
MARCH=${MARCH:-rv64gc_zba_zbb_zbs}
NAME=${NAME:-coremark}
[ -f $OUT/common/syscalls.c ] || bash $ROOT/scripts/build_bench.sh > /dev/null

# 编译参数：与常见 RISC-V CoreMark 报告同档 (-O3 + 循环展开 + 内联)。
# -fno-tree-loop-distribute-patterns 必须加：否则 syscalls.c 自带 memset 的清零循环被识别成 memset 调用，自己调自己死循环
CFLAGS="-O3 -funroll-all-loops -finline-limit=600 -falign-functions=16 -falign-jumps=4 -falign-loops=4 -fno-common -fno-tree-loop-distribute-patterns \
-march=${MARCH}_zicsr_zifencei -mabi=lp64d -mcmodel=medany -static -std=gnu99 -fno-builtin-printf -U_FORTIFY_SOURCE"
DEFS="-DITERATIONS=$ITER -DPERFORMANCE_RUN=1 -DPREALLOCATE=1"
FLAGS_STR="$CFLAGS"
LINK="-static -nostdlib -nostartfiles $(f=$($BIN/riscv64-unknown-elf-gcc -print-libgcc-file-name); [ -f "$f" ] && echo -lgcc) -T $RT/common/test.ld"

$BIN/riscv64-unknown-elf-gcc -I$ROOT/verif/coremark -I$CM -I$RT/../env -I$RT/common $CFLAGS $DEFS \
  -DFLAGS_STR="\"$FLAGS_STR\"" -o $OUT/$NAME.riscv \
  $CM/core_list_join.c $CM/core_main.c $CM/core_matrix.c $CM/core_state.c $CM/core_util.c \
  $ROOT/verif/coremark/core_portme.c $OUT/common/syscalls.c $RT/common/crt.S $LINK 2> $OUT/coremark.err \
  && echo "built $NAME.riscv (ITERATIONS=$ITER, -march=$MARCH)" || { echo "FAIL coremark: $(head -c 1500 $OUT/coremark.err)"; exit 1; }
