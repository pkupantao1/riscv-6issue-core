#!/bin/bash
# 编 riscv-tests/benchmarks (单核基准) 给 OOO6 跑：
#   - 编译参数照抄官方 Makefile
#   - syscalls.c 的 HTIF syscall 换成写 UART THR (0x1000_0000)：TB 核模式直接打印，FPGA 上同样可用；
#     exit 仍写 tohost
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RT=/export/master/src/riscv-tests/benchmarks
BIN=/export/master/rvtools/bin
OUT=$ROOT/work/tests/bench
mkdir -p $OUT/common
PY=/export/master/conda/rv/bin/python

# 打补丁的 syscalls.c
$PY - "$RT/common/syscalls.c" "$OUT/common/syscalls.c" <<'PYEOF'
import sys, re
s = open(sys.argv[1]).read()
i = s.index("static uintptr_t syscall(")
j = s.index("\n}\n", i) + 3
s = s[:i] + r'''static uintptr_t syscall(uintptr_t which, uint64_t arg0, uint64_t arg1, uint64_t arg2)
{
  /* OOO6：控制台输出走 UART 16550 THR */
  if (which == SYS_write) *(volatile uint8_t *)0x10000000 = *(const char *)arg1;
  return 0;
}
''' + s[j:]
open(sys.argv[2], "w").write(s)
PYEOF

OPTS="-U_FORTIFY_SOURCE -DPREALLOCATE=1 -mcmodel=medany -static -std=gnu99 -O2 -ffast-math -fno-common -fno-builtin-printf -fno-tree-loop-distribute-patterns -Wno-implicit-int -Wno-implicit-function-declaration -march=rv64gc_zicsr_zifencei -mabi=lp64d"
LINK="-static -nostdlib -nostartfiles $(f=$($BIN/riscv64-unknown-elf-gcc -print-libgcc-file-name); [ -f "$f" ] && echo -lgcc) -T $RT/common/test.ld"
n=0; f=0
# 自写基准放在 verif/bench/<名字>/ (同样用 riscv-tests 的 util.h / crt.S)：interp = 间接跳转 (解释器)
for b in ${BENCH:-median qsort rsort towers vvadd memcpy multiply mm dhrystone spmv interp}; do
  D=$RT/$b; [ -d $ROOT/verif/bench/$b ] && D=$ROOT/verif/bench/$b
  $BIN/riscv64-unknown-elf-gcc -I$RT/../env -I$RT/common -I$D $OPTS -o $OUT/$b.riscv \
    $D/*.c $(ls $D/*.S 2>/dev/null) $OUT/common/syscalls.c $RT/common/crt.S $LINK 2> $OUT/$b.err \
  && n=$((n+1)) || { f=$((f+1)); echo "FAIL $b: $(head -c 400 $OUT/$b.err)"; }
done
echo "built=$n failed=$f"
