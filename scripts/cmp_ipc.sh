#!/bin/bash
# 对比两个 SIMTAG 的基准 IPC 与前端统计：scripts/cmp_ipc.sh <标签A> <标签B> [基准...]
#   默认基准：dhrystone qsort median towers coremark；结果在 work/cmp_<A>_<B>.txt
ROOT=$(cd "$(dirname "$0")/.." && pwd)
A=$1; B=$2; shift 2
BENCH=${*:-dhrystone qsort median towers coremark}
OUT=$ROOT/work/cmp_${A}_${B}.txt
: > $OUT
for tg in $A $B; do
  for b in $BENCH; do
    extra=""; [ $b = coremark ] && extra="+nocosim +max_cycles=10000000"
    SIMTAG=$tg TIMEOUT=7200 bash $ROOT/scripts/run_vcs.sh run $ROOT/work/tests/bench/$b.riscv $extra > /dev/null
  done &
done
wait
for b in $BENCH; do
  for tg in $A $B; do
    l=$ROOT/work/vcs/runs_$tg/$b.riscv/sim.log
    c=$(grep -a '^mcycle' $l | awk '{print $3}'); i=$(grep -a '^minstret' $l | awk '{print $3}')
    ipc=$([ -n "$c" ] && awk -v i=$i -v c=$c 'BEGIN{printf "%.3f", i/c}')
    res=$(grep -a -m1 -oE '=== (PASS|FAIL) ===' $l)
    fe=$(grep -aE 'F3 覆盖|F1 BTB 改道|误预测' $l | tr -s ' ' | tr '\n' ';' | cut -c1-160)
    printf "%-10s %-4s IPC=%-6s %s %s\n" $b $tg "$ipc" "$res" "$fe" >> $OUT
  done
done
echo CMP_DONE >> $OUT
cat $OUT
