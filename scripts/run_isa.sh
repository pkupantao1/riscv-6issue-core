#!/bin/bash
# 跑一组 ISA 测试：scripts/run_isa.sh <模式 glob，如 'rv64ui-p-*'> [plusargs]
# 输出汇总到 work/vcs/isa_<tag>.txt
ROOT=$(cd "$(dirname "$0")/.." && pwd)
pat=$1; shift
tag=$(echo "$pat" | tr -c 'a-zA-Z0-9_' '_')
sum=$ROOT/work/vcs/isa_${tag}${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}.txt
: > $sum
for elf in $ROOT/work/tests/isa/$pat; do
  [ -f "$elf" ] || continue
  r=$(bash $ROOT/scripts/run_vcs.sh run $elf "$@" | grep -E "=== (PASS|FAIL)" | head -1)
  printf "%-28s %s\n" "$(basename $elf)" "$r" >> $sum
done
echo "PASS $(grep -c '=== PASS' $sum) / $(wc -l < $sum)"
grep -v "=== PASS" $sum | head -40
