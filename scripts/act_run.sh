#!/bin/bash
# 跑 ACT4 生成的 ELF：act_run.sh <SIMTAG> <grep 正则> <结果标签>
cd /export/master/riscv_ooo6_dev24
E=/export/master/riscv_ooo6_dev24/work/act/o6-rv64gc/elfs
find $E -name "*.elf" | grep -iE "$2" | sort > work/act_$3.list
echo "ACT ELF: $(wc -l < work/act_$3.list)"
export TAGX=$1
run1() { r=$(SIMTAG=$TAGX TIMEOUT=900 bash scripts/run_vcs.sh run $1 | grep -E "=== (PASS|FAIL)" | head -1); echo "$(basename $1) ${r:-无结果}"; }
export -f run1
xargs -P 6 -n 1 bash -c 'run1 "$0"' < work/act_$3.list | sort > work/act_$3.res
echo "ACT $3: $(grep -c '=== PASS' work/act_$3.res) / $(wc -l < work/act_$3.res)"
grep -v "=== PASS" work/act_$3.res | head -20
echo ACTRUN_DONE
