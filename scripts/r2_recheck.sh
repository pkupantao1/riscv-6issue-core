#!/bin/bash
# R2 失败用例修复后的复查：r2fp20331 (FMV / I2F 队列里不在队头的项错过分支杀，之后照常出队，写回已回收的 pdst、
# 给 ROB 报完成) 修在 rtl/fpu/o6_fpu_exu.sv 之后，用新 RTL 编 SIMTAG=r2b，
#   1 重跑原失败 ELF (work/r2/fail/r2fp20331)
#   2 按原种子重生成浮点类程序 (种子 20000 起、KINDS=fp) 跑到 TARGET 条提交，结果写 work/r2b/results.txt
# 等可用内存 >= NEED GB 再编 (132 上 DC 在跑时余量很小)
#   scripts/r2_recheck.sh [TARGET，默认 6.6e6 ≈ 100 个] [NEED，默认 8]
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TARGET=${1:-6600000}
NEED=${2:-8}
until [ "$(free -g | awk '/^Mem/ {print $7}')" -ge "$NEED" ]; do sleep 300; done
export PATH=/export/master/rvtools/bin:$PATH UVM=1 SIMTAG=r2b COV=
R=$ROOT/work/r2b
mkdir -p $R
bash $ROOT/scripts/run_vcs.sh build | tail -1
e=$ROOT/work/r2/fail/r2fp20331
TIMEOUT=3600 bash $ROOT/scripts/run_vcs.sh run $e +max_cycles=100000000 > /dev/null
log=$ROOT/work/vcs/runs_r2b/r2fp20331/sim.log
echo "原失败用例：$(grep -a -m1 -oE '=== (PASS|FAIL) === [^ ]*' $log) $(grep -a -m1 -oE '=== UVM (PASS|FAIL) ===' $log) 断言 $(grep -a -c -E '^Error: "' $log)" | tee $R/summary.txt
NOBUILD=1 R2DIR=$R R2TAG=r2b KINDS=fp JOBS=${JOBS:-2} bash $ROOT/scripts/r2_campaign.sh $TARGET 20000 | tail -2 | tee -a $R/summary.txt
echo R2_RECHECK_DONE | tee -a $R/summary.txt
# 3 R1 (docs/13 §11：RTL 改动后必须重跑)：uvm_regress 全部阶段 + soc / c1 / midrst；旧结果先归档 (结果文件是追加写的)
[ -n "$NOR1" ] && exit 0
B=$ROOT/work/uvm/old_$(date +%m%d_%H%M); mkdir -p $B; mv $ROOT/work/uvm/results_*.txt $B/ 2>/dev/null
JOBS=${R1JOBS:-4} bash $ROOT/scripts/uvm_regress.sh isa dir vt rand vrand bench irq arch soc c1 midrst report > $ROOT/work/r1_36.log 2>&1
for f in $ROOT/work/uvm/results_*.txt; do
  echo "R1 $(basename $f .txt | sed 's/results_//')：$(wc -l < $f) 个，不过 $(grep -v '=== PASS ===.*UVM PASS.*断言 0$' $f | wc -l)"
done | tee -a $R/summary.txt
echo R1_DONE | tee -a $R/summary.txt
