#!/bin/bash
# o6_vu 重构用的向量回归 (每改一步 RTL 跑一遍)：单独编 SIMTAG=vx (UVM 检查器开、覆盖率关)，
#   1 定向 work/tests/vt/*  2 vgen 随机 N 个 (全部类别，最后 N/4 个混入会出异常的访存)  3 vbench 热测拍数 (性能不退化)
#   与 Spike 逐条对拍、VU 跑空时整组比对 32 个向量寄存器；结果 work/vx/summary.txt (一行一阶段)
#   scripts/vu_regress.sh [N，默认 120] [起始种子，默认 9500]
#   环境变量：JOBS (默认 4)、NOBUILD=1、NOBENCH=1、VDEFS="+define+O6_VDLEN=1024" (换 DLEN)、SIMTAG (默认 vx)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
N=${1:-120}
S0=${2:-9500}
J=${JOBS:-4}
export UVM=1 COV= SIMTAG=${SIMTAG:-vx} ROOT VDEFS
export PATH=/export/master/rvtools/bin:$PATH
PY=/export/master/conda/rv/bin/python
R=$ROOT/work/$SIMTAG
mkdir -p $R/elf
cp $ROOT/work/rand/r.ld $R/elf/r.ld
SUM=$R/summary.txt
: > $SUM
[ -n "$NOBUILD" ] || { b=$(bash $ROOT/scripts/run_vcs.sh build | tail -1); echo "编译：$b" | tee -a $SUM; }

one() {   # one <elf> <CPU 秒> [plusargs...]：打印 "名字 | tb 判定 | UVM 判定 | 断言 N"
  local elf=$1 to=$2; shift 2
  TIMEOUT=$to bash $ROOT/scripts/run_vcs.sh run $elf "$@" > /dev/null
  local log=$ROOT/work/vcs/runs_$SIMTAG/$(basename $elf)/sim.log
  local t=$(grep -a -m1 -oE "=== (PASS|FAIL) === [^ ]*( [^ ]*)?" $log)
  local u=$(grep -a -m1 -oE "=== UVM (PASS|FAIL) ===" $log)
  local a=$(grep -a -c -E '^Error: "|RT-(MTOCMUCS|NCMUCS|MTOCMPCS|NCMPCS)' $log)
  echo "$(basename $elf) | ${t:-无结果} | ${u:-无 UVM 结果} | 断言 $a"
}
export -f one
summ() {
  local f=$R/results_$1.txt
  echo "[$1] 全过 $(grep -c '| === PASS.*| === UVM PASS === | 断言 0$' $f) / $(wc -l < $f)" | tee -a $SUM
  grep -v '| === PASS.*| === UVM PASS === | 断言 0$' $f | head -5 | sed 's/^/    不过：/' | tee -a $SUM
}

# 1 定向
ls $ROOT/work/tests/vt/* | xargs -P $J -I{} bash -c 'one {} 900' > $R/results_vt.txt
summ vt

# 2 随机
nt=$(( N / 4 ))
for ((s = S0; s < S0 + N; s++)); do
  e=$R/elf/vx$s; tr=""; [ $s -ge $(( S0 + N - nt )) ] && tr=--traps
  $PY $ROOT/verif/gen/vgen.py --seed $s --n 400 $tr -o $e.S > /dev/null &&
    riscv64-unknown-elf-as -march=rv64gcv_zicsr_zifencei_zvfh_zvfbfmin_zvfbfwma $e.S -o $e.o &&
    riscv64-unknown-elf-ld -static -nostdlib -T $R/elf/r.ld $e.o -o $e && rm -f $e.o && echo $e
done | xargs -P $J -I{} bash -c 'one {} 900 +max_cycles=3000000' > $R/results_vrand.txt
summ vrand

# 3 性能
if [ -z "$NOBENCH" ]; then
  bash $ROOT/scripts/run_vbench.sh > $R/vbench.txt 2>&1
  echo "[vbench] $(awk '{printf "%s=%s ", $1, $3}' $R/vbench.txt)" | tee -a $SUM
  grep -v "=== PASS" $R/vbench.txt | sed 's/^/    不过：/' | tee -a $SUM
fi
echo VU_REGRESS_DONE | tee -a $SUM
