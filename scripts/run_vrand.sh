#!/bin/bash
# 随机向量程序回归：scripts/run_vrand.sh <起始种子> <个数> [每个程序的指令数] [vgen 类别]
# 每个种子：vgen.py 生成 -> as/ld -> VCS 对拍；汇总到 work/vrand/summary_<起始>_<个数>.txt
# 环境变量同 run_vcs.sh (SIMTAG / SOC)；TRAPS=1 混入少量会出异常的访存
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export PATH=/export/master/rvtools/bin:$PATH
PY=/export/master/conda/rv/bin/python
s0=$1; n=$2; len=${3:-400}; cls=${4:-vset,alu,cmp,mvs,mem,fx,mul,carry,wide,narrow,ext,div,red,mlog,mset,perm,vmvr,fpa,fma,fwide,fcvt,fcmp,fred,fmisc,mseg,midx,mwhole,mmask,mff,fp8,dot,bb}
D=$ROOT/work/vrand${SOC:+_soc}
mkdir -p $D
[ -f $D/r.ld ] || cp $ROOT/work/rand/r.ld $D/r.ld
sum=$D/summary_${s0}_${n}.txt
: > $sum
for ((s = s0; s < s0 + n; s++)); do
  $PY $ROOT/verif/gen/vgen.py --seed $s --n $len --cls $cls ${TRAPS:+--traps} -o $D/v$s.S || { echo "seed $s GEN_FAIL" >> $sum; continue; }
  riscv64-unknown-elf-as -march=rv64gcv_zicsr_zifencei_zvfh_zvfbfmin_zvfbfwma_zvbb $D/v$s.S -o $D/v$s.o 2> $D/v$s.cc.log && \
  riscv64-unknown-elf-ld -static -nostdlib -T $D/r.ld $D/v$s.o -o $D/v$s.elf 2>> $D/v$s.cc.log || \
      { echo "seed $s CC_FAIL $(head -c 300 $D/v$s.cc.log)" >> $sum; continue; }
  r=$(TIMEOUT=900 bash $ROOT/scripts/run_vcs.sh run $D/v$s.elf +max_cycles=3000000 | grep -E "=== (PASS|FAIL)" | head -1)
  echo "seed $s $r" >> $sum
done
echo "PASS $(grep -c '=== PASS' $sum) / $n"
grep -v "=== PASS" $sum | head -20
