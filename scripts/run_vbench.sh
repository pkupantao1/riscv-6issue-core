#!/bin/bash
# 向量性能基准：scripts/run_vbench.sh [核...]   (环境变量同 run_vcs.sh，SIMTAG 选仿真版本)
# 每个核跑两遍，"热测" 是第二遍 (数据已在 cache) 的拍数 (rdcycle 差，经 tohost 最高位通道报给 tb)；
# 对拍照常开着 (结果正确性同时检查)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export PATH=/export/master/rvtools/bin:$PATH
PY=/export/master/conda/rv/bin/python
D=$ROOT/work/vbench${SIMTAG:+_$SIMTAG}
mkdir -p $D
all=$($PY $ROOT/verif/vbench/vbench.py $D)
ks=${*:-$all}
[ -f $D/r.ld ] || cp $ROOT/work/rand/r.ld $D/r.ld
for k in $ks; do
  riscv64-unknown-elf-gcc -march=rv64gcv_zicsr_zifencei_zvfh -mabi=lp64d -nostdlib -nostartfiles -static \
      -T $D/r.ld $D/$k.S -o $D/$k 2> $D/$k.err || { echo "$k 编译失败: $(head -c 200 $D/$k.err)"; continue; }
  out=$(TIMEOUT=1800 bash $ROOT/scripts/run_vcs.sh run $D/$k +max_cycles=20000000)
  r=$(echo "$out" | grep -aoE "=== (PASS|FAIL) ===[^c]*" | head -1)
  w=$(grep -a "^\[bench\]" $ROOT/work/vcs/runs${SIMTAG:+_$SIMTAG}/$k/sim.log | awk '{print $2}')
  printf "%-8s 热测 %8s 拍   %s\n" "$k" "$w" "$r"
done
