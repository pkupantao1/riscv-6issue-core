#!/bin/bash
# 随机程序回归：scripts/run_rand.sh <起始种子> <个数> [每个程序的指令数] [额外 plusargs...]
# 每个种子：rvgen.py 生成 -> as/ld -> VCS 对拍；汇总到 work/rand/summary_<起始>_<个数>.txt
# 环境变量 SOC=1 用 SoC 测试平台，FP=1 加浮点类别，VM=1 主体在 S 态分页下 (rvgen --vm，Sv39 / 48 / 57 随机)，RVA23=1 加 RVA23 新增指令，HYP=1 H 扩展 (rvgen --hyp)，MARCH 指定 -march (默认 rv64imac / FP 时 rv64gc)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export PATH=/export/master/rvtools/bin:$PATH
PY=/export/master/conda/rv/bin/python
s0=$1; n=$2; len=${3:-2000}; shift 3 2>/dev/null
D=$ROOT/work/rand${FP:+_fp}${CSR:+_csr}${VM:+_vm}${HYP:+_hyp}${RVA23:+_rva}${SOC:+_soc}
P=${FP:+f}${CSR:+c}${VM:+m}${HYP:+h}${RVA23:+v}${SOC:+s}r     # 文件名前缀 (运行目录按 ELF 名区分，避免并行回归互相覆盖)
mkdir -p $D
sum=$D/summary${FP:+_fp}${CSR:+_csr}${VM:+_vm}${HYP:+_hyp}${RVA23:+_rva}_${s0}_${n}.txt
: > $sum
for ((s = s0; s < s0 + n; s++)); do
  $PY $ROOT/verif/gen/rvgen.py --seed $s --n $len ${FP:+--fp} ${CSR:+--csr} ${VM:+--vm} ${HYP:+--hyp} ${RVA23:+--rva23} -o $D/$P$s.S --ld $D/r.ld || { echo "seed $s GEN_FAIL" >> $sum; continue; }
  # 不依赖 gcc：rvgen 只生成纯汇编，直接 as + ld
  riscv64-unknown-elf-as -march=${MARCH:-$([ -n "$FP" ] && echo rv64gc_zicsr_zifencei || echo rv64imac_zicsr_zifencei)${RVA23:+_zicond_zcb_zimop_zcmop_zawrs_zba_zbb_zicbom_zicboz${FP:+_zfa}}${HYP:+_h_zawrs}} $D/$P$s.S -o $D/$P$s.o 2> $D/$P$s.cc.log && \
  riscv64-unknown-elf-ld -static -nostdlib -T $D/r.ld $D/$P$s.o -o $D/$P$s.elf 2>> $D/$P$s.cc.log || \
      { echo "seed $s CC_FAIL $(head -c 200 $D/$P$s.cc.log)" >> $sum; continue; }
  r=$(TIMEOUT=600 bash $ROOT/scripts/run_vcs.sh run $D/$P$s.elf +max_cycles=2000000 "$@" | grep -E "=== (PASS|FAIL)" | head -1)
  echo "seed $s $r" >> $sum
done
echo "PASS $(grep -c '=== PASS' $sum) / $n"
grep -v "=== PASS" $sum | head -20
