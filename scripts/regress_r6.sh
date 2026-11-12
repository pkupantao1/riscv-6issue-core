#!/bin/bash
# 完整回归：riscv-tests (-p / -v) + 随机 (整数 / 浮点) + 基准，汇总到 work/regress_<标签>.txt
# 用法：scripts/regress.sh [标签]      环境变量同 run_vcs.sh (SOC=1 / FE1=1 (M1 前端))，NRAND=随机种子数 (默认 30)
# 预期不过 (不计入失败)：ma_data (未对齐访存走陷入)、amocas_* (Zacas 未实现，-v 下会死循环所以直接跳过)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
tag=${1:-$(date +%m%d_%H%M)}
R=$ROOT/work/regress_$tag.txt
: > $R
NRAND=${NRAND:-30}
EXPECT='ma_data|amocas'

bash $ROOT/scripts/run_vcs.sh build | tail -1 >> $R

# ISA：并行 JOBS 个 (默认 4)，每行输出 "<名字> <结果>"
ls $ROOT/work/tests/isa/rv64{ui,um,ua,uc,uf,ud,mi,si,uzba,uzbb,uzbs}-[pv]-* 2>/dev/null | grep -v amocas |   xargs -P ${JOBS:-4} -I{} bash -c 'r=$(TIMEOUT=900 bash '$ROOT'/scripts/run_vcs.sh run {} | grep -E "=== (PASS|FAIL)" | head -1); echo "$(basename {}) ${r:-无结果(超时/崩溃)}"'   > $ROOT/work/regress_isa_$tag.txt
total=0; pass=0; fails=""
while read -r n r; do
  if [[ $r == *"=== PASS"* ]]; then pass=$((pass+1)); total=$((total+1))
  elif [[ $n =~ $EXPECT ]]; then echo "预期不过 $n $r" >> $R
  else total=$((total+1)); fails="$fails
$n $r"; fi
done < $ROOT/work/regress_isa_$tag.txt
echo "ISA: $pass / $total" >> $R
[ -n "$fails" ] && echo -e "ISA 失败:$fails" >> $R

# 定向测试 (verif/tests/*.S，riscv gcc 编，链接脚本同随机测试)
mkdir -p $ROOT/work/tests/dir
# 链接脚本由 rvgen 顺带写出 (随机测试那一步)；新目录里还没有时先生成一份
if [ ! -f $ROOT/work/rand/r.ld ]; then
  mkdir -p $ROOT/work/rand
  /export/master/conda/rv/bin/python $ROOT/verif/gen/rvgen.py --seed 1 --n 10 -o /tmp/rvgen_ld_$$.S --ld $ROOT/work/rand/r.ld > /dev/null
  rm -f /tmp/rvgen_ld_$$.S
fi
dpass=0; dtot=0
for src in $ROOT/verif/tests/*.S; do
  n=$(basename $src .S); dtot=$((dtot+1))
  /export/master/rvtools/bin/riscv64-unknown-elf-gcc -march=rv64gc_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles     -static -T $ROOT/work/rand/r.ld $src -o $ROOT/work/tests/dir/$n 2> $ROOT/work/tests/dir/$n.err || { echo "定向 $n 编译失败" >> $R; continue; }
  r=$(TIMEOUT=600 bash $ROOT/scripts/run_vcs.sh run $ROOT/work/tests/dir/$n | grep -E "=== (PASS|FAIL)" | head -1)
  [[ $r == *"=== PASS"* ]] && dpass=$((dpass+1)) || echo "定向 $n ${r:-无结果}" >> $R
done
echo "定向: $dpass / $dtot" >> $R

# 调试模块自检 (SoC 模式才有 JTAG/DM)
if [ -n "$SOC" ]; then
  /export/master/rvtools/bin/riscv64-unknown-elf-gcc -march=rv64gc_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles     -static -T $ROOT/work/rand/r.ld $ROOT/verif/dbg/dbg_loop.S -o $ROOT/work/tests/dir/dbg_loop
  r=$(TIMEOUT=1200 bash $ROOT/scripts/run_vcs.sh run $ROOT/work/tests/dir/dbg_loop +dbgtest +nocosim | grep -E "=== (PASS|FAIL)" | head -1)
  echo "调试自检: ${r:-无结果}" >> $R
  r=$(TIMEOUT=1200 bash $ROOT/scripts/run_vcs.sh run $ROOT/work/tests/dir/dbg_loop +mbisttest +nocosim | grep -E "=== (PASS|FAIL)" | head -1)
  echo "MBIST 自检: ${r:-无结果}" >> $R
fi

s0=$((RANDOM % 9000 + 1000))
bash $ROOT/scripts/run_rand.sh $s0 $NRAND 2000 > /dev/null
echo "随机整数 (种子 $s0..): $(grep -c '=== PASS' $ROOT/work/rand${SOC:+_soc}/summary_${s0}_${NRAND}.txt) / $NRAND" >> $R
grep -v '=== PASS' $ROOT/work/rand${SOC:+_soc}/summary_${s0}_${NRAND}.txt >> $R
FP=1 bash $ROOT/scripts/run_rand.sh $s0 $NRAND 1500 > /dev/null
echo "随机浮点 (种子 $s0..): $(grep -c '=== PASS' $ROOT/work/rand_fp${SOC:+_soc}/summary_fp_${s0}_${NRAND}.txt) / $NRAND" >> $R
grep -v '=== PASS' $ROOT/work/rand_fp${SOC:+_soc}/summary_fp_${s0}_${NRAND}.txt >> $R

CSR=1 bash $ROOT/scripts/run_rand.sh $s0 $NRAND 1500 > /dev/null
echo "随机 CSR (种子 $s0..): $(grep -c '=== PASS' $ROOT/work/rand_csr${SOC:+_soc}/summary_csr_${s0}_${NRAND}.txt) / $NRAND" >> $R
grep -v '=== PASS' $ROOT/work/rand_csr${SOC:+_soc}/summary_csr_${s0}_${NRAND}.txt >> $R

echo "基准 (测量区 minstret / mcycle)：" >> $R
for b in $ROOT/work/tests/bench/*.riscv; do
  n=$(basename $b .riscv)
  out=$(TIMEOUT=3000 bash $ROOT/scripts/run_vcs.sh run $b +max_cycles=20000000 | grep -E "=== (PASS|FAIL)" | head -1)
  log=$ROOT/work/vcs/runs${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}/$n.riscv/sim.log
  c=$(grep -a '^mcycle' $log | awk '{print $3}'); i=$(grep -a '^minstret' $log | awk '{print $3}')
  ipc=$([ -n "$c" ] && awk -v i=$i -v c=$c 'BEGIN{printf "%.3f", i/c}')
  printf "  %-10s IPC=%-6s %s\n" $n "$ipc" "${out:0:40}" >> $R
done
# unique / priority case 违例 (VCS 只报警告不停仿真；综合按 unique 当并行 case，重叠会出错) —— 一律算失败
RD=$ROOT/work/vcs/runs${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}
uv=$(grep -h -A3 -E 'RT-(MTOCMUCS|NCMUCS|MTOCMPCS|NCMPCS)' $RD/*/sim.log 2>/dev/null | grep -oE '/[^ ,]*\.sv, [0-9]+' | sort | uniq -c)
if [ -n "$uv" ]; then echo "unique/priority 违例 (失败):" >> $R; echo "$uv" >> $R; else echo "unique/priority 违例: 0" >> $R; fi
echo REGRESS_DONE >> $R
cat $R
