#!/bin/bash
# R2 长随机对拍 (docs/13 §0「随机程序 >= 5×10⁷ 条」)：scripts/r2_campaign.sh [目标条数，默认 5e7] [起始种子，默认 20000]
#   单独编 SIMTAG=r2 (UVM 检查器开、覆盖率关)；按 整数 / 浮点 / CSR / vm / 向量 轮转生成长随机程序 (rvgen --n LEN、vgen --n VLEN)，
#   JOBS 个并行 (默认 2)；每个用例一行写 work/r2/results.txt (名字 | tb 判定 | UVM 判定 | 断言 | 提交条数)；
#   累计提交条数到目标就停。失败用例的 ELF 与日志留在 work/r2/fail/
#   复查 (改 RTL 后按种子重生成同一批程序再跑)：R2DIR=结果目录 R2TAG=仿真标签 KINDS="fp vm" (只跑这几类，默认全部)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TARGET=${1:-50000000}
S0=${2:-20000}
J=${JOBS:-2}
LEN=${LEN:-40000}
VLEN=${VLEN:-3000}
TAG=${R2TAG:-r2}
KINDS=${KINDS:-int fp csr vm vec}
export PATH=/export/master/rvtools/bin:$PATH UVM=1 SIMTAG=$TAG COV= ROOT
PY=/export/master/conda/rv/bin/python
R=${R2DIR:-$ROOT/work/r2}
case $R in /*) ;; *) R=$PWD/$R ;; esac      # run_vcs.sh 会 cd 进运行目录，ELF 路径必须是绝对的
mkdir -p $R/elf $R/fail
cp $ROOT/work/rand/r.ld $R/elf/r.ld
[ -n "$NOBUILD" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1

gen() {   # gen <种子>：按种子轮转类别，打印 ELF 路径
  local s=$1 k e
  case $((s % 5)) in 0) k=int;; 1) k=fp;; 2) k=csr;; 3) k=vm;; 4) k=vec;; esac
  [[ " $KINDS " == *" $k "* ]] || return 1
  e=$R/elf/r2${k}$s
  if [ $k = vec ]; then
    $PY $ROOT/verif/gen/vgen.py --seed $s --n $VLEN -o $e.S > /dev/null &&
      riscv64-unknown-elf-as -march=rv64gcv_zicsr_zifencei_zvfh_zvfbfmin_zvfbfwma $e.S -o $e.o || return 1
  else
    local o="" m=rv64imac_zicsr_zifencei
    case $k in fp) o=--fp; m=rv64gc_zicsr_zifencei;; csr) o=--csr;; vm) o=--vm;; esac
    $PY $ROOT/verif/gen/rvgen.py --seed $s --n $LEN $o -o $e.S --ld $R/elf/r.ld > /dev/null &&
      riscv64-unknown-elf-as -march=$m $e.S -o $e.o || return 1
  fi
  riscv64-unknown-elf-ld -static -nostdlib -T $R/elf/r.ld $e.o -o $e && rm -f $e.o && echo $e
}
one() {   # one <ELF>：跑并打印结果行；通过的删 ELF / .S / 运行目录省空间
  local e=$1 n=$(basename $1)
  TIMEOUT=3600 bash $ROOT/scripts/run_vcs.sh run $e +max_cycles=100000000 > /dev/null
  local log=$ROOT/work/vcs/runs_$SIMTAG/$n/sim.log
  local t=$(grep -a -m1 -oE "=== (PASS|FAIL) === [^ ]*( [^ ]*)?" $log)
  local u=$(grep -a -m1 -oE "=== UVM (PASS|FAIL) ===" $log)
  local a=$(grep -a -c -E '^Error: "|RT-(MTOCMUCS|NCMUCS|MTOCMPCS|NCMPCS)' $log)
  local c=$(grep -a -m1 -oE "commits=[0-9]+" $log | cut -d= -f2)
  echo "$n | ${t:-无结果} | ${u:-无 UVM 结果} | 断言 $a | ${c:-0}"
  if [[ "$t" == *PASS* && "$u" == *PASS* && $a -eq 0 ]]; then rm -rf $e $e.S $ROOT/work/vcs/runs_$SIMTAG/$n
  else cp $e $e.S $log $R/fail/ 2>/dev/null; mv $R/fail/sim.log $R/fail/$n.log 2>/dev/null; fi
}
export -f gen one
export R PY LEN VLEN KINDS

s=$S0
total=$(awk -F'|' '{s += $5} END {print s + 0}' $R/results.txt 2>/dev/null)
while [ "${total:-0}" -lt "$TARGET" ]; do
  # 一批 4 × J 个
  for ((i = 0; i < 4 * J; i++)); do gen $((s + i)); done | xargs -P $J -I{} bash -c 'one {}' >> $R/results.txt
  s=$((s + 4 * J))
  total=$(awk -F'|' '{s += $5} END {print s + 0}' $R/results.txt)
  nf=$(grep -v "=== PASS ===.*UVM PASS.*断言 0 " $R/results.txt | wc -l)
  echo "$(date +%H:%M) 种子到 $s：累计 $(wc -l < $R/results.txt) 个、提交 $total 条、不过 $nf 个"
done
echo R2_DONE
