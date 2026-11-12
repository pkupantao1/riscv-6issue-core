#!/bin/bash
# 分支预测设计空间评估：编 bptrace / bpsim，用 Spike 生成 trace，跑各预设
#   scripts/bp_eval.sh build                 编工具
#   scripts/bp_eval.sh trace <elf> [名字]    生成 work/bp/<名字>.bpt
#   scripts/bp_eval.sh bench                 现有基准全部生成 trace
#   scripts/bp_eval.sh run <预设...>         对 work/bp/*.bpt 跑 (BPT="a.bpt b.bpt" 可指定)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
W=$ROOT/work/bp
mkdir -p $W
export PATH=/export/master/conda/rv/bin:/export/master/rvtools/bin:$PATH
CXX=x86_64-conda-linux-gnu-g++
ISA=rv64imafdc_zicsr_zifencei_zicntr_zihpm
case "$1" in
  build)
    $CXX -O2 -std=c++17 -o $W/bptrace $ROOT/verif/bp/bptrace.cc || exit 1
    $CXX -O2 -std=c++17 -o $W/bpsim   $ROOT/verif/bp/bpsim.cc   || exit 1
    echo built ;;
  trace)
    elf=$2; n=${3:-$(basename $elf .riscv)}
    spike -l --isa=$ISA $SPIKE_ARGS $elf 2>&1 >/dev/null | $W/bptrace $W/$n.bpt $MAXI ;;
  bench)
    for b in $ROOT/work/tests/bench/*.riscv; do bash $0 trace $b; done ;;
  run)
    shift
    args=""; for p in "$@"; do args="$args -p $p"; done
    $W/bpsim $args ${BPT:-$W/*.bpt} ;;
  *) echo "usage: $0 build | trace <elf> [名字] | bench | run <预设...>";;
esac
