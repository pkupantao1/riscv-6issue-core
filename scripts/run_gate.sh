#!/bin/bash
# 门级冒烟 (tb/gate/tb_gate.sv)：
#   scripts/run_gate.sh build rtl                    参照：RTL (O6_ASIC + O6_UD_HALF，与 DC 读入一致；SRAM 宏模型 + 门控单元模型)
#   scripts/run_gate.sh build gate <网表.v>          DC 网表 + syn/gate/cells.v (由 lib2v.py 从 .lib 生成) + sram_models.v
#   scripts/run_gate.sh run rtl|gate <elf> [plusargs]
#   无复位的寄存器初值：编译带 +vcs+initreg+random，运行 +vcs+initreg+<种子> (默认 1)：只靠 X 悲观传播不会误判
#   工作目录 work/gate/<rtl|gate>；结果看 sim.log 的 "=== GATE PASS / FAIL"
ROOT=$(cd "$(dirname "$0")/.." && pwd)
G=$ROOT/syn/gate
mode=$2
W=$ROOT/work/gate/$mode
mkdir -p $W && cd $W || exit 1
COSIM="$ROOT/work/cosim/libcosim.so -LDFLAGS -Wl,-rpath,$ROOT/work/cosim"
case "$1" in
  build)
    if [ "$mode" = gate ]; then
      NL=$3
      [ -f "$NL" ] || { echo "没有网表 $NL"; exit 1; }
      vcs -full64 -sverilog -timescale=1ns/1ps +define+GATE +vcs+initreg+random -CFLAGS -g \
          $NL $G/cells.v $G/sram_models.v $ROOT/tb/gate/tb_gate.sv $COSIM -top tb_gate -o simv -l compile.log > /dev/null 2>&1
    else
      sed "s#^#$ROOT/#" $ROOT/rtl/fpu/cvfpu.f > tp.f
      find $ROOT/rtl -name '*.sv' ! -path '*/pkg/*' | sort > rtl.f
      vcs -full64 -sverilog -timescale=1ns/1ps -assert svaext +define+O6_ASIC +define+SYNTHESIS +define+O6_UD_HALF +vcs+initreg+random \
          +incdir+$ROOT/rtl/soc +incdir+$ROOT/third_party/cvfpu/src/common_cells/include -CFLAGS -g \
          -f tp.f $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv -f rtl.f $G/cells.v $G/sram_models.v \
          $ROOT/tb/gate/tb_gate.sv $COSIM -top tb_gate -o simv -l compile.log > /dev/null 2>&1
    fi
    rc=$?
    grep -E "^Error|Error-\[" -A3 compile.log | head -30
    echo "vcs rc=$rc" ;;
  run)
    elf=$3; shift 3
    name=$(basename $elf)
    mkdir -p runs/$name && cd runs/$name
    ( ulimit -t ${TIMEOUT:-3600}; exec ../../simv +elf=$elf +vcs+initreg+${SEED:-1} "$@" ) > sim.log 2>&1
    grep -a -E "=== GATE" sim.log | head -3 ;;
  *) echo "usage: $0 build rtl | build gate <netlist> | run rtl|gate <elf> [plusargs]" ;;
esac
