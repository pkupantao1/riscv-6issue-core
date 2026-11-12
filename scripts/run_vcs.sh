#!/bin/bash
# VCS 编译 + 运行 (在 132 上运行)
#   scripts/run_vcs.sh build            只编译
#   scripts/run_vcs.sh run <elf> [plusargs...]
#   TIMEOUT=N：仿真最多用 N 秒 CPU 时间 (不是墙钟)
#   环境变量：VDEFS="+define+X=1 ..." 额外宏 (性能对比实验)，SOC=1 SoC 模式，FE1=1 M1 前端，SIMTAG=x 独立 simv，COV=1 收覆盖率 (simv*.vdb，scripts/cov_report.sh 出报告)，
#     UVM=1 编进 UVM 环境 (tb/uvm，+UVM_TESTNAME=o6_base_test / o6_irq_test / o6_irq_burst_test；功能覆盖率进同一个 vdb)
# 结果看 run 目录下 sim.log 的 "=== PASS ===" / "=== FAIL"，不看退出码；sim.log 里有断言失败 (Error:) 也算 FAIL (NOASSERT=1 关掉)。
ROOT=$(cd "$(dirname "$0")/.." && pwd)
W=$ROOT/work/vcs
mkdir -p $W
cd $W
build() {
  # STUB=1：用桩函数代替对拍库 (只检查 RTL/TB 能否编译)
  if [ -n "$STUB" ]; then DPI="$ROOT/tb/cosim/stub.c"
  else DPI="$ROOT/work/cosim/libcosim.so -LDFLAGS -Wl,-rpath,$ROOT/work/cosim"; fi
  # cvfpu 必须在我们的文件之前编译 (我们文件里的 default_nettype none 会泄漏到后面)
  sed "s#^#$ROOT/#" $ROOT/rtl/fpu/cvfpu.f > tp.f
  find $ROOT/rtl -name '*.sv' ! -path '*/pkg/*' ! -path '*/asic/*' | sort > rtl.f
  vcs -full64 -sverilog -timescale=1ns/1ps -lca +lint=TFIPC-L -assert svaext \
      +define+O6_SIM +define+O6_UD_HALF +incdir+$ROOT/rtl/soc +incdir+$ROOT/tb/top +incdir+$ROOT/third_party/cvfpu/src/common_cells/include ${WAVE:+-debug_access+all -kdb +define+DUMP_FSDB} \
      -CFLAGS -g ${COV:+-cm line+cond+fsm+branch+tgl -cm_hier $ROOT/scripts/cov_hier.cfg} \
      ${VDEFS} -f tp.f $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv -f rtl.f ${UVM:+-ntb_opts uvm-1.2 +define+O6_UVM +incdir+$ROOT/tb/uvm $ROOT/tb/uvm/o6_uvm_if.sv $ROOT/tb/uvm/o6_uvm_pkg.sv} $ROOT/tb/top/tb_top.sv \
      $DPI \
      ${SOC:++define+TB_SOC} ${FE1:++define+FE_M1} -Mdir=csrc${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG} -top tb_top -o simv${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG} -l compile${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}.log > /dev/null 2>&1
  rc=$?
  grep -E "^Error|Error-\[" compile${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}.log | head -40
  echo "vcs rc=$rc"
  return $rc
}
case "$1" in
  build) build ;;
  run)
    elf=$2; shift 2
    name=$(basename $elf)
    mkdir -p runs${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}/$name && cd runs${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}/$name
    # TIMEOUT 是 CPU 秒 (ulimit -t)：机器负载高时只是墙钟变长，不会误杀；另加 10 倍的墙钟上限兜底 (防挂起不占 CPU)
    ( ulimit -t ${TIMEOUT:-1800}
      exec timeout $(( ${TIMEOUT:-1800} * 10 )) ../../simv${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG} +elf=$elf ${COV:+-cm line+cond+fsm+branch+tgl -cm_name $name} "$@" ) > sim.log 2>&1
    # 断言 ($error) 失败时测试本身可能照样 tohost = 1：先报 FAIL (调用方都取第一行 PASS/FAIL)；NOASSERT=1 只看 tohost
    ne=$(grep -c "^Error:" sim.log)
    [ -z "$NOASSERT" ] && [ "$ne" -gt 0 ] && echo "=== FAIL === 断言失败 $ne 次 (首条：$(grep -m1 -A3 '^Error:' sim.log | tail -1 | head -c 160))"
    grep -E "=== (PASS|FAIL)|=== UVM|\[cosim\]|UVM_(ERROR|FATAL) [^:]" sim.log | head -20
    ;;
  *) echo "usage: $0 build | run <elf> [plusargs]";;
esac
