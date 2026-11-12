#!/bin/bash
# Verilator 版 SoC 仿真 (长程序：Linux 启动 / CoreMark 长跑)：tb_top.sv 原样 (TB_SOC) + 对拍库 libcosim.so
#   scripts/run_verilator.sh build
#   scripts/run_verilator.sh run <elf> [plusargs...]      (Linux 用 +nocosim +max_cycles=N)
#   环境变量：VTHREADS=Verilator 线程数 (默认 4)，JOBS=C++ 编译并行数 (默认 4)
# 不开 O6_UD_HALF (门控半速时钟只为 ASIC 时序；FPGA 版同样不开)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
W=$ROOT/work/vl
mkdir -p $W && cd $W || exit 1
export PATH=/export/master/conda/rv/bin:$PATH
export CXX=x86_64-conda-linux-gnu-g++ CC=x86_64-conda-linux-gnu-gcc
unset CFLAGS CXXFLAGS CPPFLAGS LDFLAGS
build() {
  sed "s#^#$ROOT/#" $ROOT/rtl/fpu/cvfpu.f > tp.f
  find $ROOT/rtl -name '*.sv' ! -path '*/pkg/*' ! -path '*/asic/*' | sort > rtl.f
  verilator --binary --timing -sv -Wno-fatal -Wno-lint -Wno-style -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
    --threads ${VTHREADS:-4} -O3 --x-assign fast --x-initial fast --assert \
    -DO6_SIM -DTB_SOC -DVERILATOR_SIM \
    -I$ROOT/rtl/soc -I$ROOT/tb/top -I$ROOT/third_party/cvfpu/src/common_cells/include \
    -f tp.f $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv -f rtl.f $ROOT/tb/top/tb_top.sv \
    --top-module tb_top -o Vtb_soc --Mdir obj_dir -j ${JOBS:-4} \
    -LDFLAGS "-L$ROOT/work/cosim -lcosim -Wl,-rpath,$ROOT/work/cosim" > build.log 2>&1
  rc=$?
  grep -E "^%Error" build.log | head -40
  echo "verilator rc=$rc"
  return $rc
}
case "$1" in
  build) build ;;
  run)
    elf=$2; shift 2
    name=$(basename $elf)
    mkdir -p runs/$name && cd runs/$name
    # stdbuf -oL：输出重定向到文件时 C stdio 是块缓冲，长仿真 (Linux) 几个小时看不到进度，分不清「慢」和「卡死」
    timeout ${TIMEOUT:-86400} stdbuf -oL $W/obj_dir/Vtb_soc +elf=$elf "$@" > sim.log 2>&1
    grep -aE "=== (PASS|FAIL)|\[cosim\]" sim.log | head -20
    ;;
  *) echo "usage: $0 build | run <elf> [plusargs]";;
esac
