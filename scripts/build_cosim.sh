#!/bin/bash
# 编译对拍库 libcosim.so (在 132 上运行)
# Spike 头文件要求 C++20，VCS 2018 自带 gcc 4.8 编不了，所以用 conda 的 g++ 12 单独编成 .so，
# 并静态链接 libstdc++/libgcc，避免运行时与 VCS 的 libstdc++ 冲突。
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SP=/export/master/src/riscv-isa-sim
B=$SP/build
CXX=/export/master/conda/rv/bin/x86_64-conda-linux-gnu-g++
OUT=$ROOT/work/cosim
mkdir -p $OUT
$CXX -std=c++20 -O2 -g -fPIC -shared \
  -I$B -iquote $SP/riscv -iquote $SP/fesvr -iquote $SP/softfloat -iquote $SP/disasm -iquote $SP \
  $ROOT/tb/cosim/cosim.cc \
  -Wl,--whole-archive $B/libriscv.a -Wl,--no-whole-archive \
  $B/libdisasm.a $B/libsoftfloat.a $B/libfesvr.a $B/libfdt.a \
  -static-libstdc++ -static-libgcc -ldl -lpthread \
  -o $OUT/libcosim.so.new
# 原子替换：正在运行的仿真继续用旧文件 (同一 inode 被就地改写会让它们崩溃)
mv -f $OUT/libcosim.so.new $OUT/libcosim.so
echo "built $OUT/libcosim.so"
