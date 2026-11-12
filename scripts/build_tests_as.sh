#!/bin/bash
# 不依赖 riscv gcc：用主机 cpp 展开 riscv-tests 的宏，再用 riscv64-unknown-elf-as/ld 汇编链接 (只做 -p 环境)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RT=/export/master/src/riscv-tests
BIN=/export/master/rvtools/bin
CPP=/export/master/conda/rv/bin/x86_64-conda-linux-gnu-cpp
OUT=$ROOT/work/tests/isa
mkdir -p $OUT/obj
n=0; f=0
for suite in rv64ui rv64um rv64ua rv64uc rv64mi rv64si rv64uf rv64ud; do
  for src in $RT/isa/$suite/*.S; do
    t=$(basename $src .S)
    name=$suite-p-$t
    $CPP -undef -nostdinc -x assembler-with-cpp -D__riscv -D__riscv_xlen=64 -D__riscv_flen=64 \
      -D__riscv_float_abi_double -I$RT/env/p -I$RT/isa/macros/scalar -I$RT/isa $src -o $OUT/obj/$name.s 2>$OUT/obj/$name.err \
    && $BIN/riscv64-unknown-elf-as -march=rv64gc_zicsr_zifencei -mabi=lp64d $OUT/obj/$name.s -o $OUT/obj/$name.o 2>>$OUT/obj/$name.err \
    && $BIN/riscv64-unknown-elf-ld -static -nostdlib -T $RT/env/p/link.ld $OUT/obj/$name.o -o $OUT/$name 2>>$OUT/obj/$name.err \
    && n=$((n+1)) || { f=$((f+1)); echo "FAIL $name: $(head -c 200 $OUT/obj/$name.err)"; }
  done
done
echo "built=$n failed=$f"
