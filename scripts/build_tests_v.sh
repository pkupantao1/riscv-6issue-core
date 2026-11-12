#!/bin/bash
# 用 riscv64-unknown-elf-gcc 编 riscv-tests 的 -v 环境 (虚拟内存：S 态 + Sv39 页表，随机页映射)
# 编译参数照抄 riscv-tests/isa/Makefile；ENTROPY 由测试名的 md5 决定 (与官方一致)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RT=/export/master/src/riscv-tests
BIN=/export/master/rvtools/bin
OUT=$ROOT/work/tests/isa
mkdir -p $OUT/obj
n=0; f=0
for suite in ${SUITES:-rv64ui rv64um rv64ua rv64uc rv64uf rv64ud}; do
  for src in $RT/isa/$suite/*.S; do
    t=$(basename $src .S)
    name=$suite-v-$t
    ent=0x$(echo -n $name | md5sum | cut -c 1-7)
    $BIN/riscv64-unknown-elf-gcc -march=rv64gc_zicsr_zifencei -mabi=lp64d -static -mcmodel=medany -fvisibility=hidden \
      -nostdlib -nostartfiles -DENTROPY=$ent -std=gnu99 -O2 \
      -I$RT/env/v -I$RT/isa/macros/scalar -T$RT/env/v/link.ld \
      $RT/env/v/entry.S $RT/env/v/*.c $src -o $OUT/$name 2> $OUT/obj/$name.err \
    && n=$((n+1)) || { f=$((f+1)); echo "FAIL $name: $(head -c 300 $OUT/obj/$name.err)"; }
  done
done
echo "built=$n failed=$f"
