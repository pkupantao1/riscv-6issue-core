#!/bin/bash
# 编译 riscv-tests (ISA 测试) 到 work/tests/isa (在 132 上运行，需要 riscv64-unknown-elf-gcc)
# 直接在源码的 isa/ 目录里编 (它的 Makefile 自带 src_dir；不经 configure 的构建目录)
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export PATH=/export/master/rvtools/bin:$PATH
SRC=/export/master/src/riscv-tests
OUT=$ROOT/work/tests/isa
mkdir -p $OUT
cd $SRC/isa
make -j8 XLEN=64 RISCV_PREFIX=riscv64-unknown-elf- \
  rv64ui rv64um rv64ua rv64uc rv64mi rv64si rv64uf rv64ud rv64uzba rv64uzbb rv64uzbs 2>&1 | tail -3
cp -f rv64*-p-* $OUT/ 2>/dev/null || true
cp -f rv64*-v-* $OUT/ 2>/dev/null || true
rm -f $OUT/*.dump
ls $OUT | wc -l
