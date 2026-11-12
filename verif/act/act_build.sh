#!/bin/bash
# 用 ACT4 为 OOO6 生成自检 ELF：verif/act/act_build.sh [扩展列表，逗号分隔，空 = UDB 配置里的全部]
#   工具 (2026-10-20 装在 132 的 /home/master/tools)：act-env (Python 3.12 + Ruby 3.4 + gcc 15，conda-forge)、Sail 0.15、
#   UDB gem (BUNDLE_PATH=/home/master/tools/gems；ffi 强制源码编译，Z3 用 conda-forge 的 libz3 5.1.0)
#   产物：work/act/o6-rv64gc/elfs/*.elf
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
A=/export/master/src/riscv-arch-test
T=/home/master/tools
C=$ROOT/verif/act/o6-rv64gc
export PATH=$T/act-env/bin:$T/sail/bin:/export/master/rvtools/bin:$PATH
export BUNDLE_PATH=$T/gems BUNDLE_FORCE_RUBY_PLATFORM=true LD_LIBRARY_PATH=$T/act-env/lib
export VIRTUAL_ENV=$T/act-env      # Makefile：无 uv / mise 时要求「已激活的 venv」并在 PATH 里找 act / testgen / covergroupgen
[ -f $C/sail.json ] || python $ROOT/verif/act/mk_sail_json.py $A/config/cores/cvw/cvw-rv64gc/sail.json $C/sail.json
[ -f $C/link.ld ] || cp $A/config/cores/cvw/cvw-rv64gc/link.ld $C/link.ld
cd $A
make CONFIG_FILES=$C/test_config.yaml WORKDIR=$ROOT/work/act ${1:+EXTENSIONS=$1} JOBS=${JOBS:-6} FAST=True
ls $ROOT/work/act/o6-rv64gc/elfs | wc -l
echo ACT_BUILD_DONE
