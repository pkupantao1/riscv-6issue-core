#!/bin/bash
# 形式验证 (JasperGold 2020.03，许可 27000@icpc)：verif/formal/run_fv.sh <名字> [额外 analyze 宏]
#   跑 verif/formal/<名字>.tcl，工作目录 work/formal/<名字>，摘要在 <名字>.rpt；打印每条性质的结论
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
N=$1
W=$ROOT/work/formal/$N${FV_TAG:+_$FV_TAG}
rm -rf $W; mkdir -p $W; cd $W
export O6_FV_DEFS="$2" O6_ROOT=$ROOT O6_FV_RTL=${FV_RTL:-$ROOT}   # FV_RTL=<树>：用另一棵 rtl (变异实验)
( ulimit -v 16000000; timeout ${TIMEOUT:-7200} nice -n 5 /export/SoftWare/Cadence/JASPERGOLD20.03.000/bin/jg -batch \
    -tcl $ROOT/verif/formal/$N.tcl > jg.log 2>&1 )
echo "== $N  (jg rc=$?)"
grep -a -E '^\[[0-9]+\]|proven|cex|covered|unreachable|undetermined|error|ERROR' $N.rpt 2>/dev/null | head -40
grep -a -i -m5 -E 'license|^ERROR' jg.log
