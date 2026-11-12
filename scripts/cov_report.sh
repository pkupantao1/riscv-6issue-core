#!/bin/bash
# 合并覆盖率并出报告：scripts/cov_report.sh [simv 名，默认 simv_cov]
ROOT=$(cd "$(dirname "$0")/.." && pwd)
V=$ROOT/work/vcs/${1:-simv_cov}.vdb
OUT=$ROOT/work/cov_report_${1:-simv_cov}
rm -rf $OUT
urg -dir $V -report $OUT -format text -metric line+cond+fsm+branch+tgl > $OUT.log 2>&1
echo "== 总体"; sed -n '1,40p' $OUT/dashboard.txt 2>/dev/null | grep -E "SCORE|LINE|COND|TOGGLE|FSM|BRANCH|^ *[0-9]" | head -8
echo "== 各模块 (按行覆盖升序前 30)"
grep -E "^ *[0-9.]+ +[0-9.-]+ " $OUT/modlist.txt 2>/dev/null | sort -k2 -n | head -30
