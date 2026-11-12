#!/bin/bash
# 覆盖率回归：COV=1 SIMTAG=cov 编译，跑 ISA (-p/-v)、随机整数/浮点、基准，最后 urg 合并出报告
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export COV=1 SIMTAG=cov
bash $ROOT/scripts/run_vcs.sh build | tail -1
ls $ROOT/work/tests/isa/rv64{ui,um,ua,uc,uf,ud,mi,si}-[pv]-* 2>/dev/null | grep -v amocas | grep -v 'v-ma_data' | \
  xargs -P ${JOBS:-4} -I{} bash -c 'TIMEOUT=900 bash '$ROOT'/scripts/run_vcs.sh run {} > /dev/null'
bash $ROOT/scripts/run_rand.sh 8100 ${NRAND:-30} 2000 | head -1
FP=1 bash $ROOT/scripts/run_rand.sh 8200 ${NRAND:-30} 1500 | head -1
CSR=1 bash $ROOT/scripts/run_rand.sh 8300 ${NRAND:-30} 1500 | head -1
for b in $ROOT/work/tests/bench/*.riscv; do TIMEOUT=3000 bash $ROOT/scripts/run_vcs.sh run $b > /dev/null; done
bash $ROOT/scripts/cov_report.sh simv_cov
echo COV_DONE
