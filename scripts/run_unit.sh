#!/bin/bash
# 单元 TB (tb/unit/tb_<名字>.sv)：scripts/run_unit.sh <名字> [种子数 (默认 4)] [plusargs...]
#   VCS 编译 (被测 RTL + 包 + TB，收行 / 条件 / 分支覆盖率 + 功能覆盖率)，跑 S 个种子，urg 合并，打印功能覆盖率与代码覆盖率
#   工作目录 work/unit/<名字>；每个种子 run_<s>.log 里找 "=== UNIT PASS ==="
#   VDEFS="+define+..." 额外宏 (变异实验)；SRC=<目录> 用另一棵 rtl 树
ROOT=$(cd "$(dirname "$0")/.." && pwd)
N=$1; NS=${2:-4}; shift 2 2>/dev/null
SRC=${SRC:-$ROOT}
case $N in
  freelist) DUT="rtl/backend/o6_freelist.sv"; TOP=tb_freelist; HIER="tb_freelist.u256.dut tb_freelist.u192.dut" ;;
  iq)       DUT="rtl/common/o6_prio_enc.sv rtl/backend/o6_iq.sv"; TOP=tb_iq; HIER="tb_iq.u2.dut tb_iq.u1.dut" ;;
  lsu_fwd)  DUT="rtl/lsu/o6_sq_fwd.sv"; TOP=tb_lsu_fwd; HIER="tb_lsu_fwd.dut" ;;
  rob)      DUT="rtl/backend/o6_rob.sv"; TOP=tb_rob; HIER="tb_rob.dut" ;;
  mmu)      DUT="rtl/mmu/o6_pmp.sv rtl/mmu/o6_ptw.sv rtl/mmu/o6_tlb.sv"; TOP=tb_mmu; HIER="tb_mmu.u_ptw tb_mmu.u_itlb tb_mmu.u_dtlb" ;;
  dcache)   DUT="rtl/common/o6_mbist.sv rtl/common/o6_sram_sp.sv rtl/cache/o6_dcache.sv"; TOP=tb_dcache; HIER="tb_dcache.dut"; EXCL="o6_sram_sp" ;;
  # 向量 ALU 槽：新写法 vs 原写法 (tb/unit/o6_vu_alu_ref.sv) 锁步；o6_vu.sv 里引用 fpnew，带上 cvfpu
  vu_alu)   DUT="$(tr '\n' ' ' < $ROOT/rtl/fpu/cvfpu.f) rtl/vec/o6_vu.sv tb/unit/o6_vu_alu_ref.sv"; TOP=tb_vu_alu; HIER="tb_vu_alu"
            VDEFS="$VDEFS +incdir+$SRC/third_party/cvfpu/src/common_cells/include"
            # O6_DW_SIM：乘法器走 DesignWare 仿真模型 (sim_ver 里的 VCS 版靠 `include "vcs/..."，要 incdir)
            DWSV=/export/SoftWare/Synopsys/syn/O-2018.06-SP1/dw/sim_ver
            case "$VDEFS" in *O6_DW_SIM*) VDEFS="$VDEFS -y $DWSV +libext+.v +incdir+$DWSV" ;; esac ;;
  # 乘法器：VDEFS="+define+O6_DW_SIM [+define+O6_MUL_SEGW=N]" 走 DW02_multp / DW02_tree 仿真模型
  mul)      DUT="rtl/exu/o6_mul.sv"; TOP=tb_mul; HIER="tb_mul.dut"
            DWSV=/export/SoftWare/Synopsys/syn/O-2018.06-SP1/dw/sim_ver
            case "$VDEFS" in *O6_DW_SIM*) VDEFS="$VDEFS -y $DWSV +libext+.v +incdir+$DWSV" ;; esac ;;
  *) echo "未知单元 $N"; exit 1 ;;
esac
W=$ROOT/work/unit/$N${TAG:+_$TAG}
rm -rf $W; mkdir -p $W; cd $W
{ for h in $HIER; do echo "+tree $h"; done; for m in $EXCL; do echo "-moduletree $m"; done; } > cov.cfg   # EXCL：不计的子模块 (如数据 SRAM 的 MBIST)
vcs -full64 -sverilog -timescale=1ns/1ps -assert svaext +define+O6_SIM -cm line+cond+branch -cm_hier cov.cfg $VDEFS \
    $SRC/rtl/pkg/o6_isa_pkg.sv $SRC/rtl/pkg/o6_pkg.sv $(for f in $DUT; do echo $SRC/$f; done) $ROOT/tb/unit/tb_$N.sv \
    -top $TOP -o simv -l compile.log > /dev/null 2>&1 || { grep -E "^Error|Error-\[" -A4 compile.log | head -30; echo "vcs 失败"; exit 1; }
pass=0
for s in $(seq 1 $NS); do
  ./simv +ntb_random_seed=$s -cm line+cond+branch -cm_name s$s "$@" > run_$s.log 2>&1
  grep -q "=== UNIT PASS ===" run_$s.log && pass=$((pass + 1)) || grep -a -m5 -E "错误|UNIT FAIL|Error" run_$s.log
done
echo "== $N：$pass / $NS 种子通过"
urg -dir simv.vdb -report cov -format text > /dev/null 2>&1
# 功能覆盖率 (每个 covergroup 实例) + 代码覆盖率 (被测模块，cov.cfg 只收 DUT 层次)
grep -a -E "^ *[0-9.]+ +[0-9.]+ +[0-9]+ +[0-9]+ .*::" cov/groups.txt 2>/dev/null | awk '!seen[$NF]++ {print "  功能 " $1 "%  " $NF}'
grep -a -A2 "^Total Coverage Summary" cov/dashboard.txt 2>/dev/null | tail -2
