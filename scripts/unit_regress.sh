#!/bin/bash
# 单元 TB + 形式证明一起跑 (docs/13 §5.4 / §5.6)：scripts/unit_regress.sh [unit] [fv]   (默认两者都跑；单核串行)
#   unit：tb/unit 的单元 TB (各 3 种子；向量 ALU 槽两种配置各 1 种子) + AGESEL 参考模型核对；fv：verif/formal 的 F1–F6
#   每项一行结论，最后汇总；结果也写 work/unit/summary.txt
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd $ROOT
WHAT=${*:-unit fv}
OUT=$ROOT/work/unit/summary.txt
mkdir -p work/unit; : > $OUT
bad=0
for w in $WHAT; do
  case $w in
    unit)
      for u in "freelist +cycles=200000" "iq +cycles=40000" "lsu_fwd +vec=200000" "dcache +cycles=40000" \
               "mmu +cycles=20000" "rob +cycles=40000"; do
        set -- $u
        r=$(bash scripts/run_unit.sh $1 3 ${@:2} 2>&1)
        l=$(echo "$r" | grep "种子通过"); f=$(echo "$r" | grep -o "功能 [0-9.]*%" | head -1)
        echo "unit $1：$l $f" | tee -a $OUT
        echo "$l" | grep -q "：3 / 3" || bad=$((bad + 1))
      done
      r=$(TAG=c1 VDEFS="+define+O6_SQ_SIZE=8" bash scripts/run_unit.sh lsu_fwd 2 +vec=200000 2>&1 | grep "种子通过")
      echo "unit lsu_fwd (SQ 8)：$r" | tee -a $OUT; echo "$r" | grep -q "：2 / 2" || bad=$((bad + 1))
      # 向量 ALU 槽：组合 (行为乘法器) 与 4 级流水 + DesignWare 仿真模型 (综合用的分段乘法器 / 华莱士树)，各对原写法锁步
      for v in "0:" "4:+define+O6_DW_SIM"; do
        r=$(TAG=np${v%%:*} VDEFS="+define+VU_ALU_NP=${v%%:*} ${v#*:}" bash scripts/run_unit.sh vu_alu 1 2>&1 | grep "种子通过")
        echo "unit vu_alu (NP ${v%%:*})：$r" | tee -a $OUT; echo "$r" | grep -q "：1 / 1" || bad=$((bad + 1))
      done
      if [ -f $ROOT/../pdk2n/cam/agesel.py ]; then
        r=$(PYTHONIOENCODING=utf-8 python3 verif/unit/agesel_xcheck.py 100000 32 | tail -1)
        echo "agesel：$r" | tee -a $OUT; echo "$r" | grep -q "不一致 0" || bad=$((bad + 1))
      else echo "agesel：没有 ../pdk2n/cam/agesel.py，跳过" | tee -a $OUT; fi ;;
    fv)
      for n in fv_ptr_order fv_pmp fv_freelist fv_mshr fv_brmask fv_rob; do
        bash verif/formal/run_fv.sh $n > /dev/null 2>&1
        p=work/formal/$n/$n.props
        if [ -f $p ]; then
          pr=$(grep -v precondition $p | grep -c "proven")
          # (原先 F4 按名字排除 dut._assert_1 =「回填行已在组里」；该检查已改成 $error 不再是性质，排除去掉 —— 编号会随断言增删漂移)
          cx=$(grep -v precondition $p | grep -c " cex")
          ud=$(grep -v precondition $p | grep -c "undetermined")
          echo "fv $n：证明 $pr，反例 $cx，未定 $ud" | tee -a $OUT; [ $((cx + ud)) -eq 0 ] || bad=$((bad + 1))
        else
          r=$(grep -a -E "proven|cex" work/formal/$n/$n.rpt 2>/dev/null | head -2 | tr -s ' ' | tr '\n' ' ')
          echo "fv $n：$r" | tee -a $OUT
        fi
      done ;;
  esac
done
echo "== 单元 / 形式：不过 $bad 项" | tee -a $OUT
