#!/bin/bash
# UVM 覆盖率回归：UVM=1 COV=1 SIMTAG=uvm 编译，按阶段跑用例，最后 urg 出代码 + 功能覆盖率报告
#   scripts/uvm_regress.sh [阶段...]     阶段：isa dir vt rand vrand bench irq report (默认全部)；
#                                        c1 = 小队列配置 C1 (单独编译 simv_c1，不收覆盖率；见 docs/13 §6)；
#                                        arch = riscv-arch-test (ACT4) 自检 ELF (先跑 verif/act/act_build.sh 生成)；
#                                        soc = SoC 模式 (SOC=1 编译 simv_soc_uvm；L2 / AXI / CLINT / PLIC / DM / MBIST)，覆盖率在 report 合并；
#                                        smoke = R0 冒烟 (ISA 每 8 个取 1、定向、随机各 2、向量 2、短基准 2、中断 2 种子，共 57 个，6 核约 1 分钟)
#   环境变量：JOBS=并行数 (默认 6)，NOBUILD=1 不重新编译，NIRQ=irq_ext 种子数 (默认 20)，NIRQC=irq_csr 种子数 (默认 10)，NIRQT=irq_tmr 种子数 (默认 10)，NRAND=每类随机程序数 (默认 30)
#   每个用例一行结果 (tb 判定 | UVM 判定) 追加到 work/uvm/results_<阶段>.txt
ROOT=$(cd "$(dirname "$0")/.." && pwd)
export UVM=1 COV=1 SIMTAG=uvm ROOT
export PATH=/export/master/rvtools/bin:$PATH
PY=/export/master/conda/rv/bin/python
J=${JOBS:-6}
R=$ROOT/work/uvm
mkdir -p $R/elf
STAGES=${*:-isa dir vt rand vrand bench irq arch report}

one() {   # one <elf> <CPU 秒> [plusargs...]：跑一个用例，打印 "名字 | tb 判定 | UVM 判定 | 断言 N"
  local elf=$1 to=$2; shift 2
  TIMEOUT=$to bash $ROOT/scripts/run_vcs.sh run $elf "$@" > /dev/null
  local log=$ROOT/work/vcs/runs${SOC:+_soc}${FE1:+_fe1}${SIMTAG:+_$SIMTAG}/$(basename $elf)/sim.log
  local t=$(grep -a -m1 -oE "=== (PASS|FAIL) === [^ ]*( [^ ]*)?" $log)
  local u=$(grep -a -m1 -oE "=== UVM (PASS|FAIL) ===" $log)
  # RTL 即时断言 ($error) 与 VCS unique / priority 违例：只报不停仿真，这里计入判定
  local a=$(grep -a -c -E '^Error: "|RT-(MTOCMUCS|NCMUCS|MTOCMPCS|NCMPCS)' $log)
  echo "$(basename $elf) | ${t:-无结果} | ${u:-无 UVM 结果} | 断言 $a"
}
export -f one
runlist() {   # runlist <阶段名> <CPU 秒> [plusargs...] < ELF 列表
  local st=$1 to=$2; shift 2
  xargs -P $J -I{} bash -c 'one "$@"' _ {} $to "$@" > $R/results_$st.txt
  summ $st
}
gen_dir() {   # 定向测试 (verif/tests/*.S) -> $R/elf/<名字>，打印 ELF 路径
  for src in $ROOT/verif/tests/*.S; do
    n=$(basename $src .S)
    riscv64-unknown-elf-gcc -march=rv64gc_zbb_zicsr_zifencei_zfhmin_zfbfmin -mabi=lp64d -nostdlib -nostartfiles -static       -T $ROOT/work/rand/r.ld $src -o $R/elf/$n 2> $R/elf/$n.err && echo $R/elf/$n
  done
}
gen_rand() {   # gen_rand <int|fp|csr> <起始种子> <个数>
  local k=$1 s0=$2 n=$3 o m len
  case $k in int) o=""; m=rv64imac_zicsr_zifencei; len=2000;;
             fp)  o=--fp; m=rv64gc_zicsr_zifencei; len=1500;;
             csr) o=--csr; m=rv64imac_zicsr_zifencei; len=1500;;
             vm)  o=--vm;  m=rv64imac_zicsr_zifencei; len=1500;; esac
  for ((s = s0; s < s0 + n; s++)); do
    e=$R/elf/u${k}$s
    $PY $ROOT/verif/gen/rvgen.py --seed $s --n $len $o -o $e.S --ld $R/elf/r.ld > /dev/null &&
      riscv64-unknown-elf-as -march=$m $e.S -o $e.o && riscv64-unknown-elf-ld -static -nostdlib -T $R/elf/r.ld $e.o -o $e &&
      echo $e
  done
}
gen_vrand() {   # gen_vrand <起始种子> <个数> [--traps]
  [ -f $R/elf/r.ld ] || cp $ROOT/work/rand/r.ld $R/elf/r.ld
  for ((s = $1; s < $1 + $2; s++)); do
    e=$R/elf/uv$s
    # 类别用 vgen 默认 (全部已实现的，含 V4 / V5 的 mseg / midx / mwhole / mmask / mff / fp8 / dot)
    $PY $ROOT/verif/gen/vgen.py --seed $s --n 400 $3 -o $e.S > /dev/null &&
      riscv64-unknown-elf-as -march=rv64gcv_zicsr_zifencei_zvfh_zvfbfmin_zvfbfwma $e.S -o $e.o &&
      riscv64-unknown-elf-ld -static -nostdlib -T $R/elf/r.ld $e.o -o $e && echo $e
  done
}
run_irq() {   # run_irq <irq_ext 种子数> <irq_csr 种子数> <irq_tmr 种子数> <结果名>
  local t
  for t in irq_ext irq_csr irq_tmr; do
    riscv64-unknown-elf-gcc -march=rv64gc_zbb_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles -static     -T $ROOT/work/rand/r.ld $ROOT/verif/tests/$t.S -o $R/elf/$t || return 1
  done
  # 同一个 ELF 按种子复制成不同名字 (运行目录 / 覆盖率测试名按 ELF 名区分)：irq_s<种子> = irq_ext，irqc_s<种子> = irq_csr
  { for ((s = 1; s <= $1; s++)); do cp $R/elf/irq_ext $R/elf/irq_s$s; echo "$R/elf/irq_s$s +UVM_TESTNAME=o6_irq_test +ntb_random_seed=$s"; done
    for ((s = 1; s <= $2; s++)); do cp $R/elf/irq_csr $R/elf/irqc_s$s; echo "$R/elf/irqc_s$s +UVM_TESTNAME=o6_irq_burst_test +ntb_random_seed=$s"; done
    for ((s = 1; s <= $3; s++)); do cp $R/elf/irq_tmr $R/elf/irqt_s$s; echo "$R/elf/irqt_s$s +UVM_TESTNAME=o6_tmr_test +ntb_random_seed=$s"; done; } |
    xargs -P $J -L1 bash -c 'one "$1" 900 "${@:2}"' _ > $R/results_$4.txt
  summ $4
}
summ() {   # 一个阶段的汇总：全过 = tb PASS 且 UVM PASS 且 0 断言
  local f=$R/results_$1.txt
  echo "[$1] 全过 $(grep -c '| === PASS.*| === UVM PASS === | 断言 0$' $f) / $(wc -l < $f)  (tb PASS $(grep -c '| === PASS' $f)，UVM PASS $(grep -c 'UVM PASS' $f)，有断言违例 $(grep -vc '| 断言 0$' $f))"
}
export -f summ
isrv() { [ "$(head -c 4 $1 2>/dev/null | od -An -c | tr -d ' ')" = "177ELF" ]; }

[ -n "$NOBUILD" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1

for st in $STAGES; do
  case $st in
  isa)
    ls $ROOT/work/tests/isa/rv64* | grep -v amocas | grep -v 'v-ma_data' | while read f; do isrv $f && echo $f; done | runlist isa 900 ;;
  dir)
    # 定向测试；irq_ext 在这里不注入中断 (base 测试)
    gen_dir | runlist dir 900 ;;
  vt)
    for f in $ROOT/work/tests/vt/*; do isrv $f && echo $f; done | runlist vt 900 ;;
  rand)
    # 随机整数 / 浮点 / CSR / 分页程序 (rvgen；种子 8100 / 8200 / 8300 / 8400)
    # vm：S 态 + Sv39 随机页表 + 随机 PMP (rvgen --vm，种子 8400)
    { gen_rand int 8100 ${NRAND:-30}; gen_rand fp 8200 ${NRAND:-30}; gen_rand csr 8300 ${NRAND:-30}; gen_rand vm 8400 ${NRAND:-30}; } | runlist rand 600 +max_cycles=2000000 ;;
  vrand)
    # 随机向量程序 (vgen 全部类别)；最后 10 个混入会出异常的访存
    { gen_vrand 9100 $(( ${NRAND:-30} - 10 )); gen_vrand $(( 9100 + ${NRAND:-30} - 10 )) 10 --traps; } | runlist vrand 900 +max_cycles=3000000 ;;
  bench)
    ls $ROOT/work/tests/bench/*.riscv | runlist bench 3600 +max_cycles=200000000 ;;
  irq)
    # 随机外部中断注入：irq_ext (o6_irq_test) + CSR 密集的 irq_csr (o6_irq_burst_test 连发 / 短间隔，压 #28 类窗口)
    run_irq ${NIRQ:-20} ${NIRQC:-10} ${NIRQT:-10} irq
    for f in $ROOT/work/vcs/runs_uvm/irq_s*/sim.log $ROOT/work/vcs/runs_uvm/irqc_s*/sim.log $ROOT/work/vcs/runs_uvm/irqt_s*/sim.log; do grep -a -o "陷入 [0-9]* 次 (中断 [0-9]*" $f | tr '
' ' '; done; echo ;;
  smoke)
    # R0 冒烟：每次 RTL 修改后跑 (种子固定，结果可与上次直接对比)
    { ls $ROOT/work/tests/isa/rv64* | grep -v amocas | grep -v ma_data | while read f; do isrv $f && echo $f; done | awk 'NR % 8 == 1'
      gen_dir; gen_rand int 8100 2; gen_rand fp 8200 2; gen_rand csr 8300 2; gen_vrand 9100 2
      ls $ROOT/work/tests/bench/{median,towers}.riscv; } | runlist smoke 900 +max_cycles=5000000
    run_irq 2 0 1 smoke_irq ;;
  arch)
    # riscv-arch-test：每个 ELF 既与 Spike 逐条对拍，又按 Sail 算出的期望值自检 (失败写 tohost = 3、UART 打 RVCP-SUMMARY)
    if [ -d $ROOT/work/act/o6-rv64gc/elfs ]; then
      find $ROOT/work/act/o6-rv64gc/elfs -name '*.elf' | sort | runlist arch 900 +max_cycles=50000000
    else echo "[arch] 没有 ELF：先跑 verif/act/act_build.sh"; fi ;;
  soc)
    # SoC 模式：ISA 每 2 取 1 (含 -v)、随机各 8、PLIC 外部中断 (o6_irq_test)、JTAG 调试自检、MBIST 自检、AXI 乱序 / 反压 / 长延迟
    ( export SOC=1
      [ -n "$NOBUILD_SOC" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1
      riscv64-unknown-elf-gcc -march=rv64gc_zbb_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles -static \
        -T $ROOT/work/rand/r.ld $ROOT/verif/soc/irq_plic.S -o $R/elf/irq_plic || exit 1
      riscv64-unknown-elf-gcc -march=rv64gc_zbb_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles -static         -T $ROOT/work/rand/r.ld $ROOT/verif/soc/l2_evict.S -o $R/elf/l2_evict || exit 1
      cp $R/elf/l2_evict $R/elf/l2_evict_lat
      riscv64-unknown-elf-gcc -march=rv64gc_zbb_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles -static         -T $ROOT/work/rand/r.ld $ROOT/verif/soc/clint_soc.S -o $R/elf/clint_soc || exit 1
      riscv64-unknown-elf-gcc -march=rv64gc_zicsr_zifencei -mabi=lp64d -nostdlib -nostartfiles -static \
        -T $ROOT/work/rand/r.ld $ROOT/verif/dbg/dbg_loop.S -o $R/elf/dbg_loop || exit 1
      cp $R/elf/dbg_loop $R/elf/soc_dbg; cp $R/elf/dbg_loop $R/elf/soc_mbist
      { ls $ROOT/work/tests/isa/rv64* | grep -v amocas | grep -v ma_data | while read f; do isrv $f && echo $f; done | awk 'NR % 2 == 1'
        for k in int fp csr vm; do ls $R/elf/u${k}8[1-4]0[0-7] 2>/dev/null; done
        ls $ROOT/work/tests/bench/{dhrystone,memcpy,qsort}.riscv
        for s in 1 2 3 4 5 6 7 8; do cp $R/elf/irq_plic $R/elf/psoc_s$s; echo "$R/elf/psoc_s$s +UVM_TESTNAME=o6_irq_test +ntb_random_seed=$s"; done
        echo "$R/elf/soc_dbg +dbgtest +nocosim"; echo "$R/elf/soc_mbist +mbisttest +nocosim"
        echo "$R/elf/l2_evict"; echo "$R/elf/l2_evict_lat +axilat=80 +axibp=20"     # L2 换出脏行 (S_VRD / S_FVRD)
        echo "$R/elf/clint_soc"                                                    # 真实 CLINT 的 MTI / MSI / mtime
        for v in "ooo +axiooo=1" "bp +axibp=30" "lat +axilat=80" "bpooo +axibp=20 +axiooo=1"; do
          set -- $v; for s in 8100 8101 8200 8201; do e=$R/elf/u$( [ $s -lt 8200 ] && echo int || echo fp )$s
            cp $e $R/elf/axi_$1_$s; echo "$R/elf/axi_$1_$s ${@:2}"; done; done; } |
        xargs -P $J -L1 bash -c 'one "$1" 1800 +max_cycles=20000000 "${@:2}"' _ > $R/results_soc.txt
      summ soc ) ;;
  midrst)
    # 运行中复位 (tb_top +midrst=N)：core 13 个 + SoC 8 个，复位点落在缺失 / SQ 满 / AXI 在途 / L2 换出等时刻；
    #   复位后对拍库重载 ELF、Spike 重来，不确定的架构寄存器以 DUT 为准 (cosim_set_reg)。单独编 SIMTAG=mr (不收覆盖率)
    ( export SIMTAG=mr COV=
      [ -n "$NOBUILD_MR" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1
      E=$R/elf
      for t in "uint8100 3000" "uint8100 5000" "uint8100 7000" "uint8100 9000" "uint8100 10500" "ufp8200 4000" "uvm8400 8000"                "ucsr8300 6000" "st_ld_mshr 1500" "sq_flush 2500" "irq_csr 3000"; do
        set -- $t; cp $E/$1 $E/mr_$1_$2; echo "$E/mr_$1_$2 +midrst=$2"; done > $R/mr_list.txt
      cp $ROOT/work/tests/bench/dhrystone.riscv $E/mr_dhry; echo "$E/mr_dhry +midrst=50000" >> $R/mr_list.txt
      cp $ROOT/work/tests/isa/rv64ui-v-add $E/mr_vadd; echo "$E/mr_vadd +midrst=2000" >> $R/mr_list.txt
      xargs -P $J -L1 bash -c 'one "$1" 900 +max_cycles=50000000 "${@:2}"' _ < $R/mr_list.txt > $R/results_midrst.txt
      export SOC=1
      [ -n "$NOBUILD_MR" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1
      for t in "l2_evict 3000" "l2_evict_lat 6000 +axilat=80 +axibp=20" "uint8101 4000 +axiooo=1" "ufp8201 3000 +axibp=30"                "uvm8401 5000" "clint_soc 2500"; do
        set -- $t; cp $E/$1 $E/ms_$1_$2; echo "$E/ms_$1_$2 +midrst=$2 ${@:3}"; done |
        sed 's/[[:space:]]*$//' > $R/ms_list.txt     # xargs -L1 把行尾带空格的行与下一行接成一条 (曾漏跑 3 个、2 个混进别人的参数)
      cp $ROOT/work/tests/bench/dhrystone.riscv $E/ms_dhry; echo "$E/ms_dhry +midrst=40000" >> $R/ms_list.txt
      cp $ROOT/work/tests/isa/rv64ua-v-lrsc $E/ms_lrsc; echo "$E/ms_lrsc +midrst=3000" >> $R/ms_list.txt
      xargs -P $J -L1 bash -c 'one "$1" 1200 +max_cycles=50000000 "${@:2}"' _ < $R/ms_list.txt >> $R/results_midrst.txt
      summ midrst ) ;;
  c1)
    # 配置 C1 (小队列)：资源满 / 指针回绕 / 标签耗尽频繁出现 (docs/13 §6)；单独编译，只看对错不收覆盖率
    ( export SIMTAG=c1 COV= VDEFS="+define+O6_ROB_ROWS=8 +define+O6_LQ_SIZE=16 +define+O6_SQ_SIZE=8 +define+O6_IQ_AB=12 +define+O6_IQ_M=12 +define+O6_IQ_C=12 +define+O6_IQ_SD=8 +define+O6_MAX_BR=8 +define+O6_FTQ_SIZE=8"
      [ -n "$NOBUILD_C1" ] || bash $ROOT/scripts/run_vcs.sh build | tail -1
      { gen_dir; gen_rand int 8100 10; gen_rand fp 8200 10; gen_rand csr 8300 10; gen_vrand 9100 10
        ls $ROOT/work/tests/bench/*.riscv
        ls $ROOT/work/tests/isa/rv64* | grep -v amocas | grep -v ma_data | while read f; do isrv $f && echo $f; done | awk 'NR % 4 == 1'; } |
        runlist c1 3600 +max_cycles=200000000 ) ;;
  report)
    V=$ROOT/work/vcs/simv_uvm.vdb
    # SoC 模式的覆盖率 (若有) 一并合并：模块定义级合并 (实例层次不同)，功能覆盖率按 covergroup 类型合并
    [ -d $ROOT/work/vcs/simv_soc_uvm.vdb ] && V="$V $ROOT/work/vcs/simv_soc_uvm.vdb"
    rm -rf $R/cov_text $R/cov_html
    urg -dir $V -report $R/cov_text -format text -metric line+cond+fsm+branch+tgl+group > $R/urg_text.log 2>&1
    urg -dir $V -report $R/cov_html -metric line+cond+fsm+branch+tgl+group > $R/urg_html.log 2>&1
    sed -n '1,30p' $R/cov_text/dashboard.txt
    # 必达 bin (urg 合并后判；V1 未中 = 回归失败)
    PYTHONIOENCODING=utf-8 python3 $ROOT/scripts/musthit.py $R/cov_text $ROOT/scripts/musthit.txt > $R/musthit.txt
    grep -E '^(未中|缺项)|^MUSTHIT' $R/musthit.txt ;;
  esac
done
echo UVM_REGRESS_DONE
