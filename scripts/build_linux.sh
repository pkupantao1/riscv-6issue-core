#!/bin/bash
# OOO6 Linux 镜像：BusyBox (静态) → initramfs → Linux Image → 设备树 → OpenSBI fw_payload.elf
#   scripts/build_linux.sh [all|busybox|kernel|sbi]     (默认 all)
#   源码：scripts/server_setup.sh linux_src 下载到 /export/master/src/sw
#   工具链：riscv64-unknown-linux-gnu (server_setup.sh gnu_linux)；OpenSBI 用 elf 工具链
#   产物 (work/linux/out)：
#     fw_payload_fpga.elf  75 MHz、mtime 1 MHz、16 MB，console=ttyS0 115200，起到交互 shell
#     fw_payload_sim.elf   仿真：UART 时钟 = 115200×16 (分频 1)，带 HTIF (poweroff → tohost=1 结束仿真)，
#                          命令行带 o6sim (/init 打印自检后关机)
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SW=/export/master/src/sw
WK=$ROOT/work/linux
OUT=$WK/out
TC=/export/master/rvtools/bin
LINUX_VER=6.18.56
OPENSBI_VER=1.9
BUSYBOX_VER=1.38.0
JOBS=${JOBS:-4}
mkdir -p $WK $OUT

# 主机编译器用 conda 的 gcc 12 (系统 gcc 4.8 编不了新内核的主机工具)
source /home/master/miniconda3/etc/profile.d/conda.sh
conda activate /export/master/conda/rv
unset CC CXX CFLAGS CXXFLAGS CPPFLAGS LDFLAGS DEBUG_CFLAGS DEBUG_CXXFLAGS LD_LIBRARY_PATH
export PATH=$TC:$PATH
HOSTCC=x86_64-conda-linux-gnu-gcc
# 内核主机工具 (modpost) 要较新的 elf.h (R_ARM_CALL 等)，conda 编译器带的是 glibc 2.17 的旧版：
# 强制先包含交叉 glibc 的 elf.h (与体系结构无关)，旧的那份因同一头文件保护宏被跳过
# 强制包含会先拉进 features.h，源文件自己的 #define _GNU_SOURCE 来不及生效 (gen_init_cpio 缺 O_LARGEFILE)：全局给上
KHOSTCFLAGS="-D_GNU_SOURCE -include /export/master/rvtools/sysroot/usr/include/elf.h -include $ROOT/sw/linux/hostcompat.h"
XL=riscv64-unknown-linux-gnu-
XE=riscv64-unknown-elf-

step_busybox() {
  local B=$WK/busybox
  [ -d $B ] || cp -a $SW/busybox-$BUSYBOX_VER $B
  cd $B || exit 1
  make HOSTCC=$HOSTCC ARCH=riscv CROSS_COMPILE=$XL defconfig > /dev/null || exit 1
  # 静态链接；tc 用到新内核头里已删除的 CBQ 定义，关掉
  sed -i -e 's/^# CONFIG_STATIC is not set/CONFIG_STATIC=y/' -e 's/^CONFIG_TC=y/# CONFIG_TC is not set/' .config
  make HOSTCC=$HOSTCC ARCH=riscv CROSS_COMPILE=$XL oldconfig < /dev/null > /dev/null || exit 1
  make HOSTCC=$HOSTCC ARCH=riscv CROSS_COMPILE=$XL -j$JOBS > $WK/busybox.log 2>&1 || { tail -30 $WK/busybox.log; exit 1; }
  rm -rf $WK/rootfs
  make HOSTCC=$HOSTCC ARCH=riscv CROSS_COMPILE=$XL CONFIG_PREFIX=$WK/rootfs install > /dev/null || exit 1
  mkdir -p $WK/rootfs/{proc,sys,dev,tmp,etc,root}
  install -m 0755 $ROOT/sw/linux/init $WK/rootfs/init
  ls -la $WK/rootfs/bin/busybox
}

step_kernel() {
  local K=$WK/linux-build
  mkdir -p $K
  sed "s#@INITRAMFS@#$WK/rootfs $ROOT/sw/linux/initramfs.list#" $ROOT/sw/linux/o6_kernel.config > $WK/o6_kernel.config
  cd $SW/linux-$LINUX_VER || exit 1
  make O=$K HOSTCC=$HOSTCC HOSTCFLAGS="$KHOSTCFLAGS" ARCH=riscv CROSS_COMPILE=$XL tinyconfig > /dev/null || exit 1
  ARCH=riscv scripts/kconfig/merge_config.sh -m -O $K $K/.config $WK/o6_kernel.config > $WK/merge_config.log 2>&1 || exit 1
  make O=$K HOSTCC=$HOSTCC HOSTCFLAGS="$KHOSTCFLAGS" ARCH=riscv CROSS_COMPILE=$XL olddefconfig > /dev/null || exit 1
  # 片段里要求的选项若被依赖关系否掉，列出来 (不致命，但要看)
  grep -E '^CONFIG_|^# CONFIG_' $WK/o6_kernel.config | while read -r l; do
    grep -qxF "$l" $K/.config || echo "  [kconfig] 未生效: $l"
  done
  make O=$K HOSTCC=$HOSTCC HOSTCFLAGS="$KHOSTCFLAGS" ARCH=riscv CROSS_COMPILE=$XL -j$JOBS Image > $WK/kernel.log 2>&1 || { tail -40 $WK/kernel.log; exit 1; }
  cp $K/arch/riscv/boot/Image $OUT/Image
  ls -la $OUT/Image
}

mkdtb() {   # $1=名字 $2=核时钟 $3=UART 时钟 $4=时基 $5=内存字节 $6=bootargs $7=是否加 HTIF
  local d=$OUT/o6_$1.dts
  sed -e "s#@TB@#$4#" -e "s#@MEMSZ@#$5#" -e "s#@BOOTARGS@#$6#" $ROOT/sw/linux/o6.dts.in |
    awk -v uclk="$3" -v cclk="$2" '
      /serial@10000000 \{/ {inser=1}
      { if (inser && /@CLK@/) { sub(/@CLK@/, uclk) } else { sub(/@CLK@/, cclk) } print }
      inser && /^\t\t\};/ {inser=0}' > $d
  if [ "$7" = 1 ]; then
    # HTIF：只用来关机 (OpenSBI 的 tohost 变量；TB 看到 tohost=1 判结束)
    sed -i 's#^\tsoc {#\thtif {\n\t\tcompatible = "ucb,htif0";\n\t};\n\n\tsoc {#' $d
  fi
  dtc -I dts -O dtb -o $OUT/o6_$1.dtb $d || exit 1
}

step_sbi() {
  [ -f $OUT/Image ] || { echo "先编内核"; exit 1; }
  # FPGA：75 MHz，mtime 1 MHz (TICK_DIV 75)，16 MB URAM
  mkdtb fpga 75000000 75000000 1000000 0x1000000 "console=ttyS0,115200 earlycon=sbi" 0
  # 仿真：tb_top 的 TICK_DIV 10，核时钟名义 100 MHz；UART 时钟取 115200×16 使分频为 1
  mkdtb sim 100000000 1843200 10000000 0x1000000 "console=ttyS0,115200 earlycon=sbi o6sim lpj=100000" 1
  # 评估用：同仿真版，另跑 /init 里的 BusyBox 负载 (Spike 生成分支 trace)
  mkdtb bench 100000000 1843200 10000000 0x1000000 "console=ttyS0,115200 earlycon=sbi o6sim o6bench lpj=100000" 1
  local S=$WK/opensbi
  [ -d $S ] || cp -a $SW/opensbi-$OPENSBI_VER $S
  for v in fpga sim bench; do
    # OpenSBI 1.x 要求链接成 PIE：裸机 elf 工具链做不到，用 linux-gnu 工具链
    make -C $S O=$WK/sbi_$v CROSS_COMPILE=$XL PLATFORM=generic FW_TEXT_START=0x80000000 \
         FW_PAYLOAD=y FW_PAYLOAD_PATH=$OUT/Image FW_FDT_PATH=$OUT/o6_$v.dtb -j$JOBS > $WK/sbi_$v.log 2>&1 \
      || { tail -30 $WK/sbi_$v.log; exit 1; }
    cp $WK/sbi_$v/platform/generic/firmware/fw_payload.elf $OUT/fw_payload_$v.elf
    cp $WK/sbi_$v/platform/generic/firmware/fw_payload.bin $OUT/fw_payload_$v.bin
  done
  ls -la $OUT
}

case "${1:-all}" in
  busybox) step_busybox ;;
  kernel)  step_kernel ;;
  sbi)     step_sbi ;;
  all)     step_busybox; step_kernel; step_sbi ;;
  *) echo "usage: $0 [all|busybox|kernel|sbi]"; exit 1 ;;
esac
echo "== build_linux $1 done =="
