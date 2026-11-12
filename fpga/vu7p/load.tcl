# 通过 JTAG-to-AXI 装载程序并放核运行 (Vivado Hardware Manager 中 source)
# 用法 (Vivado Tcl 控制台，已 open_hw_target 并下载比特流)：
#   source load.tcl
#   o6_load <bin 文件> [装载地址，默认 0x80000000]
#   o6_run            ;# 释放核复位
#   o6_halt           ;# 重新保持复位
#   o6_status
# bin 文件用 riscv64-unknown-elf-objcopy -O binary 从 ELF 生成 (链接地址 0x8000_0000)。

set O6_CTRL 0xF0000000

proc o6_axi {} { return [lindex [get_hw_axis] 0] }

proc o6_wr32 {addr val} {
  set a [o6_axi]
  catch {delete_hw_axi_txn [get_hw_axi_txns o6w]}
  create_hw_axi_txn o6w $a -type write -address [format %08x $addr] -data [format %08x $val] -len 1
  run_hw_axi -quiet [get_hw_axi_txns o6w]
}

proc o6_rd32 {addr} {
  set a [o6_axi]
  catch {delete_hw_axi_txn [get_hw_axi_txns o6r]}
  create_hw_axi_txn o6r $a -type read -address [format %08x $addr] -len 1
  run_hw_axi -quiet [get_hw_axi_txns o6r]
  return 0x[get_property DATA [get_hw_axi_txns o6r]]
}

proc o6_halt {} { global O6_CTRL; o6_wr32 $O6_CTRL 1 }
proc o6_run  {} { global O6_CTRL; o6_wr32 $O6_CTRL 0 }
proc o6_status {} {
  global O6_CTRL
  puts "status=[o6_rd32 [expr {$O6_CTRL + 4}]] cycle=[o6_rd32 [expr {$O6_CTRL + 8}]]"
  o6_wr32 [expr {$O6_CTRL + 12}] 0x5a5aa5a5
  puts "scratch=[o6_rd32 [expr {$O6_CTRL + 12}]] (应为 0x5a5aa5a5)"
}

# 256 拍突发 (1 KB) 写入；JTAG-AXI 的 -data 要求高地址字在前
proc o6_load {file {base 0x80000000}} {
  o6_halt
  set fh [open $file rb]
  set bin [read $fh]
  close $fh
  set n [string length $bin]
  set pad [expr {(4 - $n % 4) % 4}]
  append bin [string repeat "\x00" $pad]
  set nw [expr {[string length $bin] / 4}]
  binary scan $bin iu* words
  set a [o6_axi]
  set t0 [clock milliseconds]
  for {set i 0} {$i < $nw} {incr i 256} {
    set len [expr {min(256, $nw - $i)}]
    set data ""
    for {set k [expr {$i + $len - 1}]} {$k >= $i} {incr k -1} { append data [format %08x [lindex $words $k]] }
    catch {delete_hw_axi_txn [get_hw_axi_txns o6b]}
    create_hw_axi_txn o6b $a -type write -address [format %08x [expr {$base + 4 * $i}]] -data $data -len $len -burst INCR
    run_hw_axi -quiet [get_hw_axi_txns o6b]
  }
  puts "loaded $n bytes @ [format 0x%08x $base] in [expr {[clock milliseconds] - $t0}] ms"
  # 校验首尾字
  puts "first=[o6_rd32 $base] last=[o6_rd32 [expr {$base + 4 * ($nw - 1)}]]"
}
