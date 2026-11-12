# OOO6 VU7P 原型：非工程模式全流程 (IP 生成 -> 综合 -> 布局布线 -> 比特流)
# 用法: vivado -mode batch -source build.tcl -tclargs [xdc 文件] [stop_after]
#   xdc 默认 ../board/board.xdc (板卡资料到位前用 board_template.xdc 只能跑到综合)
#   stop_after = synth | place | route | bit (默认 bit)
set root [file normalize [file dirname [info script]]/../..]
set xdc  [expr {[llength $argv] > 0 ? [lindex $argv 0] : "$root/fpga/board/board.xdc"}]
set stop [expr {[llength $argv] > 1 ? [lindex $argv 1] : "bit"}]
set part xcvu7p-flvb2104-2-e
set od   [pwd]/out_vu7p
file mkdir $od
set_param general.maxThreads 8

create_project -in_memory -part $part
set_property target_language Verilog [current_project]

# ---- IP：JTAG-to-AXI Master (AXI4，32 位)
file mkdir $od/ip
create_ip -name jtag_axi -vendor xilinx.com -library ip -module_name jtag_axi_0 -dir $od/ip
set_property -dict [list CONFIG.PROTOCOL {0} CONFIG.M_AXI_DATA_WIDTH {32} CONFIG.M_AXI_ADDR_WIDTH {32} \
                         CONFIG.M_AXI_ID_WIDTH {1}] [get_ips jtag_axi_0]
generate_target all [get_ips jtag_axi_0]
synth_ip [get_ips jtag_axi_0]

# ---- RTL
set fh [open $root/rtl/fpu/cvfpu.f r]
foreach l [split [read $fh] "\n"] { if {$l ne ""} { read_verilog -sv $root/$l } }
close $fh
read_verilog -sv [list $root/rtl/pkg/o6_isa_pkg.sv $root/rtl/pkg/o6_pkg.sv]
foreach f [lsort [glob $root/rtl/*/*.sv]] {
  if {[string match "*/pkg/*" $f] || [string match "*/asic/*" $f]} continue
  read_verilog -sv $f
}
read_verilog -sv [glob $root/fpga/rtl/*.sv]
read_xdc $xdc
read_xdc $root/fpga/vu7p/timing.xdc

synth_design -top o6_fpga_top -part $part \
  -include_dirs [list $root/rtl/soc $root/third_party/cvfpu/src/common_cells/include] \
  -verilog_define O6_FPGA -flatten_hierarchy rebuilt -directive PerformanceOptimized
write_checkpoint -force $od/post_synth.dcp
report_utilization -hierarchical -hierarchical_depth 4 -file $od/util_synth_hier.rpt
report_utilization -file $od/util_synth.rpt
report_timing_summary -max_paths 20 -file $od/timing_synth.rpt
if {$stop eq "synth"} { puts "BUILD_DONE synth"; exit }

opt_design -directive Explore
place_design -directive ExtraTimingOpt
phys_opt_design -directive AggressiveExplore
write_checkpoint -force $od/post_place.dcp
report_timing_summary -max_paths 20 -file $od/timing_place.rpt
if {$stop eq "place"} { puts "BUILD_DONE place"; exit }

route_design -directive Explore
phys_opt_design -directive AggressiveExplore
write_checkpoint -force $od/post_route.dcp
report_timing_summary -max_paths 50 -file $od/timing_route.rpt
report_utilization -file $od/util_route.rpt
report_drc -file $od/drc.rpt
report_methodology -file $od/methodology.rpt
if {$stop eq "route"} { puts "BUILD_DONE route"; exit }

write_bitstream -force $od/o6_fpga_top.bit
write_debug_probes -force $od/o6_fpga_top.ltx
puts "BUILD_DONE bit"
