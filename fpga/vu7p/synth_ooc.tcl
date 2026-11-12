# 模块级 out-of-context 综合 (VU7P)，用于检查可综合性与资源/时序粗估
# 用法: vivado -mode batch -source synth_ooc.tcl -tclargs <top> [clk_ns]
set top [lindex $argv 0]
set clk [expr {[llength $argv] > 1 ? [lindex $argv 1] : 10.0}]
set root [file normalize [file dirname [info script]]/../..]
set part xcvu7p-flvb2104-2-e
# 额外宏：环境变量 O6_DEFS (空格分隔)，例如 O6_DEFS=O6_NO_VEC (不搭载 RVV)
set defs [expr {[info exists ::env(O6_DEFS)] ? $::env(O6_DEFS) : ""}]
set fh [open $root/rtl/fpu/cvfpu.f r]
foreach l [split [read $fh] "
"] { if {$l ne ""} { read_verilog -sv $root/$l } }
close $fh
read_verilog -sv [list $root/rtl/pkg/o6_isa_pkg.sv $root/rtl/pkg/o6_pkg.sv]
foreach f [lsort [glob -nocomplain $root/rtl/*/*.sv]] {
  if {[string match "*/pkg/*" $f] || [string match "*/asic/*" $f]} continue
  read_verilog -sv $f
}
set_param general.maxThreads 8
synth_design -top $top -part $part -include_dirs [list $root/rtl/soc $root/third_party/cvfpu/src/common_cells/include] -mode out_of_context -flatten_hierarchy rebuilt -verilog_define [concat O6_FPGA $defs]
create_clock -period $clk [get_ports clk_i]
set od ooc_$top[expr {$defs ne "" ? "_[string map {{ } _} $defs]" : ""}]
file mkdir $od
report_utilization -file $od/util.rpt
report_timing_summary -max_paths 5 -file $od/timing.rpt
# 200 条不同端点的最差路径 (fpga/vu7p/path_summary.py 按模块汇总)
report_timing -max_paths 200 -nworst 1 -unique_pins -file $od/paths.rpt
write_checkpoint -force $od/post_synth.dcp
puts "OOC_DONE $top"
