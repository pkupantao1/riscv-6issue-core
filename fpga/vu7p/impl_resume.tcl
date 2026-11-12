# 从最新检查点续跑实现 (本机若有别的流程会结束 Vivado 进程，分阶段落盘、断了就从断点继续)
# 用法 (在 build.tcl 的输出目录的上一级运行)：vivado -mode batch -source impl_resume.tcl [-tclargs xdc]
set root [file normalize [file dirname [info script]]/../..]
set od   [pwd]/out_vu7p
set_param general.maxThreads 8

proc have {f} { return [file exists $f] }

if {[have $od/post_route.dcp]} {
  open_checkpoint $od/post_route.dcp
} elseif {[have $od/post_place.dcp]} {
  open_checkpoint $od/post_place.dcp
  route_design -directive Explore
  write_checkpoint -force $od/post_route0.dcp
  phys_opt_design -directive AggressiveExplore
  write_checkpoint -force $od/post_route.dcp
} elseif {[have $od/post_opt.dcp]} {
  open_checkpoint $od/post_opt.dcp
  place_design -directive ExtraTimingOpt
  phys_opt_design -directive AggressiveExplore
  write_checkpoint -force $od/post_place.dcp
  route_design -directive Explore
  phys_opt_design -directive AggressiveExplore
  write_checkpoint -force $od/post_route.dcp
} elseif {[have $od/post_synth.dcp]} {
  open_checkpoint $od/post_synth.dcp
  opt_design -directive Explore
  write_checkpoint -force $od/post_opt.dcp
  place_design -directive ExtraTimingOpt
  phys_opt_design -directive AggressiveExplore
  write_checkpoint -force $od/post_place.dcp
  route_design -directive Explore
  phys_opt_design -directive AggressiveExplore
  write_checkpoint -force $od/post_route.dcp
} else {
  puts "IMPL_FAIL 没有 post_synth.dcp，先跑 build.tcl"
  exit 1
}

report_timing_summary -max_paths 50 -file $od/timing_route.rpt
report_utilization -file $od/util_route.rpt
report_drc -file $od/drc.rpt
report_methodology -file $od/methodology.rpt
report_route_status -file $od/route_status.rpt
puts "BUILD_DONE route"
