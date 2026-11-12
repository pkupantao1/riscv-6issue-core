# JasperGold：F4 D$ 的 MSHR 无重复行 (verif/formal/fv_mshr.sv)
#   标签 / 有效 / 脏 / PLRU 阵列 stopat (放宽：每拍任意值)，数据 SRAM 黑盒；去重逻辑与路数、MSHR 数无关，缩成 2 路 / 4 个 MSHR (15 个时 40 分钟只到 5 拍)
set ROOT $::env(O6_ROOT)
set RTL  $::env(O6_FV_RTL)
analyze -sv12 +define+O6_L1_NWAY=2 {*}$::env(O6_FV_DEFS) $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv \
  $RTL/rtl/common/o6_mbist.sv $RTL/rtl/common/o6_sram_sp.sv $RTL/rtl/cache/o6_dcache.sv $ROOT/verif/formal/fv_mshr.sv
elaborate -top fv_mshr -bbox_m o6_sram_sp -parameter NMSHR 4
clock clk
reset -expression !rst_n
stopat dut.tag dut.valid dut.dirty dut.plru
set_prove_time_limit 2700s
prove -all
report -summary -force -file fv_mshr.rpt
set fo [open fv_mshr.props w]
foreach p [get_property_list -include {type {assert}}] { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
foreach p [get_property_list -include {type {cover}}]  { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
close $fo
exit
