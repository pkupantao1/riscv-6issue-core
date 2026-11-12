# JasperGold：F2 o6_pmp 与规范参考模型等价 (verif/formal/fv_pmp.sv)
set ROOT $::env(O6_ROOT)
analyze -sv12 $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv $ROOT/rtl/mmu/o6_pmp.sv $ROOT/verif/formal/fv_pmp.sv
elaborate -top fv_pmp
clock clk
reset -none
set_prove_time_limit 1800s
prove -all
report -summary -force -file fv_pmp.rpt
set fo [open fv_pmp.props w]
foreach p [get_property_list -include {type {assert}}] { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
foreach p [get_property_list -include {type {cover}}]  { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
close $fo
exit
