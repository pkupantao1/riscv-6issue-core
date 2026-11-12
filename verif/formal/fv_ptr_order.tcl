# JasperGold：F1 环形队列指针先后比较 (verif/formal/fv_ptr_order.sv)
set ROOT $::env(O6_ROOT)
analyze -sv12 $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv $ROOT/verif/formal/fv_ptr_order.sv
elaborate -top fv_ptr_order
clock clk
reset -none
prove -all
report -summary -force -file fv_ptr_order.rpt
set fo [open fv_ptr_order.props w]
foreach p [get_property_list -include {type {assert}}] { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
foreach p [get_property_list -include {type {cover}}]  { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
close $fo
exit
