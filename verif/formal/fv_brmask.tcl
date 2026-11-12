# JasperGold：F5 发射队列 br_mask 广播后清零 (verif/formal/fv_brmask.sv)
set ROOT $::env(O6_ROOT)
set RTL  $::env(O6_FV_RTL)
analyze -sv12  {*}$::env(O6_FV_DEFS) $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv $RTL/rtl/common/o6_prio_enc.sv $RTL/rtl/backend/o6_iq.sv $ROOT/verif/formal/fv_brmask.sv
elaborate -top fv_brmask
clock clk
reset -expression !rst_n
set_prove_time_limit 2400s
prove -all
report -summary -force -file fv_brmask.rpt
set fo [open fv_brmask.props w]
foreach p [get_property_list -include {type {assert}}] { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
foreach p [get_property_list -include {type {cover}}]  { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
close $fo
exit
