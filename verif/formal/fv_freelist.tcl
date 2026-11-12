# JasperGold：F3 空闲表不重复分配 (verif/formal/fv_freelist.sv)；O6_FV_DEFS 可带 +define 做变异实验
set ROOT $::env(O6_ROOT)
analyze -sv12 {*}$::env(O6_FV_DEFS) $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv $::env(O6_FV_RTL)/rtl/backend/o6_freelist.sv $ROOT/verif/formal/fv_freelist.sv
elaborate -top fv_freelist
clock clk
reset -expression !rst_n
set_prove_time_limit 1800s
prove -all
# 时限内未定的 (每次不同的 g_deq[k]，负载相关) 再用已证性质作引理单独攻
foreach p [get_property_list -include {type {assert}}] {
  if {[get_status $p] eq "undetermined"} { prove -property $p -with_proven }
}
report -summary -force -file fv_freelist.rpt
# 每条性质的结论 (反例给出长度) -> fv_freelist.props
set fo [open fv_freelist.props w]
foreach p [get_property_list -include {type {assert}}] {
  puts $fo "$p [get_status $p] [get_property_info $p -list min_length]"
}
close $fo
exit
