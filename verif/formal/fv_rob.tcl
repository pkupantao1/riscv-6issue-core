# JasperGold：F6 ROB 头尾 / 有效位不变量 (4 行) (verif/formal/fv_rob.sv)
set ROOT $::env(O6_ROOT)
set RTL  $::env(O6_FV_RTL)
analyze -sv12 +define+O6_ROB_ROWS=4 {*}$::env(O6_FV_DEFS) $ROOT/rtl/pkg/o6_isa_pkg.sv $ROOT/rtl/pkg/o6_pkg.sv $RTL/rtl/backend/o6_rob.sv $ROOT/verif/formal/fv_rob.sv
elaborate -top fv_rob
clock clk
reset -expression !rst_n
# 两阶段：_assert_3 (最老异常：全对比较 == 锦标赛树) 与其余性质同跑时 5400 s 仍只到 bound 28–30；
# 先证其余性质，再把它们当引理单独攻这一条 (2026-10-31：724 s proven)
set_prove_time_limit 900s
prove -all
set A3 {<embedded>::fv_rob.dut._assert_3}
set_prove_time_limit 5400s
if {[get_status $A3] ne "proven"} { prove -property $A3 -with_proven }
report -summary -force -file fv_rob.rpt
set fo [open fv_rob.props w]
foreach p [get_property_list -include {type {assert}}] { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
foreach p [get_property_list -include {type {cover}}]  { puts $fo "$p [get_status $p] [get_property_info $p -list min_length]" }
close $fo
exit
