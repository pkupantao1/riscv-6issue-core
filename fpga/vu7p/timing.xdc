# 与板卡无关的时序约束 (板卡时钟 create_clock 在 board.xdc 里)
# MMCM 输出时钟由 Vivado 自动派生。

# 异步复位输入与 LED/串口不做时序
set_false_path -from [get_ports sys_rst]
set_false_path -to   [get_ports {led[*]}]
set_false_path -from [get_ports uart_rx]
set_false_path -to   [get_ports uart_tx]

# RISC-V JTAG：TCK 最高 10 MHz，与核时钟异步 (DTM 内部是翻转握手 + 两级同步)
create_clock -period 100.0 -name rv_tck [get_ports rv_tck]
set_clock_groups -asynchronous -group [get_clocks rv_tck] -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *u_mmcm/CLKOUT0}]]
set_input_delay  -clock rv_tck 20.0 [get_ports {rv_tms rv_tdi}]
set_output_delay -clock rv_tck -clock_fall 20.0 [get_ports rv_tdo]

# 复位同步器第一级
set_false_path -to [get_pins -hier -filter {NAME =~ *u_rs_*/*/CLR}]
set_false_path -to [get_pins -hier -filter {NAME =~ *u_rs_*/*/PRE}]
