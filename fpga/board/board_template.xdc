# VU7P 板卡引脚约束模板 —— 引脚号全部是占位符 (XXX)，等板卡原理图/XDC 提供后填入，另存为 board.xdc
# 只用于在没有板卡资料时把综合跑通 (综合不检查引脚)。

# 板载差分系统时钟 (按实际频率改 period 与 o6_fpga_top 的 CLKIN_PERIOD/MMCM 参数)
create_clock -period 3.333 -name sys_clk [get_ports sys_clk_p]
# set_property PACKAGE_PIN XXX [get_ports sys_clk_p]
# set_property PACKAGE_PIN XXX [get_ports sys_clk_n]
# set_property IOSTANDARD DIFF_SSTL12 [get_ports {sys_clk_p sys_clk_n}]

# 复位按键 (极性见 o6_fpga_top 参数 RST_ACT_HIGH)
# set_property PACKAGE_PIN XXX [get_ports sys_rst]
# set_property IOSTANDARD LVCMOS18 [get_ports sys_rst]

# USB-UART
# set_property PACKAGE_PIN XXX [get_ports uart_rx]
# set_property PACKAGE_PIN XXX [get_ports uart_tx]
# set_property IOSTANDARD LVCMOS18 [get_ports {uart_rx uart_tx}]

# RISC-V JTAG (外部适配器，例如 FT2232H / J-Link；任意普通 IO，TCK 不在时钟专用脚上时加 CLOCK_DEDICATED_ROUTE)
# set_property PACKAGE_PIN XXX [get_ports rv_tck]
# set_property PACKAGE_PIN XXX [get_ports rv_tms]
# set_property PACKAGE_PIN XXX [get_ports rv_tdi]
# set_property PACKAGE_PIN XXX [get_ports rv_tdo]
# set_property IOSTANDARD LVCMOS18 [get_ports {rv_tck rv_tms rv_tdi rv_tdo}]
# set_property PULLUP true [get_ports {rv_tms rv_tdi}]
# set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_ports rv_tck]]

# LED
# set_property PACKAGE_PIN XXX [get_ports {led[0]}]
# set_property PACKAGE_PIN XXX [get_ports {led[1]}]
# set_property PACKAGE_PIN XXX [get_ports {led[2]}]
# set_property PACKAGE_PIN XXX [get_ports {led[3]}]
# set_property IOSTANDARD LVCMOS18 [get_ports {led[*]}]

# 配置
# set_property CONFIG_VOLTAGE 1.8 [current_design]
# set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
