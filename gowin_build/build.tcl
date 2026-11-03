# build.tcl — gw_sh 命令行构建（2026-10-17 整合版：classic_test 稳定起摆/平衡 + takeoff kick_swing 自动起摆）
# 用法: gw_sh.exe build.tcl   （输出 impl/pnr/j280_top.fs）
set path [file dirname [info script]]

add_file -type verilog "$path/../src/top.v"
add_file -type verilog "$path/../src/adc.v"
add_file -type verilog "$path/../src/encoder.v"
add_file -type verilog "$path/../src/key_filter.v"
add_file -type verilog "$path/../src/motor_ctrl.v"
add_file -type verilog "$path/../src/pid.v"
add_file -type verilog "$path/../src/swing_up.v"
add_file -type verilog "$path/../src/swing_up_energy.v"
add_file -type verilog "$path/../src/kick_swing.v"
add_file -type verilog "$path/../src/motor_drive.v"
add_file -type verilog "$path/../src/uart.v"
add_file -type verilog "$path/../src/uart_tx.v"
add_file -type verilog "$path/../src/vel_est.v"
add_file -type verilog "$path/../src/vel_est_arm.v"
add_file -type verilog "$path/../src/energy_swing.v"
add_file -type verilog "$path/../src/keypad.v"
add_file -type verilog "$path/../src/kb_input.v"
add_file -type verilog "$path/../src/oled_ssd1306.v"
add_file -type cst "$path/../src/fo_rip_top.cst"
add_file -type sdc "$path/../src/j280.sdc"

set_device -name GW2A-55C GW2A-LV55PG484C8/I7
set_option -top_module top
set_option -output_base_name j280_top
# SSPI 脚开放为普通 IO（SW4=key_sw[3] 在 T20=DIN/CLKHOLD_N，与 device.cfg 同款设置）
set_option -use_sspi_as_gpio 1
run all
