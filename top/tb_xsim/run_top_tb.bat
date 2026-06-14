@echo off
REM run_top_tb.bat — compile + run tb_lenet5_top in Vivado xsim
call E:\Xilinx\Vivado\2024.2\settings64.bat
if errorlevel 1 goto :eof

set PKG=..\lenet5_pkg.sv
set WRP=..\..\wrapper\weight_sram.sv ..\..\wrapper\requant_sram.sv ..\..\wrapper\shadow_register.sv
set WRP=%WRP% ..\..\wrapper\ahb_master.sv ..\..\wrapper\dma_scheduler.sv ..\..\wrapper\compute_mgmt.sv
set WRP=%WRP% ..\..\wrapper\apb_slave.sv ..\..\wrapper\bus_wrapper.sv
set CORE=..\..\core\compute_fsm.sv ..\..\core\im_agu.sv ..\..\core\wt_agu.sv
set CORE=%CORE% ..\..\core\pe_array.sv ..\..\core\fifo_gearbox.sv ..\..\core\requant_unit.sv
set CORE=%CORE% ..\..\core\pingpong_pool.sv ..\..\core\corner_turn.sv ..\..\core\im_sram.sv
set CORE=%CORE% ..\..\core\compute_core.sv
set TOP=..\lenet5v3_top.sv

echo ==== tb_lenet5_top ====
echo ==== xvlog (compile) ====
xvlog --nolog -sv %PKG% %WRP% %CORE% %TOP% tb_lenet5_top.sv
if errorlevel 1 (
    echo COMPILE FAILED
    goto :eof
)

echo ==== xelab (elaborate) ====
xelab --nolog -timescale 1ns/100ps -top tb_lenet5_top -snapshot tb_lenet5_top_snap
if errorlevel 1 (
    echo ELAB FAILED
    goto :eof
)

echo ==== xsim (run) ====
xsim tb_lenet5_top_snap -R
if errorlevel 1 (
    echo SIM FAILED
    goto :eof
)
echo ==== DONE ====
