@echo off
REM run_core_tb.bat — compile + run compute_core testbench in Vivado xsim
REM Usage: run_core_tb.bat
call E:\Xilinx\Vivado\2024.2\settings64.bat
if errorlevel 1 goto :eof

set PKG=..\..\top\lenet5_pkg.sv
set CORE=..\compute_fsm.sv ..\im_agu.sv ..\wt_agu.sv ..\pe_array.sv ..\fifo_gearbox.sv
set CORE=%CORE% ..\requant_unit.sv ..\pingpong_pool.sv ..\corner_turn.sv ..\im_sram.sv
set CORE=%CORE% ..\compute_core.sv

set WAVEDIR=waves
if not exist %WAVEDIR% mkdir %WAVEDIR%

echo ==== tb_compute_core ====
xvlog --nolog -sv %PKG% %CORE% tb_compute_core.sv
if errorlevel 1 (
    echo COMPILE FAILED
    goto :eof
)
xelab --nolog -timescale 1ns/100ps -top tb_compute_core -snapshot tb_compute_core_snap
if errorlevel 1 (
    echo ELAB FAILED
    goto :eof
)
xsim tb_compute_core_snap -R
if errorlevel 1 (
    echo SIM FAILED
    goto :eof
)
move /Y tb_compute_core.vcd %WAVEDIR%\ >nul 2>&1
echo ==== DONE ====
