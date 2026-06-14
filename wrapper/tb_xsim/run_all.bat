@echo off
REM run_all.bat — compile + run all wrapper testbenches in Vivado xsim
REM Usage: run_all.bat [tb_name]   (omit tb_name to run all)
call E:\Xilinx\Vivado\2024.2\settings64.bat
if errorlevel 1 goto :eof

set RTL=..\wrapper_pkg.sv ..\weight_sram.sv ..\requant_sram.sv ..\shadow_register.sv
set RTL=%RTL% ..\ahb_master.sv ..\dma_scheduler.sv ..\compute_mgmt.sv ..\apb_slave.sv ..\bus_wrapper.sv

set WAVEDIR=waves
if not exist %WAVEDIR% mkdir %WAVEDIR%

if "%1"=="" goto :all
if "%1"=="gearbox" goto :gearbox
if "%1"=="ahb" goto :ahb
if "%1"=="sched" goto :sched
if "%1"=="integ" goto :integ
echo Unknown TB: %1 (try: gearbox, ahb, sched, integ)
goto :eof

:all
call :gearbox
call :ahb
call :sched
call :integ
echo.
echo ==== ALL TBs DONE ====
goto :eof

:gearbox
echo ==== tb_gearbox ====
xvlog --nolog -sv %RTL% tb_gearbox.sv
if errorlevel 1 goto :eof
xelab --nolog -top tb_gearbox -snapshot tb_gearbox_snap
if errorlevel 1 goto :eof
xsim tb_gearbox_snap -R
move /Y tb_gearbox.vcd %WAVEDIR%\ >nul 2>&1
goto :eof

:ahb
echo ==== tb_ahb_master ====
xvlog --nolog -sv %RTL% tb_ahb_master.sv
if errorlevel 1 goto :eof
xelab --nolog -top tb_ahb_master -snapshot tb_ahb_master_snap
if errorlevel 1 goto :eof
xsim tb_ahb_master_snap -R
move /Y tb_ahb_master.vcd %WAVEDIR%\ >nul 2>&1
goto :eof

:sched
echo ==== tb_dma_scheduler ====
xvlog --nolog -sv %RTL% tb_dma_scheduler.sv
if errorlevel 1 goto :eof
xelab --nolog -top tb_dma_scheduler -snapshot tb_dma_sched_snap
if errorlevel 1 goto :eof
xsim tb_dma_sched_snap -R
move /Y tb_dma_scheduler.vcd %WAVEDIR%\ >nul 2>&1
goto :eof

:integ
echo ==== tb_dma_integ ====
xvlog --nolog -sv %RTL% tb_dma_integ.sv
if errorlevel 1 goto :eof
xelab --nolog -top tb_dma_integ -snapshot tb_dma_integ_snap
if errorlevel 1 goto :eof
xsim tb_dma_integ_snap -R
move /Y tb_dma_integ.vcd %WAVEDIR%\ >nul 2>&1
goto :eof
