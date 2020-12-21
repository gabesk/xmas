@ECHO OFF
setlocal enabledelayedexpansion

REM 
REM This script automates the synthesis, place and route, and binpack steps of
REM the IceStorm toolchain on windows assuming install via apio.
REM See https://github.com/FPGAwars/apio and 
REM     https://tinyfpga.com/bx/guide.html for install instructions.
REM 
REM In the local directory, there should be pins.pcf with the I/O pins of FPGA.
REM Then, call like:
REM 
REM     make_hdl.bat top_module file1.v file2.v file3.v
REM 

SET ICESTORM_TOOLCHAIN=%HOMEDRIVE%\%HOMEPATH%\.apio\packages\toolchain-icestorm\bin

IF NOT EXIST %ICESTORM_TOOLCHAIN% GOTO NOT_FOUND_ERROR

ECHO ###############################################################################
ECHO yosys -p "synth_ice40 -blif hardware.blif -top %1" -q -l synth_log.txt %2 %3 %4
ECHO ###############################################################################
ECHO.
%ICESTORM_TOOLCHAIN%\yosys -p "synth_ice40 -blif hardware.blif -top %1" -q -l synth_log.txt %2 %3 %4
IF %ERRORLEVEL% NEQ 0 GOTO CMD_ERROR

SET IN_STATS="0"
FOR /F "delims=" %%l IN (synth_log.txt) DO (
    IF "%%l"=="2.27. Printing statistics." (
        SET IN_STATS="1"
    )
    
    IF !IN_STATS!=="1" ECHO %%l
)

ECHO.
ECHO ###############################################################################
ECHO arachne-pnr -d 1k -P vq100 -p pins.pcf -o hardware.asc hardware.blif
ECHO ###############################################################################
ECHO.
%ICESTORM_TOOLCHAIN%\arachne-pnr -d 1k -P vq100 -p pins.pcf -o hardware.asc hardware.blif
IF %ERRORLEVEL% NEQ 0 GOTO CMD_ERROR

ECHO.
ECHO ###############################################################################
ECHO icepack hardware.asc hardware.bin
ECHO ###############################################################################
ECHO.
%ICESTORM_TOOLCHAIN%\icepack hardware.asc hardware.bin
IF %ERRORLEVEL% NEQ 0 GOTO CMD_ERROR

ECHO.
ECHO ###############################################################################
ECHO SUCCESS
ECHO ###############################################################################

spi_ctrl2.py com3 hardware.bin

GOTO END

:NOT_FOUND_ERROR
ECHO IceStorm toolchain not found in apio packages list.
ECHO See https://tinyfpga.com/bx/guide.html for install instructions.
GOTO END

:CMD_ERROR
ECHO Command failed.
GOTO END

:END