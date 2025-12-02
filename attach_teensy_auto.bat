@echo off
setlocal enabledelayedexpansion

echo ========================================
echo  Auto-Detecting and Attaching Teensy
echo ========================================

REM Teensy VID:PID is always 16c0:0483
set TEENSY_VID_PID=16c0:0483

echo Searching for Teensy (VID:PID = %TEENSY_VID_PID%)...
echo.

REM Use PowerShell to find BUSID by VID:PID
for /f "tokens=*" %%i in ('powershell -Command "usbipd list | Select-String '%TEENSY_VID_PID%' | ForEach-Object { ($_ -split '\s+')[0] }"') do (
    set BUSID=%%i
)

if not defined BUSID (
    echo ✗ ERROR: Teensy not found
    echo.
    echo Please check:
    echo   1. Teensy is plugged into USB port
    echo   2. Teensy is recognized by Windows Device Manager
    echo   3. Run 'usbipd list' to see all devices
    echo.
    pause
    exit /b 1
)

echo ✓ Found Teensy at BUSID: %BUSID%
echo.

echo Binding USB device...
powershell -Command "usbipd bind --busid %BUSID%"

echo Attaching to WSL...
powershell -Command "usbipd attach --busid %BUSID% --wsl"

if %errorlevel% equ 0 (
    echo.
    echo ========================================
    echo ✓ SUCCESS - Teensy attached to WSL
    echo ========================================
    echo.
    echo BUSID: %BUSID%
    echo Access in WSL: /dev/ttyACM0
    echo.
) else (
    echo.
    echo ✗ ERROR - Failed to attach Teensy
    echo   Make sure this script runs as Administrator
    echo.
)

pause
