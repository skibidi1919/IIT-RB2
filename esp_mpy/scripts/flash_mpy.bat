@echo off
setlocal
REM Flash MicroPython SPIRAM_OCT to ESP32-S3 N16R8
REM Usage: flash_mpy.bat [COMx]

set PORT=%~1
if "%PORT%"=="" set PORT=COM5

set ROOT=%~dp0..
set FWDIR=%ROOT%\firmware
set FWNAME=ESP32_GENERIC_S3-SPIRAM_OCT-20260824-v1.29.0.bin
set FWURL=https://micropython.org/resources/firmware/%FWNAME%
set FW=%FWDIR%\%FWNAME%

if not exist "%FWDIR%" mkdir "%FWDIR%"
if not exist "%FW%" (
  echo Downloading %FWURL%
  powershell -NoProfile -Command "Invoke-WebRequest -Uri '%FWURL%' -OutFile '%FW%'"
)

where esptool >nul 2>&1
if errorlevel 1 (
  python -m pip install -r "%ROOT%\requirements-host.txt"
)

echo Erase %PORT% ...
esptool --chip esp32s3 --port %PORT% erase_flash
echo Flash %FW% ...
esptool --chip esp32s3 --port %PORT% --baud 460800 write_flash -z 0x0 "%FW%"
echo Done. Run deploy_app.bat %PORT%
endlocal
