@echo off
setlocal EnableExtensions
rem Protobuf OTA: stream raw .bin into OTA1, reboot into it.
rem Usage: ota-flash.bat [IP] [path\to\app.bin]
rem Default IP 192.168.137.222. If no bin, compiles esp_ui (usually you pass a sketch bin).

set "IP=%~1"
if "%IP%"=="" set "IP=192.168.137.222"
set "BIN=%~2"

if "%BIN%"=="" (
  py -3 "%~dp0arm_ui\ota_upload.py" --host %IP% --compile
) else (
  py -3 "%~dp0arm_ui\ota_upload.py" --host %IP% --bin "%BIN%"
)
exit /b %ERRORLEVEL%
