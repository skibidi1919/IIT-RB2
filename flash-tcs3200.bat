@echo off
setlocal EnableExtensions
set "CLI=%LOCALAPPDATA%\arduino-cli\arduino-cli.exe"
set "FQBN=esp32:esp32:nodemcu-32s"
set "SKETCH=%~dp0tcs3200_nodemcu"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM4"

if not exist "%CLI%" (
  echo ERROR: arduino-cli not found
  exit /b 1
)

echo Compiling TCS3200 ESP32 firmware...
"%CLI%" compile --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 exit /b 1

echo Uploading to %PORT% ...
"%CLI%" upload -p "%PORT%" --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 (
  echo Upload failed. Hold BOOT on ESP32 if needed, check COM port.
  echo Usage: flash-tcs3200.bat COMx
  exit /b 1
)
echo Done. Serial monitor @ 115200.
endlocal
