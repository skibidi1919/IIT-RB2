@echo off
setlocal EnableExtensions
set "CLI=%LOCALAPPDATA%\arduino-cli\arduino-cli.exe"
set "FQBN=esp32:esp32:nodemcu-32s:UploadSpeed=115200"
set "SKETCH=%~dp0color_bridge"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM4"

if not exist "%CLI%" (
  echo ERROR: arduino-cli not found
  exit /b 1
)

echo Compiling color_bridge...
"%CLI%" compile --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 exit /b 1

echo Uploading to %PORT% ...
"%CLI%" upload -p "%PORT%" --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 exit /b 1
echo OK
