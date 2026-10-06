@echo off
setlocal EnableExtensions
set "CLI=%LOCALAPPDATA%\arduino-cli\arduino-cli.exe"
set "FQBN=arduino:avr:nano:cpu=atmega328old"
set "SKETCH=%~dp0n20_motor_test"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM3"

if not exist "%CLI%" (
  echo ERROR: arduino-cli not found
  exit /b 1
)

echo Compiling motor firmware (old bootloader)...
"%CLI%" compile --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 exit /b 1

echo Uploading to %PORT%...
"%CLI%" upload -p "%PORT%" --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 (
  echo Upload failed. Close motor GUI / free COM, then retry.
  exit /b 1
)
echo Done. Run motor-gui.bat
endlocal
