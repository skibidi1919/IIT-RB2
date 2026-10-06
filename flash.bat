@echo off
setlocal EnableExtensions

rem Meowler Nano flash helper (old ATmega328 bootloader)
set "CLI=%LOCALAPPDATA%\arduino-cli\arduino-cli.exe"
set "FQBN=arduino:avr:nano:cpu=atmega328old"
set "SKETCH=%~dp0robot_arm"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM3"

if not exist "%CLI%" (
  echo ERROR: arduino-cli not found at "%CLI%"
  exit /b 1
)

echo Compiling "%SKETCH%" ...
"%CLI%" compile --fqbn "%FQBN%" "%SKETCH%" --build-property "compiler.c.extra_flags=-DPB_NO_ERRMSG=1 -DPB_BUFFER_ONLY=1" --build-property "compiler.cpp.extra_flags=-DPB_NO_ERRMSG=1 -DPB_BUFFER_ONLY=1"
if errorlevel 1 (
  echo Compile failed.
  exit /b 1
)

echo Uploading to %PORT% ...
"%CLI%" upload -p "%PORT%" --fqbn "%FQBN%" "%SKETCH%"
if errorlevel 1 (
  echo Upload failed. Close the GUI / free the COM port, then retry.
  echo Usage: flash.bat [COMx]
  exit /b 1
)

echo Done.
endlocal
