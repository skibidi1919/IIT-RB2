@echo off
setlocal EnableExtensions
rem USB flash updater into OTA0. Fast path: esptool 2Mbaud + compress (no full erase).
rem Usage: flash-esp.bat [COMx]
rem Skip rebuild: flash-esp.bat COM7 nocompile

set "CLI=%LOCALAPPDATA%\arduino-cli\arduino-cli.exe"
set "ESPTOOL=%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe"
set "FQBN=esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600"
set "SKETCH=%~dp0esp_ui"
set "BUILD=%~dp0build\esp_ui"
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM7"
set "SKIP=%~2"
set "PARTDIR=%LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\3.3.12\tools\partitions"
copy /Y "%SKETCH%\partitions.csv" "%PARTDIR%\ota8m_16MB.csv" >nul

if /I not "%SKIP%"=="nocompile" (
  "%CLI%" compile --fqbn "%FQBN%" "%SKETCH%" --output-dir "%BUILD%" --build-property build.partitions=ota8m_16MB --build-property upload.maximum_size=8323072
  if errorlevel 1 exit /b 1
)

set "APP=%BUILD%\esp_ui.ino.bin"
set "BL=%BUILD%\esp_ui.ino.bootloader.bin"
set "PART=%BUILD%\esp_ui.ino.partitions.bin"
set "BAPP0=%PARTDIR%\boot_app0.bin"
if not exist "%APP%" (
  echo Missing %APP% — compile first.
  exit /b 1
)
if not exist "%BL%" (
  echo Missing bootloader bin in %BUILD%
  exit /b 1
)

echo Fast USB flash @ 2000000 baud --compress on %PORT%
"%ESPTOOL%" --chip esp32s3 --port %PORT% --baud 2000000 --before default-reset --after hard-reset write-flash -z --flash-mode dio --flash-freq 80m --flash-size 16MB 0x0 "%BL%" 0x8000 "%PART%" 0xe000 "%BAPP0%" 0x10000 "%APP%"
if errorlevel 1 (
  echo 2Mbaud failed — retry 1500000
  "%ESPTOOL%" --chip esp32s3 --port %PORT% --baud 1500000 --before default-reset --after hard-reset write-flash -z --flash-mode dio --flash-freq 80m --flash-size 16MB 0x0 "%BL%" 0x8000 "%PART%" 0xe000 "%BAPP0%" 0x10000 "%APP%"
)
if errorlevel 1 exit /b 1
echo USB flash OK — updater on OTA0. Then: ota-flash.bat 192.168.137.222
endlocal
