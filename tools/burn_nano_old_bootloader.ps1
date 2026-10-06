# Burn Arduino Nano OLD bootloader (ATmegaBOOT @ 57600) via ISP, then flash i2c_probe.
#
# REQUIRES a real ISP programmer — serial/COM3 UART cannot rewrite this chip.
# Easiest: second Arduino as ISP
#
# Wiring (Arduino-as-ISP programmer → target Nano):
#   D10 → RESET
#   D11 → D11 (MOSI)
#   D12 → D12 (MISO)
#   D13 → D13 (SCK)
#   5V  → 5V
#   GND → GND
#
# Steps:
#   1) On PROGRAMMER board: upload Examples → ArduinoISP
#   2) Wire as above to TARGET Nano (the stuck one)
#   3) Plug PROGRAMMER USB → note its COM port
#   4) Run:
#        powershell -File tools\burn_nano_old_bootloader.ps1 -ProgrammerPort COM5
#      (use the programmer's COM port, not the target)

param(
  [Parameter(Mandatory = $true)]
  [string]$ProgrammerPort,

  [string]$Programmer = "arduinoasisp",

  [switch]$SkipProbeUpload
)

$ErrorActionPreference = "Stop"

$cli = "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe"
if (-not (Test-Path $cli)) { $cli = "arduino-cli" }

$fqbn = "arduino:avr:nano:cpu=atmega328old"
$probe = Join-Path $PSScriptRoot "..\i2c_probe"

Write-Host "==> Burning OLD Nano bootloader via $Programmer on $ProgrammerPort"
& $cli burn-bootloader -b $fqbn -p $ProgrammerPort -P $Programmer -t -v
if ($LASTEXITCODE -ne 0) { throw "burn-bootloader failed" }

if (-not $SkipProbeUpload) {
  Write-Host "==> Uploading i2c_probe to target (old bootloader @ 57600)"
  Write-Host "    Connect TARGET Nano USB/serial (COM3 usually), then:"
  Write-Host "    arduino-cli upload -p COM3 --fqbn $fqbn -t `"$probe`""
  Write-Host "==> Or re-run after plugging target USB:"
  Write-Host "    arduino-cli compile --fqbn $fqbn `"$probe`""
  Write-Host "    arduino-cli upload -p COM3 --fqbn $fqbn -t `"$probe`""
}

Write-Host "Done. Open Serial Monitor 115200 — expect MEOWLER I2C PROBE"
