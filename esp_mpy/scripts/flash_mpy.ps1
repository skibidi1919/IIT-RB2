# Flash official MicroPython SPIRAM_OCT firmware to Freenove ESP32-S3 N16R8
# Usage:
#   .\flash_mpy.ps1              # auto-detect / prompt COM
#   .\flash_mpy.ps1 -Port COM5
#   .\flash_mpy.ps1 -Port COM5 -SkipErase
#   .\flash_mpy.ps1 -Port COM5 -FirmwarePath .\firmware\custom.bin

param(
    [string]$Port = "",
    [string]$FirmwarePath = "",
    [switch]$SkipErase,
    [int]$Baud = 460800
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$FwDir = Join-Path $Root "firmware"
$DefaultName = "ESP32_GENERIC_S3-SPIRAM_OCT-20260824-v1.29.0.bin"
$DefaultUrl = "https://micropython.org/resources/firmware/$DefaultName"

if (-not (Get-Command esptool -ErrorAction SilentlyContinue) -and
    -not (Get-Command esptool.py -ErrorAction SilentlyContinue)) {
    Write-Host "Installing host deps from requirements-host.txt ..."
    python -m pip install -r (Join-Path $Root "requirements-host.txt")
}

$Esptool = if (Get-Command esptool -ErrorAction SilentlyContinue) { "esptool" } else { "esptool.py" }

if (-not $Port) {
    Write-Host "Available serial ports:"
    [System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Host "  $_" }
    $Port = Read-Host "COM port (e.g. COM5)"
}
if (-not $Port) { throw "Port required" }

if (-not $FirmwarePath) {
    New-Item -ItemType Directory -Force -Path $FwDir | Out-Null
    $FirmwarePath = Join-Path $FwDir $DefaultName
    if (-not (Test-Path $FirmwarePath)) {
        Write-Host "Downloading $DefaultUrl ..."
        Invoke-WebRequest -Uri $DefaultUrl -OutFile $FirmwarePath
    }
}

Write-Host "Firmware: $FirmwarePath"
Write-Host "Port:     $Port"

if (-not $SkipErase) {
    Write-Host "Erasing flash ..."
    & $Esptool --chip esp32s3 --port $Port erase_flash
}

Write-Host "Writing firmware @ 0x0 ..."
& $Esptool --chip esp32s3 --port $Port --baud $Baud write_flash -z 0x0 $FirmwarePath

Write-Host "Done. Next: .\deploy_app.ps1 -Port $Port"
Write-Host "Copy secrets: copy app\secrets.py.example → device secrets.py (or edit before deploy)."
