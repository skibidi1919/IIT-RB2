# Diagnose / revive ESP32-S3 + CH343 USB serial for Meowler flash.
# Run elevated for ghost cleanup + driver reinstall.
$ErrorActionPreference = "Continue"

Write-Host "=== SERIALCOMM ==="
reg query HKLM\HARDWARE\DEVICEMAP\SERIALCOMM 2>&1

Write-Host "`n=== Live USB serial (Espressif / CH343) ==="
$live = Get-PnpDevice -PresentOnly | Where-Object {
  $_.InstanceId -match 'VID_303A|VID_1A86&PID_55D3|VID_1A86&PID_5523'
}
if (-not $live) {
  Write-Host "NONE present. Windows is not seeing the board USB serial interface."
  Write-Host "Use a data cable, Freenove USB (native) or UART port, try BOOT held on plug-in."
} else {
  $live | Format-Table Status, Class, FriendlyName, InstanceId -AutoSize
}

Write-Host "`n=== Ghost (not present) serial nodes ==="
$ghost = Get-PnpDevice | Where-Object {
  -not $_.Present -and ($_.InstanceId -match 'VID_303A|VID_1A86&PID_55D3' -or ($_.Class -eq 'Ports' -and $_.FriendlyName -match 'CH343|CP210|USB Serial'))
}
$ghost | Format-Table Status, FriendlyName -AutoSize

$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
  [Security.Principal.WindowsBuiltInRole]::Administrator)
if ($isAdmin -and $ghost) {
  Write-Host "`nRemoving ghost devices..."
  foreach ($d in $ghost) {
    pnputil /remove-device "$($d.InstanceId)" 2>&1 | Out-Host
  }
  pnputil /scan-devices 2>&1 | Out-Host
}

if ($isAdmin -and $live) {
  Write-Host "`nReinstalling drivers on live devices..."
  foreach ($d in $live) {
    pnputil /restart-device "$($d.InstanceId)" 2>&1 | Out-Host
  }
}

Write-Host "`n=== arduino-cli board list ==="
& "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe" board list 2>&1

Write-Host "`nIf still no COM: flash over WiFi -> ota-flash.bat 192.168.137.222"
