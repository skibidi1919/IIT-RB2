# Keep Mobile Hotspot up and nudge Meowler panel sticky reconnect.
# Usage: powershell -ExecutionPolicy Bypass -File tools\link-keep.ps1
$ErrorActionPreference = "Continue"
$ssid = "DarshIshaan"
$pass = "Darsh@3001"
$robot = "192.168.137.222"
$port = 3333
$panel = "http://127.0.0.1:5050"
$configured = $false

Add-Type -AssemblyName System.Runtime.WindowsRuntime | Out-Null

function Get-Tether {
  $conn = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
  if (-not $conn) { return $null }
  return [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($conn)
}

function Ensure-Hotspot {
  param([ref]$Configured)
  $tm = Get-Tether
  if (-not $tm) { Write-Host "$(Get-Date -Format HH:mm:ss) no internet profile for ICS"; return $false }
  $state = $tm.TetheringOperationalState.ToString()
  if (-not $Configured.Value) {
    try {
      $cfg = $tm.GetCurrentAccessPointConfiguration()
      $cfg.Ssid = $ssid
      $cfg.Passphrase = $pass
      $cfg.Band = [Windows.Networking.NetworkOperators.TetheringWiFiBand]::TwoPointFourGigahertz
      $op = $tm.ConfigureAccessPointAsync($cfg)
      while ($op.Status -eq [Windows.Foundation.AsyncStatus]::Started) { Start-Sleep -Milliseconds 40 }
      $Configured.Value = $true
      Write-Host "$(Get-Date -Format HH:mm:ss) hotspot configured 2.4GHz $ssid"
    } catch {
      Write-Host "$(Get-Date -Format HH:mm:ss) configure skipped: $($_.Exception.Message)"
      $Configured.Value = $true
    }
  }
  if ($state -ne "On") {
    Write-Host "$(Get-Date -Format HH:mm:ss) starting hotspot (was $state)..."
    try {
      $op2 = $tm.StartTetheringAsync()
      while ($op2.Status -eq [Windows.Foundation.AsyncStatus]::Started) { Start-Sleep -Milliseconds 40 }
      Start-Sleep -Seconds 2
    } catch {
      Write-Host "$(Get-Date -Format HH:mm:ss) start failed: $($_.Exception.Message)"
      return $false
    }
  }
  $tm2 = Get-Tether
  if (-not $tm2) { return $false }
  Write-Host ("{0} hotspot={1} clients={2}" -f (Get-Date -Format HH:mm:ss), $tm2.TetheringOperationalState, $tm2.ClientCount)
  return ($tm2.TetheringOperationalState.ToString() -eq "On")
}

function Test-RobotTcp {
  try {
    $c = New-Object System.Net.Sockets.TcpClient
    $iar = $c.BeginConnect($robot, $port, $null, $null)
    $ok = $iar.AsyncWaitHandle.WaitOne(900) -and $c.Connected
    $c.Close()
    return $ok
  } catch { return $false }
}

function Nudge-Panel {
  param([bool]$RobotUp)
  try {
    $st = Invoke-RestMethod -Uri "$panel/api/state" -TimeoutSec 2
  } catch { return }
  if ($st.connected) { return }
  if (-not $RobotUp) { return }
  # Only nudge if panel already wants a sticky link (or was connected before)
  if (-not $st.sticky_link -and -not $st.auto_reconnect -and -not $st.host) { return }
  try {
    $body = @{ host = $robot; port = $port } | ConvertTo-Json
    Invoke-RestMethod -Method Post -Uri "$panel/api/reconnect" -ContentType "application/json" -Body $body -TimeoutSec 12 | Out-Null
    Write-Host "$(Get-Date -Format HH:mm:ss) panel reconnect nudged"
  } catch {
    Write-Host "$(Get-Date -Format HH:mm:ss) panel nudge failed: $($_.Exception.Message)"
  }
}

Write-Host "Meowler link-keep: hotspot=$ssid robot=${robot}:${port} panel=$panel (Ctrl+C to stop)"
$cfgRef = [ref]$configured
while ($true) {
  $hot = Ensure-Hotspot -Configured $cfgRef
  $up = $false
  if ($hot) { $up = Test-RobotTcp }
  Write-Host ("{0} robot_tcp={1}" -f (Get-Date -Format HH:mm:ss), $up)
  if ($up) { Nudge-Panel -RobotUp $true }
  Start-Sleep -Seconds 8
}
