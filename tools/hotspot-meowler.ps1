# Force PC Mobile Hotspot to DarshIshaan / Darsh@3001 on 2.4 GHz and probe robot :3333
$ErrorActionPreference = "Stop"

$ssid = "DarshIshaan"
$pass = "Darsh@3001"
$robot = "192.168.137.222"
$port = 3333

Add-Type -AssemblyName System.Runtime.WindowsRuntime | Out-Null
$conn = [Windows.Networking.Connectivity.NetworkInformation,Windows.Networking.Connectivity,ContentType=WindowsRuntime]::GetInternetConnectionProfile()
if (-not $conn) { throw "No internet connection profile - plug Ethernet or connect Wi-Fi first (ICS needs a share source)." }
$tm = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager,Windows.Networking.NetworkOperators,ContentType=WindowsRuntime]::CreateFromConnectionProfile($conn)

$cfg = $tm.GetCurrentAccessPointConfiguration()
$cfg.Ssid = $ssid
$cfg.Passphrase = $pass
$cfg.Band = [Windows.Networking.NetworkOperators.TetheringWiFiBand]::TwoPointFourGigahertz
$op = $tm.ConfigureAccessPointAsync($cfg)
while ($op.Status -eq [Windows.Foundation.AsyncStatus]::Started) { Start-Sleep -Milliseconds 50 }
Write-Host "Configure: $($op.Status) band=$($tm.GetCurrentAccessPointConfiguration().Band)"

try {
  $p = "HKLM:\SYSTEM\CurrentControlSet\Services\icssvc\Settings"
  $b = [byte[]](Get-ItemProperty $p).PrivateConnectionSettings
  [BitConverter]::GetBytes([uint32]0).CopyTo($b, $b.Length - 4)
  Set-ItemProperty -Path $p -Name PrivateConnectionSettings -Value $b -Type Binary
  Write-Host "Registry band DWORD forced to 0 (2.4 GHz)"
} catch {
  Write-Host "Registry band write skipped (run as Admin if hotspot stays on 5 GHz): $($_.Exception.Message)"
}

$op1 = $tm.StopTetheringAsync()
while ($op1.Status -eq [Windows.Foundation.AsyncStatus]::Started) { Start-Sleep -Milliseconds 50 }
Start-Sleep -Seconds 1
$op2 = $tm.StartTetheringAsync()
while ($op2.Status -eq [Windows.Foundation.AsyncStatus]::Started) { Start-Sleep -Milliseconds 50 }
Start-Sleep -Seconds 2

Write-Host "Hotspot: $($tm.TetheringOperationalState) clients=$($tm.ClientCount) SSID=$ssid"
Write-Host "Waiting up to 45s for ${robot}:${port} ..."
$deadline = [DateTime]::UtcNow.AddSeconds(45)
$ok = $false
while ([DateTime]::UtcNow -lt $deadline) {
  $clients = $tm.ClientCount
  try {
    $c = New-Object System.Net.Sockets.TcpClient
    $iar = $c.BeginConnect($robot, $port, $null, $null)
    $wait = $iar.AsyncWaitHandle.WaitOne(800)
    if ($wait -and $c.Connected) { $ok = $true; $c.Close(); break }
    $c.Close()
  } catch {}
  Write-Host ("  clients={0} tcp={1}" -f $clients, $ok)
  if ($ok) { break }
  Start-Sleep -Seconds 2
}

if ($ok) {
  Write-Host "OK - robot listening at ${robot}:${port}"
  exit 0
}
Write-Host "FAIL - no listener at ${robot}:${port}. Power ESP, or flash SoftAP build and join Wi-Fi Meowler -> 192.168.4.1"
exit 1
