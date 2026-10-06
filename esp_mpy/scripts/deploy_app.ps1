# Deploy esp_mpy/app/*.py to the MicroPython device with mpremote
# Usage:
#   .\deploy_app.ps1
#   .\deploy_app.ps1 -Port COM5
#   .\deploy_app.ps1 -Port COM5 -WithSecrets

param(
    [string]$Port = "",
    [switch]$WithSecrets,
    [switch]$Reset
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$App = Join-Path $Root "app"

if (-not (Get-Command mpremote -ErrorAction SilentlyContinue)) {
    python -m pip install -r (Join-Path $Root "requirements-host.txt")
}

$Dev = if ($Port) { $Port } else { "auto" }

Write-Host "Deploying $App -> device ($Dev) ..."
$files = @(
    "boot.py", "main.py", "pinout.py", "pca9685.py", "vl53.py",
    "drive.py", "color_tcs.py", "protocol.py", "ota_http.py"
)

foreach ($f in $files) {
    $src = Join-Path $App $f
    if (-not (Test-Path $src)) { throw "Missing $src" }
    Write-Host "  $f"
    mpremote connect $Dev cp $src ":$f"
}

$example = Join-Path $App "secrets.py.example"
$secretsLocal = Join-Path $App "secrets.py"
if ($WithSecrets -and (Test-Path $secretsLocal)) {
    Write-Host "  secrets.py"
    mpremote connect $Dev cp $secretsLocal ":secrets.py"
} elseif (Test-Path $example) {
    mpremote connect $Dev cp $example ":secrets.py.example"
    Write-Host "Tip: create app\secrets.py then re-run with -WithSecrets"
}

if ($Reset) {
    mpremote connect $Dev reset
} else {
    Write-Host "Soft reboot ..."
    mpremote connect $Dev exec "import machine; machine.reset()"
}

Write-Host "Done. REPL: mpremote connect $Dev repl"
