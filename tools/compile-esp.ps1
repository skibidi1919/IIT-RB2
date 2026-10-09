# Fast incremental esp_ui compile (fixed cache + all cores).
# Output: esp_ui/build/esp_ui.ino.bin
$ErrorActionPreference = "Stop"
$Root = Split-Path $PSScriptRoot -Parent
$Cli = Join-Path $env:LOCALAPPDATA "arduino-cli\arduino-cli.exe"
$Sketch = Join-Path $Root "esp_ui"
$Out = Join-Path $Sketch "build"
$Cache = Join-Path $Out ".cache"
$Fqbn = "esp32:esp32:esp32s3:CDCOnBoot=default,USBMode=hwcdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600"

New-Item -ItemType Directory -Force -Path $Out, $Cache | Out-Null
$sw = [Diagnostics.Stopwatch]::StartNew()
& $Cli compile `
  --fqbn $Fqbn `
  -j 0 `
  --build-path $Cache `
  --output-dir $Out `
  --build-property "build.partitions=ota8m_16MB" `
  --build-property "upload.maximum_size=8323072" `
  $Sketch
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$sw.Stop()
$bin = Join-Path $Out "esp_ui.ino.bin"
Write-Host ("OK {0:N1}s -> {1} ({2} bytes)" -f $sw.Elapsed.TotalSeconds, $bin, (Get-Item $bin).Length)
