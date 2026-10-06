# Regenerate nanopb C + Python from proto/meowler.proto
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$py = Join-Path $root "gui\.venv\Scripts\python.exe"
$gen = Join-Path $root "third_party\nanopb\generator\nanopb_generator.py"
$protoDir = Join-Path $root "proto"
$outPb = Join-Path $root "generated\nanopb"

New-Item -ItemType Directory -Force -Path $outPb | Out-Null
Copy-Item (Join-Path $root "third_party\nanopb\pb.h"),
  (Join-Path $root "third_party\nanopb\pb_common.h"),
  (Join-Path $root "third_party\nanopb\pb_common.c"),
  (Join-Path $root "third_party\nanopb\pb_encode.h"),
  (Join-Path $root "third_party\nanopb\pb_encode.c"),
  (Join-Path $root "third_party\nanopb\pb_decode.h"),
  (Join-Path $root "third_party\nanopb\pb_decode.c") -Destination $outPb -Force

$frameSrc = Join-Path $root "color_bridge\meow_frame.h"
if (Test-Path $frameSrc) {
  $txt = Get-Content $frameSrc -Raw
  $txt = $txt -replace 'MEOW_FRAME_MAX_PAYLOAD = \d+', 'MEOW_FRAME_MAX_PAYLOAD = 120'
  Set-Content (Join-Path $outPb "meow_frame.h") $txt
}

Push-Location $protoDir
& $py $gen -I $protoDir -D $outPb meowler.proto
Pop-Location
# Arduino AVR needs quoted include for sketch-local pb.h
$pbH = Join-Path $outPb "meowler.pb.h"
(Get-Content $pbH -Raw) -replace '#include <pb.h>', '#include "pb.h"' | Set-Content $pbH -NoNewline

$sketches = @("color_bridge", "hub_bridge", "nano_drive", "robot_arm")
$files = @(
  "meowler.pb.h", "meowler.pb.c", "pb.h", "pb_common.h", "pb_common.c",
  "pb_encode.h", "pb_encode.c", "pb_decode.h", "pb_decode.c", "meow_frame.h"
)
foreach ($s in $sketches) {
  $dst = Join-Path $root $s
  New-Item -ItemType Directory -Force -Path $dst | Out-Null
  foreach ($f in $files) {
    $src = Join-Path $outPb $f
    if (Test-Path $src) { Copy-Item $src (Join-Path $dst $f) -Force }
  }
}

& $py -m grpc_tools.protoc -I $protoDir --python_out=(Join-Path $root "gui\meowler_pb") (Join-Path $protoDir "meowler.proto")
New-Item -ItemType Directory -Force -Path (Join-Path $root "robot_link\meowler_pb") | Out-Null
Copy-Item (Join-Path $root "gui\meowler_pb\meowler_pb2.py") (Join-Path $root "robot_link\meowler_pb\meowler_pb2.py") -Force
if (-not (Test-Path (Join-Path $root "robot_link\meowler_pb\__init__.py"))) {
  Set-Content (Join-Path $root "robot_link\meowler_pb\__init__.py") ""
}
Write-Host "OK: nanopb + Python regenerated"
