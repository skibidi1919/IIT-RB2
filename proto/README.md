# Meowler protobuf TCP

Schema: `meowler.proto`

## Wire format

ESP32 opens **TCP port 3333** (mDNS `meowler.local`). Client connects.

Every message (both directions):

```text
uint32 little-endian length
protobuf payload (that many bytes)
```

- Client → robot: `meowler.ClientToRobot`
- Robot → client: `meowler.RobotToClient` (`hello` on connect, `telem` ~5 Hz, `ack` after cmds, `log` for debug)
- **USB Serial** uses the same framing for `RobotToClient.log` (read with `uv run serial-log -p COM7`)

## Regenerate

```powershell
# Python
uv run --with grpcio-tools python -m grpc_tools.protoc -I proto --python_out=arm_ui/meowler_pb proto/meowler.proto

# nanopb (needs protoc on PATH)
python -m nanopb.generator -I proto -D esp_ui/src proto/meowler.proto
# then copy meowler.pb.c/h into esp_ui/src/
```
