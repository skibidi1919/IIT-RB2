# `arm_ui` USB text protocol

Baud **115200**, newline-terminated ASCII.

## Host → Nano

| Cmd | Meaning |
|-----|---------|
| `A,b,h,g` | Set arm base/height/grip (0–180) |
| `B,d` / `H,d` / `G,d` | Single axis |
| `C` | Center arm 90,90,90 |
| `D,left,right` | Drive −255..255 |
| `S` | Stop motors |
| `M` | Blocking motor self-test |
| `Z` | Zero encoders + VL53 displacement origin |
| `?` | Force one telemetry line |

## Nano → Host

### Telemetry (`T,…` @ ~5 Hz)

```
T,tof,cmdL,cmdR,base,height,grip,pca,tofOk,encL,encR,wheelLmm,wheelRmm,tofDisp,imu,yawC,pitchC,rollC
```

| Field | Unit |
|-------|------|
| tof | mm (0 = invalid) |
| cmdL/R | commanded PWM |
| pca/tofOk/imu | 0/1 |
| encL/R | quadrature steps |
| wheelL/R mm | odometry from 43 mm wheel |
| tofDisp | mm moved toward/away vs `Z` origin (origin − now) |
| yaw/pitch/roll | centidegrees (÷100 → degrees) |

### Other lines

`READY pca=… tof=… imu=…`, `OK …`, `MOTOR TEST …`, `ERR …`
