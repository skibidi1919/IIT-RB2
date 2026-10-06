# Setup

## Tools

- `arduino-cli` (`%LOCALAPPDATA%\arduino-cli`)
- Core: `arduino:avr`
- Libraries: **VL53L0X**, **7Semi BNO08x**
- Python via **`uv`**

## Flash Nano (old bootloader)

```powershell
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
arduino-cli upload -p COM3 --fqbn arduino:avr:nano:cpu=atmega328old arm_ui
```

Close anything holding COM3 (control panel, serial monitor) before upload.

## Run control panel

```powershell
cd arm_ui
uv run --with flask --with pyserial python app.py
# http://127.0.0.1:5050 → Connect COM3
```

## Wiring checklist (drive)

1. Remove **ENA** and **ENB** jumpers on L298N  
2. PCA CH4 → ENA, CH5 → ENB  
3. PCA CH6..9 → IN1..IN4  
4. PCA OE → GND  
5. Common GND Nano / PCA / L298N  
6. Battery on L298N VM  

## Sensors

- Encoders on D3–D6 (optional; odometry stays 0 if unwired)  
- BNO on soft-I2C D7/D8 (optional)  
- VL53 on Wire A4/A5  

## Archive

Old probe sketches live under `archive/diag/`. Older Flask apps under `archive/host/`.
