# 🐍 IIT-RB2 Python Subsystem & Telemetry Pipeline (`my_project`)

High-performance Python companion toolchain for the **Autonomous 4WD N20 Rover (IIT-RB2)**, powered by [Astral `uv`](https://docs.astral.sh/uv/).

---

## ⚡ Quick Start

### 1. Run Diagnostic Integrity Scan
```powershell
uv run my-project
```

### 2. Add New Python Dependencies
```powershell
uv add opencv-python numpy pyserial
```

### 3. Run Scripts with Automatic Virtual Environment Management
```powershell
uv run script.py
```

---

## 📁 Package Architecture

```text
my_project/
├── pyproject.toml              # Project metadata, dependencies & scripts
├── uv.lock                     # Deterministic, ultra-fast dependency lockfile
├── .python-version             # Managed CPython runtime (3.12)
├── README.md                   # This documentation guide
└── src/
    └── my_project/
        └── __init__.py         # Diagnostic scan & telemetry entry point
```

---

## 🚀 Capabilities & Future Extensions

* **Serial Communication & Telemetry:** Connect directly to Arduino Nano via `pyserial` on COM ports at 115200 Baud.
* **Computer Vision & Perception:** OpenCV / PyTorch integration for lane following, obstacle segmentation, and ArUco visual fiducials.
* **Autonomous Navigation:** Real-time state estimation and path planning companion.
