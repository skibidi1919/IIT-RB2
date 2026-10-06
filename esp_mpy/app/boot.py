# Runs once at boot before main.py.
# Keep minimal — WiFi/hardware start in main.py.

import gc

gc.collect()
print("meowler esp_mpy boot")
