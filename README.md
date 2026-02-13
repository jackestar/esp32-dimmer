<h3 align="center">ESP32 Based Dimmer</h3>

<div align="center">
<img src="./web/favicon.svg" width="96">

</div>

Simple ESP32-based dimmer project (firmware + PCB + web UI). This repo contains the ESP-IDF firmware, KiCad design files, and a small web UI used by the device.

## Quick overview
- **Firmware:** under `idf/esp32-dimmer` — ESP-IDF project with `CMakeLists.txt`, `sdkconfig`, and `main/esp32-dimmer.c`.
- **KiCad PCB:** `kicad/dimmer/` — schematic and PCB files for the dimmer hardware.
- **Web UI:** `web/` — `index.html`, `style.css`, `sw.js` and `manifest.json` for a small web interface (served or bundled into firmware as needed).

## Build & flash (ESP-IDF)
1. Install and set up the ESP-IDF environment as documented by Espressif.
2. From `idf/esp32-dimmer` run:

```bash
idf.py set-target esp32
idf.py build
idf.py flash
```

Adjust `sdkconfig` or run `idf.py menuconfig` to change board-specific settings.

# Diagram

![Diagram](./Diagram.svg)