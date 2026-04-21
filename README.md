# Bike Data Project - Hardware Tracker

Low-power bicycle GPS tracker with BLE sync. Records GPS coordinates during rides and syncs to a phone app via Bluetooth Low Energy. No cellular, no SIM card, no third-party cloud — fully open source.

![Prototype](docs/prototype.png)

## How it works

1. Attach the tracker to your bicycle
2. Start riding — the tracker detects motion and begins logging GPS every 5 seconds
3. Stop riding — after 30 seconds of no motion, tracking stops
4. Your phone app auto-syncs the ride data via BLE
5. The tracker goes to deep sleep until the next ride

## Features

- Automatic motion detection (wake on movement, sleep when idle)
- GPS logging at 5-second intervals
- BLE sync to phone (auto-sync or manual)
- AES-128 encrypted data — only your paired phone can read it
- Flash storage — ride data survives reboots and battery swaps
- Factory reset via BOOT button (5 second hold)
- Deep sleep with 30-second periodic wake for motion check

## Hardware

| Component | Description | Est. price |
|---|---|---|
| ESP32-C3 dev board | Microcontroller with BLE (e.g. XIAO ESP32-C3, Super Mini) | ~€2-5 |
| ATGM336H GPS module | UART GPS with ceramic antenna included | ~€3-9 |
| ADXL345 accelerometer | I2C motion detection for wake/sleep | ~€2-4 |
| 3x AA battery holder | Power supply (4.5V via onboard regulator) | ~€0.50-1 |
| **Total** | | **~€8-19** |

Optional: TP4056 USB-C charger module + Li-Po battery instead of AA.

See [docs/bom.md](docs/bom.md) for detailed bill of materials.

## Wiring

| Module | Pin | XIAO ESP32-C3 |
|---|---|---|
| ATGM336H | TX | D7 (GPIO20/RX) |
| ATGM336H | RX | D6 (GPIO21/TX) |
| ATGM336H | VCC | 3V3 |
| ATGM336H | GND | GND |
| ADXL345 | SDA | D4 (GPIO6) |
| ADXL345 | SCL | D5 (GPIO7) |
| ADXL345 | VCC | 3V3 |
| ADXL345 | GND | GND |
| Battery holder | + (red) | 5V |
| Battery holder | - (black) | GND |

See [docs/wiring.md](docs/wiring.md) for more detail.

## Firmware

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software)
- ESP32 board support: add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to Board Manager URLs
- Board: **XIAO_ESP32C3**
- Libraries: TinyGPSPlus, Adafruit ADXL345, Adafruit Unified Sensor

### Upload

1. Open `firmware/gps_tracker/gps_tracker.ino` in Arduino IDE
2. Select board and port
3. Upload

### BLE Interface

| Characteristic | UUID | Type | Description |
|---|---|---|---|
| Point | `beb5483e-...` | Read | Returns one AES-encrypted GPS point per read, "DONE" when finished |
| Count | `8ca20d91-...` | Read | Number of stored points |
| Clear | `9da30e02-...` | Write | Write any value to clear all points |
| Pair | `a1b2c3d4-...` | Write | Write 16-byte key to pair (first time only) |
| Status | `b2c3d4e5-...` | Read | "SETUP" or "PAIRED" |
| State | `c3d4e5f6-...` | Read | "IDLE" or "TRACKING" |

Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

## Phone App

The companion app is part of the [Bike Data Project app](https://github.com/bikedataproject/app) (.NET MAUI, Android). See the `feature/ble-tracker-sync` branch.

## Battery Life

Estimates with 2500mAh battery, optimized firmware:

| Usage | Battery life |
|---|---|
| 2 x 1-hour rides/day | ~25 days |
| 5 x 10-min rides/day | ~52 days |

Main drain is the GPS module (~25mA active). Deep sleep current is ~5µA.

## Factory Reset

Hold the **BOOT** button for 5 seconds. This clears the encryption key and all stored data. The device returns to "Setup" mode for re-pairing.

## License

MIT
