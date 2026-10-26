# Bike Data Project - Hardware Tracker

Low-power bicycle GPS tracker with BLE sync. Records GPS coordinates during rides and syncs to a phone app via Bluetooth Low Energy. No cellular, no SIM card, no third-party cloud — fully open source.

![Prototype](docs/prototype.png)

## How it works

1. Attach the tracker to your bicycle
2. Start riding — motion wakes it, it acquires a GPS fix and logs a point every 5 seconds
3. Stop riding — after 30 seconds without motion the ride ends and the GPS powers down
4. Your phone app syncs the ride over BLE
5. The tracker deep-sleeps until the next ride

## Features

- Wake on motion via accelerometer interrupt
- GPS logging at 5-second intervals, timestamped with GPS UTC
- GPS supply switched by a MOSFET — sleep current 0.29 mA
- BLE off while riding, reopened for a 30 s sync window every 10 minutes
- Batched, delta-encoded, AES-128 encrypted transfer
- Flash storage with CRC — survives reboots and battery swaps
- On-device event log and duty-cycle diagnostics over BLE
- Factory reset by holding BOOT for 5 seconds

## Battery life

Measured: 48 mA tracking, 0.29 mA deep sleep, 2500 mAh pack.

| Usage | Battery life |
|---|---|
| Parked, no rides | ~357 days |
| 45 min/day | ~58 days |
| 2 x 1-hour rides/day | ~24 days |

Ride time sets battery life, not standby. The GPS is ~31 mA of the 48 mA while tracking.

## Hardware

| Component | Description | Est. price |
|---|---|---|
| ESP32-C3 dev board | Microcontroller with BLE (e.g. XIAO ESP32-C3) | ~€2-5 |
| ATGM336H GPS module | UART GPS with ceramic antenna included | ~€3-9 |
| ADXL345 accelerometer | I2C motion detection for wake/sleep | ~€2-4 |
| NDS0610 P-MOSFET + 10.2 kΩ | High-side switch for the GPS supply | ~€0.25 |
| 3x AA battery holder | Power supply (4.5 V via onboard regulator) | ~€0.50-1 |
| **Total** | | **~€8-19** |

Optional: TP4056 USB-C charger + Li-Po instead of AA. See [docs/bom.md](docs/bom.md).

## Wiring

| Module | Pin | XIAO ESP32-C3 |
|---|---|---|
| ATGM336H | TX | D7 (GPIO20/RX) |
| ATGM336H | RX | D6 (GPIO21/TX) |
| ATGM336H | VCC | MOSFET **drain** (not 3V3 directly) |
| ATGM336H | GND | GND |
| MOSFET | source | 3V3 |
| MOSFET | gate | D2 (GPIO4), with 10.2 kΩ to source |
| ADXL345 | SDA / SCL | D4 (GPIO6) / D5 (GPIO7) |
| ADXL345 | INT1 | D3 (GPIO5) |
| ADXL345 | VCC / GND | 3V3 / GND |
| Battery holder | + / - | 5V / GND |

Switch the GPS supply on the **high** side. Switching ground lets the GPS backfeed through the ESP32's UART pins and stay partly alive.

See [docs/wiring.md](docs/wiring.md).

## Firmware

- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`, board **XIAO_ESP32C3**
- ESP32 board support: add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to Board Manager URLs
- Libraries: NimBLE-Arduino, TinyGPSPlus, Adafruit ADXL345, Adafruit Unified Sensor

```sh
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 firmware/gps_tracker
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn esp32:esp32:XIAO_ESP32C3 firmware/gps_tracker
```

## States

| State | GPS | BLE | Meaning |
|---|---|---|---|
| `IDLE` | off | up | Waiting. Deep-sleeps after 2 min |
| `ACQUIRING` | on | up | Motion detected, hunting for a fix |
| `TRACKING` | on | down | Fix acquired, logging points |
| `PAUSED` | on | down | Stopped within a ride; ends after 30 s |

## BLE interface

Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID | Type | Description |
|---|---|---|---|
| Batch | `beb5483e-...` | Read | Up to 10 points per read, encrypted. `DONE` when finished |
| Count | `8ca20d91-...` | Read | Stored point count, or `LOCKED` if unpaired |
| Clear | `9da30e02-...` | Write | Write any value to clear all points |
| Pair | `a1b2c3d4-...` | Write | Write a 16-byte key to pair (first time only) |
| Status | `b2c3d4e5-...` | Read | `SETUP` or `PAIRED` |
| State | `c3d4e5f6-...` | Read | `IDLE`, `ACQUIRING`, `TRACKING` or `PAUSED` |
| Seek | `d4e5f607-...` | Write | Set the batch read cursor, as a decimal string |
| Power | `f6a7b8c9-...` | Read | Duty-cycle and power summary |
| Log | `a7b8c9da-...` | Read | One event-log line per read, `DONE` at the end |
| Battery | `c9dae1f2-...` | Read | `avail=1 mv=4426 pct=85 low=0 crit=0` |

Power, Log and Battery are not gated on pairing — they carry no location data, so a device stays diagnosable even if its key is lost.

### Batch format

Decrypted payload, per read:

```
header     startIndex (u16 BE), count (u16 BE)
reference  lat (f32 LE), lng (f32 LE), epoch (u32 LE)      <- first point in batch
deltas     dlat (i16 LE), dlng (i16 LE), dt (u16 LE)       <- x (count-1)
```

Deltas are in microdegrees and accumulate from the reference. `epoch` is seconds since 2000-01-01 UTC.

## Tools

`tools/diag.py` speaks the same protocol as the app, so the device can be verified without a phone:

```sh
python3 -m venv ~/.venvs/bt && ~/.venvs/bt/bin/pip install bleak cryptography
~/.venvs/bt/bin/python tools/diag.py status    # paired state, state, point count
~/.venvs/bt/bin/python tools/diag.py pair      # generate and store a key
~/.venvs/bt/bin/python tools/diag.py download  # fetch + decode points -> points.csv
~/.venvs/bt/bin/python tools/diag.py power     # duty cycle, average current, battery
~/.venvs/bt/bin/python tools/diag.py log       # on-device event log -> events.csv
```

A sleeping tracker does not advertise — tap it to wake before connecting.

## Phone app

Part of the [Bike Data Project app](https://github.com/bikedataproject/app) (.NET MAUI, Android), branch `feature/ble-tracker-sync`.

## Factory reset

Hold **BOOT** for 5 seconds while running. Clears the key and all stored data, returning to `SETUP` mode.

Hold it while running, not across a reset: GPIO9 is the bootloader strap, so holding it at reset enters download mode instead.

## License

MIT
