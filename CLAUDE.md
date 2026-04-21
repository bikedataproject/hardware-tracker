# Bike Data Project - Hardware Tracker

## Project
Low-power bicycle GPS tracker with BLE sync. Part of the Bike Data Project.

## Architecture
- ESP32-C3 microcontroller with BLE 5
- ATGM336H GPS module (UART, 3.3V, NMEA)
- ADXL345 accelerometer (I2C, motion detection)
- Power: 3x AA or Li-Po + TP4056 USB-C charger
- Firmware: Arduino IDE (C/C++)

## Firmware
- Source: `firmware/gps_tracker/gps_tracker.ino`
- Board: XIAO ESP32-C3 (or any ESP32-C3 board)
- Libraries: TinyGPS++, Adafruit ADXL345, built-in ESP32 BLE, mbedtls (AES)
- BLE device name: "BikeTracker" (paired) / "BikeTracker (Setup)" (unpaired)
- States: IDLE → ACQUIRING → TRACKING → PAUSED → IDLE → deep sleep
- Timer-based deep sleep (30s wake cycle, checks accelerometer for motion)
- AES-128 ECB encryption for GPS data over BLE
- Batch transfer: 10 points per BLE read (diff-encoded binary)
- Resumable sync: Seek characteristic to resume from a specific index
- GPS UTC timestamps (epoch seconds from 2000-01-01)
- LittleFS for persistent point storage
- Factory reset: BOOT button held 5 seconds

## Motion Detection (hybrid)
- Wake from sleep: accelerometer > 2.0 m/s² (needs real movement)
- Keep ride alive: accelerometer > 1.5 m/s² OR GPS speed > 3 km/h
- Pause: neither accelerometer nor GPS speed detects movement
- End ride: 30 seconds of no movement from either source
- Production: ADXL345 replaced by SW-18010P vibration switch + time filter

## Pin Mapping (XIAO ESP32-C3)
- GPS: TX→D7(GPIO20), RX→D6(GPIO21)
- ADXL345: SDA→D4(GPIO6), SCL→D5(GPIO7)
- BOOT button: GPIO9

## BLE Service
- UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- Characteristics: Batch (read), Count (read), Clear (write), Pair (write), Seek (write), Status (read), State (read)

## Companion App
- Repo: github.com/bikedataproject/app (branch: feature/ble-tracker-sync)
- .NET MAUI, Android
- Devices page: scan, pair, auto-sync, ride history per device
- Buffered sync: downloads points during ride, finalizes rides only when IDLE
- Trip splitting: >10 min gap between points = separate ride
- Auto-sync every 15s while devices page is open, every 2 min in background
- AES-128 decryption, key in SecureStorage

## Key Design Decisions
- Timer-based wake (not GPIO interrupt) — avoids false triggers on breadboard
- AES-128 ECB — simple, hardware-accelerated on ESP32
- No cellular — BLE sync to phone only, no SIM or data plan
- Hybrid motion detection — accelerometer for vibration + GPS speed for smooth roads
- Buffered sync — download during ride for speed, finalize only when ride ends
- Only clear device data when tracker is IDLE
