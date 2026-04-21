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
- States: IDLE → TRACKING → IDLE → deep sleep
- Timer-based deep sleep (30s wake cycle, checks accelerometer for motion)
- AES-128 ECB encryption for GPS data over BLE
- LittleFS for persistent point storage
- Factory reset: BOOT button held 5 seconds

## Pin Mapping (XIAO ESP32-C3)
- GPS: TX→D7(GPIO20), RX→D6(GPIO21)
- ADXL345: SDA→D4(GPIO6), SCL→D5(GPIO7)
- BOOT button: GPIO9

## BLE Service UUID
`4fafc201-1fb5-459e-8fcc-c5c9c331914b`

## Companion App
- Repo: github.com/bikedataproject/app (branch: feature/ble-tracker-sync)
- .NET MAUI, Android
- Auto-sync every 2 min via background BLE scan
- AES-128 decryption, key in SecureStorage

## Key Design Decisions
- Timer-based wake (not GPIO interrupt) — avoids false triggers on breadboard
- AES-128 ECB — simple, hardware-accelerated on ESP32
- No cellular — BLE sync to phone only, no SIM or data plan
- Production target: replace ADXL345 with SW-18010P vibration switch (~€0.05)
