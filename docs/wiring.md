# Wiring Guide

## Overview

Three modules connect to the XIAO ESP32-C3:
- **ATGM336H** GPS module via UART (2 data wires + power)
- **ADXL345** accelerometer via I2C (2 data wires + power)
- **Battery** (2 wires)

Total: 10 connections, with shared 3V3 and GND lines.

## Pin Connections

```
XIAO ESP32-C3          ATGM336H GPS
┌─────────────┐        ┌───────────┐
│         D7 ─┼────────┤ TX        │
│         D6 ─┼────────┤ RX        │
│        3V3 ─┼──┬─────┤ VCC       │
│        GND ─┼──┬─────┤ GND       │
│             │  │  │  └───────────┘
│             │  │  │
│             │  │  │  ADXL345
│             │  │  │  ┌───────────┐
│         D4 ─┼──┼──┼──┤ SDA       │
│         D5 ─┼──┼──┼──┤ SCL       │
│             │  ├──┼──┤ VCC       │
│             │  │  ├──┤ GND       │
│             │  │  │  └───────────┘
│             │  │  │
│             │  │  │  3x AA Battery
│             │  │  │  ┌───────────┐
│         5V ─┼──┼──┼──┤ + (red)   │
│             │  │  └──┤ - (black) │
└─────────────┘  │     └───────────┘
                 │
          Shared 3V3 and GND rails
```

## XIAO ESP32-C3 Pin Reference

| Board label | GPIO | Used for |
|---|---|---|
| D4 | GPIO6 | ADXL345 SDA (I2C) |
| D5 | GPIO7 | ADXL345 SCL (I2C) |
| D6 | GPIO21 | GPS TX→RX |
| D7 | GPIO20 | GPS RX→TX |
| 3V3 | — | Power for GPS + accelerometer |
| 5V | — | Battery input (regulated to 3.3V) |
| GND | — | Common ground |

## Power Options

### Option A: 3x AA batteries
- Connect battery holder red (+) wire to XIAO **5V** pin
- Connect battery holder black (-) wire to XIAO **GND** pin
- 4.5V is regulated down to 3.3V by the XIAO's onboard regulator
- Replace batteries when voltage drops below ~3.0V

### Option B: Li-Po + TP4056 charger
- TP4056 B+ → Li-Po red (+)
- TP4056 B- → Li-Po black (-)
- TP4056 OUT+ → XIAO 5V
- TP4056 OUT- → XIAO GND
- Charge via USB-C on the TP4056 module
- Do NOT connect USB to XIAO and TP4056 at the same time

## Important Notes

- The ATGM336H is **3.3V only** — do NOT connect VCC to 5V
- GPS TX connects to XIAO RX (D7), and GPS RX connects to XIAO TX (D6) — they cross
- Keep wires short to reduce noise, especially the GPS UART lines
- If using a breadboard, share 3V3 and GND via the power rails
