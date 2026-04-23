# Battery Test Log

## Setup
- Power: 3x AA batteries
- Firmware: GPS tracker with deep sleep (30s timer wake)
- Usage: real-world bicycle test

## Readings

| Date | Time | Voltage | Notes |
|---|---|---|---|
| 2026-04-20 | start | 4.338V | Fresh batteries, soldered to breadboard, starting real-world test |
| 2026-04-20 | — | 4.010V | At least one ride completed |
| 2026-04-22 | — | dead | Battery died after ~2 days, ~4 rides. GPS module draws power 24/7 during sleep. |
| 2026-04-22 | start | 3.770V | Fresh rechargeable AAs. Interrupt-based wake (no timer). |
