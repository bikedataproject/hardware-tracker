# Bill of Materials

## Prototype (~€19)

### Core Components
| Component | Example | Est. price |
|---|---|---|
| ESP32-C3 dev board | Seeed Studio XIAO ESP32-C3 / ESP32-C3 Super Mini | €2-5 |
| GPS module | ATGM336H (with ceramic antenna) | €3-9 |
| Accelerometer | ADXL345 3-axis module | €2-4 |
| Power | 3x AA battery holder with wires | €0.50-1 |

### Optional
| Component | Description | Est. price |
|---|---|---|
| TP4056 USB-C charger | Li-Po charging with protection circuit | €1-2 |
| Li-Po battery | 3.7V 1000-2500mAh, JST-PH | €5-8 |
| Breadboard | 400 points, for prototyping | €1-2 |

### Tools (one-time)
| Tool | Notes |
|---|---|
| Soldering iron | Temperature-controlled, ~60-65W |
| Solder | Leaded (Sn63/Pb37), 0.5mm |
| Multimeter | For voltage measurements |
| Flush cutters | For trimming wires |
| USB-C cable | For programming the ESP32 |

## Small Batch — 100 units (~€7-8 each)

Source from AliExpress for lower prices:

| Component | Unit price |
|---|---|
| ESP32-C3 bare module | ~€1.50 |
| AT6558 GPS module + antenna | ~€2.00 |
| SW-18010P vibration switch (replaces ADXL345) | ~€0.05 |
| Custom PCB + passives | ~€1.50 |
| 3x AA battery holder | ~€0.30 |
| 3D printed enclosure | ~€1-2 |
| **Total** | **~€7-8** |

## Volume — 1000+ units (~€4-5 each)

| Component | Unit price |
|---|---|
| ESP32-C3 chip (bare) | ~€0.60 |
| AT6558 GPS chip | ~€0.80 |
| SW-18010P vibration switch | ~€0.05 |
| Custom PCB + SMD assembly (JLCPCB) | ~€1.50 |
| Injection molded enclosure | ~€1-2 |
| **Total** | **~€4-5** |

## Notes
- All components run at 3.3V logic — no level shifters needed
- ATGM336H is NOT 5V tolerant — always connect to 3V3
- GPS ceramic antenna is usually included with the ATGM336H module
- No SIM card, cellular modem, or cellular antenna needed
- For production, the ADXL345 can be replaced by a bare SW-18010P vibration switch (cheaper, zero power drain, simpler wiring)
