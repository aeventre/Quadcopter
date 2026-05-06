# DIY Quadcopter

A from-scratch DIY quadcopter with a custom-designed flight computer PCB and custom frame CAD. The ESC is off-the-shelf; everything else was built from the ground up.

## Overview

This repository contains the KiCad hardware design files for a custom flight controller board. The flight controller interfaces with four off-the-shelf ESCs, an RC receiver, and an onboard 6-axis IMU to stabilize and control the drone.

> Firmware is maintained in a separate repository.

## Hardware

### Flight Controller (Custom PCB)

| Component | Part | Role |
|---|---|---|
| MCU | Raspberry Pi Pico (RP2040 / RP2350) | Main processor |
| IMU | LSM6DSO32TR | 6-axis accelerometer + gyroscope |
| Power regulator | TPS563200 | 3A synchronous step-down, 4.5–17V in → 3.3V |
| Motor switching | AO3400A MOSFET | N-channel switching |
| Level shifting | BSS138 MOSFET | 3.3V ↔ 5V logic |
| Transient protection | SMAJ14A TVS diode | Input voltage spike protection |

### Interfaces
- **4× ESC connectors** (Motor 1–4) — off-the-shelf ESCs
- **RC receiver connector**
- **Electromagnet connector** — for payload/drop mechanism
- **SPI & I2C** — IMU and peripheral communication
- **USB** — programming and debug via Pico's onboard USB

### Frame & CAD
Custom-designed from scratch (CAD files not included in this repo).

## Repository Structure

```
Quadcopter/
├── Flight_Controller/
│   ├── Flight_Controller.kicad_pro   # KiCad project file
│   ├── Flight_Controller.kicad_sch   # Schematic
│   └── Flight_Controller.kicad_pcb   # PCB layout
└── LICENSE
```

## Getting Started

1. Install [KiCad 7.x or later](https://www.kicad.org/download/)
2. Clone this repository:
   ```bash
   git clone https://github.com/alecv2211/Quadcopter.git
   ```
3. Open `Flight_Controller/Flight_Controller.kicad_pro` in KiCad

## License

Licensed under the [Apache License 2.0](LICENSE).
