# DIY Quadcopter

A from-scratch DIY quadcopter with a custom-designed flight computer PCB and custom frame CAD. The ESC is off-the-shelf; everything else was built from the ground up.

## Overview

This repository contains the KiCad hardware design files for the custom flight controller PCB and the Arduino/PlatformIO firmware that runs on it. The flight controller interfaces with four off-the-shelf ESCs, an RC receiver via CRSF/ExpressLRS, and an onboard 6-axis IMU to stabilize and control the drone.

## Hardware

### Flight Controller (Custom PCB)

| Component | Part | Role |
|---|---|---|
| MCU | Raspberry Pi Pico (RP2040 / RP2350) | Main processor |
| IMU | LSM6DSO32TR | 6-axis accelerometer + gyroscope (SPI) |
| Power regulator | TPS563200 | 3A synchronous step-down, 4.5–17V in → 3.3V |
| Motor switching | AO3400A MOSFET | N-channel switching |
| Level shifting | BSS138 MOSFET | 3.3V ↔ 5V logic |
| Transient protection | SMAJ14A TVS diode | Input voltage spike protection |

### Interfaces
- **4× ESC connectors** (Motor 1–4) — off-the-shelf ESCs
- **RC receiver connector** — CRSF/ExpressLRS
- **Electromagnet connector** — payload drop mechanism
- **SPI** — IMU (LSM6DSO32TR on SPI1)
- **USB** — programming and debug via Pico's onboard USB

### Frame & CAD
Custom-designed from scratch (CAD files not included in this repo).

## Firmware

Built with [PlatformIO](https://platformio.org/) targeting the Raspberry Pi Pico using the [earlephilhower Arduino core](https://github.com/earlephilhower/arduino-pico).

### Flight Controller (`Code/Controller_Code/`)

The main flight firmware. Initializes the IMU, ESCs, and CRSF receiver, then runs the control loop.

**Libraries (`lib/`)**

| Library | Description |
|---|---|
| `DroneController` | Core flight controller — PID loop, flight modes, arm/disarm, RAM blackbox |
| `ImuLSM6` | SPI driver for the LSM6DSO32TR (gyro + accel reads) |
| `MotorLib` | PWM motor control via the Servo library (1000–2000 µs) |

**Flight modes**

- **Acro** — pure rate mode; stick input maps directly to roll/pitch/yaw rate setpoints fed into the inner PID loop
- **Balance** — self-leveling; an outer angle P+I loop computes rate setpoints from stick-commanded angles, which the inner rate PID tracks

**Control loop**

```
RC input (CRSF)
    │
    ├─ [Balance mode] Angle P+I → rate setpoint
    │
    └─ Rate PID (roll / pitch / yaw)
           │   gyro LP filter (α = 0.70)
           │   complementary filter for attitude (α = 0.98)
           ▼
       Motor mixer → 4× PWM ESC outputs
```

**RAM blackbox**

Up to 3000 samples are logged to RAM during flight (timestamp, RC inputs, rate commands, gyro rates, PID outputs, motor values, attitude angles). On disarm the log dumps over USB serial as CSV for offline analysis.

### Test Projects

| Project | Description |
|---|---|
| `Code/ELRS_Test/` | Standalone CRSF/ExpressLRS receiver test |
| `Code/ESC_Test/` | ESC test using DShot600 (bit-banged on Pico's second core at 2 kHz) |
| `Code/IMU_test/` | Standalone LSM6DSO32TR read/print test |

### Flight Analysis (`Code/Flight_Analysis/`)

MATLAB script (`PID_Analysis.m`) for post-flight analysis of blackbox CSV logs. Plots RC inputs, commanded vs. measured rates, rate errors, motor outputs, and estimated attitude. 103 flight logs are included.

## Repository Structure

```
Quadcopter/
├── Flight_Controller/
│   ├── Flight_Controller.kicad_pro
│   ├── Flight_Controller.kicad_sch
│   └── Flight_Controller.kicad_pcb
└── Code/
    ├── Controller_Code/          # Main flight firmware (PlatformIO)
    │   ├── src/main.cpp
    │   ├── lib/
    │   │   ├── DroneController/  # Flight controller, PID, blackbox
    │   │   ├── ImuLSM6/          # LSM6DSO32TR SPI driver
    │   │   └── MotorLib/         # PWM ESC control
    │   └── Tests/                # Component test sketches
    ├── ELRS_Test/                # ExpressLRS receiver test
    ├── ESC_Test/                 # DShot600 ESC test
    │   └── lib/DshotLib/         # DShot600 driver (dual-core)
    ├── IMU_test/                 # IMU read test
    └── Flight_Analysis/
        ├── PID_Analysis.m        # MATLAB blackbox analysis script
        └── flight_log_*.csv      # Captured flight logs (103 flights)
```

## Getting Started

### Hardware (KiCad)
1. Install [KiCad 7.x or later](https://www.kicad.org/download/)
2. Clone this repository:
   ```bash
   git clone https://github.com/alecv2211/Quadcopter.git
   ```
3. Open `Flight_Controller/Flight_Controller.kicad_pro`

### Firmware (PlatformIO)
1. Install [PlatformIO](https://platformio.org/install)
2. Open `Code/Controller_Code/` in VS Code with the PlatformIO extension
3. Connect the Pico via USB and run **Upload**

## License

Licensed under the [Apache License 2.0](LICENSE).
