# DC Motor Real-Time Predictive Health Diagnostic System

[![Python Version](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Microcontroller-Raspberry%20Pi%20Pico%20H%20(RP2040)-orange.svg)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Frontend](https://img.shields.io/badge/Interface-SCADA%20Web%20Dashboard-green.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An industrial-grade embedded condition monitoring and predictive diagnostic system for DC motors. Features a dual-stack architecture allowing operation via a real-time hardware telemetry link (Raspberry Pi Pico H + multi-sensor bus) or an electromechanical physics simulation engine with an interactive fault injection laboratory.

---

## 📑 Table of Contents
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Key Features](#key-features)
- [Quick Start Guide](#quick-start-guide)
  - [Prerequisites](#prerequisites)
  - [Running the Web Dashboard](#running-the-web-dashboard)
- [Hardware Setup & Wiring](#hardware-setup--wiring)
  - [Pinout Mapping](#pinout-mapping)
  - [Arduino IDE Firmware Setup](#arduino-ide-firmware-setup)
- [Interactive Fault Injection Lab](#interactive-fault-injection-lab)
- [Backend Architecture & Security](#backend-architecture--security)
- [Contributing](#contributing)
- [License](#license)

---

## 🏗️ System Architecture

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        SCADA WEB DASHBOARD                             │
│       (Real-Time Charts, Plain-English Diagnostics, Fault Lab)        │
└────────────────────────────────────┬───────────────────────────────────┘
                                     │ Server-Sent Events (SSE) & REST
┌────────────────────────────────────┴───────────────────────────────────┐
│                      PYTHON BACKEND CONTROLLER                         │
│                  (ThreadingHTTPServer, Port Guard)                     │
└──────────────────┬─────────────────────────────────┬───────────────────┘
                   │                                 │
         [SIMULATION MODE]                   [HARDWARE MODE]
                   │                                 │ USB Serial (115200)
┌──────────────────┴───────────────┐ ┌───────────────┴───────────────────┐
│   Electromechanical Simulator    │ │  Raspberry Pi Pico H (RP2040)     │
│   • Inertia & Friction Decay     │ │  • Ultrasonic PWM (20 kHz)        │
│   • I²R Thermal Dissipation      │ │  • DS18B20 1-Wire Temperature     │
│   • Harmonic Vibration Noise     │ │  • MPU-6050 6-DOF IMU Vibration   │
│   • Fault Injection Engine       │ │  • INA219 Voltage/Current/Power   │
│                                  │ │  • Optical Speed Encoder IRQ      │
│                                  │ │  • SH1106 1.3" OLED Local Display │
└──────────────────────────────────┘ └───────────────────────────────────┘
```

---

## 📁 Repository Structure

```text
.
├── dashboard/
│   ├── server.py                 # Threading HTTPServer, SSE streaming, serial manager
│   └── static/
│       ├── app.js                # Frontend telemetry controller & audio alert engine
│       ├── index.html            # SCADA layout & diagnostic UI panels
│       └── style.css             # Industrial high-contrast responsive styling
├── dc_motor_health_monitor/
│   └── dc_motor_health_monitor.ino # RP2040 Arduino C/C++ embedded firmware
├── run_dashboard.py              # CLI launcher script with browser auto-open
├── simulation.py                 # Standalone electromechanical physics model
├── requirements.txt              # Host Python dependencies
├── .gitattributes                # Line ending and binary file configuration
├── .gitignore                    # Comprehensive ignore rules
├── LICENSE                       # MIT Open Source License
└── README.md                     # Project documentation & wiring guide
```

---

## ⚡ Key Features

- **Industrial SCADA Web Interface:** Clean, high-contrast engineering dashboard with zero clutter or artificial bloat.
- **Human-Readable Diagnostics:** Clear, actionable alert cards explaining exact conditions, measured values vs. safety thresholds, and root mechanical risks (e.g., *"Motor Overheating (54.2°C): Surface temperature exceeds 50.0°C limit. Risk of winding insulation damage."*).
- **Dual Operating Modes:**
  - **Simulation Mode:** Standalone physical model executing inertial spin-down, thermal dynamics, and Gaussian vibration modeling without external hardware.
  - **Hardware Mode:** Live bidirectional communication with RP2040 over USB Serial (`115200 baud`).
- **Interactive Fault Injection Laboratory:** Test your maintenance reactions in real time with single-click physical fault injections:
  - 🔥 **Overheating** (casing temp above 50°C)
  - ⚡ **Overload / High Current** (surges load above 900 mA)
  - 〰️ **Severe Vibration** (induces mechanical shock past 1.80 g RMS)
  - 🛑 **Rotor Jam / Stall** (locks shaft to 0 RPM while power is applied)
  - ⚙️ **Worn Bearing Drag** (accelerates coast-down deceleration past -90 rad/s²)
  - 🔋 **Supply Voltage Drop** (drops rail to 9.8 V)
- **Audible Alerts:** Synthesized audio alarm tone via Web Audio API with convenient mute toggle.

---

## 🚀 Quick Start Guide

### Prerequisites

- **Python:** Python 3.8 or higher.
- **Virtual Environment (Recommended):**

```bash
# Clone the repository
git clone https://github.com/<your-username>/<your-repo-name>.git
cd "DC Motor Health Monitoring"

# Optional: Create and activate a virtual environment
python -m venv .venv
# On Windows:
.venv\Scripts\activate
# On Linux/macOS:
source .venv/bin/activate

# Install dependencies (only pyserial is needed for hardware connection)
pip install -r requirements.txt
```

### Running the Web Dashboard

Execute the launcher script:

```bash
python run_dashboard.py
```

Your default web browser will automatically open:
```text
http://localhost:8080
```

> **Tip:** Pass `--no-browser` if running on a headless workstation or remote server:
> ```bash
> python run_dashboard.py --no-browser
> ```

---

## 🔌 Hardware Setup & Wiring

### Pinout Mapping

| Subsystem / Peripheral | Sensor / Module | RP2040 Pin | Physical Pin | Function |
| :--- | :--- | :--- | :--- | :--- |
| **Shared I2C0 Bus** | SDA | **GP4** | Pin 6 | 400 kHz Fast-Mode I2C Data |
| | SCL | **GP5** | Pin 7 | 400 kHz Fast-Mode I2C Clock |
| | MPU-6050 | `0x68` | — | 6-DOF IMU (44 Hz DLPF configured, RMS Vibration) |
| | INA219 | `0x40` | — | High-side Current ($I_a$), Bus Voltage, Power |
| | SH1106 OLED | `0x3C` | — | **1.3-inch OLED** (128x64 display driven via Adafruit_SH110X) |
| **1-Wire Bus** | DS18B20 | **GP15** | Pin 20 | Digital casing temperature probe (**Requires 4.7kΩ pull-up to 3.3V**) |
| **Optical Encoder** | HC-89 / LM393 | **GP2** | Pin 4 | Falling-edge hardware IRQ (`PULSES_PER_REV = 4`) |
| **L298N Motor Driver** | ENA (PWM) | **GP10** | Pin 14 | **20 kHz Ultrasonic PWM** (silent motor drive) |
| | IN1 | **GP11** | Pin 15 | H-Bridge Direction 1 (GPIO Out) |
| | IN2 | **GP12** | Pin 16 | H-Bridge Direction 2 (GPIO Out) |
| **Power Rails** | 3.3V (OUT) | 3V3 | Pin 36 | Sensor logic power & pull-up supply |
| | GND | GND | Pins 3, 8, 13, etc. | Common ground with L298N and power supply |

### Antigravity & CLI Firmware Setup (ESP-Style Zero-Hardware Workflow)

No BOOTSEL button or jumper wires are needed! The system includes an automated toolchain and software reset handler (`pico_tool.py`):

#### 1-Click Commands:
```bash
# Check if your Pico H is plugged in (detects both BOOTSEL drive and COM port)
python pico_tool.py detect

# Compile the C/C++ firmware into a UF2 binary
python pico_tool.py build

# Flash firmware automatically (detects RPI-RP2 drive or 1200-baud auto-resets COM port)
python pico_tool.py upload

# Build, upload, and open the serial telemetry monitor in one step
python pico_tool.py run

# Live serial monitor with interactive motor commands
python pico_tool.py monitor

# Force Pico into UF2 bootloader mode from terminal without touching any hardware
python pico_tool.py bootsel
```

#### Antigravity / VS Code Tasks:
Press `Ctrl+Shift+B` or open the Command Palette (`Ctrl+Shift+P` -> `Tasks: Run Task`):
- **Pico: Build Firmware** (Default Build)
- **Pico: Upload / Flash Firmware**
- **Pico: Build & Upload (1-Click Flash)**
- **Pico: Serial Telemetry Monitor**
- **Pico: Software BOOTSEL Reset**

> **How it works without a BOOTSEL button or jumper wires:**
> - **Blank / Brand-new Pico H:** Because flash memory is empty, the RP2040 Boot ROM automatically mounts as drive `RPI-RP2` immediately when plugged in.
> - **Subsequent Re-flashing:** The Earle Philhower core and our firmware detect a standard **1200-baud touch** (opening the COM port at 1200 bps) and the serial command `BOOTSEL`. The board reboots directly into UF2 mode automatically before uploading, exactly like an ESP32 auto-reset circuit!

---

### Alternative: Arduino IDE Setup (Optional)

1. Open the Arduino IDE.
2. Select board **Raspberry Pi Pico** (using *Raspberry Pi Pico/RP2040 by Earle F. Philhower, III*).
3. Open [dc_motor_health_monitor/dc_motor_health_monitor.ino](dc_motor_health_monitor/dc_motor_health_monitor.ino).
4. Click **Upload** (the Philhower core automatically 1200-baud resets the board).
5. Switch the Web Dashboard to **Raspberry Pi Pico** mode and click **Connect to Pico**.

---

## 🧪 Interactive Fault Injection Lab

When running in **Simulation Mode**, the dashboard allows on-the-fly injection of anomalies to evaluate how the predictive diagnostic engine responds:

| Fault Button | Injected Condition | Diagnostic Indication | Root Hazard |
| :--- | :--- | :--- | :--- |
| **Simulate Overheating** | Temperature rises > 50.0°C | Critical Red Banner | Coil insulation degradation |
| **Simulate High Current** | Current surge > 900 mA | Critical Red Banner | Electrical short or mechanical lock |
| **Simulate Vibration** | RMS vibration > 1.80 g | Critical Red Banner | Rotor imbalance / mounting looseness |
| **Simulate Rotor Stall** | 0 RPM while PWM applied | Critical Red Banner | Armature burnout hazard |
| **Simulate Bearing Drag** | Rapid deceleration (< -90 rad/s²) | Warning Yellow Banner | Bearing race wear / lubricant loss |
| **Simulate Low Voltage** | Supply bus drops to 9.8 V | Metric Highlight | Brownout risk or battery depletion |

---

## 🔒 Backend Architecture & Security

- **Threading Model:** Built with `ThreadingHTTPServer` to guarantee that persistent Server-Sent Events (SSE) telemetry streams do not block REST requests or UI controls.
- **Path Traversal Protection:** Static file resolution validates against canonical paths, completely mitigating directory traversal risks.
- **Input Sanitization:** Speed values clamped to `0.0 - 100.0%`, actions restricted to strict whitelists, and serial ports verified with regex.
- **Safe Network Defaults:** Server binds to `127.0.0.1` by default to prevent unintended local network exposure.
- **Socket Disconnect Resilience:** Graceful recovery from client connection aborts (`BrokenPipeError`, `ConnectionAbortedError`) prevents background worker drops.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to open an issue or submit a pull request:
1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push -u origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

Distributed under the MIT License. See [LICENSE](LICENSE) for more information.
