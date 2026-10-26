# PMDC Motor Condition Monitoring & Predictive Diagnostics Test Bench

[![Python Version](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Microcontroller-ESP--32S%20NodeMCU-red.svg)](https://www.espressif.com/)
[![Driver](https://img.shields.io/badge/Motor%20Driver-L298N%20Dual%20H--Bridge-blue.svg)]()
[![Frontend](https://img.shields.io/badge/Interface-ISA--101%20SCADA%20Workstation-green.svg)]()
[![Transport](https://img.shields.io/badge/Transport-FastAPI%20%2B%20WebSocket-orange.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An industrial-grade embedded condition monitoring and predictive diagnostic test bench for Permanent Magnet DC (PMDC) motors (12V Geared Johnson DC Motor). 

The platform supports **both physical hardware operation** via high-speed ESP32 serial telemetry and **offline electromechanical physics simulation** with interactive fault injection.

---

## 📑 Table of Contents
- [System Architecture](#system-architecture)
- [Hardware Wiring & Pinout (ESP-32S)](#hardware-wiring--pinout-esp-32s)
- [L298N Jumper & Motor Setup](#l298n-jumper--motor-setup)
- [Firmware Flashing Guide (ESP32)](#firmware-flashing-guide-esp32)
- [How to Run the SCADA Dashboard & Backend](#how-to-run-the-scada-dashboard--backend)
- [Switching Between Live ESP32 Hardware and Simulation](#switching-between-live-esp32-hardware-and-simulation)
- [Command & Protocol Reference](#command--protocol-reference)
- [Predictive Diagnostic Algorithms](#predictive-diagnostic-algorithms)
- [Session CSV Data Logging](#session-csv-data-logging)

---

## 🏗️ System Architecture

```text
┌────────────────────────────────────────────────────────────────────────┐
│               ISA-101 SCADA BROWSER DASHBOARD (HTTP5 Canvas)          │
│  (Equipment Status, Process Readouts, Speed Slider, Strip Charts, LED) │
└────────────────────────────────────┬───────────────────────────────────┘
                                     │ WebSocket (/ws/telemetry) & REST API
┌────────────────────────────────────┴───────────────────────────────────┐
│               FASTAPI & UVICORN GATEWAY (Python 3.8+)                  │
│       • Full-Duplex 10 Hz Telemetry Stream                             │
│       • OLS Coast-Down Deceleration Parameter Identification           │
│       • In-Situ Armature Resistance (Ra) Trimmed Estimator             │
│       • Thread-Safe CSV Session Logging & Alarm Event Ledger           │
└──────────────────┬─────────────────────────────────┬───────────────────┘
                   │                                 │
         [SIMULATION MODE]                   [HARDWARE MODE]
         (No serial connected)               (COM port connected @ 115200)
                   │                                 │
┌──────────────────┴───────────────┐ ┌───────────────┴───────────────────┐
│   DCMotorPhysicsSimulator        │ │       ESP-32S NodeMCU (38-pin)    │
│   • Multi-Fault Physics Engine   │ │  • LEDC PWM (1 kHz, 8-bit duty)   │
│   • Inertia & Friction Decay     │ │  • HC-89 4-Slot Encoder (8 edges) │
│   • Casing Joule Heating Model   │ │  • INA219 Voltage & Current I²C   │
│   • Harmonic Vibration Model     │ │  • MPU-6050 6-DOF Vibration I²C   │
│   • Interactive Fault Injection  │ │  • DS18B20 1-Wire Temperature     │
│     (Stall, Overheat, Bearing)   │ │  • SH1106 1.3" OLED Local Display │
│                                  │ │  • Communication Safety Watchdog  │
└──────────────────────────────────┘ └───────────────────────────────────┘
```

---

## 🔌 Hardware Wiring & Pinout (ESP-32S)

| Component | Signal / Pin | ESP32-S GPIO | Notes |
|---|---|---|---|
| **L298N Driver** | ENA (Speed PWM) | **GPIO 14** | Hardware LEDC PWM (1 kHz, 0-255 duty). **Remove ENA jumper!** |
| | IN1 (Direction 1) | **GPIO 27** | Digital output: `HIGH` for forward |
| | IN2 (Direction 2) | **GPIO 26** | Digital output: `LOW` for forward |
| | GND | **GND** | **Common Ground** between ESP32, L298N, and 12V Supply |
| **Optical Encoder (HC-89)** | OUT / D0 | **GPIO 18** | `CHANGE` interrupt mode (both edges) on 4-slot disc = 8 edges/rev |
| | VCC / GND | 3.3V or 5V / GND | Optical slot sensor power |
| **I²C Sensor Bus** | SDA | **GPIO 21** | 400 kHz Fast-Mode I²C data |
| | SCL | **GPIO 22** | 400 kHz Fast-Mode I²C clock |
| | INA219 (0x40) | GPIO 21 / 22 | Measures bus voltage and armature current |
| | MPU-6050 (0x68) | GPIO 21 / 22 | Vibration RMS monitoring (mounted flat on motor) |
| | SH1106 OLED (0x3C) | GPIO 21 / 22 | Local 1.3" 128x64 diagnostic display |
| **1-Wire Temperature** | DATA | **GPIO 4** | DS18B20 data line (requires 4.7kΩ pull-up to 3.3V) |
| **Heartbeat Indicator** | LED | **GPIO 2** | ESP32 onboard blue LED (toggles at 1 Hz) |

---

## ⚙️ L298N Jumper & Motor Setup

> [!CAUTION]
> **5V Regulator Enable Jumper Rule:**
> - When feeding 12V SMPS into the L298N `12V` and `GND` screw terminals, **the 5V Regulator Enable jumper MUST BE INSTALLED**.
> - If this jumper is removed, the internal L298N logic supply ($V_{SS}$) is unpowered, the driver board LED will not light up, and the motor will not turn.
> - **ENA Jumper:** Must be **REMOVED** and connected directly to ESP32 **GPIO 14** for software PWM speed control.

**Series Sensor Connection Path:**
```text
L298N OUT1 ──► INA219 VIN+ ──► INA219 VIN- ──► Motor (+) ──► Motor (-) ──► L298N OUT2
```

---

## 💻 Firmware Flashing Guide (ESP32)

### Option A: Using Arduino IDE
1. Install **ESP32 by Espressif Systems** via Boards Manager.
2. Select board: `NodeMCU-32S` or `ESP32 Dev Module`.
3. Install required libraries via Library Manager:
   - `Adafruit INA219`
   - `Adafruit MPU6050`
   - `Adafruit Unified Sensor`
   - `DallasTemperature`
   - `OneWire`
   - `Adafruit SH110X`
   - `Adafruit GFX Library`
4. Open [`dc_motor_health_monitor/dc_motor_health_monitor.ino`](file:///d:/DAN/Antigravity%20Project%20Folder/prism/dc_motor_health_monitor/dc_motor_health_monitor.ino).
5. Compile and upload at **115200 baud**.

### Option B: Using PlatformIO CLI
```bash
pio run --target upload
```

---

## 🚀 How to Run the SCADA Dashboard & Backend

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Launch the SCADA Web Station
```bash
python run_dashboard.py
```
* Binds to `http://127.0.0.1:8080` (secure localhost).
* Automatically opens your default web browser.
* To run headless (without launching browser):
  ```bash
  python run_dashboard.py --no-browser
  ```
* To specify a custom port:
  ```bash
  python run_dashboard.py --port 9000
  ```

---

## 🔄 Switching Between Live ESP32 Hardware and Simulation

The backend automatically transitions between physical hardware and simulation without restarting the server:

### A. Operating in Live Hardware Mode
1. Plug the ESP32 into a USB port on your workstation.
2. In the top status bar of the Web Dashboard, click **SCAN** to detect serial ports.
3. Select your ESP32 COM port (e.g., `COM3`, `COM4`, `/dev/ttyUSB0`) and set Baud to `115200`.
4. Click **CONNECT**.
   - The status badge switches from `SIMULATION / OFFLINE` to `CONNECTED (COMx)`.
   - The Equipment Status panel indicates `ESP-32S DUAL-CORE`.
5. Click **ENABLE MOTOR DRIVER (ARM POWER)** to arm the driver interlock.
6. Use the **Proportional Speed Slider** or preset buttons (`50%`, `75%`, `100%`) to run the motor.
7. Click **[ MEASURE Ra ]** or **[ START COAST-DOWN TEST ]** to run automated diagnostics.

### B. Operating in Offline Simulation Mode
1. If the ESP32 is unplugged or you click **DISCONNECT**, the backend automatically enters **Simulation Mode**.
2. Telemetry continues streaming seamlessly at 10 Hz from the built-in electromechanical physics engine (`simulation.py`).
3. The Equipment Status panel reports `SIMULATION / OFFLINE`.
4. **Fault Injection Lab:** In simulation mode, use the yellow fault buttons in Panel 2 to inject real-time physical faults:
   - **OVERHEAT**: Drives casing temperature above the 50°C alarm limit.
   - **OVERCURRENT**: Simulates heavy mechanical loading (>900 mA trip limit).
   - **ROTOR JAM / STALL**: Locks rotor speed to 0 RPM under commanded PWM.
   - **BEARING DRAG**: Increases Coulomb friction and viscous damping, causing rapid coast-down decay.
   - **HIGH VIBRATION**: Introduces high-amplitude bearing defect vibration harmonics (>1.8G).
   - **VOLTAGE SAG**: Simulates high power supply impedance causing rail drops.
   - Click **CLEAR ALL FAULTS** to restore nominal motor operation.

---

## 📡 Command & Protocol Reference

Commands sent to the ESP32 via Serial (115200 Baud, newline-terminated):

| Command String | Response / Handshake | Action |
|---|---|---|
| `CMD:DRIVER_POWER:ON` | `ACK:DRIVER_POWER:ON` | Arms H-bridge driver logic; allows PWM switching |
| `CMD:DRIVER_POWER:OFF` | `ACK:DRIVER_POWER:OFF` | Disarms driver; forces PWM duty to 0 and inputs LOW |
| `CMD:SET_SPEED:<0-255>` | `ACK:SET_SPEED:<duty>` | Sets LEDC 8-bit PWM speed (requires driver armed) |
| `CMD:ESTOP` | `ACK:ESTOP` | Emergency stop: zeroes PWM immediately |
| `CMD:RUN_COAST_TEST` | `ACK:RUN_COAST_TEST` | Spins motor to 75% for 2.0s, then emits `ACK:COAST_CUTOFF` |
| `CMD:MEASURE_RA` | `ACK:MEASURE_RA:V_bus=..,Ra=..` | Delivers 60ms sub-stall pulse for $R_a$ calculation |
| `CMD:PING` | `ACK:PONG` | Heartbeat keep-alive check |

---

## 🔬 Predictive Diagnostic Algorithms

### 1. In-Situ Armature Resistance ($R_a$) Estimation
- **Equation**: $V_{terminal} = E_b + I_a R_a$. During locked-rotor or initial sub-stall pulse ($\omega \approx 0 \implies E_b \approx 0$), $R_a = \frac{V_{terminal}}{I_a}$.
- **Hardware Topology Consideration**: INA219 bus voltage measures $V(V_{IN-})$ to `GND`. The estimation algorithm compensates for the low-side Darlington transistor drop ($V_{CE(sat),low} \approx 1.0\,\text{V}$) across the L298N:
  $$R_a \approx \frac{V_{bus} - 1.0\,\text{V}}{I_a}$$

### 2. Coast-Down Inertial Deceleration & Bearing Condition Index (BCI)
- **Torque Equation**: $J \frac{d\omega}{dt} = - (T_c + b\omega)$
- **Parameter Extraction**: Ordinary Least Squares (OLS) linear regression fits $-\frac{d\omega}{dt}$ against $\omega$ during pure high-Z inertial decay:
  - Intercept = $\frac{T_c}{J}$ (Coulomb dry friction torque)
  - Slope = $\frac{b}{J}$ (Viscous damping coefficient)
- **Bearing Condition Index (BCI)**:
  $$\text{BCI} = \min\left(100.0, \frac{\Delta t_{\text{coast}}}{\Delta t_{\text{nominal}}} \times 100.0\right)$$
  - $\text{BCI} \ge 85\%$: Healthy bearings
  - $60\% \le \text{BCI} < 85\%$: Bearing drag / elevated friction
  - $\text{BCI} < 60\%$: Critical mechanical binding / dry bearings

---

## 📊 Session CSV Data Logging

Every active session automatically generates a standardized timestamped engineering CSV log in the `logs/` directory:
- **Columns**: `Timestamp_ISO`, `Elapsed_Sec`, `Operating_Mode`, `Bus_Voltage_V`, `Current_mA`, `Speed_RPM`, `Omega_rad_s`, `Surface_Temp_C`, `Vibration_RMS_G`, `Calculated_Ra_Ohm`, `Coast_Decel_rad_s2`, `BCI_Pct`, `Health_Status`.

---

## 📜 License
MIT License. Open-source for industrial, academic, and research diagnostic benches.
