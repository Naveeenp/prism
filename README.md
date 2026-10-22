# DC Motor Real-Time Predictive Health Diagnostic System
### Dual Stack Architecture: Web Dashboard, Python Simulation & Arduino C/C++ Firmware (RP2040)

A modular embedded condition monitoring and predictive diagnostic system designed for DC motors.

---

## 1. Quick Start: Launching the Localhost Dashboard

Start the dashboard server and automatically launch your browser:

```bash
python run_dashboard.py
```

Then open your browser to:
**`http://localhost:8080`**

### Dashboard Features:
- **Spacious & Modern UI:** High-contrast, clean engineering layout without visual clutter, frosted glass, or AI jargon.
- **Human-Readable Diagnostics:** Clear, plain-English alert cards explaining the exact condition, measured reading vs safety limit, and mechanical risk (e.g., *"Motor Overheating (54.2°C): Surface temperature exceeds 50.0°C limit. Risk of winding insulation damage."*).
- **Audible Alerts:** Web Audio API sound chimes that play on critical alarms (with a prominent `🔊 Alert Audio: ON/MUTED` toggle in the header).
- **Interactive Simulation Fault Lab:** Dedicated panel with clickable buttons to simulate real-world physical failure modes:
  - 🔥 **Simulate Overheating** (pushes casing temperature above 50°C)
  - ⚡ **Simulate Overload / High Current** (surges current draw above 900 mA)
  - 〰️ **Simulate Severe Vibration** (triggers mechanical shock past 1.80 g RMS)
  - 🛑 **Simulate Rotor Jam / Stall** (locks shaft to 0 RPM while power is applied)
  - ⚙️ **Simulate Worn Bearing Drag** (accelerates coast-down deceleration)
  - 🔋 **Simulate Low Supply Voltage** (drops supply rail to 9.8 V)
  - 🔄 **Clear All Injected Faults** (returns to nominal physics)
- **Data Source Switcher:**
  - `Simulation Mode`: Runs the internal physics engine without hardware connected.
  - `Raspberry Pi Pico`: Connects to your physical Pico over USB serial (`COM3` @ 115200 baud).

---

## 2. Hardware Wiring (Raspberry Pi Pico H & Sensors)

| Subsystem / Peripheral | Sensor / Module | RP2040 Pin | Physical Pin | Function |
| :--- | :--- | :--- | :--- | :--- |
| **Shared I2C0 Bus** | SDA | **GP4** | Pin 6 | 400 kHz Fast-Mode I2C Data |
| | SCL | **GP5** | Pin 7 | 400 kHz Fast-Mode I2C Clock |
| | MPU-6050 | `0x68` | — | 6-DOF IMU (44 Hz DLPF configured, RMS Vibration) |
| | INA219 | `0x40` | — | High-side Current ($I_a$), Bus Voltage, Power |
| | SH1106 OLED | `0x3C` | — | **1.3-inch OLED** (128x64 display driven via Adafruit_SH110X) |
| **1-Wire Bus** | DS18B20 | **GP15** | Pin 20 | Digital casing temperature probe (**Requires 4.7kΩ pull-up**) |
| **Optical Encoder** | HC-89 / LM393 | **GP2** | Pin 4 | Falling-edge hardware IRQ (`PULSES_PER_REV = 4`) |
| **L298N Motor Driver** | ENA (PWM) | **GP10** | Pin 14 | **20 kHz Ultrasonic PWM** (silent motor drive) |
| | IN1 | **GP11** | Pin 15 | H-Bridge Direction 1 (GPIO Out) |
| | IN2 | **GP12** | Pin 16 | H-Bridge Direction 2 (GPIO Out) |
| **Power Rails** | 3.3V (OUT) | 3V3 | Pin 36 | Sensor logic power & pull-up supply |
| | GND | GND | Pins 3, 8, 13, etc. | Common ground with L298N and power supply |

---

## 3. Arduino IDE Firmware Setup

Firmware is located in [dc_motor_health_monitor/dc_motor_health_monitor.ino](file:///d:/DC%20Motor%20Health%20Monitoring/dc_motor_health_monitor/dc_motor_health_monitor.ino).

### Required Libraries (Install via Arduino Library Manager):
1. **Adafruit MPU6050** by *Adafruit*
2. **Adafruit INA219** by *Adafruit*
3. **OneWire** by *Paul Stoffregen*
4. **DallasTemperature** by *Miles Burton*
5. **Adafruit SH110X** by *Adafruit* (for 1.3" 128x64 OLED)

### Flashing to Pico:
1. Open [dc_motor_health_monitor.ino](file:///d:/DC%20Motor%20Health%20Monitoring/dc_motor_health_monitor/dc_motor_health_monitor.ino) in Arduino IDE.
2. Select **Raspberry Pi Pico** under Tools -> Board.
3. Click **Upload**.
4. Open the Web Dashboard at `http://localhost:8080`, switch the source tab to **Raspberry Pi Pico**, and click **Connect to Pico**.

---

## 4. Backend Security & Reliability Fixes Implemented
- **Threading Model:** `ThreadingHTTPServer` prevents Server-Sent Events (SSE) from blocking standard REST API requests.
- **Path Traversal Protection:** Static file resolution is strictly verified against `os.path.commonpath` to prevent unauthorized file reads.
- **Input Sanitization:** Speed values clamped to `0.0 - 100.0%`, mode and actions restricted to whitelists, and serial port strings validated against strict regex patterns.
- **Network Safety:** Server binds to `127.0.0.1` by default to avoid unintended local network exposure.
- **Socket Guard:** Robust exception handling for client disconnects (`ConnectionAbortedError`, `BrokenPipeError`) prevents server crashes.
