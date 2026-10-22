/*
 ===============================================================================
 DC MOTOR REAL-TIME PREDICTIVE HEALTH DIAGNOSTIC SYSTEM
 ===============================================================================
 Target Microcontroller: Raspberry Pi Pico H (RP2040)
 Development Platform:   Arduino IDE (using Earle Philhower's arduino-pico core)
 Standard Libraries (Install via Arduino IDE Library Manager):
   1. "Adafruit MPU6050"   by Adafruit (includes Adafruit_Sensor)
   2. "Adafruit INA219"    by Adafruit
   3. "OneWire"            by Paul Stoffregen
   4. "DallasTemperature"  by Miles Burton
   5. "Adafruit SH110X"    by Adafruit (For 1.3" OLED: Adafruit_SH1106G 128x64)
      (Alternative: Adafruit_SSD1306 for 0.96" OLEDs)
 ===============================================================================
 HARDWARE PIN CONNECTIONS (RP2040):
   - I2C0 Bus (Shared, 400kHz):
       * SDA = GP4 (Pico Pin 6)
       * SCL = GP5 (Pico Pin 7)
       * MPU-6050 (0x68), INA219 (0x40), 1.3" SH1106 OLED (0x3C)
   - 1-Wire Bus:
       * DATA = GP15 (Pico Pin 20) with 4.7k ohm pull-up to 3.3V
       * Sensor: DS18B20 digital thermometer probe
   - Optical Encoder (HC-89 / LM393):
       * SIGNAL = GP2 (Pico Pin 4) - Hardware Falling-Edge Interrupt
       * Disc Specification: PULSES_PER_REV = 4
   - L298N Dual H-Bridge Motor Driver:
       * ENA = GP10 (Pico Pin 14) -> 20kHz Hardware PWM Speed Control
       * IN1 = GP11 (Pico Pin 15) -> Direction Pin 1
       * IN2 = GP12 (Pico Pin 16) -> Direction Pin 2
 ===============================================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS
// =============================================================================
#define PIN_I2C_SDA         4   // GP4
#define PIN_I2C_SCL         5   // GP5
#define PIN_ONEWIRE         15  // GP15 (DS18B20 1-Wire)
#define PIN_ENCODER         2   // GP2  (Optical speed sensor IRQ)
#define PIN_MOTOR_ENA       10  // GP10 (L298N PWM Speed)
#define PIN_MOTOR_IN1       11  // GP11 (L298N Direction 1)
#define PIN_MOTOR_IN2       12  // GP12 (L298N Direction 2)

// =============================================================================
// ENCODER & MECHANICAL CONSTANTS
// =============================================================================
#define PULSES_PER_REV      4   // 4-slot DIY optical encoder disc
#define PWM_FREQUENCY_HZ    20000 // 20 kHz ultrasonic PWM (eliminates motor whine)
#define PWM_RESOLUTION_BITS 10    // 10-bit PWM range (0 - 1023)

// =============================================================================
// PREDICTIVE HEALTH DIAGNOSTIC THRESHOLDS
// =============================================================================
#define FAULT_TEMP_WARN_C       45.0f
#define FAULT_TEMP_ALARM_C      50.0f  // Casing temperature > 50.0 deg C
#define FAULT_CURRENT_WARN_MA   750.0f
#define FAULT_CURRENT_ALARM_MA  900.0f // Continuous current > 900 mA
#define FAULT_VIB_WARN_G        1.40f
#define FAULT_VIB_ALARM_G       1.80f  // Excessive RMS vibration > 1.80 g
#define FAULT_STALL_TIME_MS     1500   // Stall: PWM >= 25% but RPM < 30 for 1.5s
#define FAULT_DECEL_FATIGUE     -90.0f // Angular decel (rad/s^2) indicating bearing drag

#define TELEMETRY_INTERVAL_MS   200    // 5 Hz non-blocking loop

// =============================================================================
// HARDWARE DRIVER INSTANCES (ARDUINO IDE ECOSYSTEM)
// =============================================================================
Adafruit_MPU6050   mpu;
Adafruit_INA219    ina219(0x40);
OneWire            oneWire(PIN_ONEWIRE);
DallasTemperature  tempSensor(&oneWire);

// 1.3-inch OLED is 128x64 driven by SH1106
Adafruit_SH1106G   display(128, 64, &Wire, -1);

// =============================================================================
// VOLATILE INTERRUPT & TIMING STATE VARIABLES
// =============================================================================
volatile uint32_t s_last_pulse_us = 0;
volatile uint32_t s_pulse_interval_us = 0;
volatile uint32_t s_pulse_counter = 0;

float s_rpm = 0.0f;
float s_omega_rad_s = 0.0f;
float s_last_omega_rad_s = 0.0f;
float s_deceleration_rad_s2 = 0.0f;
float s_current_pwm_pct = 0.0f;

// Coast-down Deceleration State
enum CoastState { COAST_IDLE, COAST_RECORDING };
CoastState s_coast_state = COAST_IDLE;
uint32_t s_coast_start_us = 0;
float s_coast_start_omega = 0.0f;
bool s_alarm_bearing_drag = false;

// Stall Detection Timer
uint32_t s_stall_start_ms = 0;
bool s_alarm_stall = false;

// Non-blocking loop scheduler
uint32_t s_last_telemetry_tick_ms = 0;
uint32_t s_last_temp_req_ms = 0;
float s_latest_temp_c = 25.0f;
bool s_temp_conversion_pending = false;

// Flag indicating if physical sensors acknowledged
bool s_mpu_online = false;
bool s_ina_online = false;
bool s_oled_online = false;

// =============================================================================
// OPTICAL ENCODER HARDWARE INTERRUPT SERVICE ROUTINE (FALLING EDGE)
// =============================================================================
void encoderISR() {
  uint32_t now_us = micros();
  uint32_t delta_us = now_us - s_last_pulse_us;

  // Software debounce filter: discard pulses faster than 400 us (> 37,000 RPM)
  if (delta_us > 400) {
    s_pulse_interval_us = delta_us;
    s_last_pulse_us = now_us;
    s_pulse_counter++;
  }
}

// =============================================================================
// MOTOR CONTROLLER WRAPPER FUNCTIONS (L298N)
// =============================================================================
void setMotorSpeed(float pct) {
  s_current_pwm_pct = constrain(pct, 0.0f, 100.0f);

  if (s_current_pwm_pct > 0.0f) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    // Map 0-100% to 10-bit PWM (0 - 1023)
    int duty = (int)((s_current_pwm_pct / 100.0f) * 1023.0f);
    analogWrite(PIN_MOTOR_ENA, duty);
  } else {
    // Coast-down: PWM = 0, H-Bridge inputs low
    analogWrite(PIN_MOTOR_ENA, 0);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  }
}

// =============================================================================
// ENCODER SPEED & COAST-DOWN DECELERATION CALCULATION ROUTINE
// =============================================================================
void updateSpeedAndDeceleration(float dt_sec) {
  uint32_t now_us = micros();
  uint32_t elapsed_since_pulse_us = now_us - s_last_pulse_us;

  // If no pulse received for > 250ms, motor is stopped
  if (elapsed_since_pulse_us > 250000 || s_pulse_interval_us == 0) {
    s_rpm = 0.0f;
    s_omega_rad_s = 0.0f;
  } else {
    // RPM = (60 * 1,000,000) / (PPR * interval_us) -> with PPR=4: 15,000,000 / interval
    float inst_rpm = 15000000.0f / (float)s_pulse_interval_us;
    // Single-pole IIR filter to eliminate discrete pulse ripple
    s_rpm = (0.75f * inst_rpm) + (0.25f * s_rpm);
    s_omega_rad_s = s_rpm * (2.0f * (float)PI / 60.0f);
  }

  // Calculate angular acceleration / deceleration (rad/s^2)
  if (dt_sec > 0.0f) {
    s_deceleration_rad_s2 = (s_omega_rad_s - s_last_omega_rad_s) / dt_sec;
    s_last_omega_rad_s = s_omega_rad_s;
  }

  // COAST-DOWN EDGE DIAGNOSTICS:
  // When motor is coasting (PWM = 0), record angular deceleration to detect mechanical drag
  if (s_current_pwm_pct <= 0.0f) {
    if (s_coast_state == COAST_IDLE && s_omega_rad_s > 40.0f) {
      s_coast_state = COAST_RECORDING;
      s_coast_start_us = now_us;
      s_coast_start_omega = s_omega_rad_s;
    } else if (s_coast_state == COAST_RECORDING) {
      uint32_t coast_elapsed_us = now_us - s_coast_start_us;
      if (coast_elapsed_us >= 400000) { // 400ms decay observation window
        float coast_dt = (float)coast_elapsed_us / 1000000.0f;
        float net_alpha = (s_omega_rad_s - s_coast_start_omega) / coast_dt;
        s_deceleration_rad_s2 = net_alpha;
        s_coast_state = COAST_IDLE;

        if (net_alpha < FAULT_DECEL_FATIGUE) {
          s_alarm_bearing_drag = true;
        }
      }
    }
  } else {
    s_coast_state = COAST_IDLE;
    s_alarm_bearing_drag = false;
  }
}

// =============================================================================
// OLED DISPLAY RENDERER (1.3" SH1106 / 128x64)
// =============================================================================
void renderOLED(float volts, float current_ma, float temp_c, float vib_rms, const char* status_str) {
  if (!s_oled_online) return;

  display.clearDisplay();

  // 1. Inverted Header Bar
  display.fillRect(0, 0, 128, 10, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(1);
  display.setCursor(6, 1);
  display.print("DC-MOTOR DIAGNOSTIC");

  // 2. Telemetry Rows
  display.setTextColor(SH110X_WHITE);

  // Line 1: RPM & PWM
  display.setCursor(0, 13);
  display.printf("RPM: %-4d  (%d%%)", (int)s_rpm, (int)s_current_pwm_pct);

  // Line 2: Voltage & Current
  display.setCursor(0, 23);
  display.printf("%4.1fV   %4dmA", volts, (int)current_ma);

  // Line 3: Temperature & Vibration RMS
  display.setCursor(0, 33);
  display.printf("T:%3.0fC  V:%3.2fg", temp_c, vib_rms);

  // Line 4: Coast Deceleration
  display.setCursor(0, 43);
  display.printf("dw/dt: %4.1f r/s2", s_deceleration_rad_s2);

  // Divider Line
  display.drawLine(0, 52, 127, 52, SH110X_WHITE);

  // 3. Status Banner
  if (strcmp(status_str, "ALERT") == 0) {
    display.fillRect(0, 54, 128, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(14, 55);
    display.print("! FAULT ALARM !");
  } else if (strcmp(status_str, "WARNING") == 0) {
    display.setTextColor(SH110X_WHITE);
    display.setCursor(14, 55);
    display.print("STATUS: ELEVATED");
  } else {
    display.setTextColor(SH110X_WHITE);
    display.setCursor(18, 55);
    display.print("HEALTH: NORMAL");
  }

  display.display();
}

// =============================================================================
// ARDUINO SETUP
// =============================================================================
void setup() {
  // 1. Initialize Serial Port (USB CDC)
  Serial.begin(115200);
  delay(1000);

  Serial.println("\r\n==================================================");
  Serial.println("  DC MOTOR REAL-TIME HEALTH DIAGNOSTIC SYSTEM     ");
  Serial.println("  Raspberry Pi Pico H (Arduino Core / C++)       ");
  Serial.println("==================================================");

  // 2. Initialize Shared I2C0 Bus on GP4 (SDA) / GP5 (SCL)
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz Fast-Mode

  // 3. Initialize Optical Encoder on GP2
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoderISR, FALLING);

  // 4. Initialize L298N Motor Driver Pins
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWriteFreq(PWM_FREQUENCY_HZ);      // 20 kHz Ultrasonic PWM
  analogWriteResolution(PWM_RESOLUTION_BITS); // 10-bit (0-1023)
  analogWrite(PIN_MOTOR_ENA, 0);

  // 5. Initialize DS18B20 1-Wire Digital Thermometer
  tempSensor.begin();
  tempSensor.setWaitForConversion(false); // Non-blocking asynchronous conversion
  tempSensor.requestTemperatures();
  s_last_temp_req_ms = millis();
  s_temp_conversion_pending = true;

  // 6. Initialize MPU-6050 IMU
  if (mpu.begin(0x68, &Wire)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // 44 Hz DLPF suppresses aliasing
    s_mpu_online = true;
    Serial.println("[MPU-6050] Initialized successfully at 0x68 (44Hz DLPF)");
  } else {
    Serial.println("[MPU-6050] Warning: Sensor not detected on I2C0!");
  }

  // 7. Initialize INA219 Current/Power Monitor
  if (ina219.begin(&Wire)) {
    s_ina_online = true;
    Serial.println("[INA219] Initialized successfully at 0x40 (32V/2A Range)");
  } else {
    Serial.println("[INA219] Warning: Sensor not detected on I2C0!");
  }

  // 8. Initialize 1.3-inch SH1106 OLED Display (0x3C)
  if (display.begin(0x3C, true)) {
    s_oled_online = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(4, 24);
    display.println("DC MOTOR SYSTEM");
    display.setCursor(4, 36);
    display.println("FIRMWARE READY");
    display.display();
    delay(500);
    Serial.println("[SH1106] 1.3\" OLED Display Initialized successfully");
  } else {
    Serial.println("[SH1106] Warning: OLED Display not detected on 0x3C!");
  }

  Serial.println("==================================================");
  Serial.println("System Running. Type 'HELP' for serial commands.\r\n");
}

// =============================================================================
// SERIAL COMMAND PROCESSOR
// =============================================================================
void processSerialInput() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("SPEED ")) {
      float spd = cmd.substring(6).toFloat();
      setMotorSpeed(spd);
      Serial.printf(">> [COMMAND] Motor Speed set to %.1f%%\r\n", spd);
    } else if (cmd.equalsIgnoreCase("STOP")) {
      setMotorSpeed(0.0f);
      Serial.println(">> [COMMAND] Motor Coast-Down Stop Initiated.");
    } else if (cmd.equalsIgnoreCase("HELP")) {
      Serial.println("\r\n--- SERIAL COMMANDS ---");
      Serial.println("  SPEED <0-100> : Command target PWM duty cycle percentage");
      Serial.println("  STOP          : Initiate motor coast-down stop");
      Serial.println("-----------------------\r\n");
    }
  }
}

// =============================================================================
// ARDUINO MAIN LOOP (DETERMINISTIC 5 HZ CONTROL LOOP)
// =============================================================================
void loop() {
  uint32_t now_ms = millis();

  // 1. Service Serial Console Commands
  processSerialInput();

  // 2. Periodic Telemetry, Diagnostics & Display Refresh (5 Hz / 200ms)
  if ((now_ms - s_last_telemetry_tick_ms) >= TELEMETRY_INTERVAL_MS) {
    float dt_sec = (float)(now_ms - s_last_telemetry_tick_ms) / 1000.0f;
    s_last_telemetry_tick_ms = now_ms;

    // A. Update Optical Speed & Deceleration Analytics
    updateSpeedAndDeceleration(dt_sec);

    // B. Read INA219 Voltage and Current
    float bus_volts = 0.0f;
    float current_ma = 0.0f;
    float power_mw = 0.0f;
    if (s_ina_online) {
      bus_volts = ina219.getBusVoltage_V();
      current_ma = ina219.getCurrent_mA();
      power_mw = ina219.getPower_mW();
      if (current_ma < 0.0f) current_ma = 0.0f;
    }

    // C. Read MPU-6050 Accelerometer & Calculate Vibration RMS
    float vib_rms = 0.0f;
    if (s_mpu_online) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      // a.acceleration values are in m/s^2. Convert to g:
      float ax_g = a.acceleration.x / 9.80665f;
      float ay_g = a.acceleration.y / 9.80665f;
      float az_g = a.acceleration.z / 9.80665f;

      // Compensate for 1.0g Earth gravity on Z-axis
      float dyn_z = (az_g > 0.0f) ? (az_g - 1.0f) : (az_g + 1.0f);
      vib_rms = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (dyn_z * dyn_z));
    }

    // D. Non-blocking DS18B20 Temperature Sampling (Takes ~750ms for 12-bit)
    if (s_temp_conversion_pending && (now_ms - s_last_temp_req_ms >= 750)) {
      float read_temp = tempSensor.getTempCByIndex(0);
      if (read_temp > -55.0f && read_temp < 125.0f) {
        s_latest_temp_c = read_temp;
      }
      tempSensor.requestTemperatures(); // Queue next conversion
      s_last_temp_req_ms = now_ms;
    }

    // E. Evaluate Multi-Tier Fault & Anomaly Triggers
    bool alarm_temp = s_latest_temp_c >= FAULT_TEMP_ALARM_C;
    bool alarm_current = current_ma >= FAULT_CURRENT_ALARM_MA;
    bool alarm_vib = vib_rms >= FAULT_VIB_ALARM_G;

    // Rotor Stall Detection: Commanded PWM >= 25% but RPM < 30 for > 1.5s
    if (s_current_pwm_pct >= 25.0f && s_rpm < 30.0f) {
      if (s_stall_start_ms == 0) {
        s_stall_start_ms = now_ms;
      } else if ((now_ms - s_stall_start_ms) >= FAULT_STALL_TIME_MS) {
        s_alarm_stall = true;
      }
    } else {
      s_stall_start_ms = 0;
      s_alarm_stall = false;
    }

    const char* status_str = "NORMAL";
    if (alarm_temp || alarm_current || alarm_vib || s_alarm_stall) {
      status_str = "ALERT";
    } else if (s_latest_temp_c >= FAULT_TEMP_WARN_C || current_ma >= FAULT_CURRENT_WARN_MA || vib_rms >= FAULT_VIB_WARN_G || s_alarm_bearing_drag) {
      status_str = "WARNING";
    }

    // F. Render Telemetry to 1.3" OLED Display
    renderOLED(bus_volts, current_ma, s_latest_temp_c, vib_rms, status_str);

    // G. Transmit Real-Time Formatted JSON Telemetry Packet over Serial
    Serial.printf("{\"time_ms\":%lu,\"pwm_pct\":%.1f,\"rpm\":%.1f,\"volts\":%.2f,"
                  "\"curr_ma\":%.1f,\"power_mw\":%.1f,\"temp_c\":%.2f,\"vib_rms_g\":%.3f,"
                  "\"decel_rad_s2\":%.2f,\"status\":\"%s\","
                  "\"alarms\":{\"overtemp\":%s,\"overcurrent\":%s,\"vibration\":%s,\"stall\":%s,\"bearing_drag\":%s}}\r\n",
                  now_ms,
                  s_current_pwm_pct,
                  s_rpm,
                  bus_volts,
                  current_ma,
                  power_mw,
                  s_latest_temp_c,
                  vib_rms,
                  s_deceleration_rad_s2,
                  status_str,
                  alarm_temp ? "true" : "false",
                  alarm_current ? "true" : "false",
                  alarm_vib ? "true" : "false",
                  s_alarm_stall ? "true" : "false",
                  s_alarm_bearing_drag ? "true" : "false");
  }
}
