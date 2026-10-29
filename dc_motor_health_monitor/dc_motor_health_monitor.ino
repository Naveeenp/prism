/*
 ===============================================================================
 PMDC MOTOR HEALTH DIAGNOSTIC TEST BENCH - EMBEDDED FIRMWARE (ESP-32S)
 ===============================================================================
 Target Microcontroller: ESP-32S (38-Pin NodeMCU / ESP32-WROOM-32)
 Baud Rate:             115200 Baud (Clean JSON Lines / Protocol Handshake)
 Standards:             ISA-101 Industrial Telemetry & Machine Diagnostics
 ===============================================================================
 HARDWARE PIN CONNECTIONS & L298N SPECIFICATIONS:
   - L298N Dual H-Bridge Motor Driver:
       * ENA (Speed PWM)     : GPIO 14 (Hardware LEDC PWM, 1 kHz - high torque / clean switching)
                               NOTE: Remove the jumper on ENA to control via PWM!
       * IN1 (Direction 1)   : GPIO 27
       * IN2 (Direction 2)   : GPIO 26
       * COMMON GROUND       : Connect ESP32 GND to L298N GND and Motor PSU GND!
   - Optical Shaft Encoder (HC-89):
       * SIGNAL (D0/OUT)     : GPIO 18 (Interrupt on CHANGE, counting rising & falling transitions)
       * Encoder Disc        : 4-Slot Disc (PULSES_PER_REV = 4 slots * 2 edges = 8 edges/rev)
       * VCC                 : 3.3V or 5V (Ensure GND is shared)
   - I2C Bus (Fast-Mode 400 kHz):
       * SDA                 : GPIO 21
       * SCL                 : GPIO 22
       * Sensors on Bus      : INA219 (0x40), MPU-6050 (0x68), 1.3" SH1106 OLED (0x3C)
   - 1-Wire Digital Thermometer:
       * DATA                : GPIO 4 (DS18B20 with 4.7kΩ pull-up to 3.3V)
   - Onboard Status LED:
       * Heartbeat LED       : GPIO 2 (NodeMCU Onboard Blue LED)
 ===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS (ESP-32S)
// =============================================================================
#define PIN_MOTOR_ENA        14  // GPIO 14: L298N PWM Speed (LEDC)
#define PIN_MOTOR_IN1        27  // GPIO 27: L298N Direction 1
#define PIN_MOTOR_IN2        26  // GPIO 26: L298N Direction 2
#define PIN_ENCODER          18  // GPIO 18: HC-89 Optical Encoder IRQ
#define PIN_I2C_SDA          21  // GPIO 21: I2C SDA
#define PIN_I2C_SCL          22  // GPIO 22: I2C SCL
#define PIN_ONEWIRE          4   // GPIO 4:  DS18B20 1-Wire
#define PIN_LED_HEARTBEAT    2   // GPIO 2:  ESP32 Onboard LED

// =============================================================================
// CONFIGURATION CONSTANTS
// =============================================================================
// Optical encoder: 4 slots with CHANGE mode IRQ (both rising and falling edges) = 8 edges/rev
#define ENCODER_SLOTS_PER_REV 4
#define EDGES_PER_SLOT        2        // CHANGE mode triggers on rising & falling edges
#define PULSES_PER_REV        (ENCODER_SLOTS_PER_REV * EDGES_PER_SLOT) // 8 effective counts per rev

// 1 kHz provides high starting torque and easily passes through L298N optocouplers/buffers
#define PWM_FREQ_HZ           1000     
#define PWM_RESOLUTION_BITS   8        // 8-bit resolution (0 - 255)
#define PWM_LEDC_CHANNEL      0        // LEDC Channel 0
#define TELEMETRY_PERIOD_MS   100      // 10 Hz Telemetry Transmission Rate

// =============================================================================
// HARDWARE SENSOR & DISPLAY OBJECTS
// =============================================================================
Adafruit_MPU6050      mpu;
Adafruit_INA219       ina219(0x40);
OneWire               oneWire(PIN_ONEWIRE);
DallasTemperature     tempSensor(&oneWire);
Adafruit_SH1106G      display(128, 64, &Wire, -1);

// Sensor online detection flags
bool s_mpu_online   = false;
bool s_ina_online   = false;
bool s_oled_online  = false;

// Driver Master Arm / Enable Switch
bool s_driver_armed = false;

// =============================================================================
// VOLATILE INTERRUPT & TIMING VARIABLES
// =============================================================================
volatile uint32_t s_last_pulse_us     = 0;
volatile uint32_t s_pulse_interval_us = 0;
volatile uint32_t s_pulse_counter_window = 0;
volatile uint32_t s_total_pulses      = 0;

// Motor state variables
uint8_t  s_current_duty_8bit = 0;
String   s_current_mode      = "STANDBY";
float    s_rpm               = 0.0f;
float    s_latest_temp_c     = 25.0f;

// Non-blocking timer schedulers
uint32_t s_last_telemetry_ms  = 0;
uint32_t s_last_speed_calc_ms = 0;
uint32_t s_last_temp_req_ms   = 0;
uint32_t s_last_hb_toggle_ms  = 0;
uint32_t s_telemetry_seq      = 0;     // Incremental packet sequence counter
bool     s_hb_led_state       = false;

// Coast test state machine
enum CoastTestState { COAST_STATE_IDLE, COAST_STATE_SPINUP, COAST_STATE_COASTING };
CoastTestState s_coast_test_state = COAST_STATE_IDLE;
uint32_t s_coast_start_ms = 0;

// Controlled Wind-Down state machine (2.0s per 1% speed reduction)
bool     s_winddown_active   = false;
float    s_winddown_pct      = 0.0f;
uint32_t s_last_winddown_ms  = 0;

// =============================================================================
// OPTICAL ENCODER HARDWARE ISR (GPIO 18)
// =============================================================================
void IRAM_ATTR encoderISR() {
  uint32_t now_us = micros();
  uint32_t dt_us = now_us - s_last_pulse_us;

  // Software debounce: reject pulses faster than 150 us (> 20,000 RPM)
  if (dt_us > 150) {
    s_pulse_interval_us = dt_us;
    s_last_pulse_us = now_us;
    s_pulse_counter_window++;
    s_total_pulses++;
  }
}

// =============================================================================
// MOTOR ACTUATION UTILITY (L298N VIA ESP32 LEDC)
// =============================================================================
void setMotorDuty(uint8_t duty_8bit, bool forward = true) {
  s_current_duty_8bit = duty_8bit;

  // Safety interlock: if driver is not armed, force 0 duty and low outputs
  if (!s_driver_armed) {
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcWrite(PIN_MOTOR_ENA, 0);
    #else
      ledcWrite(PWM_LEDC_CHANNEL, 0);
    #endif
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    s_current_mode = "DISARMED";
    return;
  }

  if (duty_8bit > 0) {
    // 1. Establish Direction Logic
    if (forward) {
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
    } else {
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
    }

    // 2. Drive PWM to ENA (Full 0-255 range)
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcWrite(PIN_MOTOR_ENA, duty_8bit);
    #else
      ledcWrite(PWM_LEDC_CHANNEL, duty_8bit);
    #endif

    int pct = (int)((duty_8bit / 255.0f) * 100.0f);
    s_current_mode = "RUN_" + String(pct);
  } else {
    // 3. High-Z Coasting: ENA=0, IN1=0, IN2=0 (Zero dynamic braking)
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcWrite(PIN_MOTOR_ENA, 0);
    #else
      ledcWrite(PWM_LEDC_CHANNEL, 0);
    #endif
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    s_current_mode = "ARMED_IDLE";
  }
}

void setDriverPower(bool enable) {
  s_driver_armed = enable;
  if (!s_driver_armed) {
    setMotorDuty(0);
    s_current_mode = "DISARMED";
    Serial.println("ACK:DRIVER_POWER:OFF");
  } else {
    s_current_mode = "ARMED_IDLE";
    Serial.println("ACK:DRIVER_POWER:ON");
  }
}

// =============================================================================
// HYBRID SPEED CALCULATION (DUAL WINDOW + INTERVAL FOR ACCURATE RPM)
// =============================================================================
void calculateSpeed() {
  uint32_t now_ms = millis();
  uint32_t now_us = micros();
  uint32_t dt_ms = now_ms - s_last_speed_calc_ms;
  if (dt_ms == 0) return;
  s_last_speed_calc_ms = now_ms;

  // Atomically grab pulses counted in this measurement window
  noInterrupts();
  uint32_t pulses = s_pulse_counter_window;
  s_pulse_counter_window = 0;
  uint32_t last_pulse = s_last_pulse_us;
  uint32_t interval = s_pulse_interval_us;
  interrupts();

  uint32_t elapsed_since_pulse_us = now_us - last_pulse;

  // If no pulse received for > 400 ms, shaft has stopped
  if (elapsed_since_pulse_us > 400000 || last_pulse == 0) {
    s_rpm = 0.0f;
    return;
  }

  // Method 1: Pulse frequency count over window
  // RPM = (pulses / PPR) * (60 * 1000 / dt_ms)
  float pulse_rpm = ((float)pulses / (float)PULSES_PER_REV) * (60000.0f / (float)dt_ms);

  // Method 2: High-resolution pulse period interval
  // RPM = (60 * 1,000,000) / (PULSES_PER_REV * interval_us)
  // For PPR=8, numerator is 60,000,000 / 8 = 7,500,000.0f
  float period_rpm = (interval > 0) ? ((60000000.0f / (float)PULSES_PER_REV) / (float)interval) : 0.0f;

  float computed_rpm = 0.0f;
  if (pulses >= 2) {
    computed_rpm = pulse_rpm;
  } else if (pulses == 1 && period_rpm > 0.0f && period_rpm < 12000.0f) {
    computed_rpm = period_rpm;
  } else if (pulses == 0 && elapsed_since_pulse_us < 350000) {
    // Graceful coast-down tracking without synthetic 0.70 artificial collapse
    computed_rpm = s_rpm;
  }

  // Single-pole IIR filter for jitter suppression
  s_rpm = (0.75f * computed_rpm) + (0.25f * s_rpm);
  if (s_rpm < 15.0f && pulses == 0) {
    s_rpm = 0.0f;
  }
}

// =============================================================================
// IN-SITU ARMATURE RESISTANCE (Ra) PULSE TEST ROUTINE
// =============================================================================
// MEASUREMENT TOPOLOGY & SENSING LIMITATIONS:
// Series Path: L298N OUT1 -> INA219 VIN+ -> INA219 VIN- -> Motor(+) -> Motor(-) -> L298N OUT2
// 1. INA219 getBusVoltage_V() measures the single-ended potential at VIN- relative to GND.
// 2. The high-side rail is VIN+ = BusVoltage + (ShuntVoltage_mV / 1000.0).
// 3. Differential motor terminal voltage V_motor = V(Motor+) - V(Motor-) = V(VIN-) - V(OUT2).
//    During active forward drive, OUT2 sits above ground by the L298N low-side Darlington
//    saturation drop (Vce_sat_low ≈ 0.9V - 1.2V depending on collector current).
// 4. Thus, (V_bus / I_a) includes motor Ra + low-side switch resistance + cable losses.
//    The value is labeled as ESTIMATED_UNCALIBRATED until differential sensing is calibrated.
// =============================================================================
void executeRaPulseTest() {
  if (!s_driver_armed) {
    Serial.println("ERR:DRIVER_DISARMED:ENABLE_DRIVER_FIRST");
    return;
  }

  s_current_mode = "RA_TEST";

  // Ensure motor is stationary
  setMotorDuty(0);
  delay(150);

  // Apply calibrated short pulse (60ms @ 40% duty)
  setMotorDuty(102, true);

  float v_bus_sum = 0.0f;
  float v_shunt_sum = 0.0f;
  float i_sum = 0.0f;
  int valid_samples = 0;

  // Sample during the electrical steady-state window (20ms - 55ms)
  uint32_t pulse_start = millis();
  while (millis() - pulse_start < 60) {
    if (millis() - pulse_start >= 20) {
      if (s_ina_online) {
        float v_b = ina219.getBusVoltage_V();
        float v_s = ina219.getShuntVoltage_mV();
        float i = ina219.getCurrent_mA();
        if (i > 30.0f) {
          v_bus_sum += v_b;
          v_shunt_sum += v_s;
          i_sum += i;
          valid_samples++;
        }
      }
      delay(4);
    }
  }

  // Terminate pulse immediately to avoid rotor movement
  setMotorDuty(0);

  float v_bus_avg = valid_samples > 0 ? (v_bus_sum / valid_samples) : 12.0f;
  float v_shunt_avg = valid_samples > 0 ? (v_shunt_sum / valid_samples) : 0.0f;
  float v_in_plus_avg = v_bus_avg + (v_shunt_avg / 1000.0f);
  float i_avg = valid_samples > 0 ? (i_sum / valid_samples) : 0.0f;

  // Theoretical motor voltage estimation: subtract typical L298N low-side saturation drop (~1.0V)
  // if voltage is sufficient, else fall back to raw bus ratio
  float v_est_motor = (v_bus_avg > 1.2f) ? (v_bus_avg - 1.0f) : v_bus_avg;
  float ra_calc = (i_avg > 30.0f) ? (v_est_motor / (i_avg / 1000.0f)) : 0.0f;

  Serial.printf("ACK:MEASURE_RA:V_bus=%.2f,V_in+=%.2f,I=%.1f,Ra=%.3f,note=ESTIMATED_UNCALIBRATED\r\n",
                v_bus_avg, v_in_plus_avg, i_avg, ra_calc);
}

// =============================================================================
// LOCAL SH1106 OLED DISPLAY (1.3" 128x64)
// =============================================================================
void updateOLED(float v_bus, float i_ma, float temp_c, float vib_g) {
  if (!s_oled_online) return;

  display.clearDisplay();

  // Header Bar
  display.fillRect(0, 0, 128, 10, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(1);
  display.setCursor(4, 1);
  display.print("PRISM MOTOR BENCH");

  // Telemetry Rows
  display.setTextColor(SH110X_WHITE);

  // Line 1: RPM & Pulses
  display.setCursor(0, 14);
  display.printf("RPM: %-5.0f [%s]", s_rpm, s_current_mode.c_str());

  // Line 2: Bus Voltage & Armature Current
  display.setCursor(0, 26);
  display.printf("V: %4.1fV  I: %4.0fmA", v_bus, i_ma);

  // Line 3: Temp & Vibration
  display.setCursor(0, 38);
  display.printf("T: %4.1fC  V: %4.2fG", temp_c, vib_g);

  // Divider
  display.drawLine(0, 50, 127, 50, SH110X_WHITE);

  // Line 4: Status Indicator / Driver State
  display.setCursor(0, 53);
  if (!s_driver_armed) {
    display.print("PWR: [DISARMED]");
  } else if (temp_c > 50.0f || i_ma > 900.0f || vib_g > 1.50f) {
    display.print("PWR: ON  ! ALERT !");
  } else {
    display.print("PWR: ON  [HEALTHY]");
  }

  display.display();
}

// =============================================================================
// SERIAL COMMAND INTERPRETER & HANDSHAKE PROTOCOL
// =============================================================================
void processSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) continue;

    // Command 1: Driver Master Power Switch: CMD:DRIVER_POWER:<ON|OFF|1|0>
    if (cmd.startsWith("CMD:DRIVER_POWER:")) {
      String state = cmd.substring(17);
      state.toUpperCase();
      if (state == "ON" || state == "1") {
        setDriverPower(true);
      } else {
        setDriverPower(false);
      }
    }
    // Command 2: Speed Setpoint: CMD:SET_SPEED:<0-255>
    else if (cmd.startsWith("CMD:SET_SPEED:")) {
      if (!s_driver_armed) {
        Serial.println("ERR:DRIVER_DISARMED:ARM_DRIVER_FIRST");
      } else {
        int duty = cmd.substring(14).toInt();
        duty = constrain(duty, 0, 255);
        setMotorDuty((uint8_t)duty, true);
        Serial.printf("ACK:SET_SPEED:%d\r\n", duty);
      }
    }
    // Command 3: Emergency Stop: CMD:ESTOP
    else if (cmd.equalsIgnoreCase("CMD:ESTOP") || cmd.equalsIgnoreCase("STOP")) {
      s_winddown_active = false;
      setMotorDuty(0);
      s_coast_test_state = COAST_STATE_IDLE;
      Serial.println("ACK:ESTOP");
    }
    // Command 3B: Controlled Normal Wind Down: CMD:WIND_DOWN (2.0s per 1% reduction)
    else if (cmd.equalsIgnoreCase("CMD:WIND_DOWN") || cmd.equalsIgnoreCase("CMD:NORMAL_STOP")) {
      s_coast_test_state = COAST_STATE_IDLE;
      if (!s_driver_armed || s_current_duty_8bit == 0) {
        setMotorDuty(0);
        s_winddown_active = false;
        Serial.println("ACK:WIND_DOWN:COMPLETE");
      } else {
        s_winddown_active = true;
        s_winddown_pct = ((float)s_current_duty_8bit / 255.0f) * 100.0f;
        s_last_winddown_ms = millis();
        Serial.printf("ACK:WIND_DOWN:STARTED:pct=%.1f\r\n", s_winddown_pct);
      }
    }
    // Command 4: Automated Coast-Down Test: CMD:RUN_COAST_TEST
    else if (cmd.equalsIgnoreCase("CMD:RUN_COAST_TEST")) {
      if (!s_driver_armed) {
        Serial.println("ERR:DRIVER_DISARMED:ENABLE_DRIVER_FIRST");
      } else {
        s_coast_test_state = COAST_STATE_SPINUP;
        s_coast_start_ms = millis();
        setMotorDuty(191, true); // Ramp to 75% baseline speed
        Serial.println("ACK:RUN_COAST_TEST");
      }
    }
    // Command 5: Armature Resistance Pulse: CMD:MEASURE_RA
    else if (cmd.equalsIgnoreCase("CMD:MEASURE_RA")) {
      executeRaPulseTest();
    }
    // Command 6: Ping Heartbeat: CMD:PING
    else if (cmd.equalsIgnoreCase("CMD:PING")) {
      Serial.println("ACK:PONG");
    }
    else {
      Serial.printf("ERR:UNKNOWN_COMMAND:%s\r\n", cmd.c_str());
    }
  }
}

// =============================================================================
// SETUP INITIALIZATION
// =============================================================================
void setup() {
  // 1. Initialize Serial Port (115200 Baud)
  Serial.begin(115200);
  delay(100);

  // 2. Heartbeat LED Pin
  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_LED_HEARTBEAT, HIGH);

  // 3. L298N Direction & PWM Setup
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  // ESP32 LEDC PWM Setup (1 kHz, 8-bit resolution)
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_MOTOR_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcWrite(PIN_MOTOR_ENA, 0);
  #else
    ledcSetup(PWM_LEDC_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_ENA, PWM_LEDC_CHANNEL);
    ledcWrite(PWM_LEDC_CHANNEL, 0);
  #endif

  // 4. Optical Encoder on GPIO 18 (Input with internal pull-up)
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), encoderISR, CHANGE);

  // 5. I2C Bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Wire.setTimeOut(50);

  // Initialize INA219
  if (ina219.begin(&Wire)) {
    s_ina_online = true;
  }

  // Initialize MPU-6050
  if (mpu.begin(0x68, &Wire)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    s_mpu_online = true;
  }

  // Initialize 1.3" SH1106 OLED (0x3C)
  if (display.begin(0x3C, true)) {
    s_oled_online = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(8, 26);
    display.println("ESP32 PRISM BENCH");
    display.setCursor(8, 38);
    display.println("ONLINE - 115200");
    display.display();
    delay(300);
  }

  // 6. Initialize DS18B20 1-Wire Thermometer
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  tempSensor.requestTemperatures();
  s_last_temp_req_ms = millis();
  s_last_speed_calc_ms = millis();

  digitalWrite(PIN_LED_HEARTBEAT, LOW);
  Serial.println("ACK:BOOT:ESP32_PRISM_SYSTEM_READY");
}

// =============================================================================
// MAIN 10 HZ DETERMINISTIC CONTROL LOOP
// =============================================================================
void loop() {
  uint32_t now_ms = millis();

  // 1. Process Incoming Serial Commands
  processSerialCommands();

  // 1B. Service Normal Controlled Wind-Down (10% per second reduction)
  if (s_winddown_active) {
    if (now_ms - s_last_winddown_ms >= 1000) {
      s_last_winddown_ms = now_ms;
      if (s_winddown_pct > 10.0f) {
        s_winddown_pct -= 10.0f;
        uint8_t d = (uint8_t)roundf((s_winddown_pct / 100.0f) * 255.0f);
        setMotorDuty(d, true);
        Serial.printf("ACK:WIND_DOWN:STEP:pct=%.0f:duty=%d\r\n", s_winddown_pct, d);
      } else {
        s_winddown_pct = 0.0f;
        setMotorDuty(0);
        s_winddown_active = false;
        Serial.println("ACK:WIND_DOWN:COMPLETE");
      }
    }
  }

  // 2. Service Coast-Down Test Automation
  if (s_coast_test_state == COAST_STATE_SPINUP) {
    if (now_ms - s_coast_start_ms >= 2000) {
      // 2 seconds spin-up complete: instantly cut PWM to high-Z coasting
      setMotorDuty(0);
      s_coast_test_state = COAST_STATE_COASTING;
      Serial.println("ACK:COAST_CUTOFF");
    }
  } else if (s_coast_test_state == COAST_STATE_COASTING) {
    if (s_rpm < 15.0f || (now_ms - s_coast_start_ms >= 6000)) {
      s_coast_test_state = COAST_STATE_IDLE;
      Serial.printf("ACK:COAST_STOP:dt_ms=%lu\r\n", now_ms - s_coast_start_ms - 2000);
    }
  }

  // 3. Heartbeat LED (1 Hz Blinking)
  if (now_ms - s_last_hb_toggle_ms >= 500) {
    s_last_hb_toggle_ms = now_ms;
    s_hb_led_state = !s_hb_led_state;
    digitalWrite(PIN_LED_HEARTBEAT, s_hb_led_state ? HIGH : LOW);
  }

  // 4. Non-blocking DS18B20 Temperature Sampling (~750ms interval)
  if (now_ms - s_last_temp_req_ms >= 750) {
    float read_c = tempSensor.getTempCByIndex(0);
    if (read_c > -50.0f && read_c < 125.0f) {
      s_latest_temp_c = read_c;
    }
    tempSensor.requestTemperatures();
    s_last_temp_req_ms = now_ms;
  }

  // 5. Periodic 10 Hz Telemetry Dispatch
  if (now_ms - s_last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    s_last_telemetry_ms = now_ms;
    s_telemetry_seq++;

    // A. Calculate Speed (Dual window + interval)
    calculateSpeed();

    // B. Read INA219
    float v_bus = 0.0f;
    float i_ma  = 0.0f;
    if (s_ina_online) {
      v_bus = ina219.getBusVoltage_V();
      i_ma  = ina219.getCurrent_mA();
      if (i_ma < 0.0f) i_ma = 0.0f;
    }

    // C. Read MPU-6050 Vibration
    float vib_g = 0.02f;
    if (s_mpu_online) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float ax_g = a.acceleration.x / 9.80665f;
      float ay_g = a.acceleration.y / 9.80665f;
      float az_g = a.acceleration.z / 9.80665f;
      // Sensor mounted parallel with pins pointing upward (towards sky): static gravity is along +Z (~1.0G)
      float dyn_z = az_g - 1.0f;
      vib_g = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (dyn_z * dyn_z));
    }

    // D. Update Local 1.3" OLED Display
    updateOLED(v_bus, i_ma, s_latest_temp_c, vib_g);

    // E. Emit Strict JSON Telemetry Frame Over Serial
    // Enhanced Schema: mode, v_bus, i_ma, rpm, temp_c, vib_g, ts_ms, driver_pwr, duty, seq, pulses, ina_ok, mpu_ok
    Serial.printf("{\"mode\": \"%s\", \"v_bus\": %.2f, \"i_ma\": %.1f, \"rpm\": %.1f, \"temp_c\": %.1f, \"vib_g\": %.2f, \"ts_ms\": %lu, \"driver_pwr\": %d, \"duty\": %d, \"seq\": %lu, \"pulses\": %lu, \"ina_ok\": %d, \"mpu_ok\": %d}\r\n",
                  s_current_mode.c_str(),
                  v_bus,
                  i_ma,
                  s_rpm,
                  s_latest_temp_c,
                  vib_g,
                  now_ms,
                  s_driver_armed ? 1 : 0,
                  s_current_duty_8bit,
                  s_telemetry_seq,
                  s_total_pulses,
                  s_ina_online ? 1 : 0,
                  s_mpu_online ? 1 : 0);
  }
}
