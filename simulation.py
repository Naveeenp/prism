#!/usr/bin/env python3
"""
================================================================================
DC MOTOR REAL-TIME PREDICTIVE HEALTH DIAGNOSTIC SYSTEM - SIMULATION ENGINE
================================================================================
Platform: Python 3 (Standalone, zero external dependencies required)
Features:
  - Electromechanical motor physics with moment of inertia & friction decay
  - Dynamic thermal curve modeling (I^2 * R heating and cooling dissipation)
  - Multi-harmonic vibration generation with rotational frequency & Gaussian noise
  - Interactive Fault Injection: Overheat, High Current, Rotor Jam/Stall,
    Bearing Drag/Fatigue, High Vibration, and Voltage Sag
================================================================================
"""

import sys
import time
import math
import random

# Base Physical Parameters
MOTOR_MAX_NO_LOAD_RPM    = 3200.0 # Max RPM at 100% PWM
PULSES_PER_REV           = 4      # 4-slot optical encoder disc
TAU_ACCEL_SEC            = 0.65   # Rotational acceleration time constant
TAU_COAST_SEC            = 1.35   # Inertial friction spin-down decay constant
NOMINAL_BUS_VOLTAGE_V    = 12.0   # 12V DC power source
SOURCE_IMPEDANCE_OHMS    = 0.45   # Internal resistance causing voltage sag
BASE_IDLE_CURRENT_MA     = 75.0   # Quiescent no-load motor core current
FULL_LOAD_CURRENT_MA     = 480.0  # Armature load current at 100% duty (nominal safe operating range)
THERMAL_TIME_CONST_SEC   = 45.0   # Casing thermal dissipation time constant

# Diagnostic Thresholds
FAULT_TEMP_WARN_C        = 45.0
FAULT_TEMP_ALARM_C       = 50.0   # Over-temperature alarm threshold
FAULT_CURRENT_WARN_MA    = 750.0
FAULT_CURRENT_ALARM_MA   = 900.0  # Over-current alarm threshold
FAULT_VIB_WARN_G         = 1.40
FAULT_VIB_ALARM_G        = 1.80   # Excessive vibration alarm threshold
FAULT_DECEL_FATIGUE      = -90.0  # Angular decel (rad/s^2) indicating drag
FAULT_STALL_TIME_SEC     = 1.5    # Stall timeout window


class DCMotorPhysicsSimulator:
    def __init__(self):
        # Fault Injection Flags
        self.fault_overheat = False
        self.fault_overcurrent = False
        self.fault_vibration = False
        self.fault_stall = False
        self.fault_bearing_drag = False
        self.fault_voltage_drop = False
        
        # Adjustable Simulation Parameters
        self.ambient_temp_c = 25.0
        self.load_adder_ma = 0.0
        self.vib_multiplier = 1.0

        self.reset()

    def reset(self):
        self.elapsed_time_s = 0.0
        self.omega_rad_s = 0.0
        self.last_omega_rad_s = 0.0
        self.deceleration_rad_s2 = 0.0
        self.casing_temp_c = self.ambient_temp_c
        self.commanded_pwm_pct = 0.0
        self.is_coasting = False
        self.stall_timer_s = 0.0

        # Coast-down evaluation
        self.coast_eval_active = False
        self.coast_start_time = 0.0
        self.coast_start_omega = 0.0
        self.bearing_drag_alarm = False

    def set_pwm(self, pwm_pct):
        """Set commanded PWM speed duty (0.0 to 100.0%)."""
        self.commanded_pwm_pct = max(0.0, min(100.0, float(pwm_pct)))

    def set_faults(self, faults_dict):
        """Update fault injection parameters dynamically."""
        if "overheat" in faults_dict:
            self.fault_overheat = bool(faults_dict["overheat"])
        if "overcurrent" in faults_dict:
            self.fault_overcurrent = bool(faults_dict["overcurrent"])
        if "vibration" in faults_dict:
            self.fault_vibration = bool(faults_dict["vibration"])
        if "stall" in faults_dict:
            self.fault_stall = bool(faults_dict["stall"])
        if "bearing_drag" in faults_dict:
            self.fault_bearing_drag = bool(faults_dict["bearing_drag"])
        if "voltage_drop" in faults_dict:
            self.fault_voltage_drop = bool(faults_dict["voltage_drop"])
        if "load_adder_ma" in faults_dict:
            self.load_adder_ma = max(0.0, min(1000.0, float(faults_dict["load_adder_ma"])))
        if "vib_multiplier" in faults_dict:
            self.vib_multiplier = max(0.5, min(10.0, float(faults_dict["vib_multiplier"])))
        if "ambient_temp" in faults_dict:
            self.ambient_temp_c = max(10.0, min(50.0, float(faults_dict["ambient_temp"])))

    def clear_all_faults(self):
        """Reset all fault injection flags to nominal state."""
        self.fault_overheat = False
        self.fault_overcurrent = False
        self.fault_vibration = False
        self.fault_stall = False
        self.fault_bearing_drag = False
        self.fault_voltage_drop = False
        self.load_adder_ma = 0.0
        self.vib_multiplier = 1.0

    def step(self, dt=0.10):
        """Advance the physics model by dt seconds and return telemetry."""
        if dt <= 0:
            dt = 0.10
        self.elapsed_time_s += dt

        # ----------------------------------------------------------------------
        # 1. Rotational Mechanics & Inertial Coast-Down
        # ----------------------------------------------------------------------
        if self.fault_stall:
            # Rotor is jammed/stalled: speed forced to 0 regardless of PWM
            self.omega_rad_s = 0.0
            self.is_coasting = False
            self.deceleration_rad_s2 = 0.0
            current_rpm = 0.0
        else:
            base_target = (self.commanded_pwm_pct / 100.0) * (MOTOR_MAX_NO_LOAD_RPM * 2.0 * math.pi / 60.0)
            # Bearing drag friction penalty reduces running RPM
            if self.fault_bearing_drag and base_target > 0.0:
                target_omega = base_target * 0.72 # 28% speed drop under drag
            else:
                target_omega = base_target

            if self.commanded_pwm_pct <= 0.0:
                self.is_coasting = True
                # If bearing drag fault is active, decay rate is much faster
                tau_decay = 0.35 if self.fault_bearing_drag else TAU_COAST_SEC
                decay = math.exp(-dt / tau_decay)
                self.omega_rad_s *= decay
                if self.omega_rad_s < 0.5:
                    self.omega_rad_s = 0.0
            else:
                self.is_coasting = False
                alpha = 1.0 - math.exp(-dt / TAU_ACCEL_SEC)
                self.omega_rad_s += (target_omega - self.omega_rad_s) * alpha

            # Angular acceleration / deceleration (rad/s^2)
            self.deceleration_rad_s2 = (self.omega_rad_s - self.last_omega_rad_s) / dt
            self.last_omega_rad_s = self.omega_rad_s
            current_rpm = self.omega_rad_s * 60.0 / (2.0 * math.pi)

        # Optical pulse interval calculation
        if current_rpm > 5.0:
            pulse_interval_us = int(15000000.0 / current_rpm)
            pulses_in_window = int((current_rpm / 60.0) * PULSES_PER_REV * dt)
        else:
            pulse_interval_us = 0
            pulses_in_window = 0

        # ----------------------------------------------------------------------
        # 2. Electrical Current & Voltage Model
        # ----------------------------------------------------------------------
        inrush_current = 0.0
        if self.deceleration_rad_s2 > 0.0:
            # Soften and clamp inrush current transient during speed steps
            inrush_current = min(85.0, self.deceleration_rad_s2 * 0.25)

        if self.fault_stall and self.commanded_pwm_pct > 0.0:
            # Stall causes huge current surge (direct armature DC resistance draw)
            current_ma = 1180.0 + random.gauss(0.0, 10.0)
        elif self.commanded_pwm_pct <= 0.0 and current_rpm < 10.0:
            current_ma = 18.0 + random.gauss(0.0, 1.0) # Quiescent idle
        else:
            load_ma = FULL_LOAD_CURRENT_MA * (self.commanded_pwm_pct / 100.0)
            bearing_adder = 160.0 if self.fault_bearing_drag else 0.0
            current_ma = BASE_IDLE_CURRENT_MA + load_ma + inrush_current + self.load_adder_ma + bearing_adder + random.gauss(0.0, 4.0)

        # Fault Injection: Overcurrent
        if self.fault_overcurrent:
            current_ma += 420.0 # Force current well past 950mA alarm threshold

        current_ma = max(0.0, current_ma)

        # Supply Voltage & Sag
        base_v = 9.2 if self.fault_voltage_drop else NOMINAL_BUS_VOLTAGE_V
        bus_voltage_v = base_v - ((current_ma / 1000.0) * SOURCE_IMPEDANCE_OHMS)
        bus_voltage_v += random.gauss(0.0, 0.02)
        power_mw = bus_voltage_v * current_ma

        # ----------------------------------------------------------------------
        # 3. Thermal Dissipation Model
        # ----------------------------------------------------------------------
        current_ratio = current_ma / 1000.0
        delta_t_max = 3.0 + (36.0 * (current_ratio ** 2))
        
        # Fault Injection: Thermal Runaway
        if self.fault_overheat:
            delta_t_max += 32.0 # Elevate equilibrium temperature above 57°C
            # Accelerated thermal time constant during simulated overheat
            thermal_step = 1.0 - math.exp(-dt / 6.0)
        else:
            thermal_step = 1.0 - math.exp(-dt / THERMAL_TIME_CONST_SEC)

        target_temp = self.ambient_temp_c + delta_t_max
        self.casing_temp_c += (target_temp - self.casing_temp_c) * thermal_step
        measured_temp_c = self.casing_temp_c + random.gauss(0.0, 0.05)

        # ----------------------------------------------------------------------
        # 4. Harmonic Vibration Signature
        # ----------------------------------------------------------------------
        rotational_freq_hz = max(1.0, current_rpm / 60.0)
        omega_shaft = 2.0 * math.pi * rotational_freq_hz
        speed_ratio = current_rpm / MOTOR_MAX_NO_LOAD_RPM

        a0 = 0.08 + (1.20 * (speed_ratio ** 1.5))
        a1 = a0 * 0.35

        # Fault Injection: High Vibration / Mechanical Shock
        mult = self.vib_multiplier
        if self.fault_vibration:
            mult *= 4.5 # Force vibration clearly above 1.8g alarm threshold
            extra_vib = 1.25 # Static baseline bump
        else:
            extra_vib = 0.0

        t = self.elapsed_time_s
        vib_x = (a0 * mult * math.sin(omega_shaft * t)) + (a1 * mult * math.sin(2.0 * omega_shaft * t + 0.45)) + random.gauss(0.0, 0.03)
        vib_y = (a0 * mult * math.cos(omega_shaft * t)) + (a1 * mult * math.cos(2.0 * omega_shaft * t + 1.20)) + random.gauss(0.0, 0.03)
        vib_z = 1.0 + (a0 * mult * 0.4 * math.sin(omega_shaft * t + 2.1)) + random.gauss(0.0, 0.02)
        vib_dyn_z = vib_z - 1.0
        vib_rms_g = math.sqrt((vib_x ** 2) + (vib_y ** 2) + (vib_dyn_z ** 2)) + extra_vib

        # ----------------------------------------------------------------------
        # 5. Coast-Down Bearing Drag Diagnostics
        # ----------------------------------------------------------------------
        if self.commanded_pwm_pct <= 0.0:
            if not self.coast_eval_active and self.omega_rad_s > 40.0:
                self.coast_eval_active = True
                self.coast_start_time = self.elapsed_time_s
                self.coast_start_omega = self.omega_rad_s
            elif self.coast_eval_active:
                coast_duration = self.elapsed_time_s - self.coast_start_time
                if coast_duration >= 0.35:
                    net_decel = (self.omega_rad_s - self.coast_start_omega) / coast_duration
                    self.coast_eval_active = False
                    if net_decel < FAULT_DECEL_FATIGUE or self.fault_bearing_drag:
                        self.bearing_drag_alarm = True
        else:
            self.coast_eval_active = False
            self.bearing_drag_alarm = False

        # ----------------------------------------------------------------------
        # 6. Fault & Human-Readable Warning Generation
        # ----------------------------------------------------------------------
        alarm_temp = measured_temp_c >= FAULT_TEMP_ALARM_C
        alarm_current = current_ma >= FAULT_CURRENT_ALARM_MA
        alarm_vib = vib_rms_g >= FAULT_VIB_ALARM_G

        # Rotor stall check
        alarm_stall = False
        if self.commanded_pwm_pct >= 25.0 and current_rpm < 30.0:
            self.stall_timer_s += dt
            if self.stall_timer_s >= FAULT_STALL_TIME_SEC or self.fault_stall:
                alarm_stall = True
        else:
            self.stall_timer_s = 0.0

        # Human-Readable Warnings List
        warnings_list = []
        if alarm_temp:
            warnings_list.append({
                "severity": "CRITICAL",
                "title": "Motor Overheating",
                "message": f"Surface temperature is {measured_temp_c:.1f}°C (Safety limit: {FAULT_TEMP_ALARM_C:.0f}°C). Risk of coil insulation damage."
            })
        elif measured_temp_c >= FAULT_TEMP_WARN_C:
            warnings_list.append({
                "severity": "WARNING",
                "title": "Elevated Temperature",
                "message": f"Temperature is rising ({measured_temp_c:.1f}°C). Approaching {FAULT_TEMP_ALARM_C:.0f}°C alarm limit."
            })

        if alarm_current:
            warnings_list.append({
                "severity": "CRITICAL",
                "title": "Overcurrent Detected",
                "message": f"Current draw is {current_ma:.0f} mA (Safety limit: {FAULT_CURRENT_ALARM_MA:.0f} mA). Possible mechanical binding or heavy overload."
            })
        elif current_ma >= FAULT_CURRENT_WARN_MA:
            warnings_list.append({
                "severity": "WARNING",
                "title": "High Current Draw",
                "message": f"Current draw is elevated ({current_ma:.0f} mA). Monitor mechanical load."
            })

        if alarm_vib:
            warnings_list.append({
                "severity": "CRITICAL",
                "title": "Excessive Vibration",
                "message": f"Vibration amplitude is {vib_rms_g:.2f} g RMS (Safety limit: {FAULT_VIB_ALARM_G:.2f} g). Check shaft balance and mounting."
            })
        elif vib_rms_g >= FAULT_VIB_WARN_G:
            warnings_list.append({
                "severity": "WARNING",
                "title": "Elevated Vibration",
                "message": f"Vibration magnitude is {vib_rms_g:.2f} g. Minor mechanical chatter detected."
            })

        if alarm_stall:
            warnings_list.append({
                "severity": "CRITICAL",
                "title": "Rotor Stalled / Locked",
                "message": f"Motor is commanded at {self.commanded_pwm_pct:.0f}% PWM but speed is 0 RPM. Cut power immediately to avoid burn out."
            })

        if self.bearing_drag_alarm:
            warnings_list.append({
                "severity": "WARNING",
                "title": "Bearing Drag / Friction",
                "message": f"Coast-down deceleration is {self.deceleration_rad_s2:.1f} rad/s². Bearing wear or insufficient lubrication detected."
            })

        # Overall Status
        has_critical = alarm_temp or alarm_current or alarm_vib or alarm_stall
        has_warning = len(warnings_list) > 0 and not has_critical

        if has_critical:
            status = "ALERT"
        elif has_warning:
            status = "WARNING"
        else:
            status = "NORMAL"

        return {
            "time_s": round(self.elapsed_time_s, 2),
            "elapsed_sec": round(self.elapsed_time_s, 2),
            "pwm_pct": round(self.commanded_pwm_pct, 1),
            "rpm": round(current_rpm, 1),
            "speed_rpm": round(current_rpm, 1),
            "omega_rad_s": round(self.omega_rad_s, 2),
            "pulse_us": pulse_interval_us,
            "pulses_window": pulses_in_window,
            "volts": round(bus_voltage_v, 2),
            "bus_voltage_v": round(bus_voltage_v, 2),
            "current_ma": round(current_ma, 1),
            "power_mw": round(power_mw, 1),
            "temp_c": round(measured_temp_c, 2),
            "vib_rms_g": round(vib_rms_g, 3),
            "vibration_rms_g": round(vib_rms_g, 3),
            "accel_x": round(vib_x, 3),
            "accel_y": round(vib_y, 3),
            "accel_z": round(vib_z, 3),
            "decel_rad_s2": round(self.deceleration_rad_s2, 2),
            "is_coasting": self.is_coasting,
            "status": status,
            "alarms": {
                "overtemp": alarm_temp,
                "overcurrent": alarm_current,
                "vibration": alarm_vib,
                "stall": alarm_stall,
                "bearing_drag": self.bearing_drag_alarm
            },
            "warnings": warnings_list,
            "faults_injected": {
                "overheat": self.fault_overheat,
                "overcurrent": self.fault_overcurrent,
                "vibration": self.fault_vibration,
                "stall": self.fault_stall,
                "bearing_drag": self.fault_bearing_drag,
                "voltage_drop": self.fault_voltage_drop
            }
        }


def main():
    sim = DCMotorPhysicsSimulator()
    print("DC Motor Physics Simulation Engine ready. Running step tests...")
    sim.set_pwm(50)
    for _ in range(5):
        t = sim.step(0.2)
        print(f"Time: {t['time_s']}s | RPM: {t['rpm']} | Current: {t['current_ma']}mA | Temp: {t['temp_c']}C | Status: {t['status']}")

if __name__ == "__main__":
    main()
