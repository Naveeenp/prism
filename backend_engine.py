#!/usr/bin/env python3
"""
================================================================================
PMDC MOTOR HEALTH MONITORING - INDUSTRIAL SCADA BACKEND TELEMETRY ENGINE
================================================================================
Platform: Python 3.8+ (Thread-Safe Serial Data Ingestion & Diagnostic Math)
Standards: ISA-101 Industrial Telemetry, OLS Regression Coast-Down Fitting
Hardware Target: ESP-32S (38-Pin NodeMCU) over Serial (115200 Baud)
================================================================================
"""

import os
import re
import csv
import sys
import time
import json
import math
import queue
import logging
import threading
from datetime import datetime
from typing import Dict, Any, Optional, List, Tuple, Callable

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("TelemetryBackend")


# ==============================================================================
# ALARM & EVENT DEFINITIONS (ISA-101 SEVERITY LEVELS)
# ==============================================================================
class AlarmSeverity:
    NORMAL = "NORMAL"
    CAUTION = "CAUTION"
    CRITICAL = "CRITICAL"


class AlarmEvent:
    def __init__(self, timestamp: str, severity: str, parameter: str, condition: str):
        self.timestamp = timestamp
        self.severity = severity
        self.parameter = parameter
        self.condition = condition

    def to_dict(self) -> Dict[str, str]:
        return {
            "timestamp": self.timestamp,
            "severity": self.severity,
            "parameter": self.parameter,
            "condition": self.condition,
        }


# ==============================================================================
# MATHEMATICAL & DIAGNOSTIC CALCULATION CLASSES
# ==============================================================================
class ArmatureResistanceEstimator:
    """
    In-Situ Armature Resistance (Ra) Calculator:
    Theoretical Basis:
        Vt = Eb + Ia * Ra
        At locked-rotor, sub-stall pulse, or immediate startup transient:
        angular speed omega ≈ 0 => back-EMF Eb ≈ 0
        Therefore: Ra = Vt / Ia

    Fault Detection:
        - Ra << Ra_nominal: Inter-turn short circuit / Insulation breakdown
        - Ra >> Ra_nominal: Commutator wear / Brush contact resistance / High lead impedance
    """
    def __init__(self, nominal_ra_ohms: float = 2.40, tolerance_pct: float = 20.0):
        self.nominal_ra = nominal_ra_ohms
        self.tolerance_pct = tolerance_pct
        self.last_calculated_ra: Optional[float] = None
        self.last_status: str = "NOT_MEASURED"
        self.sample_buffer: List[Tuple[float, float]] = [] # [(v_bus, i_a)]

    def record_sample(self, v_bus: float, i_ma: float):
        """Buffer voltage and current samples during an in-situ locked/pulse test."""
        if i_ma > 50.0 and v_bus > 0.5: # Valid non-zero current
            self.sample_buffer.append((v_bus, i_ma / 1000.0)) # Store current in Amperes

    def compute_ra(self) -> Tuple[Optional[float], str, str]:
        """
        Calculates Ra from buffered inrush samples using robust mean/median.
        Returns: (Ra_ohms, health_classification, diagnostic_message)
        """
        if not self.sample_buffer:
            return None, "NO_DATA", "No valid voltage/current pulse samples collected"

        # Calculate Ra for each sample: Vt / Ia
        ra_values = [v / i for (v, i) in self.sample_buffer if i > 0.05]
        self.sample_buffer.clear()

        if not ra_values:
            return None, "INVALID_CURRENT", "Current draw too low to determine Ra"

        # Robust trimmed average (discard outliers)
        ra_values.sort()
        n = len(ra_values)
        if n >= 4:
            trimmed = ra_values[n // 4 : n - n // 4]
            ra_est = sum(trimmed) / len(trimmed)
        else:
            ra_est = sum(ra_values) / len(ra_values)

        self.last_calculated_ra = round(ra_est, 3)

        # Health classification
        dev_pct = ((ra_est - self.nominal_ra) / self.nominal_ra) * 100.0

        if dev_pct < -30.0:
            status = AlarmSeverity.CRITICAL
            msg = f"Insulation Breakdown Detected: Ra={ra_est:.2f}Ω ({dev_pct:.1f}% below nominal). Risk of shorted turns."
        elif dev_pct < -15.0:
            status = AlarmSeverity.CAUTION
            msg = f"Potential Coil Degradation: Ra={ra_est:.2f}Ω ({dev_pct:.1f}% below nominal)."
        elif dev_pct > 35.0:
            status = AlarmSeverity.CRITICAL
            msg = f"Severe Contact Wear: Ra={ra_est:.2f}Ω (+{dev_pct:.1f}%). Commutator fouling or brush wear."
        elif dev_pct > 15.0:
            status = AlarmSeverity.CAUTION
            msg = f"Elevated Resistance: Ra={ra_est:.2f}Ω (+{dev_pct:.1f}%). Minor brush oxidation."
        else:
            status = AlarmSeverity.NORMAL
            msg = f"Armature Resistance Nominal: Ra={ra_est:.2f}Ω (Deviation: {dev_pct:+.1f}%)."

        self.last_status = status
        return self.last_calculated_ra, status, msg


class CoastDownAnalyzer:
    """
    Bearing & Mechanical Drag Diagnostics via Coast-Down Spin Test:
    Theoretical Basis:
        When PWM cuts to high-Z coasting (ENA=0, IN1=0, IN2=0):
        Torque equation: J * (domega/dt) = - (Tc + b * omega)
        where:
            J = Rotor mass moment of inertia (kg*m^2)
            Tc = Coulomb friction torque (N*m)
            b = Viscous damping coefficient (N*m*s/rad)
            alpha = domega/dt = - (Tc/J + (b/J)*omega)

    Ordinary Least Squares (OLS) fits:
        y = -domega/dt vs x = omega
        intercept = Tc/J, slope = b/J
    """
    def __init__(self, rotor_inertia_kg_m2: float = 1.25e-5, nominal_coast_time_s: float = 2.20):
        self.J = rotor_inertia_kg_m2
        self.nominal_coast_time = nominal_coast_time_s

        self.is_active: bool = False
        self.cutoff_timestamp_s: Optional[float] = None
        self.stop_timestamp_s: Optional[float] = None
        self.coast_trajectory: List[Tuple[float, float]] = [] # [(t_rel_s, omega_rad_s)]

        # Results
        self.last_coast_duration_s: float = 0.0
        self.last_bci_pct: float = 100.0 # Bearing Condition Index (0-100%)
        self.last_coulomb_torque_nm: float = 0.0
        self.last_viscous_coeff_nms: float = 0.0
        self.last_avg_decel_rad_s2: float = 0.0
        self.slope_markers: List[Tuple[float, float]] = [] # For UI plot overlay

    def start_test(self, start_time_s: float):
        """Marks the moment of high-Z PWM cutoff."""
        self.is_active = True
        self.cutoff_timestamp_s = start_time_s
        self.stop_timestamp_s = None
        self.coast_trajectory.clear()
        self.slope_markers.clear()

    def feed_sample(self, time_s: float, rpm: float):
        """Feeds real-time speed sample during coasting."""
        if not self.is_active or self.cutoff_timestamp_s is None:
            return

        t_rel = time_s - self.cutoff_timestamp_s
        omega = max(0.0, rpm * (2.0 * math.pi / 60.0))
        self.coast_trajectory.append((t_rel, omega))

        # Check for coast termination (motor came to full stop or speed < 15 RPM)
        if len(self.coast_trajectory) > 5 and rpm < 15.0:
            self.stop_timestamp_s = time_s
            self.finalize_test()

    def finalize_test(self) -> Dict[str, Any]:
        """Performs deceleration rate analysis and parameter identification."""
        self.is_active = False
        if not self.coast_trajectory or len(self.coast_trajectory) < 4:
            return {
                "success": False,
                "message": "Insufficient coast-down data points collected"
            }

        t_vals = [p[0] for p in self.coast_trajectory]
        w_vals = [p[1] for p in self.coast_trajectory]

        # Total Coast-Down Duration
        self.last_coast_duration_s = round(t_vals[-1] - t_vals[0], 3)
        if self.last_coast_duration_s <= 0.05:
            self.last_coast_duration_s = 0.1

        # Calculate Bearing Condition Index (BCI)
        bci = (self.last_coast_duration_s / self.nominal_coast_time) * 100.0
        self.last_bci_pct = round(max(0.0, min(100.0, bci)), 1)

        # Numerical differentiation for domega/dt
        dw_dt_list = []
        w_mid_list = []
        for i in range(1, len(self.coast_trajectory)):
            dt = t_vals[i] - t_vals[i-1]
            if dt > 0.001:
                dw = w_vals[i] - w_vals[i-1]
                decel = - (dw / dt) # Positive decel magnitude
                dw_dt_list.append(decel)
                w_mid_list.append((w_vals[i] + w_vals[i-1]) / 2.0)

        if dw_dt_list:
            self.last_avg_decel_rad_s2 = round(sum(dw_dt_list) / len(dw_dt_list), 2)
        else:
            self.last_avg_decel_rad_s2 = 0.0

        # OLS Linear Regression: decel = c0 + c1 * omega
        # where c0 = Tc/J, c1 = b/J
        if len(w_mid_list) >= 3:
            n = len(w_mid_list)
            sum_x = sum(w_mid_list)
            sum_y = sum(dw_dt_list)
            sum_xx = sum(x * x for x in w_mid_list)
            sum_xy = sum(x * y for x, y in zip(w_mid_list, dw_dt_list))

            denom = (n * sum_xx - sum_x * sum_x)
            if abs(denom) > 1e-9:
                c1 = (n * sum_xy - sum_x * sum_y) / denom # b / J
                c0 = (sum_y - c1 * sum_x) / n             # Tc / J

                # Physical parameter constraints
                c0 = max(0.0, c0)
                c1 = max(0.0, c1)

                self.last_coulomb_torque_nm = round(c0 * self.J, 6)
                self.last_viscous_coeff_nms = round(c1 * self.J, 8)
            else:
                c0 = self.last_avg_decel_rad_s2
                c1 = 0.0
                self.last_coulomb_torque_nm = round(c0 * self.J, 6)
                self.last_viscous_coeff_nms = 0.0
        else:
            self.last_coulomb_torque_nm = 0.0
            self.last_viscous_coeff_nms = 0.0

        # Mark slope start and end for UI strip-chart
        self.slope_markers = [
            (t_vals[0], w_vals[0] * 60.0 / (2.0 * math.pi)),
            (t_vals[-1], w_vals[-1] * 60.0 / (2.0 * math.pi))
        ]

        # Condition Assessment
        if self.last_bci_pct < 60.0:
            severity = AlarmSeverity.CRITICAL
            cond = f"Severe Mechanical Binding / Dry Bearings (BCI: {self.last_bci_pct}% | Coast: {self.last_coast_duration_s}s)"
        elif self.last_bci_pct < 85.0:
            severity = AlarmSeverity.CAUTION
            cond = f"Bearing Drag / Friction Elevated (BCI: {self.last_bci_pct}% | Coast: {self.last_coast_duration_s}s)"
        else:
            severity = AlarmSeverity.NORMAL
            cond = f"Bearing & Mechanical Train Healthy (BCI: {self.last_bci_pct}% | Coast: {self.last_coast_duration_s}s)"

        return {
            "success": True,
            "coast_duration_s": self.last_coast_duration_s,
            "bci_pct": self.last_bci_pct,
            "coulomb_torque_nm": self.last_coulomb_torque_nm,
            "viscous_damping_nms": self.last_viscous_coeff_nms,
            "avg_decel_rad_s2": self.last_avg_decel_rad_s2,
            "severity": severity,
            "condition": cond,
            "points_count": len(self.coast_trajectory)
        }


# ==============================================================================
# CSV DATA LOGGER
# ==============================================================================
class TelemetryCSVLogger:
    """Thread-safe, robust CSV logging of all engineering parameters."""
    def __init__(self, output_dir: str = "logs"):
        self.output_dir = output_dir
        self.file_handle = None
        self.writer = None
        self.lock = threading.Lock()
        self.start_time = time.time()

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)

        self._init_file()

    def _init_file(self):
        timestr = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"scada_telemetry_{timestr}.csv")
        try:
            self.file_handle = open(filename, "w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file_handle)
            # Write standard engineering headers
            self.writer.writerow([
                "Timestamp_ISO",
                "Elapsed_Sec",
                "Operating_Mode",
                "Bus_Voltage_V",
                "Current_mA",
                "Speed_RPM",
                "Omega_rad_s",
                "Surface_Temp_C",
                "Vibration_RMS_G",
                "Calculated_Ra_Ohm",
                "Coast_Decel_rad_s2",
                "BCI_Pct",
                "Health_Status"
            ])
            self.file_handle.flush()
            logger.info(f"Initialized CSV logger: {filename}")
        except Exception as e:
            logger.error(f"Failed to create CSV log file: {e}")

    def log_record(self, data: Dict[str, Any]):
        if not self.writer or not self.file_handle:
            return

        with self.lock:
            try:
                now_iso = datetime.now().isoformat()
                elapsed = round(time.time() - self.start_time, 3)
                rpm = float(data.get("rpm", 0.0))
                omega = round(rpm * (2.0 * math.pi / 60.0), 2)

                self.writer.writerow([
                    now_iso,
                    elapsed,
                    data.get("mode", "UNKNOWN"),
                    data.get("v_bus", 0.0),
                    data.get("i_ma", 0.0),
                    rpm,
                    omega,
                    data.get("temp_c", 0.0),
                    data.get("vib_g", 0.0),
                    data.get("ra_ohms", "N/A"),
                    data.get("decel_rad_s2", 0.0),
                    data.get("bci_pct", 100.0),
                    data.get("status", "NORMAL")
                ])
                self.file_handle.flush()
            except Exception as e:
                logger.error(f"CSV write error: {e}")

    def close(self):
        with self.lock:
            if self.file_handle and not self.file_handle.closed:
                try:
                    self.file_handle.flush()
                    self.file_handle.close()
                except Exception:
                    pass


# ==============================================================================
# SERIAL COMMUNICATION PROTOCOL & INGESTION WORKER
# ==============================================================================
class ESP32SerialWorker(threading.Thread):
    """
    Thread-safe Serial Reader and Command Dispatcher:
    - Reads line-by-line serial packets at 115200 baud
    - Parses JSON telemetry frames and fallback CSV
    - Handles bi-directional handshakes (ACK / ERR)
    - Automatically recovers from serial disconnections
    """
    def __init__(self, port: str = "COM3", baud: int = 115200, data_callback: Optional[Callable] = None):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.data_callback = data_callback

        self.ser: Optional[serial.Serial] = None
        self.is_running: bool = False
        self.is_connected: bool = False
        self.last_heartbeat_time: float = 0.0
        self.error_msg: Optional[str] = None
        self.cmd_queue: queue.Queue = queue.Queue()
        self.lock = threading.Lock()

    def connect(self, port: str, baud: int = 115200) -> Tuple[bool, str]:
        """Establishes connection to ESP32."""
        if not SERIAL_AVAILABLE:
            return False, "pyserial library not available"

        # Sanitize port
        if not re.match(r"^[a-zA-Z0-9_\/.-]+$", port):
            return False, "Invalid port name"

        self.disconnect()

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,
                write_timeout=0.5
            )
            # ESP32 auto-reset suppression / stable DTR toggle
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.05)

            self.port = port
            self.baud = baud
            self.is_connected = True
            self.error_msg = None
            logger.info(f"Connected to ESP32 on {port} at {baud} baud.")
            return True, "Connected successfully"
        except Exception as e:
            self.is_connected = False
            self.error_msg = str(e)
            logger.error(f"Failed to connect to {port}: {e}")
            return False, str(e)

    def disconnect(self):
        """Safely terminates serial connection."""
        self.is_connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        logger.info("Serial port disconnected.")

    def send_command(self, cmd_str: str) -> bool:
        """Enqueues command for thread-safe transmission."""
        if not self.is_connected or not self.ser:
            return False
        self.cmd_queue.put(cmd_str.strip())
        return True

    def stop(self):
        self.is_running = False
        self.disconnect()

    def run(self):
        self.is_running = True
        logger.info("ESP32 Serial Worker thread started.")

        rx_buffer = ""

        while self.is_running:
            # 1. Process outbound commands
            while not self.cmd_queue.empty():
                try:
                    cmd = self.cmd_queue.get_nowait()
                    if self.ser and self.ser.is_open:
                        payload = (cmd + "\n").encode("utf-8")
                        self.ser.write(payload)
                        self.ser.flush()
                        logger.info(f"Dispatched command: {cmd}")
                except Exception as e:
                    logger.error(f"Command dispatch failed: {e}")

            # 2. Ingest inbound serial stream
            if self.is_connected and self.ser and self.ser.is_open:
                try:
                    raw_bytes = self.ser.read(self.ser.in_waiting or 1)
                    if raw_bytes:
                        chunk = raw_bytes.decode("utf-8", errors="ignore")
                        rx_buffer += chunk

                        while "\n" in rx_buffer:
                            line, rx_buffer = rx_buffer.split("\n", 1)
                            line = line.strip()
                            if line:
                                self._parse_incoming_line(line)
                    else:
                        time.sleep(0.01)
                except (serial.SerialException, OSError) as e:
                    logger.error(f"Serial I/O error: {e}")
                    self.disconnect()
                    self.error_msg = "Device disconnected or port error"
                    time.sleep(0.5)
            else:
                time.sleep(0.1)

    def _parse_incoming_line(self, line: str):
        """Parses clean JSON lines, handshake ACKs, or fallback CSV."""
        self.last_heartbeat_time = time.time()

        # Handshake / Command ACK checks
        if line.startswith("ACK:") or line.startswith("ERR:"):
            logger.info(f"ESP32 Protocol Handshake: {line}")
            return

        parsed_data = None

        # Format 1: Expected JSON Frame
        # {"mode": "RUN_75", "v_bus": 12.04, "i_ma": 320.5, "rpm": 1820.0, "temp_c": 31.4, "vib_g": 0.18, "ts_ms": 104250}
        if line.startswith("{") and line.endswith("}"):
            try:
                raw_json = json.loads(line)
                parsed_data = {
                    "mode": raw_json.get("mode", "NORMAL"),
                    "v_bus": float(raw_json.get("v_bus", raw_json.get("volts", 0.0))),
                    "i_ma": float(raw_json.get("i_ma", raw_json.get("curr_ma", 0.0))),
                    "rpm": float(raw_json.get("rpm", 0.0)),
                    "temp_c": float(raw_json.get("temp_c", 25.0)),
                    "vib_g": float(raw_json.get("vib_g", raw_json.get("vib_rms_g", 0.0))),
                    "ts_ms": int(raw_json.get("ts_ms", raw_json.get("time_ms", 0))),
                    "driver_pwr": int(raw_json.get("driver_pwr", 1))
                }
            except (json.JSONDecodeError, ValueError) as e:
                logger.debug(f"JSON parse error on line: {line} ({e})")

        # Format 2: Strict CSV Fallback: mode,v_bus,i_ma,rpm,temp_c,vib_g,ts_ms
        elif "," in line and not line.startswith("["):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 7:
                try:
                    parsed_data = {
                        "mode": parts[0],
                        "v_bus": float(parts[1]),
                        "i_ma": float(parts[2]),
                        "rpm": float(parts[3]),
                        "temp_c": float(parts[4]),
                        "vib_g": float(parts[5]),
                        "ts_ms": int(float(parts[6]))
                    }
                except ValueError:
                    pass

        if parsed_data and self.data_callback:
            self.data_callback(parsed_data)


# ==============================================================================
# UNIFIED SCADA BACKEND CONTROLLER
# ==============================================================================
class SCADABackendEngine:
    """
    Central Coordinator:
    - Serial Ingestion + Hardware Handshake
    - Built-in Simulation Engine for Offline/Bench Testing
    - Real-time Diagnostic Computations (Ra, Coast-Down OLS, Threshold Alarms)
    - Historical Alarm & Event Ledger
    - CSV Session Logging
    """
    def __init__(self):
        self.lock = threading.RLock()

        # Diagnostic Processors
        self.ra_estimator = ArmatureResistanceEstimator(nominal_ra_ohms=2.40)
        self.coast_analyzer = CoastDownAnalyzer(rotor_inertia_kg_m2=1.25e-5, nominal_coast_time_s=2.20)
        self.csv_logger = TelemetryCSVLogger()

        # Serial Worker
        self.serial_worker = ESP32SerialWorker(data_callback=self._on_telemetry_frame)
        self.serial_worker.start()

        # System State
        self.is_sim_mode: bool = False
        self.sim_speed_pwm: float = 0.0
        self.sim_coast_active: bool = False

        self.latest_telemetry: Dict[str, Any] = {
            "mode": "IDLE",
            "v_bus": 12.00,
            "i_ma": 0.0,
            "rpm": 0.0,
            "temp_c": 26.5,
            "vib_g": 0.04,
            "ts_ms": 0,
            "ra_ohms": 2.40,
            "bci_pct": 100.0,
            "coast_duration_s": 2.20,
            "status": AlarmSeverity.NORMAL,
            "active_alarms": []
        }

        # Historical Event Ledger (Chronological)
        self.alarm_ledger: List[AlarmEvent] = []
        self._add_alarm(AlarmSeverity.NORMAL, "SYSTEM", "SCADA Diagnostic Engine initialized")

        # Thresholds (Industrial Engineering Safety Limits)
        self.LIMITS = {
            "temp_warn_c": 45.0,
            "temp_crit_c": 52.0,
            "curr_warn_ma": 750.0,
            "curr_crit_ma": 950.0,
            "vib_warn_g": 0.80,
            "vib_crit_g": 1.50,
            "stall_rpm": 25.0
        }

        # Simulation Thread
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

    # --------------------------------------------------------------------------
    # Telemetry Ingestion & Real-Time Analytics
    # --------------------------------------------------------------------------
    def _on_telemetry_frame(self, frame: Dict[str, Any]):
        with self.lock:
            # 1. Update In-Situ Ra if buffer is active or during startup pulse
            if frame.get("mode") == "RA_TEST" or (frame["rpm"] < 30.0 and frame["i_ma"] > 200.0):
                self.ra_estimator.record_sample(frame["v_bus"], frame["i_ma"])

            # 2. Feed speed samples to Coast-Down Analyzer
            if self.coast_analyzer.is_active:
                now_s = frame["ts_ms"] / 1000.0 if frame["ts_ms"] > 0 else time.time()
                self.coast_analyzer.feed_sample(now_s, frame["rpm"])

            # 3. Evaluate Engineering Threshold Alarms
            active_alarms, overall_status = self._evaluate_thresholds(frame)

            # 4. Synthesize Process Values
            ra_val = self.ra_estimator.last_calculated_ra or 2.40
            bci_val = self.coast_analyzer.last_bci_pct

            driver_pwr_state = bool(frame.get("driver_pwr", 1 if self.sim_speed_pwm > 0 else 0))

            self.latest_telemetry = {
                "mode": frame.get("mode", "RUN"),
                "v_bus": frame["v_bus"],
                "i_ma": frame["i_ma"],
                "rpm": frame["rpm"],
                "temp_c": frame["temp_c"],
                "vib_g": frame["vib_g"],
                "ts_ms": frame["ts_ms"],
                "ra_ohms": ra_val,
                "bci_pct": bci_val,
                "coast_duration_s": self.coast_analyzer.last_coast_duration_s,
                "status": overall_status,
                "active_alarms": active_alarms,
                "driver_pwr": driver_pwr_state
            }

            # 5. Log to CSV
            self.csv_logger.log_record(self.latest_telemetry)

    def _evaluate_thresholds(self, frame: Dict[str, Any]) -> Tuple[List[str], str]:
        active_alarms = []
        overall = AlarmSeverity.NORMAL

        temp = frame["temp_c"]
        curr = frame["i_ma"]
        vib = frame["vib_g"]
        rpm = frame["rpm"]

        # Temperature
        if temp >= self.LIMITS["temp_crit_c"]:
            msg = f"Motor Overheat: {temp:.1f}°C (Trip Limit: {self.LIMITS['temp_crit_c']}°C)"
            active_alarms.append(msg)
            overall = AlarmSeverity.CRITICAL
            self._add_alarm(AlarmSeverity.CRITICAL, "SURFACE_TEMP", msg)
        elif temp >= self.LIMITS["temp_warn_c"]:
            msg = f"Elevated Temperature: {temp:.1f}°C"
            active_alarms.append(msg)
            if overall != AlarmSeverity.CRITICAL:
                overall = AlarmSeverity.CAUTION
            self._add_alarm(AlarmSeverity.CAUTION, "SURFACE_TEMP", msg)

        # Current
        if curr >= self.LIMITS["curr_crit_ma"]:
            msg = f"Overcurrent Trip: {curr:.0f}mA (Trip Limit: {self.LIMITS['curr_crit_ma']}mA)"
            active_alarms.append(msg)
            overall = AlarmSeverity.CRITICAL
            self._add_alarm(AlarmSeverity.CRITICAL, "ARMATURE_CURRENT", msg)
        elif curr >= self.LIMITS["curr_warn_ma"]:
            msg = f"High Current Warning: {curr:.0f}mA"
            active_alarms.append(msg)
            if overall != AlarmSeverity.CRITICAL:
                overall = AlarmSeverity.CAUTION
            self._add_alarm(AlarmSeverity.CAUTION, "ARMATURE_CURRENT", msg)

        # Vibration
        if vib >= self.LIMITS["vib_crit_g"]:
            msg = f"Excessive Vibration: {vib:.2f}G RMS (Trip Limit: {self.LIMITS['vib_crit_g']}G)"
            active_alarms.append(msg)
            overall = AlarmSeverity.CRITICAL
            self._add_alarm(AlarmSeverity.CRITICAL, "VIBRATION_RMS", msg)
        elif vib >= self.LIMITS["vib_warn_g"]:
            msg = f"Vibration Warning: {vib:.2f}G RMS"
            active_alarms.append(msg)
            if overall != AlarmSeverity.CRITICAL:
                overall = AlarmSeverity.CAUTION
            self._add_alarm(AlarmSeverity.CAUTION, "VIBRATION_RMS", msg)

        return active_alarms, overall

    def _add_alarm(self, severity: str, parameter: str, condition: str):
        now_str = datetime.now().strftime("%H:%M:%S")
        # Deduplicate consecutive identical alarms
        if self.alarm_ledger and self.alarm_ledger[-1].condition == condition:
            return

        event = AlarmEvent(now_str, severity, parameter, condition)
        self.alarm_ledger.append(event)
        if len(self.alarm_ledger) > 200:
            self.alarm_ledger.pop(0)

    # --------------------------------------------------------------------------
    # Control & Diagnostic Triggers (Commands Sent to ESP32)
    # --------------------------------------------------------------------------
    def set_driver_power(self, enable: bool):
        """Enables or disables L298N driver power / arming: CMD:DRIVER_POWER:<ON|OFF>"""
        with self.lock:
            self.latest_telemetry["driver_pwr"] = enable
            if not enable:
                self.sim_speed_pwm = 0.0

        state_str = "ON" if enable else "OFF"
        if self.serial_worker.is_connected:
            cmd = f"CMD:DRIVER_POWER:{state_str}"
            self.serial_worker.send_command(cmd)
            self._add_alarm(
                AlarmSeverity.NORMAL if enable else AlarmSeverity.CAUTION,
                "DRIVER_PWR",
                f"L298N Motor Driver Power switched {state_str}"
            )
        else:
            self._add_alarm(
                AlarmSeverity.NORMAL if enable else AlarmSeverity.CAUTION,
                "SIMULATION",
                f"Simulated Driver Power switched {state_str}"
            )

    def set_target_speed(self, pct: float):
        """Dispatches speed command: CMD:SET_SPEED:<0-255>"""
        pct = max(0.0, min(100.0, pct))
        duty_8bit = int((pct / 100.0) * 255.0)

        with self.lock:
            self.sim_speed_pwm = pct
            self.sim_coast_active = False

        if self.serial_worker.is_connected:
            cmd = f"CMD:SET_SPEED:{duty_8bit}"
            self.serial_worker.send_command(cmd)
            self._add_alarm(AlarmSeverity.NORMAL, "CONTROLLER", f"Target Speed set to {pct:.0f}% (Duty: {duty_8bit}/255)")
        else:
            self._add_alarm(AlarmSeverity.NORMAL, "SIMULATION", f"Simulated Speed set to {pct:.0f}%")

    def emergency_stop(self):
        """Immediately zeroes drive output: CMD:ESTOP"""
        with self.lock:
            self.sim_speed_pwm = 0.0
            self.sim_coast_active = True

        if self.serial_worker.is_connected:
            self.serial_worker.send_command("CMD:ESTOP")
            self._add_alarm(AlarmSeverity.CRITICAL, "SAFETY", "EMERGENCY STOP (E-STOP) ACTIVATED")
        else:
            self._add_alarm(AlarmSeverity.CRITICAL, "SAFETY", "E-STOP (Simulation Coast)")

    def trigger_ra_measurement(self):
        """Issues in-situ locked/pulse test: CMD:MEASURE_RA"""
        self._add_alarm(AlarmSeverity.NORMAL, "DIAGNOSTIC", "Initiating In-Situ Armature Resistance (Ra) test pulse...")
        self.ra_estimator.sample_buffer.clear()

        if self.serial_worker.is_connected:
            self.serial_worker.send_command("CMD:MEASURE_RA")
        else:
            # Simulate sub-stall pulse (Vt = 12.0V, Ia = 5.0A -> Ra = 2.40 Ohms)
            for _ in range(8):
                v_pulse = 12.02 + (0.05 * (time.time() % 1))
                i_pulse = 4980.0 + (30.0 * (time.time() % 2))
                self.ra_estimator.record_sample(v_pulse, i_pulse)

            ra, status, msg = self.ra_estimator.compute_ra()
            self._add_alarm(status, "RA_ESTIMATOR", msg)

    def trigger_coast_down_test(self):
        """Automated Coast-Down Test Sequence."""
        self._add_alarm(AlarmSeverity.NORMAL, "DIAGNOSTIC", "Initiating Automated Coast-Down Spin Test...")

        def _coast_sequence():
            # 1. Ramp to 75% baseline speed
            self.set_target_speed(75.0)
            time.sleep(2.0) # Wait for steady state

            # 2. Trigger high-Z coasting
            self._add_alarm(AlarmSeverity.NORMAL, "DIAGNOSTIC", "Cutting PWM to High-Z Coast (ENA=0, IN1=0, IN2=0)")
            now_s = time.time()
            self.coast_analyzer.start_test(now_s)

            if self.serial_worker.is_connected:
                self.serial_worker.send_command("CMD:RUN_COAST_TEST")
            else:
                self.sim_coast_active = True
                self.sim_speed_pwm = 0.0

            # 3. Wait for coast-down to complete
            cutoff_start = time.time()
            while time.time() - cutoff_start < 4.0:
                if not self.coast_analyzer.is_active:
                    break
                time.sleep(0.1)

            results = self.coast_analyzer.finalize_test()
            if results.get("success"):
                self._add_alarm(results["severity"], "COAST_TEST", results["condition"])

        threading.Thread(target=_coast_sequence, daemon=True).start()

    # --------------------------------------------------------------------------
    # Built-in High-Fidelity Physics Simulation Loop (for Offline Bench Testing)
    # --------------------------------------------------------------------------
    def _sim_loop(self):
        """Simulates 10 Hz telemetry when no physical hardware is plugged in."""
        sim_rpm = 0.0
        sim_temp = 28.0
        sim_curr = 0.0
        start_ts = time.time()

        while True:
            time.sleep(0.10) # 10 Hz rate

            # Only simulate if serial port is not connected
            if not self.serial_worker.is_connected:
                dt = 0.10
                target_rpm = (self.sim_speed_pwm / 100.0) * 2400.0

                if self.sim_coast_active:
                    # Pure mechanical coast-down governed by J * domega/dt = - (Tc + b*omega)
                    omega = sim_rpm * (2.0 * math.pi / 60.0)
                    if omega > 0.5:
                        Tc = 0.00035  # N*m
                        b = 0.0000085 # N*m*s/rad
                        J = 1.25e-5   # kg*m^2
                        domega_dt = - (Tc + b * omega) / J
                        omega = max(0.0, omega + domega_dt * dt)
                        sim_rpm = omega * (60.0 / (2.0 * math.pi))
                    else:
                        sim_rpm = 0.0
                        self.sim_coast_active = False
                else:
                    # First-order motor response
                    sim_rpm += (target_rpm - sim_rpm) * 0.25

                # Current calculation
                if sim_rpm < 10.0 and self.sim_speed_pwm > 0:
                    sim_curr = 2400.0 # Inrush
                elif self.sim_speed_pwm > 0:
                    sim_curr = 280.0 + (sim_rpm / 2400.0) * 150.0
                else:
                    sim_curr = 0.0

                # Surface temperature dynamics
                joule_heating = (sim_curr / 1000.0) ** 2 * 2.40 * 0.15
                sim_temp += (joule_heating - (sim_temp - 25.0) * 0.02) * dt

                # Vibration calculation (unbalance + bearing floor)
                vib_base = 0.05 + (sim_rpm / 2400.0) * 0.22

                elapsed_ms = int((time.time() - start_ts) * 1000)

                frame = {
                    "mode": f"RUN_{int(self.sim_speed_pwm)}",
                    "v_bus": 12.04 if self.sim_speed_pwm > 0 else 12.18,
                    "i_ma": round(sim_curr, 1),
                    "rpm": round(sim_rpm, 1),
                    "temp_c": round(sim_temp, 1),
                    "vib_g": round(vib_base, 3),
                    "ts_ms": elapsed_ms
                }

                self._on_telemetry_frame(frame)

    def get_serial_ports(self) -> List[Dict[str, str]]:
        """Lists available serial COM ports."""
        port_list = []
        if SERIAL_AVAILABLE:
            for p in serial.tools.list_ports.comports():
                port_list.append({"device": p.device, "description": p.description})
        return port_list

    def shutdown(self):
        """Clean shutdown of background threads and CSV files."""
        self.serial_worker.stop()
        self.csv_logger.close()
        logger.info("SCADA Backend engine shutdown cleanly.")


if __name__ == "__main__":
    print("Testing SCADA Backend Engine...")
    engine = SCADABackendEngine()
    time.sleep(1.0)
    print("Issuing 75% Speed...")
    engine.set_target_speed(75.0)
    time.sleep(2.0)
    print("Triggering Coast-Down Test...")
    engine.trigger_coast_down_test()
    time.sleep(3.0)
    print(f"Latest Telemetry: {engine.latest_telemetry}")
    engine.shutdown()
    print("Backend test completed successfully.")
