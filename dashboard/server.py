#!/usr/bin/env python3
"""
================================================================================
DC MOTOR HEALTH MONITORING - SECURE SCADA TELEMETRY & CONTROL SERVER
================================================================================
Platform: Python 3 Standard Library + PySerial
Features:
  - ThreadingHTTPServer for concurrent non-blocking HTTP and SSE streams
  - Path traversal protection & strict input sanitization
  - Human-readable diagnostics engine with fault injection lab
  - Robust serial connection manager for Raspberry Pi Pico H (RP2040)
  - Zero AI frosted glass or AI jargon
================================================================================
"""

import os
import re
import sys
import time
import json
import math
import random
import threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse

# Ensure local imports work reliably
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from simulation import DCMotorPhysicsSimulator

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ==============================================================================
# SECURE GLOBAL STATE
# ==============================================================================
class SafeSystemState:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = "SIMULATION"  # "SIMULATION" or "HARDWARE"
        self.port = "COM3"
        self.baudrate = 115200
        self.serial_handle = None
        self.serial_connected = False
        self.serial_error = None
        
        # Telemetry Cache
        self.latest_telemetry = {
            "time_ms": 0,
            "mode": "SIMULATION",
            "pwm_pct": 0.0,
            "rpm": 0.0,
            "volts": 12.0,
            "curr_ma": 0.0,
            "power_mw": 0.0,
            "temp_c": 25.0,
            "vib_rms_g": 0.0,
            "decel_rad_s2": 0.0,
            "status": "NORMAL",
            "alarms": {
                "overtemp": False,
                "overcurrent": False,
                "vibration": False,
                "stall": False,
                "bearing_drag": False
            },
            "warnings": [],
            "faults_injected": {}
        }
        
        self.logs = []
        self.sim = DCMotorPhysicsSimulator()

    def add_log(self, msg, level="info"):
        timestamp = time.strftime("%H:%M:%S")
        entry = {"time": timestamp, "text": msg, "level": level}
        with self.lock:
            self.logs.append(entry)
            if len(self.logs) > 100:
                self.logs.pop(0)

    def get_logs(self):
        with self.lock:
            return list(self.logs)


state = SafeSystemState()


# ==============================================================================
# SERIAL COMMUNICATION CONTROLLER
# ==============================================================================
def connect_hardware_serial(port_name, baud=115200):
    if not SERIAL_AVAILABLE:
        state.add_log("PySerial library is not installed on host.", "error")
        return False, "PySerial not available"

    # Input sanitization against port injection
    if not re.match(r"^[a-zA-Z0-9_\/.-]+$", port_name) or len(port_name) > 32:
        return False, "Invalid port name"

    if baud not in [9600, 19200, 38400, 57600, 115200]:
        return False, "Unsupported baud rate"

    with state.lock:
        if state.serial_handle and state.serial_handle.is_open:
            try:
                state.serial_handle.close()
            except Exception:
                pass
            state.serial_handle = None
            state.serial_connected = False

        try:
            ser = serial.Serial(port=port_name, baudrate=baud, timeout=0.1)
            state.serial_handle = ser
            state.serial_connected = True
            state.port = port_name
            state.baudrate = baud
            state.serial_error = None
            state.add_log(f"Connected to Raspberry Pi Pico H on {port_name} ({baud} baud).", "info")
            return True, "Connected"
        except Exception as e:
            err_str = str(e)
            state.serial_error = err_str
            state.serial_connected = False
            state.add_log(f"Connection failed for {port_name}: {err_str}", "error")
            return False, err_str


def disconnect_hardware_serial():
    with state.lock:
        if state.serial_handle and state.serial_handle.is_open:
            try:
                state.serial_handle.close()
            except Exception:
                pass
        state.serial_handle = None
        state.serial_connected = False
        state.add_log("Hardware serial disconnected.", "warn")


def send_motor_command(command_str):
    """Sends sanitized SPEED or STOP command."""
    with state.lock:
        current_mode = state.mode

    if current_mode == "SIMULATION":
        if command_str.startswith("SPEED "):
            try:
                val = float(command_str.split()[1])
                state.sim.set_pwm(val)
                state.add_log(f"Motor Speed set to {val:.0f}% PWM (Simulation).", "info")
            except (ValueError, IndexError):
                return False, "Invalid speed format"
        elif command_str == "STOP":
            state.sim.set_pwm(0.0)
            state.add_log("EMERGENCY STOP initiated (Simulation coast-down).", "warn")
        return True, "Command applied in simulation"
    else:
        with state.lock:
            ser = state.serial_handle
            is_conn = state.serial_connected

        if is_conn and ser and ser.is_open:
            try:
                cmd_line = (command_str.strip() + "\n").encode('utf-8')
                ser.write(cmd_line)
                ser.flush()
                state.add_log(f"Dispatched to Pico: {command_str.strip()}", "info")
                return True, "Dispatched to hardware"
            except Exception as e:
                state.add_log(f"Serial transmission failed: {e}", "error")
                return False, str(e)
        else:
            return False, "Serial port not connected"


# ==============================================================================
# TELEMETRY ACQUISITION WORKER THREAD
# ==============================================================================
def telemetry_worker():
    last_tick = time.time()
    
    while True:
        now = time.time()
        dt = now - last_tick
        last_tick = now

        with state.lock:
            current_mode = state.mode

        if current_mode == "SIMULATION":
            # Advance simulation physics
            sim_data = state.sim.step(dt if dt > 0 else 0.10)
            with state.lock:
                state.latest_telemetry = {
                    "time_ms": int(sim_data["time_s"] * 1000),
                    "mode": "SIMULATION",
                    "pwm_pct": sim_data["pwm_pct"],
                    "rpm": sim_data["rpm"],
                    "volts": sim_data["volts"],
                    "curr_ma": sim_data["current_ma"],
                    "power_mw": sim_data["power_mw"],
                    "temp_c": sim_data["temp_c"],
                    "vib_rms_g": sim_data["vib_rms_g"],
                    "decel_rad_s2": sim_data["decel_rad_s2"],
                    "status": sim_data["status"],
                    "alarms": sim_data["alarms"],
                    "warnings": sim_data.get("warnings", []),
                    "faults_injected": sim_data.get("faults_injected", {})
                }
            time.sleep(0.10) # 10 Hz rate

        else:
            # Read from physical Pico serial
            ser = None
            with state.lock:
                if state.serial_connected:
                    ser = state.serial_handle

            if ser is not None and ser.is_open:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith('{') and line.endswith('}'):
                            try:
                                packet = json.loads(line)
                                # Generate human-readable warnings for hardware readings
                                temp = float(packet.get("temp_c", 25.0))
                                curr = float(packet.get("curr_ma", 0.0))
                                vib = float(packet.get("vib_rms_g", 0.0))
                                rpm = float(packet.get("rpm", 0.0))
                                pwm = float(packet.get("pwm_pct", 0.0))
                                decel = float(packet.get("decel_rad_s2", 0.0))

                                hw_warnings = []
                                if temp >= 50.0:
                                    hw_warnings.append({"severity": "CRITICAL", "title": "Motor Overheating", "message": f"Surface temperature is {temp:.1f}°C (Limit: 50°C). Risk of coil insulation damage."})
                                elif temp >= 45.0:
                                    hw_warnings.append({"severity": "WARNING", "title": "Elevated Temperature", "message": f"Temperature is {temp:.1f}°C. Approaching safety threshold."})

                                if curr >= 900.0:
                                    hw_warnings.append({"severity": "CRITICAL", "title": "Overcurrent Detected", "message": f"Current draw is {curr:.0f} mA (Limit: 900 mA). Check for mechanical binding."})
                                elif curr >= 750.0:
                                    hw_warnings.append({"severity": "WARNING", "title": "High Current Draw", "message": f"Current draw is elevated ({curr:.0f} mA). Monitor load."})

                                if vib >= 1.80:
                                    hw_warnings.append({"severity": "CRITICAL", "title": "Excessive Vibration", "message": f"Vibration amplitude is {vib:.2f} g RMS. Inspect rotor balance and mounting."})

                                if pwm >= 25.0 and rpm < 30.0:
                                    hw_warnings.append({"severity": "CRITICAL", "title": "Rotor Stalled / Locked", "message": "Motor has power applied but is not rotating. Cut power to prevent burnout."})

                                if decel < -90.0:
                                    hw_warnings.append({"severity": "WARNING", "title": "Bearing Drag Detected", "message": f"Rapid coast deceleration ({decel:.1f} rad/s²). Indicates bearing wear or high friction."})

                                status = "NORMAL"
                                if any(w["severity"] == "CRITICAL" for w in hw_warnings):
                                    status = "ALERT"
                                elif len(hw_warnings) > 0:
                                    status = "WARNING"

                                with state.lock:
                                    state.latest_telemetry = {
                                        "time_ms": packet.get("time_ms", int(now * 1000)),
                                        "mode": "HARDWARE",
                                        "pwm_pct": pwm,
                                        "rpm": rpm,
                                        "volts": float(packet.get("volts", 0.0)),
                                        "curr_ma": curr,
                                        "power_mw": float(packet.get("power_mw", 0.0)),
                                        "temp_c": temp,
                                        "vib_rms_g": vib,
                                        "decel_rad_s2": decel,
                                        "status": status,
                                        "alarms": packet.get("alarms", {}),
                                        "warnings": hw_warnings,
                                        "faults_injected": {}
                                    }
                            except json.JSONDecodeError:
                                state.add_log(f"Raw: {line}", "info")
                        else:
                            state.add_log(f"Pico: {line}", "info")
                except Exception as e:
                    state.add_log(f"Serial read error: {e}", "error")
                    time.sleep(0.05)
            else:
                time.sleep(0.20)


# ==============================================================================
# SECURE HTTP REQUEST HANDLER
# ==============================================================================
class SecureScadaHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.static_dir = os.path.join(BASE_DIR, "dashboard", "static")
        super().__init__(*args, directory=self.static_dir, **kwargs)

    def log_message(self, format, *args):
        # Silence standard HTTP logs
        return

    def send_json(self, data, status=200):
        body = json.dumps(data).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. API: Available COM Ports
        if path == "/api/ports":
            ports = []
            if SERIAL_AVAILABLE:
                try:
                    for p in serial.tools.list_ports.comports():
                        ports.append({"device": p.device, "description": p.description})
                except Exception:
                    pass
            with state.lock:
                self.send_json({"ports": ports, "current_port": state.port, "connected": state.serial_connected})

        # 2. API: System Status Snapshot
        elif path == "/api/status":
            with state.lock:
                self.send_json({
                    "mode": state.mode,
                    "port": state.port,
                    "baudrate": state.baudrate,
                    "connected": state.serial_connected,
                    "serial_error": state.serial_error,
                    "latest": state.latest_telemetry
                })

        # 3. API: Telemetry Snapshot
        elif path == "/api/telemetry":
            with state.lock:
                self.send_json(state.latest_telemetry)

        # 4. API: Diagnostic Logs
        elif path == "/api/logs":
            self.send_json({"logs": state.get_logs()})

        # 5. API: Server-Sent Events (SSE) Stream
        elif path == "/api/telemetry/stream":
            self.handle_sse_stream()

        # 6. Favicon handler (prevent 404 error)
        elif path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()

        # 7. Static File Serving with Strict Path Traversal Protection
        else:
            clean_path = path.lstrip('/')
            if clean_path == "":
                clean_path = "index.html"

            full_file_path = os.path.abspath(os.path.join(self.static_dir, clean_path))
            
            # Security verification: Ensure file is strictly inside self.static_dir
            if not full_file_path.startswith(self.static_dir) or not os.path.exists(full_file_path):
                self.send_error(404, "File Not Found")
                return

            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', 0))

        if length > 32768: # 32 KB body limit to prevent DOS
            self.send_json({"error": "Payload too large"}, status=413)
            return

        body = self.rfile.read(length).decode('utf-8') if length > 0 else ""
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            self.send_json({"error": "Malformed JSON payload"}, status=400)
            return

        # 1. API: Switch Telemetry Source (SIMULATION vs HARDWARE)
        if path == "/api/mode":
            mode = str(data.get("mode", "")).upper()
            if mode not in ["SIMULATION", "HARDWARE"]:
                self.send_json({"error": "Mode must be SIMULATION or HARDWARE"}, status=400)
                return

            with state.lock:
                state.mode = mode

            if mode == "HARDWARE":
                port = str(data.get("port", state.port))
                baud = int(data.get("baudrate", 115200))
                success, msg = connect_hardware_serial(port, baud)
                self.send_json({"success": success, "mode": mode, "message": msg})
            else:
                disconnect_hardware_serial()
                state.add_log("Switched to Simulation Engine.", "info")
                self.send_json({"success": True, "mode": mode, "message": "Simulation active"})

        # 2. API: Motor Drive Control (Speed / E-Stop)
        elif path == "/api/control":
            action = str(data.get("action", ""))
            if action == "set_speed":
                try:
                    speed = float(data.get("speed", 0.0))
                    speed = max(0.0, min(100.0, speed)) # Clamp 0 - 100%
                    success, msg = send_motor_command(f"SPEED {speed:.1f}")
                    self.send_json({"success": success, "message": msg, "speed": speed})
                except (ValueError, TypeError):
                    self.send_json({"error": "Invalid speed value"}, status=400)
            elif action == "stop":
                success, msg = send_motor_command("STOP")
                self.send_json({"success": success, "message": msg})
            else:
                self.send_json({"error": "Invalid action. Use set_speed or stop"}, status=400)

        # 3. API: Interactive Fault Injection Lab
        elif path == "/api/simulation/faults":
            with state.lock:
                state.sim.set_faults(data)
                state.add_log(f"Simulation faults updated: {data}", "warn")
            self.send_json({"success": True, "message": "Fault injection parameters applied"})

        elif path == "/api/simulation/reset":
            with state.lock:
                state.sim.clear_all_faults()
                state.add_log("All simulation faults cleared. Returned to nominal physics.", "info")
            self.send_json({"success": True, "message": "All faults cleared"})

        # 4. API: Serial Connection Controls
        elif path == "/api/serial/connect":
            port = str(data.get("port", state.port))
            try:
                baud = int(data.get("baudrate", 115200))
            except (ValueError, TypeError):
                baud = 115200
            success, msg = connect_hardware_serial(port, baud)
            self.send_json({"success": success, "message": msg, "port": port})

        elif path == "/api/serial/disconnect":
            disconnect_hardware_serial()
            self.send_json({"success": True, "message": "Disconnected"})

        else:
            self.send_json({"error": "Endpoint not found"}, status=404)

    def handle_sse_stream(self):
        """Streams live telemetry at 10 Hz via Server-Sent Events with robust exception handling."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()

        try:
            while True:
                with state.lock:
                    telem = dict(state.latest_telemetry)
                payload = f"data: {json.dumps(telem)}\n\n".encode('utf-8')
                self.wfile.write(payload)
                self.wfile.flush()
                time.sleep(0.10) # 10 Hz stream
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            # Client closed the tab or refreshed; clean up gracefully without crashing
            pass


# ==============================================================================
# SERVER INITIALIZATION
# ==============================================================================
def start_server(host="127.0.0.1", port=8080):
    server_address = (host, port)
    
    # Use ThreadingHTTPServer so SSE streams never block REST requests
    httpd = ThreadingHTTPServer(server_address, SecureScadaHandler)
    httpd.daemon_threads = True

    # Start background telemetry engine thread
    t = threading.Thread(target=telemetry_worker, daemon=True)
    t.start()

    print("\n" + "=" * 70)
    print("  DC MOTOR HEALTH DIAGNOSTIC & MONITORING DASHBOARD")
    print("=" * 70)
    print(f"  URL              : http://{host}:{port}")
    print("  Mode             : SIMULATION ENGINE (Switchable to Hardware in UI)")
    print("  Safety Features  : ThreadingHTTPServer, Path Traversal Check, SSE Guard")
    print("  Press Ctrl+C to stop.")
    print("=" * 70 + "\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard server...")
        disconnect_hardware_serial()
        httpd.server_close()


if __name__ == "__main__":
    host = "127.0.0.1"
    port = 8080
    if len(sys.argv) > 1 and sys.argv[1] == "--all-interfaces":
        host = "0.0.0.0"
    start_server(host=host, port=port)
