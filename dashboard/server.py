#!/usr/bin/env python3
"""
================================================================================
PMDC MOTOR HEALTH MONITORING - INDUSTRIAL SCADA WEB SERVER & API GATEWAY
================================================================================
Compliance: ISA-101 Industrial Telemetry Ergonomics
Backend: Uses backend_engine.SCADABackendEngine (ESP-32S Serial & Analytics)
================================================================================
"""

import os
import re
import sys
import time
import json
import threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse

# Ensure local workspace imports work reliably
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from backend_engine import SCADABackendEngine, AlarmSeverity

# Instantiate Unified Backend Engine
engine = SCADABackendEngine()


# ==============================================================================
# SECURE HTTP REQUEST HANDLER
# ==============================================================================
class IndustrialScadaHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.static_dir = os.path.join(BASE_DIR, "dashboard", "static")
        super().__init__(*args, directory=self.static_dir, **kwargs)

    def log_message(self, format, *args):
        # Silence standard HTTP logs for low latency
        return

    def send_json(self, data, status=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. API: Available COM Ports
        if path == "/api/ports":
            ports = engine.get_serial_ports()
            self.send_json({
                "ports": ports,
                "current_port": engine.serial_worker.port,
                "connected": engine.serial_worker.is_connected
            })

        # 2. API: System Status & Telemetry Snapshot
        elif path in ["/api/status", "/api/telemetry"]:
            with engine.lock:
                telemetry = dict(engine.latest_telemetry)
                telemetry["connected"] = engine.serial_worker.is_connected
                telemetry["port"] = engine.serial_worker.port
                telemetry["baud"] = engine.serial_worker.baud
                telemetry["slope_markers"] = engine.coast_analyzer.slope_markers
            self.send_json(telemetry)

        # 3. API: Chronological Alarm & Event Ledger
        elif path == "/api/logs":
            with engine.lock:
                logs = [ev.to_dict() for ev in reversed(engine.alarm_ledger[-50:])]
            self.send_json({"logs": logs})

        # 4. API: Server-Sent Events (SSE) Live Telemetry Stream (10 Hz)
        elif path == "/api/telemetry/stream":
            self.handle_sse_stream()

        # 5. Favicon
        elif path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()

        # 6. Static File Serving with Path Traversal Protection
        else:
            clean_path = path.lstrip("/")
            if clean_path == "":
                clean_path = "index.html"

            full_file_path = os.path.abspath(os.path.join(self.static_dir, clean_path))
            if not full_file_path.startswith(self.static_dir) or not os.path.exists(full_file_path):
                self.send_error(404, "File Not Found")
                return

            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get("Content-Length", 0))

        if length > 32768: # 32 KB limit
            self.send_json({"error": "Payload too large"}, status=413)
            return

        body = self.rfile.read(length).decode("utf-8") if length > 0 else ""
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            self.send_json({"error": "Malformed JSON payload"}, status=400)
            return

        # 1. API: Serial Connection Controls
        if path == "/api/serial/connect":
            port = str(data.get("port", "COM3")).strip()
            try:
                baud = int(data.get("baudrate", 115200))
            except (ValueError, TypeError):
                baud = 115200

            success, msg = engine.serial_worker.connect(port, baud)
            if success:
                engine._add_alarm(AlarmSeverity.NORMAL, "SERIAL", f"Connected to ESP32 on {port}")
            self.send_json({"success": success, "message": msg, "port": port})

        elif path == "/api/serial/disconnect":
            engine.serial_worker.disconnect()
            engine._add_alarm(AlarmSeverity.CAUTION, "SERIAL", "ESP32 disconnected")
            self.send_json({"success": True, "message": "Disconnected"})

        # 2. API: Operating Controls & Automated Diagnostics
        elif path == "/api/control":
            action = str(data.get("action", "")).lower()

            if action == "set_speed":
                try:
                    speed = float(data.get("speed", 0.0))
                    engine.set_target_speed(speed)
                    self.send_json({"success": True, "speed": speed})
                except (ValueError, TypeError):
                    self.send_json({"error": "Invalid speed"}, status=400)

            elif action == "driver_power":
                enable = bool(data.get("enable", True))
                engine.set_driver_power(enable)
                self.send_json({"success": True, "driver_pwr": enable})

            elif action in ["stop", "estop"]:
                engine.emergency_stop()
                self.send_json({"success": True, "message": "E-STOP Activated"})

            elif action == "ra_test":
                engine.trigger_ra_measurement()
                self.send_json({"success": True, "message": "Ra measurement triggered"})

            elif action == "coast_test":
                engine.trigger_coast_down_test()
                self.send_json({"success": True, "message": "Coast-down test triggered"})

            else:
                self.send_json({"error": f"Unknown action: {action}"}, status=400)

        else:
            self.send_json({"error": "Endpoint not found"}, status=404)

    def handle_sse_stream(self):
        """Streams live telemetry at 10 Hz via Server-Sent Events."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            while True:
                with engine.lock:
                    telem = dict(engine.latest_telemetry)
                    telem["connected"] = engine.serial_worker.is_connected
                    telem["slope_markers"] = engine.coast_analyzer.slope_markers
                payload = f"data: {json.dumps(telem)}\n\n".encode("utf-8")
                self.wfile.write(payload)
                self.wfile.flush()
                time.sleep(0.10)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass


# ==============================================================================
# SERVER INITIALIZATION
# ==============================================================================
def start_server(host="127.0.0.1", port=8080):
    server_address = (host, port)
    httpd = ThreadingHTTPServer(server_address, IndustrialScadaHandler)
    httpd.daemon_threads = True

    print("\n" + "=" * 70)
    print("  PMDC MOTOR HEALTH MONITOR - INDUSTRIAL SCADA WEB WORKSTATION")
    print("  Standards: ISA-101 Industrial Ergonomics | High-Contrast Charcoal")
    print("=" * 70)
    print(f"  URL              : http://{host}:{port}")
    print("  Backend Target   : ESP-32S (38-pin NodeMCU) / Auto-Simulation")
    print("  Baud Rate        : 115200 Baud")
    print("  Press Ctrl+C to stop.")
    print("=" * 70 + "\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping SCADA server...")
        engine.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    host = "127.0.0.1"
    port = 8080
    if len(sys.argv) > 1 and sys.argv[1] == "--all-interfaces":
        host = "0.0.0.0"
    start_server(host=host, port=port)
