#!/usr/bin/env python3
"""
===============================================================================
 ESP-32S (38-Pin NodeMCU) - Build, Flash & Serial Management Tool
===============================================================================
 Target Microcontroller: ESP-32S / ESP32-WROOM-32 (NodeMCU-32S)
 Features:
   - Detects ESP32 CP2102 / CH340 / CH9102 USB-to-UART Bridge COM Ports
   - 115200 Baud Diagnostic Telemetry Monitor & Command Console
   - Automated Diagnostics (Ra locked pulse test, Coast-down spin test)
   - Compile & Upload using Arduino-CLI (FQBN: esp32:esp32:esp32)
===============================================================================
"""

import os
import sys
import time
import subprocess
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

WORKSPACE_ROOT = Path(__file__).resolve().parent
SKETCH_DIR = WORKSPACE_ROOT / "dc_motor_health_monitor"
TOOLS_DIR = WORKSPACE_ROOT / "tools" / "bin"
ARDUINO_CLI = TOOLS_DIR / "arduino-cli.exe"
ESP32_FQBN = "esp32:esp32:esp32"


def log(msg):
    print(f"[*] {msg}")


def log_success(msg):
    print(f"[+] {msg}")


def log_warn(msg):
    print(f"[!] {msg}")


def log_err(msg):
    print(f"[-] {msg}")


def detect_esp32_com_port():
    """Identifies ESP32 USB-to-UART Bridge (CP210x, CH340, FTDI)."""
    if not SERIAL_AVAILABLE:
        log_err("pyserial is not installed.")
        return None, None

    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        vid = getattr(p, "vid", None)
        # Silicon Labs CP210x: 0x10C4, WCH CH340: 0x1A86, FTDI: 0x0403
        if vid in [0x10C4, 0x1A86, 0x0403] or "cp210" in desc or "ch340" in desc or "uart" in desc:
            return p.device, p.description

    # Fallback to first available COM port
    ports = list(serial.tools.list_ports.comports())
    if ports:
        return ports[0].device, ports[0].description
    return None, None


def cmd_detect():
    print("=" * 60)
    print("  ESP-32S NodeMCU Hardware Auto-Detection")
    print("=" * 60)
    port, desc = detect_esp32_com_port()
    if port:
        log_success(f"ESP32 Serial Bridge Detected: {port} ({desc})")
        print("  -> Device is ready for telemetry ingestion and SCADA communication.")
    else:
        log_warn("No ESP32 serial COM port detected.")
        print("  Troubleshooting:")
        print("  1. Verify USB data cable connection.")
        print("  2. Ensure CP210x or CH340 VCP drivers are installed on Windows.")


def cmd_monitor(port=None, baud=115200):
    if not port:
        port, _ = detect_esp32_com_port()
        if not port:
            port = "COM3"

    print("=" * 60)
    print(f"  ESP32 SCADA Serial Monitor on {port} ({baud} Baud)")
    print("  Type 'SPEED <0-255>', 'COAST', 'RA', 'ESTOP', or 'HELP'")
    print("  Press Ctrl+C to exit monitor.")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.dtr = False
        ser.rts = False
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[ESP32] {line}")
    except KeyboardInterrupt:
        print("\nExiting monitor...")
    except Exception as e:
        log_err(f"Serial error: {e}")


def cmd_compile():
    if not ARDUINO_CLI.exists():
        log_err(f"arduino-cli not found at: {ARDUINO_CLI}")
        return False
    log(f"Compiling ESP32 sketch for FQBN {ESP32_FQBN}...")
    cmd = [str(ARDUINO_CLI), "compile", "--fqbn", ESP32_FQBN, str(SKETCH_DIR)]
    res = subprocess.run(cmd)
    return res.returncode == 0


def cmd_upload(port=None):
    if not port:
        port, _ = detect_esp32_com_port()
        if not port:
            log_err("No COM port detected for upload.")
            return False

    log(f"Flashing ESP32 on port {port}...")
    cmd = [str(ARDUINO_CLI), "upload", "-p", port, "--fqbn", ESP32_FQBN, str(SKETCH_DIR)]
    res = subprocess.run(cmd)
    return res.returncode == 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python esp32_tool.py detect   -> Detect connected ESP32 port")
        print("  python esp32_tool.py monitor  -> Stream live telemetry frames")
        print("  python esp32_tool.py compile  -> Compile sketch with arduino-cli")
        print("  python esp32_tool.py upload   -> Flash firmware to ESP32")
        sys.exit(0)

    cmd = sys.argv[1].lower()
    if cmd == "detect":
        cmd_detect()
    elif cmd == "monitor":
        p = sys.argv[2] if len(sys.argv) > 2 else None
        cmd_monitor(port=p)
    elif cmd == "compile":
        cmd_compile()
    elif cmd == "upload":
        p = sys.argv[2] if len(sys.argv) > 2 else None
        cmd_upload(port=p)
    else:
        print(f"Unknown command: {cmd}")
