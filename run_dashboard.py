#!/usr/bin/env python3
"""
================================================================================
PRISM LAUNCH SCRIPT - DC MOTOR CONTROL & HEALTH MONITORING WORKSTATION
================================================================================
Usage:
    python run_dashboard.py [--port 8080] [--no-browser]
================================================================================
"""

import sys
import os
import webbrowser
import threading
import time

# Ensure dashboard module can be found
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dashboard.server import start_server

PORT = 8080

def open_browser():
    time.sleep(1.2)
    url = f"http://localhost:{PORT}"
    print(f">> Opening PRISM workstation web browser to: {url}")
    webbrowser.open(url)

if __name__ == "__main__":
    host = "127.0.0.1"
    if "--all-interfaces" in sys.argv:
        host = "0.0.0.0"

    for i, arg in enumerate(sys.argv):
        if arg == "--port" and i + 1 < len(sys.argv):
            try:
                PORT = int(sys.argv[i + 1])
            except ValueError:
                pass

    if "--no-browser" not in sys.argv:
        t = threading.Thread(target=open_browser, daemon=True)
        t.start()

    print(f"Starting PRISM Motor Health Monitoring Dashboard on {host}:{PORT}...")
    start_server(host=host, port=PORT)
