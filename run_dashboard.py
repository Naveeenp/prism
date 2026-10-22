#!/usr/bin/env python3
"""
================================================================================
LAUNCH SCRIPT - DC MOTOR SCADA CONTROL & HEALTH MONITORING DASHBOARD
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
    print(f">> Opening default web browser to: {url}")
    webbrowser.open(url)

if __name__ == "__main__":
    if "--no-browser" not in sys.argv:
        t = threading.Thread(target=open_browser, daemon=True)
        t.start()

    print(f"Starting DC Motor Health Monitoring Localhost SCADA Dashboard on port {PORT}...")
    start_server(host="0.0.0.0", port=PORT)
