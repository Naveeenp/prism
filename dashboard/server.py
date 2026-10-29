#!/usr/bin/env python3
"""
================================================================================
PRISM MOTOR HEALTH WORKSTATION - FASTAPI GATEWAY & WEBSOCKET SERVER
================================================================================
Compliance: ISA-101 Industrial Telemetry Ergonomics
Backend: Uses backend_engine.SCADABackendEngine (ESP-32S Serial & Analytics)
Protocols: WebSocket (/ws/telemetry) + RESTful Control Endpoints + Static Files
================================================================================
"""

import os
import sys
import asyncio
from typing import Dict, Any, List, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
import logging

logger = logging.getLogger("PRISMWebServer")

# Ensure local workspace imports work reliably
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from backend_engine import SCADABackendEngine, AlarmSeverity

# Instantiate Unified Backend Engine
engine = SCADABackendEngine()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: backend engine is initialized
    yield
    # Shutdown: clean up threads, serial ports, and CSV files
    engine.shutdown()


app = FastAPI(
    title="PRISM Motor Health Gateway",
    description="ISA-101 Industrial Telemetry and Predictive Diagnostic Web API",
    version="2.0.0",
    lifespan=lifespan
)

STATIC_DIR = os.path.join(BASE_DIR, "dashboard", "static")


# ==============================================================================
# REST API ENDPOINTS
# ==============================================================================

@app.get("/api/ports")
async def get_ports():
    """Lists available serial COM ports."""
    ports = engine.get_serial_ports()
    return {
        "ports": ports,
        "current_port": engine.serial_worker.port,
        "connected": engine.serial_worker.is_connected
    }


@app.get("/api/status")
@app.get("/api/telemetry")
async def get_telemetry():
    """Returns a snapshot of the latest telemetry and connection status."""
    with engine.lock:
        telemetry = dict(engine.latest_telemetry)
        telemetry["connected"] = engine.serial_worker.is_connected
        telemetry["port"] = engine.serial_worker.port
        telemetry["baud"] = engine.serial_worker.baud
        telemetry["slope_markers"] = engine.coast_analyzer.slope_markers
    return telemetry


@app.get("/api/logs")
async def get_logs():
    """Returns the chronological alarm and event ledger."""
    with engine.lock:
        logs = [ev.to_dict() for ev in reversed(engine.alarm_ledger[-50:])]
    return {"logs": logs}


@app.post("/api/serial/connect")
async def connect_serial(payload: Dict[str, Any]):
    """Connects to the specified ESP32 serial COM port."""
    port = str(payload.get("port", "COM3")).strip()
    try:
        baud = int(payload.get("baudrate", 115200))
    except (ValueError, TypeError):
        baud = 115200

    success, msg = engine.serial_worker.connect(port, baud)
    if success:
        engine._add_alarm(AlarmSeverity.NORMAL, "SERIAL", f"Connected to ESP32 on {port}")
    return {"success": success, "message": msg, "port": port}


@app.post("/api/serial/disconnect")
async def disconnect_serial():
    """Disconnects from the active serial port."""
    engine.serial_worker.disconnect()
    engine._add_alarm(AlarmSeverity.CAUTION, "SERIAL", "ESP32 disconnected")
    return {"success": True, "message": "Disconnected"}


@app.post("/api/control")
async def execute_control(payload: Dict[str, Any]):
    """Executes motor commands and diagnostic test routines."""
    action = str(payload.get("action", "")).lower()

    if action == "set_speed":
        try:
            speed = float(payload.get("speed", 0.0))
            engine.set_target_speed(speed)
            return {"success": True, "speed": speed}
        except (ValueError, TypeError):
            raise HTTPException(status_code=400, detail="Invalid speed value")

    elif action == "driver_power":
        enable = bool(payload.get("enable", True))
        engine.set_driver_power(enable)
        return {"success": True, "driver_pwr": enable}

    elif action in ["stop", "estop"]:
        engine.emergency_stop()
        return {"success": True, "message": "E-STOP Activated"}

    elif action in ["wind_down", "winddown", "normal_stop"]:
        engine.normal_wind_down()
        return {"success": True, "message": "Normal Wind Down Initiated"}

    elif action == "ra_test":
        engine.trigger_ra_measurement()
        return {"success": True, "message": "Ra measurement triggered"}

    elif action == "coast_test":
        engine.trigger_coast_down_test()
        return {"success": True, "message": "Coast-down test triggered"}

    elif action == "inject_fault":
        faults = payload.get("faults", {})
        engine.inject_faults(faults)
        return {"success": True, "faults": faults}

    elif action == "clear_faults":
        engine.clear_all_faults()
        return {"success": True, "message": "All simulation faults cleared"}

    else:
        raise HTTPException(status_code=400, detail=f"Unknown action: {action}")


# ==============================================================================
# WEBSOCKET REAL-TIME TELEMETRY STREAM (10 HZ DUAL-WAY)
# ==============================================================================

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """
    Bi-directional 10 Hz telemetry and command stream.
    Streams telemetry frames to client and accepts inbound commands via JSON.
    """
    await websocket.accept()

    async def _send_loop():
        try:
            while True:
                with engine.lock:
                    telem = dict(engine.latest_telemetry)
                    telem["connected"] = engine.serial_worker.is_connected
                    telem["port"] = engine.serial_worker.port
                    telem["slope_markers"] = engine.coast_analyzer.slope_markers

                await websocket.send_json(telem)
                await asyncio.sleep(0.10) # 10 Hz transmission rate
        except (WebSocketDisconnect, asyncio.CancelledError):
            pass
        except Exception:
            pass

    async def _recv_loop():
        try:
            while True:
                data = await websocket.receive_json()
                action = str(data.get("action", "")).lower()
                if action == "set_speed":
                    engine.set_target_speed(float(data.get("speed", 0.0)))
                elif action == "driver_power":
                    engine.set_driver_power(bool(data.get("enable", True)))
                elif action in ["stop", "estop"]:
                    engine.emergency_stop()
                elif action == "ra_test":
                    engine.trigger_ra_measurement()
                elif action == "coast_test":
                    engine.trigger_coast_down_test()
                elif action == "inject_fault":
                    faults = data.get("faults", {})
                    if isinstance(faults, dict):
                        engine.inject_faults(faults)
                elif action == "clear_faults":
                    engine.clear_all_faults()
        except (WebSocketDisconnect, asyncio.CancelledError):
            pass
        except Exception as e:
            logger.debug(f"WebSocket receive error: {e}")

    send_task = asyncio.create_task(_send_loop())
    recv_task = asyncio.create_task(_recv_loop())

    done, pending = await asyncio.wait(
        [send_task, recv_task],
        return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()


# ==============================================================================
# STATIC FILES SERVING
# ==============================================================================

@app.get("/")
async def serve_index():
    return FileResponse(
        os.path.join(STATIC_DIR, "index.html"),
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0"
        }
    )

app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")


def start_server(host="127.0.0.1", port=8080):
    print("\n" + "=" * 70)
    print("  PRISM - PMDC MOTOR HEALTH MONITOR & TELEMETRY WORKSTATION")
    print("  Standards: ISA-101 Industrial Ergonomics | High-Contrast Workspace")
    print("=" * 70)
    print(f"  URL              : http://{host}:{port}")
    print("  Transport        : Full-Duplex WebSocket (/ws/telemetry) + REST API")
    print("  Backend Target   : ESP-32S (38-pin NodeMCU) / Auto-Simulation")
    print("  Baud Rate        : 115200 Baud")
    print("  Press Ctrl+C to stop.")
    print("=" * 70 + "\n")

    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    host = "127.0.0.1"
    port = 8080
    if len(sys.argv) > 1 and sys.argv[1] == "--all-interfaces":
        host = "0.0.0.0"
    start_server(host=host, port=port)
