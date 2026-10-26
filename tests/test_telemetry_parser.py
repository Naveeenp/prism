import sys
import os
import json
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from backend_engine import ESP32SerialWorker


def test_telemetry_json_parsing():
    received = []
    worker = ESP32SerialWorker(data_callback=lambda d: received.append(d))

    raw_json = '{"mode": "RUN_75", "v_bus": 12.04, "i_ma": 320.5, "rpm": 1820.0, "temp_c": 31.4, "vib_g": 0.18, "ts_ms": 104250, "driver_pwr": 1, "duty": 191, "seq": 45, "pulses": 360, "ina_ok": 1, "mpu_ok": 1}'
    worker._parse_incoming_line(raw_json)

    assert len(received) == 1
    frame = received[0]
    assert frame["mode"] == "RUN_75"
    assert frame["v_bus"] == 12.04
    assert frame["i_ma"] == 320.5
    assert frame["rpm"] == 1820.0
    assert frame["duty"] == 191
    assert frame["seq"] == 45
    assert frame["pulses"] == 360
    assert frame["ina_ok"] == 1
    assert frame["mpu_ok"] == 1
