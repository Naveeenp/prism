import math
import pytest


def calculate_rpm_window(pulses: int, ppr: int, dt_ms: float) -> float:
    """Calculates pulse frequency RPM over a measurement window."""
    if dt_ms <= 0 or ppr <= 0:
        return 0.0
    return (float(pulses) / float(ppr)) * (60000.0 / float(dt_ms))


def calculate_rpm_interval(interval_us: float, ppr: int) -> float:
    """Calculates period-based RPM from microsecond interval between transitions."""
    if interval_us <= 0 or ppr <= 0:
        return 0.0
    return (60.0 * 1000000.0) / (float(ppr) * float(interval_us))


def test_encoder_ppr_8_steady_state():
    # 4-slot disc with CHANGE mode IRQ = 8 transitions/rev
    # At 1500 RPM: 1500 rev/min * 8 transitions/rev / 60s = 200 transitions/sec
    # In a 100ms telemetry window, expected transitions = 20
    rpm = calculate_rpm_window(pulses=20, ppr=8, dt_ms=100.0)
    assert math.isclose(rpm, 1500.0, rel_tol=1e-3)


def test_encoder_period_calculation():
    # At 3000 RPM: interval between pulses = 60e6 / (8 * 3000) = 2500 us
    interval_us = 2500.0
    rpm = calculate_rpm_interval(interval_us=interval_us, ppr=8)
    assert math.isclose(rpm, 3000.0, rel_tol=1e-3)


def test_encoder_zero_pulses():
    rpm = calculate_rpm_window(pulses=0, ppr=8, dt_ms=100.0)
    assert rpm == 0.0
