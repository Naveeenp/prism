import sys
import os
import math
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from backend_engine import CoastDownAnalyzer, AlarmSeverity


def test_coast_down_ols_linear_deceleration():
    """
    Synthetic linear deceleration:
    omega(t) = 200.0 - 100.0 * t  (from t=0.0s to t=2.0s)
    decel = -domega/dt = 100.0 rad/s^2 (constant Coulomb torque)
    """
    analyzer = CoastDownAnalyzer(rotor_inertia_kg_m2=1.25e-5, nominal_coast_time_s=2.20)
    analyzer.start_test(start_time_s=0.0)

    # Feed samples at 10 Hz from t=0.0s to t=1.9s (speed drops from ~1910 RPM to ~95 RPM)
    for i in range(20):
        t = i * 0.10
        omega = max(0.0, 200.0 - 100.0 * t)
        rpm = omega * 60.0 / (2.0 * math.pi)
        analyzer.feed_sample(t, rpm)

    # Final sample at t=2.0s (shaft stops)
    analyzer.feed_sample(2.0, 0.0)

    assert analyzer.last_coast_duration_s == pytest.approx(2.0, abs=0.05)
    assert analyzer.last_avg_decel_rad_s2 == pytest.approx(100.0, rel=0.05)
    # BCI = 2.0 / 2.2 * 100 ≈ 90.9%
    assert analyzer.last_bci_pct == pytest.approx(90.9, rel=0.05)
