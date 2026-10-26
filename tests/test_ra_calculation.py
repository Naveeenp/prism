import sys
import os
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from backend_engine import ArmatureResistanceEstimator, AlarmSeverity


def test_ra_estimation_nominal():
    # If motor has nominal Ra = 2.40 Ohms
    # With Vce_sat_low drop of 1.0V:
    # At 5.0A current: V_motor = 5.0A * 2.40 Ohms = 12.0V
    # Measured bus voltage = V_motor + 1.0V = 13.0V
    estimator = ArmatureResistanceEstimator(nominal_ra_ohms=2.40)
    for _ in range(6):
        estimator.record_sample(v_bus=13.0, i_ma=5000.0)

    ra, status, msg = estimator.compute_ra()
    assert ra == pytest.approx(2.40, abs=0.05)
    assert status == AlarmSeverity.NORMAL


def test_ra_estimation_short_turns():
    # Insulation breakdown: Ra drops to 1.2 Ohms (-50% deviation)
    # At 5.0A: V_motor = 6.0V, V_bus = 7.0V
    estimator = ArmatureResistanceEstimator(nominal_ra_ohms=2.40)
    for _ in range(6):
        estimator.record_sample(v_bus=7.0, i_ma=5000.0)

    ra, status, msg = estimator.compute_ra()
    assert ra == pytest.approx(1.20, abs=0.05)
    assert status == AlarmSeverity.CRITICAL
