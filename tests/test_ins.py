#!/usr/bin/env python3
"""
test_ins.py
===========
Unit tests for EKF, dead-reckoning, and noise params.
Run: pytest tests/test_ins.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import math
import numpy as np
import pytest
from ekf_core         import EKFCore
from imu_noise_params import IMUNoiseParams
from dead_reckon      import DeadReckon


# ── fixtures ────────────────────────────────────────────────────
@pytest.fixture
def noise():
    return IMUNoiseParams()

@pytest.fixture
def ekf(noise):
    return EKFCore(noise)

@pytest.fixture
def dr(noise):
    return DeadReckon(noise)


# ── EKF tests ───────────────────────────────────────────────────
class TestEKFCore:

    def test_initial_state_zeros(self, ekf):
        assert np.allclose(ekf.x, 0.0)

    def test_covariance_positive_definite(self, ekf):
        eigvals = np.linalg.eigvalsh(ekf.P)
        assert np.all(eigvals > 0), "Initial P must be positive definite"

    def test_predict_increases_uncertainty(self, ekf):
        """Covariance trace should grow when predicting with no updates."""
        trace_before = np.trace(ekf.P)
        accel = np.array([0.0, 0.0, -9.80665])  # hovering
        gyro  = np.zeros(3)
        for _ in range(50):
            ekf.predict(accel, gyro, 0.01)
        trace_after = np.trace(ekf.P)
        assert trace_after > trace_before

    def test_predict_gravity_stationary(self, ekf):
        """Stationary hover: position should stay near zero."""
        accel = np.array([0.0, 0.0, -9.80665])
        gyro  = np.zeros(3)
        for _ in range(100):
            ekf.predict(accel, gyro, 0.01)
        pos = ekf.state["pos"]
        # Dead-reckoning still drifts, but should be < 1 m after 1 s
        assert np.linalg.norm(pos) < 1.0

    def test_baro_update_corrects_altitude(self, ekf):
        """Injecting baro at z=10 m should pull EKF z toward it."""
        # First let some predict steps happen
        accel = np.array([0.0, 0.0, -9.80665])
        for _ in range(10):
            ekf.predict(accel, np.zeros(3), 0.01)
        # Feed baro = 10 m altitude  → NED z = -10 m
        ekf.update_baro(10.0)
        assert ekf.x[2] < 0.0, "NED z should be negative after baro update for +10 m altitude"

    def test_mag_update_sets_yaw(self, ekf):
        """Mag update should pull yaw toward measurement."""
        target_yaw = math.radians(45.0)
        for _ in range(5):
            ekf.update_mag(target_yaw)
        assert abs(ekf.x[8] - target_yaw) < math.radians(5.0)

    def test_covariance_decreases_after_update(self, ekf):
        """Covariance trace should decrease after a measurement update."""
        accel = np.array([0.0, 0.0, -9.80665])
        for _ in range(20):
            ekf.predict(accel, np.zeros(3), 0.01)
        trace_before = np.trace(ekf.P)
        ekf.update_baro(5.0)
        trace_after = np.trace(ekf.P)
        assert trace_after < trace_before

    def test_reset(self, ekf):
        ekf.x[0] = 999.0
        ekf.reset()
        assert np.allclose(ekf.x, 0.0)

    def test_angle_wrap(self, ekf):
        wrapped = ekf._wrap_angle(math.pi + 0.1)
        assert abs(wrapped) <= math.pi


# ── Dead Reckon tests ────────────────────────────────────────────
class TestDeadReckon:

    def test_stationary(self, dr):
        accel = np.array([0.0, 0.0, -9.80665])
        gyro  = np.zeros(3)
        for _ in range(100):
            dr.update(accel, gyro, 0.01)
        assert np.linalg.norm(dr.pos) < 2.0   # some drift expected

    def test_forward_motion(self, dr):
        """Small forward accel should produce positive px."""
        accel = np.array([0.5, 0.0, -9.80665])
        gyro  = np.zeros(3)
        for _ in range(100):
            dr.update(accel, gyro, 0.01)
        assert dr.pos[0] > 0.0


# ── Noise Params tests ───────────────────────────────────────────
class TestIMUNoiseParams:

    def test_defaults_positive(self, noise):
        assert noise.accel_std > 0
        assert noise.gyro_std  > 0
        assert noise.baro_std  > 0
        assert noise.mag_std   > 0

    def test_summary_string(self, noise):
        s = noise.summary()
        assert "Accel" in s
        assert "Baro"  in s


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
