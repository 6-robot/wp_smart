"""Check gyro bias, tilt correction and body-frame attitude integration."""

import importlib.util
import math
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    'imu_attitude', Path(__file__).resolve().parents[1] / 'scripts' / 'imu_attitude.py')
ATTITUDE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ATTITUDE)


def gravity(roll, pitch):
    return (-9.80665 * math.sin(pitch),
            9.80665 * math.sin(roll) * math.cos(pitch),
            9.80665 * math.cos(roll) * math.cos(pitch))


def test_quaternion_angles_and_invalid_input():
    expected = tuple(math.radians(v) for v in (30, -20, 70))
    quaternion = ATTITUDE.quaternion_from_rpy(*expected)
    assert ATTITUDE.rpy_from_quaternion(
        tuple(2 * v for v in quaternion)) == pytest.approx(expected)
    with pytest.raises(ValueError):
        ATTITUDE.rpy_from_quaternion((0, 0, 0, 0))
    with pytest.raises(ValueError):
        ATTITUDE.rpy_from_quaternion((float('nan'), 0, 0, 1))


def test_stationary_bias_is_removed_without_assuming_exact_gravity_scale():
    estimator = ATTITUDE.AttitudeEstimator(calibration_seconds=2)
    bias = (0.057, -0.008, -0.022)
    for i in range(401):
        angles = estimator.update((0, 0, 11.0), bias, i * 0.01)
        if i < 200:
            assert angles is None
    assert estimator.bias == pytest.approx(bias)
    assert angles == pytest.approx((0, 0, 0), abs=1e-9)


def test_yaw_rotation_while_tilted_uses_body_rates():
    roll, pitch = math.radians(30), math.radians(20)
    acceleration = gravity(roll, pitch)
    rate = tuple(v / 9.80665 * 0.5 for v in acceleration)
    estimator = ATTITUDE.AttitudeEstimator(calibration_seconds=0)
    estimator.update(acceleration, (0, 0, 0), 0)
    for i in range(1, 201):
        angles = estimator.update(acceleration, rate, i * 0.01)
    assert angles == pytest.approx((roll, pitch, 1.0), abs=1e-6)


def test_gravity_corrects_roll_but_not_yaw_and_gap_resets_heading():
    estimator = ATTITUDE.AttitudeEstimator(calibration_seconds=0)
    estimator.update((0, 0, 9.80665), (0, 0, 0), 0)
    estimator.quaternion = ATTITUDE.quaternion_from_rpy(0.2, 0, 0.7)
    for i in range(1, 301):
        angles = estimator.update((0, 0, 9.80665), (0, 0, 0), i * 0.01)
    assert abs(angles[0]) < 0.001
    assert angles[2] == pytest.approx(0.7, abs=1e-6)
    angles = estimator.update((0, 0, 9.80665), (0, 0, 0), 20)
    assert angles == pytest.approx((0, 0, 0))


def test_moving_or_invalid_samples_do_not_complete_bias_calibration():
    estimator = ATTITUDE.AttitudeEstimator(calibration_seconds=0.2)
    for i in range(100):
        assert estimator.update((0, 0, 9.8), (0, 0, 0.5), i * 0.01) is None
    assert not estimator.calibrated
    assert estimator.update((0, 0, 9.8), (float('nan'), 0, 0), 2) is None
    for i in range(100):
        angles = estimator.update((0, 0, 9.8), (0.01, 0, 0), 3 + i * 0.01)
    assert angles == pytest.approx((0, 0, 0), abs=1e-9)


def test_time_discontinuity_restarts_calibration():
    estimator = ATTITUDE.AttitudeEstimator(calibration_seconds=0.2)
    for i in range(10):
        estimator.update((0, 0, 9.8), (0.01, 0, 0), i * 0.01)
    assert estimator.update((0, 0, 9.8), (0.2, 0, 0), 10) is None
    for i in range(30):
        angles = estimator.update((0, 0, 9.8), (0.02, 0, 0), 11 + i * 0.01)
    assert estimator.bias == pytest.approx((0.02, 0, 0))
    assert angles == pytest.approx((0, 0, 0), abs=1e-9)
