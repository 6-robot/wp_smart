"""Estimate display-only attitude from a six-axis IMU in its sensor frame."""

import math


def quaternion_from_rpy(roll, pitch, yaw):
    """Return a scalar-first quaternion for fixed-axis roll, pitch and yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def rpy_from_quaternion(quaternion):
    """Normalize a scalar-first quaternion and return angles in radians."""
    if not all(math.isfinite(v) for v in quaternion):
        raise ValueError('non-finite quaternion')
    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm < 1e-9:
        raise ValueError('zero quaternion')
    w, x, y, z = (v / norm for v in quaternion)
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class AttitudeEstimator:
    """Fuse gravity direction and body gyro rates; yaw has no absolute reference."""

    def __init__(self, calibration_seconds=2.0, correction_gain=2.0):
        if (not math.isfinite(calibration_seconds) or calibration_seconds < 0 or
                not math.isfinite(correction_gain) or correction_gain < 0):
            raise ValueError('calibration duration and gain must be finite and nonnegative')
        self.calibration_seconds = calibration_seconds
        self.correction_gain = correction_gain
        self.bias = (0.0, 0.0, 0.0)
        self.calibrated = calibration_seconds == 0
        self.quaternion = None
        self.previous_stamp = None
        self.status = 'waiting for IMU'
        self._restart_calibration()

    def _restart_calibration(self):
        self._calibration_start = None
        self._calibration_reference = None
        self._gyro_sum = [0.0, 0.0, 0.0]
        self._samples = 0

    def update(self, acceleration, gyro, stamp):
        """Return RPY or None while waiting, calibrating or rejecting a sample."""
        if not all(math.isfinite(v) for v in (*acceleration, *gyro, stamp)):
            self.status = 'invalid IMU sample'
            return None
        norm = math.sqrt(sum(v * v for v in acceleration))
        gravity_valid = 6.0 <= norm <= 15.0
        unit = tuple(v / norm for v in acceleration) if norm > 1e-9 else (0, 0, 0)
        if not self.calibrated:
            moved = (self._calibration_reference is not None and
                     sum((a - b) ** 2 for a, b in
                         zip(unit, self._calibration_reference)) > 0.01)
            gap = (self.previous_stamp is not None and
                   not 0 < stamp - self.previous_stamp <= 0.2)
            if not gravity_valid or math.sqrt(sum(v * v for v in gyro)) > 0.3 or moved or gap:
                self._restart_calibration()
                self.previous_stamp = stamp
                self.status = 'calibration: keep IMU still'
                return None
            if self._calibration_start is None:
                self._calibration_start = stamp
                self._calibration_reference = unit
            self._samples += 1
            self._gyro_sum = [a + b for a, b in zip(self._gyro_sum, gyro)]
            self.previous_stamp = stamp
            elapsed = stamp - self._calibration_start
            if elapsed < self.calibration_seconds or self._samples < 20:
                self.status = (
                    f'calibration: keep still {elapsed:.1f}/{self.calibration_seconds:.1f}s')
                return None
            self.bias = tuple(v / self._samples for v in self._gyro_sum)
            self.calibrated = True

        dt = stamp - self.previous_stamp if self.previous_stamp is not None else 0.0
        self.previous_stamp = stamp
        if self.quaternion is None or dt <= 0 or dt > 0.2:
            if not gravity_valid:
                self.status = 'waiting for valid gravity to initialize tilt'
                return None
            ax, ay, az = acceleration
            self.quaternion = quaternion_from_rpy(
                math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az)), 0.0)
            self.status = 'raw IMU estimate; relative yaw reset to zero'
            return rpy_from_quaternion(self.quaternion)

        w, x, y, z = self.quaternion
        omega = [v - b for v, b in zip(gyro, self.bias)]
        if gravity_valid:
            predicted = (2 * (x * z - w * y), 2 * (w * x + y * z),
                         1 - 2 * (x * x + y * y))
            # Measured gravity cross predicted gravity corrects tilt, not heading.
            error = (unit[1] * predicted[2] - unit[2] * predicted[1],
                     unit[2] * predicted[0] - unit[0] * predicted[2],
                     unit[0] * predicted[1] - unit[1] * predicted[0])
            omega = [v + self.correction_gain * e for v, e in zip(omega, error)]
        magnitude = math.sqrt(sum(v * v for v in omega))
        scale = math.sin(magnitude * dt / 2) / magnitude if magnitude > 1e-12 else dt / 2
        dw = math.cos(magnitude * dt / 2)
        dx, dy, dz = (v * scale for v in omega)
        updated = (w * dw - x * dx - y * dy - z * dz,
                   w * dx + x * dw + y * dz - z * dy,
                   w * dy - x * dz + y * dw + z * dx,
                   w * dz + x * dy - y * dx + z * dw)
        length = math.sqrt(sum(v * v for v in updated))
        self.quaternion = tuple(v / length for v in updated)
        self.status = 'raw IMU estimate; relative yaw may drift'
        return rpy_from_quaternion(self.quaternion)
