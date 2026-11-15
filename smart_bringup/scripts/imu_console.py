#!/usr/bin/env python3

"""Display IMU roll, pitch and heading in the launching terminal."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from imu_attitude import AttitudeEstimator, rpy_from_quaternion


class ImuConsole(Node):
    """Print received attitude, or estimate tilt and relative yaw from raw IMU."""

    def __init__(self):
        super().__init__('imu_console')
        topic = self.declare_parameter('imu_topic', '/imu/data').value
        rate = float(self.declare_parameter('display_rate', 5.0).value)
        duration = float(self.declare_parameter('calibration_seconds', 2.0).value)
        gain = float(self.declare_parameter('correction_gain', 2.0).value)
        if not math.isfinite(rate) or not 0 < rate <= 50:
            raise ValueError('display_rate must be in (0, 50] Hz')
        self._estimator = AttitudeEstimator(duration, gain)
        self._angles = None
        self._last_received = None
        self._status = 'waiting for IMU'
        self._mode = 'raw'
        self._frame = ''
        self._bias_reported = False
        self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._print_latest)
        self.get_logger().info(
            f'Listening on {topic}; angles in degrees, in the IMU sensor frame. '
            f'Keep still for {duration:.1f}s to estimate gyro bias. '
            'Raw six-axis yaw is relative and can drift.')

    def _on_imu(self, message):
        self._last_received = time.monotonic()
        self._frame = message.header.frame_id
        q = message.orientation
        if message.orientation_covariance[0] >= 0:
            try:
                self._angles = rpy_from_quaternion((q.w, q.x, q.y, q.z))
                self._mode = 'orientation'
                self._status = 'received orientation'
            except ValueError:
                self._angles = None
                self._status = 'invalid orientation quaternion'
            return
        # A switch from an external orientation starts a fresh relative estimate.
        if self._mode != 'raw':
            self._estimator = AttitudeEstimator(
                self._estimator.calibration_seconds, self._estimator.correction_gain)
            self._bias_reported = False
        self._mode = 'raw'
        acceleration = message.linear_acceleration
        gyro = message.angular_velocity
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp == 0:
            stamp = self._last_received
        self._angles = self._estimator.update(
            (acceleration.x, acceleration.y, acceleration.z),
            (gyro.x, gyro.y, gyro.z), stamp)
        self._status = self._estimator.status
        if self._estimator.calibrated and not self._bias_reported:
            self._bias_reported = True
            self.get_logger().info(
                'Gyro bias (rad/s): [%+.5f, %+.5f, %+.5f]' % self._estimator.bias)

    def _print_latest(self):
        if self._last_received is None or time.monotonic() - self._last_received > 0.5:
            self.get_logger().warn('Waiting for fresh IMU data ...', throttle_duration_sec=2.0)
        elif self._angles is None:
            self.get_logger().info(self._status, throttle_duration_sec=1.0)
        else:
            roll, pitch, yaw = (math.degrees(v) for v in self._angles)
            heading = 'yaw(relative)' if self._mode == 'raw' else 'yaw'
            self.get_logger().info(
                f'[{self._frame}] roll={roll:+8.2f} pitch={pitch:+8.2f} '
                f'{heading}={yaw:+8.2f} deg')


def main(args=None):
    """Run the terminal display without sending chassis commands."""
    rclpy.init(args=args)
    node = None
    try:
        node = ImuConsole()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
