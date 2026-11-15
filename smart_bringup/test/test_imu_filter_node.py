"""Exercise the installed Madgwick adapter using synthetic IMU messages only."""
import math
import os
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def executable():
    return os.path.join(
        get_package_prefix('smart_bringup'), 'lib/smart_bringup/smart_imu_filter_node')


def test_synthetic_imu_bias_orientation_headers_and_reconnect(tmp_path):
    # This executable never opens serial ports and never publishes cmd_vel.
    with (tmp_path / 'filter.log').open('w') as log:
        process = subprocess.Popen([
            executable(), '--ros-args', '-p', 'calibration_seconds:=0.2',
            '-p', 'min_calibration_samples:=20',
            '-r', 'imu/data_raw:=/fusion_test/raw', '-r', 'imu/data:=/fusion_test/data',
        ], stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node('fusion_synthetic_test')
        publisher = node.create_publisher(Imu, '/fusion_test/raw', qos_profile_sensor_data)
        received = []
        subscription = node.create_subscription(Imu, '/fusion_test/data', received.append, 10)

        def drain(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)

        def publish(stamp, gyro, frame='imu_test_link'):
            msg = Imu()
            msg.header.frame_id = frame
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp * 1e9), 10**9)
            msg.linear_acceleration.z = 11.0
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = 0.1
            msg.linear_acceleration_covariance[0] = 0.4
            publisher.publish(msg)
            drain(0.012)
            return msg

        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and (
                    publisher.get_subscription_count() == 0 or
                    node.count_publishers('/fusion_test/data') == 0):
                drain(0.05)
            assert publisher.get_subscription_count() > 0
            bias = (0.057, -0.008, -0.022)
            for i in range(45):
                publish(10 + i * 0.01, bias)
            assert subscription and received
            assert all(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= 10.19
                       for m in received)
            message = received[-1]
            assert message.header.frame_id == 'imu_test_link'
            assert message.linear_acceleration.z == 11.0
            assert message.linear_acceleration_covariance[0] == 0.4
            assert message.angular_velocity_covariance[0] == 0.1
            assert message.angular_velocity.x == pytest.approx(0, abs=1e-9)
            assert message.angular_velocity.y == pytest.approx(0, abs=1e-9)
            assert message.angular_velocity.z == pytest.approx(0, abs=1e-9)
            assert message.orientation_covariance[0] == 0.0

            for i in range(1, 101):
                raw = publish(10.44 + i * 0.01, (bias[0], bias[1], bias[2] + 0.5))
            drain(0.1)
            message = received[-1]
            assert message.header == raw.header
            q = message.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            assert yaw == pytest.approx(0.5, abs=0.03)
            assert q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z == pytest.approx(1.0, abs=1e-8)

            count = len(received)
            drain(0.3)
            assert len(received) == count  # No stale samples are re-published.
            publish(20, bias)
            drain(0.05)
            assert abs(received[-1].orientation.z) < 1e-6  # Gap resets relative heading.
            count = len(received)
            publish(20.01, (float('nan'), 0.0, 0.0))
            publish(20.02, bias, frame='')
            drain(0.05)
            assert len(received) == count

            new_bias = (0.03, -0.01, 0.01)
            for i in range(10):
                publish(21 + i * 0.01, new_bias, frame='new_imu_link')
            assert len(received) == count
            for i in range(10, 45):
                publish(21 + i * 0.01, new_bias, frame='new_imu_link')
            drain(0.1)
            assert received[-1].header.frame_id == 'new_imu_link'
            assert received[-1].angular_velocity.z == pytest.approx(0, abs=1e-9)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        assert process.returncode in (0, -15)


def test_input_output_loop_is_rejected():
    result = subprocess.run([
        executable(), '--ros-args', '-p', 'input_topic:=imu/data', '-p', 'output_topic:=imu/data',
    ], capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert 'input and output topics must differ' in result.stderr
