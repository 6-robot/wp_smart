import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import threading
import time
import tty

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Int32MultiArray


HEAD = b'\xaa\x55'
CMD_VEL = 0x01
STATUS = 0x81
ENCODER_COUNTS_PER_WHEEL_REV = 1024 * 4 * 64
METERS_PER_COUNT = math.pi * 0.0657 / ENCODER_COUNTS_PER_WHEEL_REV


def checksum(msg_type, payload):
    return (msg_type + len(payload) + sum(payload)) & 0xff


def frame(msg_type, payload):
    return HEAD + bytes((msg_type, len(payload))) + payload + bytes((checksum(msg_type, payload),))


class Esp32Simulator:

    def __init__(self, fd):
        self.fd = fd
        self.running = True
        self.buffer = bytearray()
        self.linear_commands = []
        self.angular_commands = []
        self.command_samples = []
        self.last_received_time = None
        self.watchdog_expired = False
        self.watchdog_timeouts = 0
        self.left_count = 0
        self.right_count = 0
        self.rx_bytes = 0
        self.rx_valid = 0
        self.scan_id = 0
        self.last_status_time = time.monotonic()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)

    def parse_commands(self):
        while True:
            start = self.buffer.find(HEAD)
            if start < 0:
                self.buffer.clear()
                return
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return
            payload_size = self.buffer[3]
            frame_size = payload_size + 5
            if len(self.buffer) < frame_size:
                return
            packet = bytes(self.buffer[:frame_size])
            del self.buffer[:frame_size]
            payload = packet[4:-1]
            if packet[-1] != checksum(packet[2], payload):
                continue
            if packet[2] == CMD_VEL and len(payload) == 8:
                linear, angular = struct.unpack('<ff', payload)
                self.linear_commands.append(linear)
                self.angular_commands.append(angular)
                self.command_samples.append((time.monotonic(), linear, angular))
                self.last_received_time = time.monotonic()
                self.watchdog_expired = False
                self.rx_valid += 1

    def send_status(self, now):
        dt = now - self.last_status_time
        self.last_status_time = now
        linear = self.linear_commands[-1] if self.linear_commands else 0.0
        angular = self.angular_commands[-1] if self.angular_commands else 0.0
        if self.last_received_time is not None and now - self.last_received_time >= 3.0:
            if not self.watchdog_expired and (linear != 0.0 or angular != 0.0):
                self.watchdog_timeouts += 1
            self.watchdog_expired = True
            linear = angular = 0.0
        left_speed = linear - angular * 0.1805 / 2.0
        right_speed = linear + angular * 0.1805 / 2.0
        self.left_count += int(left_speed * dt / METERS_PER_COUNT)
        self.right_count -= int(right_speed * dt / METERS_PER_COUNT)
        left_rpm = left_speed / (math.pi * 0.0657) * 60.0
        right_rpm = -right_speed / (math.pi * 0.0657) * 60.0
        payload = struct.pack(
            '<ffffiifhhBIIIII',
            left_rpm, right_rpm, left_rpm, right_rpm,
            self.left_count, self.right_count, 11.8,
            100, -100, 0x01, 2026111503,
            self.rx_bytes, self.rx_valid, 0, 0,
        )
        os.write(self.fd, frame(STATUS, payload))
        stamp = int(now * 1000) & 0xffffffff
        imu = struct.pack('<Iffffff', stamp, 0.1, 0.2, 9.8, 0.01, 0.02, 0.03)
        os.write(self.fd, frame(0x82, imu))
        # Send all four cardinal directions in two fragments, interleaved with IMU.
        first = struct.pack('<IHHHBBHHHH', stamp, self.scan_id, 4, 0, 2, 0,
                            0, 1000, 9000, 2000)
        last = struct.pack('<IHHHBBHHHH', stamp, self.scan_id, 4, 2, 2, 1,
                           18000, 3000, 27000, 4000)
        os.write(self.fd, frame(0x83, first) + frame(0x82, imu) + frame(0x83, last))
        self.scan_id = (self.scan_id + 1) & 0xffff

    def run(self):
        next_status = time.monotonic()
        while self.running:
            readable, _, _ = select.select([self.fd], [], [], 0.005)
            if readable:
                data = os.read(self.fd, 512)
                self.rx_bytes += len(data)
                self.buffer.extend(data)
                self.parse_commands()
            now = time.monotonic()
            if now >= next_status:
                self.send_status(now)
                next_status = now + 0.05


def driver_arguments(slave_name, use_config, tmp_path):
    assert slave_name.startswith('/dev/pts/')
    executable = os.path.join(
        get_package_prefix('smart_bringup'), 'lib', 'smart_bringup', 'smart_base_node')
    arguments = [
        executable,
        '--ros-args',
    ]
    if use_config:
        config = yaml.safe_load(
            (Path(__file__).resolve().parents[1] / 'config/smart_base.yaml').read_text())
        # Never pass a hardware device to the test process, regardless of argument order.
        config['smart_base_node']['ros__parameters']['port'] = slave_name
        params_file = tmp_path / 'pseudo_terminal.yaml'
        params_file.write_text(yaml.safe_dump(config))
        arguments.extend(['--params-file', str(params_file)])
    arguments.extend([
        '-p', f'port:={slave_name}',
        '-p', 'reconnect_interval:=0.05',
        '-p', 'diagnostic_updater.period:=0.2',
    ])
    return arguments


@pytest.mark.parametrize('use_config', [True, False])
def test_node_over_pseudo_terminal(use_config, tmp_path):
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    process = subprocess.Popen(driver_arguments(slave_name, use_config, tmp_path) + [
        '-p', 'enable_cmd_vel_timeout:=true'])
    simulator = Esp32Simulator(master_fd)
    simulator.start()

    rclpy.init()
    node = Node('smart_base_pty_test')
    command_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    odometry_messages = []
    battery_messages = []
    imu_messages, scan_messages, sensor_scan_messages, encoder_messages = [], [], [], []
    sensor_subscriptions = [
        node.create_subscription(Imu, 'imu/data_raw', imu_messages.append,
                                 qos_profile_sensor_data),
        node.create_subscription(LaserScan, 'scan', scan_messages.append,
                                 10),
        node.create_subscription(LaserScan, 'scan', sensor_scan_messages.append,
                                 qos_profile_sensor_data),
        node.create_subscription(Int32MultiArray, 'encoder_counts', encoder_messages.append, 20),
    ]
    odometry_subscription = node.create_subscription(
        Odometry, 'odom', odometry_messages.append, 10)
    battery_subscription = node.create_subscription(
        BatteryState, 'battery_state', battery_messages.append,
        qos_profile_sensor_data)

    try:
        deadline = time.monotonic() + 8.0
        command = Twist()
        command.linear.x = 0.1
        while time.monotonic() < deadline and (
                not odometry_messages or odometry_messages[-1].pose.pose.position.x < 0.01):
            command_publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)

        assert simulator.linear_commands
        assert max(simulator.linear_commands) == pytest.approx(0.1, abs=1.0e-5)
        assert odometry_messages
        assert odometry_messages[-1].pose.pose.position.x > 0.01
        # Compare a time window so serial scheduling jitter cannot skew one sample.
        start_sample = odometry_messages[-1]
        sample_deadline = time.monotonic() + 0.6
        last_publish_at = time.monotonic()
        while time.monotonic() < sample_deadline:
            command_publisher.publish(command)
            last_publish_at = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.05)
        end_sample = odometry_messages[-1]
        start_stamp = start_sample.header.stamp
        end_stamp = end_sample.header.stamp
        elapsed = end_stamp.sec - start_stamp.sec + (
            end_stamp.nanosec - start_stamp.nanosec) * 1.0e-9
        distance = end_sample.pose.pose.position.x - start_sample.pose.pose.position.x
        assert elapsed > 0.3
        assert distance / elapsed == pytest.approx(0.1, abs=0.005)
        assert battery_messages
        assert battery_messages[-1].voltage == pytest.approx(11.8, abs=1.0e-4)
        assert odometry_subscription
        assert battery_subscription
        sensor_deadline = time.monotonic() + 2.0
        while time.monotonic() < sensor_deadline and not (
                imu_messages and scan_messages and sensor_scan_messages and encoder_messages):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert sensor_subscriptions
        assert imu_messages[-1].linear_acceleration.z == pytest.approx(9.8, abs=1e-5)
        assert imu_messages[-1].angular_velocity.z == pytest.approx(0.03, abs=1e-6)
        assert imu_messages[-1].orientation_covariance[0] == -1.0
        scan = scan_messages[-1]
        assert sensor_scan_messages[-1].header.frame_id == scan.header.frame_id
        assert scan.header.frame_id == 'laser_scan_link'
        assert len(scan.ranges) == 450
        assert scan.angle_min == 0.0
        assert scan.angle_max == pytest.approx(449 * scan.angle_increment)
        assert scan.ranges[0] == pytest.approx(1.0)  # Rear.
        assert scan.ranges[112] == pytest.approx(4.0)  # Right (90 degrees).
        assert scan.ranges[225] == pytest.approx(3.0)  # Front.
        assert scan.ranges[337] == pytest.approx(2.0)  # Left (270 degrees).
        assert sum(math.isfinite(value) for value in scan.ranges) == 4
        assert encoder_messages[-1].data[0] > 0
        assert encoder_messages[-1].data[1] < 0

        timeout_deadline = last_publish_at + 4.5
        stop_samples = []
        while time.monotonic() < timeout_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            stop_samples = [stamp for stamp, linear, _ in simulator.command_samples
                            if stamp > last_publish_at + 0.1 and abs(linear) < 1.0e-6]
            if stop_samples:
                break
        assert simulator.linear_commands[-1] == pytest.approx(0.0, abs=1.0e-6)
        assert stop_samples
        # Both installed defaults and the checked-in YAML must hold the command
        # for three seconds, then send zero within normal serial/timer jitter.
        assert 2.9 <= stop_samples[0] - last_publish_at <= 3.5
    finally:
        node.destroy_node()
        rclpy.shutdown()
        simulator.stop()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        os.close(slave_fd)
        os.close(master_fd)

    assert process.returncode in (0, -15)


@pytest.mark.parametrize('use_config', [True, False])
def test_timeout_disabled_keepalive_new_commands_and_shutdown(use_config, tmp_path):
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    process = subprocess.Popen(driver_arguments(os.ttyname(slave_fd), use_config, tmp_path))
    simulator = Esp32Simulator(master_fd)
    simulator.start()
    rclpy.init()
    node = Node('keepalive_pty_test')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait_for(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not predicate():
            spin_for(0.01)
        assert predicate()

    try:
        wait_for(lambda: publisher.get_subscription_count() > 0 and simulator.command_samples, 5)
        assert all(linear == 0 and angular == 0
                   for _, linear, angular in simulator.command_samples)
        command = Twist()
        command.linear.x, command.angular.z = 0.2, -0.4
        sent = time.monotonic()
        publisher.publish(command)
        wait_for(lambda: abs(simulator.linear_commands[-1] - 0.2) < 1e-6, 0.4)
        first = simulator.command_samples[-1]
        assert first[0] - sent < 0.3
        start_index = len(simulator.command_samples) - 1
        spin_for(1.2)
        invalid = Twist()
        invalid.linear.x = float('nan')
        publisher.publish(invalid)  # Invalid input must not replace the held velocity.
        spin_for(max(0, first[0] + 4.3 - time.monotonic()))
        held = simulator.command_samples[start_index:]
        assert 5 <= len(held) <= 6
        assert all(linear == pytest.approx(0.2) and angular == pytest.approx(-0.4)
                   for _, linear, angular in held)
        gaps = [right[0] - left[0] for left, right in zip(held, held[1:])]
        assert all(0.85 <= gap <= 1.25 for gap in gaps)
        assert simulator.watchdog_timeouts == 0

        command.linear.x, command.angular.z = -0.1, 0.2
        sent = time.monotonic()
        publisher.publish(command)
        wait_for(lambda: abs(simulator.linear_commands[-1] + 0.1) < 1e-6, 0.4)
        assert simulator.command_samples[-1][0] - sent < 0.3

        sent = time.monotonic()
        publisher.publish(Twist())
        wait_for(lambda: simulator.linear_commands[-1] == 0, 0.4)
        assert simulator.command_samples[-1][0] - sent < 0.3
        zero_index = len(simulator.command_samples) - 1
        spin_for(1.15)
        assert len(simulator.command_samples) > zero_index + 1
        assert all(linear == 0 and angular == 0
                   for _, linear, angular in simulator.command_samples[zero_index:])
        assert simulator.watchdog_timeouts == 0

        publisher.publish(command)
        wait_for(lambda: abs(simulator.linear_commands[-1] + 0.1) < 1e-6, 0.4)
        before_shutdown = len(simulator.command_samples)
        process.terminate()
        process.wait(timeout=3)
        spin_for(0.1)
        shutdown_samples = simulator.command_samples[before_shutdown:]
        assert len(shutdown_samples) >= 5
        assert all(linear == 0 and angular == 0 for _, linear, angular in shutdown_samples)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        simulator.stop()
        node.destroy_node()
        rclpy.shutdown()
        os.close(slave_fd)
        os.close(master_fd)


def test_timeout_disabled_reconnect_does_not_resume_old_velocity(tmp_path):
    first_master, first_slave = pty.openpty()
    tty.setraw(first_slave)
    port_link = tmp_path / 'base_uart'
    port_link.symlink_to(os.ttyname(first_slave))
    assert str(port_link.resolve()).startswith('/dev/pts/')
    arguments = driver_arguments(os.ttyname(first_slave), False, tmp_path)
    arguments.extend(['-p', f'port:={port_link}'])
    process = subprocess.Popen(arguments)
    first_sim = Esp32Simulator(first_master)
    first_sim.start()
    second_master = second_slave = None
    second_sim = None
    first_closed = False
    rclpy.init()
    node = Node('keepalive_reconnect_test')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate()

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0 and first_sim.command_samples, 5)
        command = Twist()
        command.linear.x = 0.1
        publisher.publish(command)
        spin_until(lambda: first_sim.linear_commands[-1] > 0.09, 0.4)
        first_sim.stop()
        os.close(first_master)
        os.close(first_slave)
        first_closed = True
        second_master, second_slave = pty.openpty()
        tty.setraw(second_slave)
        port_link.unlink()
        port_link.symlink_to(os.ttyname(second_slave))
        assert str(port_link.resolve()).startswith('/dev/pts/')
        second_sim = Esp32Simulator(second_master)
        second_sim.start()
        spin_until(lambda: len(second_sim.command_samples) >= 2, 4)
        assert all(linear == 0 and angular == 0
                   for _, linear, angular in second_sim.command_samples)
        publisher.publish(command)
        spin_until(lambda: second_sim.linear_commands[-1] > 0.09, 0.4)
        publisher.publish(Twist())
        spin_until(lambda: second_sim.linear_commands[-1] == 0, 0.4)
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        if not first_closed:
            first_sim.stop()
            os.close(first_master)
            os.close(first_slave)
        if second_sim is not None:
            second_sim.stop()
        if second_slave is not None:
            os.close(second_slave)
            os.close(second_master)
        node.destroy_node()
        rclpy.shutdown()
