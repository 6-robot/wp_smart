#!/usr/bin/env python3

"""Passively identify smart base and YDLIDAR serial protocols."""

import argparse
import math
import os
import select
import struct
import termios
import time


BASE_LENGTHS = (33, 37, 53)
MIN_VALID_FRAMES = 2


class BaseParser:

    def __init__(self):
        self.buffer = bytearray()
        self.valid_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xaa' else b''
                return
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return
            msg_type = self.buffer[2]
            payload_length = self.buffer[3]
            if msg_type != 0x81 or payload_length not in BASE_LENGTHS:
                del self.buffer[0]
                continue
            frame_length = payload_length + 5
            if len(self.buffer) < frame_length:
                return
            frame = self.buffer[:frame_length]
            expected = sum(frame[2:-1]) & 0xff
            if frame[-1] != expected or not self._valid_payload(frame[4:-1]):
                del self.buffer[0]
                continue
            self.valid_frames += 1
            del self.buffer[:frame_length]

    @staticmethod
    def _valid_payload(payload):
        values = struct.unpack_from('<ffff', payload)
        battery = struct.unpack_from('<f', payload, 24)[0]
        return all(math.isfinite(value) for value in (*values, battery))


class LidarParser:

    def __init__(self):
        self.buffer = bytearray()
        self.valid_frames = 0

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xaa' else b''
                return
            del self.buffer[:start]
            if len(self.buffer) < 10:
                return
            sample_count = self.buffer[3]
            if sample_count == 0:
                del self.buffer[0]
                continue
            packet_length = 10 + sample_count * 2
            if len(self.buffer) < packet_length:
                return
            packet = self.buffer[:packet_length]
            first_angle = _le16(packet, 4)
            last_angle = _le16(packet, 6)
            expected = _le16(packet, 8)
            if ((first_angle & 1) == 0 or (last_angle & 1) == 0 or
                    expected != lidar_checksum(packet, sample_count)):
                del self.buffer[0]
                continue
            self.valid_frames += 1
            del self.buffer[:packet_length]


def _le16(data, offset):
    return data[offset] | data[offset + 1] << 8


def lidar_checksum(packet, sample_count):
    checksum = _le16(packet, 0)
    checksum ^= packet[2] | packet[3] << 8
    checksum ^= _le16(packet, 4)
    checksum ^= _le16(packet, 6)
    for index in range(sample_count):
        checksum ^= _le16(packet, 10 + index * 2)
    return checksum


def configure_serial(fd, baud_rate=115200):
    attributes = termios.tcgetattr(fd)
    configured = list(attributes)
    configured[6] = list(attributes[6])
    configured[0] = 0
    configured[1] = 0
    configured[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    if hasattr(termios, 'CRTSCTS'):
        configured[2] &= ~termios.CRTSCTS
    configured[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    configured[3] = 0
    configured[4] = getattr(termios, f'B{baud_rate}')
    configured[5] = getattr(termios, f'B{baud_rate}')
    configured[6][termios.VMIN] = 0
    configured[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, configured)
    termios.tcflush(fd, termios.TCIFLUSH)
    return attributes


def probe_device(device, timeout, baud_rate=115200):
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        original = configure_serial(fd, baud_rate)
        base_parser = BaseParser()
        lidar_parser = LidarParser()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([fd], [], [], min(0.1, remaining))
            if not readable:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not data:
                continue
            base_parser.feed(data)
            lidar_parser.feed(data)
            base_valid = base_parser.valid_frames >= MIN_VALID_FRAMES
            lidar_valid = lidar_parser.valid_frames >= MIN_VALID_FRAMES
            if base_valid != lidar_valid:
                kind = 'base' if base_valid else 'lidar'
                count = max(base_parser.valid_frames, lidar_parser.valid_frames)
                return kind, count
            if base_valid and lidar_valid:
                return 'ambiguous', 0
        return 'unknown', 0
    finally:
        if original is not None:
            termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=3.0)
    parser.add_argument('--baud', type=int, choices=(115200, 230400, 460800, 921600),
                        default=115200)
    parser.add_argument('devices', nargs='+')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    for device in args.devices:
        try:
            kind, count = probe_device(device, args.timeout, args.baud)
            print(f'{device}\t{kind}\t{count}')
        except (OSError, termios.error) as error:
            detail = str(error).replace('\t', ' ').replace('\n', ' ')
            print(f'{device}\terror\t{detail}')


if __name__ == '__main__':
    main()
