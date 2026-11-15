#!/usr/bin/env python3
"""Windows COM-port simulator for the ESP32 differential-drive protocol."""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required. Install with: python -m pip install pyserial", file=sys.stderr)
    raise


HEAD = b"\xAA\x55"
MSG_CMD_VEL = 0x01
MSG_STATUS = 0x81
MAX_PAYLOAD = 64
STATUS_STRUCT = struct.Struct("<ffffiifhhB")
STATUS_V2_STRUCT = struct.Struct("<ffffiifhhBI")
STATUS_V3_STRUCT = struct.Struct("<ffffiifhhBIIIII")


def checksum(msg_type: int, payload: bytes) -> int:
    return (msg_type + len(payload) + sum(payload)) & 0xFF


def make_frame(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    return HEAD + bytes((msg_type, len(payload))) + payload + bytes((checksum(msg_type, payload),))


def make_cmd_vel(linear: float, angular: float) -> bytes:
    return make_frame(MSG_CMD_VEL, struct.pack("<ff", linear, angular))


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, bytes]] = []

        while True:
            start = self.buffer.find(HEAD)
            if start < 0:
                self.buffer.clear()
                return frames
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                return frames

            msg_type = self.buffer[2]
            payload_len = self.buffer[3]
            if payload_len > MAX_PAYLOAD:
                del self.buffer[0]
                continue

            frame_len = 2 + 1 + 1 + payload_len + 1
            if len(self.buffer) < frame_len:
                return frames

            payload = bytes(self.buffer[4 : 4 + payload_len])
            got = self.buffer[frame_len - 1]
            del self.buffer[:frame_len]

            if got == checksum(msg_type, payload):
                frames.append((msg_type, payload))


def format_status(payload: bytes) -> str:
    firmware = "old"
    diag_text = ""
    if len(payload) == STATUS_V3_STRUCT.size:
        (
            left_rpm,
            right_rpm,
            left_target,
            right_target,
            enc_l,
            enc_r,
            voltage,
            pwm_l,
            pwm_r,
            flags,
            firmware,
            rx_bytes,
            rx_valid,
            rx_bad_checksum,
            rx_bad_len,
        ) = STATUS_V3_STRUCT.unpack(payload)
        diag_text = f"RXB {rx_bytes:8d} OK {rx_valid:6d} BADCRC {rx_bad_checksum:4d} BADLEN {rx_bad_len:4d} "
    elif len(payload) == STATUS_V2_STRUCT.size:
        (
            left_rpm,
            right_rpm,
            left_target,
            right_target,
            enc_l,
            enc_r,
            voltage,
            pwm_l,
            pwm_r,
            flags,
            firmware,
        ) = STATUS_V2_STRUCT.unpack(payload)
    elif len(payload) == STATUS_STRUCT.size:
        left_rpm, right_rpm, left_target, right_target, enc_l, enc_r, voltage, pwm_l, pwm_r, flags = STATUS_STRUCT.unpack(payload)
    else:
        return f"bad status length={len(payload)}"
    flag_names = []
    if flags & 0x01:
        flag_names.append("HOST")
    if flags & 0x02:
        flag_names.append("TIMEOUT")
    if flags & 0x04:
        flag_names.append("LOW_BAT")
    flags_text = "|".join(flag_names) if flag_names else "0"

    return (
        f"L {left_rpm:7.2f}/{left_target:7.2f} rpm "
        f"R {right_rpm:7.2f}/{right_target:7.2f} rpm "
        f"ENC {enc_l:9d} {enc_r:9d} "
        f"PWM {pwm_l:4d} {pwm_r:4d} "
        f"BAT {voltage:5.2f}V "
        f"FW {firmware} "
        f"{diag_text}"
        f"FLAGS {flags_text}"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simulate a Raspberry Pi ROS2 node over a Windows COM port.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--linear", type=float, default=0.05, help="linear_x in m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="angular_z in rad/s")
    parser.add_argument("--rate", type=float, default=50.0, help="Command send rate in Hz")
    parser.add_argument("--timeout", type=float, default=0.0, help="Serial read timeout in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    period = 1.0 / args.rate if args.rate > 0 else 0.05
    parser = FrameParser()
    next_send = 0.0
    last_print = 0.0
    sent_count = 0

    print(f"Opening {args.port} at {args.baud} baud")
    print("Ctrl+C sends one zero cmd_vel frame before exit.")

    with serial.Serial(args.port, args.baud, timeout=args.timeout, write_timeout=0.1) as ser:
        cmd_frame = make_cmd_vel(args.linear, args.angular)
        try:
            while True:
                now = time.monotonic()
                if now >= next_send:
                    ser.write(cmd_frame)
                    sent_count += 1
                    next_send += period
                    if next_send < now - period:
                        next_send = now + period

                waiting = ser.in_waiting
                data = ser.read(waiting if waiting > 0 else 1)
                for msg_type, payload in parser.feed(data):
                    if msg_type == MSG_STATUS:
                        if now - last_print >= 0.1:
                            print(f"TX {sent_count:6d}  {format_status(payload)}")
                            last_print = now

                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nStopping motors...")
            stop_frame = make_cmd_vel(0.0, 0.0)
            for _ in range(5):
                ser.write(stop_frame)
                time.sleep(0.02)
            time.sleep(0.05)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
