import importlib.util
import os
import pty
import struct
import threading
import time


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    'scripts',
    'probe_serial_devices.py',
)
SPEC = importlib.util.spec_from_file_location('probe_serial_devices', SCRIPT)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def base_frame(payload_length=53):
    payload = bytearray(struct.pack(
        '<ffffiifhhB',
        1.0, -2.0, 3.0, -4.0, 100, -200, 11.8, 10, -11, 1,
    ))
    if payload_length >= 37:
        payload.extend(struct.pack('<I', 2026061505))
    if payload_length == 53:
        payload.extend(struct.pack('<IIII', 1000, 100, 0, 0))
    frame = bytearray(b'\xaa\x55')
    frame.extend((0x81, len(payload)))
    frame.extend(payload)
    frame.append(sum(frame[2:]) & 0xff)
    return bytes(frame)


def lidar_packet(sample_count=4):
    packet = bytearray(b'\xaa\x55')
    packet.extend((0x01, sample_count))
    packet.extend(struct.pack('<HHH', 0x1001, 0x1201, 0))
    packet.extend(struct.pack('<' + 'H' * sample_count, *range(400, 400 + sample_count)))
    checksum = PROBE.lidar_checksum(packet, sample_count)
    struct.pack_into('<H', packet, 8, checksum)
    return bytes(packet)


def test_base_parser_accepts_supported_versions_and_fragmented_data():
    parser = PROBE.BaseParser()
    stream = b'noise' + base_frame(33) + base_frame(37) + base_frame(53)
    for offset in range(0, len(stream), 7):
        parser.feed(stream[offset:offset + 7])
    assert parser.valid_frames == 3


def test_base_parser_rejects_bad_checksum():
    parser = PROBE.BaseParser()
    damaged = bytearray(base_frame())
    damaged[-1] ^= 0xff
    parser.feed(damaged + base_frame())
    assert parser.valid_frames == 1


def test_lidar_parser_accepts_fragmented_packets_and_rejects_bad_checksum():
    parser = PROBE.LidarParser()
    damaged = bytearray(lidar_packet())
    damaged[-1] ^= 0xff
    stream = b'noise' + damaged + lidar_packet(1) + lidar_packet(5)
    for offset in range(0, len(stream), 3):
        parser.feed(stream[offset:offset + 3])
    assert parser.valid_frames == 2


def run_serial_writer(master_fd, payload, stop_event):
    while not stop_event.is_set():
        os.write(master_fd, payload)
        time.sleep(0.02)


def probe_pseudo_terminal(payload):
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    stop_event = threading.Event()
    writer = threading.Thread(
        target=run_serial_writer,
        args=(master_fd, payload, stop_event),
        daemon=True,
    )
    writer.start()
    try:
        return PROBE.probe_device(slave_name, 1.0)
    finally:
        stop_event.set()
        writer.join(timeout=1.0)
        os.close(slave_fd)
        os.close(master_fd)


def test_probe_identifies_base_pseudo_terminal():
    kind, count = probe_pseudo_terminal(base_frame())
    assert kind == 'base'
    assert count >= 2


def test_probe_identifies_lidar_pseudo_terminal():
    kind, count = probe_pseudo_terminal(lidar_packet())
    assert kind == 'lidar'
    assert count >= 2
