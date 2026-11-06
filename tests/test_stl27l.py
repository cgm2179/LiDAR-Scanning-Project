"""STL-27L reader: packet decoding, CRC, chunking, loss counting, and a fake sensor end to end."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from lidar_scanning.capture.manifest import approx_pose_matrix
from lidar_scanning.capture.rig import RigCalibration
from lidar_scanning.capture.simulate import Box, Scene, raycast
from lidar_scanning.capture.station import load_station, stepper_rotation
from lidar_scanning.rig.scan import capture_planes
from lidar_scanning.rig.stl27l import (CRC_TABLE, PacketParser, STL27LReader, encode_packet, estimate_dropped,
                                       parse_packets, to_pidar_points)

ROOT = Path(__file__).resolve().parents[1]
# A real LD06 packet, from PiDAR's lib/crc_utils.py
REAL_PACKET = (b"T,\x10\x0eS+\x04\x01\xec\x13\x01\xea\x1a\x01\xea\x1e\x01\xea&\x01\xe91\x01\xe8)\x01\xeb,"
               b"\x01\xe9-\x01\xeb4\x01\xe88\x01\xe5N\x01\xd9\xe4.W\x0b\xd3")
STEP = 360 / 2160   # STL-27L at 10 Hz: 21,600 points/s


def stream(n, start=0.0, distance=lambda a: np.full(12, 3000)):
    packets, angle = [], start
    for k in range(n):
        angles = angle + STEP * np.arange(12)
        packets.append(encode_packet(angle % 360, (angle + 11 * STEP) % 360, distance(angles), np.full(12, 200),
                                     timestamp_ms=k))
        angle += 12 * STEP
    return packets


def test_real_packet_decodes():
    packets, consumed, stats = parse_packets(REAL_PACKET)
    assert len(REAL_PACKET) == 47 and stats.packets == 1 and stats.crc_errors == 0
    assert packets.speed_dps[0] / 360 == pytest.approx(10.0)
    assert packets.start_deg[0] == pytest.approx(110.91)
    assert 0 < (packets.end_deg[0] - packets.start_deg[0]) < 10
    assert packets.distance_mm[0, 0] == 260 and packets.intensity[0, 0] == 236


def test_crc_table_matches_pidar():
    path = ROOT / "third_party" / "PiDAR" / "lib" / "crc_utils.py"
    if not path.exists():
        pytest.skip("PiDAR not cloned into third_party/")
    spec = importlib.util.spec_from_file_location("pidar_crc", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for packet in stream(20):
        assert module.check_CRC8(bytearray(packet))
    assert CRC_TABLE[1] == 0x4D


def test_packets_split_across_reads_are_kept():
    data = b"".join(stream(200))
    parser, rng, got, i = PacketParser(), np.random.default_rng(0), 0, 0
    while i < len(data):
        n = int(rng.integers(1, 300))
        got += len(parser.feed(data[i:i + n]))
        i += n
    assert got == 200 and parser.stats.crc_errors == 0 and parser.stats.skipped_bytes == 0


def test_corrupt_packets_rejected_and_counted_as_dropped():
    packets = stream(100)
    for k in (10, 50):
        broken = bytearray(packets[k])
        broken[20] ^= 0xFF
        packets[k] = bytes(broken)
    decoded, _, stats = parse_packets(b"".join(packets))
    assert stats.packets == 98 and stats.crc_errors == 2
    assert estimate_dropped(decoded) == 2


def test_garbage_between_packets_skipped():
    rng = np.random.default_rng(1)
    noise = bytes(rng.integers(0, 256, 500, dtype=np.uint8)) + b"\x54\x2c\x00\x01"
    packets = stream(30)
    data = noise + b"".join(packets[:15]) + noise + b"".join(packets[15:])
    decoded, _, stats = parse_packets(data)
    assert stats.packets == 30 and estimate_dropped(decoded) == 0
    assert stats.skipped_bytes >= 2 * 500


def test_angles_interpolate_across_zero():
    packet = encode_packet(359.5, 1.33, np.full(12, 1000), np.full(12, 100))
    decoded, _, _ = parse_packets(packet)
    angles = decoded.angle_deg[0]
    assert angles[0] == pytest.approx(359.5) and angles[-1] == pytest.approx(1.33)
    assert np.allclose(np.diff(np.unwrap(np.radians(angles))), np.radians((1.33 + 0.5) / 11))


def test_pidar_point_convention():
    decoded, _, _ = parse_packets(encode_packet(30.0, 31.83, np.arange(1000, 1012), np.full(12, 90)))
    pts = to_pidar_points(decoded)
    a = np.radians(decoded.angle_deg[0]) + np.pi / 2       # PiDAR lidar_driver.polar2cartesian
    assert np.allclose(pts[:, 0], decoded.distance_mm[0] * -np.cos(a), atol=1e-3)
    assert np.allclose(pts[:, 1], decoded.distance_mm[0] * np.sin(a), atol=1e-3)
    assert np.all(pts[:, 2] == 90)


class FakeSTL27L:
    """Serial stand-in: endless valid packets of a box room as seen from the current stepper angle."""

    def __init__(self, scene, pose, rig):
        self.scene, self.pose, self.rig = scene, pose, rig
        self.stepper, self.lidar_angle, self.pending = 0.0, 0.0, b""

    def _packet(self):
        angles = self.lidar_angle + STEP * np.arange(12)
        a = np.radians(angles) + np.pi / 2
        local = np.column_stack([-np.cos(a), np.zeros(12), np.sin(a)])
        R = self.pose[:3, :3] @ stepper_rotation(self.stepper, self.rig)
        origin = self.pose[:3, :3] @ (stepper_rotation(self.stepper, self.rig) @ np.asarray(self.rig.lidar_offset_m)) \
            + self.pose[:3, 3]
        t, _ = raycast(np.broadcast_to(origin, local.shape), local @ R.T, self.scene)
        packet = encode_packet(self.lidar_angle % 360, (self.lidar_angle + 11 * STEP) % 360,
                               np.round(t * 1000), np.full(12, 200))
        self.lidar_angle = (self.lidar_angle + 12 * STEP) % 360
        return packet

    def read(self, n):
        while len(self.pending) < n:
            self.pending += self._packet()
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

    def reset_input_buffer(self):
        self.pending = self.pending[len(self.pending) // 2:]   # partial packet left over, like a real port

    def close(self):
        pass


def test_read_revolution_is_one_turn():
    rig, room = RigCalibration(), Scene(Box((0, 0, 0), (5, 4, 2.5)))
    reader = STL27LReader(FakeSTL27L(room, approx_pose_matrix({"x_m": 2, "y_m": 2, "z_m": 1.2}), rig), chunk_size=1024)
    packets = reader.read_revolution()
    assert len(packets) == 180 and reader.dropped == 0


def test_no_data_times_out():
    class Silent:
        def read(self, n):
            return b""

    with pytest.raises(TimeoutError):
        STL27LReader(Silent(), max_empty_reads=3).read_packets(1)


def test_fake_sensor_through_reader_and_p3_assembly(tmp_path):
    rig = RigCalibration(lidar_offset_m=(0.0, -0.037, 0.0))
    room = Scene(Box((0.0, 0.0, 0.0), (5.0, 4.0, 2.5)))
    pose = approx_pose_matrix({"x_m": 1.6, "y_m": 1.3, "z_m": 1.2, "heading_deg": 20})
    fake = FakeSTL27L(room, pose, rig)
    reader = STL27LReader(fake, chunk_size=1024)
    log = capture_planes(reader, lambda a: setattr(fake, "stepper", a), np.arange(0, 180, 2.0), tmp_path,
                         settle_s=0)
    assert log["planes"] == 90 and log["crc_errors"] == 0 and log["dropped_estimate"] == 0
    assert (tmp_path / "capture_log.json").exists()

    scan = load_station(tmp_path, rig)
    world = scan.points @ pose[:3, :3].T + pose[:3, 3]
    lo, hi = np.asarray(room.room.lo), np.asarray(room.room.hi)
    gap = np.min(np.abs(np.concatenate([world - lo, world - hi], axis=1)), axis=1)
    assert np.percentile(gap, 99) < 0.005
