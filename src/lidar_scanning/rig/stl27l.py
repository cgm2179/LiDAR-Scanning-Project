"""LDROBOT STL-27L (and LD06/LD19) packet parser and serial reader.

Packet, 47 bytes, little-endian:
    0      0x54 header
    1      0x2C ver/len (12 points per packet)
    2-3    speed, deg/s
    4-5    start angle, 0.01 deg
    6-41   12 x (distance mm uint16, intensity uint8)
    42-43  end angle, 0.01 deg
    44-45  timestamp, ms (wraps at 30000)
    46     CRC8 (poly 0x4D) over bytes 0-45

Differences from PiDAR's reader: reads the port in chunks instead of one
byte per call, keeps a buffer so packets split across reads aren't lost,
checks every CRC, and counts CRC errors, skipped bytes and dropped packets
(from gaps in the angle sequence) so packet loss can be measured.

parse_packets() is a pure bytes -> arrays function. If the Pi turns out too
slow, a C++ version with the same signature can replace it.
"""

from dataclasses import dataclass, field

import numpy as np

HEADER = 0x54
VER_LEN = 0x2C
PACKET_LEN = 47
POINTS_PER_PACKET = 12
STL27L_BAUD = 921600
LD06_BAUD = 230400


def _crc_table(poly=0x4D):
    table = np.zeros(256, dtype=np.uint8)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table[i] = crc
    return table


CRC_TABLE = _crc_table()


def crc8(rows):
    """CRC8 of each row of a (N x L) uint8 array, vectorised across rows."""
    rows = np.atleast_2d(rows)
    crc = np.zeros(len(rows), dtype=np.uint8)
    for k in range(rows.shape[1]):
        crc = CRC_TABLE[crc ^ rows[:, k]]
    return crc


@dataclass
class Packets:
    speed_dps: np.ndarray       # N
    start_deg: np.ndarray       # N
    end_deg: np.ndarray         # N
    timestamp_ms: np.ndarray    # N
    angle_deg: np.ndarray       # N x 12, interpolated start -> end
    distance_mm: np.ndarray     # N x 12, 0 = no return
    intensity: np.ndarray       # N x 12

    def __len__(self):
        return len(self.speed_dps)

    def take(self, index):
        return Packets(*(getattr(self, f)[index] for f in Packets.__dataclass_fields__))

    @staticmethod
    def concat(items):
        items = [p for p in items if len(p)]
        if not items:
            return empty_packets()
        return Packets(*(np.concatenate([getattr(p, f) for p in items]) for f in Packets.__dataclass_fields__))


def empty_packets():
    z1, z2 = np.zeros(0), np.zeros((0, POINTS_PER_PACKET))
    return Packets(z1, z1, z1, z1, z2, z2, z2)


@dataclass
class ParserStats:
    packets: int = 0
    crc_errors: int = 0
    skipped_bytes: int = 0


@dataclass
class PacketParser:
    """Feed raw serial bytes in any chunk sizes; get decoded packets back."""

    stats: ParserStats = field(default_factory=ParserStats)
    _buffer: bytes = b""

    def feed(self, data):
        buffer = self._buffer + bytes(data)
        packets, consumed, stats = parse_packets(buffer)
        self._buffer = buffer[consumed:]
        self.stats.packets += stats.packets
        self.stats.crc_errors += stats.crc_errors
        self.stats.skipped_bytes += stats.skipped_bytes
        return packets

    def reset(self):
        self._buffer = b""


def parse_packets(buffer):
    """Decode every valid packet in `buffer`.

    Returns (Packets, consumed, ParserStats). Bytes from `consumed` on may hold
    the start of a packet that hasn't fully arrived; pass them back in with the
    next chunk.
    """
    raw = np.frombuffer(bytes(buffer), dtype=np.uint8)
    stats = ParserStats()
    n = len(raw)
    if n < PACKET_LEN:
        return empty_packets(), 0, stats

    starts = np.flatnonzero((raw[:-1] == HEADER) & (raw[1:] == VER_LEN))
    starts = starts[starts + PACKET_LEN <= n]
    windows = raw[starts[:, None] + np.arange(PACKET_LEN)] if len(starts) else np.zeros((0, PACKET_LEN), np.uint8)
    crc_ok = crc8(windows[:, :-1]) == windows[:, -1] if len(starts) else np.zeros(0, bool)

    accepted, position = [], 0
    for k, start in enumerate(starts):
        if start < position:
            continue                      # header pattern inside a packet already taken
        if not crc_ok[k]:
            stats.crc_errors += 1
            continue
        stats.skipped_bytes += int(start - position)
        accepted.append(k)
        position = int(start) + PACKET_LEN

    # keep anything that could still be the start of an incomplete packet
    tail_start = max(position, n - (PACKET_LEN - 1))
    stats.skipped_bytes += max(0, tail_start - position)
    stats.packets = len(accepted)
    return decode(windows[accepted]), tail_start, stats


def decode(packets):
    """Decode an (N x 47) uint8 array of CRC-checked packets."""
    p = packets.astype(np.uint32)
    if len(p) == 0:
        return empty_packets()

    def u16(col):
        return p[:, col] | (p[:, col + 1] << 8)

    start = u16(4) / 100.0
    end = u16(42) / 100.0
    point_cols = 6 + 3 * np.arange(POINTS_PER_PACKET)
    distance = p[:, point_cols] | (p[:, point_cols + 1] << 8)
    intensity = p[:, point_cols + 2]
    step = np.mod(end - start, 360.0) / (POINTS_PER_PACKET - 1)
    angle = np.mod(start[:, None] + step[:, None] * np.arange(POINTS_PER_PACKET), 360.0)
    return Packets(u16(2).astype(float), start, end, u16(44).astype(float),
                   angle, distance.astype(float), intensity.astype(float))


def estimate_dropped(packets):
    """Packets missing from a continuous stream, judged from gaps in the angle sequence."""
    if len(packets) < 2:
        return 0
    span = np.mod(packets.end_deg - packets.start_deg, 360.0)
    step = span / (POINTS_PER_PACKET - 1)
    gap = np.mod(packets.start_deg[1:] - packets.end_deg[:-1], 360.0) - step[:-1]
    expected = span[:-1] + step[:-1]
    return int(np.sum(np.maximum(0, np.round(gap / np.maximum(expected, 1e-6)))))


def to_pidar_points(packets, offset_rad=np.pi / 2):
    """Rows [x_mm, y_mm, luminance] in PiDAR's convention (lidar_driver.polar2cartesian,
    OFFSET = 90°), so saved planes assemble exactly like PiDAR's."""
    a = np.radians(packets.angle_deg.ravel()) + offset_rad
    d = packets.distance_mm.ravel()
    return np.column_stack([d * -np.cos(a), d * np.sin(a), packets.intensity.ravel()]).astype(np.float32)


def encode_packet(start_deg, end_deg, distance_mm, intensity, speed_dps=3600, timestamp_ms=0):
    """Build one valid packet (for tests and a fake sensor)."""
    body = bytearray([HEADER, VER_LEN])
    body += int(speed_dps).to_bytes(2, "little")
    body += int(round(start_deg * 100) % 36000).to_bytes(2, "little")
    for d, i in zip(distance_mm, intensity):
        body += int(d).to_bytes(2, "little") + bytes([int(i)])
    body += int(round(end_deg * 100) % 36000).to_bytes(2, "little")
    body += int(timestamp_ms % 30000).to_bytes(2, "little")
    body.append(int(crc8(np.frombuffer(bytes(body), np.uint8))[0]))
    return bytes(body)


class STL27LReader:
    """Chunked serial reader. `port` is a device path, or any object with
    read(n) and reset_input_buffer() (a pyserial Serial, or a fake for tests)."""

    def __init__(self, port="/dev/ttyS0", baudrate=STL27L_BAUD, chunk_size=4096, timeout_s=0.2, max_empty_reads=25):
        if isinstance(port, str):
            import serial  # pyserial puts the port in raw mode (no echo, no XON/XOFF, no CR/LF changes)

            port = serial.Serial(port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=timeout_s)
        self.port = port
        self.chunk_size = chunk_size
        self.max_empty_reads = max_empty_reads
        self.parser = PacketParser()
        self.dropped = 0
        self._last = None   # last packet read, to spot gaps across reads

    def flush(self):
        """Discard everything received so far (call after moving the stepper)."""
        self.port.reset_input_buffer()
        self.parser.reset()
        self._last = None

    def read_packets(self, count):
        got, n, empty = [], 0, 0
        while n < count:
            chunk = self.port.read(self.chunk_size)
            if not chunk:
                empty += 1
                if empty >= self.max_empty_reads:
                    raise TimeoutError("no data from the LiDAR: check wiring, port and baud rate")
                continue
            empty = 0
            packets = self.parser.feed(chunk)
            got.append(packets)
            n += len(packets)
        result = Packets.concat(got)
        sequence = Packets.concat([self._last, result]) if self._last is not None else result
        self.dropped += estimate_dropped(sequence)
        self._last = result.take(slice(-1, None))
        return result

    def read_revolution(self, batch=20):
        """Read exactly one turn of the LiDAR: the packets whose start angle lies
        within 360° of the first one (summing packet spans would undercount,
        since angles are only reported to 0.01°)."""
        got = []
        while True:
            got.append(self.read_packets(batch))
            packets = Packets.concat(got)
            advance = np.concatenate([[0.0], np.cumsum(np.mod(np.diff(packets.start_deg), 360.0))])
            if advance[-1] >= 360.0 - 1e-6:
                return packets.take(advance < 360.0 - 1e-6)

    def close(self):
        self.port.close()
