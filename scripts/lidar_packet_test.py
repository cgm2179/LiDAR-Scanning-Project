"""Measure LiDAR packet loss on the Pi: the test that decides whether a C++ reader is needed.

Run on the Pi with the LiDAR spinning and the rig still:

    python scripts/lidar_packet_test.py --port /dev/ttyS0 --seconds 20
    python scripts/lidar_packet_test.py --port /dev/ttyS0 --seconds 20 --bytewise   # PiDAR's way

Default mode reads in 4 KB chunks (this repo's reader). --bytewise reads one
byte per call like PiDAR's driver. Both decode with the same parser, so the
difference is only how fast Python drains the serial port.
Expect ~21,600 points/s from an STL-27L (1,800 packets/s).
"""

import argparse
import time

from lidar_scanning.rig.stl27l import LD06_BAUD, STL27L_BAUD, PacketParser, Packets, estimate_dropped

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyS0")
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--ld06", action="store_true", help="LD06 at 230400 baud instead of STL-27L")
    parser.add_argument("--bytewise", action="store_true", help="read one byte per call, like PiDAR")
    args = parser.parse_args()

    import serial

    baud = LD06_BAUD if args.ld06 else STL27L_BAUD
    port = serial.Serial(args.port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.2)
    port.reset_input_buffer()
    decoder, got, received = PacketParser(), [], 0
    pending = bytearray()
    start = time.time()
    while time.time() - start < args.seconds:
        if args.bytewise:
            # one byte per call like PiDAR; decode in blocks so decoding cost doesn't skew the read test
            pending += port.read(1)
            if len(pending) < 4096:
                continue
            chunk, pending = bytes(pending), bytearray()
        else:
            chunk = port.read(4096)
        received += len(chunk)
        got.append(decoder.feed(chunk))
    elapsed = time.time() - start
    received += len(pending)
    got.append(decoder.feed(bytes(pending)))
    port.close()

    packets = Packets.concat(got)
    dropped = estimate_dropped(packets)
    stats = decoder.stats
    expected_bytes = baud / 10 * elapsed            # 8N1: 10 bits per byte
    print(f"mode:              {'byte-by-byte (PiDAR)' if args.bytewise else 'chunked'} at {baud} baud, {elapsed:.1f} s")
    print(f"bytes received:    {received:,} ({received / expected_bytes:.1%} of line rate)")
    print(f"packets decoded:   {stats.packets:,} ({stats.packets / elapsed:,.0f}/s)")
    print(f"points:            {stats.packets * 12 / elapsed:,.0f}/s")
    print(f"CRC errors:        {stats.crc_errors:,}")
    print(f"skipped bytes:     {stats.skipped_bytes:,}")
    print(f"dropped (angle gaps): {dropped:,} ({dropped / max(1, dropped + stats.packets):.2%} of packets)")
    if len(packets):
        print(f"LiDAR speed:       {packets.speed_dps.mean() / 360:.2f} rev/s")
