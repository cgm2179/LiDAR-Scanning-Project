"""Scanning station code that runs on the Raspberry Pi.

    stl27l.py   STL-27L/LD06 packet parser + chunked serial reader (CRC, loss counting)
    scan.py     stop-and-scan sweep: one LiDAR revolution per stepper angle -> plane files

Still to come: stepper control, fisheye camera capture, one-button scan cycle.
Starting point is PiDAR (https://github.com/VoxoTech/PiDAR); see third_party/.
"""
