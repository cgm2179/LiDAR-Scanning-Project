"""Import every library the project uses and print its version.

Run after setting up the environment:  python scripts/check_environment.py
Pass --pi on the Raspberry Pi to also check the rig libraries.
"""

import importlib
import importlib.metadata
import sys

PC = [
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("open3d", "open3d"),
    ("pye57", "pye57"),
    ("shapely", "shapely"),
    ("matplotlib", "matplotlib"),
    ("opencv-python", "cv2"),
    ("exifread", "exifread"),
    ("pandas", "pandas"),
    ("jsonschema", "jsonschema"),
    ("pyyaml", "yaml"),
    ("honeybee-energy", "honeybee_energy"),
    ("lidar-scanning", "lidar_scanning"),
]

PI = [
    ("pyserial", "serial"),
    ("rpi-hardware-pwm", "rpi_hardware_pwm"),
    ("RPi.GPIO", "RPi.GPIO"),
    ("gpiozero", "gpiozero"),
    ("smbus2", "smbus2"),
    ("picamera2", "picamera2"),
]


def check(packages):
    missing = []
    for label, module in packages:
        try:
            importlib.import_module(module)
            try:
                version = importlib.metadata.version(label)
            except importlib.metadata.PackageNotFoundError:
                version = ""  # e.g. picamera2 installed by apt
            print(f"  ok       {label:28s} {version}")
        except Exception as exc:
            print(f"  MISSING  {label:28s} {type(exc).__name__}: {exc}")
            missing.append(label)
    return missing


if __name__ == "__main__":
    print(f"Python {sys.version.split()[0]}")
    missing = check(PC)
    if "--pi" in sys.argv:
        print("Raspberry Pi libraries:")
        missing += check(PI)
    sys.exit(1 if missing else 0)
