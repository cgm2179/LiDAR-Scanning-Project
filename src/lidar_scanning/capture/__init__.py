"""P3 capture: turn what the rig records into one 3D point cloud per station.

Inputs are exactly what PiDAR writes on the Pi:
  - LiDAR: one file per stepper position, plane_<angle>.npy (or .csv), rows
    [x_mm, y_mm, luminance] in the LiDAR's own vertical spinning plane.
  - Camera: the Hugin-stitched equirectangular panorama (2:1) from the fisheye
    photos, used to colour the points.

    rig.py        rig calibration (P2 results: offsets, range correction, pano yaw)
    station.py    read planes + panorama, assemble one station's 3D cloud
    manifest.py   capture manifest: stations, rooms, neighbours, Disto readings
    checks.py     per-station checks against field measurements
    simulate.py   synthetic PiDAR captures for testing before the rig exists
"""
