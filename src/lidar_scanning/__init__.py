"""LiDAR scan-to-EnergyPlus pipeline.

Subpackages follow the project phases:

    rig/           P1  Raspberry Pi scanning station (runs on the Pi)
    calibration/   P2  acceptance tests and rig calibration
    registration/  P3  multi-station alignment, E57 export
    geometry/      P4  point cloud -> walls, rooms, openings
    schema/        P4  building geometry JSON schema
    energyplus/    P5  schema -> Honeybee -> IDF -> EnergyPlus run
    validation/    P6  pipeline vs tape survey, energy-impact comparison
    thermal/       extension (Section 9), deferred until P5 is done
    fileio/        shared readers/writers (PCD, PLY, E57, JSON)
"""

__version__ = "0.0.1"
