"""Capture manifest: which stations were scanned, where, and what was measured by hand.

One YAML file per capture (template: phases/phase3_capture/manifest_template.yaml).
Neighbours define which station pairs get registered; approximate poses from
the field sketch are optional and give ICP a starting point.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass
class StationInfo:
    id: str
    folder: Path
    room: str | None = None
    storey: str | int | None = None
    exterior: bool = False
    neighbours: list = field(default_factory=list)
    approx_pose: dict | None = None
    disto: dict = field(default_factory=dict)
    notes: str = ""


@dataclass
class Capture:
    id: str
    date: str | None
    reference: str
    stations: list

    def station(self, station_id):
        return next(s for s in self.stations if s.id == station_id)

    def edges(self):
        """Unique station pairs from the neighbour lists, in manifest order."""
        seen, pairs = set(), []
        for s in self.stations:
            for n in s.neighbours:
                key = frozenset((s.id, n))
                if key not in seen:
                    seen.add(key)
                    pairs.append((s.id, n))
        return pairs

    def approx_poses(self):
        return {s.id: approx_pose_matrix(s.approx_pose) for s in self.stations if s.approx_pose}


def approx_pose_matrix(pose):
    """4x4 station -> building pose from {x_m, y_m, z_m, heading_deg} (heading of stepper angle 0)."""
    t = np.radians(pose.get("heading_deg", 0.0))
    T = np.eye(4)
    T[:2, :2] = [[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]
    T[:3, 3] = [pose.get("x_m", 0.0), pose.get("y_m", 0.0), pose.get("z_m", 0.0)]
    return T


def load_manifest(path):
    """Read and check a capture manifest. Station folders resolve relative to the manifest's `root`
    (default: the manifest's own folder)."""
    path = Path(path)
    data = yaml.safe_load(path.read_text())
    info = data.get("capture") or {}
    root = path.parent / (info.get("root") or ".")

    stations = []
    for s in data.get("stations") or []:
        approx = s.get("approx_pose")
        if approx and any(v is None for v in approx.values()):
            approx = None
        stations.append(StationInfo(
            id=str(s["id"]),
            folder=(root / (s.get("folder") or str(s["id"]))).resolve(),
            room=s.get("room"),
            storey=s.get("storey"),
            exterior=bool(s.get("exterior", False)),
            neighbours=[str(n) for n in s.get("neighbours") or []],
            approx_pose=approx,
            disto={k: v for k, v in (s.get("disto") or {}).items() if v is not None},
            notes=s.get("notes") or "",
        ))
    if not stations:
        raise ValueError(f"{path} lists no stations")

    ids = [s.id for s in stations]
    duplicates = {i for i in ids if ids.count(i) > 1}
    if duplicates:
        raise ValueError(f"duplicate station ids: {sorted(duplicates)}")
    for s in stations:
        unknown = [n for n in s.neighbours if n not in ids]
        if unknown:
            raise ValueError(f"station {s.id} lists unknown neighbours {unknown}")
    reference = str(info.get("reference_station") or ids[0])
    if reference not in ids:
        raise ValueError(f"reference_station {reference} is not a station")

    return Capture(str(info.get("id") or path.stem), info.get("date"), reference, stations)
