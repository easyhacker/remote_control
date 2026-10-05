"""Joint limits from a URDF (no ROS needed)."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence

from .trajectory import Joint


def load_urdf_joints(path: str, names: Optional[Sequence[str]] = None,
                     default_max_velocity: Optional[float] = None) -> List[Joint]:
    """Revolute / prismatic / continuous joints from a URDF, with <limit> lower/upper/velocity."""
    root = ET.parse(path).getroot()
    found: Dict[str, Joint] = {}
    for j in root.iter("joint"):
        jtype = j.get("type")
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        lim = j.find("limit")
        lower = upper = vel = None
        if lim is not None:
            if jtype != "continuous":
                lower = float(lim.get("lower")) if lim.get("lower") is not None else None
                upper = float(lim.get("upper")) if lim.get("upper") is not None else None
            if lim.get("velocity") is not None and float(lim.get("velocity")) > 0:
                vel = float(lim.get("velocity"))
        found[j.get("name")] = Joint(j.get("name"), "prismatic" if jtype == "prismatic" else "revolute",
                                     lower, upper, vel if vel is not None else default_max_velocity)
    if names is None:
        return list(found.values())
    missing = [n for n in names if n not in found]
    if missing:
        raise ValueError(f"joints not in URDF {path}: {', '.join(missing)}")
    return [found[n] for n in names]
