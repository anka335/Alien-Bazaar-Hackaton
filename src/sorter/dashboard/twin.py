"""The 3D view's data (`TwinSource`): the table layout, the arm from FK, the sim items.

Units in the JSON: link poses and the camera pose are 4x4 row-major in **metres** (three.js
scene units); everything else is in mm, arm base frame.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from sorter.arm import kinematics as kin
from sorter.core.types import Zone

if TYPE_CHECKING:
    from sorter.core.config import Config
    from sorter.core.protocols import ArmController, Calibration
    from sorter.sim.world import SimWorld

log = logging.getLogger(__name__)


def _hex(bgr: tuple[int, int, int]) -> str:
    b, g, r = bgr
    return f"#{r:02x}{g:02x}{b:02x}"


def _flat(T: np.ndarray, scale: float = 1.0) -> list[float]:
    """Row-major 4x4, translation × `scale`."""
    T = np.array(T, dtype=float)
    T[:3, 3] *= scale
    return [round(float(v), 5) for v in T.flatten()]


class Twin:
    def __init__(
        self,
        arm: ArmController,
        calibration: Calibration,
        cfg: Config,
        world: SimWorld | None = None,
    ):
        self.arm, self.calibration, self.cfg, self.world = arm, calibration, cfg, world

    def layout(self) -> dict[str, Any]:
        sim = self.cfg.sim
        return {
            "simulated": self.world is not None,
            "table": sim.layout.model_dump(mode="json"),
            "zones": {
                z.value: [list(p) for p in zc.workspace_mm] for z, zc in self.cfg.zones.items()
            },
            "camera": {"width": sim.width, "height": sim.height, "focal_px": sim.focal_px},
            "item_radius_mm": sim.item_radius_mm,
        }

    def state(self) -> dict[str, Any]:
        q = np.asarray(self.arm.joints(), dtype=float)
        opening = float(getattr(self.arm, "gripper_opening", lambda: 0.0)())
        tcp = kin.fk_tcp(q)[:3, 3]
        try:
            cam = _flat(self.calibration.cam_pose(self.arm.ee_pose()), 1e-3)
        except Exception:  # no hand-eye result yet: no camera in the view
            cam = None
        out: dict[str, Any] = {
            "joints": [round(float(v), 5) for v in q],
            "links": {name: _flat(T) for name, T in kin.link_frames(q, opening).items()},
            "gripper": round(opening, 3),
            "tcp": [round(float(v), 1) for v in tcp],
            "camera": cam,
            "looking_at": None,
            "items": [],
        }
        w = self.world
        if w is not None:
            with w.lock:
                looking = w.looking_at
                out["looking_at"] = looking.value if isinstance(looking, Zone) else None
                for it in w.items:
                    held = it.location == "gripper"
                    x, y, z = (*tcp[:2], tcp[2]) if held else (it.x, it.y, w.top_z(it))
                    out["items"].append(
                        {
                            "id": it.id,
                            "color": _hex(it.bgr),
                            "class": it.color.value,
                            "location": it.location,
                            "xyz": [round(float(x), 1), round(float(y), 1), round(float(z), 1)],
                        }
                    )
        return out
