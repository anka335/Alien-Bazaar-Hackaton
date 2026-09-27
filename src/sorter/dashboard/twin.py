"""The 3D view's data (`TwinSource`): the scene's static parts, the arm from FK, the sim items.

The static parts (floor, rover, cargo box, bins, whatever a sim scene adds) are the world
body's geoms of the MuJoCo model: the simulator's, or on the rig one built from the same config.
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
    from sorter.sim.physics.world import PhysicsWorld

log = logging.getLogger(__name__)


def _flat(T: np.ndarray, scale: float = 1.0) -> list[float]:
    """Row-major 4x4, translation × `scale`."""
    T = np.array(T, dtype=float)
    T[:3, 3] *= scale
    return [round(float(v), 5) for v in T.flatten()]


def _cloth_n() -> int:
    from sorter.sim.physics.model import CLOTH_N

    return CLOTH_N


def static_parts(model) -> list[dict[str, Any]]:
    """The world body's box / cylinder / plane geoms of a MuJoCo model, for the 3D view."""
    import mujoco

    types = {
        int(mujoco.mjtGeom.mjGEOM_BOX): "box",
        int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
        int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    }
    out = []
    for i in range(model.ngeom):
        kind = types.get(int(model.geom_type[i]))
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if model.geom_bodyid[i] != 0 or kind is None or name.startswith("mark_"):
            continue
        mat = model.geom_matid[i]
        rgba = model.mat_rgba[mat] if mat >= 0 else model.geom_rgba[i]
        if name == "floor":
            rgba = np.array([0.69, 0.49, 0.30, 1.0])  # the texture's average
        out.append(
            {
                "name": name,
                "type": kind,
                "size_mm": [round(float(v) * 1000, 1) for v in model.geom_size[i]],
                "pos_mm": [round(float(v) * 1000, 1) for v in model.geom_pos[i]],
                "quat": [round(float(v), 5) for v in model.geom_quat[i]],  # w, x, y, z
                "color": "#" + "".join(f"{round(float(c) * 255):02x}" for c in rgba[:3]),
                "opacity": round(float(rgba[3]), 2),
            }
        )
    return out


class Twin:
    def __init__(
        self,
        arm: ArmController,
        calibration: Calibration,
        cfg: Config,
        world: PhysicsWorld | None = None,
    ):
        self.arm, self.calibration, self.cfg, self.world = arm, calibration, cfg, world
        self._parts: list[dict[str, Any]] | None = None

    def _static(self) -> list[dict[str, Any]]:
        if self._parts is None:
            if self.world is not None:
                model = self.world.model
            else:  # the rig: the same scene, built from the config
                import mujoco

                from sorter.sim.physics.model import build

                model = mujoco.MjModel.from_xml_string(build(self.cfg.sim).xml)
            self._parts = static_parts(model)
        return self._parts

    def layout(self) -> dict[str, Any]:
        sim = self.cfg.sim
        return {
            "simulated": self.world is not None,
            "parts": self._static(),
            "floor_z_mm": sim.layout.floor_z_mm,
            "zones": {
                z.value: {
                    "polygon": [list(p) for p in zc.workspace_mm],
                    "z_mm": zc.z_floor_mm,
                }
                for z, zc in self.cfg.zones.items()
            },
            "camera": {"width": sim.width, "height": sim.height, "focal_px": sim.focal_px},
            # every item is a cloth grid of cloth_n x cloth_n vertices (sent in `state`)
            "cloth_n": _cloth_n(),
        }

    def state(self) -> dict[str, Any]:
        if getattr(self.arm, "connected", True):
            q = np.asarray(self.arm.joints(), dtype=float)
            opening = float(getattr(self.arm, "gripper_opening", lambda: 0.0)())
        else:  # not started yet (`run` before Start): motors off, so the arm is at rest
            q, opening = np.asarray(self.arm.poses["rest"], dtype=float), 0.0
        tcp = kin.fk_tcp(q)[:3, 3]
        try:
            cam = _flat(self.calibration.cam_pose(kin.fk_link5(q)), 1e-3)
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
        if w is not None:  # the cloth itself
            looking = w.looking_at
            out["looking_at"] = looking.value if isinstance(looking, Zone) else None
            for it in w.items:
                v = w.vertices(it.id)
                loc, color = w.location(it.id)
                out["items"].append(
                    {
                        "id": it.id,
                        "color": "#" + "".join(f"{round(c * 255):02x}" for c in it.rgb),
                        "class": it.color.value,
                        "location": loc,
                        "in": color.value if color else None,  # the compartment / bin
                        "xyz": [round(float(c), 1) for c in v.mean(axis=0)],
                        "vertices": [round(float(c), 1) for c in v.ravel()],
                    }
                )
        return out
