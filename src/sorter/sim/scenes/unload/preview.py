"""Preview of the unload scene on the real rover's geometry: renders it to PNGs, or opens the
MuJoCo viewer.

    uv run python -m sorter.sim.scenes.unload.preview [--out data/preview] [--view] [--seed N]

`REAL_ROVER` overrides the committed `sim.layout` with the rover as it is (estimates from photos,
to be measured): one undivided cargo box 150 × 150 × 60 mm to the arm's left and a bit behind
it, three boxes of the same size on the floor in front of the rover. It is not the committed
layout yet: the cargo box is shared with stage A, so the change needs their agreement, and the
rig is not recomputed for it (the arm only shows, it does not move).
"""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from sorter.core.config import load_config
from sorter.sim.physics.model import ARM_JOINTS, box, build

REAL_ROVER = {
    "floor_z_mm": -160,  # deck top above the floor (wheels ~115 mm)
    "body": {"center_mm": [-110, 0], "size_mm": [380, 300]},  # chassis + wheels
    "deck": {"center_mm": [-110, 0], "size_mm": [340, 260]},
    "cargo": {  # cardboard, 150 × 150 × 60 outside, on a bracket off the deck's left side
        "center_mm": [-130, 200],
        "size_mm": [140, 140],
        "floor_z_mm": 4,
        "wall_mm": 56,
        "wall_t_mm": 5,
        "compartments": [],
    },
    "laundry": {  # the same boxes, in a row across the front of the rover
        "centers_mm": {"light": [270, 170], "dark": [270, 0], "colored": [270, -170]},
        "size_mm": 150,
        "height_mm": 60,
        "wall_t_mm": 5,
    },
}
SOCKS = {"light": 1, "dark": 1, "colored": 1}
SOCK_MM = (200, 90)  # an adult sock laid flat
ELECTRONICS_MM = ((-280, -125, 0), (-90, 20, 70))  # power strip and adapters behind the arm
BRACKET_MM = ((-210, 120, -24), (-50, 280, -20))  # holds the cargo box beside the deck
CARDBOARD_RGBA = "0.66 0.5 0.34 1"
SETTLE_S = 2.0
# free cameras: (name, lookat mm, distance mm, azimuth deg, elevation deg)
VIEWS = (
    ("overview", (40, 0, -80), 1300, 215, -30),
    ("top", (40, 30, -100), 1150, 180, -89),
    ("cargo_box", (-130, 200, 0), 480, 250, -55),
    ("from_behind_left", (-40, 60, -60), 1000, 330, -28),
)


def scene(seed: int = 0) -> tuple[mujoco.MjModel, mujoco.MjData]:
    cfg = load_config(
        overrides={
            "sim": {
                "seed": seed,
                "scenes": ["unload"],
                "layout": REAL_ROVER,
                "unload": {"cargo": SOCKS, "sock_mm": SOCK_MM},
            }
        }
    )
    root = ET.fromstring(build(cfg.sim).xml)
    world = root.find("worldbody")
    for g in world.iter("geom"):  # the base draws the cargo box grey; the real one is cardboard
        if (g.get("name") or "").startswith("cargo_"):
            g.set("rgba", CARDBOARD_RGBA)
    lo, hi = (np.array(p) / 1000 for p in ELECTRONICS_MM)
    box(world, "electronics", lo, hi, "0.13 0.13 0.14 1")
    lo, hi = (np.array(p) / 1000 for p in BRACKET_MM)
    box(world, "cargo_bracket", lo, hi, "0.55 0.57 0.6 1")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    home = np.array(cfg.poses["home"])
    qpos = [model.jnt_qposadr[model.joint(j).id] for j in ARM_JOINTS]
    data.qpos[qpos] = home
    data.ctrl[[model.actuator(j).id for j in ARM_JOINTS]] = home
    for _ in range(round(SETTLE_S / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return model, data


def render(model: mujoco.MjModel, data: mujoco.MjData, out: Path) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    with mujoco.Renderer(model, 960, 1280) as r:
        for name, lookat, dist, az, el in VIEWS:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = np.array(lookat) / 1000
            cam.distance, cam.azimuth, cam.elevation = dist / 1000, az, el
            r.update_scene(data, cam)
            path = out / f"{name}.png"
            import cv2

            cv2.imwrite(str(path), cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
            paths.append(path)
    return paths


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, default=Path("data/preview"))
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--view", action="store_true", help="open the MuJoCo viewer instead")
    a = p.parse_args()
    model, data = scene(a.seed)
    if a.view:
        import mujoco.viewer

        mujoco.viewer.launch(model, data)
        return
    for path in render(model, data, a.out):
        print(path)


if __name__ == "__main__":
    main()
