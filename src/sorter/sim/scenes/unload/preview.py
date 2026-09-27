"""Preview of the unload scene on the real rover's geometry: renders it to PNGs, or opens the
MuJoCo viewer.

    uv run python -m sorter.sim.scenes.unload.preview [--out data/preview] [--view] [--seed N]

The geometry is `sorter.sim.scenes.unload.rover.REAL_ROVER` (estimates from photos, to be
measured); the arm stands at `home`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from sorter.sim.physics.model import ARM_JOINTS, build
from sorter.sim.scenes.unload.rover import rover_config

SOCKS = {"light": 1, "dark": 1, "colored": 1}
SOCK_MM = (200, 90)  # an adult sock laid flat
SETTLE_S = 2.0
# free cameras: (name, lookat mm, distance mm, azimuth deg, elevation deg)
VIEWS = (
    ("overview", (40, 0, -80), 1300, 215, -30),
    ("top", (40, 30, -100), 1150, 180, -89),
    ("cargo_box", (-130, 200, 0), 480, 250, -55),
    ("from_behind_left", (-40, 60, -60), 1000, 330, -28),
)


def scene(seed: int = 0, **unload) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The settled scene; `unload`: overrides of `sim.unload` (e.g. station_mm)."""
    cfg = rover_config(
        {"sim": {"seed": seed, "unload": {"cargo": SOCKS, "sock_mm": SOCK_MM, **unload}}}
    )
    model = mujoco.MjModel.from_xml_string(build(cfg.sim).xml)
    data = mujoco.MjData(model)
    home = np.array(cfg.poses["home"])
    data.qpos[[model.jnt_qposadr[model.joint(j).id] for j in ARM_JOINTS]] = home
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
