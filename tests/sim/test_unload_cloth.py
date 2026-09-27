"""The unload scene's socks are limp: pinched, they hang far enough below the gripper for the
camera to see them from `show_held` (it sees nothing closer than ~55 mm below the fingers)."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from sorter.core.types import ColorClass
from sorter.sim.physics.model import CLOTH_N, TIMESTEP, ItemSpec, build, crumpled_sheet


def _hang_mm(young: float, thickness_m: float, sheet_m=(0.2, 0.09), seed: int = 0) -> float:
    """How far a sheet pinned by 4 vertices near its middle hangs below them, in the sim's
    integrator and timestep, after 3 s."""
    pts, tris = crumpled_sheet(np.random.default_rng(seed), sheet_m)
    n, i = CLOTH_N, CLOTH_N // 2
    pinned = [i * n + i, i * n + i - 1, (i - 1) * n + i, (i - 1) * n + i - 1]
    xml = f"""
<mujoco>
  <option timestep="{TIMESTEP}" integrator="discrete"/>
  <worldbody>
    <flexcomp name="s" type="direct" dim="2" radius="0.006" mass="0.05" pos="0 0 0.5"
      point="{" ".join(f"{v:.5f}" for v in pts.ravel())}" element="{" ".join(map(str, tris))}">
      <edge damping="0.05"/>
      <elasticity young="{young}" poisson="0.2" thickness="{thickness_m}" damping="0.002"
        elastic2d="both"/>
      <contact selfcollide="none" contype="0" conaffinity="0"/>
      <pin id="{" ".join(map(str, pinned))}"/>
    </flexcomp>
  </worldbody>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, round(3.0 / TIMESTEP))
    return (0.5 + pts[pinned, 2].mean() - d.flexvert_xpos[:, 2].min()) * 1000


def test_unload_socks_are_limp_enough_to_be_seen_hanging(sim_config):
    u = sim_config.sim.unload
    limp = _hang_mm(u.sock_young, u.sock_thickness_mm / 1000)
    stiff = _hang_mm(1e5, 0.004)  # the base's cloth
    assert limp > 85 > stiff


def test_unload_scene_gives_its_socks_their_stiffness(sim_config):
    sim_config.sim.scenes = ["unload"]
    sim_config.sim.unload.cargo = {ColorClass.DARK: 1}
    sim_config.sim.unload.sock_young = 2e4
    sim_config.sim.unload.sock_thickness_mm = 1.5
    scene = build(sim_config.sim)
    el = ET.fromstring(scene.xml).find(".//flexcomp/elasticity")
    assert el is not None
    assert float(el.get("young")) == 2e4
    assert float(el.get("thickness")) == 0.0015
    assert ItemSpec(ColorClass.DARK, (0, 0, 0), (0, 0, 0), 0.0).young == 1e5  # the base's default
