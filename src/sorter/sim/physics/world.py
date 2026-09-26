"""PhysicsWorld: the MuJoCo scene, stepped in its own thread, in simulated time.

The arm's control loop (`rebot_b601.Arm.tick`) runs inside this thread every 1/hz of simulated
time, so the real driver code steers the simulated motors with its real timing. With
`sim.realtime` > 0 the thread keeps simulated time at that multiple of wall time; 0 runs as
fast as the CPU allows.

Grip: soft cloth contacts let cloth slip out of a parallel gripper far more easily than real
fabric does. So when a close command has stopped the fingers on cloth that touches both pads,
the cloth vertices between the pads are attached to the gripper until an open command (a common
simulation stand-in for friction).
Everything else (the fingers stopping on the cloth, the cloth hanging, falling, landing) is
simulated.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Sequence

import mujoco
import numpy as np

from sorter.core.types import ColorClass, Zone
from sorter.sim.config import SimConfig
from sorter.sim.physics.model import ARM_JOINTS, ItemSpec, build_xml, item_specs
from sorter.sim.world import LOOK_POSES, in_rect

log = logging.getLogger(__name__)

SETTLE_S = 1.5  # the items fall into the box before anything else happens
GRIP_CATCH_NM = 0.5  # motor 7 torque that means "closing" (rebot_b601 holds a grip with 1 N·m)
GRIP_RELEASE_NM = 0.5  # and "opening"
GRIP_MAX_M = 0.035  # a finger further out than this pinches nothing
GRIP_STALL_M_S = 0.004  # fingers slower than this have stopped on something
GRIP_SETTLE_S = 0.3  # after a close command starts (the fingers are at rest at first too)


class PhysicsWorld:
    def __init__(
        self,
        cfg: SimConfig,
        poses: dict[str, list[float]],
        *,
        board: bool = False,
        board_z_mm: float = 1.0,
    ):
        self.cfg = cfg
        self.layout = cfg.layout
        self.poses = {k: np.asarray(v, dtype=float) for k, v in poses.items()}
        self.model = mujoco.MjModel.from_xml_string(
            build_xml(cfg, board=board, board_z_mm=board_z_mm)
        )
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        m = self.model
        self.items: list[ItemSpec] = item_specs(cfg)
        self.arm_qpos = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM_JOINTS])
        self.arm_dof = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM_JOINTS])
        self.arm_act = np.array([m.actuator(j).id for j in ARM_JOINTS])
        self.grip_act = m.actuator("gripper").id
        self.finger_qpos = m.jnt_qposadr[m.joint("finger_left").id]
        self.finger_dof = m.jnt_dofadr[m.joint("finger_left").id]
        self.tcp_site = m.site("tcp").id
        self._kp = m.actuator_gainprm[self.arm_act, 0].copy()
        self._gravcomp = m.body_gravcomp.copy()
        flex = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_FLEX, i): i for i in range(m.nflex)}
        self.item_flex = [flex[f"item{it.id}"] for it in self.items]  # item id → flex id
        self._flex = [(m.flex_vertadr[f], m.flex_vertnum[f]) for f in self.item_flex]
        self._vert_item = np.full(m.nflexvert, -1)  # flexvert index → item id
        for it in self.items:
            a, n = self._flex[it.id]
            self._vert_item[a : a + n] = it.id
        self._gripper_body = m.body("gripper_end").id
        self._pads = (m.geom("pad_left").id, m.geom("pad_right").id)
        # the grip constraint of every cloth vertex, in flexvert order
        self._grip_eq = np.array(
            [
                m.equality(f"grip{it.id}_{k}").id
                for it in self.items
                for k in range(self._flex[it.id][1])
            ]
        )
        self._grip_idx = np.zeros(0, int)  # cloth vertices attached to the gripper
        self._closing_since: float | None = None  # sim time the close command began
        self.enabled = False
        self.set_enabled(False)
        self.data.qpos[self.arm_qpos] = self.poses.get("rest", np.zeros(6))
        mujoco.mj_forward(m, self.data)
        self._step_for(SETTLE_S)
        self._initial = (self.data.qpos.copy(), self.data.qvel.copy())
        self._ticks: list[list] = []  # [fn, period, next time]
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self.camera = None  # the PhysicsCamera, once created (it also segments for the sim)
        # the calibration's tape marks (`sim.marks`), and where each lies when shown
        self._marks = {
            i: m.geom_pos[i].copy()
            for i in range(m.ngeom)
            if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("mark_")
        }

    # --- time and stepping ---

    def time(self) -> float:
        """Simulated seconds: the clock of the arm's control loop."""
        return float(self.data.time)

    def attach(self, fn: Callable[[], None], hz: float) -> None:
        """Call `fn` in the physics thread every 1/hz simulated seconds (the arm's control loop)."""
        with self.lock:
            self._ticks.append([fn, 1.0 / hz, self.data.time])

    def _step_for(self, seconds: float) -> None:
        for _ in range(max(1, round(seconds / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)

    def step(self, n: int = 1) -> None:
        """`n` physics steps, with the attached control loops. Caller holds no lock."""
        with self.lock:
            for _ in range(n):
                for t in self._ticks:
                    if self.data.time >= t[2]:
                        t[0]()
                        t[2] += t[1]
                self._grip()
                mujoco.mj_step(self.model, self.data)

    def _pad_contacts(self) -> tuple[set[int], set[int]]:
        """Cloth vertices (flexvert index) touching the left and the right finger pad."""
        c = self.data.contact
        n = self.data.ncon
        out: tuple[set[int], set[int]] = (set(), set())
        if n == 0:
            return out
        m = self.model
        geom, flex, elem, vert = c.geom[:n], c.flex[:n], c.elem[:n], c.vert[:n]
        for side, pad in enumerate(self._pads):
            for k in np.nonzero((geom == pad).any(axis=1))[0]:
                j = 1 if geom[k, 0] == pad else 0
                f = flex[k, j]
                if f < 0:
                    continue
                if vert[k, j] >= 0:
                    local = [vert[k, j]]
                elif elem[k, j] >= 0:  # a triangle of the cloth: its three vertices
                    a = m.flex_elemdataadr[f] + 3 * elem[k, j]
                    local = m.flex_elem[a : a + 3]
                else:
                    continue
                out[side].update(int(m.flex_vertadr[f] + v) for v in local)
        return out

    def _grip(self) -> None:
        """Catch the cloth pinched between the pads on a close command; let go on an open one.

        A command is told by the torque `rebot_b601` sends: a close drives motor 7 hard toward 0
        (then holds with 1 N·m); an open drives it the other way. The small torques around a
        reached target (the PD settling) mean neither, so a released item isn't caught again.
        """
        d, m = self.data, self.model
        torque = d.ctrl[self.grip_act]
        held = len(self._grip_idx) > 0
        if held:
            if not self.enabled or torque > GRIP_RELEASE_NM:
                d.eq_active[self._grip_eq[self._grip_idx]] = 0
                log.debug("grip: released %d vertices", len(self._grip_idx))
                self._grip_idx = np.zeros(0, int)
            return
        if not self.enabled or torque > -GRIP_CATCH_NM:
            self._closing_since = None
            return
        if self._closing_since is None:
            self._closing_since = d.time
        finger = d.qpos[self.finger_qpos]
        if (
            d.time - self._closing_since < GRIP_SETTLE_S
            or finger > GRIP_MAX_M
            or abs(d.qvel[self.finger_dof]) > GRIP_STALL_M_S
        ):
            return  # still closing: nothing is pinched yet
        left, right = self._pad_contacts()
        # the top item pressed by both fingers: the pile's items pass through each other (no
        # cloth-cloth contacts), so real fingers would squeeze the one on top
        both = set(self._vert_item[sorted(left)]) & set(self._vert_item[sorted(right)])
        both.discard(-1)
        if not both:
            return
        touch = np.array(sorted(left | right), int)
        pinched = {
            max(both, key=lambda i: d.flexvert_xpos[touch[self._vert_item[touch] == i], 2].max())
        }
        p = d.site_xpos[self.tcp_site]
        R = d.site_xmat[self.tcp_site].reshape(3, 3)
        local = (d.flexvert_xpos - p) @ R  # TCP frame: x = approach, fingers along y
        between = (
            (local[:, 0] > -0.06)
            & (local[:, 0] < 0.008)
            & (np.abs(local[:, 1]) < max(finger, 0.0) + 0.009)
            & (np.abs(local[:, 2]) < 0.02)
        )
        between &= np.isin(self._vert_item, sorted(pinched))
        touching = [v for v in left | right if self._vert_item[v] in pinched]
        idx = np.union1d(np.nonzero(between)[0], np.array(sorted(touching), int))
        g = self._gripper_body
        local_g = (d.flexvert_xpos[idx] - d.xpos[g]) @ d.xmat[g].reshape(3, 3)
        eq = self._grip_eq[idx]
        m.eq_data[eq, 3:6] = local_g  # hold each vertex where it is now, in the gripper_end frame
        d.eq_active[eq] = 1
        self._grip_idx = idx
        log.debug(
            "grip: caught %d vertices of items %s, finger %.1f mm",
            len(idx),
            sorted(int(i) for i in pinched),
            finger * 1000,
        )

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="physics", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def _loop(self) -> None:
        wall0, sim0 = time.monotonic(), self.data.time
        while self._running.is_set():
            self.step(5)
            rate = self.cfg.realtime
            ahead = 0.0
            if rate > 0:
                ahead = (self.data.time - sim0) / rate - (time.monotonic() - wall0)
                if ahead < -0.5:  # can't keep up: don't try to catch up in a burst
                    wall0, sim0 = time.monotonic(), self.data.time
            # always let go for a moment: the lock is not fair, and other threads (the driver,
            # the camera, the dashboard) would otherwise wait for it indefinitely
            time.sleep(max(ahead, 0.0002))

    def show_marks(self, on: bool) -> None:
        """Show or hide the tape marks (the dashboard shows them in the calibrate mode only):
        hidden, they lie under the table, out of every camera's sight. They never collide."""
        with self.lock:
            for i, pos in self._marks.items():
                self.model.geom_pos[i] = pos if on else (pos[0], pos[1], -0.5)
            mujoco.mj_kinematics(self.model, self.data)

    # --- the arm (used by the motor backend, under the lock) ---

    def set_enabled(self, on: bool) -> None:
        """Motors on: position servos with gravity compensation; off: limp, the arm falls."""
        m = self.model
        with self.lock:
            self.enabled = on
            m.actuator_gainprm[self.arm_act, 0] = self._kp if on else 0
            m.actuator_biasprm[self.arm_act, 1] = -self._kp if on else 0
            m.body_gravcomp[:] = self._gravcomp if on else 0
            if not on:
                self.data.ctrl[self.grip_act] = 0

    def arm_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        """q, dq, joint torques, finger position (m), finger velocity (m/s)."""
        d = self.data
        with self.lock:
            return (
                d.qpos[self.arm_qpos].copy(),
                d.qvel[self.arm_dof].copy(),
                d.actuator_force[self.arm_act].copy(),
                float(d.qpos[self.finger_qpos]),
                float(d.qvel[self.finger_dof]),
            )

    def command(self, q: Sequence[float] | None = None, gripper_nm: float | None = None) -> None:
        with self.lock:
            if q is not None:
                self.data.ctrl[self.arm_act] = q
            if gripper_nm is not None and self.enabled:
                self.data.ctrl[self.grip_act] = gripper_nm

    def teleport_arm(self, q: Sequence[float], finger_m: float | None = None) -> None:
        """Put the arm at `q` at once (tools and tests; nothing moves physically)."""
        with self.lock:
            d = self.data
            d.qpos[self.arm_qpos] = q
            d.qvel[self.arm_dof] = 0
            d.ctrl[self.arm_act] = q
            if finger_m is not None:
                d.qpos[self.finger_qpos] = finger_m
            mujoco.mj_forward(self.model, d)

    def joints(self) -> np.ndarray:
        with self.lock:
            return self.data.qpos[self.arm_qpos].copy()

    def tcp(self) -> np.ndarray:
        """TCP position, mm."""
        with self.lock:
            return self.data.site_xpos[self.tcp_site] * 1000

    @property
    def looking_at(self) -> Zone | None:
        q = self.joints()
        for zone, name in LOOK_POSES.items():
            if name in self.poses and np.allclose(q, self.poses[name], atol=math.radians(0.5)):
                return zone
        return None

    # --- items ---

    def vertices(self, item: int) -> np.ndarray:
        """Cloth vertices of item `item`, mm."""
        a, n = self._flex[item]
        with self.lock:
            return self.data.flexvert_xpos[a : a + n] * 1000

    def location(self, item: int) -> tuple[str, ColorClass | None]:
        """Where item `item` is: box / background / gripper / bin (+ color) / table."""
        with self.lock:
            if item in self._vert_item[self._grip_idx]:
                return "gripper", None
        v = self.vertices(item)
        x, y, _ = v.mean(axis=0)
        lay = self.layout
        if in_rect(x, y, lay.box):
            return "box", None
        if in_rect(x, y, lay.background):
            return "background", None
        half = lay.bins.size_mm / 2
        for color, (bx, by) in lay.bins.centers_mm.items():
            if abs(x - bx) <= half and abs(y - by) <= half:
                return "bin", color
        return "table", None

    def at(self, location: str) -> list[ItemSpec]:
        return [it for it in self.items if self.location(it.id)[0] == location]

    def reset_if_sorted(self) -> None:
        """Nothing left in the box or on the mat: the items go back into the box, as they were."""
        if self.items and all(self.location(it.id)[0] in ("bin", "table") for it in self.items):
            with self.lock:
                q0, v0 = self._initial
                arm = self.data.qpos[self.arm_qpos].copy()
                self.data.qpos[:] = q0
                self.data.qvel[:] = v0
                self.data.qpos[self.arm_qpos] = arm
                mujoco.mj_forward(self.model, self.data)
            log.info("physics: items back in the box")
