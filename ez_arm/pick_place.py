"""Pick the T-shirt off the table and put it somewhere else.

look (scan joint1 over the table) → SAM3 + depth → pre-grasp 10 cm above, gripper down, fingers
across the shirt's short side → open, straight down, close → lift → check → carry → lower, release
→ back up → look again to confirm the shirt moved.
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np
from rebot_b601 import kinematics as K

from ez_arm.robot import ArmError, Robot, load_poses

log = logging.getLogger("ez_arm")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs")


@dataclass
class PickConfig:
    scan_j1_deg: tuple[float, ...] = (0.0, 30.0, -30.0, 55.0, -55.0)
    pregrasp_above_m: float = 0.10
    pregrasp_max_z_m: float = 0.13  # gripper-down reach ends at z ≈ 0.14
    grasp_below_top_m: float = 0.015  # fingertips this far below the cloth top
    grasp_floor_z_m: float = 0.008  # never lower (table at z = 0)
    empty_below: float = 0.018  # opening after closing below this = empty (closed reads ~0.008; 1 mm/finger ≈ 0.02)
    place_xyz: tuple[float, float, float] | None = None  # None: mirror the pick across y
    place_release_z_m: float = 0.10
    speed: float = 0.3
    slow: float = 0.15  # descent / grasp moves
    attempts: int = 3


def _fingers_yaw(q) -> float:
    """Yaw in base xy of the finger closing axis (TCP y) with the gripper down."""
    R = K.fk(q)[1]
    y = R[:, 1]
    return math.atan2(y[1], y[0])


def roll_for_yaw(q, want_yaw: float) -> float:
    """joint6 [deg] that turns the finger closing axis to want_yaw (mod 180°), within limits."""
    best = None
    for j6 in np.arange(-125.0, 125.1, 2.5):
        qq = np.array(q, float)
        qq[5] = math.radians(j6)
        d = (_fingers_yaw(qq) - want_yaw + math.pi / 2) % math.pi - math.pi / 2
        cost = abs(d) + 0.002 * abs(j6 - math.degrees(q[5]))
        if best is None or cost < best[0]:
            best = (cost, j6)
    return best[1]


class PickPlace:
    def __init__(self, robot: Robot, detector=None, cfg: PickConfig | None = None, sim_target=None):
        self.r = robot
        self.det = detector
        self.cfg = cfg or PickConfig()
        self.sim_target = sim_target  # simulation: (x, y, z, yaw) instead of the camera
        os.makedirs(OUT_DIR, exist_ok=True)
        self.stamp = time.strftime("%Y%m%d-%H%M%S")

    # ---- perception ------------------------------------------------------
    def look_and_find(self, j1_list=None, tag="find"):
        look = list(load_poses()["look"])
        for j1 in j1_list or self.cfg.scan_j1_deg:
            self.r.joints([j1] + look[1:], self.cfg.speed)
            time.sleep(0.6)  # let the wrist settle before the frame
            if self.sim_target is not None:
                x, y, z, yaw = self.sim_target
                log.info("sim target at (%.3f, %.3f, %.3f)", x, y, z)
                return _SimTarget(np.array([x, y, z]), yaw), j1
            rgbd, q = self.r.snap()
            targets, dbg = self.det.detect(rgbd, q)
            path = os.path.join(OUT_DIR, f"{self.stamp}-{tag}-j1_{j1:+.0f}.jpg")
            cv2.imwrite(path, dbg)
            cv2.imwrite(path.replace(".jpg", "-raw.jpg"), rgbd.color)
            if targets:
                t = targets[0]
                log.info(
                    "shirt (%s, %.2f) at (%.3f, %.3f, %.3f), %.0f cm², yaw %.0f°, j1 %+.0f° → %s",
                    t.prompt, t.score, *t.xyz, t.area_m2 * 1e4, math.degrees(t.yaw), j1, path,
                )
                return t, j1
            log.info("nothing on the table at j1 %+.0f° (%s)", j1, path)
        return None, None

    # ---- motion ----------------------------------------------------------
    def _down_reachable(self, x, y, z) -> bool:
        try:
            self.r.plan(x, y, z, "down")
            return True
        except ArmError:
            return False

    def _highest_down(self, x, y, zmax, zmin) -> float:
        """Highest z in [zmin, zmax] the TCP reaches at (x, y) with the gripper down."""
        z = zmax
        while z > zmin and not self._down_reachable(x, y, z):
            z -= 0.01
        return max(z, zmin)

    def grasp(self, t) -> bool:
        c = self.cfg
        x, y, ztop = map(float, t.xyz)
        zg = max(ztop - c.grasp_below_top_m, c.grasp_floor_z_m)
        zp = self._highest_down(x, y, min(ztop + c.pregrasp_above_m, c.pregrasp_max_z_m), zg + 0.03)
        log.info("pre-grasp (%.3f, %.3f, %.3f), grasp z %.3f", x, y, zp, zg)
        self.r.go(x, y, zp, "down", speed=c.speed)
        # fingers close across the short side: closing axis ⟂ long axis
        j6 = roll_for_yaw(self.r.q, t.yaw + math.pi / 2)
        self.r.wrist_roll(j6, c.speed)
        self.r.grip(1.0)
        self.r.rel(dz=zg - self.r.tcp[2], approach="down", speed=c.slow)
        self.r.grip(0.0)
        time.sleep(0.8)
        opening = self.r.state()["gripper_opening"]
        self.r.rel(dz=zp - self.r.tcp[2], approach="down", speed=c.slow)
        time.sleep(0.3)
        opening = self.r.state()["gripper_opening"]
        held = opening is None or opening >= c.empty_below or self.sim_target is not None
        log.info("after lift: gripper opening %.3f → %s", opening or -1, "HOLDING" if held else "empty")
        return held

    def place_point(self, t):
        if self.cfg.place_xyz is not None:
            return np.array(self.cfg.place_xyz, float)
        x, y = float(t.xyz[0]), float(t.xyz[1])
        side = -1.0 if y >= 0 else 1.0  # the other side of the table
        return np.array([0.25, side * 0.20, self.cfg.place_release_z_m])

    def place(self, p):
        c = self.cfg
        x, y, z = p
        above = self._highest_down(x, y, c.pregrasp_max_z_m, 0.04)
        if not self._down_reachable(x, y, above):  # far point: drop it with a tilted gripper
            log.warning("gripper-down not reachable at (%.2f, %.2f); releasing with a free wrist", x, y)
            self.r.go(x, y, max(z, 0.10), "free", speed=c.speed)
            self.r.grip(1.0)
            time.sleep(0.6)
            return
        z = min(z, above)
        log.info("carry to (%.3f, %.3f, %.3f), release at z %.3f", x, y, above, z)
        # carry high so the hanging shirt clears the table, then down over the place point
        cx, cy, _ = self.r.tcp
        try:
            self.r.go((cx + x) / 2, (cy + y) / 2, 0.25, "free", speed=c.speed)
        except ArmError as e:
            log.warning("high carry point refused (%s); going direct", e)
        self.r.go(x, y, above, "down", speed=c.speed)
        self.r.rel(dz=z - self.r.tcp[2], approach="down", speed=c.slow)
        self.r.grip(1.0)
        time.sleep(0.6)
        self.r.rel(dz=above - self.r.tcp[2], approach="down", speed=c.speed)
        self.r.grip(0.3)

    # ---- the task --------------------------------------------------------
    def run(self) -> bool:
        c = self.cfg
        self.r.grip(0.3)
        self.r.pose("ready", c.speed)
        for attempt in range(1, c.attempts + 1):
            t, j1 = self.look_and_find(tag=f"try{attempt}")
            if t is None:
                log.error("no T-shirt found on the table")
                break
            place = self.place_point(t)
            self.r.pose("ready", c.speed)
            try:
                held = self.grasp(t)
            except ArmError as e:
                log.error("grasp failed: %s", e)
                self.r.grip(1.0)
                self._retreat()
                continue
            if not held:
                log.warning("missed (attempt %d/%d), retrying", attempt, c.attempts)
                self.r.grip(1.0)
                self._retreat()
                continue
            self.place(place)
            self._retreat()
            # confirm: look where it was and where it went
            if self.sim_target is None:
                j1_place = math.degrees(math.atan2(place[1], place[0]))
                after, _ = self.look_and_find([round(j1_place)], tag="verify")
                if after is not None:
                    d = np.linalg.norm(after.xyz[:2] - place[:2])
                    log.info("shirt now at (%.3f, %.3f), %.0f cm from the place point", *after.xyz[:2], d * 100)
            self.r.pose("ready", c.speed)
            self.r.home(c.speed)
            log.info("DONE: T-shirt moved to (%.2f, %.2f)", place[0], place[1])
            return True
        self.r.pose("ready", c.speed)
        self.r.home(c.speed)
        return False

    def _retreat(self):
        """Straight up to a safe height, then the ready pose."""
        x, y, z = self.r.tcp
        top = self._highest_down(x, y, self.cfg.pregrasp_max_z_m, z)
        if top > z + 0.005:
            try:
                self.r.rel(dz=top - z, approach="down", speed=self.cfg.speed)
            except ArmError as e:
                log.warning("retreat lift: %s", e)
        self.r.pose("ready", self.cfg.speed)


@dataclass
class _SimTarget:
    xyz: np.ndarray
    yaw: float
