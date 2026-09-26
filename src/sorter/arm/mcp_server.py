"""MCP server that lets an AI agent drive the SO-101 through `So101Arm` (stdio transport).

    uv run python -m sorter.arm.mcp_server [--sim | --real] [--config-dir DIR]

Real or simulated arm: `--sim` / `--real`, else `backends.arm` of the config. The simulated arm is
`So101Arm` on a `MockBus` (`backend.create_mock`), so the whole controller (IK, streaming,
hold) runs without hardware.
On exit a connected arm goes to `rest` and the motors turn off (`shutdown()`).

Every tool returns `{"ok": true, ...}` or `{"ok": false, "error": "..."}`. Units: mm and degrees,
arm base frame (+X forward, +Y left, +Z up, origin on the base plate).
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import logging
import math
import signal
import sys
import threading
from collections.abc import Callable
from typing import Any

import numpy as np
from mcp.server.fastmcp import FastMCP

from sorter.arm import kinematics as kin
from sorter.arm.backend import create, create_mock
from sorter.arm.controller import POSE_NAMES, So101Arm
from sorter.core.config import DEFAULT_CONFIG_DIR, Backend, Config, load_config
from sorter.core.errors import EStopped, SorterError

log = logging.getLogger("sorter.arm.mcp")

MAX_REACH_MM = 450.0  # fingertips farther than this from the base axis: refused
DEFAULT_SPEED = 0.5
MAX_SPEED = 1.0

INSTRUCTIONS = """\
Controls a real SO-101 arm (5 joints + gripper, hobby servos). It can hurt people and itself.
Workflow: arm_info -> arm_connect -> arm_status -> arm_plan_xyz (dry check) -> arm_move_to_xyz ...
Rules:
- Positions are mm in the arm BASE frame: +X forward, +Y left, +Z up, origin on the base plate.
  The table is about 17 mm below the origin. The tip is the point between the fingertips.
- xyz moves keep the gripper pointing down (tilted up to max_tilt_deg when needed). Straight down
  only works about 100..300 mm from the base and below z ~ 80 mm; higher, the tool tilts.
- Named poses (arm_goto_pose) are taught on the rig and safe; prefer them for big moves.
- Joint moves (arm_move_joints, arm_goto_pose, xyz with linear=false) do not check the path,
  only the goal. Near the table use linear=true or arm_move_relative (straight line, checked).
- The servos sag under load: the arm may stop a few degrees short. Check arm_status after moves.
- There is NO collision avoidance. If anything looks wrong call arm_stop (holds the position);
  arm_resume clears it. Motors are never switched off except at rest (arm_disconnect).
"""


def _deg(q) -> list[float]:
    return [round(math.degrees(float(v)), 2) for v in q]


def _mm(p) -> list[float]:
    return [round(float(v), 1) for v in p]


class ArmTools:
    """The tools, independent of MCP: plain methods that return result dicts."""

    def __init__(self, cfg: Config, make_arm: Callable[[bool], So101Arm], simulate: bool):
        self.cfg = cfg
        self._make_arm = make_arm
        self.default_simulate = simulate
        self.arm: So101Arm | None = None
        self.simulated = simulate
        self._motion = threading.Lock()  # one motion at a time; stop / status don't wait

    # --- helpers ---

    def _call(self, fn: Callable[[], dict[str, Any]]) -> dict[str, Any]:
        try:
            return {"ok": True, **fn()}
        except EStopped:
            return {"ok": False, "error": "EStopped: the arm is held; arm_resume first"}
        except SorterError as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}
        except (ValueError, TypeError, KeyError) as e:
            return {"ok": False, "error": f"invalid request: {e}"}
        except Exception as e:  # unexpected bus / SDK failure
            log.exception("unexpected error")
            return {"ok": False, "error": f"unexpected error: {e!r}"}

    def _motion_call(self, fn: Callable[[], dict[str, Any]]) -> dict[str, Any]:
        if not self._motion.acquire(blocking=False):
            return {"ok": False, "error": "another motion is running; arm_stop to interrupt it"}
        try:
            out = self._call(fn)
        finally:
            self._motion.release()
        if self.arm is not None and self.arm.bus is not None:
            state = self._call(self._state)
            out["state"] = {k: v for k, v in state.items() if k != "ok"}
        return out

    def _arm(self) -> So101Arm:
        if self.arm is None or self.arm.bus is None:
            raise SorterError("not connected; call arm_connect first")
        return self.arm

    def _speed(self, speed_scale: float) -> float:
        if not speed_scale > 0:
            raise ValueError("speed_scale must be > 0")
        return min(float(speed_scale), MAX_SPEED)

    def _check_target(self, p: np.ndarray) -> None:
        a = self.cfg.arm
        if p[2] < a.table_z_mm:
            raise ValueError(f"z {p[2]:.0f} mm is below the table guard ({a.table_z_mm:.0f} mm)")
        r = math.hypot(p[0], p[1])
        if r > MAX_REACH_MM:
            raise ValueError(f"{r:.0f} mm from the base axis, more than {MAX_REACH_MM:.0f} mm")

    def _max_tilt(self, max_tilt_deg: float | None) -> float:
        return self.cfg.arm.grasp_max_tilt_deg if max_tilt_deg is None else float(max_tilt_deg)

    def _solve(self, p: np.ndarray, max_tilt_deg: float):
        arm = self._arm()
        self._check_target(p)
        r = arm.solve_down(p, arm.read_q(), max_tilt_deg)
        if r is None:
            raise ValueError(
                f"({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f}) mm is out of reach with the tool "
                f"within {max_tilt_deg:.0f}° of straight down"
            )
        return r

    def _line(self, p1: np.ndarray, max_tilt_deg: float) -> tuple[list[np.ndarray], float]:
        arm = self._arm()
        self._check_target(p1)
        q = arm.read_q()
        tilt = kin.tilt_deg(kin.fk(q))
        if tilt > max_tilt_deg:
            raise ValueError(
                f"a straight move starts with the tool within {max_tilt_deg:.0f}° of straight "
                f"down, it is at {tilt:.0f}° now; move there with linear=false first"
            )
        p0 = arm.tcp(q)
        return arm.plan_line(p0, p1, q, max_tilt_deg), float(np.linalg.norm(p1 - p0))

    def _state(self) -> dict[str, Any]:
        arm = self._arm()
        q = arm.read_q()
        T = kin.fk(q)
        return {
            "joints_deg": dict(zip(kin.JOINT_NAMES, _deg(q), strict=True)),
            "tip_mm": _mm(arm.tcp(q)),
            "tilt_from_down_deg": round(kin.tilt_deg(T), 1),
            "gripper_opening": round(arm.read_gripper(), 3),
            "held": arm._held.is_set(),
            "at_pose": arm._at,
        }

    # --- tools ---

    def info(self) -> dict[str, Any]:
        a = self.cfg.arm
        limits = self.arm._limits if self.arm is not None else kin.LIMITS
        return {
            "ok": True,
            "arm": "SO-101 (LeRobot), 5 revolute joints + gripper, Feetech STS3215 servos",
            "frame": "arm base frame, mm: +X forward, +Y left, +Z up, origin on the base plate",
            "tip": "point between the fingertips (URDF gripper_frame + tcp_extend_mm)",
            "joints": list(kin.JOINT_NAMES),
            "joint_limits_deg": {
                n: _deg(lim) for n, lim in zip(kin.JOINT_NAMES, limits, strict=True)
            },
            "poses": {n: _deg(q) for n, q in self.cfg.poses.items()},
            "known_pose_names": list(POSE_NAMES),
            "table_guard_z_mm": a.table_z_mm,
            "max_reach_mm": MAX_REACH_MM,
            "default_max_tilt_deg": a.grasp_max_tilt_deg,
            "speed": {
                "default": DEFAULT_SPEED,
                "max": MAX_SPEED,
                "joint_deg_s_at_1": a.joint_speed_deg_s,
                "linear_mm_s_at_1": a.linear_speed_mm_s,
            },
            "gripper": "opening 0 (closed) .. 1 (open), torque limited",
            "connected": self.arm is not None and self.arm.bus is not None,
            "simulated": self.simulated if self.arm is not None else self.default_simulate,
        }

    def connect(self, simulate: bool | None = None) -> dict[str, Any]:
        def run():
            sim = self.default_simulate if simulate is None else simulate
            if self.arm is not None and self.arm.bus is not None:
                if sim != self.simulated:
                    raise SorterError("already connected to the other arm; disconnect first")
            else:
                self.arm = self._make_arm(sim)
                self.simulated = sim
            self.arm.start()  # torque on, holding the current position
            return {"simulated": self.simulated, **self._state()}

        with self._motion:
            return self._call(run)

    def status(self) -> dict[str, Any]:
        return self._call(lambda: {"simulated": self.simulated, **self._state()})

    def plan_xyz(self, x, y, z, max_tilt_deg=None, linear=False) -> dict[str, Any]:
        def run():
            p = np.array([x, y, z], dtype=float)
            tilt = self._max_tilt(max_tilt_deg)
            r = self._solve(p, tilt)
            out = {
                "reachable": True,
                "joints_deg": _deg(r.q),
                "tilt_from_down_deg": round(r.tilt_deg, 1),
                "position_error_mm": round(r.pos_err_mm, 2),
            }
            if linear:
                qs, length = self._line(p, tilt)
                out["line"] = {"waypoints": len(qs), "length_mm": round(length, 1)}
            return out

        return self._call(run)

    def move_to_xyz(self, x, y, z, max_tilt_deg=None, linear=False, speed_scale=DEFAULT_SPEED):
        def run():
            p = np.array([x, y, z], dtype=float)
            tilt, speed = self._max_tilt(max_tilt_deg), self._speed(speed_scale)
            arm = self._arm()
            if linear:
                qs, length = self._line(p, tilt)  # all planned before moving
                arm.follow(qs, length / speed)  # follow() times the line by its length
            else:
                arm.move_joints(self._solve(p, tilt).q, speed)
            return {}

        return self._motion_call(run)

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, max_tilt_deg=None, speed_scale=DEFAULT_SPEED):
        def run():
            arm = self._arm()
            p = arm.tcp(arm.read_q()) + np.array([dx, dy, dz], dtype=float)
            return {"target_mm": _mm(p)}

        target = self._call(run)
        if not target["ok"]:
            return target
        x, y, z = target["target_mm"]
        return self.move_to_xyz(x, y, z, max_tilt_deg, linear=True, speed_scale=speed_scale)

    def move_joints(self, joints_deg: list[float], speed_scale=DEFAULT_SPEED):
        def run():
            arm = self._arm()
            if len(joints_deg) != kin.N_JOINTS:
                raise ValueError(f"need {kin.N_JOINTS} joint angles {kin.JOINT_NAMES}")
            q = np.radians(np.asarray(joints_deg, dtype=float))
            lo, hi = arm._limits[:, 0], arm._limits[:, 1]
            bad = [
                f"{n} {math.degrees(v):.1f}° not in [{math.degrees(a):.1f}, {math.degrees(b):.1f}]"
                for n, v, a, b in zip(kin.JOINT_NAMES, q, lo, hi, strict=True)
                if not a <= v <= b
            ]
            if bad:
                raise ValueError("; ".join(bad))
            self._check_target(arm.tcp(q))
            arm.move_joints(q, self._speed(speed_scale))
            return {}

        return self._motion_call(run)

    def goto_pose(self, name: str, speed_scale=DEFAULT_SPEED):
        def run():
            if name not in self.cfg.poses:
                raise ValueError(f"unknown pose {name!r}; taught: {sorted(self.cfg.poses)}")
            self._arm().goto(name, self._speed(speed_scale))
            return {}

        return self._motion_call(run)

    def gripper(self, opening: float):
        def run():
            if not 0 <= opening <= 1:
                raise ValueError("opening must be in 0..1")
            return {"measured_opening": round(self._arm().set_gripper(opening), 3)}

        return self._motion_call(run)

    def stop(self) -> dict[str, Any]:
        def run():
            self._arm().hold()
            return {"held": True, "note": "motors stay on; arm_resume to move again"}

        return self._call(run)

    def resume(self) -> dict[str, Any]:
        def run():
            self._arm().start()  # clears the hold, goals = present position
            return self._state()

        with self._motion:
            return self._call(run)

    def disconnect(self, go_rest: bool = True) -> dict[str, Any]:
        def run():
            arm = self._arm()
            if go_rest:
                arm.shutdown()  # rest, then motors off (stays on, holding, if rest fails)
            else:
                arm.hold()  # motors stay on, holding here
                arm.bus.close()
                arm.bus = None
            return {"motors": "off at rest" if go_rest else "on, holding"}

        if self.arm is not None and self.arm.bus is not None:
            self.arm.hold()  # interrupt a running motion before taking the motion lock
        with self._motion:
            return self._call(run)


def build_server(tools: ArmTools) -> FastMCP:
    mcp = FastMCP("so101-arm", instructions=INSTRUCTIONS)

    async def bg(fn, *args, **kwargs) -> dict:
        return await asyncio.to_thread(fn, *args, **kwargs)

    @mcp.tool()
    def arm_info() -> dict:
        """Static facts: frame, joint limits, taught poses (deg), guards and speeds. No motion."""
        return tools.info()

    @mcp.tool()
    async def arm_connect(simulate: bool | None = None) -> dict:
        """Open the servo bus and switch the motors on, holding the current position.

        simulate=True uses a simulated arm, False the real one; default from the server config.
        """
        return await bg(tools.connect, simulate)

    @mcp.tool()
    def arm_status() -> dict:
        """Joint angles (deg), fingertip position (mm), tilt from straight down, gripper, hold."""
        return tools.status()

    @mcp.tool()
    async def arm_plan_xyz(
        x: float, y: float, z: float, max_tilt_deg: float | None = None, linear: bool = False
    ) -> dict:
        """Check WITHOUT moving whether the tip can reach (x, y, z) mm with the gripper down.

        max_tilt_deg: allowed tilt from straight down (default arm.grasp_max_tilt_deg).
        linear=True also checks the straight line from the current tip.
        """
        return await bg(tools.plan_xyz, x, y, z, max_tilt_deg, linear)

    @mcp.tool()
    async def arm_move_to_xyz(
        x: float,
        y: float,
        z: float,
        max_tilt_deg: float | None = None,
        linear: bool = False,
        speed_scale: float = DEFAULT_SPEED,
    ) -> dict:
        """Move the tip to (x, y, z) mm (base frame), gripper pointing down, and wait.

        linear=True: straight line, every point checked (use near the table or objects);
        False: joint move, only the goal is checked. speed_scale is capped at 1.
        """
        return await bg(tools.move_to_xyz, x, y, z, max_tilt_deg, linear, speed_scale)

    @mcp.tool()
    async def arm_move_relative(
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        max_tilt_deg: float | None = None,
        speed_scale: float = DEFAULT_SPEED,
    ) -> dict:
        """Move the tip by (dx, dy, dz) mm along a straight line, gripper pointing down."""
        return await bg(tools.move_relative, dx, dy, dz, max_tilt_deg, speed_scale)

    @mcp.tool()
    async def arm_move_joints(joints_deg: list[float], speed_scale: float = DEFAULT_SPEED) -> dict:
        """Move the 5 joints to angles in degrees:
        [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]. Path not checked."""
        return await bg(tools.move_joints, joints_deg, speed_scale)

    @mcp.tool()
    async def arm_goto_pose(name: str, speed_scale: float = DEFAULT_SPEED) -> dict:
        """Move to a pose taught on the rig: rest, home, look_box, look_bg, place_bg, bin_*."""
        return await bg(tools.goto_pose, name, speed_scale)

    @mcp.tool()
    async def arm_gripper(opening: float) -> dict:
        """Set the gripper: 0 closed .. 1 open. Returns the measured opening (stalls on cloth)."""
        return await bg(tools.gripper, opening)

    @mcp.tool()
    def arm_stop() -> dict:
        """Stop now and hold the position (motors stay on). Safe any time. arm_resume to go on."""
        return tools.stop()

    @mcp.tool()
    async def arm_resume() -> dict:
        """Clear a stop: hold the current position and accept motions again. No motion."""
        return await bg(tools.resume)

    @mcp.tool()
    async def arm_disconnect(go_rest: bool = True) -> dict:
        """Finish: go to `rest`, motors off, release the bus. go_rest=False keeps holding here."""
        return await bg(tools.disconnect, go_rest)

    return mcp


def make_tools(cfg: Config, simulate: bool) -> ArmTools:
    return ArmTools(cfg, lambda sim: create_mock(cfg) if sim else create(cfg), simulate)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.arm.mcp_server")
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--sim", action="store_true", help="simulated arm by default")
    g.add_argument("--real", action="store_true", help="real arm by default")
    args = p.parse_args(argv)
    # stdout belongs to the MCP protocol: log to stderr only
    logging.basicConfig(stream=sys.stderr, level=logging.INFO, format="%(name)s %(message)s")
    cfg = load_config(args.config_dir)
    simulate = args.sim or (not args.real and cfg.backends.arm == Backend.SIM)
    tools = make_tools(cfg, simulate)

    def on_exit() -> None:
        if tools.arm is not None and tools.arm.bus is not None:
            try:
                tools.arm.shutdown()
            except Exception:
                log.exception("shutdown failed")

    atexit.register(on_exit)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # run atexit on SIGTERM
    build_server(tools).run(transport="stdio")


if __name__ == "__main__":
    main()
