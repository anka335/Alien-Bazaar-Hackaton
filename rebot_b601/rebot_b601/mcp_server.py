"""MCP server that lets an AI agent drive the reBot Arm B601-RS.

Run (stdio transport):  python -m rebot_b601.mcp_server

Environment:
  REBOT_DRY_RUN=1        simulated arm by default (no CAN access)
  REBOT_VIEWER_PORT=8765 browser 3D twin (default 8765 in dry run, off otherwise; 'off' disables)
  REBOT_ON_EXIT=home     what to do with a connected arm when the server exits:
                         home (default: return to zero pose, then disable), off, leave
  REBOT_MAX_SPEED        cap for speed_scale (default 0.6), REBOT_DEFAULT_SPEED (0.3)
  REBOT_Z_MIN            lowest allowed TCP height [m] (default 0.03)
  see rebot_b601/config.py for the rest.

All motion tools return ``{"ok": true, ...}`` or ``{"ok": false, "error": "..."}``;
refusals are normal, read the message and adjust the request.
"""

from __future__ import annotations

import asyncio
import atexit
import logging
import os
import signal
import sys

from mcp.server.fastmcp import FastMCP

from . import config as C
from .arm import Arm, ArmError
from .viewer import start_viewer

# stdout belongs to the MCP protocol: log to stderr only
logging.basicConfig(stream=sys.stderr, level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("rebot_b601.mcp")

INSTRUCTIONS = """\
Controls a real 6-DoF robot arm (reBot Arm B601-RS) with a gripper. It can hurt people and itself.
Workflow: arm_info -> arm_connect -> arm_status -> arm_plan_xyz (dry check) -> arm_move_to_xyz ...
Rules:
- Positions are metres in the arm BASE frame: +X forward, +Y left, +Z up, origin on the base plate.
  The TCP is the gripper tip; the gripper approach axis points from the wrist to the fingertips.
- Always start with small, slow moves and check arm_status afterwards. Prefer arm_plan_xyz first.
- approach 'down' (gripper pointing at the table) only works low (z <~ 0.14 m); 'forward' only
  higher (z >~ 0.15 m); 'free' lets the solver choose the wrist. If a target is refused, believe it.
- There is NO collision avoidance against objects, only a table/base guard. Keep the workspace clear.
- If anything looks wrong call arm_stop (holds position). arm_emergency_disable cuts torque and the
  arm will drop: use it only for real emergencies.
- When finished call arm_disconnect (returns the arm to its rest pose and switches torque off).
"""

mcp = FastMCP("rebot-b601", instructions=INSTRUCTIONS)
_arm = Arm()
_viewer_url: str | None = None


def _result(fn, *args, **kwargs) -> dict:
    try:
        out = fn(*args, **kwargs)
        return out if isinstance(out, dict) else {"ok": True, "result": out}
    except ArmError as e:
        return {"ok": False, "error": str(e)}
    except (ValueError, TypeError) as e:
        return {"ok": False, "error": f"invalid request: {e}"}
    except Exception as e:  # pragma: no cover - unexpected hardware/SDK failure
        log.exception("unexpected error")
        return {"ok": False, "error": f"unexpected error: {e!r}"}


async def _run(fn, *args, **kwargs) -> dict:
    return await asyncio.to_thread(_result, fn, *args, **kwargs)


def _approach(approach: str, approach_vector: list[float] | None):
    if approach_vector is not None:
        return list(approach_vector)
    return approach


@mcp.tool()
def arm_info() -> dict:
    """Static facts about the arm: frames, limits, workspace and what the approach modes can reach. No motion."""
    return {
        "arm": "reBot Arm B601-RS, 6 revolute joints + gripper",
        "frame": "base frame, metres: +X forward, +Y left, +Z up; origin on the base plate (joint 1 axis at z=0.075)",
        "tcp": "gripper_end frame at the fingertip end; approach axis = from wrist towards fingertips",
        "home_pose": "all joints 0 deg (folded 'sit-down' pose): TCP at about (0.30, 0.00, 0.22), gripper pointing forward",
        "joint_limits_deg": {n: C.JOINT_LIMITS_DEG[i].tolist() for i, n in enumerate(C.JOINT_NAMES)},
        "workspace_box_m": {"x": C.WORKSPACE_X, "y": C.WORKSPACE_Y, "z": C.WORKSPACE_Z},
        "min_tcp_height_m": C.Z_MIN,
        "reach_hint": "free orientation: roughly 0.15..0.7 m from the base axis, z 0.03..0.5; "
        "approach 'down': z <= ~0.14 m and x 0.1..0.45; approach 'forward': z >= ~0.15 m",
        "speed": {"default_scale": C.DEFAULT_SPEED_SCALE, "max_scale": C.MAX_SPEED_SCALE,
                  "peak_joint_speed_deg_s_at_scale_1": C.JOINT_SPEED_DPS.tolist()},
        "gripper": "opening 0 (closed) .. 1 (open); torque limited",
        "dry_run_default": C.DRY_RUN,
        "viewer_url": _viewer_url,
    }


@mcp.tool()
async def arm_connect(simulate: bool | None = None, enable_torque: bool = True) -> dict:
    """Connect to the arm over CAN and (by default) switch the motors on, holding the current pose.

    simulate=True uses a fake arm (no hardware). enable_torque=False only reads the state (motors limp,
    no motion possible). Fails if a motor is silent or the joint readings look uncalibrated.
    """
    return await _run(_arm.connect, enable=enable_torque, simulate=simulate)


@mcp.tool()
def arm_status() -> dict:
    """Current joint angles (deg), TCP position (m), gripper opening, motor temperature and any fault."""
    return _result(_arm.status)


@mcp.tool()
async def arm_plan_xyz(x: float, y: float, z: float, approach: str = "free",
                       approach_vector: list[float] | None = None, linear: bool = False) -> dict:
    """Check whether the TCP can reach (x, y, z) [m] WITHOUT moving. Returns the joint solution and error.

    approach: 'free' | 'down' | 'up' | 'forward' (direction of the gripper axis); approach_vector
    [ax, ay, az] overrides it. linear=True also verifies a straight-line path.
    """
    return await _run(_arm.plan_xyz, x, y, z, _approach(approach, approach_vector), linear)


@mcp.tool()
async def arm_move_to_xyz(x: float, y: float, z: float, approach: str = "free",
                          approach_vector: list[float] | None = None, linear: bool = False,
                          speed_scale: float = C.DEFAULT_SPEED_SCALE) -> dict:
    """Move the TCP to (x, y, z) in metres (base frame) and wait until it arrives.

    approach: 'free' | 'down' | 'up' | 'forward' (or approach_vector). linear=True moves along a straight
    line (recommended near objects or the table). speed_scale is capped by the server (default cap 0.6).
    Refused with an error if unreachable, outside the workspace, below the table guard or in a fault state.
    """
    return await _run(_arm.move_to_xyz, x, y, z, _approach(approach, approach_vector), linear, speed_scale)


@mcp.tool()
async def arm_move_relative(dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, approach: str = "free",
                            approach_vector: list[float] | None = None, linear: bool = True,
                            speed_scale: float = C.DEFAULT_SPEED_SCALE) -> dict:
    """Move the TCP by (dx, dy, dz) metres relative to where it is now (straight line by default)."""
    return await _run(_arm.move_relative, dx, dy, dz, _approach(approach, approach_vector), linear, speed_scale)


@mcp.tool()
async def arm_move_joints(joints_deg: list[float], speed_scale: float = C.DEFAULT_SPEED_SCALE) -> dict:
    """Move all 6 joints to the given angles in degrees [joint1..joint6] (must be within the joint limits)."""
    return await _run(_arm.move_joints, joints_deg, speed_scale)


@mcp.tool()
async def arm_home(speed_scale: float = C.DEFAULT_SPEED_SCALE) -> dict:
    """Return to the calibrated zero (rest) pose: all joints at 0 degrees."""
    return await _run(_arm.home, speed_scale)


@mcp.tool()
async def arm_gripper(opening: float) -> dict:
    """Set the gripper opening: 0 = closed, 1 = fully open. Torque is limited; it holds objects gently."""
    return await _run(_arm.set_gripper, opening)


@mcp.tool()
def arm_stop() -> dict:
    """Stop the current motion immediately and hold the current position (torque stays on). Safe to call any time."""
    return _result(_arm.stop)


@mcp.tool()
def arm_emergency_disable() -> dict:
    """EMERGENCY ONLY: cut motor torque now. The arm will sag or fall under gravity. Needs a reconnect afterwards."""
    return _result(_arm.emergency_disable)


@mcp.tool()
async def arm_disconnect(go_home: bool = True) -> dict:
    """Finish: (by default) return to the rest pose, switch torque off and release the CAN bus."""
    return await _run(_arm.disconnect, go_home)


# --------------------------------------------------------------------------


def _on_exit() -> None:
    action = os.environ.get("REBOT_ON_EXIT", "home").strip().lower()
    if not _arm.connected or action == "leave":
        return
    try:
        _arm.stop()
        _arm.disconnect(go_home=(action == "home"))
    except Exception as e:  # pragma: no cover
        log.error("shutdown handling failed: %s", e)


def _start_viewer() -> None:
    global _viewer_url
    raw = os.environ.get("REBOT_VIEWER_PORT", "8765" if C.DRY_RUN else "off").strip().lower()
    if raw in ("", "off", "no", "false"):
        return
    try:
        _, _viewer_url = start_viewer(_arm, int(raw))
        log.info("3D twin: %s", _viewer_url)
    except (OSError, ValueError) as e:
        log.warning("viewer not started: %s", e)


def main() -> None:
    _start_viewer()
    atexit.register(_on_exit)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # run atexit on SIGTERM
    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
