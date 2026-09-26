import json
import math

import anyio
import numpy as np
import pytest
from mcp.shared.memory import create_connected_server_and_client_session

from sorter.arm.backend import create_mock
from sorter.arm.config import ArmConfig
from sorter.arm.mcp_server import ArmTools, build_server
from sorter.core.config import Config

POSES = {
    "rest": [0.0, -1.5, 1.5, 1.0, 0.0],
    "home": [0.0, -0.3, 0.6, 1.2, 0.0],
    "place_bg": [-0.5, 0.0, 0.5, 1.0, 0.0],
}


def _tools(on_sleep=None):
    cfg = Config(arm=ArmConfig(settle_s=0.05, table_z_mm=-20), poses=POSES)

    def sleep(_s):
        if on_sleep is not None:
            on_sleep()

    arms = []

    def make(sim):
        assert sim
        arms.append(create_mock(cfg, sleep=sleep))
        return arms[-1]

    return ArmTools(cfg, make, simulate=True), arms


def _connected(on_sleep=None):
    tools, arms = _tools(on_sleep)
    assert tools.connect()["ok"]
    return tools, arms[0]


def test_tools_need_a_connection():
    tools, _ = _tools()
    assert tools.info()["connected"] is False
    r = tools.move_to_xyz(200, 0, 30)
    assert not r["ok"] and "arm_connect" in r["error"]


def test_connect_starts_at_rest_with_the_motors_on():
    tools, arm = _connected()
    s = tools.status()
    assert s["ok"] and s["simulated"] and not s["held"]
    assert list(s["joints_deg"].values()) == pytest.approx(np.degrees(POSES["rest"]), abs=0.2)
    assert all(arm.bus.torque.values())


def test_goto_pose_and_unknown_pose():
    tools, arm = _connected()
    r = tools.goto_pose("home")
    assert r["ok"] and r["state"]["at_pose"] == "home"
    assert arm.read_q() == pytest.approx(POSES["home"], abs=3e-3)
    r = tools.goto_pose("bin_light")
    assert not r["ok"] and "unknown pose" in r["error"]


@pytest.mark.parametrize("linear", [False, True])
def test_move_to_xyz_reaches_the_point_with_the_tool_down(linear):
    tools, _ = _connected()
    tools.goto_pose("home")
    plan = tools.plan_xyz(200, 0, 40, linear=linear)
    assert plan["ok"] and plan["reachable"]
    r = tools.move_to_xyz(200, 0, 40, linear=linear, speed_scale=0.8)
    assert r["ok"], r
    assert r["state"]["tip_mm"] == pytest.approx([200, 0, 40], abs=3)
    assert r["state"]["tilt_from_down_deg"] <= 25


def test_straight_move_needs_the_tool_down_at_the_start():
    tools, _ = _connected()
    tools.move_joints([0, -10, -50, 100, 94])  # the rig's home: tool 50° from down
    r = tools.move_relative(dz=-10)
    assert not r["ok"] and "linear=false first" in r["error"]


def test_move_relative_goes_along_a_line():
    tools, _ = _connected()
    tools.move_to_xyz(200, 0, 40)
    r = tools.move_relative(dy=30, dz=20)
    assert r["ok"], r
    assert r["state"]["tip_mm"] == pytest.approx([200, 30, 60], abs=3)


@pytest.mark.parametrize(
    "xyz, msg",
    [((200, 0, -40), "below the table"), ((500, 0, 50), "from the base axis"), ((0, 0, 60), "")],
)
def test_bad_targets_are_refused_without_motion(xyz, msg):
    tools, arm = _connected()
    n = len(arm.bus.goal_log)
    for fn in (tools.plan_xyz, tools.move_to_xyz):
        r = fn(*xyz)
        assert not r["ok"] and msg in r["error"]
    assert len(arm.bus.goal_log) == n


def test_move_joints_checks_limits_and_count():
    tools, arm = _connected()
    n = len(arm.bus.goal_log)
    assert "need 5" in tools.move_joints([0, 0, 0])["error"]
    assert "shoulder_pan" in tools.move_joints([170, 0, 0, 0, 0])["error"]
    assert len(arm.bus.goal_log) == n
    r = tools.move_joints([10, -20, 30, 60, 5])
    assert r["ok"], r
    assert arm.read_q() == pytest.approx(np.radians([10, -20, 30, 60, 5]), abs=3e-3)


def test_gripper_reports_the_measured_opening():
    tools, arm = _connected()
    assert tools.gripper(1.0)["measured_opening"] == pytest.approx(1.0, abs=0.01)
    arm.bus.blocked[6] = 1400  # cloth between the fingers
    r = tools.gripper(0.0)
    assert r["ok"] and 0.1 < r["measured_opening"] < 0.3
    assert not tools.gripper(1.5)["ok"]


def test_stop_interrupts_a_motion_and_resume_clears_it():
    ticks = 0

    def stop_mid_motion():  # "another thread" pressing stop during the 5th control tick
        nonlocal ticks
        ticks += 1
        if ticks == 5:
            tools.stop()

    tools, arm = _connected(stop_mid_motion)
    r = tools.goto_pose("home")
    assert not r["ok"] and "arm_resume" in r["error"]
    assert r["state"]["held"]
    q_held = arm.read_q()
    assert not tools.move_joints([0, 0, 0, 0, 0])["ok"]
    assert arm.read_q() == pytest.approx(q_held, abs=3e-3)  # didn't move while held
    assert tools.resume()["ok"]
    assert tools.goto_pose("home")["ok"]


def test_disconnect_goes_to_rest_and_turns_the_motors_off():
    tools, arm = _connected()
    tools.goto_pose("home")
    bus = arm.bus
    r = tools.disconnect()
    assert r["ok"] and r["motors"] == "off at rest"
    assert not any(bus.torque.values()) and bus.closed
    q = arm.q_from_raw([bus.pos[sid] for sid in arm.cfg.ids])
    assert q == pytest.approx(POSES["rest"], abs=3e-3)
    assert not tools.status()["ok"]
    assert tools.connect()["ok"]  # a fresh arm


def test_mcp_protocol_lists_and_calls_the_tools():
    tools, _ = _tools()
    server = build_server(tools)

    async def run():
        async with create_connected_server_and_client_session(server._mcp_server) as client:
            names = {t.name for t in (await client.list_tools()).tools}
            out = []
            for name, args in [
                ("arm_connect", {}),
                ("arm_goto_pose", {"name": "home"}),
                ("arm_move_to_xyz", {"x": 200, "y": 0, "z": 40, "linear": True}),
                ("arm_status", {}),
            ]:
                res = await client.call_tool(name, args)
                out.append(json.loads(res.content[0].text))
            return names, out

    names, out = anyio.run(run)
    assert {
        "arm_info",
        "arm_connect",
        "arm_status",
        "arm_plan_xyz",
        "arm_move_to_xyz",
        "arm_move_relative",
        "arm_move_joints",
        "arm_goto_pose",
        "arm_gripper",
        "arm_stop",
        "arm_resume",
        "arm_disconnect",
    } <= names
    assert all(o["ok"] for o in out), out
    assert out[-1]["tip_mm"] == pytest.approx([200, 0, 40], abs=3)
    assert math.isclose(out[-1]["joints_deg"]["wrist_roll"], 0, abs_tol=0.5)
