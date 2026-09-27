"""The sock hunt on the nav sim: every sock it claims is in the pick zone, and a different one."""

from sorter.nav import episode
from sorter.nav.hunt import hunt_socks


def test_hunt_reaches_different_socks_in_turn(tmp_path, monkeypatch):
    eps = []

    class Recorded(episode.Episode):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            eps.append(self)

    monkeypatch.setattr(episode, "Episode", Recorded)
    truth = []

    def on_reached(r):
        s = eps[0].score()  # the ground truth: the sock nearest the zone's center
        truth.append((s.sock, s.in_zone, s.sock_pushed_m, s.collisions))

    report = hunt_socks(
        max_socks=2,
        detector="seg",
        scenario="multi",
        seed=0,
        config_dir=tmp_path,  # no repo config: the defaults
        on_reached=on_reached,
        verbose=False,
    )
    assert report.stopped == "done" and len(report.reached) == 2, report.summary()
    assert all(in_zone and pushed < 0.05 and coll == 0 for _, in_zone, pushed, coll in truth)
    assert truth[0][0] != truth[1][0], truth


def _true_gap(ep) -> float:
    """The front bumper to the nearest point of a sock mesh in front of it (m), ground truth."""
    import math

    import mujoco
    import numpy as np

    from sorter.nav.commands import FRONT_M, HALF_WIDTH_M

    m, d = ep.sim.model, ep.sim.data
    x, y, yaw = ep.sim.true_pose()
    c, s = math.cos(yaw), math.sin(yaw)
    best = math.inf
    bodies = {ep.sim._sock_bodies[i] for i in range(len(ep.spec.socks))}
    for g in range(m.ngeom):
        if m.geom_bodyid[g] not in bodies or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        a, n = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
        v = m.mesh_vert[a : a + n] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        rx = c * (v[:, 0] - x) + s * (v[:, 1] - y)
        ry = -s * (v[:, 0] - x) + c * (v[:, 1] - y)
        ahead = rx[(np.abs(ry) < HALF_WIDTH_M) & (rx > 0)]
        if ahead.size:
            best = min(best, float(ahead.min()) - FRONT_M)
    return best


def test_fast_approach_stops_a_few_cm_before_the_sock():
    from sorter.nav.config import NavConfig
    from sorter.nav.detect import ClassicDetector
    from sorter.nav.hunt import approach_sock
    from sorter.nav.scenario import make

    ep = episode.Episode(make("side", 2), NavConfig())
    r = approach_sock(ep.rover, ClassicDetector(), gap_m=0.05)
    s = ep.score()
    assert r.ok, r.note
    assert 0.02 < _true_gap(ep) < 0.09, _true_gap(ep)
    assert s.sock_pushed_m < 0.01 and s.collisions == 0, s
    ep.close()


def test_run_robot_from_the_rover_tab():
    import time

    from sorter.nav.config import NavConfig
    from sorter.nav.server import NavLive, RunRequest

    live = NavLive(NavConfig().model_copy(update={"realtime": 0.0}))
    try:
        for _ in range(200):
            if live.episode is not None and live.busy is None:
                break
            time.sleep(0.05)
        live.submit("run", RunRequest(detector="seg", gap_m=0.05))
        time.sleep(0.2)
        for _ in range(3600):  # up to 3 min: slow on a loaded machine (the full suite)
            if live.busy is None:
                break
            time.sleep(0.05)
        st = live.state()
        assert st["error"] is None, st["error"]
        assert st["log"][-1]["command"] == "RUN ROBOT" and st["log"][-1]["ok"], st["log"][-1]
    finally:
        live.stop()
        time.sleep(0.5)  # the panel thread renders once more after a job
        if live.episode is not None:
            live.episode.close()
