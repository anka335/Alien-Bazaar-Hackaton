"""The operator-facing parts of the nav command set."""

from pathlib import Path

import numpy as np

from sorter.nav.camera import Frame, Intrinsics
from sorter.nav.commands import snap_to_blob


def _frame(rgb: np.ndarray) -> Frame:
    h, w = rgb.shape[:2]
    K = Intrinsics(460.0, 460.0, w / 2, h / 2, w, h)
    return Frame(0, 0.0, rgb, np.zeros((h, w), np.uint16), K, np.eye(4))


def test_snap_moves_a_rough_click_to_the_blob_middle():
    rgb = np.full((480, 640, 3), 90, np.uint8)
    rgb[80:96, 340:376] = (230, 200, 20)  # a small yellow sock far away
    c = snap_to_blob(_frame(rgb), 380, 90, dist_m=2.0)
    assert c is not None and abs(c[0] - 357) <= 1 and abs(c[1] - 87) <= 1


def test_snap_on_bare_floor_finds_nothing():
    rgb = np.full((480, 640, 3), 90, np.uint8)
    assert snap_to_blob(_frame(rgb), 320, 240, dist_m=1.0) is None


def test_far_go_to_pixel_reaims_and_reports(tmp_path: Path):
    from sorter.nav.episode import Episode

    ep, res = Episode.new(tmp_path / "ep", "random", 1)
    assert res.view["zone_px"] is not None and "pose" in res.view
    # the sock is ~2.1 m away; the click is on its right tip
    res = ep.rover.go_to_pixel(380, 90)
    assert "re-aimed" in res.note and res.target["in_zone"]
    assert ep.score().success
    ep.close()


def test_seek_turns_until_it_faces_the_sock():
    from sorter.nav.config import NavConfig
    from sorter.nav.detect import SegDetector
    from sorter.nav.episode import Episode
    from sorter.nav.scenario import make

    ep = Episode(make("behind", 0), NavConfig())
    ep.rover.detector = SegDetector(ep.camera)
    res = ep.rover.seek()
    assert res.ok, res.note
    sx, sy = ep.sim.sock_in_rover(0)
    assert abs(np.degrees(np.arctan2(sy, sx))) < 12  # facing it
    ep.close()


def test_clearance_sees_the_wall_and_open_floor():
    from sorter.nav.config import NavConfig
    from sorter.nav.episode import Episode
    from sorter.nav.scenario import make

    ep = Episode(make("easy", 0), NavConfig())
    res = ep.rover.clearance()
    assert res.clear and all(m >= 0 for _, m in res.clear)
    assert "ahead" in res.note
    ep.close()


def test_nudge_takes_a_step_size():
    from sorter.nav.config import NavConfig
    from sorter.nav.episode import Episode
    from sorter.nav.scenario import make

    ep = Episode(make("easy", 0), NavConfig())
    res = ep.rover.nudge("forward", 0.12)
    assert abs(res.moved_m - 0.12) < 0.01
    res = ep.rover.nudge("right", 15)
    assert abs(res.turned_deg + 15) < 1.5
    ep.close()


def test_the_panel_lists_every_command_with_its_parameters():
    from sorter.nav.commands import Rover
    from sorter.nav.server import command_specs

    specs = {c["name"]: c for c in command_specs()}
    assert set(specs) == set(Rover.COMMANDS)
    gp = {p["name"]: p for p in specs["go_to_pixel"]["params"]}
    assert gp["u"]["required"] and gp["snap"]["type"] == "bool" and gp["far_m"]["default"] == 1.5
    assert specs["nudge"]["params"][0]["type"] == "str"
