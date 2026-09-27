"""The mission's hand-over and driving, without the arm's cloth physics."""

import dataclasses
import math

from sorter.core.config import load_config
from sorter.core.types import ColorClass
from sorter.mission.mission import color_class, rover_to_arm_mm, run_mission
from sorter.nav.boxes import detect_tags
from sorter.nav.config import NavConfig
from sorter.nav.episode import Episode
from sorter.nav.scenario import SOCK_COLORS, make


def test_color_class():
    assert color_class(SOCK_COLORS["white"]) is ColorClass.LIGHT
    assert color_class(SOCK_COLORS["black"]) is ColorClass.DARK
    assert color_class(SOCK_COLORS["navy"]) is ColorClass.DARK
    assert color_class(SOCK_COLORS["red"]) is ColorClass.COLORED
    assert color_class(SOCK_COLORS["yellow"]) is ColorClass.COLORED


def test_nav_goal_is_in_the_arm_floor_zone():
    """The approach's goal zone (Leo frame) lands in the arm's floor zone (rig.yaml)."""
    from sorter.arm.controller import in_polygon
    from sorter.core.types import Zone

    cfg = load_config()
    zone = [tuple(p) for p in cfg.zones[Zone.FLOOR].workspace_mm]
    (gx, gy), (hx, hy) = cfg.nav.goal.center_m, cfg.nav.goal.half_size_m
    for x in (gx - hx, gx, gx + hx):
        for y in (gy - hy, gy, gy + hy):
            assert in_polygon(*rover_to_arm_mm(cfg, x, y), zone), (x, y)


def test_station_tags_seen_in_order():
    spec = make("mission", 0)
    sx, sy, _ = spec.station
    cfg = NavConfig()
    ep = Episode(dataclasses.replace(spec, rover=(sx + 1.2, sy, math.pi)), cfg)
    try:
        tags = detect_tags(ep.rover.run("look").frame, cfg.boxes.station_tag_m)
    finally:
        ep.close()
    by_bearing = [t.id for t in sorted(tags, key=lambda t: -t.bearing_deg)]  # left to right
    assert by_bearing == [14, 13, 12]
    assert abs(next(t for t in tags if t.id == 13).distance - 1.2) < 0.1


def test_mission_without_arm_reaches_the_station(tmp_path):
    report = run_mission(seed=1, arm=False, out=tmp_path)
    assert report.loaded == report.socks
    assert report.station["ok"]
    assert abs(report.station["true_gap_m"] - 0.15) < 0.05
    assert (tmp_path / "mission.json").exists()
