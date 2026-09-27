"""The unload loop's vision on synthetic top-down RGB-D frames: the bins of the station, the
sock to take from the cargo box, the sock in the gripper."""

import math

import numpy as np
import pytest

from sorter.box_detector.cargo import find_sock
from sorter.box_detector.held import find_held
from sorter.box_detector.station import find_bin
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import ColorClass, Frame, Intrinsics, Observation, Zone
from sorter.sim.config import RectConfig

W, H, F = 640, 480, 615.0
K = Intrinsics(F, F, W / 2, H / 2, W, H)
FLOOR = -160.0


def top_down(center, cam_z, height, color=None, masks=()):
    """An observation from a camera looking straight down from (center, cam_z) (mm, arm frame)
    at the surface `height(x, y)` (mm, NaN = the floor). Image x = arm +x, image y = arm −y."""
    T = np.eye(4)
    T[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    T[:3, 3] = (*center, cam_z)
    v, u = np.mgrid[0:H, 0:W]
    far = cam_z - FLOOR
    z = np.full((H, W), far)
    hit = np.zeros((H, W), bool)
    for d in np.arange(far - 100.0, far + 0.5, 1.0):  # march each ray down to the surface
        x = center[0] + (u - K.cx) / F * d
        y = center[1] - (v - K.cy) / F * d
        new = ~hit & (cam_z - d <= FLOOR + np.nan_to_num(height(x, y), nan=0.0))
        z[new], hit = d, hit | new
    x = center[0] + (u - K.cx) / F * z
    y = center[1] - (v - K.cy) / F * z
    frame = Frame(
        color=np.zeros((H, W, 3), np.uint8) if color is None else color(x, y),
        depth_mm=np.rint(z).astype(np.uint16),
        intrinsics=K,
        timestamp=0.0,
        seq=0,
    )
    return Observation(frame, Zone.LAUNDRY, T, None), x, y


def ring(cx, cy, yaw, size=150.0, wall=5.0, height=60.0):
    def h(x, y):
        c, s = math.cos(-yaw), math.sin(-yaw)
        a, b = c * (x - cx) - s * (y - cy), s * (x - cx) + c * (y - cy)
        m = np.maximum(np.abs(a), np.abs(b))
        return np.where((m <= size / 2) & (m >= size / 2 - wall), height, np.nan)

    return h


@pytest.mark.parametrize(("dx", "dy", "deg"), [(0, 0, 0), (25, -18, 7), (-30, 20, -12)])
def test_finds_a_bin_off_its_place(dx, dy, deg):
    guess = (270.0, 0.0)
    real = (guess[0] + dx, guess[1] + dy)
    obs, _, _ = top_down(real, 240.0, ring(*real, math.radians(deg)))
    fit = find_bin(obs, guess, FLOOR, 150.0, 60.0, 5.0)
    assert fit.center is not None and fit.complete
    assert math.dist(fit.center, real) < 3.0
    assert math.degrees(fit.yaw) == pytest.approx(deg, abs=1.5)


def test_a_neighbor_bin_in_view_is_not_taken():
    guess = (270.0, 0.0)
    both = [ring(270.0, 10.0, 0.0), ring(270.0, 180.0, 0.0)]  # the neighbor 170 mm away

    def h(x, y):
        return np.fmax(both[0](x, y), both[1](x, y))

    obs, _, _ = top_down((270.0, 60.0), 300.0, h)
    fit = find_bin(obs, guess, FLOOR, 150.0, 60.0, 5.0)
    assert fit.center is not None and math.dist(fit.center, (270.0, 10.0)) < 3.0


def test_no_bin():
    obs, _, _ = top_down((270.0, 0.0), 240.0, lambda x, y: np.full(np.shape(x), np.nan))
    assert find_bin(obs, (270.0, 0.0), FLOOR, 150.0, 60.0).center is None


BOX = RectConfig(center_mm=(-130.0, 200.0), size_mm=(140.0, 140.0))
BOX_FLOOR = -150.0  # the box floor in this test's frame (the floor of `top_down` + 10)
WORKSPACE = [(-170.0, 160.0), (-90.0, 160.0), (-90.0, 240.0), (-170.0, 240.0)]
COLORS = {"dark": (40, 35, 30), "red": (40, 40, 200)}  # BGR


def blob(cx, cy, rx, ry, h):
    def f(x, y):
        d = ((x - cx) / rx) ** 2 + ((y - cy) / ry) ** 2
        return np.where(d < 1, 10 + h * (1 - d), np.nan)  # on the box floor (+10)

    return f


def pile():
    """A dark sock under a red one lying across it."""
    dark = blob(-140.0, 200.0, 55.0, 25.0, 12.0)
    red = blob(-120.0, 205.0, 25.0, 50.0, 24.0)

    def height(x, y):
        return np.fmax(dark(x, y), red(x, y))

    def color(x, y):
        img = np.full((H, W, 3), 120, np.uint8)
        d, r = ~np.isnan(dark(x, y)), ~np.isnan(red(x, y))
        img[d] = COLORS["dark"]
        img[r & (red(x, y) >= np.nan_to_num(dark(x, y), nan=-1))] = COLORS["red"]
        return img

    return height, color


def test_takes_the_sock_on_top_with_its_color():
    height, color = pile()
    obs, x, y = top_down(BOX.center_mm, 90.0, height, color)
    img = obs.frame.color

    def segment(_):
        red = (img == COLORS["red"]).all(axis=-1)
        dark = (img == COLORS["dark"]).all(axis=-1)
        return [Instance(dark, 1.0), Instance(red, 1.0)]

    view = find_sock(
        obs, BOX, BOX_FLOOR, BOX_FLOOR + 56, WORKSPACE, ColorClassifierConfig(), segment
    )
    t = view.target
    assert t is not None and t.color is ColorClass.COLORED and view.socks == 2
    assert math.dist((t.point.x, t.point.y), (-120.0, 205.0)) < 12
    # the red sock lies along y, the dark one under it along x: open across the red one, the
    # fingers would come down on the dark one, so they open along y
    assert abs(math.sin(t.yaws[0])) > 0.9
    assert abs(math.cos(t.yaws[-1])) > 0.9 or abs(math.cos(t.yaws[-2])) > 0.9


def test_an_empty_box():
    obs, _, _ = top_down(BOX.center_mm, 90.0, lambda x, y: np.full(np.shape(x), np.nan))
    view = find_sock(obs, BOX, BOX_FLOOR, BOX_FLOOR + 56, WORKSPACE, ColorClassifierConfig())
    assert view.empty and view.target is None


def test_the_sock_in_the_gripper():
    color = np.full((H, W, 3), 90, np.uint8)
    depth = np.full((H, W), 600, np.uint16)  # the floor, far away
    color[:150, 200:400] = (230, 235, 235)  # a light sock hanging at the top, too close to see
    depth[:150, 200:400] = 0
    color[300:400, 450:600] = (40, 40, 200)  # a red sock lying in a bin, 280 mm away
    depth[300:400, 450:600] = 280
    T = np.eye(4)
    T[:3, :3] = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    T[:3, 3] = (250.0, 0.0, 130.0)  # looking down from 290 mm over the floor
    obs = Observation(Frame(color, depth, K, 0.0, 0), Zone.LAUNDRY, T, None)
    sock = np.zeros((H, W), bool)
    sock[:150, 200:400] = True
    in_bin = np.zeros((H, W), bool)
    in_bin[300:400, 450:600] = True
    cfg = ColorClassifierConfig()
    held = find_held(obs, lambda _: [Instance(sock, 1.0), Instance(in_bin, 1.0)], cfg, FLOOR)
    assert held.color is ColorClass.LIGHT and held.count == 1
    assert find_held(obs, lambda _: [Instance(in_bin, 1.0)], cfg, FLOOR).color is None


def test_the_sock_gone_from_the_box():
    from sorter.box_detector.cargo import CargoView, SockSeen, taken
    from sorter.core.types import Overlay

    red = SockSeen(ColorClass.COLORED, 1.0, (45.0, 60.0, 40.0), (-120.0, 205.0), 3000)
    dark = SockSeen(ColorClass.DARK, 1.0, (15.0, 1.0, 2.0), (-140.0, 200.0), 2000)
    white = SockSeen(ColorClass.LIGHT, 1.0, (92.0, 0.0, 3.0), (-150.0, 180.0), 2500)
    before = CargoView(None, 9000, [red, dark, white], Overlay())
    # the dark one shows more of itself once the red one is gone, and moved a bit
    dark_after = SockSeen(ColorClass.DARK, 1.0, (16.0, 1.5, 2.0), (-132.0, 203.0), 3800)
    after = CargoView(None, 7000, [dark_after, white], Overlay())
    assert taken(before, after) == [red]
    assert taken(before, CargoView(None, 9000, [red, dark, white], Overlay())) == []


def test_a_sock_shown_more_once_the_one_on_it_is_gone_is_still_there():
    from sorter.box_detector.cargo import CargoView, SockSeen, taken
    from sorter.core.types import Overlay

    def mask(u0, u1):
        m = np.zeros((H, W), bool)
        m[200:260, u0:u1] = True
        return m

    top = SockSeen(ColorClass.DARK, 1.0, (18.0, 0.0, 0.0), (-110.0, 200.0), 6000, mask(300, 400))
    under = SockSeen(ColorClass.DARK, 1.0, (20.0, 0.0, 0.0), (-170.0, 200.0), 3000, mask(150, 200))
    # the top one taken: the one under it shows all of itself, its centroid 90 mm further
    shown = SockSeen(ColorClass.DARK, 1.0, (20.0, 0.0, 0.0), (-80.0, 200.0), 15000, mask(150, 400))
    before = CargoView(None, 9000, [top, under], Overlay())
    assert taken(before, CargoView(None, 15000, [shown], Overlay())) == [top]


def test_a_hanging_sock_is_told_by_its_side_lit_color():
    from sorter.box_detector.held import side_class

    assert side_class({"L": 19.0, "chroma": 38.0}) is ColorClass.COLORED
    assert side_class({"L": 40.0, "chroma": 1.0}) is ColorClass.LIGHT
    assert side_class({"L": 6.0, "chroma": 2.0}) is ColorClass.DARK
    assert side_class({"L": 23.0, "chroma": 3.0}) is None
