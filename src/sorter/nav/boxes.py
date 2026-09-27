"""Cardboard boxes marked with AprilTags: find them, remember their layout, drive up to one.

Our boxes carry tags of the AprilTag 36h11 family (ids 14, 13, 12 left to right; 13 is the
target). The tags are small (`nav.boxes.tag_size_m`, 3.5 cm): ~19 px across at 1 m.

- `detect_tags(frame, size)`: OpenCV's ArUco module (DICT_APRILTAG_36h11) on the RGB, again on
  a 2x upscale when nothing is found (far tags). Each tag's pose by `solvePnP` (IPPE_SQUARE)
  from its corners and size: its center and its face's normal in the rover frame. No depth.
- `remember_boxes(frame, ...)`: the tags in view and each one's offset from the target tag,
  in the target's own frame (along its face, out of it), saved to `nav.boxes.memory`. Later,
  seeing any remembered tag gives where the target is, even if it is hidden or out of view.
- `approach_box(rover, target, stop_m)`: look; nothing known in view: turn in steps until a
  remembered tag shows up; far and seen from the side: go to a point in front of the box's
  face first; then turn to the tag and drive in steps, measuring again from the camera after
  each, until the front bumper is `stop_m` from the tag. The tag stays in view to the end.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from sorter.nav.camera import Frame
from sorter.nav.commands import FRONT_M, Rover

SEARCH_STEP_DEG = 40.0  # the camera sees ~69 deg wide: steps overlap
LEG_M = 0.5  # farther than this from the stop: drive most of the way, then measure again
LEG_FRACTION = 0.75
TOL_M = 0.025  # close enough to the stop
FACE_M = 0.55  # the point in front of the box's face to line up at (tag to bumper)
FACE_ANGLE_DEG = 25.0  # seen more obliquely than this and farther than FACE_M + 0.3: line up
MIN_NORMAL_PX = 28.0  # a tag's own normal is trusted when it is at least this big
MAX_MOVES = 20
FINAL_SPEED = 0.15

_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
_DETECTOR = cv2.aruco.ArucoDetector(_DICT, cv2.aruco.DetectorParameters())


@dataclass
class Tag:
    id: int
    corners: np.ndarray  # 4x2 px: top left, top right, bottom right, bottom left
    x: float  # the tag's center, rover frame (m)
    y: float
    z: float
    nx: float  # its face's normal, horizontal unit vector toward the viewer, rover frame
    ny: float

    @property
    def u(self) -> float:
        return float(self.corners[:, 0].mean())

    @property
    def v(self) -> float:
        return float(self.corners[:, 1].mean())

    @property
    def side_px(self) -> float:
        c = self.corners
        return float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))

    @property
    def distance(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.y, self.x))

    def summary(self) -> dict:
        return {
            "id": self.id,
            "u": round(self.u),
            "v": round(self.v),
            "side_px": round(self.side_px, 1),
            "x_m": round(self.x, 3),
            "y_m": round(self.y, 3),
            "distance_m": round(self.distance, 3),
            "bearing_deg": round(self.bearing_deg, 1),
            "normal_deg": round(math.degrees(math.atan2(self.ny, self.nx)), 1),
        }


def detect_tags(frame: Frame, size_m: float) -> list[Tag]:
    """The AprilTags (36h11) in a frame with their poses, nearest first."""
    gray = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    scale = 1.0
    if ids is None:  # small far tags: once more on a 2x upscale
        scale = 2.0
        big = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = _DETECTOR.detectMarkers(big)
    if ids is None:
        return []
    K, T = frame.K, frame.T_rover_cam
    Kc = np.array([[K.fx, 0, K.cx], [0, K.fy, K.cy], [0, 0, 1]], np.float64)
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    out = []
    for c, i in zip(corners, ids.ravel(), strict=True):
        px = c[0].astype(np.float64) / scale
        ok, rvec, tvec = cv2.solvePnP(obj, px, Kc, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        p = T[:3, :3] @ tvec.ravel() + T[:3, 3]
        n = T[:3, :3] @ R[:, 2]  # the tag's z axis: out of its face
        nx, ny = float(n[0]), float(n[1])
        norm = math.hypot(nx, ny) or 1.0
        nx, ny = nx / norm, ny / norm
        if nx * (T[0, 3] - p[0]) + ny * (T[1, 3] - p[1]) < 0:  # make it face the camera
            nx, ny = -nx, -ny
        out.append(Tag(int(i), px, float(p[0]), float(p[1]), float(p[2]), nx, ny))
    return sorted(out, key=lambda t: t.distance)


def face_normal(tags: list[Tag]) -> tuple[float, float]:
    """The boxes' common face normal (toward the rover): from the line through the tags when
    several are in view (the boxes stand in a row), else the nearest tag's own."""
    if len(tags) >= 2:
        xy = np.array([[t.x, t.y] for t in tags])
        c = xy.mean(0)
        _, _, vt = np.linalg.svd(xy - c)
        ax, ay = vt[0]  # along the row
        nx, ny = -ay, ax
        if nx * -c[0] + ny * -c[1] < 0:
            nx, ny = -nx, -ny
        return float(nx), float(ny)
    return tags[0].nx, tags[0].ny


# --- memory ---


def remember_boxes(frame: Frame, size_m: float, target: int, path: str | Path) -> dict:
    """Save the tags in view and their offsets from `target` (which must be in view), in the
    target's frame: `along_m` to the left along the face (as seen from the rover), `out_m`
    out of it toward the rover."""
    tags = detect_tags(frame, size_m)
    tgt = next((t for t in tags if t.id == target), None)
    if tgt is None:
        seen = [t.id for t in tags]
        raise ValueError(f"target tag {target} is not in view (seen: {seen})")
    nx, ny = face_normal(tags)
    ax, ay = ny, -nx  # along the face, to the left as seen from the rover
    entries = {}
    for t in tags:
        dx, dy = t.x - tgt.x, t.y - tgt.y
        entries[str(t.id)] = {
            "along_m": round(dx * ax + dy * ay, 3),
            "out_m": round(dx * nx + dy * ny, 3),
        }
    mem = {
        "family": "tag36h11",
        "tag_size_m": size_m,
        "target": target,
        "tags": entries,
        "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
        "seen_from": {
            "distance_m": round(tgt.distance, 3),
            "bearing_deg": round(tgt.bearing_deg, 1),
        },
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(mem, indent=1))
    cv2.imwrite(
        str(path.with_suffix(".png")), cv2.cvtColor(overlay(frame, tags), cv2.COLOR_RGB2BGR)
    )
    return mem


def load_memory(path: str | Path) -> dict | None:
    p = Path(path)
    return json.loads(p.read_text()) if p.is_file() else None


def locate_target(tags: list[Tag], target: int, memory: dict | None) -> tuple[Tag | None, str]:
    """The target tag: seen, or placed from a remembered neighbor in view. (tag, how)."""
    for t in tags:
        if t.id == target:
            return t, "seen"
    if not memory or str(target) not in memory.get("tags", {}):
        return None, ""
    known = [t for t in tags if str(t.id) in memory["tags"]]
    if not known:
        return None, ""
    nx, ny = face_normal(known)
    ax, ay = ny, -nx
    tm = memory["tags"][str(target)]
    xs, ys = [], []
    for t in known:  # the target from each neighbor, averaged
        m = memory["tags"][str(t.id)]
        da, do = tm["along_m"] - m["along_m"], tm["out_m"] - m["out_m"]
        xs.append(t.x + da * ax + do * nx)
        ys.append(t.y + da * ay + do * ny)
    ref = known[0]
    x, y = float(np.mean(xs)), float(np.mean(ys))
    how = f"from tag {', '.join(str(t.id) for t in known)} (remembered layout)"
    return Tag(target, ref.corners, x, y, ref.z, nx, ny), how


# --- driving ---


@dataclass
class BoxResult:
    ok: bool
    target: dict | None  # the target as last measured, before the last move
    gap_m: float | None  # front bumper to the tag at the end, measured from the camera
    commands: int
    note: str


def approach_box(
    rover: Rover,
    target: int = 13,
    stop_m: float = 0.15,
    size_m: float = 0.035,
    memory: dict | None = None,
    speed: float = 0.4,
    max_moves: int = MAX_MOVES,
) -> BoxResult:
    """Drive until the front bumper is `stop_m` from tag `target` (see the module doc)."""
    n = 0

    def do(name, *a):
        nonlocal n
        n += 1
        return rover.run(name, *a)

    res = do("look")
    searched = 0.0
    relooked = lined_up = False
    last = None
    while n < max_moves:
        tags = detect_tags(res.frame, size_m)
        tgt, how = locate_target(tags, target, memory)
        if tgt is None:
            if not relooked:  # the RGB's autofocus may still be hunting: one more look first
                relooked = True
                time.sleep(0.5)
                res = do("look")
                continue
            relooked = False
            if searched >= 360.0:
                seen = sorted({t.id for t in tags})
                return BoxResult(
                    False, None, None, n, f"tag {target} not found all round (seen {seen})"
                )
            searched += SEARCH_STEP_DEG
            res = do("turn", SEARCH_STEP_DEG, 1.0)
            continue
        searched, relooked = 0.0, False
        last = tgt.summary() | {"how": how}
        gap = tgt.distance - FRONT_M  # bumper to the tag, along the line of sight
        rest = gap - stop_m
        # seen from the side and still far: line up in front of the box's face first
        nx, ny = face_normal(tags) if how == "seen" else (tgt.nx, tgt.ny)
        off = abs(math.degrees(math.atan2(-tgt.y * nx + tgt.x * ny, -(tgt.x * nx + tgt.y * ny))))
        trusted = how != "seen" or len(tags) >= 2 or tgt.side_px >= MIN_NORMAL_PX
        if not lined_up and trusted and off > FACE_ANGLE_DEG and gap > FACE_M + 0.3:
            lined_up = True
            d = FACE_M + FRONT_M  # the rover's center this far out of the face
            px, py = tgt.x + nx * d, tgt.y + ny * d
            do("turn", math.degrees(math.atan2(py, px)), 1.0)
            do("forward", math.hypot(px, py), speed)
            # face the tag again: it was at (tgt.x, tgt.y) seen from the old pose
            res = do("turn", _face_after(px, py, tgt.x, tgt.y), 1.0)
            continue
        if abs(tgt.bearing_deg) > 2.0 and rest > TOL_M:
            res = do("turn", tgt.bearing_deg, 1.0)
            continue
        if abs(rest) <= TOL_M:
            return BoxResult(True, last, gap, n, f"stopped {gap:.2f} m from tag {target} ({how})")
        if rest < 0:  # too close: back up
            res = do("forward", rest, FINAL_SPEED)
            continue
        if rest > LEG_M:
            res = do("forward", rest * LEG_FRACTION, speed)
            continue
        # the last bit: slowly, the obstacle guard just short of the stop (the box is in front)
        res = do("forward", rest, FINAL_SPEED, max(0.04, stop_m - 0.08))
    return BoxResult(False, last, None, n, f"no stop in {max_moves} moves")


def _face_after(px: float, py: float, tx: float, ty: float) -> float:
    """After driving from the origin (heading 0) straight to (px, py), the turn (deg) that
    faces (tx, ty) (both in the old rover frame)."""
    heading = math.atan2(py, px)
    want = math.atan2(ty - py, tx - px)
    return math.degrees((want - heading + math.pi) % (2 * math.pi) - math.pi)


def overlay(frame: Frame, tags: list[Tag]) -> np.ndarray:
    img = frame.rgb.copy()
    for t in tags:
        cv2.polylines(img, [t.corners.astype(np.int32)], True, (0, 255, 120), 2, cv2.LINE_AA)
        cv2.putText(
            img,
            f"{t.id} {t.distance:.2f}m",
            (int(t.u) - 20, int(t.corners[:, 1].min()) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 120),
            1,
            cv2.LINE_AA,
        )
    return img
