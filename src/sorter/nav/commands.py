"""The navigation commands: what an operator (a person, an agent, the approach algorithm) uses to
steer the rover, deciding from the OAK-D's frames only.

Every command runs to completion, then stops the rover and returns a `Result` with a fresh
frame. Motions are closed-loop on the odometry (`merged_odom`), with the Leo's limits. Moving
forward, the depth image is checked a few times a second for something in the rover's path
(taller than a sock, closer than `GUARD_M`): the rover stops there. That is the only protection:
the real Leo has no bumper.

Pixel commands (`face`, `go_to_pixel`) take a pixel of the last frame (u right, v down, origin
top left, 640x480): they work out the target's direction and distance from the rover's center,
from that frame's depth, or from where the pixel's ray meets the floor when it has no depth.
Right after a `scan` they also take a pixel of any scan frame: pass `scan_heading_deg=` the
heading printed on that frame (the scan ends facing the way it started).

Every result also carries a compact view of the new frame (`view`): the rover's odometry pose
since the episode start, the goal zone's pixel box, the floor distance of a few image rows and
the free path ahead, so an operator can judge distances by eye without extra queries.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import cv2
import numpy as np

from sorter.nav.camera import Frame, OakD
from sorter.nav.sim import RoverSim

GUARD_M = 0.22  # stop when something taller than a sock is this close in front of the bumper
FRONT_M = 0.215  # the front wheels' edge, from the rover's center
HALF_WIDTH_M = 0.225
GUARD_HZ = 5.0
SOCK_MAX_H = 0.06  # anything taller is an obstacle to the guard
DIST_TOL_M = 0.005
ANGLE_TOL = math.radians(0.7)
NUDGE_M = 0.05
NUDGE_DEG = 5.0
CLEAR_MARGIN_M = 0.05  # clearance: each side of the rover
CLEAR_HALF_FOV_DEG = 30.0  # the bearings clearance reports (the camera sees about ±34°)
CLEAR_MAX_M = 3.0
REAIM_M = 0.9  # far targets: stop this far before the spot and find it again
REAIM_MIN_SCORE = 0.55  # template match needed to trust the re-found spot
VIEW_ROWS = (
    60,
    80,
    100,
    120,
    160,
    240,
    320,
    400,
    460,
)  # image rows whose floor distance `view` reports


@dataclass
class Result:
    command: str
    args: dict
    ok: bool = True
    moved_m: float = 0.0  # odometry, signed along the rover's heading
    turned_deg: float = 0.0  # odometry, left positive
    elapsed_s: float = 0.0  # simulated
    blocked: str | None = None  # why the motion stopped early
    note: str = ""
    frame: Frame | None = None
    frames: list[tuple[float, Frame]] = field(default_factory=list)  # scan: (heading deg, frame)
    target: dict | None = None  # pixel commands: where the aimed floor spot should be now
    view: dict | None = None  # compact observation of `frame` (see `Rover._view`)
    clear: list | None = None  # clearance: [(bearing deg, free m from the bumper), ...]

    def summary(self) -> dict:
        out = {
            "command": self.command,
            "args": self.args,
            "ok": self.ok,
            "moved_m": round(self.moved_m, 3),
            "turned_deg": round(self.turned_deg, 1),
            "elapsed_s": round(self.elapsed_s, 2),
            "blocked": self.blocked,
            "note": self.note,
        }
        if self.target is not None:
            out["target"] = self.target
        if self.view is not None:
            out["view"] = self.view
        if self.clear is not None:
            out["clear"] = [[b, round(m, 2)] for b, m in self.clear]
        return out


class Cancelled(Exception):
    """The operator stopped the command (live panel)."""


class Rover:
    """The command set on a simulated rover and its camera.

    `on_tick(sim)` runs after every control period (the live panel paces and streams there);
    `cancelled()` returning True stops the current command.
    """

    COMMANDS = (
        "forward",
        "turn",
        "face",
        "go_to_pixel",
        "scan",
        "seek",
        "clearance",
        "nudge",
        "look",
        "stop",
        "set_velocity",
    )

    def __init__(
        self,
        sim: RoverSim,
        camera: OakD,
        on_tick: Callable[[RoverSim], None] | None = None,
        cancelled: Callable[[], bool] | None = None,
    ):
        self.sim, self.camera = sim, camera
        self.on_tick = on_tick
        self.cancelled = cancelled or (lambda: False)
        self.frame: Frame | None = None  # the last frame an operator saw
        self.detector = None  # what `seek` looks for socks with (default: classic)

    # --- the commands ---

    def look(self) -> Result:
        """No motion: a fresh frame."""
        return self._finish(Result("look", {}), self._mark())

    def forward(self, distance_m: float, speed: float = 0.2) -> Result:
        """Drive straight `distance_m` (negative = back up) at up to `speed` m/s (max 0.4),
        holding the heading. Stops early for an obstacle ahead (depth guard)."""
        res = Result("forward", {"distance_m": distance_m, "speed": speed})
        mark = self._mark()
        self._straight(distance_m, speed, res)
        return self._finish(res, mark)

    def turn(self, angle_deg: float, speed: float = 0.6) -> Result:
        """Turn in place by `angle_deg` (left positive) at up to `speed` rad/s (max 1.0)."""
        res = Result("turn", {"angle_deg": angle_deg, "speed": speed})
        mark = self._mark()
        self._rotate(math.radians(angle_deg), speed, res)
        return self._finish(res, mark)

    def face(self, u: float, v: float | None = None, scan_heading_deg: float = 0.0) -> Result:
        """Turn in place until the floor spot at pixel (u, v) is straight ahead of the rover's
        center (v defaults to the image middle row). `scan_heading_deg`: the pixel is from the
        scan frame with that heading (only right after `scan`, before any other motion)."""
        res = Result("face", {"u": u, "v": v, "scan_heading_deg": scan_heading_deg})
        mark = self._mark()
        frame = self._frame_at(scan_heading_deg, res)
        seen = self._mark()
        v = frame.K.cy if v is None else v
        p = frame.point(u, v)
        # above the horizon with no depth: the ray's direction
        d = p if p is not None else frame.ray(u, v)[1]
        bearing = math.atan2(d[1], d[0])
        res.note = f"bearing {math.degrees(bearing):+.1f} deg"
        self._rotate(bearing, 0.6, res)
        return self._finish(res, mark, aim=(p, seen))

    def go_to_pixel(
        self,
        u: float,
        v: float,
        stop_short_m: float = 0.43,
        speed: float = 0.25,
        scan_heading_deg: float = 0.0,
        far_m: float = 1.5,
        snap: bool = True,
    ) -> Result:
        """Go to the floor spot at pixel (u, v) of the last frame: turn to face it, then drive
        straight until it is `stop_short_m` ahead of the rover's center (negative remainder: back
        up). `stop_short_m` 0.43 = the pick zone's center (0.33-0.53 m). A click on or near a
        detected sock aims at the sock's center on the floor.
        `snap`: move the click to the middle of the blob that stands out from the floor at or
        right next to it (a rough click on the sock is enough; snap=false to aim at bare floor).
        `speed` m/s (max 0.4). `scan_heading_deg`: the pixel is from the scan frame with that
        heading (only right after `scan`). A spot beyond `far_m` is approached in two legs: the
        rover stops ~0.9 m before it, finds the clicked patch again in a new frame and measures
        it there (if it cannot, it stops there with ok=false: click the sock again).
        `target` in the result: where the spot should be now (rover frame, pixel, in the pick
        zone or not); check it against the frame."""
        res = Result(
            "go_to_pixel",
            {
                "u": u,
                "v": v,
                "stop_short_m": stop_short_m,
                "speed": speed,
                "scan_heading_deg": scan_heading_deg,
                "far_m": far_m,
                "snap": snap,
            },
        )
        mark = self._mark()
        frame = self._frame_at(scan_heading_deg, res)
        seen = self._mark()
        depth = frame.depth_at(u, v)
        p = frame.point(u, v, depth)
        if p is None:
            res.ok, res.note = False, "the pixel is above the horizon and has no depth"
            return self._finish(res, mark)
        snapped = ""
        if snap:
            c = snap_to_blob(frame, u, v, math.hypot(p[0], p[1]))
            q = frame.point(*c) if c is not None else None
            # keep it only if it is a sock-high thing about as far as the click (not a wall)
            if q is not None and c != (int(round(u)), int(round(v))):
                d0, d1 = math.hypot(p[0], p[1]), math.hypot(q[0], q[1])
                if q[2] < SOCK_MAX_H + 0.02 and abs(d1 - d0) < max(0.25, 0.15 * d0):
                    (u, v), p, depth = c, q, frame.depth_at(*c)
                    snapped = f"snapped to the blob at ({u}, {v}); "
            # a detected sock at the click: aim at its center on the floor, not at the pixel
            # (the middle of a sock's image lies nearer than its middle on the floor)
            sock = self._sock_at(frame, u, v)
            if sock is not None:
                p = np.array([sock.x, sock.y, 0.0])
                snapped += f"aimed at the sock's floor center ({sock.x:.2f}, {sock.y:+.2f}) m; "
        bearing = math.atan2(p[1], p[0])
        dist = math.hypot(p[0], p[1])
        res.note = snapped + (
            f"target {dist:.2f} m at {math.degrees(bearing):+.1f} deg "
            f"({'depth' if depth is not None else 'floor ray'})"
        )
        self._rotate(bearing, 0.6, res)
        if res.blocked is None and dist > far_m and stop_short_m < REAIM_M:
            # depth is noisy far away (±10-20 cm at 2 m): drive most of the way, find the spot
            # again in a closer frame and measure it there
            self._straight(dist - REAIM_M, speed, res)
            if res.blocked is None:
                self._settle()
                near = self.camera.capture()
                hit = self._reacquire(frame, u, v, p, seen, near)
                if hit is None:
                    res.ok = False
                    res.note += (
                        f"; far target: stopped ~{REAIM_M:.1f} m before it and lost it in the "
                        "new frame: click it again"
                    )
                    self.frame = near
                    return self._finish(res, mark, aim=(p, seen))
                (u2, v2), p2 = hit
                sock = self._sock_at(near, u2, v2) if snap else None
                if sock is not None:
                    p2 = np.array([sock.x, sock.y, 0.0])
                res.note += f"; re-aimed at ({u2}, {v2}) {math.hypot(p2[0], p2[1]):.2f} m"
                p, seen = p2, self._mark()
                self._rotate(math.atan2(p2[1], p2[0]), 0.6, res)
                dist = math.hypot(p2[0], p2[1])
        if res.blocked is None:
            self._straight(dist - stop_short_m, speed, res)
        return self._finish(res, mark, aim=(p, seen))

    def scan(self, step_deg: float = 60.0) -> Result:
        """Turn a full circle in steps of `step_deg` (60 = 6 frames, the camera sees ~69 deg
        wide), a frame at each; ends facing exactly the way it started. `frames` holds (heading
        relative to the start, left positive, frame); the heading is printed on each frame. To
        go to something seen in one, pass its heading: `go_to_pixel u v scan_heading_deg=H`."""
        res = Result("scan", {"step_deg": step_deg})
        mark = self._mark()
        n = max(1, round(360 / abs(step_deg)))
        step = math.radians(360 / n)
        yaw0 = self.sim.odom.yaw
        for k in range(n):
            self._settle()
            res.frames.append((round(math.degrees(k * step), 1), self.camera.capture()))
            self._rotate_to(yaw0 + (k + 1) * step, 0.8, res)
            if res.blocked:
                break
        return self._finish(res, mark)

    def seek(
        self,
        step_deg: float = 50.0,
        max_deg: float = 360.0,
        min_score: float = 0.7,
        max_range_m: float = 3.0,
    ) -> Result:
        """Turn in place in steps of about `step_deg` (left positive, negative = to the right)
        until the detector sees a sock (score >= `min_score`, within `max_range_m`), then turn
        to face it and stop. Nothing that sure within `max_deg`: face the likeliest candidate
        seen (best score), if any; `ok` is false when there was none. The note gives the
        sock's pixel in the last frame, ready for `go_to_pixel`. `frames` holds (heading,
        frame) as in `scan`: `turn <heading>` faces that view again."""
        res = Result(
            "seek",
            {
                "step_deg": step_deg,
                "max_deg": max_deg,
                "min_score": min_score,
                "max_range_m": max_range_m,
            },
        )
        mark = self._mark()
        n = max(1, math.ceil(abs(max_deg) / abs(step_deg) - 1e-6))
        step = math.copysign(math.radians(abs(max_deg) / n), step_deg)
        views, best, found = [], None, False  # best: (score, target yaw)
        for k in range(n):
            self._settle()
            f = self.camera.capture()
            views.append((self.sim.odom.yaw, f))
            dets = [d for d in self._socks(f) if d.distance <= max_range_m]
            sure = [d for d in dets if d.score >= min_score]
            if sure:
                d = min(sure, key=lambda d: d.distance)
                self._rotate(math.radians(d.bearing_deg), 0.6, res)
                found = True
                break
            for d in dets:
                if best is None or d.score > best[0]:
                    best = (d.score, self.sim.odom.yaw + math.radians(d.bearing_deg))
            if k < n - 1:
                self._rotate(step, 0.8, res)
                if res.blocked:
                    break
        if not found and best is not None:
            self._rotate(_wrap(best[1] - self.sim.odom.yaw), 0.6, res)
        res.frames = [(round(self._heading(yaw), 1), f) for yaw, f in views]
        res = self._finish(res, mark)
        dets = self._socks(res.frame)
        if dets:
            d = min(dets[:3], key=lambda d: abs(d.bearing_deg))
            res.note = (
                f"{'sock' if found else 'likeliest candidate'} at ({d.u:.0f},{d.v:.0f}): "
                f"{d.distance:.2f} m {d.bearing_deg:+.1f} deg score {d.score:.2f}"
            )
        else:
            res.ok, res.note = False, f"no sock in {abs(math.degrees(n * step)):.0f} deg"
        return res

    def clearance(self, step_deg: float = 5.0) -> Result:
        """No motion: from a fresh depth frame, how far (m, from the bumper) the rover could drive
        straight toward each bearing (deg, left positive) it can see, its full width plus a
        margin clear of anything taller than a sock. Capped at 3 m or where the floor stops
        being seen (a plain wall gives no depth). `clear` = [(bearing, free m), ...]."""
        res = Result("clearance", {"step_deg": step_deg})
        mark = self._mark()
        res = self._finish(res, mark)
        res.clear = free_space(res.frame, step_deg)
        best = max(res.clear, key=lambda bf: bf[1])
        ahead = min(res.clear, key=lambda bf: abs(bf[0]))
        res.note = f"ahead {ahead[1]:.2f} m, most open {best[1]:.2f} m at {best[0]:+.0f} deg"
        return res

    def nudge(self, direction: str, amount: float | None = None) -> Result:
        """A small precise step: `forward` / `back` `amount` m (default 0.05), `left` /
        `right` `amount` deg in place (default 5)."""
        res = Result("nudge", {"direction": direction, "amount": amount})
        mark = self._mark()
        if direction in ("forward", "back"):
            d = abs(amount) if amount is not None else NUDGE_M
            self._straight(d if direction == "forward" else -d, 0.1, res)
        elif direction in ("left", "right"):
            a = abs(amount) if amount is not None else NUDGE_DEG
            self._rotate(math.radians(a if direction == "left" else -a), 0.4, res)
        else:
            res.ok, res.note = False, "direction: forward | back | left | right"
        return self._finish(res, mark)

    def stop(self) -> Result:
        """Stop now."""
        return self._finish(Result("stop", {}), self._mark())

    def set_velocity(self, v: float, w: float, duration_s: float) -> Result:
        """Debug: the raw `cmd_vel` twist (m/s, rad/s) for `duration_s`, open loop."""
        res = Result("set_velocity", {"v": v, "w": w, "duration_s": duration_s})
        mark = self._mark()
        t_end = self.sim.t + duration_s
        while self.sim.t < t_end:
            self.sim.set_cmd(v, w)
            self._tick()
        return self._finish(res, mark)

    def run(self, name: str, *args, **kwargs) -> Result:
        if name not in self.COMMANDS:
            raise ValueError(f"unknown command {name!r}; one of {', '.join(self.COMMANDS)}")
        return getattr(self, name)(*args, **kwargs)

    # --- motion ---

    def _straight(self, distance: float, speed: float, res: Result) -> None:
        leo = self.sim.cfg.leo
        speed = min(abs(speed), leo.max_linear_mps)
        o = self.sim.odom
        x0, y0, yaw0 = o.x, o.y, o.yaw
        sign = 1.0 if distance >= 0 else -1.0
        lead = getattr(self.sim, "stop_lead_s", 0.0)
        t_end = self.sim.t + self.sim.cfg.max_command_s
        next_guard = self.sim.t
        while True:
            along = (o.x - x0) * math.cos(yaw0) + (o.y - y0) * math.sin(yaw0)
            # the real rover coasts on for `stop_lead_s` after the stop: count that in
            rem = abs(distance) - sign * along - abs(o.v) * lead
            if rem <= DIST_TOL_M:
                break
            if self.sim.t > t_end:
                res.blocked = "timeout"
                break
            if sign > 0 and self.sim.t >= next_guard:
                next_guard = self.sim.t + 1 / GUARD_HZ
                ahead = self._obstacle_ahead()
                if ahead is not None and ahead < GUARD_M + 0.5 * o.v:
                    res.blocked = f"obstacle {ahead:.2f} m ahead of the front wheels"
                    break
            # slow down to stop on the spot, the firmware's own ramp lags a little
            v = min(speed, math.sqrt(2 * 0.6 * leo.accel_mps2 * rem) + 0.02)
            w = float(np.clip(3.0 * _wrap(yaw0 - o.yaw), -0.5, 0.5))
            self.sim.set_cmd(sign * v, w)
            self._tick()
        self._halt()

    def _rotate(self, angle: float, speed: float, res: Result) -> None:
        self._rotate_to(self.sim.odom.yaw + angle, speed, res)

    def _rotate_to(self, target: float, speed: float, res: Result) -> None:
        """Turn in place to the odometry yaw `target` (rad, unwrapped)."""
        leo = self.sim.cfg.leo
        speed = min(abs(speed), leo.max_angular_rps)
        o = self.sim.odom
        t_end = self.sim.t + self.sim.cfg.max_command_s
        lead = getattr(self.sim, "stop_lead_s", 0.0)
        while True:
            err = target - o.yaw
            if lead and err * o.w > 0:  # turning toward the target: it coasts on this much
                err -= math.copysign(min(abs(o.w) * lead, abs(err)), err)
            if abs(err) <= ANGLE_TOL:
                break
            if self.sim.t > t_end:
                res.blocked = "timeout"
                break
            w = min(speed, math.sqrt(2 * 0.6 * leo.angular_accel_rps2 * abs(err)) + 0.05)
            self.sim.set_cmd(0.0, math.copysign(w, err))
            self._tick()
        self._halt()

    def _halt(self) -> None:
        self.sim.set_cmd(0.0, 0.0)
        for _ in range(int(2.0 / self.sim.dt)):
            self._tick()
            if (
                not self.sim.moving()
                and abs(self.sim.ref[0]) < 1e-3
                and abs(self.sim.ref[1]) < 1e-3
            ):
                break

    def _settle(self) -> None:
        """Let the chassis stop rocking before a frame."""
        for _ in range(int(0.15 / self.sim.dt)):
            self.sim.set_cmd(0.0, 0.0)
            self._tick()

    def _tick(self) -> None:
        if self.cancelled():
            self.sim.set_cmd(0.0, 0.0)
            raise Cancelled
        self.sim.tick()
        if self.on_tick is not None:
            self.on_tick(self.sim)

    def _obstacle_ahead(self) -> float | None:
        """Distance (m) from the bumper to the nearest thing in the rover's path that stands
        taller than a sock, from a fresh depth frame; None if the path looks clear."""
        return self._obstacle_in(self.camera.capture())

    @staticmethod
    def _obstacle_in(f: Frame) -> float | None:
        """`_obstacle_ahead` on a given frame."""
        p = f.points()
        x, y, z = p[..., 0], p[..., 1], p[..., 2]
        hit = (
            (np.abs(y) < HALF_WIDTH_M + 0.03) & (z > SOCK_MAX_H) & (z < 0.45) & (x > FRONT_M - 0.05)
        )
        hit &= ~np.isnan(x)
        if np.count_nonzero(hit) < 40:
            return None
        return float(np.percentile(x[hit], 2)) - FRONT_M

    # --- bookkeeping ---

    def _reacquire(self, f1: Frame, u: float, v: float, p1, seen, f2: Frame):
        """Find the patch around (u, v) of frame `f1` (target point `p1`, seen at `seen`) in the
        closer frame `f2`: template matching near where the odometry puts it, the template
        scaled for the new distance and view angle. ((u, v), rover-frame point) or None."""
        pred = self._target_now(p1, seen)
        d1 = math.hypot(p1[0] - f1.T_rover_cam[0, 3], p1[1] - f1.T_rover_cam[1, 3])
        cam_h = float(f1.T_rover_cam[2, 3])
        half = int(np.clip(f1.K.fx * 0.07 / max(d1, 0.3), 8, 60))
        h, w = f1.rgb.shape[:2]
        u0, v0 = int(round(u)), int(round(v))
        if u0 - half < 0 or v0 - half < 0 or u0 + half >= w or v0 + half >= h:
            return None
        g1, g2 = f1.rgb, f2.rgb  # color: a sock differs from the floor mostly in hue
        tpl = g1[v0 - half : v0 + half + 1, u0 - half : u0 + half + 1]
        best = None
        x_pred = pred["x_m"] - f2.T_rover_cam[0, 3]
        for err in (-0.3, -0.15, 0.0, 0.15, 0.3):  # the far range estimate's error (m)
            d2 = max(0.3, math.hypot(x_pred + err * x_pred / max(d1, 0.1), pred["y_m"]))
            p2 = (pred["x_m"] + err * x_pred / max(d1, 0.1), pred["y_m"], 0.0)
            c = f2.project(p2)
            if c is None:
                continue
            sx = d1 / d2
            sy = sx * math.sin(math.atan2(cam_h, d2)) / math.sin(math.atan2(cam_h, d1))
            t = cv2.resize(tpl, None, fx=sx, fy=sy, interpolation=cv2.INTER_LINEAR)
            th, tw = t.shape[:2]
            # search a window around the predicted spot for this range
            mu, mv = int(0.2 * w), int(0.5 * th)
            x0, x1 = int(c[0] - tw / 2 - mu), int(c[0] + tw / 2 + mu)
            y0, y1 = int(c[1] - th / 2 - mv), int(c[1] + th / 2 + mv)
            x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
            if x1 - x0 <= tw or y1 - y0 <= th:
                continue
            r = cv2.matchTemplate(g2[y0:y1, x0:x1], t, cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(r)
            if best is None or score > best[0]:
                best = (score, (x0 + loc[0] + tw // 2, y0 + loc[1] + th // 2))
        if best is None or best[0] < REAIM_MIN_SCORE:
            # the patch changed too much (a long sock seen end-on from far): the blob that
            # stands out from the floor nearest to where the odometry puts the spot
            c = f2.project((pred["x_m"], pred["y_m"], 0.0))
            b = snap_to_blob(f2, c[0], c[1], math.hypot(x_pred, pred["y_m"])) if c else None
            q = f2.point(*b) if b is not None else None
            if q is None or math.hypot(q[0] - pred["x_m"], q[1] - pred["y_m"]) > 0.35:
                return None
            return b, q
        (bu, bv) = best[1]
        d2 = f2.point(bu, bv)
        c = snap_to_blob(f2, bu, bv, math.hypot(d2[0], d2[1])) if d2 is not None else None
        q = f2.point(*c) if c is not None else None
        if (
            q is not None
            and d2 is not None
            and q[2] < SOCK_MAX_H + 0.02
            and abs(math.hypot(q[0], q[1]) - math.hypot(d2[0], d2[1])) < 0.2
        ):
            bu, bv = c
        p = f2.point(bu, bv)
        if p is None:
            return None
        return (int(bu), int(bv)), p

    def _frame_at(self, scan_heading_deg: float, res: Result) -> Frame:
        """The frame a pixel argument refers to: the last one, or (after a scan) the view at
        `scan_heading_deg`, re-taken after turning there."""
        if not scan_heading_deg:
            return self._last()
        self._rotate(_wrap(math.radians(scan_heading_deg)), 0.6, res)
        self._settle()
        self.frame = self.camera.capture()
        return self.frame

    def _finish(self, res: Result, mark, aim=None) -> Result:
        """Stop, fill in the motion, a fresh frame and its `view`. `aim`: (rover-frame point,
        the mark when it was seen) of a pixel command's target, reported as `res.target`."""
        x0, y0, yaw0, t0 = mark
        o = self.sim.odom
        self._settle()
        res.moved_m = (o.x - x0) * math.cos(yaw0) + (o.y - y0) * math.sin(yaw0)
        res.turned_deg = math.degrees(o.yaw - yaw0)
        res.elapsed_s = self.sim.t - t0
        res.frame = self.frame = self.camera.capture()
        if aim is not None and aim[0] is not None:
            res.target = self._target_now(*aim)
        res.view = self._view(res.frame)
        return res

    def _target_now(self, p, mark) -> dict:
        """Where a rover-frame point `p` seen at `mark` should be now, by the odometry."""
        x0, y0, yaw0, _ = mark
        o = self.sim.odom
        # the point in the odometry frame, then in the current rover frame
        wx = x0 + p[0] * math.cos(yaw0) - p[1] * math.sin(yaw0)
        wy = y0 + p[0] * math.sin(yaw0) + p[1] * math.cos(yaw0)
        dx, dy = wx - o.x, wy - o.y
        x = dx * math.cos(o.yaw) + dy * math.sin(o.yaw)
        y = -dx * math.sin(o.yaw) + dy * math.cos(o.yaw)
        g = self.sim.cfg.goal
        out = {"x_m": round(x, 3), "y_m": round(y, 3)}
        px = self.frame.project((x, y, 0.0)) if self.frame is not None else None
        w, h = (self.frame.K.width, self.frame.K.height) if self.frame is not None else (0, 0)
        out["pixel"] = (
            [int(round(px[0])), int(round(px[1]))]
            if px is not None and 0 <= px[0] < w and 0 <= px[1] < h
            else None
        )
        out["in_zone"] = bool(
            abs(x - g.center_m[0]) <= g.half_size_m[0]
            and abs(y - g.center_m[1]) <= g.half_size_m[1]
        )
        return out

    def _view(self, f: Frame) -> dict:
        """A compact observation of frame `f`: numbers an operator would otherwise query."""
        o = self.sim.odom
        g = self.sim.cfg.goal
        (cx, cy), (hx, hy) = g.center_m, g.half_size_m
        corners = [
            f.project((cx + a * hx, cy + b * hy, 0.0))
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))
        ]
        zone = None
        if all(c is not None for c in corners):
            us, vs = [c[0] for c in corners], [c[1] for c in corners]
            zone = [int(min(us)), int(min(vs)), int(max(us)), int(max(vs))]
        rows = {}
        for v in VIEW_ROWS:
            p = f.floor_point(f.K.cx, v)
            if p is not None:
                rows[str(v)] = round(float(p[0]), 2)
        ahead = self._obstacle_in(f)
        return {
            "pose": {
                "x_m": round(o.x, 2),
                "y_m": round(o.y, 2),
                "heading_deg": round(math.degrees(_wrap(o.yaw)), 1),
            },
            "zone_px": zone,  # [u_min, v_min, u_max, v_max] of the pick zone (green quad)
            "floor_x_m_at_row": rows,  # image row v -> floor distance ahead of the center (m)
            "clear_ahead_m": None
            if ahead is None
            else round(ahead, 2),  # None: nothing tall seen in the path
        }

    def _sock_at(self, frame: Frame, u: float, v: float, reach_px: int = 45):
        """The detected sock whose mask holds pixel (u, v) or comes within `reach_px` of it."""
        best = None
        for d in self._socks(frame):
            if d.mask is None:
                continue
            ys, xs = np.nonzero(d.mask)
            gap = float(np.min(np.hypot(xs - u, ys - v))) if xs.size else 1e9
            if gap <= reach_px and (best is None or gap < best[0]):
                best = (gap, d)
        return None if best is None else best[1]

    def _socks(self, frame: Frame) -> list:
        """The detector's socks in a frame that have a floor position, best first."""
        if self.detector is None:
            from sorter.nav.detect import ClassicDetector

            self.detector = ClassicDetector()
        return [d for d in self.detector.detect(frame) if d.x is not None]

    def _heading(self, yaw: float) -> float:
        """An odometry yaw as a heading relative to where the rover points now (deg, left
        positive): `turn <heading>` faces it again."""
        return math.degrees(_wrap(yaw - self.sim.odom.yaw))

    def _last(self) -> Frame:
        if self.frame is None:
            self.frame = self.camera.capture()
        return self.frame

    def _mark(self) -> tuple[float, float, float, float]:
        o = self.sim.odom
        return (o.x, o.y, o.yaw, self.sim.t)


def snap_to_blob(frame: Frame, u: float, v: float, dist_m: float) -> tuple[int, int] | None:
    """The centroid of the blob that stands out from the floor at or next to pixel (u, v): a
    window about 0.4 m wide at `dist_m`, the floor color from its border, the connected region
    of other colors that holds (or is nearest to) the click. None if there is none."""
    h, w = frame.rgb.shape[:2]
    r = int(np.clip(frame.K.fx * 0.2 / max(dist_m, 0.3), 12, 120))
    u0, v0 = int(round(u)), int(round(v))
    x0, x1, y0, y1 = max(0, u0 - r), min(w, u0 + r + 1), max(0, v0 - r), min(h, v0 + r + 1)
    win = frame.rgb[y0:y1, x0:x1].astype(np.float32)
    if win.shape[0] < 8 or win.shape[1] < 8:
        return None
    ring = np.concatenate(
        [
            win[:3].reshape(-1, 3),
            win[-3:].reshape(-1, 3),
            win[:, :3].reshape(-1, 3),
            win[:, -3:].reshape(-1, 3),
        ]
    )
    floor = np.median(ring, 0)
    spread = np.median(np.linalg.norm(ring - floor, axis=1))
    diff = np.linalg.norm(win - floor, axis=2)
    mask = (diff > max(35.0, 3.0 * spread)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return None
    cu, cv_ = u0 - x0, v0 - y0
    best, best_d = None, float(r) * 0.6
    for k in range(1, n):
        area = stats[k, cv2.CC_STAT_AREA]
        if area < 15 or area > 0.7 * mask.size:
            continue
        ys, xs = np.nonzero(labels == k)
        d = float(np.min(np.hypot(xs - cu, ys - cv_)))
        if d < best_d:
            best, best_d = k, d
    if best is None:
        return None
    return int(round(cents[best][0] + x0)), int(round(cents[best][1] + y0))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wall_clock() -> float:  # for the live panel's pacing
    return time.monotonic()


def free_space(frame: Frame, step_deg: float = 5.0) -> list[tuple[float, float]]:
    """For each bearing the camera sees (left positive), the free straight run (m, from the
    bumper) of a corridor as wide as the rover plus a margin: up to the nearest point taller
    than a sock in it, and no further than the floor is seen in it (no depth past a plain wall,
    or beyond the camera's view). Near the rover the corridor's sides are out of view for the
    wider bearings: turn toward a bearing and look again before trusting a long run."""
    p = frame.points()
    ok = ~np.isnan(p[..., 2])
    x, y, z = p[..., 0][ok], p[..., 1][ok], p[..., 2][ok]
    tall = (z > SOCK_MAX_H) & (z < 0.45)
    floor = np.abs(z) < 0.03
    half = HALF_WIDTH_M + CLEAR_MARGIN_M
    n = int(CLEAR_HALF_FOV_DEG // step_deg)
    out = []
    for k in range(-n, n + 1):
        th = math.radians(k * step_deg)
        c, s = math.cos(th), math.sin(th)
        xa, ya = x * c + y * s, -x * s + y * c
        inside = (np.abs(ya) < half) & (xa > 0)
        th_hit = inside & tall
        free = CLEAR_MAX_M + FRONT_M
        if np.count_nonzero(th_hit) >= 15:
            free = min(free, float(np.percentile(xa[th_hit], 2)))
        fl = inside & floor & (np.abs(ya) < half * 0.6)
        if np.count_nonzero(fl) >= 30:
            free = min(free, float(np.percentile(xa[fl], 99)))
        else:
            free = FRONT_M
        out.append((float(k * step_deg), max(0.0, free - FRONT_M)))
    return out
