"""Rover navigation sim CLI. One command per process; an episode lives in a directory.

python -m sorter.nav new DIR --scenario easy --seed 0 [--set camera.pitch_deg=30 ...]
python -m sorter.nav do DIR forward 0.5 [speed=0.3]        # any command, see `commands`
python -m sorter.nav look DIR
python -m sorter.nav depth DIR U V                          # depth and floor point of a pixel
python -m sorter.nav detect DIR [--detector classic|sam3|seg]
python -m sorter.nav finish DIR                             # ground-truth score, ends it
python -m sorter.nav auto --scenario easy --seed 0 [--detector classic] [--out DIR]
python -m sorter.nav bench --scenarios easy,side --seeds 0-9 [--detector classic]
python -m sorter.nav serve [--port 8010] [--real]           # the Rover tab on its own
python -m sorter.nav commands | scenarios

The real Leo Rover and OAK-D (nav.real; --rosbridge ws://10.0.0.1:9090 from a laptop):
python -m sorter.nav hw-check [--no-rover] [--no-camera] [--move]
python -m sorter.nav real do DIR forward 0.3               # one command, frames into DIR
python -m sorter.nav real look DIR | real detect DIR [--detector classic|sam3]
python -m sorter.nav real auto [--detector sam3] [--out DIR]  # the approach algorithm

Search for socks and drive up to them, one after another (sorter.nav.hunt.hunt_socks):
python -m sorter.nav hunt [--real] [--socks N] [--gap 0.30] [--detector classic|sam3] [--out DIR]
python -m sorter.nav hunt --scenario multi --seed 0 --socks 3   # the same on the sim

Cardboard boxes with AprilTags (36h11) on the real rover (sorter.nav.boxes):
python -m sorter.nav boxes remember                  # save the boxes in view (nav.boxes.memory)
python -m sorter.nav boxes go [--target 13] [--stop 0.15]  # drive up to the box's tag
"""

from __future__ import annotations

import argparse
import inspect
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
# one thread per process: benches run several processes, and threaded OpenCV / numpy in each
# oversubscribe the cores
for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

DEFAULT_CONFIG_DIR = "config"


def _nav_config(config_dir: str):
    from sorter.core.config import load_config

    try:
        return load_config(Path(config_dir))
    except Exception:  # noqa: BLE001 - the nav sim runs with its defaults without the repo config
        return None


def _base(args):
    cfg = _nav_config(args.config_dir)
    return (cfg.nav if cfg is not None else None), cfg


def _parse_value(s: str):
    try:
        return json.loads(s)
    except json.JSONDecodeError:
        return s


def _print_result(res, root: Path | None) -> None:
    out = res.summary()
    if root is not None and res.frame is not None:
        out["frame"] = str(root / "frames" / f"{res.frame.index:03d}_grid.png")
        out["depth"] = str(root / "frames" / f"{res.frame.index:03d}_depth.png")
    if res.frames:
        out["scan"] = [
            {"heading_deg": h, "frame": str(root / "frames" / f"{f.index:03d}_grid.png")}
            for h, f in res.frames
        ]
    print(json.dumps(out, indent=1))


def _command_args(raw: list[str]) -> tuple[list, dict]:
    """`["220", "109", "stop_short_m=0.4"]` → ([220, 109], {"stop_short_m": 0.4})."""
    pos, kw = [], {}
    for a in raw:
        if "=" in a and not a.lstrip("-").replace(".", "").isdigit():
            k, v = a.split("=", 1)
            kw[k] = _parse_value(v)
        else:
            pos.append(_parse_value(a))
    return pos, kw


def cmd_new(args) -> None:
    from sorter.nav.episode import Episode

    base, _ = _base(args)
    overrides = dict(kv.split("=", 1) for kv in args.set)
    ep, res = Episode.new(Path(args.dir), args.scenario, args.seed, overrides, base)
    _print_result(res, ep.root)
    ep.close()


def cmd_do(args) -> None:
    from sorter.nav.episode import Episode

    root = Path(args.dir)
    ep = Episode.load(root)
    pos, kw = _command_args(args.args)
    try:
        res = ep.rover.run(args.command, *pos, **kw)
    except TypeError as e:
        sys.exit(f"bad arguments for {args.command}: {e}")
    ep.record(res)
    _print_result(res, root)
    ep.close()


def cmd_depth(args) -> None:
    from sorter.nav.episode import Episode

    ep = Episode.load(Path(args.dir))
    f = ep.rover.frame
    z = f.depth_at(args.u, args.v)
    p = f.point(args.u, args.v, z)
    out = {"u": args.u, "v": args.v, "depth_m": None if z is None else round(z, 3)}
    if p is not None:
        import math

        out |= {
            "x_m": round(float(p[0]), 3),
            "y_m": round(float(p[1]), 3),
            "z_m": round(float(p[2]), 3),
            "distance_m": round(math.hypot(p[0], p[1]), 3),
            "bearing_deg": round(math.degrees(math.atan2(p[1], p[0])), 1),
            "from": "depth" if z is not None else "floor ray",
        }
    print(json.dumps(out))
    ep.close()


def cmd_detect(args) -> None:
    import cv2

    from sorter.nav import detect
    from sorter.nav.episode import Episode

    root = Path(args.dir)
    ep = Episode.load(root)
    _, full = _base(args)
    sam = full.color_classifier.sam if full is not None else None
    det = detect.make(args.detector, ep.camera, sam, ep.cfg.sam_prompt)
    f = ep.rover.frame
    if args.detector == "seg":  # renders the current state: needs the sim where the frame was
        pass
    dets = det.detect(f)
    path = root / "frames" / f"{f.index:03d}_detect_{args.detector}.png"
    cv2.imwrite(str(path), cv2.cvtColor(detect.overlay(f, dets), cv2.COLOR_RGB2BGR))
    print(json.dumps({"detections": [d.summary() for d in dets], "overlay": str(path)}, indent=1))
    ep.close()


def cmd_finish(args) -> None:
    from sorter.nav.episode import Episode

    root = Path(args.dir)
    ep = Episode.load(root)
    s = ep.score().summary()
    (root / "score.json").write_text(json.dumps(s, indent=1))
    print(json.dumps(s, indent=1))
    ep.close()


def _run_auto(
    scenario: str, seed: int, detector: str, overrides: dict, out: str | None, base, sam_cfg
) -> dict:
    from sorter.nav import detect
    from sorter.nav.controller import Approach
    from sorter.nav.episode import Episode, apply_overrides
    from sorter.nav.scenario import make

    t = time.time()
    if out:
        ep, _ = Episode.new(Path(out), scenario, seed, overrides, base)
    else:
        from sorter.nav.config import NavConfig

        ep = Episode(make(scenario, seed), apply_overrides(base or NavConfig(), overrides))
    det = detect.make(detector, ep.camera, sam_cfg, ep.cfg.sam_prompt)
    ap = Approach(ep.rover, det, ep.cfg.goal)
    if out:  # every command's frames and log go to the episode directory
        run = ep.rover.run

        def recorded(name, *a, **k):
            res = run(name, *a, **k)
            ep.record(res)
            return res

        ep.rover.run = recorded
    ok = ap.run()
    ep.commands = ap.commands
    s = ep.score().summary() | {
        "scenario": scenario,
        "seed": seed,
        "claimed": ok,
        "wall_s": round(time.time() - t, 1),
    }
    if out:
        (Path(out) / "score.json").write_text(json.dumps(s, indent=1))
        (Path(out) / "controller_log.json").write_text(json.dumps(ap.log.steps, indent=1))
    ep.close()
    return s


def cmd_auto(args) -> None:
    base, full = _base(args)
    overrides = dict(kv.split("=", 1) for kv in args.set)
    sam = full.color_classifier.sam if full is not None else None
    print(
        json.dumps(
            _run_auto(args.scenario, args.seed, args.detector, overrides, args.out, base, sam),
            indent=1,
        )
    )


def _bench_one(job):
    import cv2

    cv2.setNumThreads(1)
    scenario, seed, detector, overrides, config_dir = job
    base, full = _base(argparse.Namespace(config_dir=config_dir))
    sam = full.color_classifier.sam if full is not None else None
    try:
        return _run_auto(scenario, seed, detector, overrides, None, base, sam)
    except Exception as e:  # noqa: BLE001 - one bad episode doesn't stop the bench
        return {"scenario": scenario, "seed": seed, "error": repr(e), "success": False}


def _safe_jobs(jobs: int) -> int:
    """No more workers than cores, nor than free memory allows (~0.3 GB each, kept at half)."""
    avail_gb = 4.0
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                avail_gb = int(line.split()[1]) / 1e6
    except OSError:
        pass
    return max(1, min(jobs, os.cpu_count() or 1, int(avail_gb * 0.5 / 0.3)))


def _mean(rows: list[dict], key: str) -> float:
    return sum(r[key] for r in rows) / max(len(rows), 1)


def cmd_bench(args) -> None:
    from concurrent.futures import ProcessPoolExecutor

    a, _, b = args.seeds.partition("-")
    seeds = range(int(a), int(b or a) + 1)
    overrides = dict(kv.split("=", 1) for kv in args.set)
    jobs = [
        (s, n, args.detector, overrides, args.config_dir)
        for s in args.scenarios.split(",")
        for n in seeds
    ]
    with ProcessPoolExecutor(_safe_jobs(args.jobs)) as pool:
        rows = list(pool.map(_bench_one, jobs))
    by: dict[str, list] = {}
    for r in rows:
        by.setdefault(r["scenario"], []).append(r)
    print(f"{'scenario':10} {'success':>8} {'cmds':>6} {'sim s':>7} {'coll':>5} {'pushed':>7}")
    for name, rs in by.items():
        ok = [r for r in rs if not r.get("error")]
        sr = sum(r["success"] for r in rs) / len(rs)
        print(
            f"{name:10} {sr:8.0%} {_mean(ok, 'commands'):6.1f} {_mean(ok, 'sim_time_s'):7.1f} "
            f"{sum(r['collisions'] > 0 for r in ok):5d} {_mean(ok, 'sock_pushed_m'):7.3f}"
        )
    total = sum(r["success"] for r in rows) / len(rows)
    print(f"{'all':10} {total:8.0%}")
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=1))


def cmd_serve(args) -> None:
    import uvicorn

    from sorter.nav.server import standalone_app

    base, _ = _base(args)
    if args.real and args.rosbridge:
        from sorter.nav.config import NavConfig

        base = (base or NavConfig()).model_copy(deep=True)
        base.real.rosbridge_url = args.rosbridge
    print(
        f"Rover tab{' (REAL rover)' if args.real else ''}: http://{args.host}:{args.port}/rover",
        flush=True,
    )
    app = standalone_app(base, args.config_dir, real=args.real)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


# --- the real rover and camera ---


def _real_cfg(args):
    from sorter.nav.config import NavConfig

    base, full = _base(args)
    cfg = (base or NavConfig()).model_copy(deep=True)
    if getattr(args, "rosbridge", None):
        cfg.real.rosbridge_url = args.rosbridge
    sam = full.color_classifier.sam if full is not None else None
    return cfg, sam


def cmd_hw_check(args) -> None:
    """Checks the hardware step by step, and says what to fix for each failure."""
    import cv2

    from sorter.nav.camera import colorize_depth

    cfg, _ = _real_cfg(args)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    cam = base = None
    if not args.no_camera:
        try:
            from sorter.nav.real_oakd import RealOakD

            cam = RealOakD(cfg.camera, cfg.real)
            f = cam.capture()
            valid = float((f.depth_mm > 0).mean())
            cv2.imwrite(str(out / "hw_rgb.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(
                str(out / "hw_depth.png"),
                cv2.cvtColor(colorize_depth(f.depth_mm), cv2.COLOR_RGB2BGR),
            )
            from sorter.nav.camera import fit_floor
            from sorter.nav.model import BASE_Z

            frames = [f] + [cam.capture() for _ in range(4)]
            fit = fit_floor(frames)
            h_cfg = BASE_Z + cfg.camera.mount_xyz_m[2]
            if fit is None:
                print("  floor check: too little floor in the depth (aim at the floor, more light)")
            else:
                h, pitch, roll, n = fit
                print(
                    f"  floor check ({n} floor points): the camera is {h:.3f} m above the floor, "
                    f"{pitch:.1f} deg down, roll {roll:+.1f} deg; the config says "
                    f"{h_cfg:.3f} m, {cfg.camera.pitch_deg:.1f} deg"
                )
                if abs(h - h_cfg) > 0.02 or abs(pitch - cfg.camera.pitch_deg) > 2:
                    print(
                        f"  -> set nav.camera.mount_xyz_m z = {h - BASE_Z:+.3f} and pitch_deg = "
                        f"{pitch:.1f} (config/local.yaml); x = the camera ahead of the rover's "
                        "center (measure it)"
                    )
            if valid < 0.2:
                print("  little depth: is the floor plain (passive stereo needs texture)?")
        except Exception as e:  # noqa: BLE001 - a hardware check reports, never crashes
            ok = False
            print(f"camera FAILED: {e}\n  plug the OAK-D into USB3, `uv sync --extra nav-hw`")
    if not args.no_rover:
        try:
            from sorter.nav.real_leo import LeoBase

            base = LeoBase(cfg)
            o = base.odom
            print(
                f"rover OK: {cfg.real.rosbridge_url}, {cfg.real.odom_topic} flowing "
                f"(v {o.v:+.2f} m/s)"
            )
            if args.move:
                from sorter.nav.commands import Rover

                if cam is None:
                    print("  --move needs the camera for the obstacle guard: skipped")
                else:
                    r = Rover(base, cam)
                    a = r.turn(10, 0.3)
                    b = r.turn(-10, 0.3)
                    print(
                        f"  moved: turn +10 -> {a.turned_deg:+.1f} deg, back -> "
                        f"{b.turned_deg:+.1f} deg (both near 10: odometry and turning OK)"
                    )
        except Exception as e:  # noqa: BLE001
            ok = False
            print(
                f"rover FAILED: {e}\n  on the rover: ros2 launch rosbridge_server "
                "rosbridge_websocket_launch.xml; from a laptop: --rosbridge ws://10.0.0.1:9090"
            )
    if base is not None:
        base.close()
    if cam is not None:
        cam.close()
    print("ALL OK" if ok else "SOME CHECKS FAILED")
    sys.exit(0 if ok else 1)


def _real_session(args):
    from sorter.nav.real import RealSession

    cfg, sam = _real_cfg(args)
    return RealSession(cfg), cfg, sam


def cmd_real_do(args) -> None:
    from sorter.nav.real import last_frame, record

    root = Path(args.dir)
    s, cfg, _ = _real_session(args)
    try:
        s.rover.frame = last_frame(root, s.camera)  # pixel commands refer to the frame seen
        pos, kw = _command_args(args.args)
        res = s.rover.run(args.command, *pos, **kw)
        print(json.dumps(record(root, res, cfg.goal), indent=1))
    finally:
        s.close()


def cmd_real_detect(args) -> None:
    import cv2

    from sorter.nav import detect
    from sorter.nav.real import last_frame, record

    root = Path(args.dir)
    s, cfg, sam = _real_session(args)
    try:
        f = last_frame(root, s.camera)
        if f is None:  # nothing seen yet in this directory: look first
            res = s.rover.look()
            record(root, res, cfg.goal)
            f = res.frame
        det = detect.make(args.detector, None, sam, cfg.sam_prompt)
        dets = det.detect(f)
        path = root / "frames" / f"{f.index:03d}_detect_{args.detector}.png"
        cv2.imwrite(str(path), cv2.cvtColor(detect.overlay(f, dets), cv2.COLOR_RGB2BGR))
        print(
            json.dumps({"detections": [d.summary() for d in dets], "overlay": str(path)}, indent=1)
        )
    finally:
        s.close()


def cmd_real_auto(args) -> None:
    from sorter.nav import detect
    from sorter.nav.controller import Approach
    from sorter.nav.real import record

    s, cfg, sam = _real_session(args)
    root = Path(args.out) if args.out else None
    try:
        det = detect.make(args.detector, None, sam, cfg.sam_prompt)
        run = s.rover.run

        def logged(name, *a, **k):
            res = run(name, *a, **k)
            if root is not None:
                record(root, res, cfg.goal)
            print(json.dumps(res.summary()), flush=True)
            return res

        s.rover.run = logged
        ap = Approach(s.rover, det, cfg.goal)
        ok = ap.run()
        if root is not None:
            (root / "controller_log.json").write_text(json.dumps(ap.log.steps, indent=1))
        print(json.dumps({"claimed": ok, "commands": ap.commands}))
    except KeyboardInterrupt:
        print("stopped")
    finally:
        s.close()


def cmd_hunt(args) -> None:
    from sorter.nav.hunt import hunt_socks

    report = hunt_socks(
        real=args.real,
        max_socks=args.socks,
        detector=args.detector,
        scenario=args.scenario,
        seed=args.seed,
        config_dir=args.config_dir,
        rosbridge=args.rosbridge,
        out=args.out,
        gap_m=args.gap,
    )
    print(json.dumps(report.summary(), indent=1))


def cmd_boxes(args) -> None:
    from sorter.nav import boxes

    s, cfg, _ = _real_session(args)
    b = cfg.boxes
    try:
        if args.action == "remember":
            mem = boxes.remember_boxes(s.rover.look().frame, b.tag_size_m, b.target_id, b.memory)
            print(json.dumps(mem, indent=1))
        else:
            run = s.rover.run

            def logged(name, *a, **k):
                res = run(name, *a, **k)
                print(json.dumps(res.summary()), flush=True)
                return res

            s.rover.run = logged
            r = boxes.approach_box(
                s.rover,
                args.target if args.target is not None else b.target_id,
                args.stop if args.stop is not None else b.stop_m,
                b.tag_size_m,
                boxes.load_memory(b.memory),
                cfg.real.max_linear_mps,
            )
            print(json.dumps(vars(r), indent=1))
    except KeyboardInterrupt:
        print("stopped")
    finally:
        s.close()


def cmd_commands(args) -> None:
    from sorter.nav.commands import Rover

    for name in Rover.COMMANDS:
        fn = getattr(Rover, name)
        sig = str(inspect.signature(fn)).replace("self, ", "").replace("(self)", "()")
        doc = " ".join((fn.__doc__ or "").split())
        print(f"{name}{sig}\n    {doc}\n")


def cmd_scenarios(args) -> None:
    from sorter.nav.scenario import PRESETS

    for name, p in PRESETS.items():
        print(
            f"{name:9} distance {p.distance_m} m, |bearing| {p.bearing_deg} deg, "
            f"obstacles {p.obstacles}{' (blocking)' if p.blocking else ''}, "
            f"distractors {p.distractors}, floors {','.join(p.floors)}, "
            f"light {','.join(p.lights)}, socks {','.join(p.sock_kinds)}"
        )


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="python -m sorter.nav")
    ap.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("new")
    p.add_argument("dir")
    p.add_argument("--scenario", default="easy")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    p.set_defaults(fn=cmd_new)
    p = sub.add_parser("do")
    p.add_argument("dir")
    p.add_argument("command")
    p.add_argument("args", nargs="*")
    p.set_defaults(fn=cmd_do)
    p = sub.add_parser("look")
    p.add_argument("dir")
    p.set_defaults(fn=lambda a: cmd_do(argparse.Namespace(**vars(a), command="look", args=[])))
    p = sub.add_parser("depth")
    p.add_argument("dir")
    p.add_argument("u", type=float)
    p.add_argument("v", type=float)
    p.set_defaults(fn=cmd_depth)
    p = sub.add_parser("detect")
    p.add_argument("dir")
    p.add_argument("--detector", default="classic", choices=["classic", "sam3", "seg"])
    p.set_defaults(fn=cmd_detect)
    p = sub.add_parser("finish")
    p.add_argument("dir")
    p.set_defaults(fn=cmd_finish)
    p = sub.add_parser("auto")
    p.add_argument("--scenario", default="easy")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--detector", default="classic", choices=["classic", "sam3", "seg"])
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    p.add_argument("--out")
    p.set_defaults(fn=cmd_auto)
    p = sub.add_parser("bench")
    p.add_argument("--scenarios", default="easy,side,behind,near,far,obstacle,clutter,wall")
    p.add_argument("--seeds", default="0-4")
    p.add_argument("--detector", default="classic", choices=["classic", "sam3", "seg"])
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--out")
    p.set_defaults(fn=cmd_bench)
    p = sub.add_parser("serve")
    p.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to open it from a laptop")
    p.add_argument("--port", type=int, default=8010)
    p.add_argument("--real", action="store_true", help="the real rover and OAK-D, not the sim")
    p.add_argument("--rosbridge", help="rosbridge URL (default nav.real.rosbridge_url)")
    p.set_defaults(fn=cmd_serve)
    p = sub.add_parser("hw-check", help="check the real OAK-D and Leo Rover")
    p.add_argument("--no-rover", action="store_true")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--move", action="store_true", help="turn 10 deg and back (clear space!)")
    p.add_argument("--rosbridge")
    p.add_argument("--out", default="data/nav_hw")
    p.set_defaults(fn=cmd_hw_check)
    real = sub.add_parser("real", help="commands on the real rover").add_subparsers(
        dest="real_cmd", required=True
    )
    p = real.add_parser("do")
    p.add_argument("dir")
    p.add_argument("command")
    p.add_argument("args", nargs="*")
    p.add_argument("--rosbridge")
    p.set_defaults(fn=cmd_real_do)
    p = real.add_parser("look")
    p.add_argument("dir")
    p.add_argument("--rosbridge")
    p.set_defaults(fn=lambda a: cmd_real_do(argparse.Namespace(**vars(a), command="look", args=[])))
    p = real.add_parser("detect")
    p.add_argument("dir")
    p.add_argument("--detector", default="classic", choices=["classic", "sam3"])
    p.add_argument("--rosbridge")
    p.set_defaults(fn=cmd_real_detect)
    p = real.add_parser("auto")
    p.add_argument("--detector", default="sam3", choices=["classic", "sam3"])
    p.add_argument("--out")
    p.add_argument("--rosbridge")
    p.set_defaults(fn=cmd_real_auto)
    p = sub.add_parser("hunt", help="search for socks and drive up to them")
    p.add_argument("--real", action="store_true", help="the real rover and OAK-D, not the sim")
    p.add_argument("--socks", type=int, default=1, help="how many socks to reach")
    p.add_argument("--gap", type=float, help="fast: stop the bumper this far (m) before a sock")
    p.add_argument("--detector", choices=["classic", "sam3", "seg"])
    p.add_argument("--scenario", default="multi")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out")
    p.add_argument("--rosbridge")
    p.set_defaults(fn=cmd_hunt)
    p = sub.add_parser("boxes", help="AprilTag boxes on the real rover: remember | go")
    p.add_argument("action", choices=["remember", "go"])
    p.add_argument("--target", type=int)
    p.add_argument("--stop", type=float, help="front bumper to the tag at the stop (m)")
    p.add_argument("--rosbridge")
    p.set_defaults(fn=cmd_boxes)
    sub.add_parser("commands").set_defaults(fn=cmd_commands)
    sub.add_parser("scenarios").set_defaults(fn=cmd_scenarios)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
