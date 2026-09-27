"""Watch the load loop on one scene: the MuJoCo viewer live, and/or a video of the run.

    uv run mjpython -m sorter.sim.scenes.load.watch [--seed 3] [--socks 4] [--speed 1]
    uv run python -m sorter.sim.scenes.load.watch --no-viewer --record run.mp4 [--speed 0]

The real load loop (the same as `run --sim`, without the dashboard) on the load scene. The
console prints every decision and the counters; at the end, where each sock is. The viewer
needs `mjpython` on macOS. `--record` writes an MP4: an overview of the rover and the wrist
camera's live image side by side, one frame per 0.1 s of simulated time (played at 20 fps:
2x speed).
"""

from __future__ import annotations

import argparse
import logging
import threading
import time

import cv2
import mujoco
import numpy as np

from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.core.types import ColorClass, Command, OperatorMode, Phase

W, H = 640, 480  # each half of the video


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.sim.scenes.load.watch")
    p.add_argument("--seed", type=int, default=3)
    p.add_argument("--socks", type=int, default=4, help="how many (random colors, by seed)")
    p.add_argument("--speed", type=float, default=1.0, help="sim s per wall s; 0 = flat out")
    p.add_argument("--no-viewer", action="store_true")
    p.add_argument("--record", help="an .mp4 to write")
    p.add_argument("--time-limit", type=float, default=900.0, help="simulated s")
    p.add_argument("--config-dir", default=str(DEFAULT_CONFIG_DIR))
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    logging.getLogger("sorter.orchestrator.load").setLevel(logging.INFO)

    from sorter.app import build_system
    from sorter.orchestrator.state_machine import StateMachine

    rng = np.random.default_rng(args.seed)
    colors = [str(c) for c in rng.choice([c.value for c in ColorClass], args.socks)]
    cfg = load_config(
        args.config_dir,
        overrides={
            "sim": {
                "realtime": args.speed,
                "seed": args.seed,
                "scenes": ["load"],
                "load": {"socks": colors},
            },
            "backends": dict.fromkeys(("camera", "arm"), "sim"),
            "state_machine": {"save_runs": False},
        },
    )
    system = build_system(cfg, sim=True)
    world = system.world
    assert world is not None
    system.camera.start()
    sm = StateMachine(system)
    system.hub.set_mode(OperatorMode.LOAD)
    stop = threading.Event()
    loop = threading.Thread(target=sm.run, args=(stop,), name="state-machine", daemon=True)
    loop.start()
    system.hub.send(Command.START)
    print(f"seed {args.seed}: socks {colors}; speed {args.speed or 'flat out'}", flush=True)

    viewer = None
    if not args.no_viewer:
        from mujoco import viewer as mj_viewer

        viewer = mj_viewer.launch_passive(world.model, world.data)
        viewer.cam.lookat[:] = (0.05, 0.0, cfg.sim.layout.floor_z_mm / 1000 + 0.12)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.3, -140, -35
    video = renderer = None
    if args.record:
        video = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), 20, (2 * W, H))
        renderer = mujoco.Renderer(world.model, H, W)
        data = mujoco.MjData(world.model)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0.05, 0.02, cfg.sim.layout.floor_z_mm / 1000 + 0.12)
        cam.distance, cam.azimuth, cam.elevation = 1.35, -140, -38

    t0, next_frame, last = world.time(), 0.0, None
    try:
        while world.time() - t0 < args.time_limit:
            if viewer is not None:
                if not viewer.is_running():
                    break
                with world.lock:
                    viewer.sync()
            if video is not None and world.time() - t0 >= next_frame:
                next_frame = max(next_frame + 0.1, world.time() - t0)  # skip if behind
                with world.lock:
                    mujoco.mj_copyData(data, world.model, world.data)
                renderer.update_scene(data, camera=cam)
                left = renderer.render()[..., ::-1].copy()
                live = system.camera.latest()
                right = live.color.copy() if live is not None else np.zeros((H, W, 3), np.uint8)
                st = sm.phase.value
                n = sum(sm.counters.values())
                txt = f"{world.time() - t0:5.1f} s  {st}  in the box (counted): {n}"
                cv2.putText(left, txt, (10, 28), 0, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(right, "wrist camera", (10, 28), 0, 0.7, (255, 255, 255), 2)
                video.write(np.hstack([left, cv2.resize(right, (W, H))]))
            d = system.hub.decision()
            if d is not None and d is not last:
                last = d
                counted = {c.value: v for c, v in sm.counters.items() if v}
                print(f"{world.time() - t0:6.1f} s  {d.phase.value:12s} {d.summary}  {counted}")
            if sm.phase in (Phase.DONE, Phase.ERROR) and sm.mode != "running":
                print("END:", sm.phase.value, sm.error or "", flush=True)
                break
            time.sleep(0.01 if video is None else 0.001)
        for it in world.items:
            print(f"  sock {it.id} ({it.color.value}): {world.location(it.id)[0]}")
        if viewer is not None and not args.no_viewer:
            print("close the viewer window to quit", flush=True)
            while viewer.is_running():
                with world.lock:
                    viewer.sync()
                time.sleep(0.03)
    finally:
        stop.set()
        if video is not None:
            video.release()
            print(f"video: {args.record}")
        system.camera.close()
        world.stop()


if __name__ == "__main__":
    main()
