"""Watch the unload loop at work: one seeded scenario on the real rover's geometry, live in the
MuJoCo viewer (and the dashboard), then where every sock really ended.

    uv run mjpython -m sorter.sim.scenes.unload.demo [--seed 3] [--socks 2,2,2] [--speed 1]
    uv run python -m sorter.sim.scenes.unload.demo --no-viewer --dashboard   # the browser only

macOS needs `mjpython` for the viewer. The terminal shows each decision of the loop; the
dashboard (http://127.0.0.1:8001, `--dashboard`) shows the wrist camera and the decision frames.
"""

from __future__ import annotations

import argparse
import logging
import threading
import time

from sorter.core.types import Command, OperatorMode, Phase


def _report(system) -> None:
    from sorter.sim.scenes.unload.bench import _truth_bins, _where

    world = system.world
    truth = _truth_bins(world)
    right = 0
    print("\nwhere every sock ended (the simulator's truth):")
    for it in world.items:
        at, where = _where(world, it.id, truth)
        ok = at == "laundry" and where == it.color.value
        right += ok
        place = f"{at} {where}" if where else at
        print(f"  {'OK ' if ok else '-- '} {it.color.value:8s} sock {it.id}: {place}")
    print(f"{right}/{len(world.items)} in the bin of their color")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--seed", type=int, default=3)
    p.add_argument("--socks", default="2,2,2", help="per color: light,dark,colored")
    p.add_argument("--speed", type=float, default=1.0, help="sim seconds per second, 0 = max")
    p.add_argument("--station-mm", type=float, default=30.0)
    p.add_argument("--station-deg", type=float, default=5.0)
    p.add_argument("--no-viewer", action="store_true")
    p.add_argument("--dashboard", action="store_true", help="also serve the web dashboard")
    p.add_argument("--port", type=int, default=8001, help="the dashboard's (8000: `sorter run`)")
    a = p.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    from sorter.app import _serve, build_system
    from sorter.orchestrator.state_machine import StateMachine
    from sorter.sim.scenes.unload.rover import rover_config

    socks = [int(v) for v in a.socks.split(",")]
    cfg = rover_config(
        {
            "sim": {
                "seed": a.seed,
                "realtime": a.speed,
                "unload": {
                    "cargo": dict(zip(("light", "dark", "colored"), socks, strict=True)),
                    "sock_mm": [200, 90],
                    "station_mm": a.station_mm,
                    "station_deg": a.station_deg,
                    "bin_mm": 15,
                    "bin_deg": 8,
                },
            },
            "arm": {"speed_scale": 1.4},
            "state_machine": {"save_runs": False},
            "dashboard": {"port": a.port},
        }
    )
    system = build_system(cfg, sim=True)
    world = system.world
    system.camera.start()
    stop = threading.Event()
    sm = StateMachine(system)
    threading.Thread(target=sm.run, args=(stop,), name="state-machine", daemon=True).start()
    if a.dashboard:
        _serve(system)
    system.hub.set_mode(OperatorMode.UNLOAD)
    system.hub.send(Command.START)

    last = [None]

    def narrate() -> bool:
        """Print a new decision; True once the run is over."""
        d = system.hub.decision()
        if d is not None and d is not last[0]:
            last[0] = d
            print(f"{world.time():7.1f} s  {d.phase.value:16s} {d.summary}", flush=True)
        if sm.run_id is None:  # START not taken yet
            return False
        return sm.phase is Phase.ERROR or (sm.mode == "idle" and sm.phase is Phase.DONE)

    try:
        if a.no_viewer:
            while not narrate():
                time.sleep(0.2)
        else:
            import mujoco
            import mujoco.viewer

            # the viewer draws its own copy: the physics thread steps world.data meanwhile
            shown = mujoco.MjData(world.model)
            with world.lock:
                mujoco.mj_copyData(shown, world.model, world.data)
            with mujoco.viewer.launch_passive(world.model, shown) as viewer:
                viewer.cam.lookat[:] = (0.05, 0.03, -0.08)
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.3, 215, -30
                done = False
                while viewer.is_running():
                    with world.lock:
                        mujoco.mj_copyData(shown, world.model, world.data)
                    viewer.sync()
                    if not done and narrate():
                        done = True
                        print(f"run over: {sm.phase.value} {sm.error or ''}")
                        _report(system)
                        print("close the viewer window to quit")
                    time.sleep(1 / 30)
                if not done:
                    return
        if a.no_viewer:
            print(f"run over: {sm.phase.value} {sm.error or ''}")
            _report(system)
    finally:
        stop.set()
        system.camera.close()
        world.stop()


if __name__ == "__main__":
    main()
