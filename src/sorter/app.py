"""Wiring: build the system from config, start threads, Ctrl+C → hold → shutdown."""

from __future__ import annotations

import importlib
import logging
import threading

from sorter.core.config import Backend, BackendsConfig, Config
from sorter.core.errors import SorterError
from sorter.core.hub import Hub
from sorter.core.log import HubLogHandler
from sorter.core.observer import Observer
from sorter.core.system import System
from sorter.core.types import Command
from sorter.sim import backend as sim_backend
from sorter.sim.world import SimWorld

log = logging.getLogger(__name__)

# component → (package with the real backend, owning block)
COMPONENTS = {
    "camera": ("sorter.camera", 1),
    "arm": ("sorter.arm", 5),
    "calibration": ("sorter.calibration", 2),
    "box_detector": ("sorter.box_detector", 3),
    "color_classifier": ("sorter.color_classifier", 4),
}


def _real(name: str, cfg: Config):
    package, block = COMPONENTS[name]
    try:
        backend = importlib.import_module(f"{package}.backend")
    except ModuleNotFoundError as e:
        if e.name != f"{package}.backend":
            raise
        raise NotImplementedError(
            f"no real {name} backend yet (block {block}); set backends.{name}: sim"
        ) from None
    return backend.create(cfg)


def build_system(cfg: Config, sim: bool = False) -> System:
    """Create every component per `cfg.backends`. `sim=True` forces all of them to sim."""
    if sim:
        cfg = cfg.model_copy(update={"backends": BackendsConfig()})  # all sim
    world = SimWorld(cfg.sim) if Backend.SIM in cfg.backends.model_dump().values() else None
    parts = {
        name: sim_backend.create(name, cfg, world)
        if getattr(cfg.backends, name) is Backend.SIM
        else _real(name, cfg)
        for name in COMPONENTS
    }
    arm = parts["arm"]
    return System(
        cfg=cfg,
        camera=parts["camera"],
        arm=arm,
        calibration=parts["calibration"],
        box_detector=parts["box_detector"],
        color_classifier=parts["color_classifier"],
        observer=Observer(parts["camera"], arm, parts["calibration"]),
        hub=Hub(parts["camera"], on_hold=arm.hold),
        world=world,
    )


def run(cfg: Config, *, sim: bool = False, dashboard: bool = True, autostart: bool = False) -> None:
    from sorter.orchestrator.state_machine import StateMachine

    system = build_system(cfg, sim=sim)
    logging.getLogger("sorter").addHandler(HubLogHandler(system.hub))
    backends = system.cfg.backends.model_dump(mode="json")
    log.info("backends: %s", backends)

    system.camera.start()
    stop = threading.Event()
    sm = StateMachine(system)
    sm_thread = threading.Thread(target=sm.run, args=(stop,), name="state-machine", daemon=True)
    sm_thread.start()

    server = None
    if dashboard:
        import uvicorn

        from sorter.dashboard.server import create_app

        d = system.cfg.dashboard
        server = uvicorn.Server(
            uvicorn.Config(
                create_app(system.hub, d, views=system.cfg.views),
                host=d.host,
                port=d.port,
                log_level="warning",
            )
        )
        threading.Thread(target=server.run, name="dashboard", daemon=True).start()
        print(f"Dashboard: http://{d.host}:{d.port}", flush=True)

    if autostart:
        system.hub.send(Command.START)
    try:
        while sm_thread.is_alive():
            sm_thread.join(0.5)
        log.error("state machine thread exited")
    except KeyboardInterrupt:
        log.warning("Ctrl+C: hold, then shut down")
        system.arm.hold()
    finally:
        stop.set()
        sm_thread.join(timeout=5)
        if server is not None:
            server.should_exit = True
        try:
            system.arm.shutdown()
        except SorterError:
            log.exception("arm shutdown failed")
        system.camera.close()
