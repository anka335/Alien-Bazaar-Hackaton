"""Wiring: build the system from config, start threads, Ctrl+C → hold → shutdown."""

from __future__ import annotations

import importlib
import logging
import threading
from pathlib import Path

from sorter.core.config import Backend, Config
from sorter.core.errors import SorterError
from sorter.core.hub import Hub
from sorter.core.log import HubLogHandler
from sorter.core.observer import Observer
from sorter.core.system import System
from sorter.core.types import Command, Zone
from sorter.dashboard.twin import Twin
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


HARDWARE = ("camera", "arm")


def build_system(cfg: Config, sim: bool = False) -> System:
    """Create every component per `cfg.backends`. `sim=True` simulates the hardware (camera and
    arm); the rest follows `backends`.

    `sim.engine: physics` (MuJoCo) simulates only the hardware: calibration, box detector and
    color classifier run their real code on the rendered frames. `kinematic` also has quick
    stand-ins for them (tests).
    """
    if sim:
        backends = cfg.backends.model_copy(update=dict.fromkeys(HARDWARE, Backend.SIM))
        cfg = cfg.model_copy(update={"backends": backends})
    simulated = [n for n in COMPONENTS if getattr(cfg.backends, n) is Backend.SIM]
    world = None
    if simulated and cfg.sim.engine == "physics":
        from sorter.sim.physics import backend as physics_backend
        from sorter.sim.physics.world import PhysicsWorld

        world = PhysicsWorld(cfg.sim, cfg.poses, board=cfg.sim.board)
        make_sim = physics_backend.create
    elif simulated:
        world = SimWorld(cfg.sim, cfg.poses)
        make_sim = sim_backend.create
    parts = {}
    for name in COMPONENTS:  # in order: the camera comes before what depends on it
        if name in simulated:
            parts[name] = make_sim(name, cfg, world)
        elif cfg.sim.engine == "physics" and "camera" in simulated and name in _PHYSICS_RIG:
            parts[name] = _PHYSICS_RIG[name](cfg, parts)
        else:
            parts[name] = _real(name, cfg)
    if world is not None and not isinstance(world, SimWorld):
        world.start()  # physics: the scene runs from now on, like the real world
    arm = parts["arm"]
    twin = Twin(arm, parts["calibration"], cfg, world)
    return System(
        cfg=cfg,
        camera=parts["camera"],
        arm=arm,
        calibration=parts["calibration"],
        box_detector=parts["box_detector"],
        color_classifier=parts["color_classifier"],
        observer=Observer(parts["camera"], arm, parts["calibration"]),
        hub=Hub(parts["camera"], on_hold=arm.hold, twin=twin),
        world=world,
    )


# On the simulated camera the real calibration and color classifier use what the sim knows
# exactly: the camera mount (config/hand_eye.yaml belongs to the real camera) and the rendered
# segmentation (in place of the SAM3 service, unless `sim.use_sam3`).


def _physics_calibration(cfg: Config, parts: dict):
    from sorter.calibration.calibration import HandEyeCalibration
    from sorter.sim.physics.backend import hand_eye

    return HandEyeCalibration(hand_eye(cfg))


def _physics_classifier(cfg: Config, parts: dict):
    if cfg.sim.use_sam3:
        return _real("color_classifier", cfg)
    from sorter.color_classifier.classifier import Sam3ColorClassifier

    view = cfg.views.get(Zone.BACKGROUND)
    return Sam3ColorClassifier(
        cfg.color_classifier, parts["camera"].segment, view.roi if view else ()
    )


_PHYSICS_RIG = {"calibration": _physics_calibration, "color_classifier": _physics_classifier}


def _serve(system: System, manual=None):
    """Start the dashboard's web server in a thread; returns the uvicorn server."""
    import uvicorn

    from sorter.dashboard.server import create_app

    d = system.cfg.dashboard
    server = uvicorn.Server(
        uvicorn.Config(
            create_app(system.hub, d, views=system.cfg.views, manual=manual),
            host=d.host,
            port=d.port,
            log_level="warning",
        )
    )
    server.thread = threading.Thread(target=server.run, name="dashboard", daemon=True)
    server.thread.start()
    print(f"Dashboard: http://{d.host}:{d.port}{'/manual' if manual else ''}", flush=True)
    return server


def _close(system: System) -> None:
    try:
        system.arm.shutdown()
    except SorterError:
        log.exception("arm shutdown failed")
    system.camera.close()
    if system.world is not None and hasattr(system.world, "stop"):
        system.world.stop()


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

    server = _serve(system) if dashboard else None

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
        _close(system)


def _nominal_hand_eye(cfg: Config) -> Config:
    """Setup mode comes before the hand-eye calibration: without `config/hand_eye.yaml` the 3D
    view uses the nominal camera mount (`sim.camera_mount_mm`). Nothing in it targets by pixels."""
    from sorter.arm import kinematics as kin
    from sorter.calibration.config import HandEyeResult
    from sorter.sim.world import camera_mount

    log.warning("no config/hand_eye.yaml: manual control uses the nominal camera mount")
    T = kin.T_FLANGE_TCP @ camera_mount(cfg.sim)
    he = HandEyeResult(T_flange_cam=T.tolist(), method="nominal")
    calibration = cfg.calibration.model_copy(update={"hand_eye": he})
    return cfg.model_copy(update={"calibration": calibration})


def run_manual(cfg: Config, *, sim: bool = False, rig_file: Path) -> None:
    """Setup mode: the camera, the arm and the manual control page, no state machine. The arm
    only moves on a button press. Ctrl+C → hold → rest pose → motors off."""
    from sorter.dashboard.manual import ManualControl

    if not sim and cfg.calibration.hand_eye is None:
        cfg = _nominal_hand_eye(cfg)
    system = build_system(cfg, sim=sim)
    logging.getLogger("sorter").addHandler(HubLogHandler(system.hub))
    log.info("manual control, backends: %s", system.cfg.backends.model_dump(mode="json"))
    system.camera.start()
    system.arm.start()
    manual = ManualControl(system.arm, rig_file, system.cfg.arm.gripper.open)
    server = _serve(system, manual)
    try:
        while server.thread.is_alive():  # it exits if it can't bind the port
            server.thread.join(0.5)
        log.error("dashboard server exited (port %s in use?)", system.cfg.dashboard.port)
    except KeyboardInterrupt:
        log.warning("Ctrl+C: hold, then shut down")
        system.arm.hold()
    finally:
        server.should_exit = True
        _close(system)
