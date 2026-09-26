"""Physics sim backend factory: only the hardware (camera, motors) and the rig data the sim
knows exactly (the hand-eye result). Vision and calibration run their real code on top."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from sorter.arm.controller import Controller
from sorter.arm.driver import RebotDriver
from sorter.calibration.calibration import HandEyeCalibration
from sorter.sim.physics.camera import PhysicsCamera
from sorter.sim.physics.motors import MujocoBackend
from sorter.sim.physics.world import PhysicsWorld
from sorter.sim.world import camera_mount

if TYPE_CHECKING:
    from sorter.core.config import Config


def hand_eye(cfg: Config):
    """T_link5_cam of the simulated camera mount (what the hand-eye tool should find)."""
    return camera_mount(cfg.sim)


def create(name: str, cfg: Config, world: PhysicsWorld) -> Any:
    match name:
        case "camera":
            if world.camera is None:
                world.camera = PhysicsCamera(world)
            return world.camera
        case "arm":
            from rebot_b601.arm import Arm

            arm = Arm(clock=world.time)
            world.attach(arm.tick, arm.hz)
            driver = RebotDriver(
                max_speed_scale=cfg.arm.max_speed_scale,
                arm=arm,
                backend=MujocoBackend(world),
                own_loop=False,
            )
            return Controller(driver, cfg.arm, cfg.poses, cfg.zones)
        case "calibration":  # the real calibration with the sim's exact camera mount
            return HandEyeCalibration(hand_eye(cfg))
    raise ValueError(
        f"the physics simulator has no stand-in for {name}: set backends.{name}: real "
        "(it runs on the rendered frames)"
    )
