"""The unload loop end to end on the simulator, on the rover's layout, with the station off its
place: judged by where the sock really lands."""

import math

import pytest

from sorter.app import build_system
from sorter.core.types import ColorClass, Command, OperatorMode, Phase

pytest.importorskip("mujoco")


def test_unloads_a_sock_into_its_bin_found_by_the_camera():
    from sorter.orchestrator.state_machine import StateMachine
    from sorter.sim.scenes.unload.bench import _truth_bins, _where, unload_config

    cfg = unload_config(
        {
            "sim": {
                "realtime": 0,
                "seed": 3,
                "unload": {
                    "cargo": {"light": 0, "dark": 0, "colored": 1},
                    "sock_mm": [200, 90],
                    "station_mm": 30,
                    "station_deg": 5,
                    "bin_mm": 15,
                    "bin_deg": 8,
                },
            },
            "arm": {"speed_scale": 1.4},
            "state_machine": {"save_runs": False},
        }
    )
    s = build_system(cfg, sim=True)
    world = s.world
    try:
        s.camera.start()
        sm = StateMachine(s)
        truth = _truth_bins(world)
        s.hub.set_mode(OperatorMode.UNLOAD)
        s.hub.send(Command.START)
        sm.poll()
        t0 = world.time()
        while sm.mode != "idle" and sm.phase is not Phase.ERROR and world.time() - t0 < 200:
            sm.poll()
        assert sm.phase is Phase.DONE, sm.error
        assert _where(world, 0, truth) == ("laundry", "colored")
        assert sm.counters[ColorClass.COLORED] == 1
        loop = sm.loops[OperatorMode.UNLOAD]
        for color, b in loop.bins.items():
            assert b.found and math.dist(b.center, truth[color][:2]) < 5
    finally:
        s.camera.close()
        world.stop()


def _seen(color, xy):
    from sorter.box_detector.cargo import SockSeen

    return SockSeen(color, 1.0, (50.0, 0.0, 0.0), xy, 800)


def _target(color, xy):
    from sorter.box_detector.cargo import SockTarget
    from sorter.core.types import ArmPoint, GraspPoint, PixelPoint

    return SockTarget(
        color, 1.0, GraspPoint(PixelPoint(0, 0), 300.0), ArmPoint(*xy, 0.0), [0.0], 20.0, 900
    )


def _held(*classes):
    from sorter.box_detector.held import HeldView
    from sorter.core.types import Overlay

    return HeldView(None, 0.0, 0, Overlay(), len(classes), None, list(classes))


@pytest.mark.parametrize(
    ("held", "gone", "target", "want"),
    [
        # seen hanging wins over the box
        ((ColorClass.DARK,), [ColorClass.DARK], ColorClass.LIGHT, ColorClass.DARK),
        ((ColorClass.DARK,), [], ColorClass.LIGHT, ColorClass.DARK),
        # one seen, two of two colors gone: both may hang as one, back into the box
        ((ColorClass.DARK,), [ColorClass.DARK, ColorClass.LIGHT], ColorClass.DARK, None),
        # different colors hanging: back into the box
        ((ColorClass.DARK, ColorClass.LIGHT), [], ColorClass.DARK, None),
        # not in view: the one sock gone from the box
        ((), [ColorClass.COLORED], ColorClass.LIGHT, ColorClass.COLORED),
        # several gone: the one nearest the grasp (the first gone sock lies at the grasp)
        ((), [ColorClass.LIGHT, ColorClass.DARK], ColorClass.COLORED, ColorClass.LIGHT),
        # none gone (the pile hid the change): the grasp's target
        ((), [], ColorClass.COLORED, ColorClass.COLORED),
        ((), [], None, None),
    ],
)
def test_held_color_falls_back_to_the_box_best_match(held, gone, target, want):
    from sorter.orchestrator.unload import UnloadLoop

    at = (-100.0, 240.0)
    seen = [_seen(c, (at[0] + 40.0 * k, at[1])) for k, c in enumerate(gone)]
    t = _target(target, at) if target is not None else None
    color, why = UnloadLoop._held_color(_held(*held), seen, t)
    assert color == want, why
