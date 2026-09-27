import math

import mujoco
import pytest
from leo_sim.model import BASE_HEIGHT, build_mjcf, default_world

# leo_description 3.2.0: chassis + 2 rockers + 4 wheels (the 1 g antenna is left out)
URDF_MASS = 1.584994 + 2 * 1.387336 + 4 * 0.283642


def compile_world(seed):
    return mujoco.MjModel.from_xml_string(build_mjcf(default_world(seed)))


def test_rover_matches_the_urdf():
    model = compile_world(None)
    rover = model.body("base_link").id
    bodies = [b for b in range(model.nbody) if model.body_rootid[b] == rover]
    mass = sum(model.body_mass[b] for b in bodies)
    assert mass == pytest.approx(URDF_MASS, abs=0.01)
    assert model.nu == 4  # one velocity motor per wheel
    for name in ("leo", "oak", "chase", "map"):
        model.camera(name)


def test_rover_rests_on_its_wheels():
    model = compile_world(None)
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert data.body("base_link").xpos[2] == pytest.approx(BASE_HEIGHT, abs=0.003)


@pytest.mark.parametrize("seed", range(20))
def test_seeded_rooms_keep_solids_apart(seed):
    world = default_world(seed)
    solids = [o for o in world.objects if o.solid]
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            assert math.hypot(a.x - b.x, a.y - b.y) > a.radius + b.radius
        assert math.hypot(a.x - world.start[0], a.y - world.start[1]) > a.radius + 0.45
        assert abs(a.x) + a.radius < world.width / 2 and abs(a.y) + a.radius < world.depth / 2


def test_seed_is_reproducible():
    a, b = default_world(5), default_world(5)
    assert [(o.x, o.y) for o in a.objects] == [(o.x, o.y) for o in b.objects]
    assert a.start == b.start
    assert [(o.x, o.y) for o in default_world(6).objects] != [(o.x, o.y) for o in a.objects]
