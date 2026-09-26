"""Look and view poses: the camera over a spot, with the gripper vertical."""

import numpy as np

from sorter.arm import kinematics as kin
from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.sim.layout import axis_hit, camera_over

# the rig's mount (2026-09-26): the camera ~100 mm out from the TCP, away from the arm's base
RIG_MOUNT = np.array(
    [
        [0.105649, 0.046011, 0.993339, 72.28547],
        [0.984075, -0.148437, -0.097788, -18.485238],
        [0.142949, 0.987851, -0.06096, -51.230866],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def test_camera_over_as_close_as_the_arm_can():
    cfg = load_config(DEFAULT_CONFIG_DIR)
    box = cfg.sim.layout.box
    z = box.floor_z_mm
    # centering the camera would take the TCP closer to the base than it reaches at this height
    assert camera_over(cfg.arm, box.center_mm, 110, RIG_MOUNT, z) is None
    q = camera_over(cfg.arm, box.center_mm, 110, RIG_MOUNT, z, exact=False)
    assert q is not None
    hit = axis_hit(kin.fk_link5(q) @ RIG_MOUNT, z)
    assert np.hypot(*(hit - np.asarray(box.center_mm))) < 70  # the TCP over the center: ~100
    # low enough, it centers it (within the IK's reach limits)
    q = camera_over(cfg.arm, box.center_mm, 70, RIG_MOUNT, z, exact=False)
    hit = axis_hit(kin.fk_link5(q) @ RIG_MOUNT, z)
    assert np.hypot(*(hit - np.asarray(box.center_mm))) < 5
