"""Read-only check: motors OFF. Camera frame + SAM3 at the current pose, targets in base frame."""
import os
os.environ.setdefault("REBOT_Z_MIN", "0.005")
import cv2
from ez_arm.detect import ShirtDetector
from ez_arm.pick_place import OUT_DIR
from ez_arm.robot import Robot

r = Robot(enable=False, camera=True)
try:
    print("state", r.state())
    rgbd, q = r.snap()
    det = ShirtDetector()
    targets, dbg = det.detect(rgbd, q)
    os.makedirs(OUT_DIR, exist_ok=True)
    cv2.imwrite(os.path.join(OUT_DIR, "check.jpg"), dbg)
    insts = sum(len(det.seg.segment(rgbd.color)) for _ in [0])
    print("valid depth px", int((rgbd.depth_m > 0).sum()), "targets on table:", [(t.prompt, t.xyz.round(3).tolist()) for t in targets])
finally:
    r.close(go_home=False)
