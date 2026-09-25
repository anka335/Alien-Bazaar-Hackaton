# Decision Log

Short records of design decisions. Newest at the bottom. To reverse a decision, add a new entry that references the old one; don't edit history.

Template:

```markdown
## D-NNN: Title (YYYY-MM-DD)
**Context:** why a decision was needed.
**Decision:** what we chose.
**Consequences:** what follows from it, trade-offs accepted.
```

---

## D-001: Classic CV + depth instead of trained models (2026-09-25)

**Context:** Hackathon time budget, no dataset, and a fully controlled scene (fixed camera, lamp, uniform background).
**Decision:** Box detection and color classification use classic computer vision (reference-image differencing, depth, color statistics, thresholds), not trained ML models.
**Consequences:** No data collection or training. Thresholds are tuned on the actual demo clothes. A model may be tried only if time remains and the baseline works.

## D-002: Vision outputs pixel coordinates (2026-09-25)

**Context:** The original plan had vision blocks output arm coordinates.
**Decision:** Vision modules return pixel coordinates (plus depth where relevant). Conversion to arm coordinates lives only in the calibration module.
**Consequences:** Vision can be developed and tested on recorded frames without the arm or calibration. There is one place to fix when the calibration changes.

## D-003: Block 0, contracts and stubs first (2026-09-25)

**Context:** The original dependency list was contradictory: blocks 3–5 depended on 1–2, yet 4, 5, 7 were supposed to start right away.
**Decision:** Add block 0, which fixes the stack, repo layout, interfaces, and stubs/mocks before parallel work starts.
**Consequences:** Blocks 3–7 run in parallel against stubs. Blocks 1–2 are needed only for real-hardware integration.

## D-004: Depth camera (2026-09-25)

**Context:** The team has an RGB-D camera.
**Decision:** Use the depth stream for grasp selection in the box (pick point and grasp depth) and for empty-box detection. Calibration may use a 3D camera→arm transform instead of a planar homography (final choice in block 2).
**Consequences:** Removes the "unknown grasp depth" and "parallax on pile height" problems of a 2D setup. Depth must be aligned to the color stream.

## D-005: Stack: Python, single process (2026-09-25)

**Context:** The arm SDK (`reBotArm_control_py`) is Python with a uv project; the dashboard can run on the same laptop.
**Decision:** Python 3.12, uv, pytest, ruff; FastAPI + a single static page for the dashboard. One process with threads (camera, SDK control loop, state machine, web server).
**Consequences:** No IPC or serialization between modules; the dashboard's hold reaches the arm by a direct call. The SDK control loop shares the GIL with vision and streaming. In `posvel` mode the motors hold their targets on their own, so jitter only affects smoothness.

## D-006: Camera on the wrist (eye-in-hand) (2026-09-25)

**Context:** D-004 assumed an overhead camera. The Seeed grasping demo for this arm is eye-in-hand and ships a hand-eye calibration script (ArUco, Tsai). An overhead stand is extra mechanics, and any bump or transport breaks its calibration.
**Decision:** Mount the RGB-D camera rigidly on the wrist behind the gripper. The arm moves to a fixed look pose per zone before each decision frame. Pixel → arm conversion is `T_base_flange(q) · T_flange_cam · deproject(u, v, depth)`. Supersedes the overhead mounting and the planar-homography option of D-004; depth use from D-004 stays.
**Consequences:** Block 2 reuses the Seeed hand-eye script instead of writing calibration from scratch; the result survives transport. No camera stand. The live feed moves with the arm, so the dashboard's main panel is the decision frame. The camera descends with the gripper, so grasp points need a wall margin that covers the camera. The mount must be rigid.

## D-007: Fixed zones; rig config committed (2026-09-25)

**Context:** The box, background area, and bins are always at the same positions; only the clothes inside the box vary. There is one physical rig.
**Decision:** Look poses, place and bin poses, zone workspaces (`config/rig.yaml`), and the hand-eye transform (`config/hand_eye.yaml`) are constants in committed config. This is the exception allowed by `AGENTS.md` for these two files.
**Consequences:** Pixel ROIs per zone are constants (look poses are repeatable). Place and drop need no vision. Recalibration is only needed if the camera mount or the zone layout changes.

## D-008: Observation-driven loop, stateless vision (2026-09-25)

**Context:** The loop has to handle missed grasps, double grasps, failed drops, and restarts after a hold without special cases for each.
**Decision:** Every cycle starts by looking at the background; the box is visited only when the background is empty. The color classifier returns all blobs; a drop is verified when the blob count decreases. The box detector is stateless: failed grasp pixels are passed in as `avoid`.
**Consequences:** Double grasps are sorted one item per cycle. After any hold or error the loop resumes from `LOOK_BG`. Vision is testable on single frames. An item dropped outside the camera's view (on the way to a bin) is not detected.

## D-009: Software stop is hold, not disable (2026-09-25)

**Context:** The SDK's `estop()` disables the motors, and the arm falls onto the table and the clothes.
**Decision:** The dashboard stop button and Ctrl+C call `hold()` (freeze joint targets at the current position). A hardware e-stop switch cuts power when needed. Shutdown moves to a `rest` pose before disabling.
**Consequences:** Stopping is safe for the hardware in normal cases. After a hold, `recover()` is needed before any motion.
