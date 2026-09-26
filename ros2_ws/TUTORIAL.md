# Tutorial: set up and run the cloth pick-and-place (ROS 2)

From a fresh machine to the arm picking cloth out of a box and dropping it in a bin. Takes about an hour the first time, a minute after that. For the reference (every launch argument, topic and config key) see [README.md](README.md).

**What runs:** the reBot B601-RS arm (CAN) and a RealSense D435i on its wrist. The task goes: look at the box → SAM3 finds the cloth → approach → straight down → grasp → lift → carry to a bin → release → next cloth, until the box is empty.

> **Safety first.** The arm is strong enough to hurt. Keep the hardware e-stop in reach whenever the motors are on, keep people and their clothes away from the table (the camera looks for *any* clothing), and start every new setup slowly (`execution_speed:=0.2`, `grasp:=false`).

---

## 0. What you need

| Thing | Notes |
|---|---|
| reBot Arm B601-RS | RobStride motors, zero-calibrated, powered. On a table; the base plate is the table top (z = 0) |
| PEAK PCAN-USB | Arm CAN bus. Plug it **straight into the laptop**, not into a USB hub (it stalls behind one) |
| RealSense D435i | Bolted on the joint5 motor cap, looking along the gripper. USB 2 works (we use 640×480 at 15 fps) |
| A box with cloth, and a bin | Box within **~40 cm** of the arm base; the arm can't grasp top-down much further out |
| Hardware e-stop | Cuts arm power. Software stop (Ctrl+C) parks the arm; the e-stop is for everything else |
| StarArm102 / reBot Arm 102 leader ("Spark") | Optional but recommended: drive the arm with it to record poses |
| Laptop | Ubuntu 24.04, ROS 2 Jazzy, internet (SAM3 runs as a web service) |
| SAM3 key | Ask the team. Never commit it |

---

## 1. Install (once per machine)

ROS 2 Jazzy desktop must already be installed (`/opt/ros/jazzy`). Then:

```bash
sudo apt update
sudo apt install -y ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-pilz-industrial-motion-planner ros-jazzy-cv-bridge \
  python3-colcon-common-extensions can-utils
curl -LsSf https://astral.sh/uv/install.sh | sh     # uv, if you don't have it
```

The RealSense needs Intel's udev rules: install `librealsense2-udev-rules` from [Intel's apt repo](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) (already done on the team laptop). The ROS driver itself comes in step 4.

### Get the code

```bash
git clone <repo URL> ~/alizenbazar/Alien-Bazaar-Hackaton
cd ~/alizenbazar/Alien-Bazaar-Hackaton
git checkout feat/ros2-phase1-search     # the branch with ros2_ws/
```

### The arm driver's Python environment

The bridge to the real arm uses the team's `rebot_b601` driver and its `motorbridge` CAN library, from `rebot_b601/.venv` (Python 3.12, like ROS Jazzy):

```bash
cd ~/alizenbazar/Alien-Bazaar-Hackaton/rebot_b601
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

### The SAM3 key

Create `config/local.yaml` in the repo root (it is gitignored; a local pre-commit hook also blocks it):

```yaml
backends:
  color_classifier: real
color_classifier:
  sam:
    api_key: <the SAM3 key>
```

```bash
chmod 600 ~/alizenbazar/Alien-Bazaar-Hackaton/config/local.yaml
```

### Build

```bash
cd ~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws
scripts/setup_deps.sh                    # pydantic 2 + the leader's servo library into .pydeps (no sudo)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install           # --symlink-install matters: the tools edit the config in src/
```

---

## 2. Every new terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/install/setup.bash
```

(Put both lines in `~/.bashrc` if this laptop is only used for this.)

---

## 3. Try it in simulation (no hardware)

Everything is simulated here: arm, gripper, camera. It picks a virtual cloth and drops it at bin 1:

```bash
ros2 launch cloth_task task.launch.py use_rviz:=true
```

You should see in the terminal: `cloth at (…)`, `phase 2 done`, `MISSED` (the simulated gripper misses once on purpose), `cloth held after 2 attempt(s)`, `phase 5 done`, `DONE`. RViz shows the arm, the virtual cloth and the planned approach.

Sorting three cloths by color, until the box is empty:

```bash
ros2 launch cloth_task task.launch.py use_rviz:=true place_target:=color cycles:=0 \
  "sim_cloth_xyz:=[0.38,0.25,0.03, 0.385,0.255,0.03, 0.375,0.26,0.03]" "sim_colors:=[light,dark,colored]"
```

It ends with `DONE: 3 cloth(s) picked and placed`: light to bin 1, dark to bin 2, colored to bin 3. (A virtual cloth must be in view of `box_view`; with the recorded `box_view` that is around x 0.38, y 0.25.)

If this doesn't work, don't go on to the real arm. See [Troubleshooting](#8-troubleshooting).

---

## 4. Connect the hardware (after every boot or replug)

Power the arm and plug in the PCAN-USB, the camera and (optionally) the leader. Close Motorbridge Studio / `motorbridge-gateway` if they run: they hold the CAN bus. Then:

```bash
sudo ~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/scripts/setup_hardware.sh
```

It installs the RealSense ROS driver and `can-utils` the first time, gives your user the leader's USB port (`/dev/rebot_leader`), and brings the CAN bus `can0` up at 1 Mbit/s. It must end with **`can0 is UP (ERROR-ACTIVE)`**. If it says `couldn't set bitrate (err -32)`: unplug the PCAN-USB, plug it straight into the laptop, run it again.

Check the arm answers (read-only, motors stay off):

```bash
cd ~/alizenbazar/Alien-Bazaar-Hackaton/rebot_b601
PYTHONNOUSERSITE=1 .venv/bin/python -m rebot_b601 state
```

It prints the joint angles. The arm folded at home reads close to all zeros. Far-off numbers mean the motor zero calibration is wrong: stop and fix that first (the driver refuses to enable the motors then anyway).

---

## 5. First-time setup on the rig

Do these once, and again whenever the camera, the box or a bin moves.

### 5.1 Bring up the arm without the task

```bash
ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true run_task:=false use_rviz:=true
```

The motors switch on and hold the arm where it is. Nothing moves by itself. **Hand on the e-stop.** Leave this running for the steps below (other terminal).

### 5.2 Calibrate the leader (once, if never done in LeRobot)

Fold the leader into the same shape as the arm at home (all joints 0), gripper closed:

```bash
ros2 run cloth_task leader_teleop --calibrate
```

### 5.3 Drive the arm with the leader and record poses

```bash
ros2 run cloth_task leader_teleop
```

It prints the leader's and the arm's joint angles. Put the leader roughly in the arm's shape first (it warns if they are more than 20° apart), then press **Enter** to engage. The arm follows the leader (up to 80°/s, never into the table or the base).

At the `>` prompt, drive to each pose and type its name:

| Name | Where | Used for |
|---|---|---|
| `box_view` | Camera above the box, box fully in the image, camera ≥ 25 cm above the cloth, box within ~40 cm of the base | Where it looks from, before each grasp |
| `bin_1` (`bin_2`, `bin_3`) | Gripper 10–15 cm above the bin opening, pointing down | Where it drops the cloth |

Enter alone prints the arm's joints and fingertip position (TCP). `q` quits; the arm holds. If a joint moves the wrong way, restart with `--flip <joint>` (e.g. `--flip wrist_roll`).

Without a leader: move the arm in RViz (MotionPlanning, drag the marker, Plan & Execute), then `ros2 run cloth_task record_pose box_view`.

### 5.4 Tell the task where the bins are

In `ros2_ws/src/cloth_task/config/task.yaml`, point the place targets at your poses (a recorded pose name, or an `[x, y, z]` point in metres from the arm base):

```yaml
place_targets:
  1: bin_1
  2: bin_2
  3: bin_3
```

If MoveIt refuses a bin pose (`GOAL_IN_COLLISION`, see troubleshooting), use its TCP point instead, e.g. `1: [0.435, 0.053, 0.167]`.

### 5.5 Mark the box in the camera image

With the arm at `box_view` (drive it there with the leader) and the stack from 5.1 running:

```bash
ros2 run cloth_task roi_tool
```

Click the box corners in the window, **Enter** saves. From now on the detector ignores clothing outside the box.

### 5.6 The camera mount

`ros2_ws/src/rebot_b601_moveit_config/config/camera_mount.yaml` says where the colour lens sits, measured from the fingertip centre with the wrist straight (x forward along the gripper, y left, z up). The shipped values are an estimate. Measure yours with a ruler; every grasp position depends on it (1 cm off here ≈ 1 cm off at the grasp). No rebuild needed after editing. Step 6.1 shows whether it's right.

Stop 5.1 with **Ctrl+C**: the arm parks (lifts, goes home, torque off, ~45 s).

---

## 6. Run it

### 6.1 Dry run: stop above the cloth

```bash
~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/scripts/run_real.sh grasp:=false
```

The arm unfolds, goes to `box_view`, detects the cloth and stops 10 cm above it. Check with a ruler that the fingertips are right above the cloth. The log line `cloth at (x, y, z) m in base_link` is where the camera thinks the cloth is: if that is off from reality, correct `camera_mount.yaml` by about the same amount and repeat.

While it runs, open **http://localhost:8080**: where the task stands (phase, attempt, cloths placed), the detected cloth and its class, the arm's joints and fingers, the live camera, the last SAM3 detection and the log.

### 6.2 The real thing

```bash
~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/scripts/run_real.sh
```

One command: arm, camera, RViz and the task. Defaults: everything into bin 1, until the box is empty, at speed 0.2. Extra arguments override:

```bash
run_real.sh place_target:=color      # light → bin 1, dark → bin 2, colored → bin 3
run_real.sh cycles:=1                # just one cloth
run_real.sh execution_speed:=0.4     # faster (the arm caps at 0.6)
```

The log tells you each step. The grasp check is off (`grasp.check_fingers: false` in `task.yaml`): after the lift the arm always carries on to the bin. To turn it on (retry on a miss), set it to `true`; after the lift the log then prints `fingers at … mm`: note the value for a real miss and a real hold, and set `grasp.empty_below_m` in `task.yaml` between them (default 2 mm is a guess).

### 6.3 Stopping

| How | What happens |
|---|---|
| Ctrl+C in the `run_real.sh` terminal | The arm stops, lifts clear if low, goes home and switches the torque off (~45 s). A second Ctrl+C is ignored until it's done |
| Ctrl+C on a task started with `run_stack:=false` | The current move stops, the task ends, the arm holds |
| Hardware e-stop | Power off. Always works. Support the arm: it falls without power |

---

## 7. Everyday use

After the first-time setup, it's:

```bash
sudo ~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/scripts/setup_hardware.sh   # after a boot / replug
~/alizenbazar/Alien-Bazaar-Hackaton/ros2_ws/scripts/run_real.sh
```

---

## 8. Troubleshooting

| You see | Why | Do |
|---|---|---|
| `can0 is not up` (run_real.sh) | CAN bus down after a boot or replug | `sudo …/scripts/setup_hardware.sh` |
| `couldn't set bitrate (err -32)` | The PCAN-USB adapter stalled | Replug it straight into the laptop, run the script again |
| `no feedback from the motors` | Motors unpowered, CAN cable, or another program holds the bus | Power, cable, close Motorbridge Studio / gateway |
| `joint(s) … outside the allowed range … Refusing to enable` | Motor zero calibration missing | Re-zero the motors (Seeed's procedure) |
| `START_STATE_IN_COLLISION` | MoveIt's box-shaped collision model flags the folded home pose (link2–link4) although the real parts don't touch | `run_real.sh` unfolds the arm first, so this shouldn't happen there. Otherwise drive the arm out of home with the leader. Permanent fix (you decide, after checking on the real arm that those parts can't meet): add `<disable_collisions link1="link2" link2="link4" reason="Default"/>` (and `gripper_end`/`link4` for `bin_1`-like poses) to `rebot_b601_moveit_config/config/rebot_b601.srdf` |
| `GOAL_IN_COLLISION` going to a bin | Same model, e.g. `bin_1` (gripper_end–link4) | Use the bin's TCP point as the place target (5.4) |
| `realsense2_camera is not installed` | Driver missing | `sudo …/scripts/setup_hardware.sh` |
| `.pydeps is missing` / `motorbridge-smart-servo missing` | Python extras not installed | `scripts/setup_deps.sh` |
| `cannot open the leader … Permission denied` | No access to the USB port | `sudo …/scripts/setup_hardware.sh` (udev rule), replug the leader |
| `no box region set` warning | The box isn't marked | 5.5 |
| `got 0/3 cloth detections` | Nothing seen from `box_view` | Is the box in the image (RViz, `/cloth_detector/debug_image`)? Re-record `box_view` (5.3) and re-mark the box (5.5). After at least one cloth was placed this means "box empty" and the run ends normally |
| `bad look (…), looking again` | Detections disagreed or too few, or no color class with `place_target:=color` | Nothing, it retries. If it keeps happening, check lighting and the box region |
| `cloth z=… is outside the workspace` | Detected point outside `approach.workspace` in `task.yaml` (above 30 cm, or too far out) | Usually a wrong camera mount (5.6) or something outside the box. The box region (5.5) prevents the latter |
| `approach: … FAILURE …` | The cloth is out of top-down reach | Box closer to the arm (within ~40 cm), cloth pile lower |
| `MISSED (fingers at 0.0 mm …)` three times | The gripper closes above or beside the cloth | Camera mount (6.1), or tune `grasp.depth_m` / `grasp.empty_below_m` |
| `trajectory ran … slower than planned` | Asked faster than the driver allows | Lower `execution_speed` |
| Arm doesn't follow the leader smoothly | Leader faster than 80°/s, or USB latency | Move the leader slower; check `/dev/rebot_leader` exists |

Logs: the terminal shows everything; per-node files are in `~/.ros/log/`.

---

## 9. Where things are

| File | What |
|---|---|
| `config/local.yaml` (repo root) | SAM3 key. Local only, never commit |
| `ros2_ws/src/cloth_task/config/poses.yaml` | Recorded poses (`box_view`, `bin_1`, …) |
| `ros2_ws/src/cloth_task/config/task.yaml` | Detection, approach, grasp, place targets, workspace |
| `ros2_ws/src/cloth_task/config/detector.yaml` | The box region in the image |
| `ros2_ws/src/rebot_b601_moveit_config/config/camera_mount.yaml` | Where the camera sits |
| `ros2_ws/scripts/` | `setup_hardware.sh` (sudo), `setup_deps.sh`, `run_real.sh` |
| `ros2_ws/README.md` | Reference: all launch arguments, topics, the pipeline in detail, known limits |
