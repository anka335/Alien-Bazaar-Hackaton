# leo_sim: the Leo Rover in MuJoCo, driven by hand or by jevomir

Block 9 ([task](../../../../docs/tasks/09-rover-navigation.md), [D-034](../../../../docs/decisions.md)). A MuJoCo simulation of the Leo Rover in a furnished room, with a web UI. The rover can be driven by hand, or by **jevomir**: a VLM behind a scoring API (Qwen3.5-4B, closed-choice questions, one forward pass each; see `jevomir/API.md`) that sees only the rover's camera. This is a plain Python project (uv), separate from the ROS package around it: no ROS needed.

## The model

The rover is the official **`leo_description` 3.2.0** (Fictionlab, MIT, [LeoRover/leo_common-ros2](https://github.com/LeoRover/leo_common-ros2/tree/jazzy/leo_description), [Leo Rover docs](https://docs.fictionlab.pl/leo-rover)). Link frames, masses, inertias, joint origins and axes are copied from its `urdf/macros.xacro`. The COLLADA meshes (33 MB, which MuJoCo can't read) are split by material color, decimated and saved as STL in `leo_sim/assets/` (1.3 MB) by `tools/convert_meshes.py`.

| | Sim | Source |
|---|---|---|
| Mass | 5.49 kg (the real rover is ~6.5 kg) | URDF inertials |
| Rockers | free, ±0.24 rad, coupled opposite ways (differential bar) | `leo_sim.urdf.xacro`, mimic joint |
| Wheels | r 0.0625 m, velocity motors, 2 N m | URDF (`effort` 2) |
| Drive | skid-steer, track 0.358 m; `cmd_vel` stops 0.5 s after the last command | leo_gazebo DiffDrive, firmware `input_timeout` |
| Turning | wheel speed difference × 2.2 (`ANGULAR_MULTIPLIER`), measured on this sim | firmware `controller.angular_velocity_multiplier`: 1.76 |
| Odometry | wheels for distance + gyro for heading (like `/merged_odom`) | `odom_filter` |
| Camera `leo` | 0.097 m ahead, 12° down, 109° HFOV, 640×480 | URDF `camera_joint`, leo_gazebo sensor |
| Camera `oak` | 0.13 m ahead, 10° down, 69° HFOV, 640×360 | `../config/mounts.yaml` |

Measured in the tests: a 1 m move ends within 3 cm, turns within 3°. Known gaps: tires are ellipsoids (cylinder rims chatter in MuJoCo while skid-steering), everything except the floor has friction 0.4 so wheels slide off furniture instead of climbing it, and the stock camera's fisheye distortion is left out.

## Run

```bash
cd ros2_ws/src/rover_nav/sim
uv sync                                   # mujoco, numpy, pillow (+ pytest)
uv run pytest                             # ~10 s, headless (EGL)
uv run python -m leo_sim serve            # http://127.0.0.1:8095
```

The page shows the rover's camera (or the OAK-D / a chase view), a ground-truth map with the trail, manual driving (arrows / WASD, 25 cm or 20° per press), a room seed, and the jevomir panel: every image the model saw with its questions and probabilities.

Other commands (`--seed N` before the command picks a random room; default is a fixed layout):

```bash
uv run python -m leo_sim drive --target laundry_basket      # jevomir drives headless, log in runs/<time>/
uv run python -m leo_sim drive --target red_ball --oracle   # same loop, answers from the sim (no API)
uv run python -m leo_sim bench --policy direct               # rooms 0-11, one target each, runs/bench-*/results.json
uv run python -m leo_sim view                               # MuJoCo viewer, arrow keys drive (needs a display)
uv run python -m leo_sim snapshot                           # every camera to snapshots/*.png
```

Headless rendering uses EGL (`MUJOCO_GL=egl`, set by the CLI); without a GPU driver use `MUJOCO_GL=osmesa`.

## Connecting jevomir

Start jevomir's `api_server.py` (needs a CUDA GPU; see its README) and give the sim its URL and key:

```bash
echo 'jev_...' > .api-key                  # git-ignored; or export JEVOMIR_API_KEY=jev_...
uv run python -m leo_sim serve --api-url https://....ngrok-free.dev   # default: http://127.0.0.1:8100
```

The key stays in the Python process; the browser never sees it. Calls are spaced to 90 a minute (`--max-per-minute`, 0 = off) for a free ngrok tunnel.

### How jevomir drives

Each step renders the rover's camera at 448×336 (the API letterboxes to 448×448), asks questions, and makes one closed-loop move (odometry, like `leo-rover-mcp`'s `rover_move` / `rover_turn`).

- **`guided`** (default): the model answers perception questions, fixed rules pick the move.
  1. "Is *the red ball* visible in this photo?" Yes / No. No: turn left 40° (after a full circle, move 0.6 m if "Is the floor straight ahead free of obstacles for at least one meter?" says yes).
  2. "Where is *the red ball* in this photo?" Left side / Center / Right side. Left or right: turn 15° towards it (25° made jevomir swing left-right around the target).
  3. "How far is *the red ball* from the camera?" Less than half a meter / About one meter / Two meters or more. Stop (done), move 0.35 m or move 0.7 m.
- **`direct`**: one question per step, and each option is one move. The prompt (`DIRECT_PROMPT = "describe_half"` in `leo_sim/agent.py`) asks where the target is rather than what to do, because the model sees better than it plans: "This photo is from the front camera of a small wheeled robot. Where is *the red ball*?" In the center of the photo, still far away (0.4 m forward) / In the left half of the photo (20° left) / In the right half (20° right) / In the center and very close, large and low in the picture (stop) / Not in the photo (40° search turn).

**Memory** (`--memory off|control|prompt`, the "memory" list in the UI; `leo_sim/memory.py`): the model sees one photo at a time, so a tracker keeps what the real rover knows too, from its wheel odometry and the model's own past answers (never the sim's ground truth): the moves and their outcomes, the odometry pose, and where the target was last seen, carried through the turns since (an answer of "left half" means ~30° left of that photo's heading, re-expressed from the current heading).

- **`control`**: the agent acts on it, the model gets the plain question. A lost target is searched once towards where it was last seen, then the search keeps turning that way; a turn opposite to the previous one is halved; `direct` drives 0.6 m elsewhere after a full circle without the target.
- **`prompt`**: a few sentences go before every question, e.g. "Robot memory from its wheel odometry (step 4): Last moves, oldest first: turned 20° right; drove 0.40 m forward; drove 0.40 m forward. It last saw the green bin in the previous photo, in the center of the photo; that is roughly straight ahead of where the robot now faces. Position: x 0.74 m, y -0.26 m, heading -19° from the start; 0.8 m driven in total." **This makes jevomir worse**: any text before the question lowers its right moves on the labelled frames (82% without, 64% with only the moves and pose, 46% with a true "last seen" hint, 40% with "not seen yet"), and in the rooms it copies "has not seen it yet" and stops looking. Kept only to compare.

With **both orders** (default) every question is asked again with the options reversed and the probabilities averaged, because the model prefers some letters regardless of content (jevomir `API.md`). A move that ends `stalled` is followed by a 0.2 m back-up and a 45° turn; a tilt above 30° ends the run (`tipped`).

The model never gets the map, the pose or the bumps: only what the real rover would give it. The sim measures them for the score: a run is a **success** when the model says it arrived and the rover's front is within 0.5 m of the target. Each run writes `log.jsonl` (questions, probabilities, action, outcome, pose per step), the images and `map.png`.

The **oracle** answers the same questions from the sim's ground truth (ray-cast line of sight, bearing, distance), so the whole loop and the UI can be tried without a GPU; with it the rules show the ceiling for jevomir.

### Results (2026-09-27, `bench`: rooms 0-11, 40 steps, both orders)

| | oracle | jevomir (Qwen3.5-4B, probe-002) | API calls | wall time |
|---|---|---|---|---|
| `guided` | 9/12 | 5/12 | 986 | 11 min |
| `direct` (`describe_half`) | 10/12 | 7/12 | 584 | 6.5 min |
| `direct`, `--memory control` | 10/12 | 6/12 | 552 | 6 min |
| `direct`, `--memory prompt` | – | 2/12 | 698 | 7.5 min |
| `guided`, `--memory control` | 9/12 | 2 of the first 4 rooms (stopped) | | |

`guided` fails mostly because the model rarely answers "Center" (it swings left-right for 30 steps) and calls the table and the basket not visible. Direct prompts are compared on 50 labelled frames by `tools/eval_direct.py` (`runs/eval-direct/`): the first prompt, "What should it do next?" with Drive forward / Turn left / ..., scored 20% (always forward); `describe` 76%, `describe_half` 82%. Most remaining misses are the laundry basket, which in the sim is a plain open white box. Memory `control` fixed nothing the model gets wrong: with 12 rooms one room is noise (it won room 6 and lost room 4, where the model rarely sees the basket and the sweep kept to the wrong side); the failures left are targets the model doesn't recognise and obstacles it doesn't avoid.

## Files

| Path | What |
|---|---|
| `leo_sim/model.py` | MJCF: rover from the URDF, room, objects (`default_world(seed)`) |
| `leo_sim/rover.py` | `LeoSim`: `set_cmd_vel`, `move`, `turn`, odometry, cameras, top-down map |
| `leo_sim/runner.py` | the sim in its own real-time thread (the GL context is per thread) |
| `leo_sim/jevomir.py` | `JevomirClient` (scoring API), `OracleScorer` |
| `leo_sim/agent.py` | policies and the look-ask-move loop |
| `leo_sim/memory.py` | the path tracker whose text goes before each question (`--memory`) |
| `leo_sim/server.py`, `leo_sim/web/index.html` | web UI |
| `tools/eval_direct.py` | scores the direct prompts on labelled camera frames (real API or `--oracle`) |
| `tools/convert_meshes.py` | rebuilds `leo_sim/assets/` from `leo_description` (`uv run --group meshes ...`) |
