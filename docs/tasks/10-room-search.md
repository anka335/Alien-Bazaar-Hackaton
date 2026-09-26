# Block 10: Room Search

**Status:** todo (planned) · **Owner:** Slava + Claude · **Branch:** `block/10-room-search`
**Owned paths:** `ros2_ws/src/rover_nav/` (search nodes and tools, shared with block 9), `~/rover_nav_maps/places.yaml` (machine-specific, not committed)

## Goal

The rover searches the allowed part of the room for pieces of clothing, drives up to each one, lets the arm pick it up from the floor, carries it to the **main laundry box** (one of four boxes next to the washing machine, always at the same place), the arm throws it in, and the rover goes on searching until the room is clear ([D-021](../decisions.md), proposed). The rover must never drive into any of the boxes. Builds on block 9's map, localization, Nav2 and keepout mask. Obstacle avoidance stays simple: what Nav2 already does, plus skipping what can't be reached.

## How it works

```text
          ┌───────────────────────────── next viewpoint ◄───────────────────────────┐
          ▼                                                                          │
 GO_TO_VIEWPOINT ─► LOOK_AROUND ──no cloth──► (viewpoints left?) ──no──► DONE        │
                        │ cloth(es) seen: queue their map positions                  │
                        ▼                                                             │
                   APPROACH_CLOTH ─► PICK (arm, wrist camera) ─► GO_TO_BOX ─► DROP (arm)
                        │ unreachable / pick failed ×2: mark ignored ────────────────┘
```

1. **Viewpoints** are generated once from the saved map and the keepout mask (plain Python, like `make_keepout`): a grid of points about 1.2 m apart on free cells, at least ~0.4 m from obstacles, unknown space and the keepout areas, visited in nearest-first order from the rover's position.
2. **Look around:** at each viewpoint the rover stops and faces 4 directions (90° apart; the OAK-D sees ~70°). At each heading the search detector runs on a few OAK-D frames. Every cloth it finds becomes a point in the `map` frame. Points in the keepout areas are ignored; points within 0.3 m of a known cloth are the same cloth.
3. **Approach:** Nav2 drives to a parking pose facing the cloth, with the cloth ~0.32 m ahead of the arm's base (inside the arm's floor reach, see *Pick geometry*). The parking pose is computed from the cloth point and the rover's current position; if Nav2 can't reach it, two other angles around the cloth are tried, then the cloth is marked ignored.
4. **Pick:** the rover stands still and the arm stack picks: arm to a `floor_view` pose, wrist D435i + SAM3 finds the cloth again (the OAK-D can't see that close), straight-down grasp, lift to a `carry` pose. Failure → one more try, then ignored.
5. **Laundry box:** Nav2 drives to the box's stop pose (in `places.yaml`), which leaves the rover's front **20 cm** before the main box, facing it. The arm throws the cloth into the box (`drop_box` pose), back to `drive`. The cloth is struck off; the search continues from the nearest unvisited viewpoint.
6. **Done** when all viewpoints were visited and the cloth queue is empty. Optional second sweep (`sweeps: 2`) for clothes dropped or moved meanwhile.

## Obstacle avoidance (simple)

Already there from block 9, nothing new to build:

- Nav2's local costmap marks anything 5–90 cm high seen by the OAK-D depth (people, bags, chairs moved since mapping); the planner goes around it, the RPP controller slows down and stops before a collision.
- Nav2's recovery behaviors: clear costmaps, wait, back up, spin.

Added by the search node, all plain rules:

- A goal that fails (Nav2 aborts, or no progress for 30 s) is retried once; then the viewpoint is skipped or the cloth ignored, and the search moves on.
- Known cloth points are never used as viewpoints or parking spots on top of a cloth; clothes on the floor are lower than 5 cm and are not obstacles to Nav2, so the rover must not drive over one it hasn't picked yet (accepted risk for unseen ones).

## The laundry boxes

- Four boxes stand next to the washing machine; the **main box** (the one the clothes go into) at about **x = 1.0, y = −2.0**. It is inside the allowed area (above y = −3.5, clear of the chair strip at x −1.3 … −0.7).
- The saved map shows free floor at (1, −2) and a line of obstacles just east of it (x ≈ 1.25–1.45, y −1.3 … −2.9, probably the machine / boxes / a wall): the box wasn't there during mapping. The rover therefore approaches **from the west, facing +x**.
- **Stop pose:** rover front 0.20 m before the box's near face. The footprint's front is 0.27 m ahead of the rover's centre, so with a ~0.4 m deep box (near face at x ≈ 0.8) the stop pose is about **(0.33, −2.0, yaw 0)**. Placeholder until the box is in place; then drive there, check the 20 cm with a ruler, and save the pose with `record_place laundry_box`.
- **Not driving into boxes:** the boxes are added to the keepout mask with `make_keepout --forbid-rect` (each box's outline + 5 cm) once they stand where they'll stay. Nav2 then never plans into them, even inside the OAK-D's blind zone (the floor is visible only from ~0.4 m ahead, so a box 20 cm ahead is not seen any more). The last 0.5 m to the stop pose is driven slowly, straight on (RPP's approach slowdown).
- The stop pose is outside the box's keepout rectangle but inside its inflation: allowed, just slower.

## Pick geometry (checked offline with `rebot_b601` IK)

- Floor is ~0.27 m below the arm's base (`leo/base_link` is 0.198 m above the floor, arm base +0.07 m, placeholder).
- Straight-down grasps at floor level work from **0.25 to 0.40 m** ahead of the arm's base (0.45 m fails). Target **0.32 m**: ~0.16 m ahead of the front wheels.
- The OAK-D sees the floor only from ~0.4 m ahead of the rover, so the final detection is the wrist camera's; the OAK-D's job is to find the cloth from 0.5–3 m.
- Today the arm stack refuses these targets: see *Requests from other blocks*.

## Later: which camera finds the clothes

The arm's wrist camera (D435i) also detects laundry (cloth_task's detector). For now the OAK-D searches and the wrist camera only confirms and grasps. Options for later, not in this plan:

- search with the wrist camera instead: the arm scans from a `search_view` pose at each viewpoint (higher viewpoint, sees further over clutter; slower, the arm moves at every heading);
- use both: OAK-D for navigation and obstacles, detections from both cameras merged in the `map` frame (more coverage, more SAM3 requests).

## Components (all in `ros2_ws/src/rover_nav`, launched by `search.launch.py` on top of `navigation.launch.py`)

| Piece | What | Test without the rover |
| --- | --- | --- |
| `rover_nav/viewpoints.py` | map + keepout → viewpoint list; nearest-first order | pytest on synthetic maps |
| `rover_nav/search_logic.py` | pure state machine: cloth queue, de-duplication, keepout check, parking pose, retry/ignore rules | pytest, no ROS |
| `room_search` node | runs the logic with `nav2_simple_commander.BasicNavigator` (`goToPose`, `spin`), the detector and the arm services; publishes `/room_search/phase` + markers | on the rover with the arm faked |
| search detector | a second `cloth_detector` instance (cloth_task, unchanged) on the OAK-D topics, outputs remapped under `/search/…` | the laptop + OAK-D alone |
| `fake_arm` node | answers the arm services after a delay (for testing search + box trips before the arm is ready) | — |
| `record_place` tool | saves the rover's current map pose as a named place (`laundry_box`) into `places.yaml` | — |

## Interfaces (proposed; to agree with the arm team)

| Name | Type | Who |
| --- | --- | --- |
| `/search/cloth_detector/enable` | `std_srvs/SetBool` | room_search → search detector (on only while the rover stands still at a heading) |
| `/search/cloth_target_pose` | `geometry_msgs/PoseStamped`, OAK-D optical frame | search detector → room_search (transformed to `map` at the image stamp) |
| `/arm/pick_from_floor` | `std_srvs/Trigger` (long call, ~1 min) | room_search → arm stack: floor_view, detect with the wrist camera, grasp, lift to `carry`; `success` = holding |
| `/arm/drop_in_box` | `std_srvs/Trigger` | room_search → arm stack: throw the held cloth into the box ahead (`drop_box` pose, box 0.20 m ahead of the rover's front), back to `drive` |
| `/arm/stow` | `std_srvs/Trigger` | room_search → arm stack: go to `drive` (before any rover motion) |
| `places.yaml` | `laundry_box: {x, y, yaw}`: the rover's stop pose before the main box, `map` frame | recorded once per map |

The rover never moves while an arm service runs, and the arm never moves while the rover drives (`drive` / `carry` poses are compact and keep the OAK-D view free).

## Scope

- [ ] `viewpoints.py` + tests; preview image of the viewpoints on the map (like `keepout_preview.png`).
- [ ] `search_logic.py` + tests (queue, de-dup, keepout check, parking poses, retries).
- [ ] `record_place` tool; record `laundry_box` (stop pose, 20 cm before the main box).
- [ ] Boxes in the keepout mask (`--forbid-rect` per box) once they are in place.
- [ ] Search detector on the OAK-D (second `cloth_detector` instance, remapped); check SAM3 finds floor clothes at 0.5–3 m from the low camera.
- [ ] `room_search` node + `search.launch.py`, with `fake_arm`: full loop on the rover (search → approach → "pick" → laundry box → "drop" → continue).
- [ ] Swap `fake_arm` for the arm stack once it offers the three services.
- [ ] Tune: viewpoint spacing, headings, parking distance, speeds.

## Out of scope

Autonomous exploration of unknown space (the map exists), mapping other rooms, grasp planning itself (arm stack), the washing machine itself (clothes go into the box).

## Depends on / Unblocks

- Depends on: block 9 (map, localization, Nav2, keepout), the SAM3 service (internet on the laptop), the arm stack's floor pick and drop into the box (*Requests*).
- Unblocks: the full demo.

## Acceptance criteria

- With 3 clothes on the floor of the allowed area and the arm faked: all 3 found, each approached to within 5 cm of the parking pose, 3 box trips each ending 0.20 ± 0.05 m before the main box, `DONE`; no contact with any box; no entry into the keepout areas; a person stepping in the way makes the rover stop or go around, never collide.
- With the arm: ≥ 2 of 3 clothes end up in the main box without help.

## Notes & risks

- **SAM3 from a low camera:** clothes are seen flat and far (16 cm camera height). Detection at 2–3 m may fail; then viewpoints go closer (smaller spacing) or the heading count goes up. Test first.
- **SAM3 finds any clothing:** people's clothes too. Points above 0.3 m or outside the allowed area are dropped; a person standing still in view is still a risk.
- **Rover stability:** the arm reaching down in front shifts the weight forward; check the rover doesn't tip, and that the arm clears the OAK-D and the front wheels.
- **Power:** block 9 saw the rover's motor controller reset repeatedly (likely battery sag under load). Box trips add driving; start runs with a full battery.
- **Stopping 20 cm before a box relies on localization** (RTAB-Map + odometry), not on seeing the box: the box is in the camera's blind zone by then. Check the stop distance on every run at first; a localization error of a few cm eats into the 20 cm.
- **Latency:** each look is 4 headings × a few SAM3 requests (0.7–1.8 s each): ~20–40 s per viewpoint.

## Open questions

- Main box: size and height (for the stop pose and the arm's `drop_box`), and the exact position once it stands there.
- The other three boxes: where exactly (for their keepout rectangles)?
- Headings per viewpoint: 4, or 3 with a wider overlap?

## Requests from other blocks

(to the arm team: block 5 / ROS track `cloth_task`)
- Floor picks from the rover: driver `REBOT_Z_MIN` down to the floor (~−0.27 m) when on the rover; no `table` collision plane under `base_link` (the rover body instead); `approach.workspace` / `grasp.floor_z_m` in `task.yaml` for floor height.
- Split pick and place: pick → hold in `carry` → (rover drives) → place; as the three services above.
- Poses: `floor_view` (wrist camera looking at the floor 0.25–0.40 m ahead), `carry`, `drive`, `drop_box` (release above the main box, 0.20 m ahead of the rover's front).

## Log

- 2026-09-26: block planned: viewpoint search on the saved map, OAK-D + SAM3 to find clothes, wrist camera for the final grasp, machine at a recorded place; arm floor reach checked with `rebot_b601` IK (0.25–0.40 m ahead at floor level, top-down).
- 2026-09-26: target changed from the washing machine to the **main laundry box** (one of four next to the machine, at about (1, −2)); the rover stops 20 cm before it, approaching from the west; boxes go into the keepout mask. Wrist-camera search noted as a later option.
