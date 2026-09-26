# Block 10: Room Search

**Status:** todo (planned) · **Owner:** Slava + Claude · **Branch:** `block/10-room-search`
**Owned paths:** `ros2_ws/src/rover_nav/` (search nodes and tools, shared with block 9), `~/rover_nav_maps/places.yaml` (machine-specific, not committed)

## Goal

The rover searches the allowed part of the room for pieces of clothing, drives up to each one, lets the arm pick it up from the floor, carries it to the washing machine (always at the same place), drops it in, and goes on searching until the room is clear ([D-021](../decisions.md), proposed). Builds on block 9's map, localization, Nav2 and keepout mask. Obstacle avoidance stays simple: what Nav2 already does, plus skipping what can't be reached.

## How it works

```text
          ┌───────────────────────────── next viewpoint ◄───────────────────────────┐
          ▼                                                                          │
 GO_TO_VIEWPOINT ─► LOOK_AROUND ──no cloth──► (viewpoints left?) ──no──► DONE        │
                        │ cloth(es) seen: queue their map positions                  │
                        ▼                                                             │
                   APPROACH_CLOTH ─► PICK (arm, wrist camera) ─► GO_TO_MACHINE ─► DROP (arm)
                        │ unreachable / pick failed ×2: mark ignored ────────────────┘
```

1. **Viewpoints** are generated once from the saved map and the keepout mask (plain Python, like `make_keepout`): a grid of points about 1.2 m apart on free cells, at least ~0.4 m from obstacles, unknown space and the keepout areas, visited in nearest-first order from the rover's position.
2. **Look around:** at each viewpoint the rover stops and faces 4 directions (90° apart; the OAK-D sees ~70°). At each heading the search detector runs on a few OAK-D frames. Every cloth it finds becomes a point in the `map` frame. Points in the keepout areas are ignored; points within 0.3 m of a known cloth are the same cloth.
3. **Approach:** Nav2 drives to a parking pose facing the cloth, with the cloth ~0.32 m ahead of the arm's base (inside the arm's floor reach, see *Pick geometry*). The parking pose is computed from the cloth point and the rover's current position; if Nav2 can't reach it, two other angles around the cloth are tried, then the cloth is marked ignored.
4. **Pick:** the rover stands still and the arm stack picks: arm to a `floor_view` pose, wrist D435i + SAM3 finds the cloth again (the OAK-D can't see that close), straight-down grasp, lift to a `carry` pose. Failure → one more try, then ignored.
5. **Machine:** Nav2 drives to the machine pose (recorded once in `places.yaml`), the arm places the cloth in the drum (`place_machine` pose), back to `drive`. The cloth is struck off; the search continues from the nearest unvisited viewpoint.
6. **Done** when all viewpoints were visited and the cloth queue is empty. Optional second sweep (`sweeps: 2`) for clothes dropped or moved meanwhile.

## Obstacle avoidance (simple)

Already there from block 9, nothing new to build:

- Nav2's local costmap marks anything 5–90 cm high seen by the OAK-D depth (people, bags, chairs moved since mapping); the planner goes around it, the RPP controller slows down and stops before a collision.
- Nav2's recovery behaviors: clear costmaps, wait, back up, spin.

Added by the search node, all plain rules:

- A goal that fails (Nav2 aborts, or no progress for 30 s) is retried once; then the viewpoint is skipped or the cloth ignored, and the search moves on.
- Known cloth points are never used as viewpoints or parking spots on top of a cloth; clothes on the floor are lower than 5 cm and are not obstacles to Nav2, so the rover must not drive over one it hasn't picked yet (accepted risk for unseen ones).

## Pick geometry (checked offline with `rebot_b601` IK)

- Floor is ~0.27 m below the arm's base (`leo/base_link` is 0.198 m above the floor, arm base +0.07 m, placeholder).
- Straight-down grasps at floor level work from **0.25 to 0.40 m** ahead of the arm's base (0.45 m fails). Target **0.32 m**: ~0.16 m ahead of the front wheels.
- The OAK-D sees the floor only from ~0.4 m ahead of the rover, so the final detection is the wrist camera's; the OAK-D's job is to find the cloth from 0.5–3 m.
- Today the arm stack refuses these targets: see *Requests from other blocks*.

## Components (all in `ros2_ws/src/rover_nav`, launched by `search.launch.py` on top of `navigation.launch.py`)

| Piece | What | Test without the rover |
| --- | --- | --- |
| `rover_nav/viewpoints.py` | map + keepout → viewpoint list; nearest-first order | pytest on synthetic maps |
| `rover_nav/search_logic.py` | pure state machine: cloth queue, de-duplication, keepout check, parking pose, retry/ignore rules | pytest, no ROS |
| `room_search` node | runs the logic with `nav2_simple_commander.BasicNavigator` (`goToPose`, `spin`), the detector and the arm services; publishes `/room_search/phase` + markers | on the rover with the arm faked |
| search detector | a second `cloth_detector` instance (cloth_task, unchanged) on the OAK-D topics, outputs remapped under `/search/…` | the laptop + OAK-D alone |
| `fake_arm` node | answers the arm services after a delay (for testing search + machine trips before the arm is ready) | — |
| `record_place` tool | saves the rover's current map pose as a named place (`machine`) into `places.yaml` | — |

## Interfaces (proposed; to agree with the arm team)

| Name | Type | Who |
| --- | --- | --- |
| `/search/cloth_detector/enable` | `std_srvs/SetBool` | room_search → search detector (on only while the rover stands still at a heading) |
| `/search/cloth_target_pose` | `geometry_msgs/PoseStamped`, OAK-D optical frame | search detector → room_search (transformed to `map` at the image stamp) |
| `/arm/pick_from_floor` | `std_srvs/Trigger` (long call, ~1 min) | room_search → arm stack: floor_view, detect with the wrist camera, grasp, lift to `carry`; `success` = holding |
| `/arm/place_in_machine` | `std_srvs/Trigger` | room_search → arm stack: place into the drum, back to `drive` |
| `/arm/stow` | `std_srvs/Trigger` | room_search → arm stack: go to `drive` (before any rover motion) |
| `places.yaml` | `machine: {x, y, yaw}` in the `map` frame | recorded once per map |

The rover never moves while an arm service runs, and the arm never moves while the rover drives (`drive` / `carry` poses are compact and keep the OAK-D view free).

## Scope

- [ ] `viewpoints.py` + tests; preview image of the viewpoints on the map (like `keepout_preview.png`).
- [ ] `search_logic.py` + tests (queue, de-dup, keepout check, parking poses, retries).
- [ ] `record_place` tool; record `machine`.
- [ ] Search detector on the OAK-D (second `cloth_detector` instance, remapped); check SAM3 finds floor clothes at 0.5–3 m from the low camera.
- [ ] `room_search` node + `search.launch.py`, with `fake_arm`: full loop on the rover (search → approach → "pick" → machine → "drop" → continue).
- [ ] Swap `fake_arm` for the arm stack once it offers the three services.
- [ ] Tune: viewpoint spacing, headings, parking distance, speeds.

## Out of scope

Autonomous exploration of unknown space (the map exists), mapping other rooms, grasp planning itself (arm stack), opening the machine door.

## Depends on / Unblocks

- Depends on: block 9 (map, localization, Nav2, keepout), the SAM3 service (internet on the laptop), the arm stack's floor pick and machine place (*Requests*).
- Unblocks: the full demo.

## Acceptance criteria

- With 3 clothes on the floor of the allowed area and the arm faked: all 3 found, each approached to within 5 cm of the parking pose, 3 machine trips, `DONE`; no entry into the keepout areas; a person stepping in the way makes the rover stop or go around, never collide.
- With the arm: ≥ 2 of 3 clothes end up in the machine without help.

## Notes & risks

- **SAM3 from a low camera:** clothes are seen flat and far (16 cm camera height). Detection at 2–3 m may fail; then viewpoints go closer (smaller spacing) or the heading count goes up. Test first.
- **SAM3 finds any clothing:** people's clothes too. Points above 0.3 m or outside the allowed area are dropped; a person standing still in view is still a risk.
- **Rover stability:** the arm reaching down in front shifts the weight forward; check the rover doesn't tip, and that the arm clears the OAK-D and the front wheels.
- **Power:** block 9 saw the rover's motor controller reset repeatedly (likely battery sag under load). Machine trips add driving; start runs with a full battery.
- **Latency:** each look is 4 headings × a few SAM3 requests (0.7–1.8 s each): ~20–40 s per viewpoint.

## Open questions

- Machine: which side is the door, how high is the drum opening, can the arm reach into it from the rover?
- Is the machine inside the allowed area (above y = −3.5)? If not, the keepout needs a corridor to it.
- Headings per viewpoint: 4, or 3 with a wider overlap?

## Requests from other blocks

(to the arm team: block 5 / ROS track `cloth_task`)
- Floor picks from the rover: driver `REBOT_Z_MIN` down to the floor (~−0.27 m) when on the rover; no `table` collision plane under `base_link` (the rover body instead); `approach.workspace` / `grasp.floor_z_m` in `task.yaml` for floor height.
- Split pick and place: pick → hold in `carry` → (rover drives) → place; as the three services above.
- Poses: `floor_view` (wrist camera looking at the floor 0.25–0.40 m ahead), `carry`, `drive`, `place_machine`.

## Log

- 2026-09-26: block planned: viewpoint search on the saved map, OAK-D + SAM3 to find clothes, wrist camera for the final grasp, machine at a recorded place; arm floor reach checked with `rebot_b601` IK (0.25–0.40 m ahead at floor level, top-down).
