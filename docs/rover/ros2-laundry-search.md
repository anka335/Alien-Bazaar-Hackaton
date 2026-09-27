# ROS 2 track: laundry search in front of the boxes

**Status:** to do · **Owner:** open (for a collaborator's agent) · **Branch:** `feat/rover-laundry-search` (to create)
**Owned paths:** `ros2_ws/src/rover_nav/` (new files; the patrol and the box detector only through the changes listed below)

A task written down for another collaborator's Claude to pick up. It joins two pieces that already exist but aren't connected: the hard-coded patrol ([ros2-navigation.md → Hard-coded patrol](ros2-navigation.md), D-046) and the box detector (ArUco markers, branch `feat/rover-boxes`). Read [ros2-navigation.md](ros2-navigation.md) and `ros2_ws/src/rover_nav/README.md` first.

## Goal

The rover searches for laundry in the area in front of the three laundry boxes, roughly in the middle of the room. When it sees a piece of laundry, it goes to it, waits while the arm picks it up, and returns to the boxes.

## Behavior

1. **Get in front of the boxes.** The search always starts from a fixed pose in front of the boxes (the *box pose*): facing the middle (colored) box, centred on it, at a set distance.
   - If the rover isn't there, it drives there.
   - If it's too close, it backs up a little until it's at the set distance.
   - The box detector gives the boxes in the rover's frame (`/boxes/<label>` in `leo/base_link`, `/boxes/target_distance`), so align from the markers, not from the map. When the markers aren't in view, see [the problem](#the-problem-finding-the-boxes-again-not-solved-yet) below.
2. **Search.** From the box pose, run the patrol (a 1 m square or circle with a look-around at every stop). The pattern must lie on the room side, away from the boxes. The current patrol turns left from the start heading, so starting while facing the boxes would drive into them: turn around first (or mirror the pattern). The whole search stays in front of the boxes, roughly in the middle of the room.
3. **Laundry seen → go to it.** As soon as laundry is detected (during a look-around or while driving), stop the patrol and drive to the laundry. Stop about 20 cm before it, as in the earlier simple mission.
4. **Pick up.** Stand still and wait until the arm reports that it's done (picked, or failed).
5. **Return to the boxes.** Drive back to the remembered box pose by odometry, then line up from the markers as in step 1 (see [the problem](#the-problem-finding-the-boxes-again-not-solved-yet) below).
6. **Then** start the search again (step 2), until stopped. What the arm does at the boxes (unloading) isn't part of this task.

A stop (like `patrol_ctl stop`) must work in every state and stop the wheels at once.

## The problem: finding the boxes again (not solved yet)

Nothing remembers where the boxes are:

- The box detector knows the boxes only **while the markers are in view**: within about 1 m and inside the OAK-D's ~70° field of view. It gives their pose relative to the rover, live, and forgets it as soon as the markers leave the image.
- The patrol knows only its own start pose (odometry).

So "get in front of the boxes" works only if the markers are already in view (it is fine alignment, not finding). "Return to the boxes" after a pick has no answer: by then the rover has usually turned away, may be more than 1 m from the boxes, and sees nothing.

**How to solve it (simple, no SLAM):**

1. **Remember the boxes in the odometry frame.** Every time the markers are seen, turn the box pose from `leo/base_link` into `leo/odom` (the rover's odometry pose at the image's time) and store it. Store the *box pose* (the standing pose in front of the middle box) with it. Odometry drifts only a few cm over a 1 m search, so the stored pose stays good enough to drive back to.
2. **Return:** drive by odometry to the remembered box pose (turn towards it, drive, turn to face the boxes). The markers are then in view again. Line up from them precisely: forward or back to the set distance, turn to centre the middle box. Then update the remembered pose.
3. **At the start, with no markers in view:** turn in place in steps, as in a look-around, pausing at each, until the markers appear. If they never appear after a full turn, stop and report "boxes not found" rather than guess.
4. **Optional hint:** the fixed start pose (0, 2.5) facing south plus the known box position (about x = 1.5, y = 1, map frame) tell roughly which way to turn first. Only as a hint; the markers decide.
5. **Keep trips short.** The search pattern is small (1 m), and every return re-aligns from the markers, so the odometry drift never adds up. If the laundry is far away (a trip of several metres), expect a few cm to a dm of error on the way back, still enough to bring the markers into view.

Test it with a fake detector that publishes the boxes only when the (fake) rover faces them within 1 m, so the "not in view" cases are covered.

## What exists

- **Patrol** (`patrol_logic.py`, `patrol_node.py`, `patrol_ctl.py`, `config/patrol.yaml`, `patrol.launch.py`):
  - odometry only (`/leo/merged_odom`), commands on `/leo/cmd_vel`;
  - steps `Turn` / `Drive` / `Dwell`, planned from the start pose;
  - `/patrol/scanning` is true during each look;
  - `fake_rover` for runs without the rover;
  - no obstacle avoidance.
- **Box detector** (`boxes.py`, `box_detector.py`, `config/boxes.yaml`, `boxes.launch.py`, branch `feat/rover-boxes`):
  - OAK-D image and depth;
  - left → right: dark, colored, light;
  - 4 cm markers, seen to about 1 m.
  - The marker ids aren't in `config/boxes.yaml` yet. Without them, boxes are labelled by their order, which needs all three in view.
- **Laundry detection:** not in `rover_nav`. The segmentation model exists on the arm's side (the wrist camera; masks from the SAM3 service, D-013). Find out from the arm team what it publishes. Adding it to the OAK-D is optional.
- **Picking:** the arm team's code. No ROS interface to it exists yet.
- The rover's firmware stops the wheels 0.5 s after the last command.
- SLAM/Nav2 localization was unreliable in this room (small start differences made it abort), so this task works without the map, on odometry plus the markers.

## Suggested approach

- Write one mission node (a small state machine: `to_boxes` → `search` → `to_laundry` → `pick` → `to_boxes` …). Put the logic in plain Python without ROS, like `patrol_logic.py`, so it can be unit-tested. Reuse `patrol_logic`'s steps and controllers rather than copying them.
- Let the patrol be paused and resumed, or planned from any pose, so the mission can interrupt it.
- Agree on the interfaces with the arm team and write them into `docs/architecture.md`. For example:
  - laundry: `geometry_msgs/PoseStamped` in `leo/base_link`;
  - picking: a request/answer (e.g. `std_srvs/Trigger` or an action).
  - Until the real interfaces exist, stand in for both with fakes, so everything runs with `fake_rover`.
- Tests: the state machine with a simulated rover (see `test/test_patrol_logic.py`), fake boxes and fake laundry. Then one end-to-end run with `fake_rover`.

## Acceptance criteria

- Started anywhere within about 1 m of the boxes with the markers in view, the rover ends at the box pose. Started too close, it backs up.
- The search stays in front of the boxes and never drives into them.
- A detection interrupts the search. The rover stops about 20 cm before the laundry, waits for the pick, and returns to the box pose.
- Stop works in every state.
- Unit tests for the logic, one run with `fake_rover`, and the docs updated (README, [ros2-navigation.md](ros2-navigation.md), a decision in `docs/decisions.md`).

## Open questions (ask the user)

- The box pose: how far in front of the boxes (distance to the middle box's marker)?
- Which pattern (square or circle) and on which side, and how many laps before it goes back to the boxes if it finds nothing?
- What the laundry detector publishes and on which camera; what the arm needs from the rover to pick (how close, facing which way).
- The box marker ids (for `config/boxes.yaml`).
- The last known layout (map frame; the start pose is (0, 2.5) facing south): the three boxes at about x = 1.5, y = 1; places to avoid at x < −1 and at x > 0.5 with y > 1.8. Check this before relying on it: the room has changed before.

## Log

- 2026-09-27: task written down (not started).
- 2026-09-27: the problem of finding the boxes again (markers seen only within ~1 m, nothing remembers them) and the proposed fix (remember them in `leo/odom`, re-align from the markers) written down.
