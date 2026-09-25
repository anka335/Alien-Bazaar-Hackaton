# Block 5: Arm Control

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (arm driver(s), arm controller, pose tools)

## Goal

Safe, blocking, high-level arm operations callable from the state machine.

## Scope

- [ ] Control the arm from code (SDK / ROS / serial). Low-level driver behind the block 0 interface.
- [ ] Record fixed poses into config: home, above box, above background, bin light / dark / colored. Provide a tool to teach/record poses.
- [ ] High-level operations: `pick(point, depth)`, `place_on_background()`, `drop_to_bin(color)`, `home()`. Exact signatures per the block 0 contract.
- [ ] Every motion blocks until finished. The state machine takes a frame right after.
- [ ] **Safety:** clamp all targets to workspace limits (table, box walls); travel at a safe height between poses; emergency stop callable from the dashboard and from the keyboard.
- [ ] A "park" pose that moves the arm out of the camera's view before frames are taken.
- [ ] Tune grasp depth, approach, and grip force on real clothes. Record the working values in config and here.
- [ ] Mock driver for other blocks (part of block 0 stubs, maintained here).

## Depends on / Unblocks

- Depends on: 0 (interface). Hardware.
- Unblocks: 2 (reference-point motion), integration.

## Acceptance criteria

- All named poses reachable repeatedly. Pick/place/drop cycle works on real clothes with hard-coded points.
- A target outside the workspace is clamped or rejected, never executed. E-stop halts motion.

## Notes & risks

- Start with the **manual grasp test** (see block 3). It is the earliest signal of whether the approach works.
- Check reach: box, background, and all 3 bins inside the reachable area with the gripper pointing down.

## Open questions

- Arm model, DOF, SDK, gripper type? Does the SDK expose Cartesian moves and the current position?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
