# Block 1: Setup & Camera

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (camera module, camera tools)

## Goal

A fixed, repeatable physical rig and a camera module that returns fresh, aligned color + depth frames.

## Scope

### Physical

- [ ] Position the arm, mixed-clothes box, uniform background area (mid-gray, so both white and black clothes contrast), and 3 bins. All within the arm's reach.
- [ ] Fix everything to the table so nothing shifts during operation. Mark positions with tape.
- [ ] Dedicated lamp for stable lighting.
- [ ] Mount the depth camera overhead so the frame covers the box and the background area. Bins may stay out of frame.
- [ ] Take photos of the rig for the docs, so it can be rebuilt after transport.

### Software

- [ ] Camera module implementing the frame contract from block 0: color + depth, depth aligned to color.
- [ ] Always return the **latest** frame (drain the SDK buffer), never a stale one captured before the arm moved.
- [ ] Lock auto exposure / white balance if the SDK allows.
- [ ] Tool to record frames/sessions to disk, so blocks 3–4 can work offline.
- [ ] Tool to capture reference images of the empty box and empty background (if blocks 3–4 need them).

## Depends on / Unblocks

- Depends on: block 0 (frame contract). The physical part can start right away.
- Unblocks: 2, real frames for 3 and 4.

## Acceptance criteria

- The rig can be disassembled and reassembled to the same positions.
- The camera module passes its contract test; frames are fresh (verified by moving an object and reading immediately).
- A recorded dataset of the real scene is available to blocks 3–4: empty box, box with clothes, background with single items of each class.

## Notes & risks

- Place the camera so the arm is out of view (or can move out of view) when frames are taken.
- Check the depth camera's minimum range. Mounting too low gives holes in the depth map.

## Open questions

- Camera model/SDK? Where does the dataset live (it shouldn't go into git if large)?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
