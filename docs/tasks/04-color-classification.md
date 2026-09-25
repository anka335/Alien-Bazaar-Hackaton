# Block 4: Color Classification

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/color_classifier/`, `tests/color_classifier/`, `config/default.yaml` → `color_classifier`

## Goal

Given a frame from the `look_bg` pose, find every item on the background and return its color class (light / dark / colored) and a re-grasp point. An empty list means the background is empty.

## Scope

- [ ] Implement `ColorClassifier.classify(frame) -> BackgroundResult` per [architecture.md](../architecture.md).
- [ ] Segment items in the background ROI (`views.background.roi`): difference from the known gray, and/or depth above the background plane. Split into blobs, drop blobs below the min area, sort by area (largest first).
- [ ] Classify each blob on robust color statistics (e.g. median in Lab/HSV): high chroma → **colored**, otherwise lightness splits **light** vs **dark**. Erode the mask so shadows and edges don't skew it. Fill `confidence` and `stats`.
- [ ] Re-grasp point: the **highest point** (smallest depth) among pixels deep inside the blob (distance transform above a threshold). Not the centroid: it can fall outside non-convex shapes.
- [ ] `touches_roi_edge` when the blob touches the ROI border.
- [ ] Thresholds in config, tuned on the demo clothes. A small tool that prints the stats of an item helps tuning.
- [ ] Overlay: masks, grasp points, color labels.
- [ ] Tests on recorded observations: each class, empty background, two items.

## Out of scope

Pixel → arm conversion (block 2). What to do with the result (block 6).

## Depends on / Unblocks

- Depends on: 0 (contract, stubs). Real frames from 1 (recorded from the look pose); start with handheld photos or recordings.
- Unblocks: integration.

## Acceptance criteria

- 100% correct class on the chosen demo clothes (recorded frames and live rig).
- An empty background is never reported as an item, and an item is never missed. Two separate items give two results.

## Notes & risks

- **The blob count is a contract.** The state machine verifies a drop by the count going down ([D-008](../decisions.md)). Noise blobs or one item split into two break the counters.
- Agree on class rules with the team before tuning: jeans, gray, prints, multicolor. Write the rules here. For the demo, prefer unambiguous items.
- Mid-gray clothes on a mid-gray background are hard to segment. Avoid them in the demo or use depth.
- A flat cloth is hard to grasp with a parallel gripper. That's why the re-grasp point is the highest point. Block 5 releases items from a height so they land crumpled.

## Open questions

- Class rules for ambiguous items.

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contract fixed: `classify` returns all blobs (`items`, largest first) instead of one item; the re-grasp point is the highest point inside the blob (D-008).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/color_classifier/config.py` (placeholder). Real backend: `sorter/color_classifier/backend.py` → `create(cfg) -> ColorClassifier`. See architecture.md → Wiring.
