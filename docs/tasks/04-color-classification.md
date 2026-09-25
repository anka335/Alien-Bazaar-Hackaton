# Block 4: Color Classification

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (color classifier module, its tests and test data)

## Goal

Given a frame with one item on the uniform background: classify the color as light, dark, or colored, return a re-grasp point, or report that the background is empty.

## Scope

- [ ] Segment the item on the background ROI (difference from the empty-background reference; depth may help).
- [ ] Classify on robust color statistics of the item pixels (e.g. median in Lab/HSV): high chroma/saturation → **colored**, otherwise lightness splits **light** vs **dark**. Erode the mask so shadows and edges don't skew it.
- [ ] Re-grasp point: a spot deep inside the item (e.g. distance-transform maximum), not the centroid (it can fall outside non-convex shapes).
- [ ] Detect an empty background. This signal is used by the state machine both for "grasp from box missed" and "item removed after drop".
- [ ] Thresholds in config, tuned on the demo clothes. A small tool to print the stats of an item helps tuning.
- [ ] Debug output for the dashboard: mask, color stats.
- [ ] Tests on recorded frames: each class, empty background.

## Out of scope

Pixel→arm conversion (block 2).

## Depends on / Unblocks

- Depends on: 0 (contract, stubs). Real frames from 1; start with phone photos or recordings.
- Unblocks: integration.

## Acceptance criteria

- 100% correct on the chosen demo clothes (recorded frames and live rig).
- An empty background is never classified as an item, and an item is never missed.

## Notes & risks

- Agree on class rules with the team before tuning: jeans, gray, prints, multicolor. Write the rules here. For the demo, prefer unambiguous items.
- Mid-gray clothes on a mid-gray background will be hard to segment. Avoid them in the demo or use depth.

## Open questions

- Class rules for ambiguous items. Fallback bin for low-confidence results?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
