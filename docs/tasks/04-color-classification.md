# Block 4: Color Classification

**Status:** in progress · **Owner:** Maciej · **Branch:** `block/04-color-classification`
**Owned paths:** `src/sorter/color_classifier/`, `tests/color_classifier/`, `config/default.yaml` → `color_classifier`

## Goal

Given a frame from the `look_bg` pose, find every item on the background and return its color class (light / dark / colored) and a re-grasp point. An empty list means the background is empty.

## Scope

- [x] Implement `ColorClassifier.classify(frame) -> BackgroundResult` per [architecture.md](../architecture.md): `Sam3ColorClassifier` in `classifier.py`, `backend.create(cfg)`.
- [x] Segment items in the background ROI (`views.background.roi`, whole frame while it is empty) with the remote **SAM3 service** ([D-013](../decisions.md), `segmenter.py`): one instance per item, one request per prompt in `color_classifier.sam.prompts` (default `clothing`, `sock`). Pixels without depth (gripper fingers, glare) are dropped; an instance mostly covered by a better-scored one is dropped; blobs below `min_area_px` or above `max_area_frac` of the ROI are dropped; sorted by area (largest first).
- [x] Classify each blob on the median Lab of the eroded mask: L* below `lightness_dark` → **dark**; chroma ≥ `chroma_colored` → **colored**; otherwise L* ≥ `lightness_light` → **light**, else **dark**. `confidence` = 0.5 at the deciding threshold, 1 at `confidence_margin` from it. `stats`: `L`, `a`, `b`, `chroma`, `px`, `score`.
- [x] Re-grasp point: the **highest point** at least `grasp_inset_px` inside the blob (distance transform); points within `grasp_depth_tol_mm` of the highest are ties, and the one most inside the top region and the blob wins (a flat cloth is grasped in its middle). Depth = median of the non-zero depths in a small window.
- [x] `touches_roi_edge` when the blob touches the ROI border or the image border.
- [ ] Thresholds in config, tuned on the demo clothes. Tool: `uv run python -m sorter.color_classifier.stats <obs.npz ...> [--sim N] [--prompts a,b] [--threshold T] [--save DIR]` prints the stats of every item and saves overlays.
- [x] Overlay: union mask, outline + label per item, grasp marker with class and confidence, item count.
- [x] Tests (`tests/color_classifier/`): RLE decoding, the HTTP client and its errors (mocked), each class on sim frames with a depth-based SAM3 stand-in, empty background, three items, duplicates / fingers / ROI edge, the re-grasp point, and the full sim loop with the real classifier. A live-service test runs when `SAM3_API_KEY` is set.
- [ ] Tests on recorded real observations (needs block 1 recordings).

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
- **SAM3 is remote.** Unreachable service, bad key, or timeout (`sam.timeout_s`) → `SegmentationError` → the loop goes to `ERROR`, paused; Reset retries. No fallback (D-013). Check the ngrok URL and the key before the demo.
- **A SAM3 miss is an unsorted item:** the loop sees an empty background and moves on to the box. On the rendered sim cloth, prompt `clothing` finds nothing and `blob` scores only 0.2–0.7, so sim runs with the real classifier leave items behind. Validate prompt and threshold on the real demo clothes.
- **Prompts are object categories.** Tested on public photos of real cloth (flat fake depth): `clothing` finds a T-shirt but no socks; `sock` finds only socks; `cloth`, `textile`, `towel` find nothing. Add a prompt for every garment type in the demo set (e.g. `pants`), and check each costs one more request.
- **Real-photo colors (untuned):** white T-shirt → light 1.00, dark gray sock → dark 1.00. In a pile of knitted socks under warm light, mid-gray socks come out dark at 0.54–0.72, a lilac sock is dark at 0.55, and burgundy is dark by the `lightness_dark` rule. Tune `lightness_light` and `chroma_colored` on the demo clothes under the rig lamp, not on internet photos.
- SAM3 also finds the gripper fingers (prompt `object`/`blob`); they are removed because they have no depth. If the real camera reports depth on the fingers, the ROI must exclude them.
- A flat cloth is hard to grasp with a parallel gripper. That's why the re-grasp point is the highest point. Block 5 releases items from a height so they land crumpled.

## Open questions

- Class rules for ambiguous items. Current rules: very dark saturated items (navy, dark brown, L* < 22) are **dark**; pastel items (chroma < 20) are **light** or **dark** by L*. Jeans, prints, and multicolor items are still to agree with the team.
- Best SAM3 prompts and threshold for the real demo clothes on the mat.
- Burgundy / deep purple: dark or colored? The current rule says dark (L* < 22).

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contract fixed: `classify` returns all blobs (`items`, largest first) instead of one item; the re-grasp point is the highest point inside the blob (D-008).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/color_classifier/config.py` (placeholder). Real backend: `sorter/color_classifier/backend.py` → `create(cfg) -> ColorClassifier`. See architecture.md → Wiring.
- 2026-09-25 (block 6): run logs are written: `data/runs/<run_id>/<cycle:04d>_<phase>.npz` + `.json` per sense phase (vision result, decision). Usable as test data; format in architecture.md → Recording format.
- 2026-09-25 (block 4): real backend on the SAM3 service (D-013). New config section `color_classifier` (typed, `extra="forbid"`); the API key goes in `config/local.yaml` or `SAM3_API_KEY`. `classify` can raise `SegmentationError` (a `SorterError`).
- 2026-09-26 (block 5): the simulator changed (D-014, D-015): `sim.motion_s` is now `sim.time_scale` (updated in `stats.py`), the mat is 240 x 180 mm and seen from ~260 mm, `item_radius_mm` is 30, and `free_point_on_background()` spreads items as far apart as it can. A second item of a double grasp lands on a free spot of the mat.
- 2026-09-26 (Softjey + Claude): in the physics simulator (D-016) the real classifier runs on rendered frames, with the render's segmentation in place of the SAM3 client (`sim.use_sam3: true` calls the service). The stats tool's `--sim` uses the kinematic engine.
