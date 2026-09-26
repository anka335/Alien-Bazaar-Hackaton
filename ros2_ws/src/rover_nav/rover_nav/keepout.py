"""Nav2 keepout mask from a saved map: forbid one side of a line and/or rectangles (D-019).

    ros2 run rover_nav make_keepout ~/rover_nav_maps/room.yaml --line X1 Y1 X2 Y2 --keep X Y \
        [--forbid-rect X1 Y1 X2 Y2 ...]

All coordinates are in the `map` frame (metres). The line goes through two points; `--keep` is
any point on the side the rover may use. Each `--forbid-rect` (repeatable) forbids an extra
axis-aligned rectangle given by two opposite corners, e.g. furniture the rover must not drive
through. Writes `keepout.pgm` + `keepout.yaml` next to the map (or `-o PREFIX`): black =
forbidden, white = allowed, same resolution and origin as the map, for Nav2's KeepoutFilter.

No ROS imports: the map files are the map_server format (`map_saver_cli` output).
"""

import argparse
import os
import sys

import numpy as np
import yaml

FORBIDDEN = 0  # black: occupied in trinary mode (negate 0) = keepout
ALLOWED = 254  # white: free


def read_pgm(path: str) -> np.ndarray:
    """Binary PGM (P5) as a 2D array, row 0 = top of the image."""
    with open(path, "rb") as f:
        data = f.read()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:  # magic, width, height, maxval; comments start with '#'
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: not a binary PGM (P5)")
    width, height, maxval = (int(t) for t in tokens[1:])
    pos += 1  # exactly one whitespace byte before the pixels
    dtype = np.uint8 if maxval < 256 else np.dtype(">u2")
    return np.frombuffer(data, dtype=dtype, count=width * height, offset=pos).reshape(height, width)


def write_pgm(path: str, img: np.ndarray) -> None:
    img = np.asarray(img, dtype=np.uint8)
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def load_map(yaml_path: str) -> dict:
    """The map's YAML, with `image` resolved to an absolute path."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    meta["image"] = image
    return meta


def forbidden_side(
    shape: tuple[int, int],
    resolution: float,
    origin: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    keep: tuple[float, float],
) -> np.ndarray:
    """Bool mask (True = forbidden) of the cells whose centre is on the other side of the line
    p1-p2 than `keep`. Cells on the line stay allowed. `origin` = map frame position of the
    bottom-left corner of the image (map_server convention, yaw 0)."""
    (x1, y1), (x2, y2) = p1, p2
    if (x1, y1) == (x2, y2):
        raise ValueError("the line needs two different points")

    def side(x, y):
        return (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)

    keep_side = side(*keep)
    if keep_side == 0:
        raise ValueError("the --keep point lies on the line: pick a point clearly on one side")
    height, width = shape
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (height - np.arange(height) - 0.5) * resolution  # row 0 = top = largest y
    s = side(xs[np.newaxis, :], ys[:, np.newaxis])
    return s * np.sign(keep_side) < 0


def forbidden_rect(
    shape: tuple[int, int],
    resolution: float,
    origin: tuple[float, float],
    corner1: tuple[float, float],
    corner2: tuple[float, float],
) -> np.ndarray:
    """Bool mask (True = forbidden) of the cells whose centre is inside the axis-aligned
    rectangle with opposite corners `corner1` and `corner2` (edges included)."""
    (xa, ya), (xb, yb) = corner1, corner2
    if xa == xb or ya == yb:
        raise ValueError(f"rectangle {corner1} - {corner2} has no area")
    height, width = shape
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (height - np.arange(height) - 0.5) * resolution  # row 0 = top = largest y
    in_x = (xs >= min(xa, xb)) & (xs <= max(xa, xb))
    in_y = (ys >= min(ya, yb)) & (ys <= max(ya, yb))
    return in_y[:, np.newaxis] & in_x[np.newaxis, :]


def make_keepout(map_yaml: str, out_prefix: str, line=None, rects=()) -> float:
    """Write `out_prefix`.pgm + .yaml. `line` = (p1, p2, keep) or None; `rects` = pairs of
    opposite corners. Returns the forbidden share of the map."""
    if line is None and not rects:
        raise ValueError("nothing to forbid: give --line/--keep and/or --forbid-rect")
    meta = load_map(map_yaml)
    origin = meta["origin"]
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise ValueError(f"{map_yaml}: map origin yaw {origin[2]} is not supported (expected 0)")
    shape = read_pgm(meta["image"]).shape
    res, org = float(meta["resolution"]), (origin[0], origin[1])
    mask = np.zeros(shape, dtype=bool)
    if line is not None:
        mask |= forbidden_side(shape, res, org, *line)
    for corner1, corner2 in rects:
        mask |= forbidden_rect(shape, res, org, corner1, corner2)
    img = np.where(mask, FORBIDDEN, ALLOWED).astype(np.uint8)
    write_pgm(out_prefix + ".pgm", img)
    out_meta = {
        "image": os.path.basename(out_prefix) + ".pgm",
        "mode": "trinary",
        "resolution": float(meta["resolution"]),
        "origin": [float(v) for v in origin[:2]] + [0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(out_prefix + ".yaml", "w") as f:
        f.write(f"# Keepout mask for {os.path.basename(map_yaml)}: line (p1, p2, keep) {line}")
        f.write(f", rectangles {list(rects)}\n")
        yaml.safe_dump(out_meta, f, sort_keys=False)
    return float(mask.mean())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map_yaml", help="saved map (map_saver_cli output), e.g. room.yaml")
    parser.add_argument(
        "--line",
        nargs=4,
        type=float,
        metavar=("X1", "Y1", "X2", "Y2"),
        help="two points of the border, map frame, m",
    )
    parser.add_argument(
        "--keep",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="a point on the side the rover may use (with --line)",
    )
    parser.add_argument(
        "--forbid-rect",
        nargs=4,
        type=float,
        action="append",
        default=[],
        metavar=("X1", "Y1", "X2", "Y2"),
        help="also forbid this rectangle (two opposite corners, m); repeatable",
    )
    parser.add_argument("-o", "--out", help="output prefix (default: keepout next to the map)")
    # ros2 run passes --ros-args ... when launched from a launch file; ignore it
    args, _ = parser.parse_known_args(argv)
    if (args.line is None) != (args.keep is None):
        parser.error("--line and --keep go together")
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.map_yaml)), "keepout")
    line = None
    if args.line is not None:
        line = (tuple(args.line[:2]), tuple(args.line[2:]), tuple(args.keep))
    rects = [(tuple(r[:2]), tuple(r[2:])) for r in args.forbid_rect]
    try:
        share = make_keepout(args.map_yaml, out, line=line, rects=rects)
    except (OSError, ValueError, KeyError) as e:
        print(f"make_keepout: {e}", file=sys.stderr)
        return 1
    print(f"wrote {out}.pgm and {out}.yaml: {share:.0%} of the map is forbidden")
    return 0


if __name__ == "__main__":
    sys.exit(main())
