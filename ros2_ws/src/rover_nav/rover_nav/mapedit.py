"""Draw obstacles into a saved map: things that weren't there when the room was mapped.

    ros2 run rover_nav add_to_map ~/rover_nav_maps/room.yaml --rect X1 Y1 X2 Y2 [--rect ...] \\
        [-o ~/rover_nav_maps/room_nav]

Each `--rect` (two opposite corners, `map` frame, metres) becomes occupied in a copy of the map;
the original stays untouched. Default output: `<map>_nav.pgm` + `.yaml` next to the map, which
navigation.launch.py serves to Nav2 as the static map when it exists.

No ROS imports: map_server's file format, like make_keepout.
"""

import argparse
import os
import sys

import numpy as np
import yaml

from rover_nav.keepout import forbidden_rect, load_map, read_pgm, write_pgm

OCCUPIED = 0  # black; with negate 0 in trinary mode = occupied


def add_to_map(map_yaml: str, rects, out_prefix: str) -> int:
    """Copy the map with every rectangle occupied. Returns the number of cells changed."""
    if not rects:
        raise ValueError("nothing to add: give at least one --rect")
    meta = load_map(map_yaml)
    origin = meta["origin"]
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise ValueError(f"{map_yaml}: map origin yaw {origin[2]} is not supported (expected 0)")
    img = read_pgm(meta["image"]).copy()
    if img.dtype != np.uint8:
        raise ValueError(f"{meta['image']}: only 8-bit maps are supported")
    if int(meta.get("negate", 0)) != 0:
        raise ValueError(f"{map_yaml}: negate 1 maps are not supported")
    mask = np.zeros(img.shape, dtype=bool)
    for corner1, corner2 in rects:
        mask |= forbidden_rect(
            img.shape, float(meta["resolution"]), (origin[0], origin[1]), corner1, corner2
        )
    if not mask.any():
        raise ValueError("the rectangles don't cover any map cell (outside the map?)")
    changed = int(np.count_nonzero(img[mask] != OCCUPIED))
    img[mask] = OCCUPIED
    write_pgm(out_prefix + ".pgm", img)
    out_meta = {k: v for k, v in meta.items() if k != "image"}
    out_meta = {"image": os.path.basename(out_prefix) + ".pgm", **out_meta}
    with open(out_prefix + ".yaml", "w") as f:
        f.write(f"# {os.path.basename(map_yaml)} + obstacles {list(rects)}\n")
        yaml.safe_dump(out_meta, f, sort_keys=False)
    return changed


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map_yaml", help="saved map (map_saver_cli output), e.g. room.yaml")
    parser.add_argument(
        "--rect",
        nargs=4,
        type=float,
        action="append",
        default=[],
        metavar=("X1", "Y1", "X2", "Y2"),
        help="obstacle: two opposite corners, map frame, m; repeatable",
    )
    parser.add_argument("-o", "--out", help="output prefix (default: <map>_nav next to the map)")
    args, _ = parser.parse_known_args(argv)  # ignore --ros-args
    out = args.out or os.path.splitext(os.path.abspath(args.map_yaml))[0] + "_nav"
    if os.path.abspath(out + ".yaml") == os.path.abspath(args.map_yaml):
        print("add_to_map: refusing to overwrite the input map", file=sys.stderr)
        return 1
    rects = [(tuple(r[:2]), tuple(r[2:])) for r in args.rect]
    try:
        changed = add_to_map(args.map_yaml, rects, out)
    except (OSError, ValueError, KeyError) as e:
        print(f"add_to_map: {e}", file=sys.stderr)
        return 1
    print(f"wrote {out}.pgm and {out}.yaml: {changed} cells newly occupied")
    return 0


if __name__ == "__main__":
    sys.exit(main())
