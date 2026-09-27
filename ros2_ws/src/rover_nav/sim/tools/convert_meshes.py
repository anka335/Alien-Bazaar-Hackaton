"""Rebuild leo_sim/assets/ from the official Leo Rover model (leo_description, MIT).

MuJoCo cannot read the COLLADA (.dae) visual meshes of leo_description, and they are 33 MB.
This splits every part by material color, decimates each color group and writes binary STL
plus assets/meshes.json ({part: [{"file", "rgba"}]}), which leo_sim.model turns into colored
visual geoms. The collision outlines (small STL) are copied unchanged.

    uv run --group meshes python tools/convert_meshes.py                 # clones tag 3.2.0
    uv run --group meshes python tools/convert_meshes.py --src ~/leo_common-ros2/leo_description
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import trimesh

REPO = "https://github.com/LeoRover/leo_common-ros2.git"
TAG = "3.2.0"  # jazzy branch, 2025-12-29
ASSETS = Path(__file__).resolve().parents[1] / "leo_sim" / "assets"

# part -> (face budget for the whole part, max color groups)
PARTS = {
    "Chassis": (14000, 7),
    "Rocker": (5000, 4),
    "WheelA": (3000, 3),
    "WheelB": (3000, 3),
    "Antenna": (600, 2),
}
OUTLINES = ["Chassis_outline.stl", "Rocker_outline.stl"]


def color_of(mesh: trimesh.Trimesh) -> tuple[float, ...]:
    material = getattr(mesh.visual, "material", None)
    color = getattr(material, "main_color", None)
    if color is None:
        return (0.6, 0.6, 0.6, 1.0)
    return tuple(round(float(c) / 255, 3) for c in color)


def split_by_color(path: Path, max_groups: int) -> dict[tuple, trimesh.Trimesh]:
    scene = trimesh.load(path, force="scene")
    groups: dict[tuple, list[trimesh.Trimesh]] = {}
    for mesh in scene.dump(concatenate=False):  # node transforms applied
        plain = trimesh.Trimesh(mesh.vertices, mesh.faces, process=False)
        groups.setdefault(color_of(mesh), []).append(plain)
    merged = {c: trimesh.util.concatenate(ms) for c, ms in groups.items()}
    order = sorted(merged, key=lambda c: -len(merged[c].faces))
    keep, rest = order[:max_groups], order[max_groups:]
    for c in rest:  # small groups join the nearest kept color
        near = min(keep, key=lambda k: np.linalg.norm(np.subtract(k[:3], c[:3])))
        merged[near] = trimesh.util.concatenate([merged[near], merged.pop(c)])
    return merged


def decimate(mesh: trimesh.Trimesh, faces: int) -> trimesh.Trimesh:
    mesh.merge_vertices()
    if len(mesh.faces) <= faces:
        return mesh
    return mesh.simplify_quadric_decimation(face_count=faces)


def convert(src: Path) -> None:
    models = src / "models"
    if ASSETS.exists():
        shutil.rmtree(ASSETS)
    ASSETS.mkdir(parents=True)
    index: dict[str, list[dict]] = {}
    for part, (budget, max_groups) in PARTS.items():
        groups = split_by_color(models / f"{part}.dae", max_groups)
        total = sum(len(m.faces) for m in groups.values())
        index[part] = []
        for i, (rgba, mesh) in enumerate(sorted(groups.items(), key=lambda kv: -len(kv[1].faces))):
            share = max(80, int(budget * len(mesh.faces) / total))
            small = decimate(mesh, share)
            name = f"{part}_{i}.stl"
            small.export(ASSETS / name)
            index[part].append({"file": name, "rgba": list(rgba)})
            print(f"{name}: {len(mesh.faces)} -> {len(small.faces)} faces, rgba {rgba}")
    for name in OUTLINES:
        shutil.copy(models / name, ASSETS / name)
    (ASSETS / "meshes.json").write_text(json.dumps(index, indent=1) + "\n")
    license_file = src.parent / "LICENSE"
    notice = f"Meshes converted from leo_description {TAG} ({REPO}) by tools/convert_meshes.py.\n\n"
    (ASSETS / "LICENSE").write_text(notice + license_file.read_text())
    size = sum(p.stat().st_size for p in ASSETS.iterdir()) / 1e6
    print(f"wrote {ASSETS} ({size:.1f} MB)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--src", type=Path, help="a leo_description directory (default: clone the tag)"
    )
    args = parser.parse_args()
    if args.src:
        convert(args.src.expanduser().resolve())
        return
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(["git", "clone", "-q", "--depth", "1", "-b", TAG, REPO, tmp], check=True)
        convert(Path(tmp) / "leo_description")


if __name__ == "__main__":
    main()
