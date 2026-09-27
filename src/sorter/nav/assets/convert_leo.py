"""One-off: Leo Rover CAD (leo_description, MIT, Fictionlab) → decimated STL per color + manifest.

MuJoCo can't load the .dae files and they weigh 33 MB. This merges each part's submeshes by color,
decimates them to a face budget and writes `leo/<part>_<n>.stl` plus `leo/manifest.json`
({part: [[file, [r, g, b, a]], ...]}). Run once, needs extra packages:

    git clone --depth 1 -b ros2 https://github.com/LeoRover/leo_common-ros2.git /tmp/leo
    uv run --no-project --with trimesh --with pycollada --with fast-simplification --with scipy \
        python src/sorter/nav/assets/convert_leo.py /tmp/leo/leo_description/models
"""

import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import trimesh

PARTS = {"Chassis": 24000, "Rocker": 6000, "WheelA": 5000, "WheelB": 5000, "Antenna": 600}
OUT = Path(__file__).parent / "leo"


def main(models: Path) -> None:
    OUT.mkdir(exist_ok=True)
    manifest = {}
    for part, budget in PARTS.items():
        scene = trimesh.load(models / f"{part}.dae", force="scene")
        groups: dict[tuple, list] = defaultdict(list)
        for node in scene.graph.nodes_geometry:
            tf, name = scene.graph[node]
            g = scene.geometry[name].copy()
            g.apply_transform(tf)
            rgba = tuple(int(c) for c in g.visual.material.main_color)
            groups[tuple(round(c / 8) * 8 for c in rgba[:3])].append(g)
        total = sum(len(m.faces) for ms in groups.values() for m in ms)
        entries = []
        for i, (rgb, meshes) in enumerate(sorted(groups.items(), key=lambda kv: kv[0])):
            m = trimesh.util.concatenate(meshes)
            m.merge_vertices()
            target = max(40, int(budget * len(m.faces) / total))
            if len(m.faces) > target:
                m = m.simplify_quadric_decimation(face_count=target)
            if len(m.faces) < 4:
                continue
            f = f"{part.lower()}_{i}.stl"
            m.export(OUT / f)
            entries.append([f, [round(c / 255, 3) for c in rgb] + [1.0]])
        manifest[part.lower()] = entries
        print(part, total, "→", sum(len(trimesh.load(OUT / e[0]).faces) for e in entries))
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print("bounds check", np.round(trimesh.load(OUT / manifest["chassis"][0][0]).bounds, 3))


if __name__ == "__main__":
    main(Path(sys.argv[1]))
