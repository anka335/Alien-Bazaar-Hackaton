"""Download the CAD meshes and the JS libraries used by the 3D viewer.

The meshes are Seeed's reBot Arm CAD as published (together with the URDF) by
Motorbridge Studio: https://motorbridge.github.io/motorbridge-studio/ .  They
are fetched on demand into ``rebot_b601/viewer_assets/`` and are *not* part of
this package (third-party files, ~37 MB); the directory is git-ignored.

Only the meshes referenced by ``<visual>`` elements are downloaded; the
collision-only meshes (about 30 MB) are skipped.

    python -m rebot_b601 fetch-assets
"""

from __future__ import annotations

import json
import re
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

ASSETS_DIR = Path(__file__).resolve().parent / "viewer_assets"

STUDIO = "https://motorbridge.github.io/motorbridge-studio/resources/arm03/reBot_Lite_RS_with_gripper"
URDF_URL = f"{STUDIO}/urdf/reBot_Lite_RS_with_gripper.urdf"
MESH_URL = f"{STUDIO}/meshes/{{name}}"

THREE_VERSION = "0.170.0"     # last release with a single-file build/three.module.min.js
THREE_FILES = {
    "three.module.min.js": f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/build/three.module.min.js",
    "STLLoader.js": f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/loaders/STLLoader.js",
    "OrbitControls.js": f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/controls/OrbitControls.js",
}


def _color_for(mesh: str) -> str:
    """The URDF gives every part the same grey; colour by part type like the real arm."""
    m = mesh.lower()
    if "green" in m:
        return "#2e9e5b"
    if "black" in m:
        return "#2a2d33"
    if m.startswith("motor"):
        return "#4a4f58"
    if m.startswith("cnc"):
        return "#c9ced6"
    return "#b7bcc6"           # base_link, link1, link6 ... (printed / anodised body parts)


def _get(url: str, timeout: float = 120.0) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "rebot_b601-fetch-assets"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = r.read()
    if not data:
        raise RuntimeError(f"empty response from {url}")
    return data


def _write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".part")
    tmp.write_bytes(data)
    tmp.replace(path)


def assets_present(dest: Path = ASSETS_DIR) -> bool:
    return (dest / "manifest.json").is_file()


def fetch_assets(dest: Path = ASSETS_DIR, force: bool = False, log=print) -> Path:
    """Download meshes + three.js into ``dest`` and write ``manifest.json``.  Returns the manifest path."""
    dest = Path(dest)
    for name, url in THREE_FILES.items():
        target = dest / "three" / name
        if force or not target.is_file():
            log(f"  {name}")
            _write(target, _get(url))

    urdf = _get(URDF_URL)
    root = ET.fromstring(urdf)
    links: dict[str, list[dict]] = {}
    for link in root.findall("link"):
        parts = []
        for vis in link.findall("visual"):
            mesh_el = vis.find("geometry/mesh")
            if mesh_el is None:
                continue
            fname = mesh_el.get("filename", "").split("/")[-1]
            if not re.fullmatch(r"[A-Za-z0-9_.-]+\.STL", fname, flags=re.I):
                raise RuntimeError(f"unexpected mesh name in URDF: {fname!r}")
            origin = vis.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split())]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split())]
            parts.append({"mesh": f"meshes/{fname}", "xyz": xyz, "rpy": rpy, "color": _color_for(fname)})
        if parts:
            links[link.get("name")] = parts

    wanted = sorted({p["mesh"] for parts in links.values() for p in parts})
    log(f"{len(wanted)} visual meshes")
    for rel in wanted:
        target = dest / rel
        if force or not target.is_file():
            log(f"  {rel}")
            _write(target, _get(MESH_URL.format(name=Path(rel).name)))

    _write(dest / "reBot_Lite_RS_with_gripper.urdf", urdf)
    manifest = {
        "source": URDF_URL,
        "note": "third-party CAD (Seeed reBot Arm) fetched from Motorbridge Studio; do not redistribute",
        "links": links,
    }
    manifest_path = dest / "manifest.json"
    _write(manifest_path, json.dumps(manifest, indent=1).encode())
    total = sum(f.stat().st_size for f in dest.rglob("*") if f.is_file())
    log(f"done: {total / 1e6:.1f} MB in {dest}")
    return manifest_path
