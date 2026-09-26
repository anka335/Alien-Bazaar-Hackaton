"""Build the Alien Bazaar reference assembly in Autodesk Fusion.

Fusion uses centimetres internally. Source measurements below are converted
to centimetres at the boundary. The four requested items intentionally stay
as separate top-level components in a 2 x 2 reference layout.
"""

from __future__ import annotations

import json
import os
import traceback
from pathlib import Path

import adsk.core
import adsk.fusion


PROJECT_DIR = Path(
    os.environ.get("ALIEN_BAZAAR_3D_ROOT", Path(__file__).resolve().parents[2])
).resolve()
ASSETS_DIR = PROJECT_DIR / "assets"
OUTPUT_DIR = PROJECT_DIR / "fusion_output"
OUTPUT_F3D = OUTPUT_DIR / "alien_bazaar_reference_assembly.f3d"
STATUS_FILE = OUTPUT_DIR / "build_status.json"


def _write_status(status: str, **details: object) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    payload = {"status": status, **details}
    STATUS_FILE.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _transform(
    x_cm: float = 0.0,
    y_cm: float = 0.0,
    z_cm: float = 0.0,
    rz_rad: float = 0.0,
) -> adsk.core.Matrix3D:
    matrix = adsk.core.Matrix3D.create()
    if rz_rad:
        matrix.setToRotation(
            rz_rad,
            adsk.core.Vector3D.create(0, 0, 1),
            adsk.core.Point3D.create(0, 0, 0),
        )
    matrix.translation = adsk.core.Vector3D.create(x_cm, y_cm, z_cm)
    return matrix


def _new_component(
    parent: adsk.fusion.Component,
    name: str,
    transform: adsk.core.Matrix3D | None = None,
) -> adsk.fusion.Occurrence:
    occurrence = parent.occurrences.addNewComponent(
        transform or adsk.core.Matrix3D.create()
    )
    occurrence.component.name = name
    return occurrence


def _add_box(
    component: adsk.fusion.Component,
    name: str,
    length_cm: float,
    width_cm: float,
    height_cm: float,
    center_x_cm: float = 0.0,
    center_y_cm: float = 0.0,
    bottom_z_cm: float = 0.0,
) -> adsk.fusion.BRepBody:
    box = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(
            center_x_cm,
            center_y_cm,
            bottom_z_cm + height_cm / 2,
        ),
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0),
        length_cm,
        width_cm,
        height_cm,
    )
    temporary = adsk.fusion.TemporaryBRepManager.get().createBox(box)
    body = component.bRepBodies.add(temporary)
    body.name = name
    return body


def _add_cylinder(
    component: adsk.fusion.Component,
    name: str,
    center_x_cm: float,
    center_y_cm: float,
    bottom_z_cm: float,
    radius_cm: float,
    height_cm: float,
) -> adsk.fusion.BRepBody:
    manager = adsk.fusion.TemporaryBRepManager.get()
    temporary = manager.createCylinderOrCone(
        adsk.core.Point3D.create(center_x_cm, center_y_cm, bottom_z_cm),
        radius_cm,
        adsk.core.Point3D.create(
            center_x_cm, center_y_cm, bottom_z_cm + height_cm
        ),
        radius_cm,
    )
    body = component.bRepBodies.add(temporary)
    body.name = name
    return body


def _add_mesh_component(
    parent: adsk.fusion.Component,
    name: str,
    source: Path,
    transform: adsk.core.Matrix3D,
) -> None:
    occurrence = _new_component(parent, name, transform)
    imported = occurrence.component.meshBodies.add(
        os.fspath(source), adsk.fusion.MeshUnits.MeterMeshUnit
    )
    if not imported or imported.count == 0:
        raise RuntimeError(f"Fusion did not import mesh: {source}")
    for index in range(imported.count):
        imported.item(index).name = name


def _add_leorover(root: adsk.fusion.Component) -> None:
    rover = _new_component(root, "LeoRover (official collision geometry)")
    component = rover.component
    source_dir = ASSETS_DIR / "leorover"

    # Fixed transforms from leo_description/urdf/macros.xacro. The official
    # collision meshes use metres and remain mesh bodies in the F3D archive.
    base_z = 19.783
    _add_mesh_component(
        component,
        "Chassis",
        source_dir / "Chassis_outline.stl",
        _transform(z_cm=base_z),
    )
    _add_mesh_component(
        component,
        "Left rocker",
        source_dir / "Rocker_outline.stl",
        _transform(0.263, 14.167, base_z - 4.731, 3.141592653589793),
    )
    _add_mesh_component(
        component,
        "Right rocker",
        source_dir / "Rocker_outline.stl",
        _transform(0.263, -14.167, base_z - 4.731),
    )

    wheel_positions = (
        ("Front-left wheel", 15.519, 22.381, 6.250, 3.141592653589793),
        ("Rear-left wheel", -14.993, 22.381, 6.250, 3.141592653589793),
        ("Front-right wheel", 15.519, -22.381, 6.250, 0.0),
        ("Rear-right wheel", -14.993, -22.381, 6.250, 0.0),
    )
    for name, x_cm, y_cm, z_cm, rz_rad in wheel_positions:
        _add_mesh_component(
            component,
            name,
            source_dir / "Wheel_outline.stl",
            _transform(x_cm, y_cm, z_cm, rz_rad),
        )

    _add_cylinder(component, "Antenna envelope", -0.52, 5.6, 19.133, 0.55, 5.6)
    component.attributes.add(
        "alien_bazaar",
        "source",
        "LeoRover/leo_common-ros2, MIT; transforms from macros.xacro",
    )


def _add_rebot_arm(
    app: adsk.core.Application, root: adsk.fusion.Component
) -> None:
    occurrence = _new_component(
        root,
        "reBot Arm B601-DM p-6740 (official STEP)",
        _transform(x_cm=60.0),
    )
    source = (
        ASSETS_DIR
        / "rebot_b601_dm"
        / "reBot_B601_DM_v1.1_20260625.step"
    )
    options = app.importManager.createSTEPImportOptions(os.fspath(source))
    options.isViewFit = False
    imported = app.importManager.importToTarget2(options, occurrence.component)
    if not imported:
        raise RuntimeError(f"Fusion did not import STEP assembly: {source}")
    occurrence.component.attributes.add(
        "alien_bazaar",
        "source",
        "Seeed-Projects/reBot-DevArm v1.1 2026-06-25, CERN-OHL-W-2.0",
    )


def _add_power_supply(root: adsk.fusion.Component) -> None:
    occurrence = _new_component(
        root,
        "MEAN WELL LRS-600N2-48 power supply",
        _transform(y_cm=-60.0),
    )
    component = occurrence.component
    # Manufacturer mechanical envelope: 225 x 124 x 41 mm.
    _add_box(component, "Case (225 x 124 x 41 mm)", 22.5, 12.4, 4.1)
    _add_cylinder(component, "Cooling fan clearance", 5.4, 0.0, 4.1, 2.35, 0.2)
    for index in range(9):
        _add_box(
            component,
            f"Terminal {index + 1}",
            1.2,
            0.9,
            0.8,
            center_x_cm=-4.8 + index * 1.2,
            center_y_cm=-5.75,
            bottom_z_cm=4.1,
        )
    component.attributes.add(
        "alien_bazaar",
        "source",
        "MEAN WELL LRS-600N2 datasheet, Case No. 292",
    )


def _add_laptop(root: adsk.fusion.Component) -> None:
    occurrence = _new_component(
        root,
        "Medium laptop (generic 15-inch envelope)",
        _transform(x_cm=60.0, y_cm=-60.0),
    )
    component = occurrence.component
    _add_box(component, "Base (340 x 240 x 18 mm)", 34.0, 24.0, 1.8)
    _add_box(
        component,
        "Display (340 x 12 x 220 mm)",
        34.0,
        1.2,
        22.0,
        center_y_cm=11.4,
        bottom_z_cm=1.8,
    )
    _add_box(
        component,
        "Keyboard clearance",
        29.0,
        10.5,
        0.15,
        center_y_cm=-2.0,
        bottom_z_cm=1.8,
    )
    component.attributes.add(
        "alien_bazaar",
        "source",
        "Generic layout envelope; replace when the laptop model is selected",
    )


def _build() -> Path:
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        raise RuntimeError("The active Fusion product is not a design")
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent

    _add_leorover(root)
    _add_rebot_arm(app, root)
    _add_power_supply(root)
    _add_laptop(root)

    root.attributes.add("alien_bazaar", "layout", "2x2 reference grid; centimetres")
    root.attributes.add(
        "alien_bazaar",
        "warning",
        "B601-DM is the requested reference model; the sorter rig uses B601-RS",
    )
    app.activeViewport.fit()
    app.doEvents()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if OUTPUT_F3D.exists():
        OUTPUT_F3D.unlink()
    options = design.exportManager.createFusionArchiveExportOptions(
        os.fspath(OUTPUT_F3D), root
    )
    if not design.exportManager.execute(options):
        raise RuntimeError(f"Fusion failed to export {OUTPUT_F3D}")

    _write_status(
        "success",
        output=os.fspath(OUTPUT_F3D),
        top_level_components=[
            "LeoRover (official collision geometry)",
            "reBot Arm B601-DM p-6740 (official STEP)",
            "MEAN WELL LRS-600N2-48 power supply",
            "Medium laptop (generic 15-inch envelope)",
        ],
    )
    app.log(f"Alien Bazaar Fusion archive exported to {OUTPUT_F3D}")
    return OUTPUT_F3D


def run(context: dict[str, object]) -> None:
    del context
    try:
        _write_status("running")
        _build()
    except Exception:
        error = traceback.format_exc()
        _write_status("failed", error=error)
        adsk.core.Application.get().log(error)


def stop(context: dict[str, object]) -> None:
    del context
