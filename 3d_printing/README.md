# 3D printing reference assembly

This folder contains an Autodesk Fusion builder and its source geometry for a
four-component reference layout:

- Leo Rover, using official low-polygon collision meshes and official URDF transforms;
- Seeed reBot Arm B601-DM (`p-6740`), using Seeed's complete v1.1 STEP assembly;
- MEAN WELL LRS-600N2-48 power supply, modeled from the 225 x 124 x 41 mm Case No. 292 drawing;
- a generic medium 15-inch laptop envelope (340 x 240 x 18 mm base).

The components are arranged separately in a 2 x 2 grid. This is a reference
layout, not a claimed mounting solution. In particular, the requested B601-DM
model is not the B601-RS arm used by the sorter software and physical rig.

## Open the Fusion project

Open `fusion_output/alien_bazaar_reference_assembly.f3d` in Autodesk Fusion.
The browser tree contains one top-level component for each requested item.

To regenerate it:

1. In Fusion, open **Utilities > Add-Ins > Scripts and Add-Ins**.
2. On the **Add-Ins** tab, add
   `3d_printing/fusion/AlienBazaarAssembly` and click **Run**.
3. Wait for `fusion_output/build_status.json` to report `success`.

The builder recreates and overwrites only the generated F3D archive and its
status file. Fusion uses centimetres internally; all source dimensions are
converted at the script boundary.

## Fidelity and licensing

The reBot arm is detailed vendor STEP geometry. The rover is the official ROS
collision representation, suitable for layout and clearance work but not for
manufacturing individual rover parts. The power supply and laptop are envelope
models; replace the laptop component after the demo-machine model is known.

Source URLs, representation notes, and SHA-256 hashes are recorded in
`sources.json`. Third-party license texts are kept alongside their assets. The
Leo Rover ROS description is MIT licensed. Seeed's hardware CAD is
CERN-OHL-W-2.0 licensed.

