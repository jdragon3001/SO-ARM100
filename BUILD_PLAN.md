# Build plan: SO-101 pair, vision and tool experiments

Confirmed: Bambu Lab A1 mini; camera undecided; a hand-operated leader and powered
follower; future demonstrations with spatula/whisk-style interchangeable tools.

## Printable baseline

`Build/A1mini/leader/` and `Build/A1mini/follower/` each contain **three STL
plates** and matching SVG footprint diagrams. Import one plate at a time into
Bambu Studio, select the A1 mini, and inspect the sliced preview before printing.
These are geometry files, not sliced jobs or G-code.

The upstream files named `BambuLabA1mini_*` span approximately 382 × 372 mm
(leader) and 372 × 369 mm (follower). This fork separates their 12 watertight
solids per role and repacks them for a 180 × 180 mm bed. Only translations and
90° rotations around Z are applied. Existing print orientations, triangles and
solid volumes are preserved, including the tiny separate solid in the follower
jaw file. It must not be dropped silently.

The layouts reserve a 5 mm bed margin and 6 mm between solid bounding boxes.
Supports or brims can extend into that spacing; inspect those separately and
move a part to its own plate if needed. Geometry fitting the bed does not prove
that the chosen support/brim strategy fits.

`Build/A1mini/manifest.json` records the original file checksums, source solid
IDs, output checksums, dimensions, transformations and triangle-count-based part
name candidates. The original source files remain unchanged. Reproduce the package:

```sh
uv run python Simulation/Workbench/prepare_prints.py
```

First print the appropriate upstream fit gauges in `STL/Gauges/`. Use the servo
gauges when you have a servo, or the Lego gauges for an initial printer fit check.
For the stock arm, upstream's starting settings are PLA+, a 0.4 mm nozzle with
0.2 mm layers, and 15% infill. Preserve the source orientation and inspect support
placement, especially horizontal screw holes. These settings concern the stock
structure, not suitability for a hot kitchen tool.

## Camera baseline

The simulation includes the existing
[SO-101 hex-nut camera mount](Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module/README.md).
Its STEP/STL files are in that folder. It uses the follower wrist's existing nut
recesses and is specific to SO-101. The upstream mounting hardware is four M2
screws, two M3 × 8 mm screws and two M3 nuts. The mount guide specifies 40%
infill and tree supports, rather than the stock arm's infill setting.

Before printing this optional mount, select a camera and verify its PCB size,
hole pitch, lens clearance, connector access, cable bend and focus distance.
The existing 32 × 32 mm module is a simulation placeholder, not a purchase
decision. Measure camera/mount/cable mass and replace the assumptions in the
model. Check visibility of both the tool and the work surface over the whole
task; the tool can occlude the wrist camera. The external camera in the
simulation is a separate viewpoint, useful for planning data collection.

## Interchangeable tool interface

Keep the leader mechanically stock for the first iteration. Change its software
mapping only when needed to match the follower's tool behavior. The follower's
tool selection must eventually set the matching TCP, payload, camera calibration
and dataset task/tool identifier.

The spatula and whisk in this version are **rigidly mounted simulation concepts**,
not printable adapters. Their assumed reach and mass let us reject poor layouts
before designing the mount. The stock jaw remains in the scene; it does not
prove that a real utensil can be held securely.

The first physical tooling iteration should use a lightweight holder for a
removable commercial utensil. Before making adapter CAD, measure the available
gripper/wrist interface and utensil handle, choose retention and anti-rotation
features, and agree the operating loads. We need:

1. Tool insertion/removal clearance and repeatable alignment.
2. Mechanical retention under pull, twist and reversal, with a measured load
   target. A friction-only connection has not been validated.
3. A repeatable TCP for each tool and an unambiguous tool identification method.
4. No interference with the camera, motor cabling, moving jaw or work surface.
5. A camera/mount/tool mass budget and measured servo tracking under task forces.

Keep early demonstrations to cold, dry tabletop tasks: moving a lightweight
utensil between marked poses, stirring empty space, or pushing a light block.
Whisking viscous material and pressing a spatula add reaction forces that a
gravity-only payload test misses. The workbench includes a downward external
force test for this reason, but it does not model the material being stirred.
Heat, cleanability, food-contact materials and tool retention need separate
physical assessment before cooking. No hot-food or stove operation is validated.

## Iteration order

| Stage | Concrete result before proceeding |
|---|---|
| Review this simulation | Agree on arm layout, camera visibility, working area and first tool |
| Select camera and utensil | Actual dimensions, masses, lens/FOV and attachment measurements |
| Design holder | STEP + STL, retention features, TCP and collision/inertia updates |
| Print fit pieces | Gauge and small interface test before full new components |
| Build/calibrate stock pair | Correct motor gearing, wiring, zero convention, joint limits and camera calibration |
| Record demonstrations | Real synchronized follower state, leader actions and camera streams through LeRobot |
| Train one narrow task | Separate training/evaluation episodes, repeatable success criterion, supervised trials |

The follower uses six 1/345 servos. The leader uses different gear ratios by
joint; do not purchase twelve identical follower servos for the pair. Follow the
[official assembly and calibration guide](https://huggingface.co/docs/lerobot/so101)
and the repository's existing BOM. Training follows the
[official imitation-learning workflow](https://huggingface.co/docs/lerobot/il_robots).
