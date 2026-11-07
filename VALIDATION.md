# ATC-02 rack revision 2 validation — 7 November 2026

The original rack let rendered tools and holders intersect while the cycle
reported READY. The old check only saw MuJoCo contacts from simplified ledge and
tool proxies. It did not test the rendered posts or retaining gates. The
original dock and gate files are superseded.

## What the revised test checks

- The workbench independently checks rendered rack and gate surfaces against
  the arm, tool heads, camera, cube and other stations at **every 2 ms physics
  step**, including the full gate stroke. It faults before advancing the changer
  and leaves the first offending pose visible. The fault records the phase,
  simulation time and part pair. Proxy contacts are reported separately.
- All **12 directed changes** between different modules complete under that
  guard. A separate four-tool round trip checks that the tool remains retained
  during each support transfer and that the tool rail is off before unlocking.
- A deliberate obstruction moves only the rendered gate into the spatula's
  rack envelope. The test checks that MuJoCo reports no proxy contact, the
  geometry guard faults during gate opening, both retention constraints stay
  active, and tool power is disabled. The simulation stays at the fault pose.
- The separate CAD check reports **0 mm³** solid overlap between each fixed
  module body and its dock at the nominal parked pose, and between the gate and
  the dock/module at 0, 10 and 20 mm of gate opening. The runtime check includes
  the moving jaw, fingers and purchased-tool geometry represented in the viewer.

The runtime check uses the same meshes as the viewer and tests surface
intersection at discrete poses. It is not a continuous clearance certificate
or a contact force solver. It does not establish a tolerance budget, pressure
on the mating contacts, strength or durability. See [MuJoCo collision
documentation](https://mujoco.readthedocs.io/en/stable/computation/) and the
[FCL mesh query interface](https://github.com/BerkeleyAutomation/python-fcl).

## Nominal cycle audit

Reproduce [the JSON trace](Build/Tooling/validation.json) with:

```sh
uv run python Tooling/design/validate_changes.py
uv run pytest -q
```

| Requested module | Simulation seconds | Rack geometry samples | Rack intersections |
|---|---:|---:|---:|
| Four-finger gripper | 30.40 | 15,200 | 0 |
| Spatula | 32.60 | 16,300 | 0 |
| Powered stirrer | 34.84 | 17,420 | 0 |
| Two-finger gripper | 35.44 | 17,720 | 0 |

The audit records phase changes, synthetic sensor state, tool power, torque
and separately sampled proxy contacts. No proxy penetration above the 0.5 mm
reporting threshold was recorded in the round trip. Shoulder/elbow torque
still reaches the assumed **1.60 Nm stall ceiling** in motion. These results do
not establish cycle time or acceptable duty cycle on hardware.

## Wrist inspection

The STEP audit samples the 7 mm pin stroke at eight positions and the key's
approach with the pin retracted at nine positions. Both report zero solid
overlap. A 1 mm pull from nominal seating with the pin engaged intersects the
key by 7.42 mm³, establishing geometric capture for this one rigid motion.
This does not predict pin strength, print wear, play or retention under load.
See [the pin audit](Build/Tooling/inspection/pin_geometry.json).

The browser inspector was checked with the lock and contact overlay visible
and with modeled jaw and stirrer motion in a separate state. The 1280×960 wrist
and overhead snapshots were checked against their reported dimensions and
camera centers. The electrical overlay is a proposal, separate from the CAD
and from the actual simulation actuator model.

## Build and bench limits

The generator exports **46 valid STEP/STL pairs** and **41** printable component
definitions. All printable meshes are watertight and fit within 170×170×180 mm.
The largest component dimension is approximately 168 mm; check supports and
brims in the slicer. The actual upstream four-hole horn interface and nominal
receiver/key solid fit are checked during generation.

Rack and wrist retention still use ideal weld constraints. The rack has a
small nominal gap around the ears. Its hardware locating pads, gate linkage,
spring pin, electrical mating and actual tool envelope must be checked on a
single physical dock before printing the full rack. Camera cabling, TPU
behavior, real motor response, and repeated hardware docking remain untested.
Follow [the bench acceptance procedure](Tooling/ACCEPTANCE.md).
