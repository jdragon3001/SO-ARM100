# Build plan: A1 mini and automatic interchangeable tools

The design target is now **automatic tool changing**, superseding the earlier
manual-swap/fixed-tool concepts. See [Tooling/README.md](Tooling/README.md) for
the current design and its qualification limits.

## Start with fit pieces

1. Stock servo/Lego printer-fit gauges from `STL/Gauges/`.
2. `Build/Tooling/print/test_plug.stl` and three socket coupons.
3. Receiver, side cover, collar and one contact carrier fit coupon.
4. One segmented dock, gate and release-linkage bench assembly.
5. One removable gripper, then the other modules after the interface works.

New `print/` files are individual components placed on Z=0. STEP files and
assembly-coordinate meshes are in `Build/Tooling/parts/`. Do not send assembly
meshes to the printer: full rack pedestals and the complete stirrer housing must
be assembled from their separate printable sections. The manifest distinguishes
`printable_component` from assembly meshes and verifies bed extents.

`Tooling/BOM.csv` lists candidate hardware and explicitly marks parts that remain
unselected or need measurements. The electrical package is a design/reference
contract, not a routed board or flashable controller. Follow the physical
[acceptance procedure](Tooling/ACCEPTANCE.md) before powered automatic changes.

## Stock arms

`Build/A1mini/leader/` and `Build/A1mini/follower/` retain three repacked baseline
arm plates each. The upstream files named `BambuLabA1mini_*` span roughly
370–380 mm; this fork preserves their 12 solids per role while arranging them
for a 180 mm bed. Triangle counts/volume and source hashes are checked. Brims and
supports can require extra room. These are STL layouts, not G-code.

The follower baseline plate includes the original permanent-gripper parts. For
the modular build, the wrist end is replaced by the new receiver/module; use
the individual stock parts plus the new assembly plan rather than blindly
printing both complete end assemblies. The leader remains stock.

Use upstream arm-print settings as a starting point: PLA+, 0.4 mm nozzle and
0.2 mm layers. Check horizontal holes and source orientation in Bambu Studio.
TPU fingers and structural rack/receiver parts require their own profiles and
fit/load tests; stock arm settings do not validate them.

## Remaining physical work

Choose the camera, actual contacts, spring, sensors, shaft/coupling, blade and
controller components. Constrain the dock actuator assemblies around actual
parts, implement the controller firmware, and collect fit/load/docking evidence.
The simulator's automatic exchanges are ready for exploration; physical
autonomous changes have not been commissioned. No cooking operation is validated.
