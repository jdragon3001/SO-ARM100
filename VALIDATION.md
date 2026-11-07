# ATC-02 validation — 7 November 2026

**49 automated checks passed** on Apple Silicon/macOS, Python 3.12, MuJoCo
3.17.0 and Viser 1.1.1. Run `uv run pytest -q`. CAD generation uses build123d
0.13.0 in the optional `cad` dependency group. Dependencies are pinned.

## Digital evidence

- All four initial modules settle with and without the wrist camera.
- All **12 directed changes** between different modules complete.
- A full four-tool round trip completes while each tool remains supported by a
  rack or arm constraint. The rack capture is active before the wrist unlocks.
- Missing support, stuck rail voltage, wrong tool identity and failed lock block
  advancement, time out and disable tool power.
- Tool power removal actually zeros the actuator gains/force, rather than merely
  displaying an off indicator. The shaft coasts; fingers remain passive.
- One tendon command moves all four fingers. The stirrer has its own actuator.
- Full dock orientation and pre-approach reach are checked against arm limits.
- Arm commands, invalid configuration, loaded-change rejection, cameras,
  synchronized FK, snapshots, independent interlocks and passive IDs are checked.
- The reference bus rejects corrupt/replayed/out-of-range commands and stops on
  watchdog expiry. This is not a flashed firmware test.
- Generated printable components fit the reserved A1 mini bed envelope. The
  original stock-arm plate repack still preserves its source triangles.

Interactive verification: opened the updated local UI, completed **Cycle all
four modules**, confirmed READY with the gripper reattached, saved a camera/state
snapshot and exported the full scene preview. No physical robot was connected.

## Nominal cycle audit

Reproduce `Build/Tooling/validation.json` with:

```sh
uv run python Tooling/design/validate_changes.py
```

| Requested module | Simulation seconds to change | Result |
|---|---:|---|
| Four-finger gripper | 19.44 | READY |
| Spatula | 21.08 | READY |
| Powered stirrer | 22.68 | READY |
| Two-finger gripper | 22.88 | READY |

These timings are simulation-clock results, not hardware cycle-time promises.
The audit records phase-level synthetic sensor state, power and rack capture,
peak arm torques, samples at the assumed ceiling and reported contact penetration.
Shoulder/elbow torque reaches the **1.60 Nm ceiling during motion**. That is a
stall-based model limit, not a continuous rating; these runs establish neither
thermal capability nor an acceptable physical duty cycle. Lower speeds, reduced
mass and measured motor response may be required.

The nominal first exchange reports a transient penetration of approximately
0.60 mm with the contact proxies; the other three audited exchanges report none
above the 0.5 mm reporting threshold. The digital change faults above 1.5 mm.
Those are numerical/prototype thresholds, not electrical mating tolerances or
proof of interference-free physical parts.

## CAD evidence

The generator exports **46 valid solid STEP/STL pairs**, of which **41 are
individual printable component definitions**; some must be printed multiple
times. Full dock meshes and the assembled stirrer housing are reference
assemblies, not single printable objects. Every component print mesh is watertight
and fits within 170×170×180 mm. The longest printable component dimension is
approximately 151 mm.

The actual upstream four-hole horn interface is checked from STEP cylinder axes.
The nominal receiver/insert solid intersection is zero. A 12 mm crank over ±17°
provides a calculated 7.02 mm pin-release stroke. Fit coupons, nominal clearance,
part bounds, source/output hashes and rack poses are recorded in the manifest.
These checks do not validate screw engagement, spring force, wear or loaded fit.

## Physical evidence still required

Key/pin/gate retention uses ideal weld constraints, and feedback is synthetic.
Insertion/contact-spring forces and TPU material behavior are not modeled.
Rack actuator installation is not a constrained, validated assembly. Contact
selection, actual camera, PCB, flashed MCU firmware, speed feedback, shaft
retention, physical load limits and repeated automatic docking remain bench
integration and testing. Follow [Tooling/ACCEPTANCE.md](Tooling/ACCEPTANCE.md).

This package is a working **digital automatic-change prototype and fit-test CAD**,
not a qualified physical automatic tool changer or cooking system.
