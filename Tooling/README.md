# ATC-02: automatic interchangeable tools for SO-101

This version replaces the follower's permanently mounted gripper with a common
receiver. Four **complete modules** park in a rack: two-finger gripper,
four-finger tendon gripper, spatula holder and powered stirrer. The leader stays
stock. A camera bracket remains on the arm-side receiver.

**Status: digital prototype and printable fit-test CAD.** The simulation completes
automatic changes. Actual spring-pin retention, blind mating, tool actuation,
contact force, firmware and repeated physical docking have not been qualified.
Do not interpret a completed digital cycle as a working physical tool changer.

## What to open

- `Build/Tooling/parts/`: editable STEP and assembly-coordinate STL files.
- `Build/Tooling/print/`: components translated onto Z=0, sized for the A1 mini.
  Assembly meshes are excluded. These are individual parts, not sliced jobs.
- `Build/Tooling/manifest.json`: CAD validity, dimensions, source mounting-hole
  extraction, estimated print mass, hashes and dock poses.
- [Mechanism and assembly notes](MECHANICAL.md).
- [Electrical architecture and pin assignment](electronics/INTERFACE.md).
- [Research and design rationale](RESEARCH.md).
- [Physical acceptance procedure](ACCEPTANCE.md).

Run the workbench as described in [WORKBENCH.md](../WORKBENCH.md). In **Vision &
tools**, select a module and click **Automatically change to selected tool**,
or **Cycle all four modules**. **Apply setup + reset** changes experimental
parameters, not the module selection. Changing is allowed only without added
test payload or external force. The planner assumes an empty tool.

## Automatic cycle

1. Stop tool motion, close the ordinary gripper and move through a clear posture.
2. Align with the current tool's dock, approach from 150 mm behind its seating
   plane, and insert the tool into its support. Close the rack retaining gate.
3. Confirm capture/support/alignment, disable tool power, and wait for measured rail
   voltage below 0.5 V. The digital voltage decay is only a model.
4. Retract the lock using the dock actuator; confirm unlocked, transfer support
   to the stand and withdraw the empty receiver.
5. Align with the requested module, seat, engage the spring-retained pin, and
   confirm locked/present/correct tool ID before opening the rack gate.
6. Initialize the module at zero command, restore power, open the rack gate,
   withdraw, and return home.

Missing support, a stuck power rail, wrong identity or failed lock prevents the
next step and ends in a fault. Reset requires deliberate operator action. No
automatic retry or automatic unlock is performed after a fault.

## What is physically modeled

Five arm joints retain upstream limits. Every tool is a separate free body. Each
has an independent rack-retention constraint and arm-coupling constraint. A tool
remains supported by at least one constraint throughout a successful change.
The two-finger jaw moves; eight hinged proxies represent the four compliant
fingers driven by one tendon; the stirring shaft has a velocity actuator.

The receiver, tools and rack render from generated CAD. Collision proxies are
separate from concave CAD meshes so the receiver socket is not accidentally
filled by a convex collision hull. The guide key, steel pin, rack captures and
electrical contacts are **not contact-mechanism simulations**. Their retention
is represented by ideal welds. All lock/support/ID/voltage signals are explicitly
synthetic. TPU hinges approximate motion rather than predicting material strain.

The arm uses an experimental position gain of 300 with actuator torque capped at
1.60 Nm. That ceiling is an approximate stall figure, not a continuous rating.
The seated-position gate is a simulation tolerance and is looser than a final
spring-contact interface may permit. Physical calibration and compliance are
required; repeatability and insertion forces are not established here.

## Regenerate and verify

```sh
uv sync --python 3.12 --group dev --group cad
uv run --group cad python Tooling/cad/generate.py
uv run pytest -q
```

The generator validates each exported solid and removes only degenerate or
duplicate tessellation faces. It does not patch holes. It also checks that the
nominal key and receiver have zero solid interference in the seated CAD pose.

Camera, tool masses, friction, motor response and material parameters remain
estimates. There is no physical servo adapter, flashable MCU firmware, trained
policy or hardware deployment in this version.
