# SO-101 workbench — ATC-02

This fork now runs a **five-axis arm with four complete interchangeable tool
modules and an automatic-change rack**. It supersedes the earlier fixed
spatula/whisk-on-gripper concepts. The leader remains the stock hand-operated
SO-101 design. Original upstream files are preserved.

## Start

```sh
uv sync --python 3.12
uv run python Simulation/Workbench/app.py
```

Open http://127.0.0.1:8080. On this Mac, `Start Workbench.command` runs the same
server. It binds only to this computer. Ctrl-C stops it.

## Automatic changes

In **Vision & tools**, choose **Two-finger gripper**, **Four-finger gripper**,
**Spatula** or **Powered stirrer**, then click **Automatically change to selected
tool**. **Cycle all four modules** runs a complete round trip. Watch the arm
return its current module, withdraw empty, pick up the next, and return home.
The selected tool's empty stand remains visible. The camera stays on the receiver.

The change display shows phase, attached tool, lock, rail voltage and power.
**Stop change + hold position** ends in a fault and requires reset. Missing
feedback does not automatically cause retries or release. Tool changes require
zero added test payload/external force and assume an empty tool.

## Operate a module

The five joint sliders move the arm. **Tool command %** has a module-specific
meaning: original gripper travel, four-finger tendon pull, or 0–60 RPM stirring
target. Enable tool power with its button first. There is no motor in the
spatula. The leader's trigger represents this tool command; the other five
leader joints correspond to the arm joints.

The 3D target and **Solve target + move** are a position-only reach helper.
Tool orientation is free; this helper is not the oriented docking planner and
does not guarantee a collision-free path. The automatic changer uses precomputed
oriented dock/pre-dock poses. Manual motion can still hit objects or the rack.
The arm's five positioning joints cannot realize every arbitrary six-axis pose.

**Apply setup + reset** changes camera, assumed actuator ceiling and load
parameters while retaining the current tool type. It resets the world. Changing
the dropdown by itself does not teleport a different tool onto the wrist.

## Vision and recording

Wrist and overhead views are MuJoCo-rendered RGB, 640×480 at nominally 10 Hz of
simulation time. The wrist camera is an undecided 32 mm module with assumed FOV
and optical pose; physical calibration remains necessary.

**Record** saves `metadata.json`, `frames.jsonl` and camera PNGs in
`Simulation/Workbench/runs/`. Format `so-arm-workbench/2` records **five arm
angles**, a separate tool command/observation, actual active tool, change phase,
rack capture, synthetic sensors, motor torque, contacts and camera transforms.
It intentionally differs from v1's six-angle arm/gripper record. Use timestamps;
wall-clock performance can be slower than simulation time. Files are never
uploaded automatically and are ignored by Git.

**Save pose + camera snapshot** captures one synchronized state. **Export 3D
preview** saves the scene to `Build/workbench-preview.png` (or an inspector close-up if open). No tool is connected
to a physical USB/serial adapter. There is no trained policy or native LeRobot
dataset exporter in this version.

## Inspect the lock and power path

Open **Wrist & power → Open wrist close-up**. The receiver and key are sectioned
from the STEP solids so the 3 mm steel pin and bore are visible. Scrub the
coupling step or click **Play lock and release**. With **Power and run** selected,
switch to **Tool motion** to try the grippers, the stirrer and the passive
spatula. The preview uses separate simulation state and cannot move the main arm.
The contact diagram is placed off the parts: the board and its mounting have
not been designed in place. **Frame live wrist** in **Vision & tools** zooms the
main simulation instead. **Save HD pose + camera snapshot** writes 1280×960 stills
with matching camera intrinsics, without changing the recording size.

## Evidence and limitations

The arm includes gravity, inertia, limits, motor torque saturation, joint
damping and command slew. Modules are separate free bodies. The articulated jaw,
four-finger tendon proxies and rotating shaft have independent actuators.
The rendered rack and moving gates are checked against the scene geometry at
each 2 ms step. An intersection stops motion and reports the parts and phase.
This is a sampled surface check; MuJoCo proxy contact forces remain separate.

**Coupling and rack retention are ideal weld constraints.** Key/pin/gate contact,
spring-contact mating force, flexible TPU material, actual lock sensors and
electrical behavior are not physically simulated or measured. Sensor inputs and
the rail decay are synthetic. Passing a digital cycle is not qualification of
physical automatic changes. Printed mass and motor dynamics are estimates;
1.60 Nm is an approximate stall ceiling, not a continuous duty rating.

See [Tooling/README.md](Tooling/README.md), [research](Tooling/RESEARCH.md),
[mechanical notes](Tooling/MECHANICAL.md), [electrical interface](Tooling/electronics/INTERFACE.md)
and [physical acceptance](Tooling/ACCEPTANCE.md). New CAD is in `Build/Tooling/`.
Stock A1 mini arm plates remain in `Build/A1mini/`.

```sh
uv sync --group dev --group cad --python 3.12
uv run --group cad python Tooling/cad/generate.py
uv run pytest -q
```
