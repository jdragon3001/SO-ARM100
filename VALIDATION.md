# Workbench validation

Tested locally on Apple Silicon/macOS with Python 3.12, MuJoCo 3.17.0 and Viser
1.1.1. Dependencies are pinned in `uv.lock`. Command: `uv run pytest -q`.

**21 checks passed** for all three tool choices with/without the camera,
finite stable integration, joint command bounds and slew, torque saturation,
payload/external force response, position IK acceptance/rejection, collision
rejection below the bench, leader trigger mapping, configuration validation,
RGB rendering, synchronized camera/TCP kinematics, snapshots and A1 mini geometry preservation.

At the default home configuration, after five seconds of simulation:

| Extra test mass | Shoulder lift torque magnitude | Elbow torque magnitude | Interpretation |
|---|---:|---:|---|
| 0 g | 0.438 Nm | 0.497 Nm | Camera-equipped stock gripper |
| 100 g | 0.690 Nm | 0.759 Nm | Higher static load |
| 250 g | 1.066 Nm | 1.148 Nm | Higher static load |
| 500 g | 1.600 Nm | 1.600 Nm | Assumed ceiling reached; roughly 3.2°/7.9° tracking error |

These are model outputs for one pose and assumed actuator settings. They do not
establish a safe payload, continuous-duty capability or physical performance.

The A1 mini package preserves all 12 source solids per role and all triangles,
matches aggregate source volume within floating-point export tolerance, keeps
each plate's geometry within the reserved bed area, and checks part bounding
boxes for the specified gap. Slicing, support and material checks remain physical
build steps.

Interactive verification: opened the local browser workbench, ran the motion
demo, switched the tool setup, rendered wrist/external camera feeds, and recorded
and saved an episode. The local recording files are excluded from Git.
The final recorded demo contained 179 frames over 19.76 simulation seconds;
timestamps increased, all referenced RGB files were present at 320 × 240, and
camera calibration matrices and six-joint state/action vectors were verified.

This validates a useful first simulation workspace. It does not validate a
quick-change mount, real servo control, a trained policy, food/heat handling,
or physical grasp reliability.
