"""Shared v0.2 prototype dimensions. Millimetres in CAD, metres in MuJoCo.

These are design values, not measured or certified hardware ratings.
"""

from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "Build/Tooling"
VERSION = "ATC-02"
MOUNT_HALF_PITCH = 4.9497474683058  # extracted from upstream STEP cylinder axes
RECEIVER_HEIGHT = 24.0
INSERTION = 9.0
CLEARANCE = 0.25  # per side, start with coupons
PIN_DIAMETER = 3.0
PIN_TRAVEL = 7.0
PIN_TIP_X = 8.0
PIN_LENGTH = 27.0
PIN_AXIS_Z = 20.0
LIVE_RESOLUTION = (640, 480)
SNAPSHOT_RESOLUTION = (1280, 960)
EAR_X = 55.0
RACK_FRONT = 8.5  # axial position of ledge face; 2.5 mm ahead of the ear
RACK_GATE_TRAVEL = 20.0
DOCK_PANS = [-1.02, -0.34, 0.34, 1.02]
# Put rack ears across the station, rather than one above the other.
DOCK_Q = [np.array([p, 0.3, -0.6, 0.9, np.pi / 2]) for p in DOCK_PANS]
HOME = np.array([-1.25, -0.6, 0.0, 0.8, 0.0])
TRANSIT = np.array([0.0, -1.0, -0.5, 0.2, 0.0])
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
MODULES = {
    "gripper": {
        "label": "Two-finger gripper",
        "mode": "position",
        "tcp_mm": [0, 0, 111],
        "hardware_g": 55,
        "colour": [0.96, 0.68, 0.16, 1],
        "id_ohms": 1000,
    },
    "soft_gripper": {
        "label": "Four-finger gripper",
        "mode": "tendon",
        "tcp_mm": [0, 0, 100],
        "hardware_g": 65,
        "colour": [0.22, 0.72, 0.52, 1],
        "id_ohms": 2200,
    },
    "spatula": {
        "label": "Spatula",
        "mode": "passive",
        "tcp_mm": [0, 0, 105],
        "hardware_g": 25,
        "colour": [0.34, 0.64, 0.92, 1],
        "id_ohms": 4700,
    },
    "stirrer": {
        "label": "Powered stirrer",
        "mode": "velocity",
        "tcp_mm": [0, 0, 125],
        "hardware_g": 28,
        "colour": [0.75, 0.42, 0.85, 1],
        "id_ohms": 10000,
    },
}
KEYS = tuple(MODULES)


def source_model():
    import mujoco

    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "Simulation/SO101/so101_new_calib.xml")
    )
    return model, mujoco.MjData(model)


def receiver_pose(q):
    import mujoco

    model, data = source_model()
    data.qpos[:5] = q
    mujoco.mj_forward(model, data)
    body = model.body("gripper").id
    return data.xpos[body].copy(), data.xmat[body].reshape(3, 3) @ np.diag(
        [1.0, -1.0, -1.0]
    )


def dock_poses():
    result = {}
    for key, q in zip(KEYS, DOCK_Q):
        p, r = receiver_pose(q)
        result[key] = (p + r @ np.array([0.0, 0.0, RECEIVER_HEIGHT / 1000]), r)
    return result
