"""Headless physics and kinematics; upstream CAD/models are never overwritten."""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "Simulation/SO101"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
TOOLS = ("Gripper", "Spatula concept", "Whisk concept")
HOME = np.array([0., -0.35, 0.65, 0.15, 0., 0.45])


@dataclasses.dataclass(frozen=True)
class Config:
    tool: str = "Gripper"
    payload_g: float = 0.0
    torque_nm: float = 1.6
    camera: bool = True
    camera_fovy: float = 70.0
    tool_force_n: float = 0.0
    # These are assumptions for experiments, not calibrated servo parameters.
    kp: float = 80.0
    kv: float = 2.7

    def __post_init__(self):
        if self.tool not in TOOLS:
            raise ValueError(f"Unknown tool: {self.tool}")
        for name, low, high in (("payload_g", 0, 500), ("torque_nm", .1, 3.35),
                                ("camera_fovy", 25, 120), ("tool_force_n", 0, 20),
                                ("kp", 1, 1000), ("kv", 0, 30)):
            value = getattr(self, name)
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be finite and within [{low}, {high}]")


def follower_xml(config: Config) -> str:
    path = MODEL_DIR / ("so101_new_calib_camera.xml" if config.camera else "so101_new_calib.xml")
    root = ET.parse(path).getroot()
    root.find("compiler").set("meshdir", str(MODEL_DIR / "assets"))
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    ET.SubElement(visual, "headlight", ambient="0.5 0.5 0.5", diffuse="0.7 0.7 0.7")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="0.3 -0.4 1.5", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "geom", name="bench", type="plane", size="1 1 .02", pos="0 0 -.004",
                  rgba=".15 .19 .24 1", friction=".8 .01 .001")
    # Restore a conservative base collider omitted by upstream. The feet still
    # need measured geometry; this box covers the central fixed base only.
    base = root.find(".//body[@name='base']")
    ET.SubElement(base, "geom", name="base_proxy", type="box", pos=".005 0 .019",
                  size=".025 .033 .019", group="3", rgba=".3 .6 .8 .2")
    # This proxy overlaps the shoulder's convex mesh around their joint. Only
    # this adjacent pair is excluded; other arm links can hit the fixed base.
    contact = ET.SubElement(root, "contact")
    ET.SubElement(contact, "exclude", body1="base", body2="shoulder")
    cube = ET.SubElement(world, "body", name="practice_cube", pos=".40 -.02 .014")
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(cube, "geom", name="cube", type="box", size=".018 .018 .018", mass=".025",
                  rgba=".12 .65 .85 1", friction="1 .01 .001", condim="4")
    # A second camera gives an external view, as used in demonstration collection.
    ET.SubElement(world, "camera", name="overhead", pos=".25 0 .9", quat="1 0 0 0", fovy="55")
    grip = root.find(".//body[@name='gripper']")
    if config.camera:
        # Virtual optical frame relative to the wrist. Physical camera mesh and
        # mass are upstream's; these extrinsics/FOV must be calibrated later.
        camera_body = root.find(".//body[@name='wrist_camera']")
        ET.SubElement(camera_body, "camera", name="wrist_rgb", pos="0 0 .030",
                      quat="0 1 0 0", fovy=str(config.camera_fovy))
    tcp = np.array([-.0079, -.000218121, -.0981274])
    if config.tool != "Gripper":
        tool = ET.SubElement(grip, "body", name="tool_concept", pos=" ".join(map(str, tcp)))
        # Rigid attachment is an explicit mounting assumption, not a simulated
        # successful grasp or an engineered quick-change coupler.
        ET.SubElement(tool, "geom", name="tool_shank", type="box", size=".006 .003 .040",
                      pos="0 0 -.025", mass=".025", rgba=".65 .7 .75 1")
        if config.tool == "Spatula concept":
            ET.SubElement(tool, "geom", name="spatula_blade", type="box", size=".030 .004 .035",
                          pos="0 0 -.10", mass=".045", rgba=".8 .82 .84 1")
            tcp[2] -= .135
        else:
            # Several capsules approximate the outer wires, retaining gaps.
            for i in range(8):
                a = 2 * np.pi * i / 8
                x, y = .024 * np.cos(a), .024 * np.sin(a)
                for j, (p, q) in enumerate((("0 0 -.065", f"{x} {y} -.105"),
                                            (f"{x} {y} -.105", "0 0 -.145"))):
                    ET.SubElement(tool, "geom", name=f"whisk_{i}_{j}", type="capsule",
                                  fromto=f"{p} {q}", size=".0015", mass=".0025", rgba=".8 .82 .84 1")
            tcp[2] -= .145
    root.find(".//site[@name='gripperframe']").set("name", "stock_gripperframe")
    ET.SubElement(grip, "site", name="tcp", pos=" ".join(map(str, tcp)), size=".004", rgba=".2 1 .6 1")
    # Test mass at the working tip, independent of tool/camera mass.
    payload = ET.SubElement(grip, "body", name="test_payload", pos=" ".join(map(str, tcp)))
    mass = max(config.payload_g / 1000., 1e-6)
    ET.SubElement(payload, "inertial", mass=str(mass), pos="0 0 0",
                  diaginertia=" ".join([str(mass * .00004)] * 3))
    ET.SubElement(payload, "geom", name="payload_marker", type="sphere", size=".012",
                  contype="0", conaffinity="0", group="2",
                  rgba=f".85 .3 .5 {'.7' if config.payload_g > 0 else '0'}")
    for actuator in root.findall("actuator/position"):
        actuator.set("kp", str(config.kp))
        actuator.set("kv", str(config.kv))
        actuator.set("forcerange", f"{-config.torque_nm} {config.torque_nm}")
    return ET.tostring(root, encoding="unicode")


def leader_model() -> mujoco.MjModel:
    root = ET.parse(MODEL_DIR / "so101_leader_new_calib.urdf").getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", Path(mesh.get("filename")).name)
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", meshdir=str(MODEL_DIR / "assets"), discardvisual="false")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


class Simulation:
    def __init__(self, config: Config = Config()):
        self.config = config
        self.model = mujoco.MjModel.from_xml_string(follower_xml(config))
        self.data = mujoco.MjData(self.model)
        self.jids = np.array([self.model.joint(n).id for n in JOINTS])
        self.qids = self.model.jnt_qposadr[self.jids]
        self.vids = self.model.jnt_dofadr[self.jids]
        self.limits = self.model.jnt_range[self.jids].copy()
        self.site = self.model.site("tcp").id
        self.target = HOME.copy()
        self.rate_limit = np.deg2rad(45.)  # soft command slew, rad/s
        self.reset()

    @property
    def q(self):
        return self.data.qpos[self.qids].copy()

    @property
    def tip(self):
        return self.data.site_xpos[self.site].copy()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qids] = HOME
        self.data.ctrl[:] = HOME
        self.target = HOME.copy()
        mujoco.mj_forward(self.model, self.data)

    def command(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("A command must contain six finite joint angles")
        self.target = np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def step(self, steps: int = 10):
        dt = self.model.opt.timestep
        for _ in range(steps):
            self.data.ctrl[:] += np.clip(self.target - self.data.ctrl,
                                         -self.rate_limit * dt, self.rate_limit * dt)
            self.data.qfrc_applied[:] = 0
            if self.config.tool_force_n:
                mujoco.mj_applyFT(self.model, self.data, np.array([0., 0., -self.config.tool_force_n]),
                                 np.zeros(3), self.tip, self.model.body("gripper").id,
                                 self.data.qfrc_applied)
            mujoco.mj_step(self.model, self.data)
        # mj_step integrates qpos after computing kinematics. Refresh so cameras,
        # contacts, TCP and recorded qpos all refer to the same instant.
        mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or any(w.number for w in self.data.warning):
            raise RuntimeError("Physics became unstable; reset and reduce load or gains")

    def contacts(self, data=None):
        data = self.data if data is None else data
        result = []
        for c in data.contact:
            if c.dist > .0001:
                continue
            bodies = [int(self.model.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
            names = ["bench" if b == 0 else mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in bodies]
            # Resting cube contact isn't a robot collision.
            if set(names) <= {"practice_cube", "bench"}:
                continue
            result.append({"bodies": names, "penetration_mm": round(max(0., -c.dist) * 1000, 3)})
        return result

    def ik(self, target, seed=None):
        """Position-only IK; reject unreachable/colliding goals and sampled paths.

        This is a reach aid, not a collision-avoiding motion planner. Five arm
        joints cannot give arbitrary position plus arbitrary tool orientation.
        """
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Target must be a finite xyz vector in metres")
        d = mujoco.MjData(self.model)
        start = self.q if seed is None else np.array(seed, dtype=float)
        def evaluate(x):
            d.qpos[:] = self.data.qpos
            d.qpos[self.qids] = np.r_[x, start[5]]
            mujoco.mj_forward(self.model, d)
            return d.site_xpos[self.site] - target
        best = None
        for guess in (start[:5], HOME[:5], np.array([0, -.8, 1.2, -.5, 0.])):
            fit = least_squares(evaluate, np.clip(guess, self.limits[:5, 0], self.limits[:5, 1]),
                                bounds=(self.limits[:5, 0], self.limits[:5, 1]),
                                diff_step=1e-4, max_nfev=160, ftol=1e-8, gtol=1e-8, xtol=1e-8)
            error = float(np.linalg.norm(evaluate(fit.x)))
            if best is None or error < best[0]:
                best = error, np.r_[fit.x, start[5]]
            if error < .001:
                break
        error, q = best
        if error > .003:
            return {"ok": False, "error_mm": error * 1000, "reason": "Outside reachable workspace", "q": q}
        for alpha in np.linspace(0, 1, 41):
            d.qpos[:] = self.data.qpos
            d.qpos[self.qids] = start * (1 - alpha) + q * alpha
            mujoco.mj_forward(self.model, d)
            hits = [c for c in self.contacts(d) if c["penetration_mm"] > 1.]
            if hits:
                return {"ok": False, "error_mm": error * 1000,
                        "reason": "Sampled path intersects geometry", "contacts": hits, "q": q}
        return {"ok": True, "error_mm": error * 1000, "reason": "Position target found", "q": q}

    def state(self):
        tau = self.data.actuator_force.copy()
        return {"time_s": float(self.data.time), "command_rad": self.target.tolist(),
                "actual_rad": self.q.tolist(), "velocity_rad_s": self.data.qvel[self.vids].tolist(),
                "torque_nm": tau.tolist(), "stall_fraction": (np.abs(tau) / self.config.torque_nm).tolist(),
                "tcp_m": self.tip.tolist(), "contacts": self.contacts(),
                "config": dataclasses.asdict(self.config)}

    def save(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.state(), indent=2) + "\n")


def map_to_leader(q, follower_limits, leader_limits):
    result = np.array(q, dtype=float).copy()
    # Leader trigger and follower jaw have different ranges, unlike the 5 arm joints.
    fraction = np.clip((q[5] - follower_limits[5, 0]) / np.ptp(follower_limits[5]), 0, 1)
    result[5] = leader_limits[5, 0] + fraction * np.ptp(leader_limits[5])
    return np.clip(result, leader_limits[:, 0], leader_limits[:, 1])
