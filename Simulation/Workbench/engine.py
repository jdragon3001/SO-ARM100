"""ATC-02 digital prototype. Five arm axes, separate powered tool modules.
Couplings/rack use ideal welds, sensors are synthetic, no hardware is driven.
"""

from __future__ import annotations
import dataclasses, json, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "Tooling/design"))
from spec import ARM_JOINTS, BUILD, HOME, MODULES, KEYS, DOCK_Q, TRANSIT, dock_poses
from assets import build as add_tools
from interlocks import Sensors, may_unlock, may_depart, may_energize

MODEL_DIR = ROOT / "Simulation/SO101"
JOINTS = ARM_JOINTS
LEADER_JOINTS = (*ARM_JOINTS, "gripper")
TOOLS = tuple(v["label"] for v in MODULES.values())
LABEL_TO_KEY = {v["label"]: k for k, v in MODULES.items()}


@dataclasses.dataclass(frozen=True)
class Config:
    tool: str = "Two-finger gripper"
    payload_g: float = 0.0
    torque_nm: float = 1.6
    camera: bool = True
    camera_fovy: float = 70.0
    tool_force_n: float = 0.0
    kp: float = 300.0
    kv: float = 3.0

    def __post_init__(self):
        if self.tool not in TOOLS:
            raise ValueError(f"Unknown tool {self.tool}")
        for name, lo, hi in (
            ("payload_g", 0, 500),
            ("torque_nm", 0.1, 3.35),
            ("camera_fovy", 25, 120),
            ("tool_force_n", 0, 20),
            ("kp", 1, 1000),
            ("kv", 0, 30),
        ):
            x = getattr(self, name)
            if not np.isfinite(x) or not lo <= x <= hi:
                raise ValueError(f"Invalid {name}")


def follower_xml(config):
    root = ET.parse(MODEL_DIR / "so101_new_calib.xml").getroot()
    root.find("compiler").set("meshdir", str(MODEL_DIR / "assets"))
    ET.SubElement(
        root, "option", timestep=".002", integrator="implicitfast", gravity="0 0 -9.81"
    )
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="640", offheight="480")
    ET.SubElement(vis, "headlight", ambient=".5 .5 .5", diffuse=".7 .7 .7")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos=".2 -.4 1.5", dir="0 0 -1")
    ET.SubElement(
        world,
        "geom",
        name="bench",
        type="plane",
        size="1 1 .02",
        pos="0 0 -.004",
        rgba=".15 .19 .24 1",
    )
    cube = ET.SubElement(world, "body", name="practice_cube", pos=".32 -.20 .014")
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(
        cube,
        "geom",
        name="cube",
        type="box",
        size=".018 .018 .018",
        mass=".025",
        rgba=".12 .65 .85 1",
        friction="1 .01 .001",
    )
    ET.SubElement(
        world, "camera", name="overhead", pos=".22 0 .9", quat="1 0 0 0", fovy="60"
    )
    for a in root.findall("actuator/position"):
        a.set("kp", str(config.kp))
        a.set("kv", str(config.kv))
        a.set("forcerange", f"{-config.torque_nm} {config.torque_nm}")
    add_tools(
        root,
        LABEL_TO_KEY[config.tool],
        config.camera,
        config.camera_fovy,
        config.payload_g,
    )
    contact = ET.SubElement(root, "contact")
    for key in KEYS:
        ET.SubElement(contact, "exclude", body1="receiver", body2=f"tool_{key}")
        for side in (-1, 1):
            ET.SubElement(
                contact, "exclude", body1=f"ledge_{key}_{side}", body2=f"tool_{key}"
            )
    return ET.tostring(root, encoding="unicode")


def leader_model():
    root = ET.parse(MODEL_DIR / "so101_leader_new_calib.urdf").getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", Path(mesh.get("filename")).name)
    ex = ET.SubElement(root, "mujoco")
    ET.SubElement(
        ex, "compiler", meshdir=str(MODEL_DIR / "assets"), discardvisual="false"
    )
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


class Simulation:
    def __init__(self, config=Config()):
        self.config = config
        self.model = mujoco.MjModel.from_xml_string(follower_xml(config))
        self.data = mujoco.MjData(self.model)
        self.drive_gains = self.model.actuator_gainprm.copy()
        self.drive_bias = self.model.actuator_biasprm.copy()
        self.jids = np.array([self.model.joint(n).id for n in JOINTS])
        self.qids = self.model.jnt_qposadr[self.jids]
        self.vids = self.model.jnt_dofadr[self.jids]
        self.limits = self.model.jnt_range[self.jids].copy()
        self.site = self.model.site("tcp").id
        self.receiver_id = self.model.body("receiver").id
        self.rate_limit = np.deg2rad(35)
        self.poses = dock_poses()
        self.moves = {}
        self.fault_injection = None
        self.reset()
        for key, q in zip(KEYS, DOCK_Q):
            p, r = self.poses[key]
            pre = self.solve_receiver(p - r @ [0, 0, 0.174], r, q)
            if pre["error_m"] > 0.0001 or pre["angle_rad"] > 0.002:
                raise ValueError(f"Dock alignment failed {key}: {pre}")
            self.moves[key] = {
                "seat": q.copy(),
                "pre": pre["q"],
                "transit": np.r_[q[0], TRANSIT[1:]],
            }

    @property
    def q(self):
        return self.data.qpos[self.qids].copy()

    @property
    def tip(self):
        return self.data.site(
            f"tip_{self.attached}" if self.attached else "receiver_face"
        ).xpos.copy()

    @property
    def busy(self):
        return self.phase not in ("READY", "FAULT")

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qids] = HOME
        self.data.ctrl[:5] = HOME
        self.target = HOME.copy()
        self.attached = LABEL_TO_KEY[self.config.tool]
        self.requested = self.attached
        self.rack_captured = {k: k != self.attached for k in KEYS}
        self.tool_command = 0.0
        self.power_enabled = False
        self.rail_v = 0.0
        self.locked = True
        self.phase = "READY"
        self.error = ""
        self.plan = []
        self.elapsed = 0.0
        self.change_log = []
        self.set_tool_control(0.0)
        mujoco.mj_forward(self.model, self.data)
        self.update_tcp()

    def update_tcp(self):
        z = 0.024 + (MODULES[self.attached]["tcp_mm"][2] / 1000 if self.attached else 0)
        self.model.site_pos[self.site] = [0, 0, z]
        mujoco.mj_forward(self.model, self.data)

    def set_tool_control(self, value):
        if not np.isfinite(value) or not 0 <= value <= 1:
            raise ValueError("Tool command must be in [0,1]")
        self.tool_command = float(value)
        for key, name in (
            ("gripper", "tool_gripper"),
            ("soft_gripper", "tool_soft_gripper"),
            ("stirrer", "tool_stirrer"),
        ):
            active = key == self.attached and self.power_enabled
            fraction = self.tool_command if active and self.phase == "READY" else 0.0
            aid = self.model.actuator(name).id
            self.model.actuator_gainprm[aid] = self.drive_gains[aid] if active else 0
            self.model.actuator_biasprm[aid] = self.drive_bias[aid] if active else 0
            self.data.ctrl[aid] = (
                -0.05 + fraction * 1.6
                if key == "gripper"
                else fraction * 0.05
                if key == "soft_gripper"
                else fraction * 2 * np.pi
            )

    def command(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError("Expected five finite arm angles")
        if self.busy or self.phase == "FAULT":
            raise ValueError("Changer owns arm motion; reset after a fault")
        self.target = np.clip(q, self.limits[:, 0], self.limits[:, 1])

    def solve_receiver(self, p, r, seed):
        d = mujoco.MjData(self.model)
        d.qpos[:] = self.data.qpos

        def residual(q):
            d.qpos[self.qids] = q
            mujoco.mj_forward(self.model, d)
            orient = Rotation.from_matrix(
                r.T @ d.xmat[self.receiver_id].reshape(3, 3)
            ).as_rotvec()
            return np.r_[d.xpos[self.receiver_id] - p, orient * 0.1]

        fit = least_squares(
            residual,
            np.clip(seed, self.limits[:, 0], self.limits[:, 1]),
            bounds=(self.limits[:, 0], self.limits[:, 1]),
            max_nfev=120,
            xtol=1e-10,
            gtol=1e-10,
            ftol=1e-10,
            diff_step=1e-4,
        )
        err = residual(fit.x)
        return {
            "q": fit.x,
            "error_m": float(np.linalg.norm(err[:3])),
            "angle_rad": float(np.linalg.norm(err[3:]) / 0.1),
        }

    def ik(self, target, seed=None):
        p = np.asarray(target, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("Expected finite XYZ")
        d = mujoco.MjData(self.model)
        d.qpos[:] = self.data.qpos
        start = self.q if seed is None else np.array(seed)

        def residual(q):
            d.qpos[self.qids] = q
            mujoco.mj_forward(self.model, d)
            r = d.xmat[self.receiver_id].reshape(3, 3)
            z = 0.024 + (
                MODULES[self.attached]["tcp_mm"][2] / 1000 if self.attached else 0
            )
            return d.xpos[self.receiver_id] + r @ [0, 0, z] - p

        best = None
        for guess in (start, HOME, np.array([0, -0.5, 0.7, 0.4, 0])):
            fit = least_squares(
                residual,
                np.clip(guess, self.limits[:, 0], self.limits[:, 1]),
                bounds=(self.limits[:, 0], self.limits[:, 1]),
                max_nfev=150,
                diff_step=1e-4,
            )
            error = float(np.linalg.norm(residual(fit.x)))
            if best is None or error < best[0]:
                best = (error, fit.x)
            if error < 0.001:
                break
        error, q = best
        ok = error < 0.003 and p[2] > 0.01
        return {
            "ok": ok,
            "q": q,
            "error_mm": error * 1000,
            "reason": "Position target found; orientation free"
            if ok
            else "Target outside usable workspace",
        }

    def sensors(self, key=None):
        key = self.attached if key is None else key
        supported = seated = False
        if key:
            p, r = self.poses[key]
            t = self.data.body(f"tool_{key}")
            a = self.data.body("receiver")
            supported = (
                np.linalg.norm(t.xpos - p) < 0.004
                and np.linalg.norm(
                    Rotation.from_matrix(r.T @ t.xmat.reshape(3, 3)).as_rotvec()
                )
                < 0.06
            )
            seated = (
                np.linalg.norm(a.xpos + a.xmat.reshape(3, 3) @ [0, 0, 0.024] - t.xpos)
                < 0.003
                and np.linalg.norm(
                    Rotation.from_matrix(
                        t.xmat.reshape(3, 3).T @ a.xmat.reshape(3, 3)
                    ).as_rotvec()
                )
                < 0.035
            )
        s = Sensors(
            dock_supported=bool(supported and self.rack_captured.get(key, False)),
            seated=bool(seated),
            locked=self.locked,
            unlocked=not self.locked,
            tool_present=self.attached is not None,
            tool_id=self.attached,
            rail_v=self.rail_v,
            tool_stopped=bool(abs(self.data.joint("stirrer_spin").qvel[0]) < 0.1),
            power_good=self.power_enabled,
        )
        changes = {
            "wrong_id": {"tool_id": "unknown"},
            "lock_failed": {"locked": False, "unlocked": True},
            "rail_stuck_on": {"rail_v": 7.4},
            "unsupported": {"dock_supported": False},
        }
        return dataclasses.replace(s, **changes.get(self.fault_injection, {}))

    def change_tool(self, label):
        key = LABEL_TO_KEY.get(label, label)
        if key not in KEYS:
            raise ValueError("Unknown module")
        if self.busy or self.phase == "FAULT":
            raise ValueError("Changer busy/faulted; reset first")
        if self.config.payload_g or self.config.tool_force_n:
            raise ValueError("Remove test payload/force before an automatic change")
        if key == self.attached:
            return
        old = self.attached
        self.requested = key
        self.set_tool_control(0)
        self.plan = []
        if old:
            if not may_depart(self.sensors(old), old):
                raise ValueError("Current tool identity/lock not verified")
            self.power_enabled = True
            self.plan.append(("STOP_TOOL", "stop", old))

        def move(name, q):
            self.plan.append((name, "move", q.copy()))

        if old:
            move("CLEAR_CURRENT", np.r_[self.q[0], TRANSIT[1:]])
            for phase, point in (
                ("ALIGN_OLD", "transit"),
                ("APPROACH_OLD", "pre"),
                ("SEAT_OLD", "seat"),
            ):
                move(phase, self.moves[old][point])
            self.plan += [
                ("CAPTURE_OLD", "capture", old),
                ("VERIFY_SUPPORT", "support", old),
                ("DISCHARGE", "discharge", old),
                ("UNLOCK", "unlock", old),
                ("PARK_OLD", "detach", old),
            ]
            move("RETRACT_EMPTY", self.moves[old]["pre"])
            move("CLEAR_RACK", self.moves[old]["transit"])
        for phase, point in (
            ("ALIGN_NEW", "transit"),
            ("APPROACH_NEW", "pre"),
            ("SEAT_NEW", "seat"),
        ):
            move(phase, self.moves[key][point])
        self.plan += [
            ("LOCK", "lock", key),
            ("IDENTIFY", "identify", key),
            ("VERIFY_LOCK", "verify", key),
            ("POWER_READY", "enable", key),
            ("OPEN_RACK", "uncapture", key),
        ]
        move("WITHDRAW_NEW", self.moves[key]["pre"])
        move("CLEAR_NEW", self.moves[key]["transit"])
        move("RETURN_HOME", HOME)
        self.plan += [("ENABLE_TOOL", "enable", key)]
        self.phase = "START"
        self.elapsed = 0.0
        self.advance()

    def advance(self):
        if not self.plan:
            self.phase = "READY"
            self.target = self.q
            self.elapsed = 0.0
            return
        self.phase, self.action, self.argument = self.plan.pop(0)
        self.elapsed = 0.0
        key = self.argument if isinstance(self.argument, str) else self.attached
        self.change_log.append(
            {
                "time_s": float(self.data.time),
                "phase": self.phase,
                "tool": self.attached,
                "sensors": dataclasses.asdict(self.sensors(key)),
                "power_enabled": self.power_enabled,
                "rack_captured": dict(self.rack_captured),
            }
        )
        if self.action == "move":
            self.target = self.argument.copy()

    def fail(self, reason):
        self.error = reason
        self.phase = "FAULT"
        self.plan = []
        self.power_enabled = False
        self.tool_command = 0.0
        self.target = self.q
        self.set_tool_control(0.0)
        self.change_log.append(
            {"time_s": float(self.data.time), "phase": "FAULT", "reason": reason}
        )

    def tick_change(self, dt):
        if not self.busy:
            return
        self.elapsed += dt
        if self.elapsed > 18:
            self.fail(f"Timeout in {self.phase}")
            return
        key = self.argument if isinstance(self.argument, str) else self.attached
        if self.action == "move":
            if (
                np.max(abs(self.q - self.target)) < 0.008
                and np.max(abs(self.data.qvel[self.vids])) < 0.03
            ):
                self.advance()
        elif self.action == "stop":
            s = self.sensors(key)
            closed = (
                key != "gripper"
                or abs(self.data.joint("gripper").qpos[0] + 0.05) < 0.08
            )
            if s.tool_stopped and closed:
                self.advance()
        elif self.action == "capture":
            if self.elapsed > 0.3 and self.sensors(key).seated:
                self.rack_captured[key] = True
                self.data.eq_active[self.model.equality(f"park_{key}").id] = 1
                self.advance()
        elif self.action == "support":
            s = self.sensors(key)
            if s.dock_supported and s.seated and s.tool_stopped:
                self.advance()
        elif self.action == "discharge":
            self.power_enabled = False
            if self.sensors(key).rail_v < 0.5:
                self.advance()
        elif self.action == "unlock":
            if self.elapsed > 0.4 and may_unlock(self.sensors(key)):
                self.locked = False
                self.advance()
        elif self.action == "detach":
            s = self.sensors(key)
            if s.unlocked and s.dock_supported:
                self.data.eq_active[self.model.equality(f"park_{key}").id] = 1
                self.data.eq_active[self.model.equality(f"couple_{key}").id] = 0
                self.attached = None
                self.update_tcp()
                self.advance()
        elif self.action == "lock":
            s = self.sensors(key)
            if self.elapsed > 0.4 and s.seated and s.dock_supported and s.rail_v < 0.5:
                self.data.eq_active[self.model.equality(f"couple_{key}").id] = 1
                self.locked = True
                self.attached = key
                self.update_tcp()
                self.advance()
        elif self.action == "identify":
            if self.sensors(key).tool_id == key:
                self.advance()
        elif self.action == "verify":
            if may_depart(self.sensors(key), key):
                self.advance()
        elif self.action == "uncapture":
            if self.elapsed > 0.3 and may_depart(self.sensors(key), key):
                self.rack_captured[key] = False
                self.data.eq_active[self.model.equality(f"park_{key}").id] = 0
                self.advance()
        elif self.action == "enable":
            if may_energize(self.sensors(key), key):
                self.power_enabled = True
                self.advance()

    def step(self, steps=10):
        dt = self.model.opt.timestep
        for _ in range(steps):
            self.model.geom_pos[self.model.geom("locking_pin").id, 0] = 0.0215 + (
                0 if self.locked else 0.007
            )
            for key in KEYS:
                p, r = self.poses[key]
                self.model.body_pos[self.model.body("rack_gate_" + key).id] = [
                    0,
                    0,
                    p[2],
                ] + r @ np.array([0, 0 if self.rack_captured[key] else -0.020, 0])
            self.rail_v += ((6.0 if self.power_enabled else 0) - self.rail_v) * min(
                1, dt / 0.08
            )
            self.data.ctrl[:5] += np.clip(
                self.target - self.data.ctrl[:5],
                -self.rate_limit * dt,
                self.rate_limit * dt,
            )
            self.data.qfrc_applied[:] = 0
            if self.attached and self.config.tool_force_n and not self.busy:
                mujoco.mj_applyFT(
                    self.model,
                    self.data,
                    np.array([0.0, 0.0, -self.config.tool_force_n]),
                    np.zeros(3),
                    self.tip,
                    self.model.body(f"tool_{self.attached}").id,
                    self.data.qfrc_applied,
                )
            self.set_tool_control(self.tool_command)
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.tick_change(dt)
        if self.busy and any(c["penetration_mm"] > 1.5 for c in self.contacts()):
            self.fail("Unexpected contact during change")
        if not np.isfinite(self.data.qpos).all() or any(
            w.number for w in self.data.warning
        ):
            self.fail("Physics instability; reset required")

    def contacts(self, data=None):
        d = self.data if data is None else data
        hits = []
        for c in d.contact:
            if c.dist > -0.0005:
                continue
            ids = [int(self.model.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
            names = ["bench" if b == 0 else self.model.body(b).name for b in ids]
            if set(names) <= {"bench", "practice_cube"}:
                continue
            hits.append({"bodies": names, "penetration_mm": float(-c.dist * 1000)})
        return hits

    def state(self):
        return {
            "time_s": float(self.data.time),
            "command_rad": self.target.tolist(),
            "actual_rad": self.q.tolist(),
            "velocity_rad_s": self.data.qvel[self.vids].tolist(),
            "torque_nm": self.data.actuator_force[:5].tolist(),
            "stall_fraction": (
                np.abs(self.data.actuator_force[:5]) / self.config.torque_nm
            ).tolist(),
            "tcp_m": self.tip.tolist(),
            "contacts": self.contacts(),
            "config": dataclasses.asdict(self.config),
            "tool": self.attached,
            "requested_tool": self.requested,
            "tool_command": self.tool_command,
            "changer_phase": self.phase,
            "error": self.error,
            "synthetic_sensors": dataclasses.asdict(self.sensors()),
            "rack_captured": dict(self.rack_captured),
            "tool_observation": {
                "gripper_rad": float(self.data.joint("gripper").qpos[0]),
                "finger_rad": [
                    float(self.data.joint(f"finger_{i}_{s}").qpos[0])
                    for i in range(4)
                    for s in ("prox", "dist")
                ],
                "stirrer_rpm": float(
                    self.data.joint("stirrer_spin").qvel[0] * 60 / (2 * np.pi)
                ),
            },
            "coupling_model": "ideal weld; physical latch/insertion unqualified",
        }

    def save(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.state(), indent=2) + "\n")


def map_to_leader(q, follower_limits, leader_limits, tool_command=0):
    result = np.r_[
        q, leader_limits[5, 0] + np.clip(tool_command, 0, 1) * np.ptp(leader_limits[5])
    ]
    return np.clip(result, leader_limits[:, 0], leader_limits[:, 1])
