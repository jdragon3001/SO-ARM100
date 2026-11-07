"""Build four separate free tool bodies with an idealized lock/rack interface."""

import copy, json
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from spec import *


def nums(x):
    return " ".join(str(float(v)) for v in x)


def quat(r):
    q = Rotation.from_matrix(r).as_quat()
    return np.r_[q[3], q[:3]]


def geom(parent, **kw):
    return E.SubElement(
        parent,
        "geom",
        **{
            k: nums(v) if isinstance(v, (list, tuple, np.ndarray)) else str(v)
            for k, v in kw.items()
        },
    )


def visual(root, parent, name, colour, pos=(0, 0, 0), q=(1, 0, 0, 0)):
    if root.find(f"asset/mesh[@name='atc_{name}']") is None:
        E.SubElement(
            root.find("asset"),
            "mesh",
            name="atc_" + name,
            file=str(BUILD / "parts" / f"{name}.stl"),
            scale=".001 .001 .001",
        )
    geom(
        parent,
        name=parent.get("name", "world") + "_visual_" + name,
        type="mesh",
        mesh="atc_" + name,
        group=2,
        contype=0,
        conaffinity=0,
        rgba=colour,
        pos=pos,
        quat=q,
    )


def box(parent, name, size, pos, colour=(0.2, 0.3, 0.4, 0)):
    geom(parent, name=name, type="box", size=size, pos=pos, group=3, rgba=colour)


def mass(parent, m, centre=(0, 0, 0.035), radius=0.03):
    E.SubElement(
        parent,
        "inertial",
        mass=str(m),
        pos=nums(centre),
        diaginertia=nums([max(1e-8, m * radius**2)] * 3),
    )


def build(root, initial_tool, camera=True, fovy=70, payload_g=0):
    manifest = json.loads((BUILD / "manifest.json").read_text())
    world = root.find("worldbody")
    old = root.find(".//body[@name='gripper']")
    for child in list(old):
        if child.tag != "joint":
            old.remove(child)
    old.set("name", "wrist_output")
    receiver = E.SubElement(old, "body", name="receiver", quat="0 1 0 0")
    mass(
        receiver,
        manifest["parts"]["receiver"]["estimated_print_g"] / 1000
        + 0.009
        + (0.035 if camera else 0),
        (0, 0.005, 0.015),
    )
    visual(root, receiver, "receiver", (0.78, 0.8, 0.83, 1))
    visual(
        root,
        receiver,
        "pin_housing_cover",
        (0.35, 0.4, 0.45, 1),
        (0.0275, -0.008, 0.018),
        (0.70710678, 0.70710678, 0, 0),
    )
    geom(
        receiver,
        name="locking_pin",
        type="cylinder",
        size=(0.0015, 0.0135),
        pos=(0.0215, 0, 0.020),
        quat=(0.70710678, 0, 0.70710678, 0),
        rgba=(0.8, 0.83, 0.86, 1),
        contype=0,
        conaffinity=0,
        group=2,
    )
    box(receiver, "receiver_base", (0.021, 0.021, 0.0035), (0, 0, 0.0035))
    for side in (-1, 1):
        box(
            receiver,
            f"receiver_wall_x_{side}",
            (0.003, 0.018, 0.0085),
            (side * 0.015, 0, 0.0155),
        )
        box(
            receiver,
            f"receiver_wall_y_{side}",
            (0.012, 0.003, 0.0085),
            (0, side * 0.015, 0.0155),
        )
    E.SubElement(receiver, "site", name="receiver_face", pos="0 0 .024", size=".003")
    E.SubElement(receiver, "site", name="tcp", pos="0 0 .135", size=".004")
    if camera:
        visual(root, receiver, "camera_bracket_32mm", (0.26, 0.28, 0.32, 1))
        geom(
            receiver,
            name="camera_board",
            type="box",
            size=(0.016, 0.016, 0.004),
            pos=(0, 0.044, 0.031),
            rgba=(0.08, 0.09, 0.1, 1),
            contype=0,
            conaffinity=0,
            group=2,
        )
        p = np.array([0.0, 0.044, 0.038])
        z = p - np.array([0.0, 0.0, 0.15])
        z /= np.linalg.norm(z)
        x = np.cross([0, -1, 0], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        E.SubElement(
            receiver,
            "camera",
            name="wrist_rgb",
            pos=nums(p),
            quat=nums(quat(np.column_stack([x, y, z]))),
            fovy=str(fovy),
        )
    equality = root.find("equality")
    if equality is None:
        equality = E.SubElement(root, "equality")
    for a in list(root.find("actuator")):
        if a.get("joint") == "gripper":
            root.find("actuator").remove(a)
    tendon = E.SubElement(root, "tendon")
    poses = dock_poses()
    hp, hr = receiver_pose(HOME)
    for key, spec in MODULES.items():
        p, r = poses[key]
        dock = E.SubElement(
            world, "body", name=f"dock_{key}", pos=nums([p[0], p[1], 0])
        )
        visual(root, dock, f"dock_{key}", (0.24, 0.3, 0.38, 1))
        for side in (-1, 1):
            ledge = E.SubElement(
                dock,
                "body",
                name=f"ledge_{key}_{side}",
                pos=nums([0, 0, p[2]]),
                quat=nums(quat(r)),
            )
            box(
                ledge,
                f"rack_ledge_{key}_{side}",
                (0.007, 0.011, 0.002),
                (side * EAR_X / 1000, 0, (RACK_FRONT + 2) / 1000),
            )
        gate = E.SubElement(
            dock,
            "body",
            name=f"rack_gate_{key}",
            pos=nums([0, 0, p[2]]),
            quat=nums(quat(r)),
        )
        visual(root, gate, "dock_retaining_gate", (0.72, 0.32, 0.20, 1))
        ip, ir = (hp + hr @ [0, 0, 0.024], hr) if key == initial_tool else (p, r)
        tool = E.SubElement(
            world, "body", name=f"tool_{key}", pos=nums(ip), quat=nums(quat(ir))
        )
        E.SubElement(tool, "freejoint", name=f"free_{key}")
        part = {
            "gripper": "gripper_body",
            "soft_gripper": "four_finger_palm",
            "spatula": "spatula_holder",
            "stirrer": "stirrer_housing",
        }[key]
        m = (
            manifest["parts"][part]["estimated_print_g"] / 1000
            + spec["hardware_g"] / 1000
            + (payload_g / 1000 if key == initial_tool else 0)
        )
        mass(tool, m)
        visual(root, tool, part, spec["colour"])
        box(tool, f"tool_plate_{key}", (0.021, 0.021, 0.003), (0, 0, 0.003))
        E.SubElement(
            tool,
            "site",
            name=f"tip_{key}",
            pos=nums(np.array(spec["tcp_mm"]) / 1000),
            size=".003",
        )
        if key == "gripper":
            frame = E.SubElement(
                tool, "body", name="gripper_frame", pos="0 0 .006", quat="0 1 0 0"
            )
            original = (
                E.parse(ROOT / "Simulation/SO101/so101_new_calib.xml")
                .getroot()
                .find(".//body[@name='gripper']")
            )
            for g in original.findall("geom"):
                if g.get("mesh") == "sts3215_03a_v1" and g.get("class") == "visual":
                    frame.append(copy.deepcopy(g))
            frame.append(
                copy.deepcopy(original.find("body[@name='moving_jaw_so101_v1']"))
            )
            box(tool, "fixed_finger_proxy", (0.007, 0.014, 0.030), (-0.018, 0, 0.081))
            E.SubElement(
                root.find("actuator"),
                "position",
                name="tool_gripper",
                joint="gripper",
                kp="30",
                kv=".2",
                ctrlrange="-.17453 1.74533",
                forcerange="-.35 .35",
            )
        elif key == "soft_gripper":
            fixed = E.SubElement(tendon, "fixed", name="finger_tendon")
            for i, a in enumerate(np.arange(4) * np.pi / 2):
                radial = np.array([np.cos(a), np.sin(a), 0])
                axis = np.cross(radial, [0, 0, 1])
                finger = E.SubElement(
                    tool,
                    "body",
                    name=f"finger_{i}",
                    pos=nums(radial * 0.034 + [0, 0, 0.043]),
                )
                E.SubElement(
                    finger,
                    "joint",
                    name=f"finger_{i}_prox",
                    axis=nums(axis),
                    range="0 1.1",
                    damping=".005",
                    stiffness=".04",
                    springref="0",
                    armature=".00003",
                )
                mass(finger, 0.003, (0, 0, 0.015), 0.015)
                geom(
                    finger,
                    name=f"prox_{i}",
                    type="capsule",
                    size=".005",
                    fromto="0 0 0 0 0 .030",
                    rgba=spec["colour"],
                )
                distal = E.SubElement(
                    finger, "body", name=f"finger_distal_{i}", pos="0 0 .030"
                )
                E.SubElement(
                    distal,
                    "joint",
                    name=f"finger_{i}_dist",
                    axis=nums(axis),
                    range="0 1.2",
                    damping=".003",
                    stiffness=".025",
                    springref="0",
                    armature=".00002",
                )
                mass(distal, 0.002, (0, 0, 0.012), 0.012)
                geom(
                    distal,
                    name=f"dist_{i}",
                    type="capsule",
                    size=".0045",
                    fromto="0 0 0 0 0 .025",
                    rgba=spec["colour"],
                )
                E.SubElement(fixed, "joint", joint=f"finger_{i}_prox", coef=".008")
                E.SubElement(fixed, "joint", joint=f"finger_{i}_dist", coef=".005")
            E.SubElement(
                root.find("actuator"),
                "position",
                name="tool_soft_gripper",
                tendon="finger_tendon",
                kp="1500",
                kv="1",
                ctrlrange="0 .05",
                forcerange="0 8",
            )
            visual(root, tool, "four_finger_lid", (0.3, 0.35, 0.4, 1), (0, 0, 0.046))
            visual(root, tool, "equalizer_guard", (0.3, 0.35, 0.4, 1), (0, 0, 0.049))
            visual(
                root,
                tool,
                "tendon_spool",
                (0.8, 0.8, 0.8, 1),
                (0.0202, -0.020, 0.0294),
                (0.70710678, 0.70710678, 0, 0),
            )
            geom(
                tool,
                type="mesh",
                mesh="sts3215_03a_v1",
                pos=(0.0077, -0.0001, 0.0294),
                quat=(0.70710678, 0.70710678, 0, 0),
                contype=0,
                conaffinity=0,
                group=2,
                rgba=(0.1, 0.1, 0.1, 1),
            )
            box(tool, "soft_servo_proxy", (0.023, 0.013, 0.018), (0, 0, 0.027))
        elif key == "spatula":
            box(
                tool,
                "spatula_tang",
                (0.006, 0.003, 0.018),
                (0, 0, 0.035),
                (0.7, 0.73, 0.76, 1),
            )
            geom(
                tool,
                name="spatula_blade",
                type="box",
                size=(0.025, 0.002, 0.027),
                pos=(0, 0, 0.078),
                rgba=(0.7, 0.73, 0.76, 1),
            )
        else:
            spinner = E.SubElement(tool, "body", name="stirring_shaft", pos="0 0 .05")
            E.SubElement(
                spinner,
                "joint",
                name="stirrer_spin",
                axis="0 0 1",
                limited="false",
                damping=".0001",
                armature=".00001",
            )
            mass(spinner, 0.006, (0, 0, 0.04), 0.02)
            geom(
                spinner,
                name="shaft",
                type="cylinder",
                size=(0.002, 0.035),
                pos=(0, 0, 0.035),
                rgba=(0.7, 0.73, 0.76, 1),
            )
            geom(
                spinner,
                name="stir_paddle",
                type="box",
                size=(0.018, 0.002, 0.008),
                pos=(0, 0, 0.067),
                rgba=(0.6, 0.65, 0.7, 1),
            )
            E.SubElement(
                root.find("actuator"),
                "velocity",
                name="tool_stirrer",
                joint="stirrer_spin",
                kv=".004",
                ctrlrange="0 6.283185",
                forcerange="-.008 .008",
            )
        E.SubElement(
            equality,
            "weld",
            name=f"couple_{key}",
            body1="receiver",
            body2=f"tool_{key}",
            relpose="0 0 .024 1 0 0 0",
            active=str(key == initial_tool).lower(),
            solref=".01 1",
        )
        E.SubElement(
            equality,
            "weld",
            name=f"park_{key}",
            body1=f"dock_{key}",
            body2=f"tool_{key}",
            relpose=nums(np.r_[[0, 0, p[2]], quat(r)]),
            active=str(key != initial_tool).lower(),
            solref=".01 1",
        )
