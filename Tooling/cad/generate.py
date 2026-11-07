"""Generate editable STEP + STL prototype components using actual upstream CAD.

Run uv run --group cad python Tooling/cad/generate.py. Never overwrites upstream.
"""

import hashlib
import json
import sys
from pathlib import Path

import build123d as b
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "design"))
from spec import *

C = b.Align.CENTER
N = b.Align.MIN
parts = {}


def box(w, d, h, x=0, y=0, z=0):
    return b.Pos(x, y, z) * b.Box(w, d, h, align=(C, C, N))


def cylinder(r, h, x=0, y=0, z=0):
    return b.Pos(x, y, z) * b.Cylinder(r, h, align=(C, C, N))


def bore_x(r, length, x, y, z):
    return b.Pos(x, y, z) * b.Rot(0, 90, 0) * b.Cylinder(r, length, align=(C, C, N))


def keyed(width, z):
    a = width / 2
    wire = b.Polyline((-a, -a), (a, -a), (a, a - 4), (a - 4, a), (-a, a), close=True)
    return b.Pos(0, 0, z) * b.make_face(wire)


def insert(clearance=0):
    # Tip at -9, root at 0. A single clipped corner prevents reversed insertion.
    return b.loft([keyed(21.5 + 2 * clearance, -9), keyed(23.5 + 2 * clearance, 0)])


def puck():
    p = box(42, 42, 6) + insert()
    for side in (-1, 1):
        p += box(EAR_X - 9, 14, 6, x=side * (EAR_X / 2 + 10.5))
    p -= bore_x(1.65, 36, -18, 0, -4)
    # Separate contact PCB/strip mounts on this ledge; pads are not printed.
    for x in (-10, 10):
        p -= cylinder(1.1, 10, x, -16, -1)
    return p.clean()


def add(name, shape, material="PLA/PETG fit prototype", printable=True):
    if not shape.is_valid:
        raise ValueError(f"Invalid CAD: {name}")
    if len(shape.solids()) != 1:
        raise ValueError(
            f"{name}: expected one connected solid, got {len(shape.solids())}"
        )
    folder = BUILD / "parts"
    folder.mkdir(parents=True, exist_ok=True)
    step = folder / f"{name}.step"
    stl = folder / f"{name}.stl"
    b.export_step(shape, step)
    b.export_stl(shape, stl, tolerance=0.05, angular_tolerance=0.12)
    mesh = trimesh.load_mesh(stl)
    # STEP booleans can tessellate zero-area seam triangles. Remove only those
    # degeneracies and duplicate faces; do not fill or invent missing surfaces.
    mesh.merge_vertices(digits_vertex=7)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    if not mesh.is_watertight:
        raise ValueError(f"Non-watertight STL {name}")
    mesh.export(stl)
    print_mesh = mesh.copy()
    print_mesh.apply_translation(-mesh.bounds[0])
    print_path = BUILD / "print" / f"{name}.stl"
    print_path.parent.mkdir(exist_ok=True)
    if printable:
        print_mesh.export(print_path)
    else:
        print_path.unlink(missing_ok=True)
    bounds = shape.bounding_box()
    parts[name] = {
        "step": str(step.relative_to(ROOT)),
        "stl": str(stl.relative_to(ROOT)),
        "print_stl": str(print_path.relative_to(ROOT)),
        "material": material,
        "prototype": True,
        "volume_mm3": shape.volume,
        "bounds_mm": [list(bounds.min), list(bounds.max)],
        "printable_component": printable,
        "bed_fit": bool(np.all(mesh.extents <= [170, 170, 180])),
        "solids": len(shape.solids()),
        "watertight": True,
        "sha256": hashlib.sha256(stl.read_bytes()).hexdigest(),
        "estimated_print_g": shape.volume / 1000 * 1.24 * 0.55,
    }
    print(
        name, "volume", round(shape.volume), "bed", parts[name]["bed_fit"], flush=True
    )
    return shape


def intersection_volume(a, b):
    intersection = a & b
    return intersection.volume if intersection is not None else 0.0


def main():
    # Verify the copied wrist screw pattern against actual cylindrical STEP faces.
    source = ROOT / "STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step"
    stock = b.import_step(source)
    axes = []
    for f in stock.faces().filter_by(b.GeomType.CYLINDER):
        a = f.axis_of_rotation
        if abs(f.radius - 1.6) < 1e-5 and abs(a.direction.Z) > 0.999:
            axes.append(list(a.position))
    assert len(axes) == 4
    centre = np.mean(axes, axis=0)
    for p in axes:
        assert np.allclose(
            np.abs(np.array(p)[:2] - centre[:2]), MOUNT_HALF_PITCH, atol=1e-6
        )

    receiver = box(42, 42, 7) + box(36, 36, 17, z=7)
    cavity = insert(CLEARANCE).moved(b.Pos(0, 0, 24))
    receiver -= cavity
    receiver -= cylinder(2.8, 12, z=-1)
    receiver -= cylinder(10.15, 1.1, z=-0.1)
    for x in (-MOUNT_HALF_PITCH, MOUNT_HALF_PITCH):
        for y in (-MOUNT_HALF_PITCH, MOUNT_HALF_PITCH):
            receiver -= cylinder(1.65, 10, x, y, -1)
            # Tool cavity exposes the heads after detaching the module.
            receiver -= cylinder(3.1, 19, x, y, 4)
    # Spring and sliding steel-pin housing. Outer collar needs a removable cover.
    receiver += box(24, 16, 12, x=27, y=0, z=12)
    receiver -= bore_x(1.7, 36, 6, 0, PIN_AXIS_Z)
    receiver -= bore_x(3.0, 17, 20, 0, PIN_AXIS_Z)
    receiver -= box(18, 10, 7, x=29, y=-5, z=17)
    for x in (20, 35):
        for z in (14, 22):
            receiver -= (
                b.Pos(x, -10, z)
                * b.Rot(-90, 0, 0)
                * b.Cylinder(1.0, 8, align=(C, C, N))
            )
    # Camera bracket screw pattern on permanent receiver, not on tool modules.
    for x in (-10, 10):
        receiver -= (
            b.Pos(x, 20, 12) * b.Rot(90, 0, 0) * b.Cylinder(1.65, 9, align=(C, C, N))
        )
    add("receiver", receiver)
    cover = box(23, 12, 2) - box(14, 3, 4, y=2, z=-1)
    for x in (-7.5, 7.5):
        for y in (-4, 4):
            cover -= cylinder(1.2, 4, x, y, -1)
    add("pin_housing_cover", cover)
    collar = cylinder(2.7, 3) - cylinder(1.45, 5, z=-1)
    collar += box(8, 3, 3, x=0, y=4)
    add("pin_collar", collar)
    plug = add("test_plug", puck())
    inspection = BUILD / "inspection"
    inspection.mkdir(exist_ok=True)
    for name, part in (("receiver", receiver), ("test_plug", plug)):
        # Keep the far half and its actual cross-section, including the pin bore.
        cut = part & box(160, 60, 80, y=-30, z=-30)
        assert cut.is_valid
        b.export_stl(
            cut,
            inspection / f"{name}_section.stl",
            tolerance=0.012,
            angular_tolerance=0.08,
        )

    def pin_at(travel):
        return bore_x(PIN_DIAMETER / 2, PIN_LENGTH, PIN_TIP_X + travel, 0, PIN_AXIS_Z)

    path = (0, 1, 2, 4, 6, 8, 9, 12, 24)
    seated = b.Pos(0, 0, RECEIVER_HEIGHT) * plug
    stroke = [
        intersection_volume(pin_at(s), receiver)
        + intersection_volume(pin_at(s), seated)
        for s in np.linspace(0, PIN_TRAVEL, 8)
    ]
    released_path = [
        intersection_volume(pin_at(PIN_TRAVEL), b.Pos(0, 0, RECEIVER_HEIGHT + g) * plug)
        for g in path
    ]
    locked_pull = intersection_volume(
        pin_at(0), b.Pos(0, 0, RECEIVER_HEIGHT + 1) * plug
    )
    if max(stroke + released_path) > 1e-5 or locked_pull <= 1e-5:
        raise ValueError(
            f"Pin envelope check failed: {stroke=}, {released_path=}, {locked_pull=}"
        )
    pin_audit = {
        "pin_clearance_at_8_stroke_samples_max_mm3": max(stroke),
        "released_pin_key_interference_at_9_approach_samples_max_mm3": max(
            released_path
        ),
        "locked_pin_intersection_on_1mm_pull_mm3": locked_pull,
        "evidence": "Rigid, nominal STEP geometry only; no strength, preload, or repeatability claims.",
    }
    (inspection / "pin_geometry.json").write_text(
        json.dumps(pin_audit, indent=2) + "\n"
    )
    overlap = intersection_volume(receiver, seated)
    if overlap > 1e-5:
        raise ValueError(f"Receiver/module CAD interference: {overlap} mm3")

    # Complete removable gripper body; preserves upstream servo pocket/jaw geometry.
    stock = stock.moved(b.Pos(0, -centre[1], 6 - 0.949706))
    add("gripper_body", puck() + stock)
    jaw = b.import_step(ROOT / "STEP/SO101/Follower_Specific/Moving_Jaw_SO101.step")
    # Some STEP versions contain disconnected modeled support; export each solid.
    for i, solid in enumerate(jaw.solids()):
        add(f"gripper_jaw_{i + 1}", solid)

    # Four-finger palm: open STS3215 pocket, cap and tendon passages.
    palm = puck() + box(55, 39, 40, z=6)
    palm -= box(46.0, 25.5, 40, z=9)
    for x in (-24.8, 24.8):
        for y in (-15, 15):
            palm -= cylinder(1.0, 50, x, y, -1)
    palm -= (
        b.Pos(20.2, -25, 29.4) * b.Rot(-90, 0, 0) * b.Cylinder(5.5, 50, align=(C, C, N))
    )
    for x, y in ((29, 0), (-29, 0), (0, 24), (0, -24)):
        palm += box(12, 12, 8, x, y, 38)
        palm -= cylinder(1.65, 12, x, y, 36)
        palm -= cylinder(1.1, 50, x * 0.7, y * 0.7, 1)
    add("four_finger_palm", palm)
    lid = box(55, 39, 3)
    for x in (-24.8, 24.8):
        for y in (-15, 15):
            lid -= cylinder(1.2, 5, x, y, -1)
    add("four_finger_lid", lid)
    guard = box(55, 39, 2, z=10)
    for x in (-24.8, 24.8):
        for y in (-15, 15):
            guard += box(5, 6, 10, x, y)
            guard -= cylinder(1.2, 14, x, y, -1)
    add("equalizer_guard", guard)
    # Flat TPU finger, two thin flexure regions, tendon eyelets. Bend response
    # must be measured; MuJoCo uses hinged proxies rather than soft material FEM.
    finger = box(12, 14, 5) + box(12, 5, 27, y=4.5, z=9) + box(12, 5, 25, y=1.5, z=40)
    finger += box(12, 1.5, 5, y=4.5, z=4.5) + box(12, 1.5, 5.5, y=3, z=35.5)
    finger -= cylinder(1.65, 10, 0, 0, -1)
    for z in (17, 30, 48, 59):
        finger -= (
            b.Pos(0, -6, z) * b.Rot(-90, 0, 0) * b.Cylinder(1.1, 16, align=(C, C, N))
        )
    add("soft_finger_print_four", finger, "TPU 95A flexure prototype")
    # Pulley-tree differential: one central and two secondary floating bars.
    for name, length in (
        ("tendon_equalizer_primary", 26),
        ("tendon_equalizer_secondary", 18),
    ):
        bar = box(length, 7, 3)
        for x in (-length / 2 + 3, 0, length / 2 - 3):
            bar -= cylinder(0.9, 5, x, 0, -1)
        add(name, bar)
    spool = cylinder(7, 7) + cylinder(9, 1) + cylinder(9, 1, z=6)
    spool -= cylinder(2.75, 10, z=-1)
    # Horn adapter is fastened to the supplied horn; not an unverified printed spline.
    for x in (-4.95, 4.95):
        for y in (-4.95, 4.95):
            spool -= cylinder(1.6, 10, x, y, -1)
    add("tendon_spool", spool)

    # Dedicated commercial-blade holder, two-piece clamp for a rectangular tang.
    spatula = puck() + box(26, 17, 43, z=6)
    spatula -= box(12.5, 6.5, 35, z=17)
    spatula -= box(28, 5.25, 33, y=-5.875, z=17)
    for x in (-9, 9):
        for z in (26, 40):
            spatula -= (
                b.Pos(x, -12, z)
                * b.Rot(-90, 0, 0)
                * b.Cylinder(1.65, 24, align=(C, C, N))
            )
    spatula -= (
        b.Pos(0, -12, 23) * b.Rot(-90, 0, 0) * b.Cylinder(1.65, 24, align=(C, C, N))
    )
    add("spatula_holder", spatula)
    jaw_clamp = box(26, 5, 30)
    for x in (-9, 9):
        for z in (8, 22):
            jaw_clamp -= (
                b.Pos(x, -4, z)
                * b.Rot(-90, 0, 0)
                * b.Cylinder(1.65, 12, align=(C, C, N))
            )
    jaw_clamp -= (
        b.Pos(0, -4, 5) * b.Rot(-90, 0, 0) * b.Cylinder(1.65, 12, align=(C, C, N))
    )
    add("spatula_clamp", jaw_clamp)

    # N20 motor housing, lid, shaft support. No printed blade is supplied for food.
    stirrer = puck() + box(29, 25, 55, z=6)
    stirrer -= box(12.5, 10.5, 28, z=10)
    stirrer -= cylinder(2.1, 65, z=-1)
    stirrer -= cylinder(5.3, 18, z=38)
    stirrer -= cylinder(5.1, 4.2, z=56.8)  # 4x10x4 bearing seat with print allowance
    for x in (-11, 11):
        stirrer -= cylinder(1.0, 12, x, 0, 50)
        for z in (15, 48):
            stirrer -= (
                b.Pos(x, -15, z)
                * b.Rot(-90, 0, 0)
                * b.Cylinder(1.2, 30, align=(C, C, N))
            )
    add("stirrer_housing", stirrer, printable=False)
    front = stirrer & box(100, 50, 55, y=-25, z=6)
    add("stirrer_front_shell", front)
    add("stirrer_back_shell", stirrer - front)
    motor_lid = box(29, 25, 3) - cylinder(2.1, 5, z=-1)
    for x in (-11, 11):
        motor_lid -= cylinder(1.2, 5, x, 0, -1)
    add("stirrer_lid", motor_lid)
    # Bench-only coupling coupon; actual metal shaft coupling is required.
    shaft = cylinder(5, 16) - cylinder(1.6, 8, z=-0.1) - cylinder(2.1, 8.2, z=8)
    for z in (4, 12):
        shaft -= bore_x(1.25, 12, -6, 0, z)
    add("shaft_coupling_fit_coupon", shaft)

    # Camera structure remains on arm-side receiver. Slot centres allow 24–30mm.
    bracket = (
        box(30, 12, 11, y=22, z=7)
        + box(30, 5, 18, y=26, z=7)
        + box(36, 36, 3, y=44, z=24)
    )
    bracket -= box(23, 23, 5, y=44, z=23)
    for x in (-10, 10):
        bracket -= (
            b.Pos(x, 24, 12) * b.Rot(90, 0, 0) * b.Cylinder(1.65, 10, align=(C, C, N))
        )
    for x in (-14, 14):
        for y in (30, 58):
            bracket -= cylinder(1.2, 6, x, y, 22)
    add("camera_bracket_32mm", bracket)

    # Fit coupons: three clearances to select with actual A1 mini/material.
    for clearance in (0.15, 0.25, 0.35):
        coupon = box(32, 32, 12) - insert(clearance).moved(b.Pos(0, 0, 12))
        coupon -= bore_x(1.65, 36, -18, 0, 8)
        add(f"socket_coupon_{int(clearance * 100):02d}", coupon)

    dock_manifest = {}
    dock_solids = {}
    for key, (p, r) in dock_poses().items():
        # Independent pedestal per station. World-up printing, unique yaw.
        # Fork ledges support the module ears. Rack actuator mounts are on base.
        centre_xy = p[:2] * 1000
        frame = b.Location(
            tuple(p * 1000 - np.r_[centre_xy, 0]),
            tuple(Rotation.from_matrix(r).as_euler("XYZ", degrees=True)),
        )
        # Outboard uprights leave an open throat for the entire head, including
        # the blade and fingers. The slotted guides are beyond the tool ears.
        yaw = np.degrees(np.arctan2(r[1, 0], r[0, 0]))
        yaw_frame = b.Rot(0, 0, yaw)
        dock = box(164, 39, 6)
        for x in (-75, 75):
            for y in (-13, 13):
                dock -= cylinder(2.2, 8, x, y, -1)
        dock = yaw_frame * dock
        for side in (-1, 1):
            post_x = side * 74
            top = p * 1000 + r @ [post_x, 0, RACK_FRONT + 3]
            w = top - np.r_[centre_xy, 0]
            post = box(12, 18, w[2] - 6.0 + 0.2, z=5.8)
            for z in (110, 130):
                post -= bore_x(1.7, 24, -12, 0, z)
            dock += b.Pos(w[0], w[1], 0) * yaw_frame * post
            shelf = box(30, 22, 6, side * 63, 0, RACK_FRONT)
            sidewall = box(12, 25, RACK_FRONT + 13, side * 70, -7, -8)
            sidewall -= box(18, 25, 4.8, side * 64, -7, -5.8)
            # A cap reconnects the outer guide edge above the runner slot.
            sidewall += box(4, 25, RACK_FRONT + 13, side * 74, -7, -8)
            dock += frame * (shelf + sidewall)
        # The servo bracket and linkage are separate bench integration work.
        pad = box(30, 35, 7, y=10, z=5.8)
        for x in (-8, 8):
            pad -= cylinder(1.65, 16, x, 10, -1)
        dock += yaw_frame * pad
        # Segment the tall pedestal for the A1 mini.
        add(f"dock_{key}", dock, printable=False)
        dock_solids[key] = dock, frame
        lower = dock & box(240, 240, 120)
        upper = dock & box(240, 240, 200, z=120)
        for label, component in (("lower", lower), ("upper", upper)):
            for i, solid in enumerate(component.solids()):
                add(f"dock_{key}_{label}_{i + 1}", solid)
        dock_manifest[key] = {
            "position_m": p.tolist(),
            "rotation": r.tolist(),
            "pedestal_origin_m": [float(p[0]), float(p[1]), 0],
            "dock_q_rad": DOCK_Q[KEYS.index(key)].tolist(),
        }
    release_fork = box(22, 8, 4) - box(6, 10, 5, x=0, z=-0.5)
    release_fork += box(10, 20, 4, y=-10)
    release_fork -= (
        cylinder(1.7, 6, 0, -17, -1)
        + cylinder(1.7, 6, 0, -13, -1)
        + box(3.4, 4, 6, y=-15, z=-1)
    )
    add("dock_release_fork", release_fork)
    gate = box(2 * EAR_X + 12, 8, 2.7, y=-28, z=-4.7)
    for side in (-1, 1):
        gate += box(18, 38, 2.7, side * 58, -11, -4.7)
    gate -= cylinder(1.65, 5, 0, -28, -6)
    add("dock_retaining_gate", gate)
    # Independent STEP solid check: catch nominal overlap (including complete
    # containment) even when no triangle surfaces cross in a sampled frame.
    for key, name in zip(
        KEYS, ("gripper_body", "four_finger_palm", "spatula_holder", "stirrer_housing")
    ):
        dock, frame = dock_solids[key]
        tool = frame * b.import_step(BUILD / "parts" / f"{name}.step")
        intersections = {"module_dock": intersection_volume(tool, dock)}
        for opening in (0, RACK_GATE_TRAVEL / 2, RACK_GATE_TRAVEL):
            gate_at = frame * b.Pos(0, -opening, 0) * gate
            intersections[f"gate_dock_at_{opening:g}_mm_open"] = intersection_volume(
                gate_at, dock
            )
            intersections[f"gate_module_at_{opening:g}_mm_open"] = (
                gate_at & tool
            ).volume
        if max(intersections.values()) > 1e-5:
            raise ValueError(f"Nominal rack interference at {key}: {intersections}")
        dock_manifest[key]["nominal_intersection_mm3"] = intersections
    cradle = b.import_step(ROOT / "STEP/SO101/Motor_holder_SO101_Wrist.step")
    add("dock_servo_cradle", cradle)
    crank = cylinder(16, 3) - cylinder(2.8, 5, z=-1)
    for x in (-MOUNT_HALF_PITCH, MOUNT_HALF_PITCH):
        for y in (-MOUNT_HALF_PITCH, MOUNT_HALF_PITCH):
            crank -= cylinder(1.65, 5, x, y, -1)
    crank -= cylinder(1.65, 5, 0, 12, -1)
    add("dock_servo_crank", crank)
    splice = box(20, 26, 40) - box(12.5, 18.5, 42, z=-1)
    for z in (10, 30):
        splice -= bore_x(1.7, 24, -12, 0, z)
    add("dock_post_splice_print_eight", splice)
    contact = box(25, 7, 3)
    for i in range(8):
        contact -= cylinder(0.95, 5, (i - 3.5) * 2.54, 0, -1)
    for x in (-10, 10):
        contact -= cylinder(1.1, 5, x, 2.3, -1)
    add("contact_carrier_fit_coupon", contact)
    manifest = {
        "version": VERSION,
        "units": "mm",
        "status": "FIT_PROTOTYPE_NOT_LOAD_QUALIFIED",
        "mount_source": str(source.relative_to(ROOT)),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "mount_holes_source_mm": axes,
        "mount_centre_source_mm": centre.tolist(),
        "hole_diameter_source_mm": 3.2,
        "half_pitch_mm": MOUNT_HALF_PITCH,
        "receiver_height_mm": RECEIVER_HEIGHT,
        "pin_diameter_mm": PIN_DIAMETER,
        "pin_travel_mm": PIN_TRAVEL,
        "clearance_per_side_mm": CLEARANCE,
        "receiver_insert_interference_mm3": overlap,
        "release_crank_radius_mm": 12,
        "release_angle_degrees": 17,
        "release_calculated_stroke_mm": float(24 * np.sin(np.deg2rad(17))),
        "rack_revision": 2,
        "rack_gate_travel_mm": RACK_GATE_TRAVEL,
        "rack_ledge_face_mm": RACK_FRONT,
        "docks": dock_manifest,
        "parts": parts,
    }
    BUILD.mkdir(parents=True, exist_ok=True)
    (BUILD / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Exported {len(parts)} valid solid STEP/STL pairs", flush=True)


if __name__ == "__main__":
    main()
