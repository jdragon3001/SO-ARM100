"""Physics, tool exchange, CAD contracts and electrical protocol acceptance."""

import dataclasses, json, sys
from pathlib import Path
import mujoco
import numpy as np
import pytest
import trimesh
from engine import (
    Config,
    HOME,
    JOINTS,
    ROOT,
    Simulation,
    TOOLS,
    KEYS,
    DOCK_Q,
    leader_model,
    LEADER_JOINTS,
)
from interlocks import Sensors, may_unlock, may_depart, may_energize, identify_resistor

sys.path.insert(0, str(ROOT / "Tooling/electronics"))
from protocol import encode, decode, Endpoint


def finish(sim, limit=60):
    end = sim.data.time + limit
    while sim.busy and sim.data.time < end:
        sim.step(20)
    assert sim.phase == "READY", (sim.phase, sim.error, sim.contacts())


@pytest.mark.parametrize("tool", TOOLS)
@pytest.mark.parametrize("camera", [False, True])
def test_all_module_models_settle(tool, camera):
    s = Simulation(Config(tool=tool, camera=camera))
    s.step(1000)
    assert np.max(abs(s.q - HOME)) < 0.01
    assert not s.contacts()
    assert np.max(abs(s.data.actuator_force[:5])) <= s.config.torque_nm + 1e-8
    assert len(s.q) == 5 and s.model.nu == 8


def test_four_actual_free_modules_and_one_arm_coupling():
    s = Simulation()
    assert all(
        s.model.joint("free_" + k).type[0] == mujoco.mjtJoint.mjJNT_FREE for k in KEYS
    )
    assert sum(s.data.eq_active[s.model.equality("couple_" + k).id] for k in KEYS) == 1
    assert sum(s.data.eq_active[s.model.equality("park_" + k).id] for k in KEYS) == 3
    assert s.model.body("receiver").id >= 0
    assert s.model.body("tool_gripper").id != s.model.body("receiver").id


@pytest.mark.parametrize(
    "source,destination", [(a, b) for a in TOOLS for b in TOOLS if a != b]
)
def test_every_directed_tool_change(source, destination):
    s = Simulation(Config(tool=source))
    s.change_tool(destination)
    finish(s)
    assert s.attached == KEYS[TOOLS.index(destination)]
    assert not s.contacts()


def test_full_round_trip_all_four_tools_with_retention_and_power():
    s = Simulation()
    for tool in TOOLS[1:] + TOOLS[:1]:
        s.change_tool(tool)
        while s.busy:
            s.step(20)
            assert s.data.time < 130
            for key in KEYS:
                assert (
                    s.data.eq_active[s.model.equality("couple_" + key).id]
                    or s.data.eq_active[s.model.equality("park_" + key).id]
                )
            if not s.locked:
                assert not s.power_enabled and s.rail_v < 0.5
                if s.attached:
                    assert s.data.eq_active[s.model.equality("park_" + s.attached).id]
        assert s.phase == "READY", s.error
        assert s.attached == KEYS[TOOLS.index(tool)]
        assert s.power_enabled and s.locked and not s.contacts()
    phases = [x["phase"] for x in s.change_log]
    assert (
        phases.index("VERIFY_SUPPORT")
        < phases.index("UNLOCK")
        < phases.index("PARK_OLD")
    )
    assert phases.index("VERIFY_LOCK") < phases.index("WITHDRAW_NEW")


@pytest.mark.parametrize(
    "fault,phase",
    [
        ("unsupported", "VERIFY_SUPPORT"),
        ("rail_stuck_on", "DISCHARGE"),
        ("wrong_id", "IDENTIFY"),
        ("lock_failed", "VERIFY_LOCK"),
    ],
)
def test_missing_feedback_halts_instead_of_advancing(fault, phase):
    s = Simulation()
    s.change_tool("Spatula")
    while s.busy and s.phase != phase:
        s.step(1)
    assert s.phase == phase
    s.fault_injection = fault
    for _ in range(210):
        s.step(50)
    assert s.phase == "FAULT" and not s.power_enabled
    assert all(
        s.data.eq_active[s.model.equality("couple_" + k).id]
        or s.data.eq_active[s.model.equality("park_" + k).id]
        for k in KEYS
    )


def test_power_off_removes_tool_actuator_force():
    s = Simulation(Config(tool="Powered stirrer"))
    s.power_enabled = True
    s.set_tool_control(0.6)
    s.step(500)
    assert s.data.joint("stirrer_spin").qvel[0] > 1
    s.power_enabled = False
    s.step(1)
    assert s.data.actuator_force[s.model.actuator("tool_stirrer").id] == 0


def test_four_fingers_respond_to_one_tendon_command():
    s = Simulation(Config(tool="Four-finger gripper"))
    s.power_enabled = True
    s.set_tool_control(0.4)
    s.step(1000)
    angles = s.state()["tool_observation"]["finger_rad"]
    assert all(a > 0.1 for a in angles)
    assert s.model.ntendon == 1


def test_docks_have_full_orientation_alignment_not_only_position():
    s = Simulation()
    for key, q in zip(KEYS, DOCK_Q):
        p, r = s.poses[key]
        fit = s.solve_receiver(p - r @ [0, 0, 0.024], r, q)
        assert fit["error_m"] < 1e-5 and fit["angle_rad"] < 1e-4
        assert np.all(s.moves[key]["pre"] >= s.limits[:, 0]) and np.all(
            s.moves[key]["pre"] <= s.limits[:, 1]
        )


def test_arm_commands_and_ik_are_bounded_and_nonmutating():
    s = Simulation()
    before = s.data.qpos.copy()
    s.command([100] * 5)
    np.testing.assert_allclose(s.target, s.limits[:, 1])
    np.testing.assert_array_equal(s.data.qpos, before)
    assert s.ik(s.tip + [-0.02, 0, 0.01])["ok"]
    assert not s.ik([2, 0, 0.2])["ok"]
    assert not s.ik([0.25, 0, -0.04])["ok"]
    with pytest.raises(ValueError):
        s.command([np.nan] * 5)


@pytest.mark.parametrize(
    "field,value",
    [("tool", "x"), ("torque_nm", np.nan), ("payload_g", -1), ("camera_fovy", 180)],
)
def test_invalid_setup(field, value):
    with pytest.raises(ValueError):
        Config(**{field: value})


def test_loaded_tool_change_requires_unloading_test_mass():
    s = Simulation(Config(payload_g=100))
    with pytest.raises(ValueError):
        s.change_tool("Spatula")


def test_camera_state_and_rgb_frames_are_valid():
    s = Simulation()
    s.step(100)
    before = s.data.cam_xpos.copy()
    tip = s.tip.copy()
    mujoco.mj_forward(s.model, s.data)
    np.testing.assert_allclose(before, s.data.cam_xpos, atol=1e-12)
    np.testing.assert_allclose(tip, s.tip, atol=1e-12)
    renderer = mujoco.Renderer(s.model, height=240, width=320)
    try:
        for name in ("wrist_rgb", "overhead"):
            renderer.update_scene(s.data, camera=name)
            im = renderer.render()
            assert im.shape == (240, 320, 3) and im.std() > 10
    finally:
        renderer.close()


def test_snapshot_contains_real_tool_state(tmp_path):
    s = Simulation()
    p = tmp_path / "state.json"
    s.save(p)
    v = json.loads(p.read_text())
    assert len(v["actual_rad"]) == 5 and "finger_rad" in v["tool_observation"]
    assert "synthetic_sensors" in v


def test_leader_retains_six_inputs():
    m = leader_model()
    assert all(m.joint(n).id >= 0 for n in LEADER_JOINTS)


@pytest.mark.parametrize(
    "field,value",
    [
        ("dock_supported", False),
        ("seated", False),
        ("tool_stopped", False),
        ("rail_v", 0.5),
        ("rail_v", np.nan),
        ("rail_v", -1),
        ("estop", True),
    ],
)
def test_unlock_requires_all_independent_permissions(field, value):
    s = Sensors(dock_supported=True, seated=True, tool_stopped=True, rail_v=0.1)
    assert may_unlock(s)
    assert not may_unlock(dataclasses.replace(s, **{field: value}))


def test_identity_and_lock_are_required_to_depart_and_power():
    s = Sensors(locked=True, tool_present=True, tool_id="gripper")
    assert may_depart(s, "gripper") and may_energize(s, "gripper")
    assert not may_depart(s, "spatula")
    assert not may_depart(dataclasses.replace(s, unlocked=True), "gripper")
    assert identify_resistor(1000) == "gripper" and identify_resistor(4700) == "spatula"
    assert identify_resistor(0) is None and identify_resistor(float("inf")) is None


def test_protocol_crc_replay_bounds_and_watchdog():
    e = Endpoint("stirrer")

    def packet(n, op, **kwargs):
        return encode(n, dict(tool="stirrer", op=op, **kwargs))

    assert e.receive(packet(1, "arm", locked=True, power_good=True), 0)["ok"]
    assert e.receive(packet(2, "set", value=30), 0.01)["output"] == 30
    e.tick(0.3)
    assert not e.enabled and e.output == 0
    assert not e.receive(packet(2, "set", value=30), 0.31)["ok"]
    assert e.receive(packet(3, "arm", locked=True, power_good=True), 0.32)["ok"]
    assert not e.receive(packet(4, "set", value=100), 0.33)["ok"]
    corrupt = bytearray(packet(5, "status"))
    corrupt[-1] ^= 1
    with pytest.raises(ValueError):
        decode(corrupt)
    assert not e.receive(corrupt, 0.34)["ok"]


def test_generated_cad_and_a1_print_files():
    manifest = json.loads((ROOT / "Build/Tooling/manifest.json").read_text())
    assert manifest["receiver_insert_interference_mm3"] < 1e-5
    assert manifest["release_calculated_stroke_mm"] >= 7
    for name, p in manifest["parts"].items():
        assert p["watertight"] and p["solids"] == 1 and (ROOT / p["step"]).exists()
        if p["printable_component"]:
            m = trimesh.load_mesh(ROOT / p["print_stl"])
            assert m.is_watertight and np.all(
                m.extents <= [170.001, 170.001, 180.001]
            ), name
            assert m.bounds[0, 2] >= -0.001


def test_original_print_repack_remains_complete():
    manifest = json.loads((ROOT / "Build/A1mini/manifest.json").read_text())
    for role in manifest["roles"].values():
        source = trimesh.load_mesh(ROOT / role["source"])
        total = sum(
            len(trimesh.load_mesh(ROOT / p["file"]).faces) for p in role["plates"]
        )
        assert total == len(source.faces)
