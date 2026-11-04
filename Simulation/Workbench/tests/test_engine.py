import dataclasses
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
import trimesh

from engine import Config, HOME, JOINTS, ROOT, Simulation, TOOLS, leader_model, map_to_leader
from prepare_prints import fits


@pytest.mark.parametrize("tool", TOOLS)
@pytest.mark.parametrize("camera", [True, False])
def test_dynamic_models_settle_without_contacts_or_warnings(tool, camera):
    sim = Simulation(Config(tool=tool, camera=camera))
    sim.step(1500)
    assert np.max(np.abs(sim.q-HOME)) < .025
    assert not sim.contacts()
    assert np.max(np.abs(sim.data.actuator_force)) <= sim.config.torque_nm+1e-8
    assert sim.model.nq == 13  # 6 hinges plus the independent free cube


def test_payload_and_external_force_change_required_torque():
    torques = []
    for config in (Config(), Config(payload_g=250), Config(tool_force_n=2)):
        sim = Simulation(config)
        sim.step(1500)
        torques.append(abs(sim.data.actuator_force[2]))
    assert torques[1] > torques[0] + .4
    assert torques[2] > torques[0] + .4


def test_low_torque_saturates_and_lags_instead_of_teleporting():
    sim = Simulation(Config(torque_nm=.2, payload_g=250))
    sim.step(1000)
    assert abs(sim.q[2]-HOME[2]) > .05
    assert np.max(np.abs(sim.data.actuator_force)) <= .200001


def test_joint_commands_are_bounded_and_rate_limited():
    sim = Simulation()
    old_q, old_ctrl = sim.q, sim.data.ctrl.copy()
    sim.command([100]*6)
    np.testing.assert_allclose(sim.target, sim.limits[:,1])
    np.testing.assert_array_equal(sim.q, old_q)
    sim.step(1)
    assert np.max(np.abs(sim.data.ctrl-old_ctrl)) <= sim.rate_limit*.002+1e-8
    with pytest.raises(ValueError):
        sim.command([np.nan]*6)


def test_position_ik_reaches_and_rejects_without_mutating_physics():
    sim = Simulation()
    target = sim.tip + [-.03,.02,.01]
    original_q = sim.data.qpos.copy()
    result = sim.ik(target)
    assert result["ok"], result
    assert result["error_mm"] < 1
    assert not sim.ik([2.,0.,.2])["ok"]
    np.testing.assert_array_equal(sim.data.qpos, original_q)
    with pytest.raises(ValueError):
        sim.ik([0,np.inf,0])


def test_below_bench_goal_is_rejected():
    sim = Simulation()
    result = sim.ik([.27, 0, -.04])
    assert not result["ok"]


def test_leader_has_real_handle_and_normalized_trigger_mapping():
    sim = Simulation()
    leader = leader_model()
    limits = np.array([leader.joint(n).range for n in JOINTS])
    assert leader.mesh("Handle_SO101").id >= 0
    assert leader.mesh("Trigger_SO101").id >= 0
    q = HOME.copy()
    for side in (0,1):
        q[5] = sim.limits[5,side]
        mapped = map_to_leader(q, sim.limits, limits)
        np.testing.assert_allclose(mapped[:5], q[:5], atol=1e-5)
        assert mapped[5] == pytest.approx(limits[5,side])


@pytest.mark.parametrize("kwargs", [{"payload_g":-1}, {"torque_nm":np.nan},
    {"tool":"unknown"}, {"camera_fovy":180}, {"tool_force_n":-1}])
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_rendered_cameras_are_nonempty_and_calibrated_to_requested_fov():
    sim = Simulation(Config(camera_fovy=80))
    sim.step(100)
    assert sim.model.camera("wrist_rgb").fovy[0] == 80
    renderer = mujoco.Renderer(sim.model, height=240, width=320)
    try:
        for camera in ("wrist_rgb", "overhead"):
            renderer.update_scene(sim.data, camera=camera)
            image = renderer.render()
            assert image.shape == (240,320,3)
            assert image.std() > 10
    finally:
        renderer.close()


def test_pose_snapshot_is_serializable(tmp_path):
    sim = Simulation()
    path = tmp_path / "snapshot.json"
    sim.save(path)
    data = json.loads(path.read_text())
    assert len(data["actual_rad"]) == 6
    assert data["config"] == dataclasses.asdict(sim.config)


def test_camera_and_tcp_kinematics_match_recorded_joint_instant():
    sim = Simulation()
    sim.command(HOME + [.3,.1,0,0,0,0])
    sim.step(30)
    tip = sim.tip
    cameras = sim.data.cam_xpos.copy()
    mujoco.mj_forward(sim.model, sim.data)
    np.testing.assert_allclose(tip, sim.tip, atol=1e-12, rtol=0)
    np.testing.assert_allclose(cameras, sim.data.cam_xpos, atol=1e-12, rtol=0)


def test_repacked_prints_preserve_all_solids_and_fit_bed():
    manifest = json.loads((ROOT / "Build/A1mini/manifest.json").read_text())
    for role in manifest["roles"].values():
        source = trimesh.load_mesh(ROOT / role["source"])
        total_faces = 0
        total_volume = 0.
        ids = []
        for plate in role["plates"]:
            mesh = trimesh.load_mesh(ROOT / plate["file"])
            assert np.all(mesh.extents <= [170.001,170.001,180])
            assert mesh.bounds[0,2] >= -.001
            placed = []
            for solid in plate["solids"]:
                rect = solid["footprint_mm"]
                assert fits(rect, placed)
                placed.append(rect)
                ids.append(solid["source_solid"])
            total_faces += len(mesh.faces)
            total_volume += abs(mesh.volume)
        assert sorted(ids) == list(range(1,role["solid_count"]+1))
        assert total_faces == len(source.faces)
        assert total_volume == pytest.approx(abs(source.volume), rel=1e-5)
