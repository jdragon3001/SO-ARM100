"""Local browser workbench. Run with: uv run python Simulation/Workbench/app.py"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import dataclasses
import json
from pathlib import Path
import queue
import time

import mujoco
import numpy as np
from PIL import Image
import trimesh
import viser

from engine import (
    Config,
    HOME,
    JOINTS,
    LEADER_JOINTS,
    MODULES,
    KEYS,
    MODEL_DIR,
    ROOT,
    Simulation,
    TOOLS,
    leader_model,
    map_to_leader,
)

RUNS = Path(__file__).parent / "runs"
FOLLOWER_OFFSET = np.array([0.05, 0.12, 0.0])
LEADER_OFFSET = np.array([-0.5, -0.30, 0.0])


def quaternion(matrix):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(matrix).ravel())
    return q


class RobotVisual:
    def __init__(self, server, model, prefix, offset, leader=False):
        self.model, self.offset, self.items = model, offset, []
        for i in range(model.ngeom):
            kind = model.geom_type[i]
            if kind == mujoco.mjtGeom.mjGEOM_PLANE:
                continue
            collision = bool(model.geom_contype[i] or model.geom_conaffinity[i])
            # URDF separates its visual and collision copies in groups 1 and 0.
            if leader and collision:
                continue
            if (
                not leader
                and kind == mujoco.mjtGeom.mjGEOM_MESH
                and model.geom_group[i] == 3
            ):
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if (
                name == "payload_marker"
                and not model.body_mass[model.body("test_payload").id] > 1e-5
            ):
                continue
            rgba = model.geom_rgba[i].copy()
            mat = model.geom_matid[i]
            if mat >= 0:
                rgba = model.mat_rgba[mat].copy()
            if leader and rgba[0] > 0.6 and rgba[1] > 0.5:
                rgba[:3] = [0.13, 0.64, 0.68]
            color = tuple((rgba[:3] * 255).astype(int))
            proxy = not leader and model.geom_group[i] == 3
            kwargs = dict(
                color=(235, 90, 80) if proxy else color,
                opacity=0.2 if proxy else float(rgba[3]),
                visible=not proxy,
            )
            n = f"/{prefix}/geom_{i}"
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                mid = model.geom_dataid[i]
                va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
                fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
                handle = server.scene.add_mesh_simple(
                    n,
                    model.mesh_vert[va : va + vn],
                    model.mesh_face[fa : fa + fn],
                    **kwargs,
                )
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                handle = server.scene.add_box(
                    n, dimensions=model.geom_size[i] * 2, **kwargs
                )
            elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                handle = server.scene.add_icosphere(
                    n, radius=float(model.geom_size[i, 0]), **kwargs
                )
            elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                mesh = trimesh.creation.capsule(
                    radius=model.geom_size[i, 0], height=2 * model.geom_size[i, 1]
                )
                handle = server.scene.add_mesh_simple(
                    n, mesh.vertices, mesh.faces, **kwargs
                )
            elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
                mesh = trimesh.creation.cylinder(
                    radius=model.geom_size[i, 0], height=2 * model.geom_size[i, 1]
                )
                handle = server.scene.add_mesh_simple(
                    n, mesh.vertices, mesh.faces, **kwargs
                )
            else:
                continue
            self.items.append((i, handle, proxy))

    def update(self, data, show_proxy=False):
        for i, handle, proxy in self.items:
            handle.position = data.geom_xpos[i] + self.offset
            handle.wxyz = quaternion(data.geom_xmat[i])
            if proxy:
                handle.visible = show_proxy

    def remove(self):
        for _, handle, _ in self.items:
            handle.remove()


class Workbench:
    def __init__(self, port=8080):
        self.server = viser.ViserServer(
            host="127.0.0.1", port=port, label="SO-101 Workbench"
        )
        self.server.gui.configure_theme(
            control_width="large",
            dark_mode=True,
            brand_color=(34, 180, 181),
            show_share_button=False,
        )
        self.server.gui.main_panel.dock_right()
        self.server.scene.set_up_direction("+z")
        self.server.scene.add_grid(
            "/bench",
            width=1.6,
            height=1.15,
            cell_size=0.05,
            section_size=0.25,
            position=(0.1, -0.05, -0.004),
            plane_color=(25, 33, 44),
            plane_opacity=1.0,
            cell_color=(55, 68, 82),
            section_color=(85, 105, 120),
        )
        self.server.scene.add_label(
            "/leader_label", "LEADER · move by hand", position=(-0.46, -0.3, 0.36)
        )
        self.server.scene.add_label(
            "/follower_label", "FOLLOWER · camera + tool", position=(0.12, 0.12, 0.36)
        )
        self.sim = Simulation()
        self.leader = leader_model()
        self.leader_data = mujoco.MjData(self.leader)
        self.leader_ids = [self.leader.joint(n).qposadr[0] for n in LEADER_JOINTS]
        self.leader_limits = np.array(
            [self.leader.joint(n).range for n in LEADER_JOINTS]
        )
        self.lv = RobotVisual(
            self.server, self.leader, "leader", LEADER_OFFSET, leader=True
        )
        self.fv = RobotVisual(self.server, self.sim.model, "follower", FOLLOWER_OFFSET)
        self.tcp_frame = self.server.scene.add_frame(
            "/tcp", axes_length=0.04, axes_radius=0.0015
        )
        self.target = self.server.scene.add_transform_controls(
            "/target",
            scale=0.07,
            position=self.sim.tip + FOLLOWER_OFFSET,
            disable_rotations=True,
            translation_limits=((-0.1, 0.7), (-0.4, 0.6), (0.01, 0.65)),
        )
        self.frustum = self.server.scene.add_camera_frustum(
            "/wrist_camera",
            fov=np.deg2rad(70),
            aspect=4 / 3,
            scale=0.09,
            thickness=0.001,
            color=(90, 210, 240),
        )
        self.events = queue.Queue()
        self.paused = False
        self.demo_start = None
        self.tool_cycle = []
        self.renderer = None
        self.recording = None
        self.record_path = None
        self.frame_count = 0
        self.last_images = {}
        self.message = (
            "Drag the target, then solve; or move the leader with the sliders."
        )
        self.build_gui()

        @self.server.on_client_connect
        def _(client):
            client.camera.position = (0.95, -1.15, 0.85)
            client.camera.look_at = (0.12, -0.04, 0.20)
            client.camera.up_direction = (0.0, 0.0, 1.0)

    def action(self, handle, action):
        handle.on_click(lambda _: self.events.put(action))

    def build_gui(self):
        gui = self.server.gui
        gui.add_markdown(
            "# SO-101 Workbench\n**Move → observe → refine → build**\n\nTeal: hand-operated leader. Yellow: powered follower. Orbit: drag · pan: right-drag · zoom: scroll."
        )
        tabs = gui.add_tab_group()
        with tabs.add_tab("Operate"):
            with gui.add_folder("Leader input → follower"):
                self.sliders = []
                for i, name in enumerate(JOINTS[:5]):
                    h = gui.add_slider(
                        name.replace("_", " ").title(),
                        min=float(np.rad2deg(self.sim.limits[i, 0])),
                        max=float(np.rad2deg(self.sim.limits[i, 1])),
                        step=0.1,
                        initial_value=float(np.rad2deg(HOME[i])),
                    )
                    h.on_update(lambda _: self.events.put("joints"))
                    self.sliders.append(h)
                self.jaw = gui.add_slider(
                    "Tool command %",
                    min=0.0,
                    max=100.0,
                    step=0.5,
                    initial_value=0.0,
                    hint="Gripper: travel; four fingers: tendon pull; stirrer: 0–60 RPM. Enable tool power first.",
                )
                self.jaw.on_update(lambda _: self.events.put("tool_command"))
                self.action(gui.add_button("Enable / disable tool power"), "tool_power")
                self.action(gui.add_button("Run motion demo"), "demo")
                self.action(gui.add_button("Reset home + cube"), "reset")
                self.pause_button = gui.add_button("Pause physics")
                self.action(self.pause_button, "pause")
            with gui.add_folder("Reach a point"):
                gui.add_markdown(
                    "Drag the 3D axis handle at the follower's tip. Solve checks a sampled straight joint path. Tool orientation is free."
                )
                self.action(gui.add_button("Solve target + move"), "ik")
                self.action(gui.add_button("Target to current tip"), "target_tip")
            self.status = gui.add_markdown("")
            self.telemetry = gui.add_markdown("")
        with tabs.add_tab("Vision & tools"):
            self.tool = gui.add_dropdown("Attachment", TOOLS)
            self.action(
                gui.add_button("Automatically change to selected tool"), "change"
            )
            self.action(gui.add_button("Cycle all four modules"), "cycle")
            self.action(
                gui.add_button("Stop change + hold position", color="red"),
                "stop_change",
            )
            self.changer_status = gui.add_markdown("")
            self.camera = gui.add_checkbox(
                "32 × 32 mm wrist camera", initial_value=True
            )
            self.load = gui.add_slider(
                "Extra tip payload (g)", min=0.0, max=500.0, step=5.0, initial_value=0.0
            )
            self.force = gui.add_slider(
                "Downward test force (N)",
                min=0.0,
                max=10.0,
                step=0.1,
                initial_value=0.0,
            )
            self.torque = gui.add_slider(
                "Servo torque ceiling (Nm)",
                min=0.2,
                max=3.35,
                step=0.01,
                initial_value=1.6,
            )
            self.fov = gui.add_slider(
                "Assumed vertical FOV (°)",
                min=25.0,
                max=120.0,
                step=1.0,
                initial_value=70.0,
            )
            self.action(gui.add_button("Apply setup + reset"), "setup")
            gui.add_markdown(
                "**ATC-02 automatic tool-change prototype.** These modules replace the complete gripper. The rack and arm exchange support through ideal weld constraints; lock, ID and power sensors are synthetic. The digital cycle does not validate a physical latch or connector.\n\nTool power is switched off before coupling separation. Motor limits, camera optics and printed mass are estimates."
            )
            self.rgb = gui.add_image(
                np.zeros((240, 320, 3), dtype=np.uint8), label="Wrist RGB · simulated"
            )
            self.overhead = gui.add_image(
                np.zeros((240, 320, 3), dtype=np.uint8),
                label="Overhead RGB · simulated",
            )
            self.vision_status = gui.add_markdown("Camera initializing…")
            self.proxy = gui.add_checkbox("Show collision proxies", initial_value=False)
        with tabs.add_tab("Record"):
            gui.add_markdown(
                "Record synchronized simulated camera frames, leader commands, actual follower joints, torque and contacts.\n\nThis is a local experiment format. Training and LeRobot dataset export are future work; simulated demonstrations alone don't validate cooking."
            )
            self.record_button = gui.add_button("Start recording")
            self.action(self.record_button, "record")
            self.record_status = gui.add_markdown("No recording yet.")
            self.action(gui.add_button("Save pose + camera snapshot"), "snapshot")
            self.action(gui.add_button("Export 3D preview"), "preview")
            gui.add_markdown(
                "Files stay on this computer in `Simulation/Workbench/runs/`. Recordings are excluded from Git."
            )
        with tabs.add_tab("Build"):
            gui.add_markdown(
                "## ATC-02 build package\n**Printer:** Bambu Lab A1 mini · 180 mm bed\n\n**Four removable modules:** two-finger gripper, four-finger gripper, spatula, powered stirrer. The permanent wrist receiver carries the camera.\n\n**Automatic change:** oriented dock approach, rack capture, power discharge, pin release, support transfer, new tool identification and lock verification.\n\nSTEP/STL: `Build/Tooling/parts/`. Bed-sized components: `Build/Tooling/print/`. Full pedestal/housing meshes are assembly references.\n\nStart with plug/socket coupons and one receiver. Read `Tooling/README.md`, `MECHANICAL.md`, `electronics/INTERFACE.md` and `ACCEPTANCE.md`.\n\nThe rack actuator installation, electrical PCB and MCU firmware remain bench work. Sensors and retention are idealized in simulation.\n\n[Your fork](https://github.com/jdragon3001/SO-ARM100/tree/sim/workbench) · [Assembly guide](https://huggingface.co/docs/lerobot/so101)"
            )

    def sync_sliders(self):
        for h, q in zip(self.sliders, self.sim.target):
            h.value = float(np.rad2deg(q))
        self.jaw.value = self.sim.tool_command * 100

    def stop_recording(self):
        if self.recording is not None:
            self.recording.close()
            self.recording = None
            self.record_button.label = "Start recording"
            self.record_status.content = (
                f"Saved **{self.frame_count} frames** to `{self.record_path}`."
            )

    def metadata(self):
        return {
            "format": "so-arm-workbench/2",
            "simulated": True,
            "joint_names": list(JOINTS),
            "units": "metres, radians, seconds, Newton-metres",
            "world_frame": "Follower base origin, +Z up; display offsets excluded",
            "upstream_commit": "a758567c3978dfeefe282ede0500085a48fe8f78",
            "config": dataclasses.asdict(self.sim.config),
            "capture_rate_hz": 10,
            "resolution": [320, 240],
            "camera_extrinsics_calibrated": False,
            "tool_mount": "ATC-02 ideal coupling, synthetic sensors",
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }

    def camera_state(self, name):
        i = self.sim.model.camera(name).id
        fy = 240 / (2 * np.tan(np.deg2rad(self.sim.model.cam_fovy[i]) / 2))
        return {
            "world_position_m": self.sim.data.cam_xpos[i].tolist(),
            "world_rotation_matrix": self.sim.data.cam_xmat[i].reshape(3, 3).tolist(),
            "convention": "MuJoCo/OpenGL: right +X, up +Y, forward -Z",
            "intrinsics": [[float(fy), 0, 160], [0, float(fy), 120], [0, 0, 1]],
        }

    def handle(self, action):
        if action == "joints":
            self.demo_start = None
            q = np.deg2rad([h.value for h in self.sliders])
            self.sim.command(q)
        elif action == "tool_command":
            if self.sim.busy:
                raise ValueError("Wait for tool change to finish")
            self.sim.set_tool_control(self.jaw.value / 100)
        elif action == "tool_power":
            if self.sim.busy or self.sim.phase == "FAULT":
                raise ValueError("Changer must be READY")
            self.sim.power_enabled = not self.sim.power_enabled
        elif action in ("change", "cycle"):
            self.demo_start = None
            self.paused = False
            self.pause_button.label = "Pause physics"
            self.tool_cycle = list(TOOLS[1:]) + [TOOLS[0]] if action == "cycle" else []
            self.sim.change_tool(
                self.tool_cycle.pop(0) if self.tool_cycle else self.tool.value
            )
        elif action == "stop_change":
            self.tool_cycle = []
            self.demo_start = None
            self.sim.fail("Stopped by operator; reset required")
        elif action == "reset":
            self.stop_recording()
            self.demo_start = None
            self.tool_cycle = []
            self.sim.reset()
            self.sync_sliders()
            self.target.position = self.sim.tip + FOLLOWER_OFFSET
            self.message = "Reset. All motion stays inside this simulation."
        elif action == "pause":
            self.paused = not self.paused
            self.pause_button.label = (
                "Resume physics" if self.paused else "Pause physics"
            )
        elif action == "demo":
            if self.sim.busy or self.sim.phase == "FAULT":
                raise ValueError(
                    "Finish the change or reset the fault before running a demo"
                )
            self.demo_start = self.sim.data.time
            self.paused = False
            self.pause_button.label = "Pause physics"
            self.message = "Leader motion demo running for 12 seconds. Record it from the Record tab."
        elif action == "target_tip":
            self.target.position = self.sim.tip + FOLLOWER_OFFSET
        elif action == "ik":
            self.demo_start = None
            result = self.sim.ik(np.array(self.target.position) - FOLLOWER_OFFSET)
            self.message = (
                f"{result['reason']} · position residual {result['error_mm']:.2f} mm."
            )
            if result["ok"]:
                self.sim.command(result["q"])
                self.sync_sliders()
        elif action == "setup":
            self.stop_recording()
            current = (
                MODULES[self.sim.attached]["label"] if self.sim.attached else TOOLS[0]
            )
            config = Config(
                tool=current,
                payload_g=self.load.value,
                torque_nm=self.torque.value,
                camera=self.camera.value,
                camera_fovy=self.fov.value,
                tool_force_n=self.force.value,
            )
            candidate = Simulation(config)
            self.demo_start = None
            self.tool_cycle = []
            if self.renderer is not None:
                self.renderer.close()
                self.renderer = None
            self.fv.remove()
            self.sim = candidate
            self.fv = RobotVisual(
                self.server, self.sim.model, "follower", FOLLOWER_OFFSET
            )
            self.frustum.fov = np.deg2rad(config.camera_fovy)
            self.frustum.visible = config.camera
            self.last_images = {}
            self.sync_sliders()
            self.target.position = self.sim.tip + FOLLOWER_OFFSET
            self.message = "Setup reset. Use automatic change to exchange modules."
        elif action == "record":
            if self.recording is not None:
                self.stop_recording()
            else:
                if self.renderer is None or not self.last_images:
                    self.message = "Wait for rendered camera frames before recording."
                    return
                self.record_path = RUNS / datetime.now().strftime(
                    "episode_%Y%m%d_%H%M%S_%f"
                )
                self.record_path.mkdir(parents=True)
                for name in self.last_images:
                    (self.record_path / name).mkdir()
                (self.record_path / "metadata.json").write_text(
                    json.dumps(self.metadata(), indent=2)
                )
                self.recording = (self.record_path / "frames.jsonl").open("w")
                self.frame_count = 0
                self.record_button.label = "Stop + save recording"
        elif action == "snapshot":
            path = RUNS / datetime.now().strftime("snapshot_%Y%m%d_%H%M%S_%f")
            self.capture(record=False)
            self.sim.save(path / "state.json")
            (path / "metadata.json").write_text(json.dumps(self.metadata(), indent=2))
            (path / "cameras.json").write_text(
                json.dumps(
                    {name: self.camera_state(name) for name in self.last_images},
                    indent=2,
                )
            )
            for name, rgb in self.last_images.items():
                Image.fromarray(rgb).save(path / f"{name}.png")
            self.record_status.content = f"Snapshot saved to `{path}`."
        elif action == "preview":
            clients = self.server.get_clients()
            if clients:
                camera = next(iter(clients.values())).camera
                previous_fov = camera.fov
                try:
                    camera.fov = np.deg2rad(38.0)
                    rgb = camera.get_render(700, 1100, timeout=10.0)
                finally:
                    camera.fov = previous_fov
                path = ROOT / "Build/workbench-preview.png"
                path.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(rgb).save(path)
                self.record_status.content = "Saved `Build/workbench-preview.png`."

    def update_scene(self):
        q = map_to_leader(
            self.sim.target, self.sim.limits, self.leader_limits, self.sim.tool_command
        )
        self.leader_data.qpos[self.leader_ids] = q
        mujoco.mj_forward(self.leader, self.leader_data)
        self.lv.update(self.leader_data)
        self.fv.update(self.sim.data, self.proxy.value)
        self.tcp_frame.position = self.sim.tip + FOLLOWER_OFFSET
        self.tcp_frame.wxyz = quaternion(self.sim.data.site_xmat[self.sim.site])
        if self.sim.config.camera:
            i = self.sim.model.camera("wrist_rgb").id
            # MuJoCo camera uses -Z forward, Viser/OpenCV uses +Z forward.
            rotation = self.sim.data.cam_xmat[i].reshape(3, 3) @ np.diag(
                [1.0, -1.0, -1.0]
            )
            self.frustum.position = self.sim.data.cam_xpos[i] + FOLLOWER_OFFSET
            self.frustum.wxyz = quaternion(rotation)

    def capture(self, record=True):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.sim.model, height=240, width=320)
        self.last_images = {}
        cameras = ["overhead"] + (["wrist_rgb"] if self.sim.config.camera else [])
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        options.sitegroup[:] = 0
        for name in cameras:
            self.renderer.update_scene(self.sim.data, camera=name, scene_option=options)
            self.last_images[name] = self.renderer.render().copy()
        self.overhead.image = self.last_images["overhead"]
        self.rgb.visible = self.sim.config.camera
        if self.sim.config.camera:
            self.rgb.image = self.last_images["wrist_rgb"]
        self.vision_status.content = "Live simulated RGB · 320 × 240 · 10 Hz simulation clock. Optical frame/FOV are provisional."
        if self.recording is not None and record:
            record = self.sim.state()
            record["frame_index"] = self.frame_count
            record["leader_rad"] = self.leader_data.qpos[self.leader_ids].tolist()
            record["images"] = {}
            record["cameras"] = {
                name: self.camera_state(name) for name in self.last_images
            }
            for name, rgb in self.last_images.items():
                relative = f"{name}/{self.frame_count:06d}.png"
                Image.fromarray(rgb).save(self.record_path / relative)
                record["images"][name] = relative
            self.recording.write(json.dumps(record) + "\n")
            self.recording.flush()
            self.frame_count += 1
            self.record_status.content = (
                f"Recording **{self.frame_count} frames** · `{self.record_path.name}`"
            )

    def update_status(self):
        state = self.sim.state()
        s = state["synthetic_sensors"]
        self.changer_status.content = (
            f"**{self.sim.phase}** · attached: {self.sim.attached or 'empty'}\n\n"
            f"Lock: {s['locked']} · tool rail: {s['rail_v']:.2f} V · power: {self.sim.power_enabled}\n\n"
            f"{self.sim.error}\n\nFeedback is simulated; hardware has not been qualified."
        )
        hits = state["contacts"]
        tracking = np.max(np.abs(np.rad2deg(self.sim.target - self.sim.q)))
        peak = max(state["stall_fraction"])
        condition = (
            "CONTACT" if hits else ("TORQUE SATURATED" if peak > 0.95 else "RUNNING")
        )
        if self.paused:
            condition = "PAUSED"
        self.status.content = f"**{condition}** · {state['time_s']:.1f} s · worst joint error {tracking:.1f}°\n\n{self.message}"
        rows = "\n".join(
            f"| {n.replace('_', ' ')} | {np.rad2deg(a):.1f}° | {t:.2f} | {p * 100:.0f}% |"
            for n, a, t, p in zip(
                JOINTS, state["actual_rad"], state["torque_nm"], state["stall_fraction"]
            )
        )
        contact = "; ".join(" / ".join(c["bodies"]) for c in hits[:3]) or "none"
        self.telemetry.content = (
            "| Joint | Actual | Nm | Ceiling |\n|---|---:|---:|---:|\n"
            + rows
            + f"\n\nTCP: {', '.join(f'{v * 1000:.0f}' for v in state['tcp_m'])} mm\n\nRobot contacts: {contact}\n\nTorque % is relative to the assumed stall ceiling, not a continuous-duty rating."
        )

    def run(self):
        last_capture = -1.0
        try:
            while True:
                started = time.monotonic()
                while not self.events.empty():
                    try:
                        self.handle(self.events.get_nowait())
                    except (ValueError, RuntimeError, TimeoutError) as error:
                        self.message = str(error)
                if not self.paused:
                    if self.sim.phase == "FAULT":
                        self.tool_cycle = []
                        self.demo_start = None
                    elif not self.sim.busy and self.tool_cycle:
                        self.sim.change_tool(self.tool_cycle.pop(0))
                    if self.sim.busy:
                        self.demo_start = None
                    if self.demo_start is not None:
                        t = self.sim.data.time - self.demo_start
                        q = HOME.copy()
                        if t < 12:
                            q[0] += 0.4 * np.sin(t * np.pi / 6)
                            q[1] += 0.12 * np.sin(t * np.pi / 3)
                            q[2] += 0.12 * np.sin(t * np.pi / 3)
                            self.sim.command(q)
                        else:
                            self.demo_start = None
                            self.sim.command(HOME)
                            self.sync_sliders()
                            self.message = "Motion demo complete. Try a tool or change the payload."
                    try:
                        self.sim.step(10)
                    except RuntimeError as error:
                        self.paused = True
                        self.pause_button.label = "Resume physics"
                        self.stop_recording()
                        self.message = str(error)
                self.update_scene()
                now = self.sim.data.time
                if now < last_capture:
                    last_capture = -1.0
                if now - last_capture >= 0.1 - 1e-8 and self.server.get_clients():
                    try:
                        self.capture()
                    except Exception as error:
                        self.stop_recording()
                        self.vision_status.content = (
                            f"Camera rendering unavailable: {error}"
                        )
                    self.update_status()
                    last_capture = now
                elif self.paused:
                    self.update_status()
                time.sleep(max(0.0, 0.02 - (time.monotonic() - started)))
        finally:
            self.stop_recording()
            if self.renderer is not None:
                self.renderer.close()
            self.server.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    Workbench(args.port).run()
