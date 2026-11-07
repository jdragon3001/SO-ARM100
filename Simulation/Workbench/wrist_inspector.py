"""Isolated wrist cutaway and actuator demonstration; never drives the main arm."""

from __future__ import annotations

import json
import time

import numpy as np
import trimesh

from engine import Config, KEYS, MODULES, TOOLS, Simulation
from spec import (
    BUILD,
    PIN_AXIS_Z,
    PIN_DIAMETER,
    PIN_LENGTH,
    PIN_TIP_X,
    PIN_TRAVEL,
    RECEIVER_HEIGHT,
)

ORIGIN = np.array([1.8, 0.0, 0.3])
# Local Z projects right and local X up when viewed from the removed Y half.
ROTATION = (2**-0.5, 0.0, -(2**-0.5), 0.0)
MM = 0.001
ROOT = "/wrist_inspector"
ARM = ROOT + "/cutaway"
MODULE = ROOT + "/module"

# Labels, axial separation (mm), pin displacement (mm), motor request.
STAGES = (
    ("1 · Separate", 24, PIN_TRAVEL, False),
    ("2 · Seat the key", 0, PIN_TRAVEL, False),
    ("3 · Engage the pin", 0, 0, False),
    ("4 · Power and run", 0, 0, True),
    ("5 · Isolate power", 0, 0, False),
    ("6 · Retract the pin", 0, PIN_TRAVEL, False),
)

STAGE_TEXT = (
    "The rack would hold the tool. The spring pin is pulled out of the key's path.",
    "The clipped corner lines up with the socket. The key enters 9 mm; the pin remains clear.",
    "The 3 mm steel pin enters the key's cross-hole. The printed key resists twist; the pin blocks pullout.",
    "After confirming seating, lock and tool ID, the base can enable the protected tool rail.",
    "Stop the tool and switch off its rail. Keep the pin engaged until voltage has fallen below 0.5 V and the rack has captured the tool.",
    "With the rail off and the tool supported, the dock would pull the pin back 7 mm.",
)

DRIVE_TEXT = {
    "gripper": "Module controller → TTL servo → horn pivots the moving jaw.",
    "soft_gripper": "Module controller → servo and spool → one tendon pulls four fingers. Hinges approximate the printed flexures.",
    "spatula": "Passive ID only in this demonstration. The spatula has no motor; the arm moves the blade.",
    "stirrer": "Module controller → motor driver → N20 gearmotor turns a shaft and paddle. The displayed RPM is simulated.",
}

CONTACTS = (
    ("+6 V ×2", (231, 94, 75)),
    ("Return ×2", (163, 175, 190)),
    ("RS-485 A/B", (93, 175, 247)),
    ("ID / present", (246, 197, 86)),
)


def pose_between(source, destination, fraction):
    """Interpolate adjacent stages only. Holding the lock still is deliberate."""
    _, start_gap, start_pin, _ = STAGES[source]
    _, end_gap, end_pin, _ = STAGES[destination]
    t = np.clip(fraction, 0, 1)
    t = t * t * (3 - 2 * t)
    return (1 - t) * start_gap + t * end_gap, (1 - t) * start_pin + t * end_pin


class WristInspector:
    def __init__(self, server, visual_factory, dispatch):
        self.server = server
        self.visual_factory = visual_factory
        self.active = False
        self.stage_index = 0
        self.animation = None
        self.tool_view = None
        self.demo = None
        self.last_tick = 0.0
        self.drive_is_on = False
        self.drive_parts = []
        self.report = json.loads((BUILD / "inspection/pin_geometry.json").read_text())
        scene = self.server.scene
        self.root = scene.add_frame(
            ROOT, position=ORIGIN, wxyz=ROTATION, show_axes=False, visible=False
        )
        self.cutaway = scene.add_frame(ARM, show_axes=False)
        self.module = scene.add_frame(MODULE, show_axes=False, visible=False)
        self.plug = scene.add_frame(ARM + "/plug", show_axes=False)

        def section(path, name, color):
            mesh = trimesh.load_mesh(BUILD / "inspection" / f"{name}_section.stl")
            return scene.add_mesh_simple(
                path, mesh.vertices * MM, mesh.faces, color=color
            )

        section(ARM + "/receiver", "receiver", (153, 191, 217))
        section(ARM + "/plug/key", "test_plug", (247, 166, 76))
        pin = trimesh.creation.cylinder(
            radius=PIN_DIAMETER * MM / 2, height=PIN_LENGTH * MM, sections=48
        )
        pin.apply_transform(
            trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
        )
        self.pin = scene.add_mesh_simple(
            ARM + "/pin", pin.vertices, pin.faces, color=(250, 224, 105)
        )

        def label(path, text, position):
            scene.add_label(
                path,
                text,
                position=np.array(position) * MM,
                anchor="center-center",
                font_screen_scale=0.9,
            )

        def callout(path, text, start, end):
            scene.add_line_segments(
                path + "_line",
                np.array([[start, end]]) * MM,
                (191, 209, 223),
                thickness=1.4,
                thickness_units="screen",
            )
            label(path, text, end)

        callout(
            ARM + "/receiver_label",
            "Receiver · stays on wrist",
            [-14, 0, 7],
            [-29, 0, -6],
        )
        callout(ARM + "/plug/key_label", "Key + tool plate", [-8, 0, -3], [-19, 0, 15])
        callout(ARM + "/pin_label", "3 mm steel pin", [28, -1, 20], [44, 0, 3])
        label(
            ARM + "/contact_label",
            "Proposed contact row · shown off-part",
            [-60, 0, 30],
        )

        # Off-part electrical overlay, deliberately separate from the CAD.
        # The contact geometry and the final mounting are not specified here.
        self.contact_tool = scene.add_frame(ARM + "/contact_tool", show_axes=False)
        scene.add_box(
            ARM + "/carrier",
            dimensions=(0.025, 0.005, 0.004),
            color=(36, 55, 75),
            position=(-0.044, 0, (RECEIVER_HEIGHT - 3) * MM),
        )
        scene.add_box(
            ARM + "/contact_tool/pad_board",
            dimensions=(0.025, 0.005, 0.003),
            color=(48, 69, 85),
            position=(-0.044, 0, 0.0015),
        )
        self.pins = []
        for i in range(8):
            x = (-44 + (i - 3.5) * 2.54) * MM
            color = CONTACTS[i // 2][1]
            scene.add_box(
                ARM + f"/contact_tool/pad_{i}",
                dimensions=(0.0017, 0.0036, 0.0005),
                color=color,
                position=(x, 0, -0.00025),
            )
            scene.add_box(
                ARM + f"/contact_{i}",
                dimensions=(0.0011, 0.003, 0.002),
                color=color,
                position=(x, 0, (RECEIVER_HEIGHT - 1.5) * MM),
            )
            lamp = scene.add_box(
                ARM + f"/contact_active_{i}",
                dimensions=(0.0013, 0.0032, 0.0021),
                color=color,
                position=(x, 0.0018, (RECEIVER_HEIGHT - 1.5) * MM),
                visible=False,
            )
            self.pins.append(lamp)

        label(ARM + "/supply", "Base · 6 V switch", [-42, 4, -17])
        label(ARM + "/contact_tool/driver", "Tool driver", [-42, 4, 23])
        self.power_lines = []
        for group, (name, points) in zip(
            (ARM, ARM + "/contact_tool"),
            (
                ("input", [[-42, 2, -9], [-42, 2, 20]]),
                ("output", [[-42, 2, 3], [-42, 2, 15]]),
            ),
        ):
            scene.add_line_segments(
                group + "/" + name + "_off",
                np.array([points]) * MM,
                (84, 99, 114),
                thickness=2.5,
                thickness_units="screen",
            )
            self.power_lines.append(
                scene.add_line_segments(
                    group + "/" + name + "_on",
                    np.array([points]) * MM,
                    CONTACTS[0][1],
                    thickness=3.5,
                    thickness_units="screen",
                    visible=False,
                )
            )

        self.module_label = scene.add_label(
            MODULE + "/label", "", anchor="center-center", position=(0.057, 0, 0.07)
        )
        self.set_pose(24, PIN_TRAVEL)

        gui = self.server.gui
        gui.add_markdown(
            "**Inspect the wrist close up.** The cutaway uses the receiver and test plug CAD. The colored contact row is a proposed layout shown apart from the parts; it is not an installed connector."
        )
        self.open_button = gui.add_button("Open wrist close-up", color="teal")
        self.open_button.on_click(lambda _: dispatch("inspect_open"))
        self.back_button = gui.add_button("Return to robot view")
        self.back_button.on_click(lambda _: dispatch("inspect_back"))
        self.view = gui.add_dropdown("View", ("Lock cutaway", "Tool motion"))
        self.view.on_update(lambda _: dispatch("inspect_view"))
        self.stage = gui.add_dropdown("Coupling step", tuple(s[0] for s in STAGES))
        self.stage.on_update(lambda event: event.client and dispatch("inspect_stage"))
        self.play = gui.add_button("Play lock and release")
        self.play.on_click(lambda _: dispatch("inspect_play"))
        self.tool = gui.add_dropdown("Demo module", TOOLS)
        self.tool.on_update(lambda _: dispatch("inspect_tool"))
        self.command = gui.add_slider(
            "Demo demand (%)", min=0, max=100, step=1, initial_value=55
        )
        self.command.on_update(lambda _: dispatch("inspect_demand"))
        self.readout = gui.add_markdown("")
        self.electrical = gui.add_markdown("")
        gui.add_markdown(
            "**Power path (proposed):** base supply → protected 6 V switch → wrist loom → "
            "spring contacts and copper pads → module controller/driver → motor. "
            "Return uses a separate pair. RS-485 carries commands; ID and presence are passive. "
            "The stock arm servo bus stays separate.\n\n"
            "**Still unbuilt:** dock linkage, spring preload, contact carrier installation, PCB, wiring and firmware."
        )
        self.refresh()

    @property
    def key(self):
        return KEYS[TOOLS.index(self.tool.value)]

    def camera(self, client):
        view = self.view.value
        look = (-0.023, 0.0, -0.004) if view == "Lock cutaway" else (-0.062, 0.0, 0.0)
        pos = (
            (-0.026, 0.154, 0.024) if view == "Lock cutaway" else (-0.158, -0.19, 0.088)
        )
        client.camera.up_direction = (0.0, 0.0, 1.0)
        base_fov = np.deg2rad(47 if view == "Lock cutaway" else 48)
        aspect = client.camera.aspect if client.camera.image_height else 1.0
        width_margin = max(1.0, 0.82 / max(0.35, aspect))
        client.camera.fov = min(
            np.deg2rad(76), 2 * np.arctan(np.tan(base_fov / 2) * width_margin)
        )
        client.camera.position = ORIGIN + pos
        client.camera.look_at = ORIGIN + look

    def set_pose(self, gap, pin):
        self.plug.position = (0, 0, (RECEIVER_HEIGHT + gap) * MM)
        self.pin.position = (
            (PIN_TIP_X + PIN_LENGTH / 2 + pin) * MM,
            0,
            PIN_AXIS_Z * MM,
        )
        self.contact_tool.position = (0, 0, (RECEIVER_HEIGHT + gap) * MM)

    def open(self):
        self.active = True
        self.root.visible = True
        self.select_view()

    def close(self):
        self.active = False
        self.animation = None
        self.root.visible = False

    def select_view(self):
        tool = self.view.value == "Tool motion"
        self.module.visible = tool
        self.cutaway.visible = not tool
        if self.active:
            if tool:
                self.ensure_demo()
            for client in self.server.get_clients().values():
                self.camera(client)
        self.refresh()

    def ensure_demo(self):
        if self.demo is not None and self.demo.attached == self.key:
            return
        if self.tool_view is not None:
            self.tool_view.remove()
        for part in self.drive_parts:
            part.remove()
        self.drive_parts.clear()
        self.demo = Simulation(Config(tool=self.tool.value, camera=False))
        self.tool_body = self.demo.model.body("tool_" + self.key).id
        self.tool_view = self.visual_factory(
            self.server,
            self.demo.model,
            MODULE.lstrip("/") + "/tool",
            np.zeros(3),
            root_body=self.tool_body,
        )
        scene = self.server.scene
        if self.key == "stirrer":
            housing = self.demo.model.geom("tool_stirrer_visual_stirrer_housing").id
            for index, mesh, _ in self.tool_view.items:
                if index == housing:
                    mesh.opacity = 0.33
            self.drive_parts.append(
                scene.add_box(
                    MODULE + "/motor_envelope",
                    dimensions=(0.012, 0.010, 0.026),
                    color=(225, 175, 112),
                    position=(0, 0, 0.024),
                )
            )
            self.drive_parts.append(
                scene.add_label(
                    MODULE + "/motor_note",
                    "N20 motor envelope",
                    position=(-0.026, 0, 0.024),
                    anchor="center-center",
                )
            )
        elif self.key == "spatula":
            mesh = trimesh.load_mesh(BUILD / "parts/spatula_clamp.stl")
            self.drive_parts.append(
                scene.add_mesh_simple(
                    MODULE + "/clamp",
                    mesh.vertices * MM,
                    mesh.faces,
                    color=(88, 127, 161),
                    position=(0, -0.00575, 0.018),
                )
            )
        self.module_label.text = self.tool.value
        self.tool_view.update(self.demo.data, relative_to=self.tool_body)

    def select_stage(self, index):
        self.animation = None
        self.stage_index = index
        self.stage.value = STAGES[index][0]
        _, gap, pin, _ = STAGES[index]
        self.set_pose(gap, pin)
        self.refresh()

    def play_cycle(self):
        self.select_stage(0)
        self.animation = (time.monotonic(), 0)
        self.play.label = "Replay lock and release"

    def tick(self, now):
        if not self.active:
            return
        if self.animation is not None:
            start, old = self.animation
            target = (old + 1) % len(STAGES)
            pause = 2.2 if old in (3, 4) else 1.0
            fraction = (now - start - pause) / 0.9
            if fraction >= 1:
                self.stage_index = target
                self.stage.value = STAGES[target][0]
                self.set_pose(*STAGES[target][1:3])
                self.animation = None if target == 0 else (now, target)
                self.refresh()
            elif fraction >= 0:
                self.set_pose(*pose_between(old, target, fraction))
        if self.view.value == "Tool motion":
            self.ensure_demo()
            # Same MuJoCo actuator/force model as the main workbench, independent state.
            self.demo.power_enabled = self.drive_is_on
            self.demo.set_tool_control(self.command.value / 100)
            self.demo.step(10)
            self.tool_view.update(self.demo.data, relative_to=self.tool_body)
            if now - self.last_tick >= 0.2:
                self.refresh()
                self.last_tick = now

    def refresh(self):
        label, gap, _, requested = STAGES[self.stage_index]
        powered = requested and self.key != "spatula"
        self.drive_is_on = powered
        for trace in self.power_lines:
            trace.visible = powered
        for i, lamp in enumerate(self.pins):
            lamp.visible = (gap == 0 and i >= 6) or (powered and i < 6)
        measures = ""
        if (
            self.demo is not None
            and self.demo.attached == self.key
            and self.view.value == "Tool motion"
        ):
            observation = self.demo.state()["tool_observation"]
            if self.key == "gripper":
                measures = f" · Jaw angle {np.degrees(observation['gripper_rad']):.0f}°"
            elif self.key == "soft_gripper":
                finger = observation["finger_rad"][::2]
                measures = (
                    f" · Average proximal bend {np.degrees(np.mean(finger)):.0f}°"
                )
            elif self.key == "stirrer":
                measures = f" · Shaft speed {observation['stirrer_rpm']:.1f} RPM"
        self.readout.content = (
            f"### {label}\n{STAGE_TEXT[self.stage_index]}\n\n"
            f"**{self.tool.value}:** {DRIVE_TEXT[self.key]}\n\n"
            f"**Demo drive:** {'enabled' if powered else 'off'}{measures}"
        )
        status = "proposed 6 V on" if powered else "off"
        signal = "commands enabled" if powered else "inactive"
        passive = (
            f"{MODULES[self.key]['id_ohms'] / 1000:g} kΩ / present"
            if gap == 0
            else "open"
        )
        self.electrical.content = (
            f"| Contacts | Demo state |\n|---|---|\n"
            f"| 1–2 · Red · +6 V | {status} |\n"
            f"| 3–4 · Grey · Return | {'mated' if gap == 0 else 'open'} |\n"
            f"| 5–6 · Blue · RS-485 | {signal} |\n"
            f"| 7–8 · Gold · ID/present | {passive} |\n\n"
            "**Nominal STEP check:** the retracted pin clears the key; a 1 mm pull "
            f"with the pin engaged produces {self.report['locked_pin_intersection_on_1mm_pull_mm3']:.2f} mm³ of overlap. This checks geometry, not retention force."
        )
