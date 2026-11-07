"""Render actual CAD-backed tool models, not concept artwork."""

import sys
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "Simulation/Workbench"))
from engine import Simulation, Config, TOOLS, KEYS


def main():
    output = ROOT / "Build/Tooling/previews"
    output.mkdir(exist_ok=True)
    canvas = Image.new("RGB", (1280, 1090), "#f3f5f8")
    draw = ImageDraw.Draw(canvas)
    try:
        title = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 28)
        text = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 18)
    except OSError:
        title = text = ImageFont.load_default()
    draw.text(
        (25, 18), "ATC-02 · four complete removable modules", font=title, fill="#172c3c"
    )
    draw.text(
        (25, 56),
        "CAD-backed digital prototypes · common receiver and permanent wrist camera",
        font=text,
        fill="#40596d",
    )
    for index, (label, key) in enumerate(zip(TOOLS, KEYS)):
        sim = Simulation(Config(tool=label))
        sim.step(150)
        m, d = sim.model, sim.data
        tool = m.body("tool_" + key).id
        receiver = m.body("receiver").id

        def belongs(body, root):
            while body:
                if body == root:
                    return True
                body = int(m.body_parentid[body])
            return False

        for g in range(m.ngeom):
            if not (
                belongs(int(m.geom_bodyid[g]), tool)
                or belongs(int(m.geom_bodyid[g]), receiver)
            ):
                m.geom_group[g] = 5
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        options.geomgroup[5] = 0
        options.sitegroup[:] = 0
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = d.body("tool_" + key).xpos + d.body(
            "tool_" + key
        ).xmat.reshape(3, 3) @ [0, 0, 0.045]
        camera.distance = 0.235
        camera.azimuth = 25
        camera.elevation = -20
        renderer = mujoco.Renderer(m, height=440, width=640)
        renderer.update_scene(d, camera=camera, scene_option=options)
        im = Image.fromarray(renderer.render())
        renderer.close()
        im.save(output / f"{key}.png")
        x = (index % 2) * 640
        y = 95 + (index // 2) * 490
        canvas.paste(im, (x, y))
        draw.text((x + 20, y + 448), label, font=title, fill="#172c3c")
    draw.text(
        (25, 1060),
        "Ideal couplings and synthetic sensors; physical retention and automatic docking are not qualified.",
        font=text,
        fill="#7b3935",
    )
    canvas.save(output / "four_modules.png")


if __name__ == "__main__":
    main()
