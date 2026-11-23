"""Render a run's MuJoCo frames (frames.jsonl from peers/printer_plant.py) to an MP4.

usage: python render.py printer.xml frames.jsonl out.mp4 [speed]
Every number on screen comes from the run: virtual time, MuJoCo joint positions, step-counted E.
"""
import json
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
BEAD_R = 0.0002  # 0.4 mm bead
BEAD_RGBA = [0.95, 0.75, 0.15, 1]
WIDE = dict(lookat=(0.1, 0.1, 0.17), distance=0.75, azimuth=135, elevation=-22)
CLOSE = dict(lookat=(0.1, 0.1, 0.225), distance=0.12, azimuth=120, elevation=-18)


def camera(lookat, distance, azimuth, elevation):
    c = mujoco.MjvCamera()
    c.lookat[:] = lookat
    c.distance, c.azimuth, c.elevation = distance, azimuth, elevation
    return c


def load(path):
    return [json.loads(l) for l in open(path)]


def frames_to_images(xml, frames, cams, width, height):
    """Yield (frame, [image per camera]) with the deposited filament drawn; bead segments ride on the bed."""
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[model.joint(n).id] for n in "xyz"]
    nozzle, bed = model.site("nozzle").id, model.body("bed").id
    r = mujoco.Renderer(model, height, width, max_geom=20000)
    cams = [camera(**c) for c in cams]
    beads, prev = [], None
    for f in frames:
        data.qpos[qadr] = f["q"]
        mujoco.mj_forward(model, data)
        tip = data.site_xpos[nozzle].copy() - data.xpos[bed]
        if prev is not None and f["e"] > prev[1] + 1e-4:
            beads.append((prev[0], tip))
        prev = (tip, f["e"])
        images = []
        for cam in cams:
            r.update_scene(data, cam)
            s = r.scene
            for a, b in beads:
                if s.ngeom >= s.maxgeom:
                    break
                g = s.geoms[s.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9),
                                    np.array(BEAD_RGBA, np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, BEAD_R, a + data.xpos[bed], b + data.xpos[bed])
                s.ngeom += 1
            images.append(r.render())
        yield f, images


def main(xml, frames_path, out, speed=2.0):
    font = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=16)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{W}x{H}", "-r", str(round(30 * speed)), "-i", "-",
                           "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "18", out], stdin=subprocess.PIPE)
    for f, halves in frames_to_images(xml, load(frames_path), [WIDE, CLOSE], W // 2, H):
        img = Image.fromarray(np.hstack(halves))
        d = ImageDraw.Draw(img)
        x, y, z = (v * 1000 for v in f["q"])
        d.text((16, 14), "Stock Marlin on a simulated Teensy 4.1 (pyrite)  ->  STEP/DIR pins  ->  MuJoCo", font=font, fill="white")
        d.text((16, H - 30), f"t = {f['t'] / 1e9:6.2f} s virtual   X {x:6.2f}  Y {y:6.2f}  Z {z:5.2f} mm   E {f['e']:6.2f} mm   ({speed:g}x)",
               font=small, fill="white")
        ff.stdin.write(img.tobytes())
    ff.stdin.close()
    ff.wait()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], *(float(a) for a in sys.argv[4:5]))
