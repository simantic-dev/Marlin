"""Headline video: MuJoCo printer (left), Marlin console + commanded-vs-printed path drawn live (right).

usage: python video.py <run dir> logo.gcode out.mp4 [speed]   (run dir: frames.jsonl, trace.txt, sim.txt)
Every number on screen comes from the run. Needs render.py and printer.xml (this dir) and mujoco.
"""
import json
import os
import re
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import render
render.BEAD_RGBA = [0.1, 0.35, 0.95, 1]  # blue bead: visible on the amber bed
from accuracy import gcode_segments, nearest

W, H, VW = 1280, 720, 720
CAM = dict(lookat=(0.1, 0.1, 0.21), distance=0.17, azimuth=90, elevation=-45)
TITLE = "Stock Marlin on a simulated Teensy 4.1 + MuJoCo printer"
FOOT = ("Modelled: MuJoCo mechanics, X on a SimpleFOC servo (2nd simulated Teensy 4.1, PMSM + MT6835), Y/Z/E steppers,\n"
        "datasheet belt/motor stiffness, stand-in damping (zeta 0.1). Firmware unmodified; pyrite simulator.")
BG, FG, DIM = (18, 20, 24), (235, 235, 235), (140, 140, 150)


def main(run, gc, out, speed=2.0):
    here = os.path.dirname(os.path.abspath(__file__))
    font = ImageFont.load_default(size=24)
    small = ImageFont.load_default(size=14)
    mono = ImageFont.load_default(size=13)
    frames = [f for f in render.load(os.path.join(run, "frames.jsonl")) if f["t"] >= 4.5e9]  # skip the G4 S5 encoder alignment
    tr = np.loadtxt(os.path.join(run, "trace.txt"))
    tr_t, ext = tr[:, 0], np.r_[False, np.diff(tr[:, 6]) > 1e-5]
    # console: "[dut] [12.345678s] (usb1) text" lines from sim.txt, guest time
    con = []
    for l in open(os.path.join(run, "sim.txt"), errors="replace"):
        m = re.match(r"(?:\[dut\] )?\[(\d+\.\d+)s\] \(usb1\) (.*)", l)
        if m and m.group(2).strip():
            con.append((float(m.group(1)) * 1e9, m.group(2).strip()[:60]))
    segs = gcode_segments(gc)
    err = np.zeros(len(tr))
    err[ext] = nearest(tr[ext][:, 1:3], segs) * 1000
    worst = 0.0
    x0, x1, y0, y1 = segs[:, [0, 2]].min() - 2, segs[:, [0, 2]].max() + 2, segs[:, [1, 3]].min() - 2, segs[:, [1, 3]].max() + 2
    PX, PY, PW = VW + 20, 360, W - VW - 40
    s = PW / (x1 - x0)
    PH = (y1 - y0) * s
    to = lambda x, y: (PX + (x - x0) * s, PY + PH - (y - y0) * s)
    base = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(base)
    for a in segs:
        d.line([to(a[0], a[1]), to(a[2], a[3])], fill=(90, 90, 100), width=1)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                           "-r", str(round(30 * speed)), "-i", "-", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "23", out],
                          stdin=subprocess.PIPE)
    k, now = 0, 0.0
    for f, (img,) in render.frames_to_images(os.path.join(here, "printer.xml"), frames, [CAM], VW, H - 110):
        while k < len(tr_t) and tr_t[k] <= f["t"]:
            if ext[k]:
                p = to(tr[k, 1], tr[k, 2])
                c = min(1.0, err[k] / 150)  # amber at 0 um -> red at >= 150 um
                d.ellipse([p[0] - 0.9, p[1] - 0.9, p[0] + 0.9, p[1] + 0.9], fill=(245, int(190 * (1 - c) + 30 * c), int(40 * (1 - c))))
                worst, now = max(worst, err[k]), err[k]
            k += 1
        fr = base.copy()
        fr.paste(Image.fromarray(img), (0, 60))
        g = ImageDraw.Draw(fr)
        g.rectangle([0, 0, W, 58], fill=BG)
        g.text((16, 14), TITLE, font=font, fill=FG)
        g.text((PX, PY - 22), "top view: G-code (grey) vs printed path, amber 0 um -> red >= 150 um", font=small, fill=DIM)
        g.text((PX, 70), "Marlin console (USB CDC)", font=small, fill=DIM)
        lines = [c for c in con if c[0] <= f["t"]][-14:]
        for i, (t, txt) in enumerate(lines):
            g.text((PX, 92 + i * 16), f"{t / 1e9:7.2f}  {txt}", font=mono, fill=FG)
        x, y, z = (v * 1000 for v in f["q"])
        g.text((PX, PY + PH + 14), f"t {f['t'] / 1e9:6.2f} s virtual   X {x:7.2f}  Y {y:7.2f}  Z {z:5.2f} mm   ({speed:g}x)",
               font=small, fill=FG)
        g.text((PX, PY + PH + 34), f"path error now {now if ext[k - 1] else 0:5.0f} um   worst so far {worst:5.0f} um", font=small, fill=FG)
        g.rectangle([0, H - 50, W, H], fill=BG)
        g.multiline_text((16, H - 46), FOOT, font=small, fill=DIM)
        ff.stdin.write(fr.tobytes())
    ff.stdin.close()
    ff.wait()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], *(float(a) for a in sys.argv[4:5]))
