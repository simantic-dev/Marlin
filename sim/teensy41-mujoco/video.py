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
from matplotlib import colormaps
from PIL import Image, ImageDraw, ImageFont

import render
render.BEAD_RGBA = [0.1, 0.35, 0.95, 1]  # blue bead: visible on the amber bed
from accuracy import gcode_segments, nearest

W, H, VW = 1280, 720, 720
CAM = dict(lookat=(0.1, 0.1, 0.225), distance=0.19, azimuth=90, elevation=-45)  # low front view: the nozzle sits under the head, the print stays in view
TITLE = "Stock Marlin on a simulated Teensy 4.1 + MuJoCo printer"
FOOT = "X and Y on closed-loop FOC servos (an upgrade some printers use); Z/E steppers"
BG, FG, DIM = (18, 20, 24), (235, 235, 235), (140, 140, 150)
EMAX = 150  # um: top of the error colour scale (turbo: blue 0 um -> green -> yellow -> red >= EMAX)
TURBO = colormaps["turbo"]


def ecol(e):
    r, g, b, _ = TURBO(0.1 + 0.9 * min(1.0, e / EMAX))  # skip turbo's darkest blues, unreadable on the dark panel
    return int(r * 255), int(g * 255), int(b * 255)


def main(run, gc, out, speed=2.0):
    here = os.path.dirname(os.path.abspath(__file__))
    font = ImageFont.load_default(size=24)
    small = ImageFont.load_default(size=14)
    mono = ImageFont.load_default(size=13)
    tr = np.loadtxt(os.path.join(run, "trace.txt"))
    tr_t, ext = tr[:, 0], np.r_[False, np.diff(tr[:, 6]) > 1e-5]
    start = tr_t[ext][0]  # open on the first extrusion: the head has parked at the logo's top-left corner
    frames = [f for f in render.load(os.path.join(run, "frames.jsonl")) if f["t"] >= start]
    # console: "[dut] [12.345678s] (usb1) text" lines from sim.txt, guest time
    # drive: "[xdrive] [t] (usb1) t=.. x target=.. angle=.. ... loop_hz=.." status lines, every 100 ms
    con, drv = [], []
    for l in open(os.path.join(run, "sim.txt"), errors="replace"):
        m = re.match(r"\[(dut|xdrive)\] \[(\d+\.\d+)s\] \(usb1( in)?\) (.*)", l)
        if not m:
            continue
        t, txt = float(m.group(2)) * 1e9, ("> " if m.group(3) else "") + m.group(4).strip()
        if m.group(1) == "xdrive":
            ax = re.findall(r"(\w) target=(\S+) angle=(\S+) maxerr=\S+ loop_hz=(\d+)", txt)
            if ax:  # angle error in um of belt: 40 mm per rev
                drv.append((t, "   ".join(f"{a.upper()} {abs(float(tg) - float(an)) * 40e3 / 6.2832:4.0f} um" for a, tg, an, _ in ax)
                            + f"   loop {ax[0][3]} Hz per motor"))
        elif txt and not re.match(r"echo:|//action", txt):
            con.append((t, txt[:64]))
    foot = FOOT if any(" Y " in t for _, t in drv) else "X on a closed-loop FOC servo (an upgrade some printers use); Y/Z/E steppers"
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
    bx, by, bw = PX + 4, PY - 52, 160  # colour key for the path error, on its own line above the label
    for i in range(bw):
        d.line([(bx + i, by), (bx + i, by + 8)], fill=ecol(EMAX * i / (bw - 1)))
    for v in (0, 50, 100, 150):
        d.text((bx + v / EMAX * (bw - 1) - 6, by + 10), f"{v}" + ("+" if v == EMAX else ""), font=ImageFont.load_default(size=10), fill=DIM)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                           "-r", str(round(30 * speed)), "-i", "-", "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "23", out],
                          stdin=subprocess.PIPE)
    k, now = 0, 0.0
    for f, (img,) in render.frames_to_images(os.path.join(here, "printer.xml"), frames, [CAM], VW, H - 110):
        while k < len(tr_t) and tr_t[k] <= f["t"]:
            if ext[k]:
                p = to(tr[k, 1], tr[k, 2])
                d.ellipse([p[0] - 1, p[1] - 1, p[0] + 1, p[1] + 1], fill=ecol(err[k]))
                worst, now = max(worst, err[k]), err[k]
            k += 1
        fr = base.copy()
        fr.paste(Image.fromarray(img), (0, 60))
        g = ImageDraw.Draw(fr)
        g.rectangle([0, 0, W, 58], fill=BG)
        g.text((16, 14), TITLE, font=font, fill=FG)
        g.text((PX, PY - 22), "top view: G-code (grey), printed path by error (um)", font=small, fill=DIM)
        g.text((PX, 70), "Marlin console (USB CDC): > G-code in (dim), firmware replies", font=small, fill=DIM)
        lines = [c for c in con if c[0] <= f["t"]][-7:]
        for i, (t, txt) in enumerate(lines):
            g.text((PX, 92 + i * 16), f"{t / 1e9:7.2f}  {txt}", font=mono, fill=DIM if txt.startswith("> ") else FG)
        g.text((PX, 222), "FOC drive (2nd Teensy 4.1): angle error now", font=small, fill=DIM)
        dl = [c for c in drv if c[0] <= f["t"]][-1:]
        if dl:
            g.text((PX, 244), f"{dl[0][0] / 1e9:7.2f}  {dl[0][1]}", font=mono, fill=FG)
        x, y, z = (v * 1000 for v in f["q"])
        g.text((PX, PY + PH + 14), f"t {f['t'] / 1e9:6.2f} s virtual   X {x:7.2f}  Y {y:7.2f}  Z {z:5.2f} mm   ({speed:g}x)",
               font=small, fill=FG)
        g.text((PX, PY + PH + 34), f"path error now {now if ext[k - 1] else 0:5.0f} um   worst so far {worst:5.0f} um", font=small, fill=FG)
        g.rectangle([0, H - 50, W, H], fill=BG)
        g.text((16, H - 34), foot, font=small, fill=DIM)
        ff.stdin.write(fr.tobytes())
    for _ in range(round(60 * speed)):  # hold the parked last frame 2 s
        ff.stdin.write(fr.tobytes())
    ff.stdin.close()
    ff.wait()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], *(float(a) for a in sys.argv[4:5]))
