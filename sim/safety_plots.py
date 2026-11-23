"""Plots and video for the thermal-protection scenes, from sim/safety.sh output only.
Usage: python safety_plots.py OUT_DIR SCREENSHOT_DIR [--video]   (--video needs ffmpeg; writes safety.mp4 + safety.gif)"""
import csv, os, re, subprocess, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT, SHOT = sys.argv[1], sys.argv[2]
SCENES = {"heating-failed": "heater disconnected (plant gets no power), M104 S200",
          "early-fall": "thermistor detached at 39 s, M104 S90",
          "thermal-runaway": "thermistor detached at 125 s, M104 S200"}
RED = ("Error:", "Heating Failed", "THERMAL RUNAWAY", "halted")


def load(name):
    d = f"{OUT}/{name}"
    con = [(int(t) / 1e6, l.rstrip("\n")) for t, l in (x.split(" ", 1) for x in open(f"{d}/console.txt") if " " in x)]
    rep = [(t, float(m[1]), float(m[2])) for t, l in con if (m := re.search(r"T:([\d.]+) /([\d.]+)", l))]
    halt = next(t for t, l in con if "Printer halted" in l)
    plant = [(float(r["t_s"]), float(r["temp_c"])) for r in csv.DictReader(open(f"{d}/thermal.csv"))]
    return con, rep, [p for p in plant if p[0] <= halt], halt


def draw(ax, rep, plant, now):
    p = [x for x in plant if x[0] <= now] or [(0, 25)]
    ax.plot([x[0] for x in p], [x[1] for x in p], color="#2c3e50", lw=1.4, label="plant temperature (first-order model)")
    r = [x for x in rep if x[0] <= now] or [(0, 25, 0)]
    ax.plot([x[0] for x in r], [x[1] for x in r], ".", color="#e67e22", ms=3, label="firmware-reported T (M155, via ADC3 PF4)")
    ax.step([x[0] for x in r], [x[2] for x in r], where="post", color="#c0392b", lw=1, label="target")
    ax.set_xlabel("guest time (s)"); ax.set_ylabel("hotend (C)"); ax.grid(alpha=0.3)


for name, what in SCENES.items():
    con, rep, plant, halt = load(name)
    err = next(l for t, l in con if l.startswith("Error:"))
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=130)
    draw(ax, rep, plant, halt)
    ax.axvline(halt, color="red", lw=1)
    ax.text(halt, ax.get_ylim()[1] * 0.97, f"{err.split(',')[0]}  {halt:.2f} s ", color="red", ha="right", va="top", fontsize=8)
    ax.set_title(f"{name}: {what}", fontsize=10); ax.legend(fontsize=8, loc="center left")
    fig.tight_layout(); fig.savefig(f"{SHOT}/safety-{name}.png"); plt.close(fig)
    print(f"{name}: {err} | halt {halt:.3f} s | plant {plant[-1][1]:.1f} C | last report {rep[-1][1]} C")

if "--video" in sys.argv:
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    con, rep, plant, halt = load("thermal-runaway")
    lines = [(t, l) for t, l in con if l.strip() and not l.startswith("echo:  ")]
    fps, speed, tail = 10, 8.0, 3.0  # speed: guest seconds per video second
    fig = plt.figure(figsize=(12, 6.75), dpi=80, facecolor="#111")
    fig.text(.5, .95, "Marlin bugfix-2.1.x thermal runaway on a simulated BTT Octopus (STM32F446ZE)", color="w",
             ha="center", fontsize=16, weight="bold")
    fig.text(.5, .02, "MCU and firmware simulated; the hotend is a first-order thermal model (not measured); "
             "the thermistor is detached at 125 s by the peer", color="#aaa", ha="center", fontsize=10)
    cax = fig.add_axes([.02, .08, .44, .82], facecolor="#000"); cax.axis("off")
    txt = cax.text(0, 1, "", va="top", family="monospace", fontsize=8, color="#9f9")
    red = cax.text(0, .02, "", va="bottom", family="monospace", fontsize=9, color="#ff4040", weight="bold")
    ax = fig.add_axes([.54, .14, .43, .74])

    def frame(i):
        now = min(i / fps * speed, halt)
        ax.clear(); draw(ax, rep, plant, now)
        ax.set(xlim=(0, halt + 5), ylim=(0, 320)); ax.legend(fontsize=8, loc="upper left")
        ax.tick_params(colors="#ccc"); ax.xaxis.label.set_color("#ccc"); ax.yaxis.label.set_color("#ccc")
        ax.text(.98, .03, f"t = {now:6.1f} s", transform=ax.transAxes, ha="right", fontsize=11)
        shown = [(t, l) for t, l in lines if t <= now]
        txt.set_text("\n".join([f"{t:7.2f} {l[:50]}" for t, l in shown if not any(k in l for k in RED)][-34:]))
        red.set_text("\n".join(f"{t:7.2f} {l[:48]}" for t, l in shown if any(k in l for k in RED)))
        if now >= halt:
            ax.axvline(halt, color="red")

    FuncAnimation(fig, frame, frames=int((halt / speed + tail) * fps)).save(f"{SHOT}/safety.mp4", writer=FFMpegWriter(fps=fps, bitrate=1500))
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", f"{SHOT}/safety.mp4", "-vf",
                    "fps=6,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse",
                    f"{SHOT}/safety.gif"], check=True)
    print("gif", os.path.getsize(f"{SHOT}/safety.gif") // 1024, "KiB")
