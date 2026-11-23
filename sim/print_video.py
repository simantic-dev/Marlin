"""Headline video of the normal run, from that run's own logs: console (USART1) in guest time, toolpath rebuilt from
STEP/DIR pad edges (sim --trace-out), hotend temperature and target. Usage (needs ffmpeg):
  python print_video.py OUT_DIR SCREENSHOT_DIR      -> print.mp4 + print.gif"""
import bisect, csv, json, re, subprocess, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.collections import LineCollection

OUT, SHOT = sys.argv[1], sys.argv[2]
SPM = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 500.0}            # as plots.py / printer.py
STEP = {("gpioPortF", 13): "X", ("gpioPortG", 0): "Y", ("gpioPortF", 11): "Z", ("gpioPortF", 9): "E"}
DIR = {("gpioPortF", 12): "X", ("gpioPortG", 1): "Y", ("gpioPortG", 3): "Z", ("gpioPortF", 10): "E"}
POSLVL = {"X": 1, "Y": 0, "Z": 1, "E": 1}
pos, d = {"X": 20.0, "Y": 25.0, "Z": 10.0, "E": 0.0}, {a: 0 for a in "XYZE"}
track = [(0.0, dict(pos))]
for line in open(f"{OUT}/pins.jsonl"):
    e = json.loads(line)
    k = (e["port"], e["pin"])
    if k in DIR:
        d[DIR[k]] = e["level"]
    elif k in STEP and e["level"]:
        a = STEP[k]
        s = 1 if d[a] == POSLVL[a] else -1
        pos[a] = pos[a] + s / SPM[a] if a == "E" else max(0.0, pos[a] + s / SPM[a])
        track.append((e["t"], dict(pos)))
tt = [t for t, _ in track]

con = [(int(t) / 1e6, l.rstrip("\n")) for t, l in (x.split(" ", 1) for x in open(f"{OUT}/console.txt") if " " in x)]
sent = [(int(t) / 1e6, l.rstrip("\n")) for t, l in (x.split(" ", 1) for x in open(f"{OUT}/sent.txt"))]
rep = [(t, float(m[1]), float(m[2])) for t, l in con if (m := re.search(r"T:([\d.]+) /([\d.]+)", l))]
plant = [(float(r["t_s"]), float(r["temp_c"])) for r in csv.DictReader(open(f"{OUT}/thermal.csv"))]
lines = sorted([(t, "fw  " + l) for t, l in con if l.strip() and not re.match(r"\s*T:|echo:  |echo:busy", l)]
               + [(t, "host> " + l) for t, l in sent])
end = [t for t, l in con if l.startswith("X:")][-1] + 0.5      # the final M114 reply
print_t = next(t for t, l in sent if l.startswith("G1 Z0.3"))
homing = next(t for t, l in sent if l.startswith("G28"))

# toolpath segments after the first layer move: red while E advanced, grey travel
segs, cols, segt = [], [], []
i0 = bisect.bisect_left(tt, print_t)
for i in range(max(1, i0), len(track)):
    (_, p0), (t1, p1) = track[i - 1], track[i]
    if (p0["X"], p0["Y"]) != (p1["X"], p1["Y"]) or p1["E"] != p0["E"]:
        j = max(i0, i - 40)
        segs.append([(p0["X"], p0["Y"]), (p1["X"], p1["Y"])])
        cols.append("#e74c3c" if p1["E"] > track[j][1]["E"] + 1e-9 else "#95a5a6")
        segt.append(t1)

# guest time per frame: slow for homing and printing, fast through the heat-up wait
FPS = 10
times, t = [], 0.0
while t < end:
    times.append(t)
    t += (0.6 if t >= print_t - 1 else 1.5 if homing <= t < homing + 13 else 10.0) / FPS
times += [end] * (3 * FPS)

fig = plt.figure(figsize=(12, 6.75), dpi=75, facecolor="#111")
fig.text(.5, .95, "Stock Marlin on a simulated BTT Octopus (STM32F446ZE)", color="w", ha="center", fontsize=18, weight="bold")
fig.text(.5, .02, "MCU + Marlin bugfix-2.1.x simulated. Modelled: hotend (first-order thermal model, not measured), "
         "ideal steppers, scripted endstops", color="#aaa", ha="center", fontsize=10)
cax = fig.add_axes([.01, .07, .40, .84], facecolor="#000"); cax.axis("off")
txt = cax.text(0, 1, "", va="top", family="monospace", fontsize=8, color="#9f9")
ax = fig.add_axes([.47, .12, .24, .76], facecolor="#1b1b1b")
tax = fig.add_axes([.77, .12, .21, .76], facecolor="#1b1b1b")
for a in (ax, tax):
    a.tick_params(colors="#ccc"); a.xaxis.label.set_color("#ccc"); a.yaxis.label.set_color("#ccc")
ax.set(xlim=(-2, 80), ylim=(-2, 80), aspect="equal", xlabel="X (mm)", ylabel="Y (mm)")
ax.set_title("toolpath from STEP/DIR edges", color="#ccc", fontsize=10)
lc = LineCollection([], lw=2); ax.add_collection(lc)
head, = ax.plot([], [], "o", color="#f1c40f", ms=6)
postxt = ax.text(.02, .97, "", transform=ax.transAxes, color="#f1c40f", family="monospace", fontsize=8, va="top")
tax.set(xlim=(0, end), ylim=(0, 220), xlabel="guest time (s)", ylabel="hotend (C)")
tax.set_title("hotend", color="#ccc", fontsize=10)
pl, = tax.plot([], [], color="#5dade2", lw=1.5, label="plant (model)")
fwt, = tax.plot([], [], ".", color="#e67e22", ms=3, label="Marlin reads")
tg, = tax.step([], [], where="post", color="#e74c3c", lw=1, label="target")
tax.legend(fontsize=7, loc="lower right")
clock = tax.text(.03, .95, "", transform=tax.transAxes, color="w", fontsize=10, va="top")


def frame(i):
    now = times[i]
    txt.set_text("\n".join(f"{t:7.2f} {l[:46]}" for t, l in [x for x in lines if x[0] <= now][-38:]))
    p = track[max(0, bisect.bisect_right(tt, now) - 1)][1]
    n = bisect.bisect_right(segt, now)
    lc.set_segments(segs[:n]); lc.set_color(cols[:n])
    head.set_data([p["X"]], [p["Y"]])
    postxt.set_text(f"X{p['X']:6.2f} Y{p['Y']:6.2f} Z{p['Z']:5.2f} E{p['E']:5.2f}")
    q = [x for x in plant if x[0] <= now]; r = [x for x in rep if x[0] <= now]
    pl.set_data([x[0] for x in q], [x[1] for x in q])
    fwt.set_data([x[0] for x in r], [x[1] for x in r]); tg.set_data([x[0] for x in r], [x[2] for x in r])
    clock.set_text(f"t = {now:6.1f} s")


FuncAnimation(fig, frame, frames=len(times)).save(f"{SHOT}/print.mp4", writer=FFMpegWriter(fps=FPS, bitrate=1500))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", f"{SHOT}/print.mp4", "-vf",
                "fps=8,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse",
                f"{SHOT}/print.gif"], check=True)
print(f"{len(times)} frames, end {end:.2f} s, {len(segs)} toolpath segments")
