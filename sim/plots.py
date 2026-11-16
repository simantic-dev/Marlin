"""Plots from a run's own output. Usage: python plots.py OUT_DIR SCREENSHOT_DIR
Reads pins.jsonl (sim --trace-out: every GPIO pad edge), console.txt and thermal.csv written by the peers.
The toolpath is rebuilt from STEP/DIR pad edges alone (steps.csv, which machine_peer.py wrote while running,
is only used to cross-check it)."""
import csv, json, os, re, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT, SHOT = sys.argv[1], sys.argv[2]
os.makedirs(SHOT, exist_ok=True)
SPM = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 500.0}
START = {"X": 20.0, "Y": 25.0, "Z": 10.0, "E": 0.0}
STEP = {("gpioPortF", 13): "X", ("gpioPortG", 0): "Y", ("gpioPortF", 11): "Z", ("gpioPortF", 9): "E"}
DIR = {("gpioPortF", 12): "X", ("gpioPortG", 1): "Y", ("gpioPortG", 3): "Z", ("gpioPortF", 10): "E"}
STOP = {("gpioPortG", 6): "X", ("gpioPortG", 9): "Y", ("gpioPortG", 10): "Z"}
HEATER = ("gpioPortA", 2)

POSLVL = {"X": 1, "Y": 0, "Z": 1, "E": 1}  # INVERT_Y_DIR is true in stock Marlin
pos, d = dict(START), {a: 0 for a in "XYZE"}
track, stops, heater = [], {a: [] for a in "XYZ"}, []
nstep = {a: 0 for a in "XYZE"}
for line in open(f"{OUT}/pins.jsonl"):
    e = json.loads(line)
    k = (e["port"], e["pin"])
    t, lv = e["t"], e["level"]
    if k in DIR:
        d[DIR[k]] = lv
    elif k in STEP and lv:
        a = STEP[k]
        s = 1 if d[a] == POSLVL[a] else -1
        if a != "E" and pos[a] <= 0 and s < 0:
            continue  # at the hard stop
        pos[a] = max(0.0, pos[a] + s / SPM[a]) if a != "E" else pos[a] + s / SPM[a]
        nstep[a] += 1
        track.append((t, pos["X"], pos["Y"], pos["Z"], pos["E"], a, s))
    elif k in STOP:
        stops[STOP[k]].append((t, lv))
    elif k == HEATER:
        heater.append((t, lv))
print("steps from pad edges:", nstep, "final", {a: round(v, 3) for a, v in pos.items()})

# cross-check against the peer's own count
try:
    rows = list(csv.DictReader(open(f"{OUT}/steps.csv")))
    last = rows[-1]
    print("peer count final:", last["x_mm"], last["y_mm"], last["z_mm"], last["e_mm"], "rows", len(rows))
except Exception as ex:
    print("no steps.csv", ex)

# console / sent
con = [(int(l.split(" ", 1)[0]), l.split(" ", 1)[1].rstrip("\n")) for l in open(f"{OUT}/console.txt") if " " in l]
sent = [(int(l.split(" ", 1)[0]), l.split(" ", 1)[1].rstrip("\n")) for l in open(f"{OUT}/sent.txt")]
sent_t = {c: t for t, c in reversed(sent)}


def when(cmd, which=0):
    m = [t for t, c in sent if c.startswith(cmd)]
    return m[which] / 1e6 if len(m) > which else None


# ---- toolpath
xs = [(r[1], r[2], r[4], r[0]) for r in track if r[5] in "XY"]
after = when("G1 Z0.3")
fig, ax = plt.subplots(figsize=(6.4, 6.4), dpi=130)
if after:
    prev = None
    xy = [r for r in track if r[0] >= after and r[5] in "XY"]
    for i, r in enumerate(xy):
        t, x, y, e = r[0], r[1], r[2], r[4]
        if prev:
            ext = e > xy[max(0, i - 40)][4] + 1e-9  # E advanced within the last 40 XY steps
            ax.plot([prev[1], x], [prev[2], y], color="#c0392b" if ext else "#7f8c8d", lw=1.6 if ext else 0.8)
        prev = (t, x, y, e)
ax.plot([30, 70, 70, 30, 30], [30, 30, 70, 70, 30], "k:", lw=0.8, label="G-code square (30..70 mm)")
ax.plot([], [], color="#c0392b", label="extruding (E advancing)")
ax.plot([], [], color="#7f8c8d", label="travel")
ax.set_aspect("equal"); ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)")
ax.set_title(f"Toolpath rebuilt from STEP/DIR pad edges only\n{sum(nstep.values())} steps, X/Y 80 steps/mm, E 500 steps/mm", fontsize=10)
ax.legend(loc="lower right", fontsize=8); ax.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(f"{SHOT}/toolpath.png"); plt.close(fig)

# ---- homing timeline
g0 = when("G28"); g1 = None
for t, l in con:
    if g0 and t / 1e6 > g0 and l.startswith("ok") and "X:" not in l:
        pass
m114 = when("M114")
t0, t1 = g0, m114
fig, axs = plt.subplots(3, 1, figsize=(8, 6.4), dpi=130, sharex=True)
for ax, a in zip(axs, "XYZ"):
    seg = [(r[0] / 1, r["XYZ".index(a) + 1]) for r in track if r[0] / 1 >= t0 - 0.05 and r[0] / 1 <= t1 + 0.05 and True]
    pts = [(r[0] / 1, r[1 + "XYZ".index(a)]) for r in track if t0 - 0.05 <= r[0] / 1 <= t1 + 0.05]
    if pts:
        ax.step([p[0] for p in pts], [p[1] for p in pts], where="post", color="#2c3e50", lw=1.2)
    ax.set_ylabel(f"{a} (mm)")
    ax2 = ax.twinx()
    ev = [(t, l) for t, l in stops[a] if t0 - 0.05 <= t <= t1 + 0.05]
    prevl = [l for t, l in stops[a] if t < t0 - 0.05]
    xsx = [t0 - 0.05] + [t for t, _ in ev] + [t1 + 0.05]
    ysx = [prevl[-1] if prevl else 0] + [l for _, l in ev]
    ysx.append(ysx[-1])
    ax2.step(xsx, ysx, where="post", color="#c0392b", lw=1.2)
    ax2.set_ylim(-0.1, 1.4); ax2.set_ylabel(f"{a}-MIN pad", color="#c0392b"); ax2.set_yticks([0, 1])
    ax.grid(alpha=0.3)
axs[-1].set_xlabel("guest time (s)")
axs[0].set_title("G28 against scripted endstop switches: axis position (steps counted from STEP/DIR) and the endstop pad level", fontsize=9)
fig.tight_layout(); fig.savefig(f"{SHOT}/homing.png"); plt.close(fig)

# ---- temperature
th = list(csv.DictReader(open(f"{OUT}/thermal.csv")))
tt = [float(r["t_s"]) for r in th]; tp = [float(r["temp_c"]) for r in th]
rep = []
tgt = []
for t, l in con:
    m = re.search(r"T:([\d.]+) /([\d.]+)(?: B:[\d.]+ /[\d.]+)? @:(\d+)", l) or re.search(r"T:([\d.]+) /([\d.]+) .*?@:(\d+)", l)
    if m:
        rep.append((t / 1e6, float(m.group(1)), float(m.group(2)), int(m.group(3))))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6.4), dpi=130, sharex=True, gridspec_kw={"height_ratios": [2, 1]})
a1.plot(tt, tp, color="#2c3e50", lw=1.4, label="plant temperature (first-order model)")
a1.plot([r[0] for r in rep], [r[1] for r in rep], ".", color="#e67e22", ms=3, label="firmware-reported T (M105/M155, via ADC3 PF4)")
a1.step([r[0] for r in rep], [r[2] for r in rep], where="post", color="#c0392b", lw=1, label="target")
a1.set_ylabel("hotend (C)"); a1.legend(fontsize=8, loc="center right"); a1.grid(alpha=0.3)
a1.set_title("Heater PID on Marlin vs. a lumped thermal plant (MODELED SIMPLIFICATION)", fontsize=10)
# heater duty: 0.5 s bins of the HEATER_0 pad
bins = {}
lastt, lastl = 0.0, 0
for t, l in heater + [(tt[-1] if tt else 0, 0)]:
    t0b = lastt
    while t0b < t:
        b = int(t0b / 0.5)
        end = min(t, (b + 1) * 0.5)
        bins[b] = bins.get(b, 0.0) + lastl * (end - t0b)
        t0b = end
    lastt, lastl = t, l
a2.step([b * 0.5 for b in sorted(bins)], [bins[b] / 0.5 for b in sorted(bins)], where="post", color="#16a085", label="HEATER_0 pad duty (PA2, 0.5 s mean)")
a2.plot([r[0] for r in rep], [r[3] / 127 for r in rep], ".", color="#8e44ad", ms=3, label="firmware @: PWM / 127")
a2.set_ylabel("duty"); a2.set_xlabel("guest time (s)"); a2.legend(fontsize=8); a2.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(f"{SHOT}/temperature.png"); plt.close(fig)

# ---- console transcript
sel = []
merged = sorted([(t, "<<" if False else ">>", l) for t, l in con] + [(t, "<-", l) for t, l in sent])
head = [x for x in merged if x[0] < 2_000_000 and not x[2].startswith("echo:  ")][:14]
body = [x for x in merged if x[0] >= 2_000_000 and not re.match(r"ok T:|T:|Cap:|ok$", x[2].strip())][:26]
lines = [f"{t / 1e6:9.3f}s {'host>' if k == '<-' else 'fw  <'} {l}" for t, k, l in head + body]
fig = plt.figure(figsize=(9.5, 0.22 * len(lines) + 0.7), dpi=130, facecolor="#0b0f14")
fig.text(0.01, 0.99, "USART1 console, host peer <-> Marlin (guest time)", color="#9fb3c8", va="top", fontsize=8, family="monospace")
for i, ln in enumerate(lines):
    fig.text(0.01, 0.95 - i * (0.9 / len(lines)), ln[:120], color="#d8e1ea", va="top", fontsize=7.2, family="monospace")
fig.savefig(f"{SHOT}/console.png", facecolor=fig.get_facecolor()); plt.close(fig)
print("done")
