"""Accuracy graphs from run logs: commanded G-code path vs the MuJoCo-printed path.

usage: python accuracy.py logo.gcode <run dir> <label> [<run dir> <label>] 
A run dir holds trace.txt from printer_plant.py (1 kHz: t_ns qx qy qz tx ty e e_dep flow; mm, tx/ty = Marlin's
step-counted position). Error = distance from each printed sample (while E advances) to the nearest extruding
G-code segment. Writes graphs/*.png and prints the table.
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLORS = ["tab:red", "tab:blue"]


def gcode_segments(path):
    segs, x, y = [], None, None
    for l in open(path):
        if not l.startswith("G1 "):
            continue
        p = {t[0]: float(t[1:]) for t in l.split()[1:] if t[0] in "XYE"}
        nx, ny = p.get("X", x), p.get("Y", y)
        if "E" in p and x is not None:
            segs.append((x, y, nx, ny))
        x, y = nx, ny
    return np.array(segs)


def nearest(pts, segs):
    a, b = segs[:, :2], segs[:, 2:]
    v = b - a
    vv = np.maximum((v * v).sum(1), 1e-12)
    out = np.empty(len(pts))
    for i in range(0, len(pts), 2000):
        p = pts[i:i + 2000, None, :]
        t = np.clip(((p - a) * v).sum(2) / vv, 0, 1)
        out[i:i + 2000] = np.linalg.norm(p - a - t[..., None] * v, axis=2).min(1)
    return out


def load(d, segs):
    r = np.loadtxt(os.path.join(d, "trace.txt"))
    # tx/ty count steps from the carriage's power-on position; Marlin's frame starts where G28 found the switch.
    # Shift them onto Marlin's frame with its final M114 count (the axes are at rest there).
    m114 = [l for l in open(os.path.join(d, "sim.txt"), errors="replace") if "Count X:" in l][-1]
    cx, cy = (int(m114.split(f"{a}:")[-1].split()[0]) / 80 for a in ("Count X", "Y"))
    r[:, 4] += cx - r[-1, 4]
    r[:, 5] += cy - r[-1, 5]
    t = r[:, 0] / 1e9
    ext = np.r_[False, np.diff(r[:, 6]) > 1e-5]
    err = nearest(r[ext][:, 1:3], segs) * 1000
    spd = np.r_[0, np.hypot(np.diff(r[:, 4]), np.diff(r[:, 5])) / np.diff(t)]
    return dict(t=t, r=r, ext=ext, err=err, spd=spd)


def main(gc, runs):
    segs = gcode_segments(gc)
    os.makedirs("graphs", exist_ok=True)
    R = [(lab, load(d, segs)) for d, lab in runs]
    print("| drive | samples | RMS um | p95 um | max um | X follow max um | Y follow max um |\n|---|---|---|---|---|---|---|")
    for lab, s in R:
        e, r, x = s["err"], s["r"], s["ext"]
        fx, fy = (np.abs(r[x, 4 + i] - r[x, 1 + i]).max() * 1000 for i in (0, 1))
        s["stats"] = (math.sqrt((e ** 2).mean()), np.percentile(e, 95), e.max())
        print(f"| {lab} | {len(e)} | {s['stats'][0]:.0f} | {s['stats'][1]:.0f} | {s['stats'][2]:.0f} | {fx:.0f} | {fy:.0f} |")

    # 1. overlay + zoom on the worst point of the first run
    fig, axs = plt.subplots(1, 2, figsize=(15, 4.6), gridspec_kw={"width_ratios": [3.2, 1]})
    lab0, s0 = R[0]
    w = np.argmax(s0["err"])
    wx, wy = s0["r"][s0["ext"]][w, 1:3]
    for ax in axs:
        for sg in segs:
            ax.plot(sg[[0, 2]], sg[[1, 3]], color="0.6", lw=1.2, zorder=1)
        for (lab, s), c in zip(R, COLORS):
            p = s["r"][s["ext"]]
            ax.plot(p[:, 1], p[:, 2], ".", ms=1.2, color=c, label=f"printed ({lab})", zorder=2)
        ax.set_aspect("equal")
    axs[0].plot([], [], color="0.6", label="G-code (commanded)")
    axs[0].legend(loc="upper right", markerscale=6, fontsize=8)
    axs[0].add_patch(plt.Rectangle((wx - 1.5, wy - 1.5), 3, 3, fill=False, color="k"))
    axs[0].set_title("Commanded vs printed path, whole print (mm)")
    axs[1].set_xlim(wx - 1.5, wx + 1.5); axs[1].set_ylim(wy - 1.5, wy + 1.5)
    axs[1].set_title(f"zoom: worst point {s0['err'][w]:.0f} um\n({lab0})", fontsize=9)
    fig.tight_layout(); fig.savefig("graphs/overlay.png", dpi=110); plt.close(fig)

    # 2. error vs time with commanded speed
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 5.5), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    for (lab, s), c in zip(R, COLORS):
        a1.plot(s["t"][s["ext"]], s["err"], ".", ms=1.5, color=c, label=lab)
    a1.set_ylabel("path error (um)"); a1.legend(markerscale=6); a1.set_title("Path error while extruding, vs virtual time")
    s = R[0][1]
    a2.plot(s["t"], s["spd"], color="0.3", lw=0.6)
    a2.set_ylabel("commanded XY speed (mm/s)\n(Marlin step position)"); a2.set_xlabel("virtual time (s)")
    lo = s["t"][s["ext"]].min() - 0.5
    a1.set_xlim(lo, s["t"][s["ext"]].max() + 0.5); a2.set_ylim(0, 120)
    fig.tight_layout(); fig.savefig("graphs/error_vs_time.png", dpi=110); plt.close(fig)

    # 3. CDF
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for (lab, s), c in zip(R, COLORS):
        e = np.sort(s["err"])
        ax.plot(e, np.arange(1, len(e) + 1) / len(e), color=c, label=lab)
        for v, ls, n in zip(s["stats"], (":", "--", "-"), ("RMS", "p95", "max")):
            ax.axvline(v, color=c, ls=ls, lw=0.8)
            ax.text(v, 0.02 + 0.08 * ("max", "p95", "RMS").index(n), f" {n} {v:.0f}", color=c, fontsize=8)
    ax.set_xlabel("path error (um)"); ax.set_ylabel("fraction of printed samples"); ax.legend(loc="center right")
    ax.set_title("Path error CDF"); fig.tight_layout(); fig.savefig("graphs/error_cdf.png", dpi=110); plt.close(fig)

    # 4. per-axis following error
    fig, axs = plt.subplots(2, 1, figsize=(12, 5), sharex=True)
    for i, ax in enumerate(axs):
        top = 0
        for (lab, s), c in zip(R, COLORS):
            fe = (s["r"][:, 4 + i] - s["r"][:, 1 + i]) * 1000
            ax.plot(s["t"], fe, color=c, lw=0.5, label=lab)
            top = max(top, np.abs(fe[s["t"] >= lo]).max() * 1.1)
        ax.set_ylim(-top, top)
        ax.set_ylabel(f"{'XY'[i]}: step pos - joint (um)")
    axs[0].legend(); axs[0].set_title("Per-axis following error: Marlin's step-counted position minus MuJoCo joint")
    axs[1].set_xlabel("virtual time (s)"); axs[1].set_xlim(lo, R[0][1]["t"].max())
    fig.tight_layout(); fig.savefig("graphs/following_error.png", dpi=110); plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1], list(zip(sys.argv[2::2], sys.argv[3::2])))
