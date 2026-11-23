"""Join the two half-print runs (logo-a.gcode, logo-b.gcode) into one run dir for accuracy.py and video.py.

usage: python merge_halves.py out/<name>-a out/<name>-b out/<name>
Each half is a separate sim run from power-on (alignment, G28, its loops). The merged run keeps half a up to 0.3 s after
its last extrusion, then half b from 0.3 s before its first extrusion, shifted in time to follow on. E of half b is
offset by half a's final E. Half b's final M114 is the merged run's.
"""
import json
import os
import re
import sys

import numpy as np

PAD = 0.3e9


def main(a, b, out):
    ta, tb = (np.loadtxt(os.path.join(d, "trace.txt")) for d in (a, b))
    ext = lambda r: r[1:][np.diff(r[:, 6]) > 1e-5, 0]
    a_end, b_start = ext(ta).max() + PAD, ext(tb).min() - PAD
    shift = a_end + 1e6 - b_start
    e0, d0 = ta[ta[:, 0] <= a_end][-1, 6:8]
    tb = tb[tb[:, 0] >= b_start]
    tb[:, 0] += shift
    tb[:, 6] += e0
    tb[:, 7] += d0
    os.makedirs(out, exist_ok=True)
    np.savetxt(os.path.join(out, "trace.txt"), np.vstack([ta[ta[:, 0] <= a_end], tb]),
               fmt=["%d"] + ["%.4f"] * 5 + ["%.4f", "%.5f", "%.3f"])
    with open(os.path.join(out, "frames.jsonl"), "w") as f:
        for l in open(os.path.join(a, "frames.jsonl")):
            if json.loads(l)["t"] <= a_end:
                f.write(l)
        for l in open(os.path.join(b, "frames.jsonl")):
            fr = json.loads(l)
            if fr["t"] >= b_start:
                fr["t"] += shift
                fr["e"] += e0
                f.write(json.dumps(fr) + "\n")
    stamp = re.compile(r"^((?:\[\w+\] )?\[)(\d+\.\d+)(s\].*)")
    with open(os.path.join(out, "sim.txt"), "w") as f:
        for l in open(os.path.join(a, "sim.txt"), errors="replace"):
            m = stamp.match(l)
            if not m or float(m.group(2)) * 1e9 <= a_end:
                f.write(l)
        for l in open(os.path.join(b, "sim.txt"), errors="replace"):
            m = stamp.match(l)
            if m and float(m.group(2)) * 1e9 >= b_start:
                f.write(f"{m.group(1)}{(float(m.group(2)) * 1e9 + shift) / 1e9:.6f}{m.group(3)}\n")


if __name__ == "__main__":
    main(*sys.argv[1:4])
