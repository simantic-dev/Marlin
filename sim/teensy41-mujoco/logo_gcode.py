"""Simantic logo (mark + wordmark, simantic_logo.svg) -> perimeter G-code for stock Marlin.

usage: uv run --with svgelements python logo_gcode.py [width_mm] [layers] [svg] [part] > logo.gcode
part "a"/"b": only the loops left/right of X 100 (one sim run each stays under 5 min wall with the FOC drive).
G4 S5 first: the FOC X drive aligns its encoder for 4.5 s before it follows STEP/DIR.
Every closed outline in the SVG (rounded square, dot, letter outlines incl. counters) becomes one perimeter,
sampled to 0.25 mm chords, scaled to width_mm and centred on the 200 x 200 bed.
"""
import math
import sys

from svgelements import SVG, Path, Shape, Move, Close

W_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0
LAYERS = int(sys.argv[2]) if len(sys.argv) > 2 else 1
LAYER_H, CHORD, F_PRINT, F_TRAVEL = 0.3, 0.25, 2400, 6000
EF = 0.4 * LAYER_H / (math.pi * 0.875 ** 2)  # filament mm per mm of path: 0.4 mm bead, 1.75 mm filament

svg = SVG.parse(sys.argv[3] if len(sys.argv) > 3 else "simantic_logo.svg")
loops = []
for el in svg.elements():
    if not isinstance(el, Shape):
        continue
    sub = []
    for seg in Path(el).segments():
        if isinstance(seg, Move):
            if len(sub) > 2:
                loops.append(sub)
            sub = [seg.end]
        elif isinstance(seg, Close) or seg.start is None:
            continue
        else:
            n = max(1, math.ceil(seg.length(error=1e-3) / 1.0))  # svg units; refined after scaling below
            sub += [seg.point(i / n) for i in range(1, n + 1)]
    if len(sub) > 2:
        loops.append(sub)

xs = [p[0] for l in loops for p in l]
ys = [p[1] for l in loops for p in l]
s = W_MM / (max(xs) - min(xs))
cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
mm = [[(100 + (p[0] - cx) * s, 100 - (p[1] - cy) * s) for p in l] for l in loops]  # SVG y is down

def resample(l):
    out = [l[0]]
    for p in l[1:] + [l[0]]:
        if math.dist(out[-1], p) >= CHORD:
            out.append(p)
    return out + [l[0]]

mm = [resample(l) for l in mm]
PART = sys.argv[4] if len(sys.argv) > 4 else ""
if PART:
    mm = [l for l in mm if (sum(p[0] for p in l) / len(l) < 100) == (PART == "a")]
g, e = ["G4 S5", "G28", "M302 P1", "G92 E0", "G1 Z5 F600"], 0.0
for layer in range(LAYERS):
    g.append(f"G1 Z{LAYER_H * (layer + 1):.2f} F600")
    for l in mm:
        g.append(f"G1 X{l[0][0]:.3f} Y{l[0][1]:.3f} F{F_TRAVEL}")
        g.append(f"G1 F{F_PRINT}")
        for a, b in zip(l, l[1:]):
            e += math.dist(a, b) * EF
            g.append(f"G1 X{b[0]:.3f} Y{b[1]:.3f} E{e:.4f}")
g += ["G1 Z5 F600", "M400", "M114"]
print("\n".join(g))
path = sum(math.dist(a, b) for l in mm for a, b in zip(l, l[1:])) * LAYERS
print(f"; {len(mm)} loops, {path:.0f} mm printed, {len(g)} lines", file=sys.stderr)
