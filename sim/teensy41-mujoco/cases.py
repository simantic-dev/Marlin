"""Mechanical imperfection cases: the same stock Marlin firmware and G-code, one MuJoCo plant per case.

usage: python cases.py run [case ...]     # one sim run per case, sequentially (out/<case>/)
       python cases.py report             # metrics table + out/cases.png
Each run is the real firmware closing its own loop (homing reads the carriage's MuJoCo position), so a case
changes what Marlin sees as well as what the nozzle does.
"""
import json
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "peers"))
import printer_plant as peer  # STEPPER / NOMINAL for the v5 cases

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.environ.get("SIM", "sim")
ELF = os.environ.get("MARLIN_ELF") or os.path.abspath(os.path.join(HERE, "../../.pio/build/teensy41/firmware.elf"))
THERMISTOR_H = os.environ.get("MARLIN_THERMISTOR_H") or os.path.abspath(os.path.join(os.path.dirname(ELF), "../../../Marlin/src/module/thermistor/thermistor_1.h"))
FOC_DIR = os.path.join(HERE, "foc")
FOC_ELF = os.path.join(FOC_DIR, ".pio/build/teensy41/firmware.elf")
MUJOCO_SITE = os.environ.get("MUJOCO_SITE", "")  # site-packages holding mujoco, for the plant peer inside sim

# name, what it represents, plant overrides (peers/printer_plant.py NOMINAL is the datasheet baseline)
CASES = [
    ("rigid", "near-rigid drive: the G-code geometry", {"x": {"k": 2e6, "zeta": 0.5}, "y": {"k": 2e6, "zeta": 0.5}}),
    ("nominal", "GT2 6 mm belt + NEMA17 (datasheet stiffness), zeta 0.1", {}),
    ("backlash-0.05", "XY backlash 0.05 mm (tight pulley/belt)", {"x": {"backlash_mm": 0.05}, "y": {"backlash_mm": 0.05}}),
    ("backlash-0.2", "XY backlash 0.2 mm (worn / loose set screw)", {"x": {"backlash_mm": 0.2}, "y": {"backlash_mm": 0.2}}),
    ("loose-belt", "belt at 1/4 stiffness (under-tensioned)", {"x": {"k": 27.5e3}, "y": {"k": 27.5e3}}),
    ("heavy-head", "+0.3 kg toolhead (direct-drive extruder)", {"x": {"mass_add": 0.3}}),
    ("low-damping", "zeta 0.02 (stiff rails, little friction)", {"x": {"zeta": 0.02}, "y": {"zeta": 0.02}}),
    ("z-backlash", "Z nut backlash 0.1 mm", {"z": {"backlash_mm": 0.1}}),
    ("torque-starved", "motor force capped at 3 N (no step loss modelled)", {"x": {"fmax": 3.0}, "y": {"fmax": 3.0}}),
]
# v5: extruder melt chamber and stepper non-idealities (peers/printer_plant.py "e", STEPPER). The stock ELF has LIN_ADVANCE off
# (Configuration_adv.h:2499), so there is no M900 case: Marlin cannot pre-compensate the melt lag seen here.
V5_CASES = [("realistic", "direct-drive melt lag (tau 0.05 s) + NEMA17 microstep error, detent, 24 V rolloff",
             {"x": peer.STEPPER, "y": peer.STEPPER, "e": {"tau_s": 0.05}}),
            ("realistic-bowden", "as realistic, Bowden melt lag (tau 0.5 s)",
             {"x": peer.STEPPER, "y": peer.STEPPER, "e": {"tau_s": 0.5}})]
# X on a SimpleFOC servo (Nanotec DB42S02 BLDC + MT6835) running on a second Teensy 4.1, fed Marlin's X STEP/DIR.
FOC_CASES = [("bldc-foc", "X: BLDC + FOC servo on a 2nd Teensy (SimpleFOC), belt only", {}),
             ("bldc-foc-ff-raw", "X: + velocity feedforward from the STEP rate", {}),
             ("bldc-foc-ff", "X: + feedforward, velocity clamp 200 rad/s, no output ramp", {}),
             ("bldc-foc-ff-fast", "X: + unfiltered step-rate estimate", {})]
FOC_ENV = {"bldc-foc": "teensy41", "bldc-foc-ff-raw": "teensy41_ff_raw", "bldc-foc-ff": "teensy41_ff",
           "bldc-foc-ff-fast": "teensy41_ff_fast"}  # foc/platformio.ini env per case

# Teensy firmware uses the fast GPIO mirrors: Marlin X STEP = pin 0 (GPIO6.03), X DIR = pin 1 (GPIO6.02); the drive takes STEP on pin 2 (GPIO9.04),
# DIR on pin 3 (GPIO9.05).
FOC_NETS = """nets:
  - [dut.gpio6.3, xdrive.gpio9.4]
  - [dut.gpio6.2, xdrive.gpio9.5]
"""

DRIVE_BOARD = """// SimpleFOC X drive (foc/PINS.md): plant reads FlexPWM2, MT6835 on LPSPI4 behind CS pin 10.
plant: Scripted.ScriptedPeripheral @ sysbus 0xA0000000
    size: 0x4
    file: "{d}/plant/foc_plant.py"

mux: SPI.SPIMultiplexer @ lpspi4
    ActiveLowMask: 0x1

encoder: SPI.ScriptedSpiSlave @ mux 0
    file: "{d}/plant/mt6835_peer.py"

gpio2:
    0 -> mux@0
gpio7:
    0 -> mux@0
"""

# 10 mm square, 3 layers of 0.3 mm, two perimeters; layer 3 at 100 mm/s to excite ringing.
LAYER_H, SQUARE = 0.3, [(95.0, 105.0), (95.45, 104.55)]
SPEEDS = [2400, 2400, 6000]
EF = 0.4 * LAYER_H / (math.pi * 0.875 ** 2)  # filament mm per mm of path: 0.4 mm bead, 1.75 mm filament


HEAT = ["M140 S60", "M104 S210", "M190 S60", "M109 S210"]


def gcode(heat=False):
    # G4 S5: the FOC drive's alignment takes 4.5 s; every case waits the same so all runs share one timeline.
    L, e = ["G4 S5", *(HEAT if heat else []), "G28", *([] if heat else ["M302 P1"]), "G92 E0", "G1 Z5 F600", "G1 X95 Y95 F6000"], 0.0
    for layer, f in enumerate(SPEEDS):
        L.append(f"G1 Z{LAYER_H * (layer + 1):.2f} F600")
        for a, b in SQUARE:
            pts = [(a, a), (b, a), (b, b), (a, b), (a, a)]
            L.append(f"G1 X{a:.2f} Y{a:.2f} F6000")
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                e += math.hypot(x1 - x0, y1 - y0) * EF
                L.append(f"G1 X{x1:.2f} Y{y1:.2f} E{e:.4f} F{f}")
    L += ["G1 Z5 F600", "M400", "M114"]
    return "\n".join(L) + "\n"


DRIVE_BOARD_Y = """
encoder_y: SPI.ScriptedSpiSlave @ mux 1
    file: "{d}/plant/mt6835_peer.py"
    config: "{{\\"axis\\": \\"y\\"}}"

gpio2:
    11 -> mux@1
gpio7:
    11 -> mux@1
"""
# Marlin Y STEP/DIR (dut fast GPIO9.04/9.05) -> drive pins 20/21 (fast GPIO6.26/6.27)
FOC_NETS_Y = """  - [dut.gpio9.4, xdrive.gpio6.26]
  - [dut.gpio9.5, xdrive.gpio6.27]
"""


def run(name, phys, foc=False, thermal=None, gc=None, timeout=24, mj=True, axes="x"):
    d = os.path.join(HERE, "out", name)
    os.makedirs(d, exist_ok=True)
    cfg = {"mjcf": os.path.join(HERE, "printer.xml"), "phys": phys,
           "trace": os.path.join(d, "trace.txt"), "frames": os.path.join(d, "frames.jsonl")} if mj else {}
    if foc:
        cfg.update(x_drive={"pmsm_dir": os.path.join(FOC_DIR, "plant"), "axes": axes}, phys_us=40)
    if thermal is not None:
        cfg["thermal"] = {"table": THERMISTOR_H, "trace": os.path.join(d, "thermal.txt"), **thermal}
    board = open(os.path.join(HERE, "printer.board")).read()
    if thermal is not None:  # heater pins 37/36 = GPIO7 bits 19/18, fan pin 29 = GPIO9 bit 31 (core_pins.h, Teensy 4.1)
        board += "gpio7: 19 -> printer@8\ngpio7: 18 -> printer@9\ngpio9: 31 -> printer@10\n"
    board = board.replace('file: "peers/printer_plant.py"', f'file: "{os.path.join(HERE, "peers/printer_plant.py")}"')
    board = "\n".join(f"    config: {json.dumps(json.dumps(cfg))}" if l.startswith("    config:") else l
                      for l in board.splitlines()) + "\n"
    if thermal is not None:  # peers/thermal_peer.py: thermistor volts onto the ADC
        board += (f'\nthermal: Scripted.ScriptedPeripheral @ sysbus 0xA0001000\n    size: 0x4\n'
                  f'    file: "{os.path.join(HERE, "peers/thermal_peer.py")}"\n')
    open(os.path.join(d, "printer.board"), "w").write(board)
    scen = open(os.path.join(HERE, "scenario.yaml")).read()
    scen = scen.replace("../../../marlin-t41/.pio/build/teensy41/firmware.elf", ELF)
    if thermal is not None:  # the peer drives A1/A2 itself; fixed points would overwrite it every slice
        scen = scen.replace("    analog:\n", "").replace("      - {adc: adc1, pin: A2, points: [[0, 3.152]]}\n", "").replace("      - {adc: adc1, pin: A1, points: [[0, 3.152]]}\n", "")
    if foc:
        drive = DRIVE_BOARD.format(d=FOC_DIR)
        if axes == "xy":
            drive = drive.replace("ActiveLowMask: 0x1", "ActiveLowMask: 0x3").replace(
                '    size: 0x4\n', '    size: 0x4\n    config: "{\\"axes\\": \\"xy\\"}"\n', 1) + DRIVE_BOARD_Y.format(d=FOC_DIR)
        open(os.path.join(d, "drive.board"), "w").write(drive)
        scen += f"  xdrive:\n    mcu: Teensy41\n    elf: {FOC_ELF.replace('teensy41/', FOC_ENV[name] + '/')}\n    overlay: drive.board\n" + FOC_NETS + (FOC_NETS_Y if axes == "xy" else "")
    open(os.path.join(d, "scenario.yaml"), "w").write(scen)
    env = {**os.environ, "PYTHONPATH": MUJOCO_SITE}
    with open(os.path.join(d, "sim.txt"), "w") as out:
        subprocess.run([SIM, "run", "--scenario", os.path.join(d, "scenario.yaml"), "--timeout", str(timeout), "--ascii",
                        "--usb-cdc-console", "dut.usb1=" + (gc or gcode())], stdout=out, stderr=subprocess.STDOUT,
                       env=env, check=True, timeout=1800)


def seg_dist(p, a, b):
    ax, ay, bx, by = *a, *b
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(p[0] - ax - t * vx, p[1] - ay - t * vy), (ax + t * vx, ay + t * vy)


SEGS = [(p, q) for a, b in SQUARE for p, q in zip(*(lambda s: (s, s[1:] + s[:1]))([(a, a), (b, a), (b, b), (a, b)]))]


def metrics(name):
    """Printed-path error over the samples where Marlin commands extrusion (as before v5), plus extrusion error. Each sample carries
    its local flow ratio w: filament deposited (trace column 7, == commanded E for an ideal extruder) per mm of path over the last
    20 ms, over EF. ooze_mm is filament that leaves the nozzle after Marlin stopped extruding (travel moves, Z lifts)."""
    rows = [list(map(float, l.split())) for l in open(os.path.join(HERE, "out", name, "trace.txt"))]
    s_cum = [0.0]
    for p, r in zip(rows, rows[1:]):
        s_cum.append(s_cum[-1] + math.hypot(r[1] - p[1], r[2] - p[2]))
    pts, dev, flow, ooze = [], [], [], 0.0
    for i in range(1, len(rows)):
        r = rows[i]
        if r[6] <= rows[i - 1][6] + 1e-5:
            ooze += r[7] - rows[i - 1][7]
            continue
        layer = min(range(3), key=lambda l: abs(r[3] - LAYER_H * (l + 1)))
        dist, near = min(seg_dist((r[1], r[2]), a, b) for a, b in SEGS)
        ds = s_cum[i] - s_cum[max(0, i - 20)]
        w = (r[7] - rows[max(0, i - 20)][7]) / (ds * EF) if ds > 0.2 else None  # None: too slow to define a ratio
        pts.append((r[1], r[2], near, layer, 1.0 if w is None else min(3.0, w)))
        dev.append((dist, layer, r[3] - LAYER_H * (layer + 1)))
        if w is not None:
            flow.append(w - 1)
    m = {}
    for layer in range(3):
        d = [x[0] for x in dev if x[1] == layer]
        m[f"L{layer + 1}_max_um"] = max(d) * 1000
        m[f"L{layer + 1}_rms_um"] = math.sqrt(sum(x * x for x in d) / len(d)) * 1000
    m["z_err_um"] = max(abs(x[2]) for x in dev) * 1000
    m["flow_max_pct"] = max(abs(x) for x in flow) * 100  # extrusion error vs the ideal bead, worst 20 ms window
    m["flow_rms_pct"] = math.sqrt(sum(x * x for x in flow) / len(flow)) * 100
    m["ooze_mm"] = ooze
    m["e_resid_mm"] = rows[-1][6] - rows[-1][7]  # commanded minus deposited at the end: filament still in the melt chamber
    sim_txt = open(os.path.join(HERE, "out", name, "sim.txt")).read()
    m["m114"] = [l.split(") ", 1)[1] for l in sim_txt.splitlines() if "Count" in l][-1]
    return m, pts


def report():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    print(f"{'case':17s} {'L1 max':>7s} {'L2 max':>7s} {'L3 max':>7s} {'L3 rms':>7s} {'Z err':>6s} {'flow max%':>9s} {'flow rms%':>9s} {'ooze mm':>8s}  M114")
    for group, fname, shape, size in ((CASES, "cases.png", (3, 3), (13, 13)), (CASES[1:2] + V5_CASES, "v5-cases.png", (1, 3), (16, 5.8))):
        fig, axs = plt.subplots(*shape, figsize=size)
        for ax, (name, desc, _) in zip(axs.flat, group):
            m, pts = metrics(name)
            if fname == "cases.png" or name != "nominal":
                print(f"{name:17s} {m['L1_max_um']:7.1f} {m['L2_max_um']:7.1f} {m['L3_max_um']:7.1f} {m['L3_rms_um']:7.1f} "
                      f"{m['z_err_um']:6.1f} {m['flow_max_pct']:9.1f} {m['flow_rms_pct']:9.1f} {m['ooze_mm']:8.3f}  {m['m114']}")
            gain = 50  # deviation drawn 50x
            for a, b in SQUARE:
                ax.plot([a, b, b, a, a], [a, a, b, b, a], color="0.75", lw=1)
            for layer, color in enumerate(("tab:blue", "tab:green", "tab:red")):
                sel = [(n[0] + (x - n[0]) * gain, n[1] + (y - n[1]) * gain, w) for x, y, n, l, w in pts if l == layer]
                # dot area follows the local flow ratio (1 = the ideal bead): fat dots are over-extrusion
                ax.scatter([p[0] for p in sel], [p[1] for p in sel], s=[1.5 * p[2] ** 2 for p in sel], color=color, linewidths=0,
                           label=f"L{layer + 1} {'100' if SPEEDS[layer] == 6000 else '40'} mm/s")
            ax.set_title(f"{name}: {desc}\nL3 max {m['L3_max_um']:.0f} um, rms {m['L3_rms_um']:.0f} um, Z {m['z_err_um']:.0f} um, "
                         f"flow err max {m['flow_max_pct']:.0f}%", fontsize=9)
            ax.set_aspect("equal")
            ax.set_xlim(92, 108)
            ax.set_ylim(92, 108)
            ax.tick_params(labelsize=7)
        axs.flat[0].legend(fontsize=7, loc="center")
        fig.suptitle("Stock Marlin on a simulated Teensy 4.1 -> MuJoCo printer: printed path vs G-code (deviation x50, dot area = local flow)")
        fig.tight_layout()
        fig.savefig(os.path.join(HERE, "out", fname), dpi=110)


if __name__ == "__main__":
    if sys.argv[1] == "run":
        for name, desc, phys in CASES + V5_CASES + FOC_CASES:
            if len(sys.argv) == 2 or name in sys.argv[2:]:
                print("run", name, flush=True)
                run(name, phys, foc=(name, desc, phys) in FOC_CASES)
    else:
        report()
