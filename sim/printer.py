"""State shared by the peers (they run in one interpreter, as in efs-zeropilot-4.0/sim/zp_plant.py).

MODELED SIMPLIFICATION (everything in this file is a stand-in for hardware that is not on the bench):
  * hotend: first-order lumped thermal model  C dT/dt = P*duty - (T - T_amb)/R_th
  * stepper axes: ideal, no lost steps, with a hard stop at 0 mm
Parameters (typical E3D-V6-class hotend, NOT measured on a printer; see README):
  P = 40 W      the nominal rating of the usual 24 V cartridge heater
  C = 12 J/K    ~13 g aluminium heater block x 0.9 J/(g K), rounded; the nozzle and heatbreak are ignored
  R_th = 6.9 K/W so that 100 % duty settles at T_amb + 40 W * 6.9 K/W = 301 C (typical hotend max ~300 C)
  T_amb = 25 C
Thermistor: Marlin's own table 1 (100 kOhm NTC, beta 4092 K, 4.7 kOhm pull-up, Marlin
Marlin/src/module/thermistor/thermistor_1.h), inverted, so the firmware reads back exactly the temperature the plant has.
"""
import os
import re

OUT = os.environ.get("SIM_OUT", "out")
os.makedirs(OUT, exist_ok=True)
HERE = os.path.dirname(os.path.abspath(__file__))

P_W, C_JK, R_KW, T_AMB = 40.0, 12.0, 6.9, 25.0
VREF = 3.3
STEPS_PER_MM = {"X": 80.0, "Y": 80.0, "Z": 400.0, "E": 500.0}  # DEFAULT_AXIS_STEPS_PER_UNIT in Configuration.h
# DIR pin level that means "positive": high unless Configuration.h has INVERT_<axis>_DIR true (INVERT_Y_DIR is true in stock Marlin).
DIR_POSITIVE_LEVEL = {"X": 1, "Y": 0, "Z": 1, "E": 1}
START_MM = {"X": 20.0, "Y": 25.0, "Z": 10.0, "E": 0.0}        # where the carriages are parked at power-on (arbitrary)

pos = dict(START_MM)
steps = {a: 0 for a in "XYZE"}       # signed step count since power-on (E is not hard-stopped)
temp = T_AMB
heater = {"level": 0, "t": 0, "acc": 0}


def heater_edge(t_us, level):
    heater["acc"] += heater["level"] * (t_us - heater["t"])
    heater["t"], heater["level"] = t_us, int(level)


def heater_take_duty(t_us, since_us):
    """Mean heater pin level over (since_us, t_us]."""
    heater_edge(t_us, heater["level"])
    d = heater["acc"] / max(1, t_us - since_us)
    heater["acc"] = 0
    return d


def _table():
    text = open(os.path.join(HERE, "..", "Marlin/src/module/thermistor/thermistor_1.h")).read()
    rows = [(int(a), int(b)) for a, b in re.findall(r"OV\(\s*(\d+)\)\s*,\s*(-?\d+)\s*\}", text)]
    return sorted(rows)  # raw (10-bit scale) ascending, temperature descending


TABLE = _table()


def volts_for(temp_c):
    """Pin voltage that Marlin's table 1 decodes as temp_c."""
    for (r0, t0), (r1, t1) in zip(TABLE, TABLE[1:]):
        if t1 <= temp_c <= t0:
            raw = r0 + (r1 - r0) * (t0 - temp_c) / (t0 - t1)
            return raw / 1024.0 * VREF
    return (TABLE[0][0] if temp_c > TABLE[0][1] else TABLE[-1][0]) / 1024.0 * VREF


def thermal_step(duty, dt_s):
    global temp
    temp += (P_W * duty - (temp - T_AMB) / R_KW) / C_JK * dt_s
    return temp


if __name__ == "__main__":
    assert volts_for(200) > volts_for(250) and volts_for(25) > volts_for(200)
    t = 25.0
    for _ in range(100000):
        t = thermal_step(1.0, 0.01)
    assert abs(t - 301.0) < 1.0, t
    print("ok", round(volts_for(25), 3), round(volts_for(200), 3))
