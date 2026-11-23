# Cartesian printer plant for Marlin on a Teensy 4.1: STEP/DIR pins in, min endstops out.
# Edges arrive in burst mode, each stamped at its own virtual time, so step timing is the firmware's.
#
# config keys:
#   "log":    path, one "t_ns axis pos_steps" line per step
#   "mjcf":   MuJoCo scene; with it, the carriages move in MuJoCo and the endstops read the carriage position
#   "frames": path, 30 fps {"t","q","c","e"} for render.py
#   "trace":  path, 1 kHz "t_ns qx qy qz tx ty e" for metrics
#   "x_drive": {"pmsm_dir": "<dir of foc/plant/pmsm.py>"}: X is a BLDC servo on another machine; the belt couples its
#             rotor (pmsm.MOTOR) to the carriage, and the belt force goes back to the rotor as load torque
#   "phys_us": plant step (default PHYS_US)
#   "thermal": {"table": "<Marlin thermistor_1.h>", "trace": path, "fault": {"t": s, "kind": "open"|"dead"}, "scale": k}: first-order
#             hotend and bed (class Thermal). Heater pins 37/36 (inputs 8/9) and fan pin 29 (input 10) are read; the thermistor
#             volts reach the ADC by ctx.set_property("adc1", "ch12"/"ch8") (hotend A2 / bed A1) from on_timer
#   "phys":   per-axis overrides {"x": {"k", "zeta", "mass_add", "backlash_mm", "fmax", "micro_err", "detent_n", "vsup"}, "y": {...},
#             "z": {"backlash_mm"}, "e": {"tau_s"}}; trace columns 7/8 = deposited filament (mm) and its flow (mm/s)
import json
import math

AXES = "XYZE"
STEPS_PER_MM = (80, 80, 400, 500)   # Marlin DEFAULT_AXIS_STEPS_PER_UNIT
INVERT_DIR = (False, True, False, False)  # INVERT_X/Y/Z/E0_DIR
STEP_LINE = {0: 0, 1: 1, 2: 2, 6: 3}
DIR_LINE = {3: 0, 4: 1, 5: 2, 7: 3}
# A real axis starts somewhere off its switch; G28 has to find it.
START_MM = (6.0, 8.0, 2.0, 0.0)
PHYS_US = 250
FRAME_NS = 1_000_000_000 // 30
TRACE_NS = 1_000_000

# X/Y drive stiffness, motor and belt in series:
#   belt: Gates 17611-T, 2M PowerGrip GT2 modulus 18,000 lb/in/in at 1" = 80.1 kN/strain; 6 mm = x 6/25.4 x 0.82
#         (note 4, < 1/4") = 15.5 kN/strain; carriage mid-travel on a 450 mm loop, two spans in parallel:
#         4 x 15.5e3 / 0.45 = 138 kN/m
#   motor: 1.8 deg hybrid stepper, 50 rotor teeth, stiffness ~ T_hold x 50 = 0.45 N.m x 50 = 22.5 N.m/rad;
#         20T GT2 pulley r = 6.37 mm: 22.5 / r^2 = 554 kN/m
#   series: 1 / (1/138e3 + 1/554e3) = 110 kN/m
# fmax = 0.45 N.m / 6.37 mm = 70 N (holding torque; a stepper's torque falls with speed).
# MODELED SIMPLIFICATION: zeta (belt + rail damping) has no datasheet value; 0.1 is a stand-in that the cases sweep.
BELT_K = 138e3     # N/m, the belt alone (Gates 17611-T, above): a servo replaces the stepper's magnetic stiffness
R_PULLEY = 6.37e-3  # m, 20T GT2
#
# Stepper non-idealities (off in NOMINAL; the "realistic" cases turn them on). NEMA17 17HS4401-class datasheet values,
# recalled from the vendor sheet and not re-fetched: step accuracy +-5% of a full step, non-accumulative; detent torque
# 2.2 N.cm; 1.7 A/phase, 1.8 ohm, 2.8 mH. A 1.8 deg step on the 20T GT2 pulley (40 mm/rev) is 0.2 mm of belt.
FULL_STEP_M = 0.2e-3
CYCLE_M = 4 * FULL_STEP_M  # 50 rotor teeth: the detent and electrical period is 4 full steps (7.2 deg)
STEPPER = {"micro_err": 0.05, "detent_n": 0.022 / R_PULLEY, "vsup": 24.0}
# Torque-speed rolloff: phase current = min(I_rated, (V - Ke w) / Z), Z = sqrt(R^2 + (w_e L)^2), w_e = 50 w,
# Ke = T_hold / (sqrt2 I) for two phases on.
# MODELED SIMPLIFICATION: the microstep error is a sinusoid over one electrical cycle (the datasheet gives only the
# +-5% bound); no chopper current-rise simulation; V is the board supply (24 V is typical, not from a datasheet).
MOTOR_R, MOTOR_L, MOTOR_I, MOTOR_POLES = 1.8, 2.8e-3, 1.7, 50
# Extruder melt chamber: first-order lag from filament drive to nozzle flow, tau = Linear Advance K (s: mm of filament
# compression per mm/s). Marlin "Linear Advance" and Klipper "Pressure advance" docs put PLA through a 0.4 mm nozzle at
# about 0.02-0.1 s direct drive and 0.4-1 s Bowden (recalled from those docs, not re-fetched). tau 0 = ideal extruder.
NOMINAL = {
    "x": {"k": 110e3, "zeta": 0.1, "mass_add": 0.0, "backlash_mm": 0.0, "fmax": 70.0, "micro_err": 0.0, "detent_n": 0.0, "vsup": 0.0},
    "y": {"k": 110e3, "zeta": 0.1, "mass_add": 0.0, "backlash_mm": 0.0, "fmax": 70.0, "micro_err": 0.0, "detent_n": 0.0, "vsup": 0.0},
    "z": {"backlash_mm": 0.0},
    "e": {"tau_s": 0.0},
}


class Thermal:
    """dT/dt = (P * duty - (h + fan_h * fan) * (T - T_amb)) / C per heater, explicit Euler on each ADC slice (1 ms); duty is the
    exact high time of the heater pin since the last step. Marlin's own thermistor table maps T to the ADC volts.
    MODELED SIMPLIFICATION: typical values, not measured. Hotend: 40 W cartridge, C 16 J/K, h 0.095 W/K, fan +0.05 W/K.
    Bed: 200 W, C 800 J/K, h 0.6 W/K. `scale` multiplies P; `speed` divides C (same steady state, time constants /speed)."""
    T_AMB = 25.0
    PARAM = {"hot": dict(p=40.0, c=16.0, h=0.095, fan_h=0.05), "bed": dict(p=200.0, c=800.0, h=0.6, fan_h=0.0)}

    def __init__(self, cfg):
        import re
        pts = [(int(a), int(b)) for a, b in re.findall(r"OV\(\s*(\d+)\)\s*,\s*(-?\d+)", open(cfg["table"]).read())]
        self.tab = sorted((t, r) for r, t in pts)  # (T, raw10), T ascending
        self.cfg, self.t_adv, self.fan = cfg, 0, 0.0
        self.T = {k: self.T_AMB for k in self.PARAM}
        self.lvl = {k: False for k in (*self.PARAM, "fan")}
        self.t_edge = dict.fromkeys(self.lvl, 0)
        self.on = dict.fromkeys(self.lvl, 0)
        self.trace = open(cfg["trace"], "w", buffering=1) if cfg.get("trace") else None

    def edge(self, k, t, lvl):
        if self.lvl[k]:
            self.on[k] += max(0, t - max(self.t_edge[k], self.t_adv))
        self.lvl[k], self.t_edge[k] = lvl, t

    def volts(self, k):
        f = self.cfg.get("fault")
        if f and f["kind"] == "open" and self.t_adv >= f["t"] * 1e9:
            return 3.3  # open circuit: the pull-up alone
        T = min(max(self.T[k], self.tab[0][0]), self.tab[-1][0])
        for (t0, r0), (t1, r1) in zip(self.tab, self.tab[1:]):
            if T <= t1:
                return 3.3 * (r0 + (r1 - r0) * (T - t0) / (t1 - t0)) / 1023

    def step(self, now):
        dt = (now - self.t_adv) * 1e-9
        if dt <= 0:
            return
        duty = {}
        for k in self.lvl:
            self.edge(k, now, self.lvl[k])  # close the high interval at `now`
            duty[k], self.on[k] = self.on[k] / (dt * 1e9), 0
        if getattr(self, "fan_pwm", None) is not None:  # hardware-PWM fan, read by peers/thermal_peer.py
            duty["fan"] = self.fan_pwm
        self.fan += (duty["fan"] - self.fan) * min(1.0, dt / 0.1)  # PWM fan seen through 100 ms
        f = self.cfg.get("fault")
        dead = f and f["kind"] == "dead" and now >= f["t"] * 1e9
        for k, q in self.PARAM.items():
            p = 0.0 if dead else q["p"] * self.cfg.get("scale", 1.0) * duty[k]
            self.T[k] += (p - (q["h"] + q["fan_h"] * self.fan) * (self.T[k] - self.T_AMB)) / (q["c"] / self.cfg.get("speed", 1.0)) * dt
        self.t_adv = now
        if self.trace:
            self.trace.write(f"{now / 1e9:.4f} {self.T['hot']:.3f} {self.T['bed']:.3f} {duty['hot']:.3f} {duty['bed']:.3f} {self.fan:.3f}\n")


class Slave:
    BURST_GAP_NS = 20_000

    def __init__(self, ctx):
        self.ctx = ctx
        cfg = getattr(ctx, "config", {}) or {}
        self.pos = [round(m * s) for m, s in zip(START_MM, STEPS_PER_MM)]
        self.dirs = [[(0, False)] for _ in AXES]  # (t_ns, level) history per axis
        self.hit = [None, None, None]
        self.log = open(cfg["log"], "w", buffering=1) if cfg.get("log") else None
        self.mj = bool(cfg.get("mjcf"))
        self.phys_us = cfg.get("phys_us", PHYS_US)
        self.motor = None
        self.th = Thermal(cfg["thermal"]) if cfg.get("thermal") else None
        if self.th:
            import sys
            sys._printer_thermal = self.th  # peers/thermal_peer.py writes its volts to the ADC
        if cfg.get("x_drive"):
            import sys
            sys.path.insert(0, cfg["x_drive"]["pmsm_dir"])
            from pmsm import MOTOR
            self.motor, self.th0 = MOTOR, MOTOR.theta_m
        if self.mj:
            self._init_mujoco(cfg)
            ctx.schedule_periodic(self.phys_us)
        for a in range(3):
            self._endstop(a, self._axis_mm(a))

    def _init_mujoco(self, cfg):
        import mujoco
        self.mujoco = mujoco
        m = self.model = mujoco.MjModel.from_xml_path(cfg["mjcf"])
        phys = {ax: {**NOMINAL[ax], **cfg.get("phys", {}).get(ax, {})} for ax in NOMINAL}
        self.phys = phys
        m.body_mass[m.body("carriage").id] += phys["x"]["mass_add"]
        m.body_mass[m.body("gantry").id] += phys["y"]["mass_add"]
        m.opt.timestep = self.phys_us * 1e-6
        self.data = mujoco.MjData(m)
        mujoco.mj_setConst(m, self.data)
        mass = {"x": m.body_subtreemass[m.body("carriage").id], "y": m.body_subtreemass[m.body("gantry").id]}
        self.c = {ax: 2 * phys[ax]["zeta"] * math.sqrt(phys[ax]["k"] * mass[ax]) for ax in "xy"}
        self.belt_c = 2 * phys["x"]["zeta"] * math.sqrt(BELT_K * mass["x"])
        self.qadr = [m.jnt_qposadr[m.joint(n).id] for n in "xyz"]
        self.vadr = [m.jnt_dofadr[m.joint(n).id] for n in "xyz"]
        self.act = [m.actuator(n).id for n in ("mx", "my")]
        self.target = [self.pos[a] / STEPS_PER_MM[a] / 1000 for a in range(3)]
        self.prev_target = list(self.target)
        self.lost = [0.0, 0.0]  # steps the rotor has slipped, m
        self.e_dep = 0.0  # filament pushed out of the nozzle, mm
        self.data.qpos[self.qadr] = self.target
        mujoco.mj_forward(m, self.data)
        self.frames = open(cfg["frames"], "w", buffering=1) if cfg.get("frames") else None
        self.trace = open(cfg["trace"], "w", buffering=1) if cfg.get("trace") else None
        self.next_frame_ns = self.next_trace_ns = 0

    def _axis_mm(self, a):
        if self.mj:
            return float(self.data.qpos[self.qadr[a]]) * 1000
        return self.pos[a] / STEPS_PER_MM[a]

    def _play(self, a, cmd):
        """Backlash as a dead zone: the driven side follows the motor side only once the gap closes."""
        half = self.phys["xyz"[a]]["backlash_mm"] / 2000
        t = self.target[a]
        if cmd > t + half:
            t = cmd - half
        elif cmd < t - half:
            t = cmd + half
        return t

    def on_timer(self):
        d, dt = self.data, self.phys_us * 1e-6
        for a in range(3):
            cmd = self.pos[a] / STEPS_PER_MM[a] / 1000
            if a < 2:  # the rotor sits off the ideal microstep angle by up to micro_err of a full step
                cmd += self.lost[a] + self.phys["xy"[a]]["micro_err"] * FULL_STEP_M * math.sin(2 * math.pi * cmd / CYCLE_M)
            self.target[a] = self._play(a, cmd)
        for i, ax in enumerate("xy"):
            if i == 0 and self.motor:
                m = self.motor
                xm = START_MM[0] / 1000 + (m.theta_m - self.th0) * R_PULLEY
                f = BELT_K * (xm - d.qpos[self.qadr[0]]) + self.belt_c * (m.wm * R_PULLEY - d.qvel[self.vadr[0]])
                d.ctrl[self.act[0]] = f
                m.load = f * R_PULLEY
                continue
            p = self.phys[ax]
            err = self.target[i] - d.qpos[self.qadr[i]]
            derr = (self.target[i] - self.prev_target[i]) / dt - d.qvel[self.vadr[i]]
            fmax = p["fmax"]
            if p["vsup"]:
                w = abs(d.qvel[self.vadr[i]]) / R_PULLEY
                ke = fmax * R_PULLEY / (math.sqrt(2) * MOTOR_I)
                z = math.hypot(MOTOR_R, MOTOR_POLES * w * MOTOR_L)
                fmax *= max(0.0, min(1.0, (p["vsup"] - ke * w) / z / MOTOR_I))
            d.ctrl[self.act[i]] = max(-fmax, min(fmax, p["k"] * err + self.c[ax] * derr))
            if p["detent_n"]:  # the rotor magnet pulls the carriage toward the nearest tooth
                d.ctrl[self.act[i]] -= p["detent_n"] * math.sin(2 * math.pi * d.qpos[self.qadr[i]] / CYCLE_M)
            if p["micro_err"] and abs(err) > FULL_STEP_M:  # lagging past one full step is past the torque peak: a step is lost
                self.lost[i] -= math.copysign(FULL_STEP_M, err)
        # Z: the leadscrew (~14 MN/m, 100x the belts) follows its step target through the nut's backlash.
        d.qpos[self.qadr[2]], d.qvel[self.vadr[2]] = self.target[2], 0.0
        self.prev_target = list(self.target)
        self.mujoco.mj_step(self.model, d)
        for a in range(3):
            self._endstop(a, self._axis_mm(a))
        t = self.ctx.now_ns
        e = self.pos[3] / STEPS_PER_MM[3]
        tau, prev = self.phys["e"]["tau_s"], self.e_dep
        self.e_dep = prev + (e - prev) * dt / tau if tau else e
        qf = (self.e_dep - prev) / dt
        if self.trace and t >= self.next_trace_ns:
            self.next_trace_ns = t + TRACE_NS
            q = [float(d.qpos[i]) * 1000 for i in self.qadr]
            self.trace.write(f"{t} {q[0]:.4f} {q[1]:.4f} {q[2]:.4f} {self.target[0]*1000:.4f} {self.target[1]*1000:.4f} {e:.4f} {self.e_dep:.5f} {qf:.3f}\n")
        if self.frames and t >= self.next_frame_ns:
            self.next_frame_ns = t + FRAME_NS
            q = [float(d.qpos[i]) for i in self.qadr]
            self.frames.write(json.dumps({"t": t, "q": q, "c": list(self.target), "e": e}) + "\n")

    def _dir_at(self, a, t):
        for tt, lvl in reversed(self.dirs[a]):
            if tt <= t:
                return lvl
        return self.dirs[a][0][1]

    def _endstop(self, a, mm):
        hit = mm <= 0
        if hit != self.hit[a]:
            self.hit[a] = hit
            self.ctx.gpio(a).set(hit)  # X/Y/Z_MIN_ENDSTOP_HIT_STATE HIGH

    def on_burst(self, line, edges):
        if line >= 8:  # heater / fan pins
            for t, lvl in edges:
                self.th.edge(("hot", "bed", "fan")[line - 8], t, bool(lvl))
            return None
        if line in DIR_LINE:
            a = DIR_LINE[line]
            self.dirs[a].extend(edges)
            del self.dirs[a][:-64]
            return None
        a = STEP_LINE.get(line)
        if a is None:
            return None
        reply = []
        for t, lvl in edges:
            if not lvl:
                continue
            fwd = self._dir_at(a, t) != INVERT_DIR[a]
            self.pos[a] += 1 if fwd else -1
            if self.log:
                self.log.write(f"{t} {AXES[a]} {self.pos[a]}\n")
            # Without MuJoCo the switch reads the step position, at the exact step time.
            if a < 3 and not self.mj:
                hit = self.pos[a] <= 0
                if hit != self.hit[a]:
                    self.hit[a] = hit
                    reply.append((0, hit))  # offset from the burst's last edge
        return reply or None

    def get_property(self, name):
        if name == "pos_mm":
            return json.dumps([p / s for p, s in zip(self.pos, STEPS_PER_MM)])
        if name in ("tv0", "tv1"):  # thermistor volts: advance the thermal model to now
            self.th.step(self.ctx.now_ns)
            return self.th.volts(("hot", "bed")[int(name[2])])
        raise KeyError(name)
