# Surface PMSM on a 3-PWM bridge, as the SimpleFOC drive (foc/src/main.cpp) drives it
# (SimpleFOC BLDCDriver3PWM on Teensy 4.1 pins 4/5/6 = FlexPWM2 SM0/SM1/SM2 channel A, edge-aligned, 12 V supply). The
# duty is read from FlexPWM2 VAL3/VAL1 (RM ch.54: A sets at VAL2=0, clears at VAL3, period VAL1+1) when OUTEN.PWMA_EN drives the pad; the register file (outer set) is read, which equals the active set once SimpleFOC's LDOK has loaded, and no shunt
# outputs: the drive has no current sensing. The rotor angle it integrates is shared with
# mt6835_peer.py through pmsm.MOTOR.
#
# Model (rotor dq frame, amplitude-invariant Park): phase-to-neutral volts from the three duty
# cycles CCRx / ARR; L did/dt = vd - R id + we L iq, L diq/dt = vq - R iq - we L id - we psi;
# J dwm/dt = 1.5 p psi iq - B wm; we = p wm.
#
# MODELED SIMPLIFICATION: averaged switching; the bridge is always enabled (no enable pin); the carriage
# reaches the rotor only as m.load, the belt torque from peers/printer_plant.py; the plant steps every 40 us (the 25 kHz PWM period) with 10 sub-steps.
import math

from pmsm import MOTOR

# Nanotec DB42S02 datasheet (rev 6): delta 0.19 ohm / 0.245 mH line-line -> star-equivalent 0.095 ohm / 0.1225 mH;
# Kt 0.014 N.m/A = 1.5 P psi -> psi = 2.33 mWb; rotor inertia 2.4e-6 kg m^2. B (bearing drag) is a small stand-in.
R, L, PSI, P = 0.095, 1.225e-4, 0.014 / (1.5 * 4), 4
J, B = 2.4e-6, 1e-6
VBUS = 12.0
FLEXPWM2 = 0x403E0000
SM_STRIDE, VAL1, VAL3, OUTEN = 0x60, 0x0E, 0x16, 0x180
STEP_US, SUB = 40, 10


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        ctx.schedule_periodic(STEP_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(STEP_US)
        req.value = 0

    def on_timer(self):
        rd = lambda off: self.ctx.bus_read(FLEXPWM2 + off, 2)
        outen = rd(OUTEN)
        duty = []
        for sm in range(3):
            base = sm * SM_STRIDE
            on = (outen >> (8 + sm)) & 1
            duty.append(min(rd(base + VAL3) / (rd(base + VAL1) + 1), 1.0) if on else 0.0)
        m = MOTOR
        dt = STEP_US * 1e-6 / SUB
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        for _ in range(SUB):
            th = P * m.theta_m
            c = [math.cos(th - k * 2 * math.pi / 3) for k in range(3)]
            s = [math.sin(th - k * 2 * math.pi / 3) for k in range(3)]
            we = P * m.wm
            vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
            vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
            did = (vd - R * m.id + we * L * m.iq) / L
            diq = (vq - R * m.iq - we * L * m.id - we * PSI) / L
            m.id += did * dt
            m.iq += diq * dt
            m.wm += (1.5 * P * PSI * m.iq - B * m.wm - m.load) / J * dt
            m.theta_m += m.wm * dt
