# Hotend thermal plant + thermistor on TH0 (PF4 = ADC3 IN14), a ScriptedPeripheral parked on a 4-byte window.
# Every 10 ms: duty = mean level of HEATER_0 (PA2) since the last tick (from machine_peer.py's edges),
# one explicit-Euler step of the first-order model in printer.py, then the thermistor divider voltage is
# written to the ADC. MODELED SIMPLIFICATION, parameters in printer.py.
import printer

TICK_US = 10_000
ADC, CH = "adc3", "ch14"


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.last = 0
        self.n = 0
        self.log = open(f"{printer.OUT}/thermal.csv", "w", buffering=1)
        self.log.write("t_s,temp_c,duty,volts\n")
        ctx.schedule_periodic(TICK_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(TICK_US)

    def on_timer(self):
        self.n += 1
        t = self.n * TICK_US  # a ScriptedPeripheral has no clock; its periodic timer is exact
        duty = printer.heater_take_duty(t, self.last)
        temp = printer.thermal_step(0.0 if printer.HEATER_OPEN else duty, (t - self.last) / 1e6)
        self.last = t
        v = printer.volts_for(printer.T_AMB if printer.DETACH_US is not None and t >= printer.DETACH_US else temp)
        self.ctx.set_property(ADC, CH, v)
        if self.n % 50 == 0:
            self.log.write(f"{t / 1e6:.3f},{temp:.3f},{duty:.3f},{v:.4f}\n")
