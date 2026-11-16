# The machine around the stepper drivers and the heater MOSFET, as a gpio-device (see scenario.yaml).
# Inputs, in scenario order: X/Y/Z/E STEP, X/Y/Z/E DIR, HEATER_0. Outputs: X/Y/Z min endstop switches.
# STEP rising edge moves the axis one step in the DIR direction (A4988 standalone, INVERT_*_DIR false:
# DIR high = positive except Y, whose INVERT_Y_DIR is true in stock Marlin). Each axis has a hard stop at 0 mm where its endstop switch opens; Marlin's
# X/Y/Z_MIN_ENDSTOP_HIT_STATE is HIGH with pull-ups, so the switch holds the pin low until the carriage
# reaches the stop and then releases it (the pull-up takes the pin high).
import printer

AXES = "XYZE"


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.dir = [0, 0, 0, 0]
        self.sw = {}
        self.log = open(f"{printer.OUT}/steps.csv", "w", buffering=1)
        self.log.write("t_us,axis,x_mm,y_mm,z_mm,e_mm\n")
        for i in range(3):
            self._endstop(i)

    def _endstop(self, i):
        hit = printer.pos["XYZ"[i]] <= 0.0
        if hit != self.sw.get(i):
            self.sw[i] = hit
            pin = self.ctx.gpio(i)
            pin.release() if hit else pin.set(False)
            self.ctx.info(f"endstop {'XYZ'[i]} {'TRIGGERED' if hit else 'open'} at {self.ctx.now_us} us")

    def on_gpio(self, line, level):
        t = self.ctx.now_us
        if line < 4:  # STEP
            if not level:
                return
            a = AXES[line]
            d = 1 if self.dir[line] == printer.DIR_POSITIVE_LEVEL[a] else -1
            if a != "E" and printer.pos[a] <= 0.0 and d < 0:
                return  # hard stop: the carriage cannot go further, the step is lost
            printer.steps[a] += d
            printer.pos[a] += d / printer.STEPS_PER_MM[a]
            if a != "E" and printer.pos[a] < 0.0:
                printer.pos[a] = 0.0
            self.log.write(f"{t},{a},{printer.pos['X']:.4f},{printer.pos['Y']:.4f},{printer.pos['Z']:.4f},{printer.pos['E']:.4f}\n")
            if a != "E":
                self._endstop("XYZ".index(a))
        elif line < 8:
            self.dir[line - 4] = int(level)
        else:
            printer.heater_edge(t, level)
