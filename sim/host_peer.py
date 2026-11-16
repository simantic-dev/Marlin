# The host on USART1 (PA9/PA10, 115200 8N1): sends one G-code line, waits for the firmware's `ok`, sends the next.
# G-code file: $JOB (default job.gcode), first line at START_S. Logs every received line with its guest time to out/console.txt
# and every sent line to out/sent.txt.
import os
import printer

START_S = 2


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        here = os.path.dirname(os.path.abspath(__file__))
        lines = [l.split(";")[0].strip() for l in open(os.path.join(here, os.environ.get("JOB", "job.gcode")))]
        self.lines = [l for l in lines if l]
        self.i = 0
        self.rx = bytearray()
        self.waiting = False
        self.con = open(f"{printer.OUT}/console.txt", "w", buffering=1)
        self.sent = open(f"{printer.OUT}/sent.txt", "w", buffering=1)
        ctx.schedule_oneshot(int(START_S * 1e6))

    def on_timer(self):
        self._send()

    def _send(self):
        if self.i < len(self.lines) and not self.waiting:
            line = self.lines[self.i]
            self.i += 1
            self.waiting = True
            self.sent.write(f"{self.ctx.now_us} {line}\n")
            self.ctx.send((line + "\n").encode())

    def on_byte(self, b):
        if b == 10:
            line = self.rx.decode(errors="replace").rstrip("\r")
            self.rx.clear()
            self.con.write(f"{self.ctx.now_us} {line}\n")
            if line.startswith("ok") and self.waiting:
                self.waiting = False
                self._send()
        else:
            self.rx.append(b)
