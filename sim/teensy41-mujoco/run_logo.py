"""One logo run: python run_logo.py <logo-step|logo-foc|logo-foc-xy> logo.gcode <virtual s>  -> out/<name>/ (sim.txt, trace.txt, frames.jsonl)."""
import sys, time, cases
name = sys.argv[1]
cases.FOC_ENV[name] = "teensy41_ff_fast_xy" if "foc-xy" in name else "teensy41_ff_fast"
t = time.time()
cases.run(name, {}, foc="foc" in name, gc=open(sys.argv[2]).read(), timeout=float(sys.argv[3]),
          axes="xy" if "foc-xy" in name else "x")
print(name, "wall", round(time.time() - t, 1))
