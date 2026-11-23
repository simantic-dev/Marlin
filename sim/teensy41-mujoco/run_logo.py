"""One logo run: python run_logo.py <logo-foc|logo-step> logo.gcode <virtual s>  -> out/<name>/ (sim.txt, trace.txt, frames.jsonl)."""
import sys, time, cases
cases.FOC_ENV["logo-foc"] = "teensy41_ff_fast"
name, foc = sys.argv[1], sys.argv[1].endswith("foc")
t = time.time()
cases.run(name, {}, foc=foc, gc=open(sys.argv[2]).read(), timeout=float(sys.argv[3]))
print(name, "wall", round(time.time() - t, 1))
