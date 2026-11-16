#!/usr/bin/env bash
# Run the printer demo on Simantic's `sim` (0.10.1) and draw the plots.
#   sim/run.sh [virtual-seconds]     default 130; JOB=job.gcode selects the G-code file in sim/
# Outputs in sim/out/: console.txt (what the host peer received, guest us + line), sent.txt, pins.jsonl
# (every GPIO pad edge, sim --trace-out), steps.csv + thermal.csv (the peers' own logs), u.txt (UART as printed).
set -euo pipefail
cd "$(dirname "$0")/.."
T=${1:-130}
ELF=.pio/build/STM32F446ZE_btt/firmware.elf
[ -f "$ELF" ] || { echo "$ELF missing: run sim/build.sh first" >&2; exit 1; }
export SIM_OUT=$PWD/sim/out JOB=${JOB:-job.gcode}
mkdir -p sim/out
sim run --scenario sim/scenario.yaml --timeout "$T" --ascii --trace-out sim/out/pins.jsonl > sim/out/u.txt 2> sim/out/e.txt
PY=${PYTHON:-.sim-tools/venv/bin/python}
"$PY" sim/plots.py sim/out sim/screenshots
