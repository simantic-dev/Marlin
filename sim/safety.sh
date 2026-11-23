#!/bin/sh
# Thermal-protection scenes on the committed image: the closed loop of sim/scenario.yaml with one fault injected
# into the hotend peer ($FAULT, see printer.py). Each scene must end with Marlin's own error and halt.
#   sim/safety.sh [outdir]   default sim/out/safety; writes <scene>/{u.txt,console.txt,thermal.csv}
cd "$(dirname "$0")/.." || exit 1
OUT=${1:-sim/out/safety}
fail=0
scene() { # name fault job timeout expected
  mkdir -p "$OUT/$1"
  SIM_OUT=$PWD/$OUT/$1 FAULT=$2 JOB=$3 ${SIM:-sim} run --scenario sim/scenario.yaml --timeout "$4" --ascii \
    > "$OUT/$1/u.txt" 2> "$OUT/$1/e.txt"
  if grep -q "$5" "$OUT/$1/console.txt" && grep -q "Printer halted" "$OUT/$1/console.txt"; then
    echo "ok   $1: $(grep -m1 "$5" "$OUT/$1/console.txt")"
  else echo "FAIL $1"; fail=1; fi
}
scene heating-failed heater-open safety_200.gcode 50 "Heating failed"
scene early-fall detach:39 safety_90.gcode 50 "Heating failed"
scene thermal-runaway detach:125 safety_200.gcode 180 "Thermal Runaway"
exit $fail
