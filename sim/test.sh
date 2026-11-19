#!/bin/sh
# Runs the committed Marlin image (sim/fw/marlin_octopus.elf, BTT Octopus / STM32F446ZE) on the simulator against
# the Python printer peers: G-code host on USART1, three scripted endstops, STEP/DIR stepper axes, hotend plant.
# Job: sim/job_short.gcode (M115, G28 homing, 40 mm square, diagonal, M114); the heat-up wait of the full demo is
# left out (cold extrusion enabled with M302 P1). Pass = Marlin boots, answers M115, homes, and the steps the peer
# counted from the STEP pins match Marlin's own position (X70 Y70 Z0.30 E6.40, Count X5600 Y5600 Z120).
cd "$(dirname "$0")/.." || exit 1
OUT=$(mktemp -d) || exit 1
trap 'rm -rf "$OUT"' EXIT
export SIM_OUT=$OUT JOB=job_short.gcode
${SIM:-sim} run --scenario sim/scenario.yaml --timeout 21 --ascii > "$OUT/u.txt" 2> "$OUT/e.txt"
rc=$?
fail=0
chk() { # name pattern file
  if grep -qF -- "$2" "$3"; then echo "ok   $1"; else echo "FAIL $1"; fail=1; fi
}
[ $rc -eq 0 ] && echo "ok   sim exit 0" || { echo "FAIL sim exit $rc"; fail=1; }
chk "boot banner" "Marlin bugfix-2.1.x" "$OUT/u.txt"
chk "M115 reply" "FIRMWARE_NAME:Marlin" "$OUT/console.txt"
chk "homed (Z hop to 5 mm after G28)" "X:0.00 Y:0.00 Z:5.00 E:0.00 Count X:0 Y:0 Z:2000" "$OUT/console.txt"
chk "square+diagonal position and step counts" "X:70.00 Y:70.00 Z:0.30 E:6.40 Count X:5600 Y:5600 Z:120" "$OUT/console.txt"
if grep -qiE "HardFault|panic|Guru|Printer halted|kill\(\)|MAXTEMP|Error:" "$OUT/u.txt" "$OUT/e.txt" "$OUT/console.txt"; then
  echo "FAIL fault/halt line seen"; fail=1
else echo "ok   no fault/halt lines"; fi
exit $fail
