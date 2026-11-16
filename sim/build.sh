#!/usr/bin/env bash
# Build Marlin for the BigTreeTech Octopus V1.1 (STM32F446ZE) with PlatformIO 6.1.19 in a private venv and a
# private PlatformIO core dir under .sim-tools (nothing in ~/.platformio is touched).
# Drops .pio/build/STM32F446ZE_btt/firmware.elf. First run downloads the toolchain (a few hundred MB).
set -euo pipefail
cd "$(dirname "$0")/.."
TOOLS=$PWD/.sim-tools
[ -x "$TOOLS/venv/bin/pio" ] || { python3 -m venv "$TOOLS/venv"; "$TOOLS/venv/bin/pip" -q install platformio==6.1.19 matplotlib; }
export PLATFORMIO_CORE_DIR=$TOOLS/pio
"$TOOLS/venv/bin/pio" run -e STM32F446ZE_btt   # if scons fails with SCons.Tool.FortranCommon, just rerun
