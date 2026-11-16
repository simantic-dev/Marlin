# Marlin in a simulator: a 3D printer with no printer

Marlin (`bugfix-2.1.x`) built for the BigTreeTech Octopus V1.1 (STM32F446ZE, 12 MHz crystal) and run on a
simulated STM32F446ZE with the printer around it written as Python peers: a G-code host on the serial port, a
hotend with a heater and a thermistor, three scripted endstop switches and stepper axes that follow the STEP/DIR
pins. No board, no printer. Backend: Simantic's `sim` 0.10.1 (build ccc73c89, pyrite engine, JIT on by default).
Everything below was run on 2026-11-16.

![Toolpath rebuilt from STEP/DIR pad edges](screenshots/toolpath.png)

*A 40 mm square (extruding, red) and a diagonal (travel), rebuilt only from the STEP and DIR pad edges that
`sim --trace-out` wrote. Cross-check: the peer that counted the same edges while running ends on the same
X 70.00 Y 70.00 Z 0.30 E 6.40 mm, and the firmware's own `M114` reports `Count X:5600 Y:5600 Z:120` at the end.*

![Hotend temperature and heater duty](screenshots/temperature.png)

*Marlin's stock PID heating the plant to 190 C (`M109 S190`), then `M104 S0`. The firmware's own `M155` readings
(orange) come back through ADC3 channel 14 and Marlin's thermistor table, and sit within 0.43 C of the plant's
true temperature over 107 reports. The lower panel is the HEATER_0 pad duty from the pad edges and the firmware's
own `@:` value divided by 127.*

![Homing sequence](screenshots/homing.png)

*`G28`: each axis runs to its scripted endstop, backs off, and approaches again slowly (Z first hops up 5 mm, the
stock `Z_HOMING_HEIGHT`). The pad levels are the peer's switches; positions are counted from STEP/DIR.*

![Console transcript](screenshots/console.png)

*The boot banner and the first G-code lines on USART1, with guest timestamps.*

## What runs

| Part | Model | Where |
| --- | --- | --- |
| Firmware | Marlin `bugfix-2.1.x`, `STM32F446ZE_btt` env, four lines of `Configuration.h` changed (below) | STM32F446ZE, `BOARD_BTT_OCTOPUS_V1_1` |
| G-code host | `host_peer.py`: sends one line, waits for `ok` | USART1 (PA9/PA10), 115200 8N1 |
| Steppers | `machine_peer.py`: ideal axes, STEP rising edge moves one step in the DIR direction, hard stop at 0 mm. A4988 standalone mode (no TMC UART/SPI) | X PF13/PF12, Y PG0/PG1, Z PF11/PG3, E0 PF9/PF10 |
| Endstops | the same peer: pad driven low until the axis reaches 0 mm, then released so the firmware's pull-up reads it high (`*_MIN_ENDSTOP_HIT_STATE HIGH`) | X PG6, Y PG9, Z PG10 |
| Hotend | `thermal_peer.py` + `printer.py`: first-order heater plant, thermistor divider written to the ADC every 10 ms | HEATER_0 PA2 in, TH0 PF4 (ADC3 IN14) out |

The scenario, as `sim graph --scenario sim/scenario.yaml` prints it:

```mermaid
flowchart LR
  dut["dut<br/>STM32F446ZE"]
  dut_host(["host<br/>UART.ScriptedUartPeer"])
  dut_machine(["machine<br/>Scripted.ScriptedGpioPeer"])
  dut -- "sysbus" --- dut_machine
  dut_thermal(["thermal<br/>Scripted.ScriptedPeripheral"])
  dut -- "sysbus" --- dut_thermal
  medium0{{"uart"}}
  dut -- "usart1" --- medium0
  dut_host --- medium0
```

## Configurations tried

One run of `sim/run.sh`, `sim` 0.10.1, pyrite, 2026-11-16, `--timeout 110`.

| Configuration | Virtual s | Wall s | Result |
| --- | --- | --- | --- |
| `sim/job.gcode` as shipped (M115, M155, M104 S190, G28, M109 S190, 40 mm square, diagonal, M104 S0) | 110 | 153 | MEASURED: banner at 0.014 s; `G28` finished at 13.65 s (X triggers at 3.98 s and 4.50 s, Y 5.11 s and 5.64 s, Z 9.51 s and 12.26 s); `M109` returned at 103.1 s; 14401 X / 14801 Y / 13481 Z / 3200 E steps on the pads; final position X 70 Y 70 Z 0.30 E 6.40; temperature first at 189.0 C at 87.5 s, peak 191.1 C |

Wall time is about 1.4x the virtual time on this machine (an M-series laptop with other jobs running).
Not run: a heated bed, a TMC driver over UART or SPI, the SD card, USB, bed levelling or a probe, `M303` autotune,
thermal runaway or MAXTEMP fault injection, a second run to check repeatability, anything on real hardware.
These are `unverified`.

## Run it

Needs Python 3, Simantic's `sim` 0.10.1 on PATH, and network for the first build.

```
sim/build.sh     # private venv with PlatformIO 6.1.19 and a private PLATFORMIO_CORE_DIR in .sim-tools/
sim/run.sh 110   # runs the scenario, writes sim/out/*, redraws sim/screenshots/*.png
```

If the first scons link fails with `SCons.Tool.FortranCommon`, run `build.sh` again.
`sim --trace-out` records every GPIO edge. If the firmware halts (a MAXTEMP, for example) the LED pin toggles in a
tight loop and the file grows by gigabytes in a minute, so keep `--timeout` short while changing things.

## Changes to Marlin

`Marlin/Configuration.h`, nothing else under `Marlin/`:

| Line | Stock | Here | Why |
| --- | --- | --- | --- |
| `MOTHERBOARD` | `BOARD_RAMPS_14_EFB` | `BOARD_BTT_OCTOPUS_V1_1` | an STM32F446ZE board the sim models |
| `SERIAL_PORT` | `0` | `1` | USART1 on PA9/PA10 |
| `BAUDRATE` | `250000` | `115200` | |
| `TEMP_SENSOR_BED` | `1` | `0` | no bed in the demo: one thermistor to feed |

Everything else is the stock default: A4988 drivers, `DEFAULT_AXIS_STEPS_PER_UNIT { 80, 80, 400, 500 }`,
`INVERT_Y_DIR true` (the Y peer models that), thermistor type 1 (100 kOhm, beta 4092 K, 4.7 kOhm pull-up), the
default PID (`M301 P22.20 I1.08 D114.00`), thermal protection. `sim/octopus.board` only declares the 12 MHz
crystal (`HSE_VALUE` in `buildroot/share/PlatformIO/variants/MARLIN_BTT_OCTOPUS_V1`).

## MODELED SIMPLIFICATION: the hotend

`C dT/dt = P * duty - (T - T_amb) / R_th`: one lump, explicit Euler every 10 ms of guest time, `duty` the mean
level of the HEATER_0 pad over the tick. This is the textbook first-order model. The parameters are typical
values for an E3D-V6-class hotend, **not measured on any printer**:

| Parameter | Value | Where it comes from |
| --- | --- | --- |
| P | 40 W | the usual rating of a 24 V cartridge heater |
| C | 12 J/K | about 13 g of aluminium block at 0.9 J/(g K); nozzle and heat break ignored |
| R_th | 6.9 K/W | chosen so 100 % duty settles at 25 + 40 x 6.9 = 301 C, the usual hotend ceiling |
| T_amb | 25 C | |

R_th is a fit to that ceiling, not a measurement. Marlin's default PID is the Ultimaker-derived stock value and is
not tuned for this plant: the heater is fully on until about 68 s (177 C), then the PID tapers the duty from 0.9
to 0.6 and the temperature creeps up to the target. The thermistor voltage is the inverse of Marlin's own
`thermistor_1.h` table (10-bit scale, 3.3 V reference), so the firmware reads back the plant's temperature.

Other stand-ins: the axes have no inertia, no lost steps and no limit except 0 mm; the carriages start at
X 20, Y 25, Z 10 mm (arbitrary parking positions); the endstop switch does not bounce; the heater MOSFET and the
24 V rail are not modelled, the plant sees the pad level directly.

## Gaps

- **A halted firmware spins the LED pin.** After `kill()` PA13 toggles every few hundred ns, so a `--trace-out`
  file reaches gigabytes (seen: 7.2 million edges in 6 s of guest time). That is the firmware's own halt loop; the
  trace is unbounded.
- **Slower than real time**: about 1.4 s of wall time per guest second here.
- **`ScriptedPeripheral` has no clock** (`ctx.now_us` raises `AttributeError`), so `thermal_peer.py` counts its own
  exact periodic ticks. Other peer types have `now_us`.
- **Scenario `uart-device` parts take no `config:`** (only `gpio-device` does), so `host_peer.py` reads its G-code
  file from `$JOB`.

## Licence

Marlin is free software under the **GNU General Public License v3** (`LICENSE`: "Version 3, 29 June 2007"; the
source headers say "version 3 of the License, or (at your option) any later version"). This fork keeps the licence
and the copyright notices; the files under `sim/` are offered under the same terms. Upstream is
https://github.com/MarlinFirmware/Marlin. This is an unofficial simulation showcase and has not been submitted
upstream.
