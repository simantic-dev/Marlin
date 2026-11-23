# Teensy 4.1 servo drive pins

Voltage-mode FOC, SimpleFOC 2.3.4 `BLDCDriver3PWM` (fast PWM via `analogWrite`, 25 kHz, 8-bit duty), MT6835 on `SPI`, `StepDirListener`.

| Signal | Teensy pin | Pad / port | Peripheral |
|---|---|---|---|
| Phase A PWM | 4 | EMC_06 | FlexPWM2 SM0 A: VAL1 @0x403E000E (period-1), VAL3 @0x403E0016 (duty) |
| Phase B PWM | 5 | EMC_08 | FlexPWM2 SM1 A: VAL1 @0x403E006E, VAL3 @0x403E0076 |
| Phase C PWM | 6 | B0_10 | FlexPWM2 SM2 A: VAL1 @0x403E00CE, VAL3 @0x403E00D6 |
| (all) | | | OUTEN.PWMA_EN[3:0] @0x403E0180 bits 8-10 |
| SPI SCK / MOSI / MISO | 13 / 11 / 12 | | LPSPI4 (`SPI`), mode 3, 1 MHz |
| MT6835 CS (software) | 10 | B0_00 | GPIO2.00 = fast GPIO7.00 |
| STEP in | 2 | EMC_04 | GPIO4.04 = fast GPIO9.04 |
| DIR in | 3 | EMC_05 | GPIO4.05 = fast GPIO9.05 |

Duty = VAL3 / (VAL1 + 1) (A sets at VAL2 = 0, clears at VAL3). STEP is attached with `attachInterrupt` on GPIO4 (slow bank); the board wires both banks.

## Y axis (env `teensy41_ff_fast_xy`, `-DDUAL_AXIS`)

Same motor, encoder and gains as X, from the same Teensy and the same `loop()`.

| Signal | Teensy pin | Pad / port | Peripheral |
|---|---|---|---|
| Phase A PWM | 22 | AD_B1_08 | FlexPWM4 SM0 A |
| Phase B PWM | 23 | AD_B1_09 | FlexPWM4 SM1 A |
| Phase C PWM | 8 | B1_00 | FlexPWM1 SM3 A |
| MT6835 CS (software) | 9 | B0_11 | GPIO2.11 = fast GPIO7.11, second device on LPSPI4 (`ActiveLowMask 0x3`) |
| STEP in | 20 | AD_B1_10 | GPIO1.26 = fast GPIO6.26, netted from Marlin Y STEP (dut GPIO9.04) |
| DIR in | 21 | AD_B1_11 | GPIO1.27 = fast GPIO6.27, netted from Marlin Y DIR (dut GPIO9.05) |

Scale 2*pi/3200 rad per step (Marlin X and Y: 80 steps/mm, 40 mm/rev). DIR high = positive angle; Marlin inverts Y DIR, so the plant mirrors the Y belt.
