# Tesla Model 3 PCS controller (US)

STM32F103 controller firmware for a Power Conversion System from a US Tesla
Model 3. The controller communicates over IPC CAN at 500 kbit/s, keeps DCDC
support active and emulates the US Type-1/NACS charge-port messages needed to
request AC charging without a physical charge port, proximity input or EVSE
pilot interface.

## Important safety notice

The PCS, HV battery and 220 V mains can cause fatal injury. This firmware is
not a safety controller. Use correctly rated fuses, isolation, interlocks,
precharge, hardware current limiting and an independent emergency disconnect.
Do not apply AC/HV until the logic-level outputs and power circuit have been
verified at the PCS connector.

## What was fixed

- Correct `0x264` AC-current decoding: the attached trace is 220 V, 0 A, 0 W,
  not 51 A.
- Correct US `0x2B2` power-request buffer and DLC handling.
- Correct `0x22A` DLC, support/charge mode and measured-HV packing.
- Correct post-2020 `0x23D` to the four-byte charge-status format required by
  the captured 32 A single-phase PCS. The later `3.txt` capture confirms the
  PCS receives that format, although it also proves `0x23D` was not the cause
  of the remaining 8 A clamp.
- Removed the synthetic transmitted `0x2C4` that collided with PCS telemetry.
- Added a consistent 16 A live request using `0x13D`, the `0x21D` pilot,
  `0x23D` and `0x2B2`. The independent US cable/UI capability fields remain
  at the tested 32 A/48 A profile values so they cannot create a hidden lower
  clamp inside the PCS.
- Added the runtime `CHGcurrentSetpointA` register (0...16 A), synchronized
  current-limit frames and an AC-voltage-based `0x2B2` power request.
- Added automatic start after stable AC is confirmed by the PCS.
- Added CAN/HV/AC/PCS fault states and `0x424` alert diagnostics.
- Added decoding of the PCS per-phase current request from `0x204`. The
  `I set/PCS` display now shows the commanded current and the current the PCS
  actually requests internally (for example `16/8`).
- Added host-side protocol regression tests and a portable ARM Makefile.

The original capture reports `0x204 = 78 00 00 FF 00 00 00 09`, which decodes
as charger main state 8 and HV charge status 3: both are `faulted`. A full safe
PCS power-cycle is required before testing the corrected firmware.

## Build and test

```sh
make test
make -j4
```

Generated release files:

- `build/tesla_charger.bin`
- `build/tesla_charger.elf`
- `build/tesla_charger.map`

Verified size (`text=44,836`, `data=128`, `bss=3,480`):

- Flash: 44,964 / 65,536 bytes
- RAM: 3,608 / 20,480 bytes

Current BIN SHA-256:

```text
41957406914a8227cebb314cf44411d900798c6b6f621b536d515f5a88ca7226
```

## Flashing the correct image

The tracked STM32CubeIDE launch configuration downloads
`build/tesla_charger.elf`. It intentionally does not rebuild before launch, so
run `make test && make -j4` first. Do not select `Debug/Tesla Charger.elf`:
`Debug/` is an ignored CubeIDE output directory and may contain an older local
firmware image.

When flashing the BIN directly, use `build/tesla_charger.bin` at address
`0x08000000` and enable post-write verification. The current firmware is easy
to identify without trusting the file name:

- the display header is `PCS RC4` and contains `I set/PCS=`;
- CAN `0x22A` has DLC 8 and starts with `00 0B`;
- CAN `0x2B2` has DLC 5 for the current tested PCS configuration.

## Documentation

- [`DEBUGGING.md`](DEBUGGING.md): settings, state codes, expected CAN frames,
  first-start procedure and trace-capture checklist.
- [`output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf`](output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf): complete Russian PDF report.

## Protocol references

- [Damien Maguire: Tesla Model 3 Charger](https://github.com/damienmaguire/Tesla-Model-3-Charger)
- [US-tested M3_PCS_V6 controller](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/M3_PCS_V6.ino)
- [STM32 PCS controller and alert decoding](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/stm32-M3-PCS/src/PCSCan.cpp)
- [Model 3 CAN DBC](https://github.com/joshwardell/model3dbc/blob/master/Model3CAN.dbc)
