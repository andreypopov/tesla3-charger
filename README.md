# Tesla Model 3 PCS controller (32 A single-phase)

STM32F103 controller firmware for a 32 A single-phase Tesla Model 3 Power
Conversion System. The controller communicates over IPC CAN at 500 kbit/s,
keeps DCDC support active and emulates the charge-port messages needed to
request AC charging without a physical charge port, proximity input or EVSE
pilot interface. RC7 keeps the coherent Euro IEC profile and fixes
`0x333 UI_chargeRequest` for the newer five-byte protocol generation. The
tested unit also requires the newer five-byte `0x2B2`; those frame lengths are
protocol-generation details, not proof of PCS region.

## Important safety notice

The PCS, HV battery and 220 V mains can cause fatal injury. This firmware is
not a safety controller. Use correctly rated fuses, isolation, interlocks,
precharge, hardware current limiting and an independent emergency disconnect.
Do not apply AC/HV until the logic-level outputs and power circuit have been
verified at the PCS connector.

## What was fixed

- Correct `0x264` AC-current decoding: the attached trace is 220 V, 0 A, 0 W,
  not 51 A.
- Correct `0x2B2` power-request buffer and DLC handling.
- Correct `0x22A` DLC, support/charge mode and measured-HV packing.
- Correct post-2020 `0x23D` to the four-byte charge-status format required by
  the captured 32 A single-phase PCS. The later `3.txt` capture confirms the
  PCS receives that format, although it also proves `0x23D` was not the cause
  of the remaining 8 A clamp.
- Removed the synthetic transmitted `0x2C4` that collided with PCS telemetry.
- Added a consistent 16 A live request using `0x13D`, the `0x21D` pilot,
  `0x23D` and `0x2B2`. The independent cable/UI capability fields remain at
  32 A/48 A so they cannot create a hidden lower clamp inside the PCS.
- Added the runtime `CHGcurrentSetpointA` register (0...16 A), synchronized
  current-limit frames and an AC-voltage-based `0x2B2` power request.
- Added automatic start after stable AC is confirmed by the PCS.
- Added CAN/HV/AC/PCS fault states and `0x424` alert diagnostics.
- Added decoding of the PCS per-phase current request from `0x204`. The
  `I set/PCS` display now shows the commanded current and the current the PCS
  actually requests internally (for example `16/8`).
- The `5.txt` capture proves RC5 really doubled `0x2B2` to 6976 W, while PCS
  still requested and drew exactly 8.0 A. The factor-of-two hypothesis was
  therefore false; RC6 removes that compensation and restores standard
  1 W/bit power requests.
- RC6 switches the two linked charge-port identity fields together:
  `0x21D` uses the valid EU line-charge pilot value `0x2D`, and `0x25D` uses
  `CP_type = Euro IEC` (`0xD9`). This is the only command-side experiment in
  RC6; all live limits remain 16 A.
- The `6.txt` capture exposed the concrete remaining protocol error:
  `0x3A4` reports `CAN rationality` and `UI_MIA` while the controller sends
  obsolete `0x333` DLC 4. RC7 sends `04 30 29 07 00` with DLC 5, matching the
  independently verified fix for newer PCS firmware.
- RC7 decodes the relevant `0x3A4` alert bits, keeps both matrix pages, and
  stores the complete last `0x424` payload/DLC for SWD diagnostics.
- Added correct `0x2B4` DCDC rail decoding. The same capture reports 14.2 V and
  12.1 A, proving DCDC was active even though the legacy `0x224` field was zero.
- Added a CAN overcurrent guard: a fresh measured current more than 2 A above
  the setpoint immediately disables CHG and latches control state 14. Hardware
  overcurrent protection is still mandatory.
- Added stateful capture of the multiplexed `0x76C` phase A/B/C charger-output
  pages for SWD debugging. An aggregated CAN export keeps only one mux page
  and cannot provide these three measurements by itself.
- Replaced the misleading `USpcs` switch with two explicit settings:
  `PCS_CHARGE_PORT_PROFILE` for region/pilot semantics and
  `PCS_2B2_START_SHORT` for frame length.
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

Verified RC7 size (`text=46,824`, `data=128`, `bss=3,616`):

- Flash: 46,952 / 65,536 bytes
- RAM: 3,744 / 20,480 bytes

Current BIN SHA-256 (RC7):

```text
dea6cbcbdd47ca4e2f7a88da3e5d9be6e416299fb9a5c15dd8977a8ad720fea8
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

- the display header is `PCS RC7 EU` and contains `I set/PCS=` and `P/CAN=`;
- CAN `0x22A` has DLC 8 and starts with `00 0B`;
- CAN `0x2B2` has DLC 5 for the current tested PCS configuration.
- CAN `0x333` has DLC 5 and equals `04 30 29 07 00`.

## Documentation

- [`DEBUGGING.md`](DEBUGGING.md): settings, state codes, expected CAN frames,
  first-start procedure and trace-capture checklist.
- [`output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf`](output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf): earlier Russian report. Per the current test workflow it has deliberately not been updated for RC7; `DEBUGGING.md` is authoritative until the hardware test succeeds.

## Protocol references

- [Damien Maguire: Tesla Model 3 Charger](https://github.com/damienmaguire/Tesla-Model-3-Charger)
- [US-tested M3_PCS_V6 controller](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/M3_PCS_V6.ino)
- [STM32 PCS controller and alert decoding](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/stm32-M3-PCS/src/PCSCan.cpp)
- [Model 3 CAN DBC](https://github.com/joshwardell/model3dbc/blob/master/Model3CAN.dbc)
