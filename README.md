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
- Removed the synthetic transmitted `0x2C4` that collided with PCS telemetry.
- Added consistent 16 A US port emulation using `0x13D`, `0x21D`, `0x23D` and
  `0x25D`.
- Added automatic start after stable AC is confirmed by the PCS.
- Added CAN/HV/AC/PCS fault states and `0x424` alert diagnostics.
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

Verified size:

- Flash: 44,140 / 65,536 bytes
- RAM: 3,592 / 20,480 bytes

Current BIN SHA-256:

```text
1b3204d3d8e02f81f05b1b728655b7b2b1a937704ddc88547d7fe2540248b8f0
```

## Documentation

- [`DEBUGGING.md`](DEBUGGING.md): settings, state codes, expected CAN frames,
  first-start procedure and trace-capture checklist.
- [`output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf`](output/pdf/tesla_model_3_pcs_can_fix_and_test_guide.pdf): complete Russian PDF report.

## Protocol references

- [Damien Maguire: Tesla Model 3 Charger](https://github.com/damienmaguire/Tesla-Model-3-Charger)
- [US-tested M3_PCS_V6 controller](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/M3_PCS_V6.ino)
- [STM32 PCS controller and alert decoding](https://github.com/damienmaguire/Tesla-Model-3-Charger/blob/master/Software/stm32-M3-PCS/src/PCSCan.cpp)
- [Model 3 CAN DBC](https://github.com/joshwardell/model3dbc/blob/master/Model3CAN.dbc)
