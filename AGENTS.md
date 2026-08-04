---
title: "AGENTS.md — ambit-iot firmware"
date: 2026-08-04
generated_by: skills-for-architects
---

# AGENTS.md — ambit-iot firmware

ESP32-C3 (Arduino framework via PlatformIO) firmware for the AMBIT leaf-sensor board:
ADPD6100 (fluorescence), AS7341 (spectral/PAR), MLX90632 (leaf temperature). An AMBIT is
either carried by an Ambyte datalogger (binary FSM over UART0/FFC) or driven directly by
the openJII app/host (text LINE + JSON over the same UART). One source tree builds both
product variants.

## Commands

- Build field variant: `pio run -e ambyte` — binary FSM protocol, light-sleep idle
- Build app variant: `pio run -e cloud` — openJII LINE/JSON protocol (default env)
- Flash: add `-t upload`; serial monitor: 115200 8N1
- Linux/macOS quirk: `platformio.ini` sets `build_dir = ${sysenv.LOCALAPPDATA}/pio_build/…`
  (Windows OneDrive/AV workaround). Off Windows, export something sane first, e.g.
  `LOCALAPPDATA=$HOME/.cache pio run -e ambyte`.
- No automated test suite. The regression gate for anything touching `run_esp.cpp` or the
  `data_utils.cpp` FSM is the hardware conformance procedure in `plans/HW_CONFORMANCE.md`
  (byte-diff against a real Ambyte).

## Architecture — one CORE + one CONFIG + thin transport adapters

`plans/MERGE_PLAN.md` is the authoritative design doc (this tree is the merge of two
diverged forks); read it before structural changes. The spine:

```
wire bytes → [adapter: decode + undo wire conventions] → normalized args → core op → [sink]
```

- `src/core.{h,cpp}` — transport-agnostic command core. Takes normalized typed args
  (gains 0-indexed, real widths), does **no** wire I/O: no Serial/printf/JSON.
- `src/config.h` + PAM globals — single config (`adpd_*_config`, `adpd_mode`) with
  "set, then apply on run" semantics; setters mark it dirty, the run re-applies it.
  `CONNECTION_TYPE` (PLOTTING / AMBYTE / COMPUTER) selects the output sink.
- Adapters:
  - `src/run_esp.cpp` — binary/Ambyte adapter. **Frozen wire contract** (below).
  - `src/do_command.h` — text/console verbs (`hash()` dispatch switch).
  - `src/frontend_cloud.cpp` — openJII adapter (snapshot tree-builders + streamed
    measurement values), gated on `VARIANT_CLOUD`.
- `lib/openjii_proto/` — device-agnostic openJII serial protocol module (first-byte
  LINE-vs-JSON detect, envelope with flat `set`, dot-path queries, §9 error taxonomy).
  Knows nothing about AMBIT; devices register snapshot/stream handlers.
- `src/PAM.cpp` — measurement engine (array runs, MPF, triggers, env logging);
  `src/data_utils.cpp` — FSM data framing + checksums; `src/nvs1.*` — calibration,
  metadata, and fw-info structs persisted in NVS; `src/src/` — sensor drivers.
- Variant gating: `-DVARIANT_AMBYTE` / `-DVARIANT_CLOUD` (see `platformio.ini`). The
  ambyte image must link **no JSON** — ArduinoJson code exists only behind
  `VARIANT_CLOUD` guards.

## The frozen binary wire contract (do not break)

The `ambyte` path is a wire contract with shipped Ambyte firmware. The source of truth is
the Ambyte repo (`ambyte-iot`): `components/uart_sensors/uart_sensors.c` and
`components/device_commands/include/ambit_protocol.h`. MERGE_PLAN §2 lists every frozen
element. Rules:

- Refactor behind the wire freely; never change the bytes. Obscure-but-frozen encodings
  stay: 1-indexed gains on the wire (stored −1), base-128 MPF length split
  (`cmd[1]<<7 | cmd[2]`), byte-sum-mod-256 data checksum, struct sizes 136/48/248 with
  `eof_mark==2025`.
- Wire conventions are decoded **in the adapter, never the core** — the core only ever
  sees normalized values.
- Any change to the binary path re-runs `plans/HW_CONFORMANCE.md` before merge.
- UART0 doubles as the log console; the ambyte variant silences all ESP logging at boot
  so ASCII can't interleave into the binary stream.

## Version reporting (currently three divergent copies — keep in sync)

- `src/nvs1.h` `MAJOR/MINOR/BATCH_VERSION` — the compiled fw version, reported over the
  binary wire via info cmd 33 subtype 2; bump `BATCH_VERSION` every hosted build so a
  post-OTA version read confirms the update landed.
- `src/frontend_cloud.cpp` `cmd_hello` hardcodes its own `"version"` string.
- The calibratron repo pins a minimum ambit fw version on its side.

## Conventions

- Heavy rationale comments are the house style — a constraint's *why* lives at its
  definition (wire freezes, sleep thresholds, build-dir workaround). Keep it that way.
- Conventional Commits for commits and PR titles: `type(scope): description`.
- License: CERN-OHL-S-2.0 (hardware-adjacent open license — keep headers/attribution).
- Vendored driver libs under `src/src/` (as7341, adpd, mlx90632) are third-party
  imports; don't reformat or "clean up" wholesale.
