---
title: "AGENTS.md — ambit-iot firmware"
date: 2026-08-04
generated_by: skills-for-architects
---

# AGENTS.md — ambit-iot firmware

ESP32-C3 (Arduino framework via PlatformIO) firmware for the AMBIT leaf-sensor board:
ADPD6100 (fluorescence), AS7341 (spectral/PAR), MLX90632 (leaf temperature). One image
serves every host at runtime over UART0: an Ambyte datalogger (binary FSM over the FFC),
the openJII app and the Calibratron (text console), and the openJII JSON envelope.

## Commands

- Build: `pio run` (single env `ambit`)
- Flash: add `-t upload`; serial monitor: 115200 8N1
- Linux/macOS quirk: `platformio.ini` sets `build_dir = ${sysenv.LOCALAPPDATA}/pio_build/…`
  (Windows OneDrive/AV workaround). Off Windows, export something sane first, e.g.
  `LOCALAPPDATA=$HOME/.cache pio run`.
- Version is build-injected by `tools/version.py`: `$AMBIT_RELEASE_VERSION` (CI) →
  `git describe` (dev) → `0.0.0-dev`. Never hand-edit a version literal; everything
  (cmd 33/2, `hello`, JSON identity) reports the same injected value.
- Releases: semantic-release keyed on Conventional-Commit PR titles (squash-merge).
  `pr.yml` validates the title, builds, and uploads the artifact; `release.yml` reuses
  that artifact bit-identically and publishes the GitHub release (app image,
  bootloader, partitions, boot_app0, `manifest.json` with offsets + sha256). The
  Calibratron flashes devices from these release assets.
- No automated test suite. The regression gate for anything touching `run_esp.cpp`,
  the `data_utils.cpp` FSM, or the `loop()` router is the hardware conformance
  procedure in `plans/HW_CONFORMANCE.md` (byte-diff against a real Ambyte).

## Architecture — one CORE + one CONFIG + thin transport adapters, routed at runtime

`plans/MERGE_PLAN.md` is the historical design doc for the fork merge (its §10/§11
build-variant split was superseded by the single image once every host landed on the
same UART0); read it before structural changes. The spine:

```
wire bytes → [adapter: decode + undo wire conventions] → normalized args → core op → [sink]
```

- **First-byte router** (`ambit-1.ino` `loop()`): >127 → binary FSM branch (every
  frozen framing byte is >127); `{`/`[` → openJII JSON envelope (`ojii::poll`); other
  printable → legacy text console (`do_command`). `ojii::poll()` must never see binary
  traffic (it consumes bytes and drops >127 junk silently) — peek, then route. While
  `ojii::busy()` the envelope owns the stream.
- **Sticky host latch** (`HOST_UNKNOWN/BINARY/TEXT`): BINARY/UNKNOWN idle in light
  sleep (the Ambyte re-sends its wake until acked, so sleep-lost bytes are safe);
  TEXT never sleeps (text/JSON hosts don't retry; UART sleep-wake eats in-flight
  bytes) and releases after 2 min idle. The latch also gates the 0x85 idle heartbeat.
- `src/core.{h,cpp}` — transport-agnostic command core. Takes normalized typed args
  (gains 0-indexed, real widths), does **no** wire I/O: no Serial/printf/JSON.
- `src/config.h` + PAM globals — single config (`adpd_*_config`, `adpd_mode`) with
  "set, then apply on run" semantics. `CONNECTION_TYPE` (PLOTTING / AMBYTE / COMPUTER)
  selects the measurement output sink; the JSON envelope path uses the `json_output`
  flag through `run_arr_type1` instead.
- Adapters:
  - `src/run_esp.cpp` — binary/Ambyte adapter. **Frozen wire contract** (below).
  - `src/do_command.h` — text/console verbs (`hash()` dispatch switch). This is the
    dialect the openJII platform's ambit driver and the Calibratron actually speak.
  - `src/frontend_json.cpp` — openJII envelope handlers (snapshot tree-builders +
    streamed `arrun`), registered from `setup()`.
- `lib/openjii_proto/` — device-agnostic openJII serial protocol module. Devices
  register snapshot/stream handlers; `busy()`/`reset()` exist for router coexistence.
  Bare-line LINE mode is intentionally unreachable in this firmware (no live consumer;
  bare lines belong to the text console).
- `src/PAM.cpp` — measurement engine (array runs, MPF, triggers, env logging);
  `src/data_utils.cpp` — FSM data framing + checksums; `src/nvs1.*` — calibration,
  metadata, and fw-info structs persisted in NVS; `src/src/` — sensor drivers.

## The frozen binary wire contract (do not break)

The binary path is a wire contract with shipped Ambyte firmware. The source of truth is
the Ambyte repo (`ambyte-iot`): `components/uart_sensors/uart_sensors.c` and
`components/device_commands/include/ambit_protocol.h`. MERGE_PLAN §2 lists every frozen
element. Rules:

- Refactor behind the wire freely; never change the bytes. Obscure-but-frozen encodings
  stay: 1-indexed gains on the wire (stored −1), base-128 MPF length split
  (`cmd[1]<<7 | cmd[2]`), byte-sum-mod-256 data checksum, struct sizes 140/48/248 with
  `eof_mark==2025`.
- Wire conventions are decoded **in the adapter, never the core** — the core only ever
  sees normalized values.
- The cmd 33/2 fw-info struct is 48 bytes with fixed offsets; `hw_rev` lives in the
  first formerly-reserved byte (old readers never looked at it). Do not move fields.
- Any change to the binary path or the router re-runs `plans/HW_CONFORMANCE.md`.
- UART0 doubles as the log console; setup silences all ESP logging, and nothing may
  `printf` outside explicit console replies — ASCII in front of a binary frame breaks
  the Ambyte session. The boot banner + NVS dump predate the freeze and are tolerated;
  do not add more.

## Version reporting (single source)

`tools/version.py` injects `AMBIT_FW_VERSION` (string) + `AMBIT_FW_MAJOR/MINOR/BATCH`
(uint8 fields of the cmd 33/2 struct). Reported by: binary cmd 33/2, text `hello`
(`NEW <name> Ready FW:<version>` — "NEW"/"Ready" are matched by the app driver and
Calibratron, the FW token is additive), and the JSON `hello`/envelope identity.

## Conventions

- Heavy rationale comments are the house style — a constraint's *why* lives at its
  definition (wire freezes, sleep thresholds, build-dir workaround). Keep it that way.
- Conventional Commits for commits and PR titles: `type(scope): description`.
  PR titles drive semantic-release (feat → minor, fix/perf → patch).
- License: CERN-OHL-S-2.0 (hardware-adjacent open license — keep headers/attribution).
- Vendored driver libs under `src/src/` (as7341, adpd, mlx90632) are third-party
  imports; don't reformat or "clean up" wholesale.
