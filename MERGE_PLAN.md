# Ambit Firmware Merge Plan (v2) — built around dialect + config unification

Collapse the two diverged Ambit ESP32-C3 forks into **one** source tree that builds
both product variants from one codebase:

- **`ambyte`** variant — binary FSM protocol over UART0/FFC, idle light-sleep, the
  field/companion-board use case (today = `fw_new`, was `ambitPIO`).
- **`cloud`/`usb`** variant — text + JSON over native USB-CDC, feeds the OpenJII
  platform / phone app (today = `fw_appCompatible`, was `openJII`).

Sources:
- `T` = **`fw_new`** (ambyte/binary line, this repo) — the **trunk**.
- `A` = **`fw_appCompatible`** (cloud/JSON line) — a *capability fork*: it added the
  JSON/line frontend but **deleted** the binary frontend, Wrench, and power
  management. Donor of the text/USB frontend only.
- `H` = **`ambyte-iot-ludo`** (the datalogger) — **not** ambit FW. It is the
  *authoritative spec* of the binary wire contract ([uart_sensors.c], [ambit_protocol.h]).
  Treated here as the frozen reference, never modified.

Back-compat of the **text** path is not a constraint; the **binary** path is a frozen
wire contract (§2). Git holds history. Reviewed 2026-06-05.

---

## 0. Revision history

### v3 (current) — openJII comms layer

- **Core returns typed data, not a JSON tree.** The shared core is JSON-free; the JSON
  tree is built only in the app-side (cloud) adapter, so the `ambyte`/binary image links
  no JSON (§5, §7c).
- **The openJII text/JSON layer is a device-agnostic shared module** (§7d), built **fresh**
  to [CommunicationProtocolOpenJIISerial.md] — CO2Dot is the doc's named reference but is
  too old to lift (D5).
- **Envelope `set` is flat** (one element per executed command/repeat). **Snapshot →
  tree; measurement → one streamed JSON value** per `set` position (D6).
- **Comms-layer workflow:** implement to the spec doc → HW-test against the real app (the
  oracle) → revise the spec doc. The binary path is *not* iterated this way — it's frozen.

### v2 — unification spine

This revision is built around the **command-dialect + config unification** as the
headline task, not as one step among many. Specifically:

- **D1 (Wrench): DELETE, not gate.** Tracing shows Wrench is reachable from exactly
  one text-console entry point and exposes zero unique commands (§8). It goes away
  entirely, behind a single stakeholder check.
- **The spine is now: one CORE + one CONFIG + three thin transport adapters** (§4–§7).
  Every other task (JSON re-graft, PAM sink merge, framing fix, power mgmt) hangs off
  that spine.
- **Constraint (c) is promoted to its own section (§2)** with an explicit, verified
  list of frozen wire elements and a conformance gate, because the deployed ambyte
  (`H`) already pins several of them — including the data checksum, which makes
  `fw_new`'s "checksum fix" **mandatory for compatibility**, not optional.

---

## Where we are — 2026-06-05 (resume here)

Two build variants from one tree (`platformio.ini`): **`ambyte`** (binary FSM over
UART0/FFC, HW-verified field path) and **`cloud`** (openJII LINE+JSON over UART0 — the
test HW has no USB-C; the native-USB-CDC flags are kept in a comment for a future PCB).

**Done + HW-verified** (commits through `51a7ffc`):
- Wrench deleted; the 3 config copies unified; transport-agnostic CORE (`core.*`); text +
  binary frontends thin over it; `pam_send_results` shared; echo (#16) fixed; nvs checksum
  (#6); dead-code purge (#3); cheap correctness (#8/#9/#10). Binary path byte-conformant vs
  a real ambyte (`HW_CONFORMANCE.md`).
- **openJII module `lib/openjii_proto/`** (device-agnostic): first-byte mode detect, LINE
  (bare scalar / JSON container + dot-path + `keys()`), JSON envelope (flat `set` + frame
  token), §9 errors, receiver. Snapshot API `on()` + stream/measurement API `on_stream()`.
- **Cloud variant** (`src/frontend_cloud.cpp`): snapshots hello/temp/get_par/PAR (verified)
  and **`arrun` measurement-in-envelope** (verified — runs `run_arr_type1`'s new
  `json_output` path, emits `{"ENV":[..],"Fluo":[..],"Fluoref":[..],"SUN":[..],"leaf":[..]}`).
  ENV channels byte-identical to fork A's `send_json`; ENV decoded from fw_new centi-°C.

**Next (pick up here):**
1. **Expand command coverage.** Snapshots: `set_currents`/`set_gains`/`set_pd_gain`/`baseline`/
   `battery`/`set_act`/`set_name`/`set_emit`/`set_spec`/`clean_nvs`/`reboot`/`check` — each a
   one `on()` tree-builder calling the CORE. Measurements: `q`/`w`/`mpf`/`ww`/`ff` — give
   `run_trigger_spacer` (and `MPF`) the same `json_output` sink `run_arr_type1` has, register
   via `on_stream`.
2. **Confirm formats against the app (D6/D2).** ENV is `{"temp_c":<°C>}` per point now (fork A
   had `{"t_ms":..,"temp_c":..}` + step marks, dropped in fw_new). Adjust `send_env_json` if the
   app needs the old shape. Check ArduinoJson float precision (trims trailing zeros vs the
   spec's fixed 4 dp) is tolerated.
3. **Test the rest of LINE mode on HW:** dot-path queries, `keys()`, the error taxonomy.
4. **Open decisions:** D2/D3 (env centi-°C + ambient — consumer sign-off), D6 (measurement
   shape — partly resolved by the working `arrun`).

---

## 1. Headline: the disease and the cure

**Disease.** The same ~25 logical operations are implemented as **four parallel
command parsers**, each with its own argument decoding *and* its own copy of the
device config:

| Parser | File | Config it mutates |
|---|---|---|
| text/computer | [do_command.h] `hash()` switch | global `adpd_*_config` |
| binary/ambyte | [run_esp.cpp] `cmd_arr[0]` switch | `adpd_*_config_local` (file-static) |
| Wrench bridge | `do_c.cpp` native fns | `do_c.cpp` statics (3rd copy) |
| JSON/line (fork `A`) | [commands.cpp] `startsWith` chain | global `adpd_*_config` |

Adding one command means editing up to four places; the same setting lives in three
structs that can silently diverge (set currents via text, run via binary → different
values applied). **This duplication is the root cause of the forks themselves.**

**Cure.** Split every operation into:

```
  wire bytes ─► [thin transport adapter] ─► normalized args ─► CORE op ─► result ─► [sink]
                         │                                        │
              (decodes that transport's                  one implementation,
               own wire convention)                      one CONFIG struct
```

- **One CORE** ([core.cpp]): one function per logical operation, taking *normalized*
  typed args and writing to a *sink*, never touching `Serial`/`printf` directly (§5).
- **One CONFIG** ([config.h]): a single `AmbitConfig` struct every path reads/writes (§6).
- **Three thin adapters**: binary (frozen wire), text-line, JSON (§7). Each adapter's
  only job is wire⇄normalized-args translation + choosing the sink. No measurement or
  config logic lives in an adapter.

Note the realistic scope: because the binary wire is frozen (§2), we **do not** merge
the three *parsers* into one parser. We collapse the three *cores* and three *configs*
into one each, and make all three parsers thin shims over them. That is the achievable,
high-value unification.

---

## 2. The frozen binary contract (constraint c)

The binary path is a wire contract with **shipped ambyte firmware** (`H`). The contract
is defined by what [uart_sensors.c] / [ambit_protocol.h] already implement. Refactor
*behind* it freely; do **not** change the bytes. De-obscuring is allowed only on the
text/USB path.

**Frozen elements (verified against `H`):**

| Element | Value / encoding | Source of truth |
|---|---|---|
| Wake / ack | host `0xAA ×3` → ambit `0x80`; boot-idle `0x85` | [uart_sensors.c:32-37] |
| Cmd frame | header `0xA0` + 8-byte `cmd[8]` (+ optional extra) → `0xA1` done → … → `0xF0` end | [uart_sensors.c], [run_esp.cpp] |
| Command IDs | 1,2,10,20,21,31,32,33,34,37,4,5,6,17,18 | [ambit_protocol.h:15-31] |
| Info subtypes | 33 → 1=calib, 2=fw, 3=metadata | [ambit_protocol.h:33-37] |
| Response structs | calibration ~136 B, fw_info ~48 B, metadata ~248 B (ESP32 default alignment, `eof_mark==2025` to accept) | [ambit_protocol.h:39-87] |
| Raw responses | spec = 24 B (12×u16, last 4 = float PAR); temp = 4 B (2×i16 ×10); temp_raw = 14 B (7×i16) | [run_esp.cpp], [ambit_protocol.h:83-87] |
| FSM data handshake | wake 211, awake 210, data-hdr 212, len-marker 150, ready 200, data-pass 180, reset 222 | [data_utils.h:9-16], [uart_sensors.c:39-46] |
| FSM header checksum | sum of header bytes `[0..6]` == byte `[7]` | [uart_sensors.c:265-269] |
| **FSM data checksum** | **byte-sum of every payload byte mod 256** (`u32_byte_sum`) | [uart_sensors.c:323-330] |
| MPF length encoding | **base-128 split**: `cmd[1]=len>>7`, `cmd[2]=len&0x7F` | [device_commands.c:1027-1028], [run_esp.cpp:92] |
| Gain wire convention | cmd 1 gains are **1-indexed** on the wire (1..6), stored `−1` | [run_esp.cpp:58-63] |

**Consequences for the merge:**

1. **The byte-sum checksum is required, not optional.** `H` verifies it; `fw_original`'s
   low-byte-only checksum would be rejected. `fw_new` already byte-sums — keep it. This
   alone confirms `fw_new` (not `fw_original`) is the correct trunk for the binary path.
2. **Obscure-but-frozen encodings stay** (base-128 MPF length, 1-indexed gains, struct
   alignment). We *document* them at the adapter boundary; we do not "clean them up."
3. **Per-transport wire conventions are decoded in the adapter, not the core.** The CORE
   `set_gains` takes 0-indexed normalized gains; the binary adapter subtracts 1, the
   text/JSON adapters pass through. Same for the base-128 length.
4. **Semantic (non-wire) contracts are a separate freeze (D2/D3).** `H`'s C layer
   forwards env/data arrays **raw** (it does not decode env temperature), so changing
   the env-temp *encoding* won't break the wire — but it changes what the *consumer*
   (Lua/cloud) must parse. Freeze those by **decision + consumer sign-off**, not by the
   conformance test.

---

## 3. Decisions

| # | Decision | Resolution |
|---|---|---|
| **D1** | Wrench scripting VM (`wrench.*` 17.4k LOC + `do_c.cpp`) | **DELETE entirely** (§8). One text-only entry point, zero unique commands. **Stakeholder check (the only gate):** confirm nobody sends multi-line `C…?` *scripts* (loops/control-flow) from the bench console today. Single primitive calls → pure win. Real scripting → that workflow moves host-side (the ambyte and app already drive primitive sequences themselves). |
| **D2** | Env-temperature wire/format | **Keep `fw_new` centi-°C** plain `int16` (de-obscured from the old `time<<16\|type<<12\|(mlx+20)*20`). Step-mark events are dropped. **Consumer sign-off required:** verify (a) the cloud/Lua consumer tolerates a plain centi-°C series, and (b) the dropped step-marks did not feed `H`'s `mark_event_*` subsystem ([device_commands.c:78]). |
| **D3** | Ambient (sun/leaf) correction | **Keep `fw_new`** offset-corrected (`>65000 ? x−65000 : 0`). If the cloud backend needs *raw*, branch at the `put()` on `CONNECTION_TYPE==CLOUD` only. **Verify with the data consumer before locking.** |
| **D4** | `SunRint`/`LeafRint` gain fields + `read_light_env()` (fork `A` only) | Keep **iff** the cloud/app variant needs the `set_pd_gain` / `read_light_env` snapshot. Otherwise drop the two fields and the function. Default: **keep** (the app uses `set_pd_gain`). |
| **D5** | openJII text/JSON implementation source | **Build the comms layer fresh** to [CommunicationProtocolOpenJIISerial.md] as a **device-agnostic shared module** (§7d). CO2Dot is the doc's named reference but is **too old to reuse** — do not lift its code. Reuse fork `A`'s parser/framing only as a starting skeleton, never its hand-printed command bodies. |
| **D6** | openJII measurement (`arrun`) in-envelope JSON shape | **Resolve by HW test, not now.** Observed app expectation is `arrun → [{…},{…}]` (one element per produced array). It appears to conflict with **command-as-root** (§2.1 of the spec): a bare array `[{…},{…}]` is command-as-root-compliant, a `{"arrun":[…]}` wrapper is not. The app is the oracle — confirm the exact shape on hardware, then revise the doc. |

**Comms-layer workflow (D5/D6).** For the openJII text/JSON layer **only**: implement to the
spec doc → HW-test against the real app → revise the spec doc to match what the app
actually accepts. The binary/ambyte path is **not** iterated this way — it is frozen (§2).

---

## 4. Target architecture

```
                         ┌─────────────────── one CORE (core.cpp) ───────────────────┐
                         │  set_currents · set_gains · set_pd_gain · config_detector  │
 host bytes              │  run_array · run_mpf · external_trigger · flash_trigger    │
   │                     │  get_par · get_temp · get_temp_raw · baseline             │
   ├─ binary adapter ───►│  set_actinic · set_name · set_metadata · set_emit/spec    │
   │  (run_esp.cpp,      │  blink · nvs_clean · nvs_update · info · check · reboot    │
   │   FROZEN wire §2)   │  read_light_env (D4)                                       │
   │                     └───────────────┬──────────────────────────┬────────────────┘
   ├─ text-line adapter ─────────────────┤                          │
   │  (frontend_text.cpp)                ▼                          ▼
   │                          one CONFIG (AmbitConfig)      output via sink:
   └─ json adapter ─────────►   currents + gains +            BINARY  → fsm_send_esp / raw writes
      (frontend_json.cpp,        pd_gain + mode               TEXT    → send_serial / CSV
       cloud/usb only)                                        JSON    → send_json
                                                              PLOT    → CSV printf
```

- **Sink selector = `CONNECTION_TYPE`** (already the right seam in [config.h]). Extend
  the enum with `CLOUD`; fork `A`'s `json_output` bool and `T`'s per-call branching both
  collapse onto it.
- **Core functions take an explicit sink/connection arg or read the global
  `CONNECTION_TYPE`** — but never hardcode a transport.

---

## 5. The unified CORE (`core.cpp` / `core.h`)

Extract each command body (currently duplicated across the four parsers) into **one**
function. Rules:

- Parameters are **normalized typed values** (0-indexed gains, real `uint16` lengths) —
  the adapter has already undone its wire convention.
- **Returns typed native data (structs/scalars), never formatted text and never a JSON
  tree.** No `Serial`/`printf`/ArduinoJson inside the core — so the `ambyte` image links
  no JSON. Snapshot ops return by value/out-param (e.g. `get_par(u16 spec[10]) -> float`);
  the adapter renders it (binary bytes, LINE text, or the JSON tree built in the cloud
  adapter §7c).
- **Streaming measurements** write raw frames through `dataclass`/the streaming sink,
  keyed by `CONNECTION_TYPE`; the adapter decides framing (binary FSM bytes vs a streamed
  JSON array element). The core never builds a tree for a run — runs are thousands of
  points.
- Returns a typed status, so adapters can format errors per dialect (binary `CMD_END`,
  text `BAD COMMAND`, JSON `{"error":…}`).

Operation set (the single source of truth; the §11 coverage matrix maps each dialect
onto these):

```
set_currents(u8 i620, u8 i720, u8 ir)
set_gains(u8 fluo, u8 fluoRef, u8 ir, u8 irRef, u8 sun, u8 leaf)      // 0-indexed
set_pd_gain(u8 sunTia, u8 sunRint, u8 leafTia, u8 leafRint)           // D4
config_detector(...)                  // conf_slow_FR_1 wrapper
run_array(u8 len, const u8* arr, u8 persist, bool allow_int)
run_mpf(u16 length, u8 interval, bool change_act, u8 act)
external_trigger(), flash_trigger(u32 gate, u32 dt, u16 num)
get_par(u16 spec[10]) -> float
get_temp(double* leaf, double* chip)
get_temp_raw(...) ; baseline(u32 ret[6], bool save)
set_actinic(float), set_name(const char*), set_metadata(const metadata_t*)
set_emit(double), set_spec(float)
blink(u8 id, u8 intensity)
nvs_clean(), nvs_update_scalar(...), nvs_update_array(...)
info(u8 subtype) ; check() ; reboot()
read_light_env()                      // D4
```

---

## 6. The unified CONFIG (`AmbitConfig`)

Collapse the three copies into one global struct, read/written by every adapter:

```c
struct AmbitConfig {
    struct { uint8_t i620, i720, ir; bool init; } current;
    struct { uint8_t fluo, fluoRef, ir, irRef, sun, leaf;
             uint8_t sunRint, leafRint;       // D4
             bool init; } gain;               // ALL 0-indexed (normalized)
    uint8_t adpd_mode;                        // ADPD_CONFIG_MODE
};
extern AmbitConfig g_cfg;
```

- Delete `adpd_current_config_local` / `adpd_gains_config_local` ([run_esp.cpp:21-22])
  and the `do_c.cpp` statics (gone with Wrench).
- **Preserve "set then apply on run" semantics.** Today the text path sets the global
  and marks `adpd_mode = MPF_MODE` ("not applied"); the binary path applies at
  `config_detector`/`run_array` time. Unify on: setters mutate `g_cfg` only; the
  measurement core calls `config_detector` from `g_cfg` when `adpd_mode` requires it.
- **Init/default handling once.** Fork `A` force-sets `init=true` to silence the
  "preset not initialized" logs ([commands.cpp:267-268]); the binary path warns. Pick one
  rule (default-initialized config is valid) and apply it in the core.
- *Checkpoint after this step:* set currents/gains via the text adapter, run via the
  binary adapter → identical values applied.

---

## 7. The three adapters

### 7a. Binary adapter (`run_esp.cpp`) — FROZEN wire
- Keep the `cmd_arr[0]` switch and **every byte** it reads/writes (§2).
- Each case: decode wire → undo wire convention (gain `−1`, base-128 length) → call the
  **same core fn** the other adapters call → encode response in the frozen format.
- This is where §2's conformance gate applies.

### 7b. Text-line adapter (`frontend_text.cpp`, replaces `do_command.h`) — de-obscure freely
- Keep the verb names (`set_currents`, `arrun`, `q`, `ww`, `ff`, `get_par`, …) as the
  human/console surface; route each to a core fn.
- Replace the `hash()` dispatch (silent-collision risk, sweep #19) with a
  `strcmp`/`string_view` table if cheap; not a blocker.

### 7c. openJII text/JSON adapter (cloud/usb only) — built to the spec, not ported wholesale
- Implements [CommunicationProtocolOpenJIISerial.md]: first-byte mode detect (`{`/`[` →
  JSON envelope, else LINE), flat `set` envelope, command-as-root, the error taxonomy.
- Reuse fork `A`'s parser/framing ([commands.cpp]) as a **skeleton only**; **discard its
  hand-printed `Serial.printf("{…}")` bodies** — they are exactly the per-command
  format-strings the spec forbids. Each command instead supplies a **tree-builder** that
  consumes the core's typed result.
- **Snapshot commands → build a small tree → serialize.** **Measurement commands
  (`arrun`/`q`/MPF) → stream one well-formed JSON value** (an array of frame objects) into
  the command's single `set` position — no `stream_start`/`stream_end` side-objects, no
  raw bytes (those corrupt the envelope). Exact shape per **D6** (HW-verified).
- ArduinoJson is pulled in **only** for the cloud env (§11); the `ambyte` image stays JSON-free.
- Brings the two `A`-only verbs (`set_pd_gain`, `read_light_env`) into the core (D4).

### 7d. The shared openJII comms module (`lib/openjii_proto/`) — device-agnostic (D5)
The protocol machinery is **not** amBIT-specific. Factor spec §11's five pieces into a
reusable library other firmware can adopt verbatim:

1. **Receiver** — first-non-whitespace byte selects mode; string-aware brace/bracket-depth
   detects JSON completeness; overflow → `{"error":"rx_overflow"}`, JSON idle 1 s →
   `{"error":"json_timeout"}`.
2. **JSON serializer** (strict JSON) + **3. scalar-aware LINE serializer** (scalar → bare,
   container → JSON).
4. **Path resolver** — `.`-walk (map key / array index) + `keys()`, with the §9 error set.
5. **Envelope writer** — header → flat `set` (one element per executed command/repeat) →
   frame token.

Device-facing API is tiny: per command, the device registers a **tree-builder** (snapshot)
or a **stream-writer** (measurement) that consumes a core typed result. amBIT supplies
those; the module knows nothing about amBIT. Built **fresh** to the spec (CO2Dot too old —
D5); iterate the spec via HW tests (D5/D6 workflow).

**Wire-convention boundary (the key correctness rule):** normalization happens in the
adapter. Example — gains: binary subtracts 1 and validates 1..6; text/JSON pass the raw
0-indexed value. The core only ever sees 0-indexed gains.

---

## 8. Wrench deletion (D1)

Tracing (this session) established Wrench's full footprint:

- **Single entry point:** [do_command.h:60] `case hash("C")` reads chars until `?` and
  calls `do_c(c)`. Text/computer console only.
- **Never on the binary or app paths:** [run_esp.cpp] has no Wrench reference; `H` never
  sends a `C` script; fork `A` already shipped without Wrench.
- **Zero unique commands.** `do_c.cpp` registers 11 native fns (`set_currents`,
  `set_gains`, `config`/`detector_preset`, `set_arr`, `reset`, `run`, `run_mpf`,
  `get_par`, `get_temp`, `print`, `disp`) — every one is a thin wrapper over a core fn
  that already exists as a text verb **and** a binary opcode. (`idle` is defined but
  never registered — dead even inside the VM.)
- **It is the 3rd config copy:** `do_c.cpp`'s `set_currents`/`set_gains` write file-local
  statics independent of both other configs.
- **It owns the `arr_reset` overflow** (sweep #5, now gone with it) and an 8-slot `wr_run_arr[]`
  store referenced nowhere else.

**Action:** delete `src/src/wrench.{cpp,h}`, `src/do_c.cpp`, the `C` case, and all
`#include "src/wrench.h"`. The only capability lost is **on-device control-flow scripting**
(loops/variables/the 8 pre-loaded array slots) on the bench console — which moves
host-side, matching how the ambyte and app already operate.

**Wins:** −17.4k LOC from the default image, −1 config copy, −1 buffer overflow, −1 dead fn.

---

## 9. PAM / data_utils / sink unification (folded)

`PAM.cpp` diverged on two orthogonal axes — treat separately.

- **Axis A (transport):** fork `A`'s `json_output` bool + 5-arg `run_arr_type1` overload
  vs `T`'s `CONNECTION_TYPE` branches. **Resolution:** keep `T`'s structure; fold `A`'s
  JSON branch in as `case CLOUD` at each send site (~4: `run_arr_type1`,
  `external_trigger_run`, `MPF`, the env path). Delete the 5-arg overload + `json_output`.
- **Axis B (semantics):** the D2 centi-°C encoding and D3 ambient correction already
  live in `T` ([PAM.cpp] diffs confirmed). Keep them; rewrite fork `A`'s
  `send_env_json_decoded` for centi-°C (its old-format unpack is stale — sweep #7).
- `data_utils.cpp`: base = `T` (full FSM send + the **required** byte-sum checksum, §2).
  **Re-graft `send_json(const char*)`** from `A` for the `CLOUD` sink (~10 lines, decl in
  [data_utils.h]).
- While here, sweep the cheap co-located correctness items (uninit locals #10,
  leak-on-early-return #11, unaligned `*(float*)&cmd_arr[3]` → `memcpy` #12).
- *Out of scope:* the `dataclass` ring-buffer over-engineering (#18).

---

## 10. main loop + text-path framing (de-obscure, text only)

Base = `T`'s [ambit-1.ino] (it has the power management — **keep**: `ambit_light_sleep()`,
UART/GPIO sleep-wake config, sleep thresholds, `AMBIT_BOOT_IDLE` heartbeat, triple-tap-BOOT
reset ISR).

**Cherry-pick fork `A`'s framing** ([ambit-1.ino:89-111] in `A`) as the de-obscuring fix
for sweep #20: decide LINE vs JSON by the **first printable char** and drop junk
bytes while mode is `UNKNOWN`, instead of `T`'s glitch-sensitive `c > 127` MSB heuristic
that mis-fires on the DTR/RTS open glitch and emits `Unknown cmd`. This is a **text-path**
change and does not touch the binary contract.

Also (sweep #16): **stop echoing parsed bytes** in `serial_read_until`/`flush_serial`
— that echo currently injects bytes into the binary stream that `H` has to discard as
"debug/echo bytes" ([uart_sensors.c:382]). Move these helpers' single definition into
`serial.*` (kills the 4-TU re-declaration, #15).

**Variant separation over runtime auto-detect.** The ambyte is a separate product on a
dedicated FFC UART; the app is USB-CDC. Prefer **build-flag-separated frontends** (§11.5)
to runtime first-byte transport sniffing — cleaner and less bug-prone. Keep the framed/junk-
drop fix for robustness either way.

---

## 11. platformio.ini + build flags

```ini
[platformio]
default_envs = ambyte

[env]
platform = espressif32
board    = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
build_flags = -DDEBUG=0

[env:ambyte]            ; binary FSM over UART0/FFC, light-sleep
build_flags =
  ${env.build_flags}
  -DARDUINO_USB_CDC_ON_BOOT=0
  -DVARIANT_AMBYTE=1

[env:cloud]             ; text+JSON over native USB-CDC
build_flags =
  ${env.build_flags}
  -DARDUINO_USB_MODE=1
  -DARDUINO_USB_CDC_ON_BOOT=1
  -DUSB_VID=0x16C0
  -DUSB_PID=0x0483
  '-DUSB_MANUFACTURER="JII"'
  '-DUSB_PRODUCT="Ambit"'
  -DVARIANT_CLOUD=1
lib_deps = ${env.lib_deps}
  bblanchon/ArduinoJson    ; only the cloud image pays for JSON
```

- `VARIANT_AMBYTE` → binary adapter + FSM + light-sleep idle loop.
- `VARIANT_CLOUD` → JSON/line adapter + `send_json` + (D4) `read_light_env`.
- **No `ENABLE_WRENCH`** — Wrench is deleted (D1).
- Core (`core.*`, drivers, `AmbitConfig`) compiles in both.
- After changing envs, delete the per-env `sdkconfig.*` so it regenerates.

### Coverage matrix (every dialect → the §5 core ops)

| Core op | text (`do_command`) | binary (`run_esp` cmd) | JSON (`commands`) |
|---|:--:|:--:|:--:|
| set_currents | ✓ | 2 | ✓ |
| set_gains | ✓ | 1 | ✓ |
| set_pd_gain (D4) | — | (1) | ✓ |
| config_detector | (via run) | 10 | (via run) |
| run_array | `arrun` | 21 | `arrun` |
| run_mpf | `q` | 20 | `q` |
| get_par | ✓ | 31 | ✓ |
| get_temp / _raw | `temp` | 32 / 34 | `temp` |
| baseline | ✓ | 6 | ✓ |
| set_actinic | ✓ | 4 | ✓ |
| set_name | ✓ | 37 | ✓ |
| set_metadata | — | 37 | — |
| set_emit / set_spec | ✓ | — | ✓ |
| external / flash trigger | `ww` / `ff` | — | `ww` / `ff` |
| read_light_env (D4) | — | — | ✓ |
| nvs clean / update | `clean_nvs` | 17 / 18 | `clean_nvs` |
| info / check | `check` | 33 | `check` |
| reboot | `reboot` | — | `reboot` |
| blink | `a`/`aa` | 5 | `a`/`aa` |

---

## 12. Execution order (unification-first)

1. **Branch + scaffold.** Branch off `fw_new`. Add `CLOUD` to the `CONNECTION_TYPES`
   enum; add the two PIO envs + flags (§11). *Checkpoint: `ambyte` env still builds/runs.*
2. **Delete Wrench (D1, §8).** Remove `wrench.*`, `do_c.cpp`, the `C` case, includes.
   *Checkpoint: `ambyte` builds; every former Wrench op still reachable as a text verb.*
3. **Unify CONFIG (§6).** Collapse the three copies into `AmbitConfig`. *Checkpoint: set
   via text, run via binary → identical currents/gains applied.*
4. **Extract CORE (§5).** Move each command body into one core fn; point the text and
   binary adapters at the shared fns. Normalize wire conventions at the adapter boundary.
   *Checkpoint: every §11 row reachable from both `ambyte` and text, same numbers.*
5. **Consolidate serial helpers + drop echo (§10).** Single prototype in `serial.*`,
   no echo-on-parse. *Checkpoint: clean binary round-trip, no stray echoed bytes.*
6. **PAM + data_utils sink merge (§9).** Add `case CLOUD`, re-graft `send_json`, rewrite
   env-JSON for centi-°C, sweep #10/#11/#12. *Checkpoint: AMBYTE numbers byte-identical
   to pre-merge `fw_new`; CLOUD JSON validated vs backend schema.*
7. **Port JSON adapter (§7c).** Bring `commands.cpp`'s parser onto the core as
   `frontend_json.cpp` (cloud env only); delete `commands.cpp/.h`.
8. **main-loop framing (§10).** Adopt fork `A`'s first-printable-char framing on the text
   path; keep `fw_new`'s power management. *Checkpoint: DTR/RTS open glitch no longer
   emits `Unknown cmd`.*
9. **Build both envs; HW-test both variants (§14).**

---

## 13. Frozen-contract conformance gate (§2)

> **Runnable procedure: [HW_CONFORMANCE.md](HW_CONFORMANCE.md).** Step 4b (binary frontend
> → core) was **HW-verified byte-conformant on 2026-06-05** against a real ambyte. Keep the
> procedure as the reusable gate for later `run_esp.cpp` / FSM changes (Step 5+).

Before merging the binary adapter changes (step 4–5), establish a regression artifact and
keep it green through every later step:

- **Capture a golden byte trace** of the current `fw_new` ↔ `H` exchange for each binary
  command (or derive the expected frames from [uart_sensors.c] + [ambit_protocol.h]):
  wake/ack, the 8-byte cmd frame, `CMD_DONE`/`CMD_END`, raw response sizes, and a full FSM
  data round-trip (header checksum, **byte-sum** data checksum, length encoding).
- **Assert byte-for-byte identity** of the refactored binary adapter's output against the
  golden trace. Any diff = a contract break.
- Explicitly assert the obscure-but-frozen encodings: base-128 MPF length, 1-indexed
  gains, struct sizes (136 / 48 / 248), `eof_mark==2025`.
- The conformance gate covers the **wire**; D2/D3 semantic changes are gated separately by
  **consumer sign-off** (§3), since `H` forwards those arrays raw.

---

## 14. Risks & test checklist

| Risk | Mitigation |
|---|---|
| Refactoring the binary adapter breaks the wire contract with shipped ambytes | §13 golden-trace conformance gate, kept green every step; freeze fw_new binary behavior as reference |
| D2/D3 semantic changes alter measured values / cloud ingestion | Consumer sign-off before step 6; freeze AMBYTE numbers; branch raw-vs-corrected on `CLOUD` only if needed |
| Dropped step-marks were feeding `H`'s `mark_event_*` | Confirm with `H` owner before locking D2; else add a dedicated marker channel |
| CONFIG unification changes "when config is applied" semantics | Preserve set-then-apply-on-run; checkpoint 3 verifies cross-adapter equivalence |
| Wrench deletion removes a used scripting workflow | D1 stakeholder check first; move scripting host-side |

**Per-variant test checklist**
- [ ] `ambyte`: full wake→length→data FSM round-trip to a real ambyte; **byte-sum checksum OK**.
- [ ] `ambyte`: idle current drops to light-sleep between commands; triple-tap BOOT resets; UART wakes within threshold.
- [ ] `cloud`: line + JSON over USB-CDC; `set_pd_gain`/`read_light_env` (if D4=keep).
- [ ] both: `set_currents`/`set_gains` from any adapter reflected in the run (config unify).
- [ ] both: `get_par`, `get_temp`, `baseline`, `arrun`, `run_mpf` expected output.
- [ ] regression: AMBYTE measurement numbers byte-identical vs pre-merge `fw_new`.
- [ ] glitch: opening the USB port (DTR/RTS) does not emit `Unknown cmd`.

---

## Appendix — file-by-file actions

Legend: `TRUNK`=take `fw_new` · `MERGE`=reconcile · `PORT`=bring `A` feature onto core ·
`DROP`=remove · `KEEP`=unchanged.

| File | Action |
|---|---|
| `wrench.{cpp,h}`, `do_c.cpp` | **DROP** (D1, §8) |
| `do_command.h` | **MERGE** → `frontend_text.cpp` (thin text adapter, §7b) |
| `run_esp.cpp` | **TRUNK + MERGE** → binary adapter, frozen wire, calls core (§7a) |
| `commands.cpp/.h` (fork `A`) | **SKELETON ONLY** → openJII shared module `lib/openjii_proto/` (§7c/§7d); discard hand-printed bodies |
| new `lib/openjii_proto/` | **NEW** — device-agnostic openJII comms module, built fresh to the spec (§7d, D5) |
| `PAM.cpp` / `PAM.h` | **MERGE** — `CONNECTION_TYPE` sinks + `CLOUD`; D2/D3 kept; D4 re-add (§9) |
| `data_utils.cpp` | **MERGE** — FSM + byte-sum checksum (keep) + re-graft `send_json` (§9) |
| `data_utils.h` | TRUNK + re-add `send_json` decl |
| `config.h` | TRUNK + add `CLOUD`; home of `AmbitConfig` (§6) |
| new `core.cpp/.h` | **NEW** — the unified operation set (§5) |
| `serial.cpp/.h` | TRUNK + absorb `serial_read_until`/`flush_serial`, drop echo (§10) |
| `ambit-1.ino` | **MERGE** — `fw_new` power mgmt + `A` text framing (§10) |
| `nvs1.*`, `self_test.cpp`, `devices_init.*`, `pin_config.h`, drivers | KEEP (identical) — nvs checksum `&`→`|` done (#6) |
| `platformio.ini` | **MERGE** — multi-env + flags (§11) |
| `open-jii/`, `*.ipynb` | OUT OF SCOPE (server/tooling, not firmware) |

---

## Appendix B — correctness & cleanup catalog (folded from REFACTOR_PLAN)

The single-fork review's items, folded here so this file is the only plan. Referenced
above as `sweep #N` / `#N` (original numbering kept). **Status:** ✅ done · ▶ owned by a
named step (do it there) · ☐ open sweep item (cheap, isolated — fold into the nearest
step) · ⏸ deferred/optional. Line numbers are current as of this session.

| # | Item | Where (current) | Status |
|---|------|-----------------|--------|
| 1 | Wrench VM (17.4k LOC) for ~8 commands | `wrench.*`, `do_c.cpp` | ✅ deleted |
| 2 | Three parallel config copies | global / `run_esp` `_local` / `do_c` statics | ✅ unified to the PAM global |
| 3 | Massive dead code | `PAM.cpp` `/* */` blocks + `data_utils.cpp` `//` block | ✅ deleted (PAM.cpp 1913→1418, data_utils.cpp 716→561; commented-out, so **zero binary change** — Flash identical) |
| 4 | Measurement loop copy-pasted 5–6× | `PAM.cpp` `run_arr_type1`(153)/`run_trigger_spacer`(402)/`external_trigger_run`(580)/`_Flash`(727)/`MPF`(910)/`fluor_offset`(1319) | ▶ Step 4 — extract `PamBuffers` RAII + `read_one_frame()` + `send_results()` |
| 5 | `arr_reset` buffer overflow | was `do_c.cpp` | ✅ gone with Wrench |
| 6 | FW-info checksum `&`→`\|` | `nvs1.cpp:16` | ✅ fixed this session |
| 7 | `PAM_retrieve_env` decode vs centi-°C encode | `PAM.cpp` env path | ▶ Step 6 — rewrite for centi-°C (D2) |
| 8 | Truncating casts in parsing | `w` length clipped to uint8_t; `a`/`aa` pointless casts | ✅ done — `w` length now uint16_t, casts dropped |
| 9 | Non-void FSM fns miss returns (UB) | uncalled `fsm_send_waitesp` (missing return) + dead `fsm_wake_up_calls(void)` | ✅ both deleted (uncalled). Reset-guards N/A — per-run `dataclass` objects reset on construction |
| 10 | Uninitialized locals | `run_arr_type1`/`run_trigger_spacer`/`fluor_offset` comma-decls | ✅ initialized (behavior-neutral; were assigned before use) |
| 11 | Leaks on early return | `PAM.cpp:180-186` 7× `new dataclass` then `return -1` | ☐ stack-allocate `dataclass`, drop new/delete |
| 12 | Unaligned/strict-alias float reads (trap on C3) | `run_esp.cpp:234,242,294,317` `*((float*)&cmd_arr[..])` | ☐ `memcpy` — **frozen wire, bytes unchanged** |
| 13 | Protocol magic numbers scattered | `data_utils.h` vs literals (`ambit-1.ino:167`, `run_esp.cpp`) | ▶ §2 freezes them; centralize into one `proto` header |
| 14 | Pins half-symbolic | `digitalWrite(10/1,…)`, raw `GPIO_NUM_*` | ☐ name pin 10, use symbols |
| 15 | `serial_read_until` re-declared in 4 TUs | `data_utils.cpp`, `PAM.cpp`, `run_esp.cpp`, `ambit-1.ino:59` | ▶ Step 5 — one prototype in `serial.*` |
| 16 | `serial_read_until`/`flush_serial` echo non-target bytes onto the link | corrupts the binary stream | ▶ Step 5 — drop echo |
| 17 | `load_calibration_info` `isKey()`+`getX()` ×~20 | `nvs1.cpp` | ⏸ deferred — dropping `isKey` makes absent keys use the `getX` default not the struct default (config semantics, needs sign-off) |
| 18 | `dataclass` ring-buffer over-engineered | `data_utils.cpp:142` wrap path likely dead; inverted `length`/`_length` | ⏸ deferred (out of scope) |
| 19 | `hash()` dispatch silent-collision risk | `do_command.h:20` | ⏸ optional — `strcmp`/`string_view` table |
| 20 | `c > 127` mode-switch glitch-sensitive | `ambit-1.ino:164` | ▶ Step 8 — framed sync / first-printable-char (text path) |

[CommunicationProtocolOpenJIISerial.md]: ../CommunicationProtocolOpenJIISerial.md
[uart_sensors.c]: ../ambyte-iot-ludo/components/uart_sensors/uart_sensors.c
[ambit_protocol.h]: ../ambyte-iot-ludo/components/device_commands/include/ambit_protocol.h
[device_commands.c]: ../ambyte-iot-ludo/components/device_commands/device_commands.c
[do_command.h]: src/do_command.h
[run_esp.cpp]: src/run_esp.cpp
[commands.cpp]: ../fw_appCompatible/src/commands.cpp
[PAM.cpp]: src/PAM.cpp
[data_utils.h]: src/data_utils.h
[data_utils.cpp]: src/data_utils.cpp
[config.h]: src/config.h
[ambit-1.ino]: src/ambit-1.ino
[core.cpp]: src/core.cpp
[core.h]: src/core.h
