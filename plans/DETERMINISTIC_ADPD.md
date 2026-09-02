# Branch plan: `deterministic-adpd` — exact-N triggered ADPD acquisition

> Status (2026-09-02): **Phase 0 and Phase 1 code landed on `deterministic-adpd`, builds
> clean (`pio run`). Gate V0 PASSED on the bench (§8). Gate V1 not yet run.** The plan was
> then reviewed against the ESP8685 and ADPD6000 datasheets (§10); the review's code changes
> are in and listed in §9. This document ports
> the ideas from the abandoned `feature/arrun-deterministic-trigger` branch of the
> pre-cleanroom repo (`LEGACY/ambit-IoT`, last commit 2026-07-03) onto today's single-image
> firmware. Nothing from that branch was cherry-picked (no shared git history); the
> diagnostics and `run_arr_trigger` were ported by hand and adapted to the calibration
> boundary and the clean-room driver. See §9 for what landed and where it deviates.

## 1. The problem

`arrun` (binary cmd 21, text `arrun*`, JSON `arrun`) all end in `run_arr_type1`
([src/PAM.cpp](../src/PAM.cpp)). That function puts the ADPD6100 in **free-run** (`adpd.RUN()`,
GO=1, internal `TIMESLOT_PERIOD` = `run_freq(freq)`) and only calls `STOP()` once `num_ptx`
samples have been *read from the FIFO*. So `num_ptx` bounds samples **stored**, not timeslot
sequences **executed**: between the last useful FIFO read and `STOP()` the sequencer keeps
firing LED pulses, and at high `freq` the loop's `esp_light_sleep_start()` cadence means many
sequences run per wake. The leaf under test receives an unknown, rate-dependent number of
extra 630 nm pulses (photodamage, perturbation of the system being measured), and the
emitted-pulse count is not reproducible run to run.

## 2. The idea

The ADPD6100 has an **external sync** mode already used in-tree: `GPIO0_cfg=1`,
`SYNC_GPIO=0`, `EXT_SYNC_EN=1` (`adpd.gpio_config`, see `run_trigger_spacer`
[PAM.cpp:563](../src/PAM.cpp#L563), `external_trigger_run`, `external_trigger_run_Flash`, `MPF`).
In that mode **one rising edge on ESP32 GPIO10 → ADPD GPIO0 launches exactly one full timeslot
sequence**; the internal period timer is gated off. `adpd_trigger()` is a 1 µs pulse on GPIO10
([PAM.cpp:555](../src/PAM.cpp#L555)).

So: keep the per-line protocol (`type / farred / num_ptx / freq / actinic / subsampling`)
byte-identical, but replace free-run with a software-paced loop that fires **exactly `num_ptx`
edges** at period `1/freq`, reads one sequence from the FIFO after each edge, and aborts (never
stores a partial sample) if an edge is lost. Emitted sequences == triggers == stored == N.

Ship it as a **new command** (`arrunt`) beside `arrun`. `arrun` stays byte- and
behaviour-identical until a separate, calibration-gated default swap (see §7).

## 3. Facts re-verified against *this* codebase (2026-09-02)

| Item | Status here | Note |
|---|---|---|
| Free-run acquisition | `run_arr_type1` [PAM.cpp:261](../src/PAM.cpp#L261) | `adpd.RUN()` + `delay(2)`, FIFO drained in a `while (fifo_c >= expected_readout_bytes)` inner loop, light sleep between polls |
| EXT_SYNC template | `run_trigger_spacer` [PAM.cpp:563](../src/PAM.cpp#L563) | Type-1 only, 100 ms `millis()` timeout, `goto del_classes` teardown, restores `GPIO0_cfg=0 / EXT_SYNC_EN=0 / ARRAY_MODE1` |
| Fluor slot integration | `preset_config_2(1, 1)` [PAM.cpp:57](../src/PAM.cpp#L57); `calc_signal(..., num_integration=1)` | Fluorescence is already integ=1. The `4` is on **ambient (ts0)** and **730 (ts2)** only: `preset_config_1(0,4)`, `preset_config_3(2,4)` |
| Far-red | `farred == 1` → `num_ts(9)`, six illumination-only slots ts3..8 with `_repeats` pulses each, **0 FIFO bytes** | Opt-in per line via `line[1]`; no spare line bytes |
| 8-byte line layout | `[0]=type [1]=farred [2..3]=num_ptx(BE) [4..5]=freq [6]=actinic [7]=subsampling` | **Frozen** (Ambyte builds these bytes) |
| Calibration at result boundary | `apply_adpd_calibration(S630/R630/SUN/LEAF/S730/R730, …)` in `run_arr_type1` | **New since LEGACY** (LEGACY had a `ret[0]-65000` hack). The triggered path must apply the identical calibration or the wire values diverge |
| Output sinks | `pam_send_results()` (AMBYTE FSM / COMPUTER ASCII), PLOTTING inline, `json_output` flag, `retain` → `g_async` (cmd 22/23/24) | All four sinks must be reachable from the triggered path |
| ADPD driver | JII clean-room `jii::adpd6000::Driver` via the `ADPD6` wrapper | `read_fifo_samples` still does **one SPI transaction per 3-byte value**; `Driver::read_fifo(buf, len)` already drains `len` bytes in one CS-low transaction (≤ 640) |
| `fifo_count()` | returns **0** on I/O error *or* when count > 640 (`kOutOfRange`) | Still a silent-zero trap: an overflowed FIFO looks empty |
| `watch_dog_timer` | declared + reset, never checked ([PAM.cpp:313](../src/PAM.cpp#L313)) | No timeout exit in the free-run loop |
| N cap | `MAX_DATACLASS_SIZE 2000` ([data_utils.h:8](../src/data_utils.h#L8)); `run_preprocess_type1` always returns 0 | `dataclass::init` fails → `return -1` **without deleting the 8 `new` buffers** (heap leak on every failed run) |
| Logging | `-DCORE_DEBUG_LEVEL=0`: every `ESP_LOGx` compiles away | Error paths are silent on all hosts. New console verbs need an explicit reply on failure |
| Build | single `ambit` env, version injected by `tools/version.py` | The LEGACY branch's hand-edited `nvs1.h` version bump is obsolete: **never** hand-edit versions here |
| Binary cmd ids in use | 1 2 4 5 6 10 17 18 20-29 31-35 37 | A new binary id must be agreed with the Ambyte repo (`ambit_protocol.h`) before use |
| SPI | `max_transfer_sz = 4092`, `SPI_DMA_CH_AUTO` ([devices_init.cpp](../src/src/devices_init.cpp)) | Largest readout 8 × 3 = 24 B, far below the DMA cap |

## 4. What the LEGACY bench work already learned (carry forward)

These came out of running `tseq` / `tidle` / `arrunt` on hardware in June–July 2026. They are
recorded only in code comments on that branch; **no V0 timing numbers were ever written down**.

1. **EXT_SYNC truly gates the sequencer.** `tidle` (arm EXT_SYNC, send zero edges, watch the
   FIFO for 2 s at a 100 ms parked period) showed **no GO auto-run sequence and no
   self-triggering**. Consequence: any "extra bytes in FIFO" seen after a triggered read is a
   `fifo_count()` byte-count read artifact, not an extra LED pulse. LEGACY recovered with
   `clear_fifo()` in GO mode, as `run_trigger_spacer` still does. **Datasheet review (§10):
   CLEAR_FIFO is specified "while not operating"**, so that recovery is off-spec. The
   triggered path now treats any byte count other than exactly one sequence as a desync and
   aborts (`ARR_TRIG_FIFO_DESYNC`, −5); the diagnostics drain by reads and count residuals.
   V0 saw zero residuals in 1500 edges.
2. **Far-red swallows re-triggers.** With `num_ts(9)` the FIFO is full after ts0..ts2, but the
   six illumination-only slots are **still firing**. An edge sent into that tail is lost:
   observed as exactly 50 % timeouts. The pacing loop must not re-trigger before the *whole*
   sequence ends; LEGACY used a provisional `1000 + 2200·_repeats` µs floor on the period and
   flagged it for scope verification. At high requested rates far-red therefore runs at
   `min(freq, ceiling)`, it does not abort.
3. **Park the internal period high** (`run_freq(10)`, 100 ms) while in EXT_SYNC so that even
   if gating failed the self-trigger rate would be visible and slow.
4. **GPIO10 must be held LOW through light sleep** (`gpio_sleep_set_direction` OUTPUT +
   `PULLDOWN_ONLY`), otherwise a sleep/wake transition can produce a spurious edge.
5. **Lost trigger → abort the run**, never store a partial or desynced sample. LEGACY used a
   20 ms hard bound per sample on `esp_timer_get_time()`.
6. **A first-sample spike was observed** on the raw counts of the first triggered sequence.
   The open question (LEGACY's uncommitted `tratio` diagnostic) was whether it is
   common-mode (LED intensity → signal/reference ratio preserved → harmless for relative
   yield) or channel-specific (AFE settling → warm-up or drop-first needed). **Unresolved.**
7. **Sequence time and ceiling — measured at V0 (§8), superseding the earlier estimate.**
   `t_seq` = 410 µs (amb/730 integ 1) / 423 µs (integ 4) at `num_ts=3`; the 26 µs spread is
   one `fifo_count()` poll (20 µs), so the chip's own jitter is below the resolution of
   `tseq`. The time is fixed by the slot registers, not by integration: per data slot
   8 µs precondition + DARK1 48 µs (the ambient-loop minimum the datasheet requires) +
   LED 60 + LIT 64/72 + DARK2 90 + `NUM_INT` samples at 1 µs in two regions, ≈105 µs, plus
   the HFO wake-up; integ 1→4 adds 3 samples × 2 regions × 2 slots = 12 µs (measured 13).
   **Integration is therefore not a speed knob.** Chip ceiling ≈ 2.3 kHz. The practical
   ceiling is the SPI: at the ADPD's 10 MHz maximum each transaction still costs ≈20 µs of
   driver overhead, so the per-value FIFO read is 8 × 20 ≈ 160 µs → ≈1.6 kHz absolute,
   ≈1.2 kHz with margin before Phase 2, ≈2 kHz after the single-transaction read.
   **Far-red:** head 425 µs (FIFO complete after ts0..ts2); tail from the far-red slot
   registers = 6 × (MIN_PERIOD 320 µs × `_repeats` + ≈60 µs) ≈ 1.92 ms·`_repeats` + 0.4 ms,
   which gives the provisional floor `1000 + 2200·_repeats` 15–40 % margin. It stays until
   V1 scopes the LED line; far-red is capped near 300 Hz either way.

## 5. Invariants every phase re-asserts

1. **Ambyte wire format byte-frozen** — layout *and* channel presence (`sun/leaf` only when
   `subsampling>0`, `730` only when `has_730`), all three stream shapes (type-1, type-1+farred,
   type-2). Gate: `plans/HW_CONFORMANCE.md`.
2. **Deterministic count** — sequences == edges == stored == N, zero tolerance, every rate.
3. **Guaranteed teardown on every exit** — single-exit cleanup restoring `GPIO0_cfg=0`,
   `EXT_SYNC_EN=0`, `SYNC_GPIO`, `adpd_mode=ARRAY_MODE1`, ADPD `STOP()`, LED off unless persist,
   and all 8 dataclass buffers freed. A leftover `EXT_SYNC_EN=1` silently breaks the next
   free-run `arrun`.
4. **µs-bounded per-sample timeout** and a real abort path (not a never-checked counter).
5. **`num_integration` single source of truth** — the fluor slot's on-chip integration and the
   `calc_signal` divisor must stay equal.
6. **Frozen-binary semantics** — `arrun`/cmd 21 untouched; the triggered path is additive.
7. **Nothing prints on UART0 except explicit console replies** (AGENTS.md). Diagnostics are
   console verbs and are build-flag gated so they cannot ship.
8. **Trigger pulse ≥ 5 µs.** The ADPD's LF state machine ticks at 960 kHz (1.04 µs); the
   legacy 1 µs pulse is one tick wide. `kTrigPulseUs` = 5. V1a confirms on the scope.
9. **No CLEAR_FIFO while OP_MODE = 1.** Clear only after STOP; in GO, drain by reads or abort.
10. **Light-sleep wake margin scales with the gap.** The ESP RTC slow clock is a 136 kHz RC
    oscillator (percent-level after calibration); margin = max(1.5 ms, 2 % of the gap), and
    `tstat` reports late edges so V1j can size it.

## 6. Phases for this codebase

Each phase compiles (`pio run`), is flashed, and passes its gate before the next starts.

### Phase 0 — Scaffolding + bench characterisation
- Add `plans/DETERMINISTIC_ADPD.md` (this file).
- Port the diagnostics as branch-only console verbs, gated by `-DAMBIT_DIAG_TRIGGER` in
  `platformio.ini` (on for this branch, off for release):
  - `tseq,<reps>,<farred>,<integ>[,<frrep>]` — `measure_tseq()`: EXT_SYNC, one edge → time to
    full FIFO, min/mean/max over `reps`, implied max rate. Includes the far-red settle fix (§4.2).
  - `tidle,<farred>,<integ>,<gate>` — `measure_idle()`: zero edges for 2 s, report GO-autorun
    bytes and self-trigger bytes.
  - `tratio,<reps>,<N>,<freq>,<integ>` — `measure_first_ratio()`: first-sample s/r vs steady
    state with a Welford SEM noise floor (answers §4.6). Depends on Phase 1's `run_arr_trigger`,
    so it lands at the end of Phase 1.
- All three touch only PAM.cpp/PAM.h/do_command.h. No change to `run_arr_type1`, the router,
  `run_esp.cpp` or `data_utils.cpp`.

**Gate V0** — run `tseq,500,0,1`, `tseq,500,0,4`, `tseq,500,1,4`, and `tidle,0,1,0` on the bench.
Pass: `t_seq` spread < ±10 %, `tidle` reports zero autorun and zero self-trigger. **Record the
numbers in this file** (LEGACY never did). They set the rate ceiling and the far-red period floor.

### Phase 1 — `run_arr_trigger()` + `arrunt` (additive, non-breaking)
- `run_arr_trigger(length, arr, led_persist, allow_interrupt, json_output, retain)` in PAM.cpp,
  modelled on `run_trigger_spacer` for the EXT_SYNC bring-up/teardown and on `run_arr_type1` for
  the per-line branching, calibration, subsampling and sinks.
- **Factor, don't duplicate, the per-sample store.** Pull the block in `run_arr_type1` from
  `apply_adpd_calibration(...)` through the subsampling `put()`s and the PLOTTING `printf` into a
  `static` helper used by both paths. This is the only way invariant 1 holds without a second
  copy drifting; it is a pure refactor of `run_arr_type1` and is covered by HW_CONFORMANCE.
- Per-sample loop: light-sleep for gaps > 3 ms, busy-wait to the edge, `adpd_trigger()`, pace the
  next edge from *this* edge (`esp_timer_get_time()`), apply the far-red period floor, µs-bounded
  FIFO poll (20 ms), `readfifo`, `clear_fifo()` on residual, store, env resample on the same
  cadence/guards as `run_arr_type1`, `PAM_interrupt` check.
- Single-exit `goto cleanup` covering: init failure (fixes the 8-buffer leak for this path),
  lost-trigger abort, interrupt, normal completion. Return −1 on abort; the text adapter replies
  `ERROR` explicitly since logging is compiled out.
- Reject `num_ptx >= MAX_DATACLASS_SIZE` at the command boundary with an explicit error.
- Core op `core_run_array_triggered(...)` in core.cpp so every adapter shares one entry.
- Text verbs `arrunt` / `arrunt1` (PLOTTING) / `arrunt2` (COMPUTER) in do_command.h mirroring
  `arrun*`, plus an `arrunt` JSON handler in frontend_json.cpp (`json_output=true`). Binary and
  async adapters wait for Phase 4.
- From the datasheet review (§10): 5 µs trigger pulse (inv. 8); byte count ≠ one sequence →
  abort −5, no `clear_fifo()` in GO (inv. 9); gap-scaled sleep margin (inv. 10); `tstat`
  console verb (diag-gated) reporting samples / late edges / max overshoot / residuals for V1j.

**Gate V1** — on a scope / logic analyser with the LED-driver line and GPIO10: V1a trigger width
and no runts; V1b **zero extra pulses**; V1c no self-trigger; V1e three-count correlation
(GPIO10 == LED sequences == stored == N) at 10/100/200/500/1k Hz, type-1, type-1+farred, type-2;
V1f first-sample integrity (`tratio` decides warm-up/drop-first); V1g timeout aborts cleanly with
a console error; V1h teardown read-back (`arrun` free-run works correctly right after an aborted
`arrunt`); V1i N-cap boundary 1999 ok / 2000 rejected; V1j mean interval and drift (`tstat`:
late edges and max overshoot at 1, 10, 100, 1000 Hz — sizes the sleep margin).

### Phase 2 — Batched FIFO read (throughput; V0 showed the SPI, not the chip, sets the ceiling)
- Add an `ADPD6::readfifo_block()` that calls `Driver::read_fifo(buf, expected_readout_bytes)`
  once and unpacks MSB-first; keep the per-value path for the other callers. Check the return
  code (the current callers ignore it).
- Make `fifo_count()` overflow observable to the triggered path (a sentinel, or use the
  two-arg overload and treat `kOutOfRange` as an abort), instead of the silent 0.

**Gate V2** — bit-exact vs per-value over ≥100 k samples including values > 65 000; overflow
fault-injection aborts instead of hanging; `fifo_count()==0` after every read.

### Phase 3 — Speed knobs
- `num_integration` on the ambient/730 slots is **not** a speed knob (V0: 13 µs for 1→4, §4.7).
  Expose it only if the SNR study below wants it; default stays 4, fluorescence stays integ=1.
- Remove the deliberate `delay(2)` after `RUN()` in the triggered path only; keep `delay(5)`
  after arming EXT_SYNC (settle) unless V3 shows it is unnecessary.
- Replace the provisional far-red period floor with the V0-measured full-sequence time.
- Keep `Serial.printf` (PLOTTING) out of the hot path at rates where it would stall the pace.

**Gate V3** — max-rate exact count + framing (≥100 k samples, zero timeouts, zero residuals);
back-pressure guard band ≥1.5× `t_seq`; SNR integ=1 vs 4 with a pre-stated decision rule and
`F(integ1) == F(integ4)/4` within noise; ≥10 min thermal soak at max rate; zero SPI errors.

### Phase 4 — Productionise the additive command
- Binary adapter: new cmd id agreed with the Ambyte repo (`ambit_protocol.h`,
  `uart_sensors.c`); same payload as cmd 21. Async variant analogous to cmd 22 only if the
  Ambyte needs it. Both decode wire conventions in `run_esp.cpp`, never in the core.
- `retain=true` honours the `g_async` ownership contract identically to `run_arr_type1`.
- Build flag `AMBIT_DIAG_TRIGGER` off; `tseq/tidle/tratio` do not ship.

**Gate V4** — full `plans/HW_CONFORMANCE.md` re-run (router + `run_esp.cpp` touched); golden
byte-diff of `arrun` before vs after the branch (must be zero); `arrunt` vs `arrun` diff limited
to payload values; async over-cap returns ERROR with flat heap.

### Phase 5 — Merge; default swap is a separate decision
- Merge with `arrun` unchanged (PR title `feat(adpd): exact-N triggered acquisition (arrunt)`).
- Update `HW_CONFORMANCE.md` with the V0–V4 captures.
- **Do not repoint `arrun`.** Fixing over-pulsing changes the numeric F values the fleet
  receives for the same line (fewer prior pulses → different steady state). The swap needs
  side-by-side F comparison and explicit sign-off from calibration, the Ambyte team and the
  app, then ships as its own PR.

## 7. Risks

| Risk | Mitigation |
|---|---|
| Duplicated store logic drifts from `run_arr_type1` | Shared per-sample helper (Phase 1); V4 golden diff |
| Default-swap changes fleet F values | Additive `arrunt` only; swap deferred (§6 Phase 5) |
| EXT_SYNC left on after an early return | Single-exit cleanup; V1h read-back |
| Far-red edge swallowed in the illumination tail | Period floor from V0 measurement; abort-on-lost-trigger backstop |
| Silent errors (logging compiled out) | Explicit console/JSON error replies; return codes checked |
| FIFO overflow reads as empty | Phase 2 sentinel; V2 fault-injection |
| Spurious edge across light sleep | GPIO10 sleep hold LOW; `tidle` |
| Text host latch + long busy-wait at kHz | TEXT host never sleeps anyway; JSON envelope owns the stream while `busy()` — no router change needed |
| Binary id collision with Ambyte | Agree the id in `ambit_protocol.h` first; Phase 4 only |

## 8. Bench results

_(fill in at each gate; date, firmware `git describe`, DUT serial)_

**V0 — 2026-09-02, FW `1.2.0-1-gb8dc1ef-dirty` (branch tree, uncommitted), DUT NVS name
`AmbitV003` (no physical serial recorded). PASS: spread ±3 % (criterion ±10 %), 1500/1500
edges landed, zero GO-autorun, zero self-trigger.**

| Gate | Config | Result |
|---|---|---|
| V0 `tseq,500,0,1` | num_ts=3, amb/730 integ=1 | ok=500 timeouts=0; min/mean/max 403/410/429 µs; ~2439 Hz |
| V0 `tseq,500,0,4` | num_ts=3, amb/730 integ=4 | ok=500 timeouts=0; min/mean/max 409/423/430 µs; ~2364 Hz |
| V0 `tseq,500,1,4` | num_ts=9 far-red, frrep=1 | ok=500 timeouts=0; head min/mean/max 423/425/430 µs; tail not visible in FIFO (see §4.7) |
| V0 `tidle,0,1,0` | num_ts=3, integ=1, 2 s, no edges | GO_autorun=0 bytes, idle_max=0 bytes, 99 305 polls (≈20 µs per `fifo_count()`) |

## 9. Implementation notes (what landed 2026-09-02)

Files: `platformio.ini`, `src/PAM.{h,cpp}`, `src/core.{h,cpp}`, `src/do_command.h`,
`src/frontend_json.cpp`. `run_esp.cpp`, `data_utils.cpp` and the router are untouched.

- **Shared helpers (pure refactor of `run_arr_type1`)** — `pam_store_type1_sample()` is the
  per-sample block (calibration → buffers → subsampling → PLOTTING line) lifted verbatim;
  `pam_finish_results()` is the end-of-run sink (JSON / retain→`g_async` / `pam_send_results`).
  `run_arr_type1` now calls both; its behaviour is unchanged but it is covered by
  `HW_CONFORMANCE.md` and must be byte-diffed at V4.
- **`run_arr_trigger()`** in PAM.cpp: EXT_SYNC arm/disarm helpers (`trig_arm_ext_sync`,
  `trig_disarm_ext_sync`) shared with the diagnostics, `trig_fire_and_wait()` (20 ms
  µs-bounded), pacing from the edge, provisional far-red floor `1000 + 2200·_repeats` µs,
  single `cleanup:` exit. Return codes are `ArrTriggerResult` (PAM.h): `0` ok, `-2` bad line,
  `-3` alloc, `-4` lost trigger.
- **Validation at the boundary** — `run_arr_trigger_validate()` rejects total N of 0 or
  ≥ `MAX_DATACLASS_SIZE` and any line with `num_ptx > 0 && freq == 0`, before anything is
  allocated or written to the chip.
- **Interrupt polling deviates from `run_arr_type1`** — `PAM_interrupt()` spins up to 15 ms
  when the input is idle, which would wreck pacing above ~60 Hz. The triggered path only
  consults it when `Serial.available() > 0` (and after a light-sleep wake, which is cheap).
- **Adapters** — core op `core_run_array_triggered()`; text `arrunt` / `arrunt1` / `arrunt2`
  (reply `ERROR arrunt <code>` on failure; `len` outside 1..16 rejected before reading the
  array); JSON `arrunt` (`{"error":"arrunt_failed","code":<n>}` on failure, the data object
  only on success). Binary and async adapters wait for Phase 4.
- **Diagnostics** (`-DAMBIT_DIAG_TRIGGER`, on in `platformio.ini` for this branch):
  `tseq`, `tidle`, `tratio` as specified in §6. Two changes vs LEGACY: (a) each diagnostic
  leaves `adpd_mode = MPF_MODE` so the next array run re-applies `conf_slow_FR_1` (LEGACY left
  `ARRAY_MODE1` and the next `arrun` inherited the overridden ts0/ts2 integration);
  (b) `tidle` gate 1 (GO_SLEEP) is refused — it relied on the vendored ADI driver's sleep-mode
  call, which the clean-room driver does not expose. Gate 0 is what V0 requires.
- **Datasheet review changes (2026-09-02, after V0):** `kTrigPulseUs = 5` in
  `trig_fire_and_wait()` (the legacy `adpd_trigger()` keeps its 1 µs pulse for the frozen MPF
  and trigger-spacer paths); byte count ≠ one sequence after an edge → `ARR_TRIG_FIFO_DESYNC`
  (−5) and cleanup, no `clear_fifo()` in GO; sleep margin = max(1.5 ms, 2 % of gap);
  `g_trig_stats` + `tstat` verb (samples, late edges, max overshoot µs, residuals, result).
  `tseq` no longer clears the FIFO between edges (drains by reads, reports `residuals=`);
  `tratio` discards a run on a residual.
- **Open before Phase 2**: run V1 on the scope (V1a pulse width, V1e counts, V1j `tstat`);
  measure the far-red tail and replace the provisional floor.

## 10. Datasheet review (2026-09-02)

Sources: *ESP8685 Series Datasheet* v1.6 (Espressif; the ESP32-C3 die) and *ADPD6000 Data
Sheet* Rev. 0 (Analog Devices; local copy `Projects/1.1 Ambit/0.8 Savedir/datasheet_adpd6000.pdf`).

| Plan element | Datasheet says | Consequence |
|---|---|---|
| EXT_SYNC gates the sequencer (§2, §4.1) | GPIO_EXT[2] EXT_SYNC_EN: "use the GPIO selected by EXT_SYNC_GPIO to trigger samples rather than the period counter"; in GO with external sync "the device enters the sleep state before the first wake-up and time slot regions begin" | Confirms `tidle`. Parking the period at 100 ms is redundant but harmless |
| Trigger pulse 1 µs | LF state machine clock = 960 kHz internal RC (1.04 µs tick); no minimum sync pulse width is specified | One-tick pulse is marginal → 5 µs (inv. 8) |
| `clear_fifo()` on residual (§4.1) | FIFO_STATUS[15] CLEAR_FIFO: "empty the FIFO while not operating" | Off-spec in GO → abort on residual, drain by reads in diagnostics (inv. 9) |
| Sequence time dominated by integration (§4.7 old) | Two-region digital integration: DARK1 ≥ 48 µs ambient loop, ADC samples at 1 µs; firmware slots use offsets 48/60/64–72/90 µs | Slot ≈ 105 µs regardless of integ; measured 13 µs for 1→4 |
| Far-red tail | Far-red slots: MIN_PERIOD 320 µs (per repeat), LED width 200 µs, 6 slots | Tail ≈ 1.92 ms·`_repeats`; provisional floor has 15–40 % margin |
| SPI | ADPD f_SCLK max 10 MHz (firmware already at 10 MHz); ESP GP-SPI2 up to 60 MHz | Bound is ESP-IDF per-transaction overhead (≈20 µs), not the bus → Phase 2 is throughput |
| GPIO10 through light sleep (§4.4) | ESP8685 Table 5-8: in Light-sleep "all GPIOs are high-impedance" | The pull-down sleep hold is required, keep it |
| Light-sleep wake timing | RTC slow clock = internal 136 kHz RC (or 32 kHz XTAL, not fitted) | Percent-level error → gap-scaled margin (inv. 10), `tstat` late-edge count |
| Register writes in GO | "Register writes that affect operating modes cannot occur during go mode" | Code already brackets every `run_freq`/`num_ts`/`repeats_only` between STOP and RUN |
| ESP logic levels vs ADPD input | ESP VOH ≥ 0.8·VDD; ADPD GPIO VIH ≥ 0.7·IOVDD | Direct drive is fine |
