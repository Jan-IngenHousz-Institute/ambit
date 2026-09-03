# Branch plan: `deterministic-adpd` — exact-N triggered ADPD acquisition

> Status (2026-09-03): **Phases 0, 1 and 2 landed on `deterministic-adpd`; V0, V1 (except V1a,
> scope on GPIO10, TODO) and V2 passed on the bench.** Ten bench sessions (§8) found and closed three
> things the plan did not anticipate: the BOOT-pin reset gesture fired by LED coupling, the
> running ESP core degrading the 730 channel (fixed by sleeping the core through each
> sequence), and a ≈21 % free-run time-base error (handed off, `plans/ADPD_OSC_TRIM.md`).**
> Datasheet review in §10. What landed and where it deviates: §9. **Phase 3 is handed off:
> `plans/PHASE3_HANDOFF.md`.** This document ports
> the ideas from the abandoned `feature/arrun-deterministic-trigger` branch of the
> pre-cleanroom repo (`LEGACY/ambit-IoT`, last commit 2026-07-03) onto today's single-image
> firmware. Nothing from that branch was cherry-picked (no shared git history); the
> diagnostics and `run_arr_trigger` were ported by hand and adapted to the calibration
> boundary and the clean-room driver. See §9 for what landed and where it deviates.

## 1. The problem

> Bench addendum (§8, 2026-09-02): the free-run time base is also wrong. It runs on the ADPD's
> uncalibrated internal 960 kHz RC oscillator and on the test unit is ≈21 % fast at every rate,
> so `arrun` delivers N samples in 0.79·N/freq seconds while the host assumes N/freq. The
> triggered engine paces from the ESP's crystal-derived timer and is exact.

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
8. **The ESP core must be quiet while the ADPD converts (2026-09-03, §8 sessions 5–10).** A
   busy-waiting core doubles the spread of the 730 channels and shifts their means; SPI
   silence, slot timing and integration do not matter, only whether the core is running.
   Free-run looked clean solely because `run_arr_type1` light-sleeps between FIFO drains. The
   triggered engine light-sleeps through the sequence (≥ ≈820 µs actual, fits ≤ ≈750 Hz), falls
   back to a WFI-blocked wait above that (half the benefit), never busy-waits by default.
9. **The BOOT-pin reset gesture is fooled by the LED pulses.** GPIO9 runs only to the FFC (the
   Ambyte's remote reset), and LED edges at a 10 ms period match the ISR's 8–12 ms toggle
   window. Free-run sleeps through most pulses; the triggered engine caught every one. The
   gesture is paused for the duration of a triggered run.
10. **First samples after idle are a three-sample transient** (r_630 −16 %, +7 %, +0.6 %), in
    both engines. Phase 3 owns a warm-up sequence or a documented discard count.
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
   ≈1.2 kHz with margin before Phase 2, ≈2 kHz after the single-transaction read
   (**measured after Phase 2: 2 kHz with exact counts and zero lost edges**, block read 46 µs).
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
11. **The core is quiet while the ADPD converts.** Per line: light sleep through the sequence
    when the period allows, WFI otherwise (`trig_pick_quiet_mode`); `tstat` reports
    `quiet_mode`. A busy-waiting core is a regression on the 730 channel (§4.8).
12. **The BOOT-pin gesture is paused during a triggered run** (§4.9).

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

### Phase 2 — Batched FIFO read (throughput; V0 showed the SPI, not the chip, sets the ceiling) — DONE 2026-09-03
- `ADPD6::readfifo_block()` calls `Driver::read_fifo(buf, n·width)` once and unpacks MSB-first;
  the per-value `readfifo()` stays for the free-run path. The triggered engine checks the return
  code and aborts with `ARR_TRIG_IO_ERROR` (−6) instead of storing zeros.
- `trig_fire_and_wait()` polls with the rc-returning `fifo_count(&c)`: an I/O error aborts −6,
  `kOutOfRange` (count > 640) aborts −5; FIFO_STATUS (0x0000) is captured at every abort so
  `tstat` shows OFLOW/UFLOW and the count. The silent-zero trap is gone.
- `kTrigReadBudgetUs` 350 → 150 (block read measured 46–55 µs), which moves the light-sleep
  band to ≈900 Hz and the WFI band to the chip ceiling.

**Gate V2** — bit-exact vs per-value over ≥100 k samples including values > 65 000; overflow
fault-injection aborts instead of hanging; `fifo_count()==0` after every read. **PASSED**, §8.

### Phase 3 — Settle, characterise, harden (handoff: `plans/PHASE3_HANDOFF.md`, written 2026-09-03; the list below is the original and is superseded by that file)
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
| Core activity degrades the 730 channel | Inv. 11: sleep/WFI through the sequence; verified across 10 Hz–1 kHz (§8 session 10). Above ≈750 Hz only WFI fits, 730 noise ≈1.5× free-run there. **Accepted (2026-09-03): the 730 channel is not used in normal measurements**, so the residual high-rate penalty needs no hardware action; documented here only |
| LED coupling into GPIO9 resets the ESP | Inv. 12: gesture paused during runs. `arrun` at 90–125 Hz is still exposed in the field: separate fix on `main` |
| Baselines taken with the free-run engine | Phase 5: re-take `fluor_offset` baselines with the shipping engine (≈12-count dark offset measured before the quiet fix; re-measure after) |

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

**V1 (partial) — 2026-09-02, same FW and DUT, scope on GPIO10 + LED driver line, `tseq`
back-to-back (no pacing, so the scope rate is the loop period, not the chip).** 500 edges →
500 LED sequences in all three configs (V1b zero extra pulses, V1e count correlation at the
zero-gap limit). Loop period − FIFO-ready ≈ 190–200 µs = the eight per-value SPI reads +
one poll: the current ceiling is ≈1.6 kHz and Phase 2 is what raises it.

| Gate | Config | Result |
|---|---|---|
| V1 `tseq,500,0,1` | num_ts=3, integ=1 | scope 1.666 kHz (600 µs loop), 500 hits |
| V1 `tseq,500,0,4` | num_ts=3, integ=4 | scope 1.608 kHz (622 µs loop), 500 hits |
| V1 `tseq,500,1,4` | far-red, frrep=1 | scope 308.4 Hz (3243 µs loop = the 3200 µs software floor), 500 hits; **tail still unmeasured** — read the burst length first-pulse→last-far-red-pulse on the LED trace |
| V1a pulse width | | **TODO later**: GPIO10 has no probe point, a PCB trace must be scraped open first. Expect 5 µs (`kTrigPulseUs`) |

**V1e / V1j — 2026-09-02, FW `1.2.0-2-g55d8925-dirty`, DUT `AmbitV003`, driven over COM61 by
`scratchpad/v1_bench.py` (`arrunt2` + `tstat` after each run; scope counts from the earlier
manual session). Stored count == N in every completed run; TIMING block confirms the rate.**

| Line | N | Stored | TIMING span | `tstat` | Note |
|---|---|---|---|---|---|
| type-1 1 Hz | 20 | 20 | 19.05 s | late=13 max 8 µs | 1 s light sleeps: no overshoot (inv. 10 holds) |
| type-1 10 Hz | 100 | 100 | 9.95 s | late=74 max 8 µs | env 5 points (2 s cadence) |
| type-1 200 Hz | 1000 | 1000 | 5.07 s | late=708 max 9 µs | |
| type-1 500 Hz | 1000 | 1000 | 2.05 s | late=133 max 8 µs | |
| type-1 1000 Hz | 1000 | 1000 | 1.07 s | late=634 max 8 µs | |
| far-red 10 Hz | 100 | 100 | 9.93 s | max 8 µs | |
| far-red 200 Hz | 1000 | 1000 | **5.47 s** | max 9 µs | floor-limited to 185 Hz (`_repeats`=2 → 5.4 ms), as computed in §4.7 |
| far-red 500 Hz | 1000 | 1000 | **3.23 s** | max 8 µs | floor-limited to 312 Hz (`_repeats`=1 → 3.2 ms) |
| type-2 10 Hz | 100 | 100 | 9.93 s | max 8 µs | |
| type-2 200 Hz | 1000 | — | — | `result=-5 samples=1 residual=1` | **one FIFO_BYTE_COUNT ≠ 18 at the first edge**, 1 event in ≈11 000 edges; now re-read once before aborting, byte count recorded |
| type-2 1000 Hz | 1000 | 1000 | 1.03 s | late=91 max 9 µs | |
| type-1 5000 Hz (over ceiling) | 1000 | 1000 | 0.65 s | late=1000 **max 421 µs** | runs at the ceiling (1.54 kHz), does not abort — protocol decision still open |
| type-1 **100 Hz** | 100 / 500 | — | — | — | **software reset ≈250 ms after the command, 3/3, nothing printed before the ROM banner** (see below) |
| V1i N=1999 @1 kHz | 1999 | 1999 | 2.05 s | | accepted |
| V1i N=2000 | | | | `ERROR arrunt -2` | rejected before anything fires |
| V1i freq=0 | | | | `ERROR arrunt -2` | rejected |
| V1g/V1h `tdrop,50` | | | | — | not completed: the drop test was run at 100 Hz and hit the reset above; redo at 200 Hz |

The `late` counts are an artefact: the busy-wait exits at ≥ `next_trigger` and the edge is
timestamped one call later, so every edge reads 8–9 µs late. `kTrigLateThresholdUs` = 50 now
gates the counter; `max_late_us` stays raw. Real overshoot (the 5 kHz run) reads hundreds of µs.

**Reset at 100 Hz.** `rst:0x3 (RTC_SW_SYS_RST)`, saved PC in `esp_restart_noos_dig`, no panic
text: a clean `esp_restart()`. The only silent caller is the boot-pin gesture ISR
(`RB_toggle`, GPIO9, restart after 4 toggles spaced 8–12 ms). 100 Hz is the one tested rate
inside that window; 10/200/500/1000 Hz never reset. Free-run `arrun2` at 100 Hz survived once
(500 samples). Working hypothesis: the LED sequence couples an edge onto GPIO9 and the awake
ESP catches every one. Open: what drives GPIO9 on this board, and whether the scope probe is the
coupling path. Also noted: opening COM61 power-cycles the DUT (`rst:0x1 POWERON`), so scripted
sessions must wait for the boot banner before the first command.

**Slot-C noise is 2–3× higher in EXT_SYNC than in free-run at the same rate** (same target,
same session, `cmp_runs.py`):

| Rate | Mode | r_730 sd | s_730 sd | r_630 sd | sun sd |
|---|---|---|---|---|---|
| 10 Hz | free-run | 44 | 5.8 | 2.0 | 2.2 |
| 10 Hz | triggered | 82 | 8.8 | 1.5 | 2.5 |
| 500 Hz | free-run | 42 | 5.8 | 1.7 | 2.3 |
| 500 Hz | triggered | 88 | 10.2 | 1.6 | 2.4 |

Slots A (ambient) and B (fluor) are unaffected; slot C (730 LED, driver 2) is. Rate-independent,
so it is a property of EXT_SYNC operation, not of the pacing. Candidates: the parked 100 ms
TIMESLOT_PERIOD (the datasheet says the period counter is bypassed; `tpark,<hz>` tests it) or
per-edge sleep/wake settling of the 730 LED driver. **Must be resolved before the default swap
(Phase 5): it changes the 730 reflectance SNR the fleet receives.**

Other observations: no first-sample spike in back-to-back runs (earlier manual runs, minutes
apart, showed +30…+142 on r_630) → the spike is idle-time dependent, `tratio` to confirm; the
earlier rate-dependent s_730 was the target (stable at 980 across rates this session).

**V1 second session — 2026-09-02, new build flashed over COM61 (late threshold, FIFO re-read,
`tpark`), `scratchpad/v1b_bench.py`, bright target (s_730 ≈ 9450).**

| Item | Result |
|---|---|
| V1g `tdrop,50` @200 Hz N=500 | `ERROR arrunt -4`; `tstat result=-4 samples=49` — 49 edges fired, nothing stored. **PASS** |
| V1h after the abort | free-run `arrun2` 200 Hz N=500 clean (first sample normal), then `arrunt2` 200 Hz N=500 clean. EXT_SYNC teardown verified. **PASS** |
| V1j with threshold | `late=0 max_late_us=7` on every 200 Hz run. **PASS** |
| Parked period vs slot-C noise | r_730 sd 86.9 / 87.1 / 89.2 / 89.7 at park 10 / 200 / 1000 / 1 Hz; free-run 37.8 / 39.7. **No effect** — the period counter is bypassed exactly as the datasheet says; the noise is intrinsic to EXT_SYNC operation |
| Noise pattern | channel 2 of the integ-4 slots: leaf (slot A ch2) sd 5.7–6.0 vs 3.9–4.2 free-run, r_730 (slot C ch2) 87 vs 38; channel 1 of the same slots barely (sun 2.5 vs 2.4, s_730 11.5 vs 10.4); slot B (integ 1) identical (r_630 1.6 both). **Next experiment: amb/730 integ 1 vs 4 in EXT_SYNC** (Phase 3 SNR study, now with a reason) |
| Type-2 200 Hz ×3 | all three complete, `count_glitches=1` in each run — the FIFO_BYTE_COUNT over-read at the first edge of a type-2 line is deterministic, one per run; the re-read returns 18 and the stored data are normal. Record the value and sample index next |
| `tratio,20,20,500,1` / `…,4` | first s/r deviates +7.0 % / +1.3 % from steady, noise floor ±18 % / ±15 % (s_630 is tiny on this target) → verdict COMMON-MODE but statistically weak; no first-sample outlier on r_630 in any back-to-back triggered run (free-run produced one garbage first sample: r_630 4870, sun 1599) |
| **Free-run time base is ≈21 % fast on this unit** | TIMING spans: free-run 200 Hz N=1000 → 4.14 / 4.20 s (5.0 s expected), N=500 → 2.13 s; 100 Hz N=500 → 4.14 s; 10 Hz N=100 → ≈8.3 s. Triggered: 5.07 / 5.05 / 5.01 / 5.05 s. The ADPD's internal 960 kHz RC oscillator (`OSC_960K_FREQ_ADJ`, never calibrated by this firmware) sets the free-run rate, so `arrun`'s "freq" is honoured only to the RC tolerance and the Ambyte's assumed point timing is off by the same factor. The triggered engine paces from the ESP crystal-derived `esp_timer` and is exact. **This is a second, independent reason for the swap and belongs in §1** |
| s_630 mean triggered vs free-run | 163–166 vs 151–153 (+9 %, SEM ≈1.5) with r_630 equal. **Resolved 2026-09-03 with `traw` (below): a ≈12-count dark-level offset on channel 1 of the fluor slot, not a gain change** |

**V1 third and fourth sessions — 2026-09-03, builds with the BOOT-gesture pause, `traw`,
`tquiet` (`v1c_bench.py`, `v1d_bench.py`).**

| Item | Result |
|---|---|
| BOOT-gesture pause (`ambit_boot_gesture_pause/resume` around the triggered run) | `arrunt` at 90 / 100 / 125 Hz, N=300 each: all complete, no reset. Free-run 100 Hz control clean. **The 100 Hz reset is closed.** PCB netlist: GPIO9 (U8.15) goes only to J1.1, the FFC to the Ambyte — the gesture is the Ambyte's remote reset, no pull-up on board besides the ESP's 45 k |
| `traw` raw dark/lit, both engines, 200 Hz and 10 Hz | lit values identical to the count in both engines (s_lit 16 394–16 401, r_lit 21 965, r_dark 16 390). **s_dark is 10–14 counts lower in EXT_SYNC** (16 472–16 473 vs 16 481–16 487); s_lit−s_dark −75…−80 vs −82…−93. That difference, run through `calc_signal`'s +348 pedestal, is the whole "+9 %" — a 0.07 %-of-full-scale offset on channel 1 of slot B. Negligible on a leaf; **relevant for the baseline**: `fluor_offset()` / `baseline` acquire in free-run, so a triggered run subtracts a baseline ≈12 counts off. Phase 5 must re-take baselines with the shipping engine |
| Slot-C noise vs SPI activity (`tquiet` 0 / 300 / 380 / 400 µs of bus silence after the edge) | r_730 sd 88 / 83 / 81 / 78, repeat at 0: 96; free-run with the core light-sleeping: 42.7 / 42.2. **Bus silence changes nothing.** `traw` mode 0 (free-run, core awake polling) also gives 78–96. So the correlate is "ESP core awake during the sequence", not SPI traffic and not EXT_SYNC — next test is light-sleeping through the sequence (`tsleepq`) |
| Slot-C means | s_730 −0.4 %, r_730 −1.6 % in EXT_SYNC vs free-run, every session; r_630 equal. Unexplained; belongs with the noise item |
| Post-idle transient | first run after boot: r_630 = 4879, 6231, 5858, then 5823 steady — three settling samples, also seen in free-run. Back-to-back runs never show it. Phase 3: warm-up sequence or a documented discard count, not just drop-first |
| Free-run time base | root cause found: the clean-room driver never loads the factory 960 kHz trim (ADI `adi_adpd6000_device_cal_960k_osc`). Handed off as `plans/ADPD_OSC_TRIM.md` for a separate PR on `main` |
| ADPD facts from the ADI header | GO_SLEEP = SYS_CTL (0x0F) bit 4, undocumented in the datasheet; that is what LEGACY's `tidle` gate 1 wrote. No sync start-up-delay register exists on the ADPD6000 |

**Slot-C noise, sessions five and six (2026-09-03, `v1e_bench.py`, `v1f_bench.py`) — the ESP is
ruled out; it is the 730 slot itself under external sync.**

| ESP behaviour during the sequence (all 200 Hz, N=1000, same target, warm) | r_730 sd | s_730 sd |
|---|---|---|
| free-run, core light-sleeping, FIFO read in bursts (`arrun2`) ×4 | 35–39 | 12 |
| EXT_SYNC, polling from the edge (`quiet=0`) ×3 | 87–96 | 15–16 |
| EXT_SYNC, bus silent 300–400 µs then poll | 78–83 | 11–15 |
| EXT_SYNC, bus silent 450 / 500 / 600 / 800 µs (poll strictly after the sequence) | 80 / 80 / 84 / 74 | 13–15 |
| EXT_SYNC, core light-sleeping 300 / 350 µs after the edge (`tsleepq`) | 87 / 90 | 15 |

Neither SPI traffic nor the awake core moves it. The distributions differ in shape, not just
width: free-run r_730 sits in a ±12 band (p10–p90 2535–2560) with ≈4 % outliers; EXT_SYNC r_730
spreads p10–p90 2396–2604 with two clusters (≈2400 and ≈2550) and a 1.5–2 % lower mean; s_730
follows at a smaller scale; r_630 (slot B, same LED-offset/LIT-offset settings, red LED) is
identical in both engines.

Working hypothesis: **slot C samples on the knee of the IR photodiode rise.** Slot C uses
LED_OFFSET 60, LED_WIDTH 19, LIT_OFFSET 72 — the lit samples start 12 µs after LED-on. The
datasheet's IR empirical values are tD_RISE 10 µs, LED_WIDTH 36, LIT_OFFSET 10 µs after LED-on
with a 36 µs pulse (Table 15); red tD_RISE is 6 µs, so slot B is settled at 12 µs and slot C is
not. Under an asynchronous external edge the sequence start is not phase-locked to the internal
clocks the way a timer start is, so a sub-µs to 1 µs phase difference between LED-on and the
1 µs ADC grid turns into a large value change on the knee — and into a two-level distribution.
Free-run starts every sequence on the same clock phase, so the knee error is a constant offset,
not noise. **Next test:** widen slot C's LED pulse and move LIT_OFFSET later (e.g. width 36,
LIT 82, DARK2 after LED-off + IR tD_FALL 40 µs) via a diag knob and compare r_730 sd in EXT_SYNC
against free-run. If it tightens, the fix is a slot-C timing change (Phase 3) that also improves
free-run's 730 channel; if not, the chip's sync path is the cause and 730 in EXT_SYNC stays
noisier than free-run.

**Session seven (2026-09-03, `tslotc`, `v1g_bench.py`) — the knee hypothesis is refuted.**

| Slot C LIT / LED_WIDTH / DARK2 | r_730 sd free-run | r_730 sd EXT_SYNC | s_730 sd free / EXT |
|---|---|---|---|
| 72 / 19 / 90 (production) | 41, 38 | 92.5, 92.8 | 5.6 / 10.8 |
| 82 / 36 / 136 (DS IR-like) | 66 | 101 | 7.9 / 11.5 |
| 76 / 24 / 100 | 58 | 108 | 7.3 / 12.3 |
| 92 / 36 / 136 (LIT 32 µs after LED-on) | 50 | 108 | 6.2 / 11.4 |

The EXT_SYNC/free-run ratio stays ≈2–2.5× at every timing, including sampling 32 µs after
LED-on, and r_630 (slot B) is 1.6–1.8 in every run. So it is not the photodiode rise. The
wider LED pulses also make free-run's 730 noisier (41 → 50–66), so production timing stays.

Also from that session: **the earlier `tsleepq` test never slept** — `sleepq_rejects=1000`,
each attempt returned after 115–160 µs. `esp_light_sleep_start()` refuses durations shorter
than its own entry/exit overhead, so 300–350 µs was rejected and the "core awake" hypothesis is
still untested. Next: `tsleepq` 1500 / 2500 µs (a 5 ms period leaves room), and `tinteg` 1 vs 4
on slots A/C — NUM_INT=4 is the one property slots A and C share and slot B (unaffected) lacks.

**Session eight (2026-09-03, `tinteg`, `tsleepq` 1500/2500, `v1h_bench.py`) — ROOT CAUSE FOUND:
the running ESP core.** With the core light-sleeping through the sequence (rejects = 0, slept
1600–2620 µs) every EXT_SYNC-vs-free-run difference disappears:

| 200 Hz, N=1000 | s_630 | r_730 sd | r_730 mean | s_730 sd | s_730 mean |
|---|---|---|---|---|---|
| free-run (×2) | 100 ± 42, 105 ± 42 | 36.6, 40.3 | 2546 | 5.5, 5.9 | 1677 |
| EXT_SYNC, core busy-waiting | 112 ± 52 | 88.0, 84.9 | 2500 | 10.3, 10.0 | 1668 |
| EXT_SYNC, core light-sleeping 1500 µs | 106 ± 44 | 40.5 | 2548 | 5.6 | 1677 |
| EXT_SYNC, core light-sleeping 2500 µs | 107 ± 43 | 44.5 | 2548 | 5.7 | 1677 |

Integration is irrelevant: sd/mean stays 3.5 % (busy) vs 1.5 % (free-run) at NUM_INT 1, 2 and 4.
SPI silence with a spinning core did not help (sessions 5–6), so it is the core's activity —
supply/ground coupling into the 730 channel, the reference photodiode path most of all — not the
bus. Free-run was only ever quiet because `run_arr_type1` light-sleeps between FIFO drains.
The −1.6 % r_730 mean shift and the ≈12-count s_dark offset are the same effect.

Consequence for the design: **the triggered engine must keep the core quiet while the ADPD
converts.** Light sleep is refused below ≈1 ms of requested sleep and costs ≈100 µs entry/exit,
which caps that method near 400 Hz; a blocked task (idle → WFI) on a one-shot timer is the
candidate for higher rates (`twfi`, session nine). Hardware note for the next PCB spin: the 730
reference path is sensitive to core activity; check decoupling/ground return on PD4 and VC2.

**Session nine (2026-09-03, `v1i_bench.py`) — quiet-method characterisation at 200 Hz:**

| Core during the sequence | requested | actual | r_730 sd | s_730 sd | s_630 |
|---|---|---|---|---|---|
| busy-wait | — | — | 88.6 | 10.4 | 107 ± 50 |
| light sleep | 500 µs | 818–884 µs, 0 rejects | 42.0 | 5.7 | 102 ± 40 |
| light sleep | 800 µs | 900–924 µs | 39.7 | 5.3 | 104 ± 42 |
| light sleep | 1000 µs | 1100–1121 µs | 44.5 | 5.9 | 104 ± 40 |
| WFI (task blocked on esp_timer, idle task) | 450 µs | 468–477 µs | 62.7 (200 Hz), 70.6 (500 Hz), 67.6 (1 kHz) | 7.3–8.4 | 109–114 ± 47 |
| free-run (×2) | | | 35.0, 42.0 | 5.2, 5.9 | 100–101 ± 42 |

Light sleep is accepted from a 500 µs request and always lasts ≥ ≈820 µs (the SDK adds ≈300 µs
entry/exit; 300 µs requests were refused), so it fits periods ≥ ≈1.3 ms (≤ ≈750 Hz). WFI costs
what is asked and recovers about half the noise. **Engine rule (landed):** per line, light-sleep
through the sequence when the period allows, else WFI, else busy; `tstat` reports `quiet_mode`.

**Session ten (2026-09-03, automatic quiet mode, `v1j_bench.py`) — VERIFIED.** The target was
moved during the first two runs of the session (s_730 6500 → 1300, drifting for a minute), so
s_730 spreads here are confounded by the target; r_730 (reference, target-independent) is the
metric. Counts exact, `late=0`, `sleepq_rejects=0` in every run.

| Line | quiet_mode chosen | actual quiet | r_730 sd trig / free-run | TIMING |
|---|---|---|---|---|
| 10 Hz N=100 | sleep | 823–886 µs | 47.8 / 24.3 (target still drifting) | 9.95 s |
| 200 Hz | sleep | 818–885 µs | 47.7 / 41.5 | 5.07 s |
| 500 Hz | sleep | 823–877 µs | 42.8 / 49.1 | 2.07 s |
| 700 Hz | sleep | 817–885 µs | 44.0 / — | 1.46 s |
| 1000 Hz (×2) | WFI | 468–480 µs | 67.2, 66.8 / (busy was 88–92) | 1.05, 1.03 s |
| far-red 200 Hz | sleep | | 47.6 | 5.45 s (floor-limited, as before) |
| type-2 200 Hz | sleep | | — | 5.03 s, no count glitch |

**Gate V2 — 2026-09-03, `v2_bench.py` (`tblk`, `tovf`, production runs with the block read).
PASS on all three criteria.**

| Test | Result |
|---|---|
| `tblk,4000,500` ×3 + `tblk,2000,1000`: interleaved per-value / block reads, 7000 sequences per method = 112 000 values | every column agrees within noise (largest Δmean 4.7 on r_730 whose sd is 90; r_dark/r_lit/sun Δ ≤ 0.1), sd equal; `rc_err=0/0`; **`leftover=0/0`**; every sequence carried values > 65 535 (raw sun ≈ 65 846, leaf ≈ 65 485) so the 3-byte MSB-first unpack is exercised on all 14 000 |
| read time | per-value 169–171 µs, block 46 µs (`read_max_us` 45–55 in production runs, poll exit to data) |
| production runs with the block read: 1 kHz ×2, 500 Hz, 200 Hz, far-red 200 Hz, type-2 1 kHz, N=1999/1000 | counts exact, `late=0`, `leftover=0`, `io_error=0`, timing exact |
| `tovf,50` (31 unread sequences injected before sample 50 → 744 B into a 640 B FIFO) | `ERROR arrunt -5` after 0.6 s, `residual_bytes=576 residual_sample=50 fifo_status=0x3A40` (bit 13 INT_FIFO_OFLOW set, count 576); no hang. The next run was clean (500 samples, `leftover=0`) — cleanup's STOP + CLEAR_FIFO recovers |
| `traw,1,100,200` through the block read | raw sun 65 845.6, 100/100 values > 65 535 |

**Rate bands after Phase 2 (`kTrigReadBudgetUs` = 150), N=1999 each, counts exact, `late=0`:**

| Rate | quiet_mode | r_730 sd | TIMING | Note |
|---|---|---|---|---|
| 700 Hz | sleep | 42.6 | 2.908 s | parity with free-run |
| 900 Hz | sleep | 46.8 | 2.255 s | parity; the new upper edge of the sleep band |
| 1000 Hz | WFI | 65.0 | 2.092 s | ≈1.5× free-run (accepted, 730 not used in normal measurements) |
| 1500 Hz | WFI | 65.1 | 1.365 s | |
| 2000 Hz | busy | 93.5 | 1.036 s | `max_late_us=46`, zero lost edges: **the engine now reaches 2 kHz** with exact counts |

Ceiling after Phase 2: ≥ 2 kHz exact-count (chip limit ≈ 2.3 kHz from t_seq); it was ≈1.6 kHz
with the per-value read.

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
- **2026-09-03:** `ambit_boot_gesture_pause()/resume()` in ambit-1.ino, called around the
  triggered run (V1 reset root-caused to the BOOT-pin gesture ISR); `tstat` gained
  `glitch_bytes/glitch_sample/park_hz/quiet_us`; diagnostics `traw` (raw dark/lit dump, both
  engines), `tquiet` (SPI-silent wait after the edge), `tsleepq` (light-sleep through the
  sequence). All diag-gated.
- **2026-09-03, later:** automatic core-quiet mode (`trig_pick_quiet_mode`, `trig_quiet_sleep`,
  `trig_quiet_wfi`, constants `kTrigQuiet*`), reset to BUSY at cleanup and in every diagnostic
  so `tseq` measures the chip; `tstat` gained `sleepq_rejects/min/max`, `wfi_us`, `quiet_mode`.
  Diag knobs kept (all `-DAMBIT_DIAG_TRIGGER` only): `tpark`, `tquiet`, `tsleepq`, `twfi`,
  `tslotc`, `tinteg`, `traw`, `tdrop`, `tstat`, and for V2 `tblk` (per-value vs block) and
  `tovf` (FIFO overflow injection).
- **Phase 2 (2026-09-03):** `ADPD6::readfifo_block()` (u_adpd6100.{h,cpp}); triggered engine
  uses it with the rc checked (−6 on I/O error); `fifo_count(&c)` rc checked in the poll (−6 I/O,
  −5 out-of-range); FIFO_STATUS captured at abort; `tstat` gained `io_error`, `fifo_status`,
  `read_max_us`, `leftover` (leftover check compiled only with the diag flag);
  `kTrigReadBudgetUs` 350 → 150. Bench driver scripts lived in the session
  scratchpad (`v1*_bench.py`); they are ≈100 lines of pyserial each and trivial to recreate:
  open COM61 at 115200, wait for the boot banner, send a line, read until `Data sent` /
  `ERROR` / `tstat:` or an `ESP-ROM:` banner, summarise the `Data:` arrays.
- **Open before Phase 2**: V1a on the scope when the trace is opened; measure the far-red tail
  and replace the provisional floor; re-measure the free-run-vs-triggered dark offset with the
  quiet mode on (expected to be gone); Phase 3 warm-up policy for the post-idle transient.

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
