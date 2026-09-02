# Branch plan: `deterministic-adpd` — exact-N triggered ADPD acquisition

> Status: **Phase 0 not started on this codebase.** This document ports the ideas from the
> abandoned `feature/arrun-deterministic-trigger` branch of the pre-cleanroom repo
> (`LEGACY/ambit-IoT`, last commit 2026-07-03) onto today's single-image firmware. Nothing
> from that branch has been cherry-picked; the two repos share no git history.

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
   `fifo_count()` byte-count read artifact, not an extra LED pulse. Recovery = `clear_fifo()`,
   as `run_trigger_spacer` already does.
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
7. **Expected ceiling** (from datasheet arithmetic, not measured): ~500 Hz safe /
   ~1.2–1.5 kHz at `num_ts=3`; ~300 Hz with far-red. Bounded by sequence time, dominated by
   the 4× integration on the ambient/730 slots.

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

**Gate V1** — on a scope / logic analyser with the LED-driver line and GPIO10: V1a trigger width
and no runts; V1b **zero extra pulses**; V1c no self-trigger; V1e three-count correlation
(GPIO10 == LED sequences == stored == N) at 10/100/200/500/1k Hz, type-1, type-1+farred, type-2;
V1f first-sample integrity (`tratio` decides warm-up/drop-first); V1g timeout aborts cleanly with
a console error; V1h teardown read-back (`arrun` free-run works correctly right after an aborted
`arrunt`); V1i N-cap boundary 1999 ok / 2000 rejected; V1j mean interval and drift.

### Phase 2 — Batched FIFO read
- Add an `ADPD6::readfifo_block()` that calls `Driver::read_fifo(buf, expected_readout_bytes)`
  once and unpacks MSB-first; keep the per-value path for the other callers. Check the return
  code (the current callers ignore it).
- Make `fifo_count()` overflow observable to the triggered path (a sentinel, or use the
  two-arg overload and treat `kOutOfRange` as an abort), instead of the silent 0.

**Gate V2** — bit-exact vs per-value over ≥100 k samples including values > 65 000; overflow
fault-injection aborts instead of hanging; `fifo_count()==0` after every read.

### Phase 3 — Speed knobs
- Expose `num_integration` for the ambient/730 slots (`repeats_only(0/2, integ, 1)`) as a
  header-level parameter of `arrunt`, default unchanged (4). Fluorescence stays integ=1.
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

| Gate | Config | Result |
|---|---|---|
| V0 `tseq,500,0,1` | num_ts=3, amb/730 integ=1 | |
| V0 `tseq,500,0,4` | num_ts=3, amb/730 integ=4 | |
| V0 `tseq,500,1,4` | num_ts=9 far-red, frrep=1 | |
| V0 `tidle,0,1,0` | | |
