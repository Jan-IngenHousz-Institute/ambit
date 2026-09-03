# Handoff: Phase 3 of `plans/DETERMINISTIC_ADPD.md` — settle, characterise, and harden `arrunt`

> Status: **executed 2026-09-03 on unit `AD88`** (results in `plans/DETERMINISTIC_ADPD.md` §8, Phase 3
> table). Items 1.1–1.4, 1.6–1.8 closed; 1.5 (over-rate policy) is a user decision; V1a still TODO.
> Kept as the record of method. Originally written 2026-09-03 after Phases 0–2 passed their gates.
> Read `plans/DETERMINISTIC_ADPD.md` first: the status line, §4 (learnings 7–10), §5 (invariants
> 8–12), §8 (bench sessions, all numbers), §9 (what landed), §10 (datasheet review). This file
> tells you what Phase 3 is *now*, item by item, with the experiment, the commands, the pass
> criterion and the code pointer for each. Everything here runs on the branch
> `deterministic-adpd`; `arrun` / cmd 21 stay untouched (plan §5 invariant 6).

## 0. Where things stand

- The triggered engine (`run_arr_trigger`, `src/PAM.cpp`) is exact-count from 1 Hz to 2 kHz for
  all three line shapes, aborts cleanly on a lost edge (−4), FIFO desync/overflow (−5) or SPI
  fault (−6), rejects bad lines (−2), and keeps the ESP core quiet while the ADPD converts
  (light sleep ≤ ≈900 Hz, WFI ≤ ≈1.5 kHz, busy above). Text verbs `arrunt/arrunt1/arrunt2`, JSON
  `arrunt`, core op `core_run_array_triggered()`.
- Every diagnostic is a console verb behind `-DAMBIT_DIAG_TRIGGER` (on in `platformio.ini` for
  the branch): `tseq tidle tratio tstat tdrop tpark tquiet tsleepq twfi tslotc tinteg traw tblk
  tovf`. `tstat` prints the last run's statistics; read `print_trig_stats()` for the fields.
- Phase 3's original list (plan §6) was written before the bench. Two of its items are settled:
  integration is **not** a speed knob (13 µs for NUM_INT 1→4), and the read-out is already
  single-transaction (Phase 2). What remains, and what the bench added, is below.

## 1. Items

### 1.1 Post-idle warm-up policy (the "three-sample transient")

**Observed** (§8, sessions 3, 7): the first run after a long idle or a cold boot returns r_630 =
4879, 6231, 5858 then steady at 5823 (−16 %, +7 %, +0.6 %), in *both* engines. Back-to-back runs
never show it; ~1 min idle gave +2.5 %. r_630 is the reference photodiode looking at the LED, and
a cold LED reads bright, not dim, so LED thermal is out. Best hypothesis (§10 reasoning): the ADC
references VREF1/VREF2 (1.2 V, decoupled with 2.2 µF here vs the 1.0 µF the datasheet asks for)
power down in standby and settle over ~10 ms after GO; standby "resets the digital portion of
the ADC, all pulse generators and the state machine".

**Experiment** (decides time-based vs per-sequence):
1. Add a knob `twarm,<ms>`: in `run_arr_trigger`, after `adpd.RUN()` and `delay(kTrigArmSettleMs)`,
   wait `<ms>` more before the first edge. Follow the `tquiet` pattern: a `static uint32_t`, a
   `diag_set_*()` printer, a `case hash("twarm")` in `src/do_command.h`, a decl in `src/PAM.h`
   inside the `#ifdef AMBIT_DIAG_TRIGGER` block, and print the value in `tstat`.
2. Bench: power-cycle or leave the DUT idle ≥ 5 min. Run `twarm,0` then
   `arrunt2,1,0,1,0,0,100,0,200,0,1` (200 Hz, N=100) and inspect the first 5 r_630 values. Idle
   ≥ 5 min again, `twarm,50`, same line. Idle again, `twarm,200`, same line.
3. Interpretation: first sample clean at `twarm,50` → **time-based**, the fix is a fixed
   pre-delay after `RUN()` (make `kTrigArmSettleMs` ≈ the measured settle, or a separate
   `kTrigWarmupMs`). First sample still low at `twarm,200` → **per-sequence**, the fix is N dummy
   sequences after arming (fire N edges with `trig_fire_and_wait`, read and discard) before the
   first stored one; in EXT_SYNC three dummies cost ≈ 1.3 ms.

**Implement** the winner in `run_arr_trigger` only (behind the wire; the frozen `arrun` gets it
via `plans/ADPD_OSC_TRIM.md`-style handoff if wanted). Keep `num_ptx` = stored count exact: dummy
sequences are *extra* edges, so document them in the plan (§5 invariant 2 says
"sequences == edges == stored"; amend it to "stored sequences" and state the warm-up count).

**Pass:** after ≥ 5 min idle, first stored sample within 1 % of the run median on r_630, three
runs. Record idle time and result in plan §8.

### 1.2 Far-red period floor: replace the estimate with a measurement

**Now:** `trig_farred_sequence_us(repeats) = 1000 + 2200·repeats` µs, a datasheet-arithmetic
estimate (§4.2, §4.7). The register maths says the tail is 6 slots × (MIN_PERIOD 320 µs ×
repeats + ≈60 µs) ≈ 1.92 ms·repeats + head 425 µs, i.e. the floor has 15–40 % margin and caps
far-red near 312 Hz at `repeats`=1 (measured: 200 Hz far-red runs at 185 Hz, 500 Hz at 312 Hz).

**Experiment A (software, no scope):** add `tseqfr,<frrep>,<start_us>,<step_us>,<reps>`: arm
EXT_SYNC with the far-red config (`diag_config(true, 4, frrep)`), then for an inter-edge delay
starting at `start_us` and shrinking by `step_us`, fire `reps` edges at that delay and count
timeouts (`trig_fire_and_wait` false). The largest delay that still loses edges is the tail
boundary; report it per `frrep` = 1, 2, 4, 40 (the values `_repeats = 400/freq` produces at
400/200/100/10 Hz). Model `trig_farred_sequence_us()` from the measured points + 10 % margin.

**Experiment B (scope, if available):** LED-driver line, far-red line at 500 Hz requested
(`arrunt2,1,0,1,1,3,232,1,244,0,1`, `repeats`=1): measure one burst from its first pulse to the
last far-red pulse. Expect ≈ 2.3 ms; compare with A.

**Pass:** far-red at the new floor: 1000 edges → 1000 stored, zero timeouts, at every `frrep`
tested; the far-red rate cap moves from ≈312 Hz to whatever the measurement allows. Update
§4.2/§4.7 and the function comment.

### 1.3 Arm settle (`kTrigArmSettleMs` = 5 ms after `RUN()`)

Inherited from `run_trigger_spacer`. With 1.1 settled, test 5 → 1 → 0 ms at 200 Hz, N=1000,
three runs each, back-to-back (so the post-idle transient does not confound): first-sample r_630
within 1 % of median and no lost edge → keep the smallest that passes. Small win (per line, not
per sample), low priority.

### 1.4 PLOTTING output in the hot path (`arrunt1`)

`arrunt1` prints one line per sample from `pam_store_type1_sample()` (`Serial.printf` +
`Serial.flush()`, ~1 ms at 115200). Above ≈ 300 Hz that stalls the pacing; `tstat` will show
`late` climbing. Options: (a) buffer the PLOTTING lines and emit them after the line completes
(loses the "live" character but keeps timing), (b) reject `arrunt1` above a rate with an explicit
`ERROR`, (c) accept lateness and document. Recommend (a) with a bounded ring or (b). Measure first:
`arrunt1,1,0,1,0,3,232,0,200,0,1` then `tstat`, repeat at 500 Hz. Whatever is chosen, the
COMPUTER (`arrunt2`) and JSON paths are unaffected (they print after the run).

### 1.5 Over-requested rate: silent cap or reject? (protocol decision, needs the user)

A line asking for more than the ceiling does not abort: the loop waits for the FIFO, finds the
next edge overdue and fires at once, so the run completes at the ceiling (5 kHz request →
1.54 kHz measured, `tstat late=1000 max_late_us=421`). Count stays exact, timing is not what was
asked. Options: reject `freq` above a constant (≈2000 Hz measured ceiling, chip ≈2.3 kHz) in
`run_arr_trigger_validate()` with −2, or accept and report via the TIMING block. Put the choice
to the user; whichever way, write it into plan §6 and the `arrunt` docs.

### 1.6 SNR study — only if a leaf target is available

Integration is settled as a speed non-issue. The remaining question from the original Phase 3
is SNR: `F(integ1)` vs `F(integ4)/4` within noise, and whether exposing `num_integration` per
line buys anything. This needs a real fluorescing target (all sessions so far measured "in air",
s_630 ≈ 100 counts = pedestal). Use `tinteg,<n>` (already there, applies to both engines) and
`arrunt2` at 200 Hz, N=1000, on a leaf; compare s_630/r_630 mean and sd at NUM_INT 1, 2, 4.
Decision rule: expose the knob only if integ 1 gives ≥ the same F SNR at ≥ 2× the sample rate;
otherwise leave 4 and close the item. Note the user's remark: 730 noise is acceptable because 730
is not used in normal measurements — do not spend time on 730 SNR.

### 1.7 Gate V3 as written (plan §6), adapted

- Max-rate exact count + framing: ≥ 100 k samples at 2 kHz (50 × N=1999), zero timeouts, zero
  residuals, `io_error=0`, `leftover=0` — script it (`tstat` after each run).
- Back-pressure guard band ≥ 1.5× `t_seq` — the engine has no back-pressure any more (one edge,
  one read); replace with: at 2 kHz, `max_late_us` < 10 % of the period across the 100 k run.
- ≥ 10 min thermal soak at max rate: 2 kHz N=1999 back-to-back for 10 min; watch r_630 mean
  drift (LED heating) and `late`. Report drift as an observation, not a gate failure.
- Zero SPI errors: `io_error=0` throughout (already instrumented).

### 1.8 Baselines with the shipping engine (Phase 5 prep, note only)

`fluor_offset()` / the `baseline` verb acquire in free-run. Before the core-quiet fix the triggered
dark level was ≈12 counts off; with the fix the two engines matched within noise (§8 session 8).
Re-measure once with `traw,0,200,200` vs `traw,1,200,200` (s_dark column) and record; if they
match, baselines can stay free-run; if not, `fluor_offset()` must gain a triggered variant.

## 2. Bench recipe (what worked for ten sessions)

- **DUT** `AmbitV003` on **COM61** (Teensy-class USB bridge, VID 16C0). The user's terminal must
  be closed; opening the port may power-cycle the board (`rst:0x1 POWERON`) — wait for the boot
  banner (`FW: 1.2.0` line) before the first command.
- **Flash:** `pio run -t upload --upload-port COM61` (the pio binary is
  `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`; not on PATH in the agent's Bash). Standing
  permission to flash was given by the user for the 2026-09-03 session only — ask again.
- **Driver script pattern** (pyserial from the pio venv): open at 115200, wait for the banner,
  for each command: `reset_input_buffer()`, write `cmd + "\n"`, read until `Data sent` / `ERROR` /
  `tstat:` / your verb's end marker or an `ESP-ROM:` banner (a reboot), else 1.5 s quiet; log
  raw text; summarise `Data:<tag>,Length:N  v,v,...` arrays as mean/sd/percentiles. Send `tstat`
  after every `arrunt`. ~100 lines; earlier copies were in the agent scratchpad and are gone.
- **Console line format:** verb terminated by `,` or `:`; `Serial_Input_Long` times out 10 ms
  between tokens, so send whole lines at once. `arrunt2,<len>,<persist>,<8×len bytes>`, byte
  order `type, farred, N_hi, N_lo, freq_hi, freq_lo, actinic, subsampling`. N=1999 → `7,207`;
  N=1000 → `3,232`; 1000 Hz → `3,232`; 500 Hz → `1,244`; 2000 Hz → `7,208`.
- **Always run a warm-up line first** and discard it (post-idle transient, 1.1). Keep the target
  still for a whole session: r_730 is target-independent (reference PD on the LED) and is the
  right noise metric; s_730/s_630 move with the target.
- **Do not judge s_630 SNR in air**: ≈100 counts there is `calc_signal`'s pedestal, not signal.
- **BOOT gesture:** the triggered engine pauses it; free-run `arrun` at 90–125 Hz can still reset
  the DUT (handoff `plans/ADPD_OSC_TRIM.md` item B). If a run "reboots" with nothing before
  `ESP-ROM:`, that is what happened.
- **Adding a knob:** copy `tquiet` — one static, one `diag_set_*()` that prints, a `case` in
  `do_command.h`, a decl inside the `#ifdef AMBIT_DIAG_TRIGGER` block of `PAM.h`, and a field in
  `print_trig_stats()`. Reset any per-run state at `cleanup:` and in `diag_arm()` (see how
  `g_trig_quiet_mode` is reset) so `tseq` never inherits it.
- **Agent tooling gotcha:** long inline Python heredocs fail in the Bash tool on this machine;
  write scripts to a file and run them. Use the Windows path form for Python file arguments.

## 3. Exit criteria for Phase 3

1.1 policy implemented and verified; 1.2 floor measured and replaced; 1.3 decided; 1.4 decided
and implemented; 1.5 decision recorded; 1.6 closed either way; 1.7 numbers in plan §8; 1.8 noted.
Then update `plans/DETERMINISTIC_ADPD.md` (status line, §6 Phase 3, §8, §9) and hand over to
Phase 4 (binary adapter — needs a cmd id agreed in the Ambyte repo's `ambit_protocol.h` first).
