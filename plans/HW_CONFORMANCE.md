# amBIT HW Conformance Test — Step 4b (binary frontend → core)

**Status: PASSED 2026-06-05.** Step 4b's binary path is HW-verified byte-conformant
against a real ambyte; the dirty-mark change (§4) was accepted. Keep this procedure as the
**reusable conformance gate** for any future change touching `run_esp.cpp` or the
`data_utils.cpp` FSM (Step 5 onward) — re-run it after each such change.

It is the verification gate for the binary/ambyte wire contract (MERGE_PLAN §2, §13).

---

## 0. What Step 4b changed (what's at risk)

`run_esp.cpp` cmd `1/2/10/20/21` now call shared `core_*` functions instead of inline
logic. The intent is **zero change on the wire**:

- Every `Serial.write(ESP_CMD_DONE/END)` and `Serial.readBytes(...)` is byte-for-byte
  unchanged; only the internal config/run calls were factored out.
- Wire normalization still lives in the adapter: gains 1-indexed (1..6, stored −1),
  current byte `≥127` = "no change", MPF length base-128 (`cmd[1]<<7 | cmd[2]`).

**One intentional internal change** (not a wire change): binary `set_gains`/`set_currents`
(cmd 1/2) now mark the config *dirty* (they didn't before). Consequences:
- Normal flow `cmd1 → cmd2 → cmd10 → cmd21`: **identical** (cmd 10 re-applies config
  regardless of the dirty flag).
- Degenerate flow `set gains/currents → cmd21 without cmd10`: now correctly re-applies
  the just-set values (previously could run with stale config). This is the only
  observable behavioural delta, and it is a fix — confirm no ambyte workflow relied on
  the old stale-config behaviour.

---

## 1. Setup

- **DUT:** an amBIT flashed with the current `fw_new` (post-Step-4b):
  ```
  cd fw_new
  "$HOME/.platformio/penv/Scripts/pio.exe" run -t upload     # add -t upload to flash
  ```
- **Host (one of):**
  - **A real ambyte** (preferred) on the FFC/UART0 link, driving the amBIT via its
    `device.ambit_*` Lua commands / CLI (`ambit_*`). This exercises the genuine binary
    protocol end-to-end.
  - **A logic analyzer / UART sniffer** on the FFC TX & RX (115200 8N1) to capture raw
    bytes for the strict byte-diff (Test A). Best combined with the ambyte.
- **Reference build (for the strict diff):** a pre-Step-4 amBIT image — e.g. `git stash`
  the working tree, or check out the commit before `core.{h,cpp}` was added, build, and
  keep `firmware.bin` aside as `firmware_preStep4.bin`.

---

## 2. Test A — byte-level framing conformance (gold standard)

Goal: the **protocol framing** is identical pre- vs post-Step-4b. (Measurement *payload*
values vary run-to-run with sensor noise — only the framing/lengths/checksum validity are
deterministic; do not expect data bytes to match.)

1. Flash `firmware_preStep4.bin`. With the sniffer capturing, drive the standard sequence
   from the ambyte (see §3 for the sequence). Save the trace as `pre.bin`.
2. Flash the post-Step-4b image. Repeat the **identical** host sequence. Save `post.bin`.
3. Compare. For each command, the following must match exactly:

| Command | Deterministic bytes that MUST match |
|---|---|
| wake | host `0xAA 0xAA 0xAA` → amBIT `0x80` |
| set_gains (1) | frame `0xA0`+8B cmd → `0xA1` (CMD_DONE). No data. |
| set_currents (2) | frame → `0xA1` |
| config (10) | frame → `0xA1` |
| get_spec (31) | `0xA1` + **24 B** + `0xF0` |
| get_spec_raw (35) | `0xA1` + **32 B** + `0xF0`. Additive command — absent from any pre-v1.2.0 reference build, so it has no `pre.bin` counterpart; verify the length and layout against §6 instead of diffing. |
| get_temp (32) | `0xA1` + **4 B** + `0xF0` |
| get_info (33,2) | `0xA1` + **`sizeof(ambit_fw_info_t)` B** + `0xF0` |
| get_temp_raw (34) | `0xA1` + **14 B** + `0xF0` |
| run (21) / mpf (20) | `0xA1` … FSM arrays (`0xD3` wake, `0xD2` ack, `0xD4 0x96` len-hdr w/ checksum byte, data, `0xD4 .. checksum`, `0xB4` pass) … `0xF0`. **Array count, each array length, header checksum, and data byte-sum checksum must verify.** Data values may differ. |

**Pass:** all framing/handshake/length/checksum bytes identical; only measurement payload
values differ. **Fail:** any framing byte, length field, command-ID echo, or checksum
*scheme* differs → a contract break; do not ship, diff the offending command.

> **Expected value delta from the spectral clamp (v1.2.0).** cmd 31's ten channel words
> are `counts × Spec_COE*` in a `uint16`, which used to wrap; they now saturate at 65535,
> and its PAR float is computed from the unclamped counts. Under bright illumination the
> post image therefore reports *different values* for cmd 31 — larger words and a much
> larger PAR (a wrapped F1 at 5462 counts read 8 and PAR 0.19; it now reads 65535 and PAR
> 1573). This is the intended fix, not a regression. **Framing, length and byte count for
> cmd 31 are unchanged and must still diff clean.** To compare values, take the reference
> and post traces under illumination dim enough that no channel exceeds its wrap threshold
> (F1 ≤ 5461 counts is the tightest); there the two images agree bit-for-bit.

> If no sniffer is available, Test A reduces to "the ambyte completes every command in §3
> with no timeout / no `checksum mismatch` / no `CMD_DONE not received` log" — a weaker but
> still useful gate (`uart_sensors.c` logs all three on failure).

---

## 3. Test B — functional regression (per MERGE_PLAN §14)

Drive each command class from the ambyte and confirm it completes and the numbers are
sane. Minimum sequence:

- [ ] **wake / ping** — amBIT acks `0x80`; ambyte marks channel CONNECTED.
- [ ] **set_gains (cmd 1)** then **set_currents (cmd 2)** — each returns `CMD_DONE`.
- [ ] **config (cmd 10)** — `CMD_DONE`.
- [ ] **run (cmd 21)** with a small array — FSM round-trip completes, **all array
      checksums pass**, array count = expected (env, fluor, fluoref, sun, leaf, 730,
      730ref), values in a plausible range.
- [ ] **run_mpf (cmd 20)** — completes; length decodes correctly (test a length >127 to
      exercise the base-128 split, e.g. 200 → `cmd[1]=1, cmd[2]=72`).
- [ ] **get_spec (cmd 31)** / **get_temp (cmd 32)** / **get_temp_raw (cmd 34)** — correct
      byte counts (24 / 4 / 14), values sane (temp ≈ ambient °C).
- [ ] **get_spec_raw (cmd 35)** — 32 B, layout per §6. Check, under both dim and bright
      illumination:
      - `format == 1`; `atime == 99`, `astep == 499`, `gain_low == gain_high == 2`
        (`AS7341_GAIN_2X`) — these echo what the firmware programs, so a mismatch means
        `dual_exposure()` and the reported metadata have drifted apart.
      - **every `raw[i] ≤ (atime+1) × (astep+1)` = 50000.** A count above full scale means
        the channel remap or the read length is wrong. `raw[i]` must never wrap — that is
        the entire purpose of the command.
      - `flags` bit0 set only when some `raw[i]` reaches 50000; clear in shade.
      - cmd 35's `par` equals cmd 31's `par` (same calibration, same acquisition) in dim
        light. In bright light both are still equal — cmd 31's PAR is now also computed
        from unclamped counts — but cmd 31's *channel words* peg at 65535 while cmd 35's
        keep resolving. That divergence is the expected behaviour.
      - Sanity: `raw[i] × Spec_COE_i` reproduces cmd 31's word wherever that word is
        below 65535 (weights 12,10,11,10,10,9,7,4,1,1 for F1..F8,NIR,Clear).
- [ ] **cmd 35 against a pre-v1.2.0 image** — an older AMBIT must simply not answer
      (unknown opcodes fall into `default:` and write nothing), so the host takes its full
      read timeout. Confirms why hosts gate on cmd 33/2 rather than probing.
- [ ] **get_info (cmd 33)** subtypes 1/2/3 — struct sizes match `ambit_protocol.h`
      (calibration ~136 B, fw_info ~48 B, metadata ~248 B).
- [ ] **baseline (cmd 6)**, **blink (cmd 5)**, **actinic (cmd 4)** — complete without error.
- [ ] **set_metadata (cmd 37)** with `eof_mark = 2025` — accepted and persisted.

**Compare measurement values** from cmd 21/20 against the pre-Step-4 reference build on the
same target/sample: they should match within sensor noise (Step 4b does not change the
measurement math).

---

## 4. Test C — the dirty-mark behavioural delta (the only intended change)

Run this exact non-standard sequence **without a cmd 10 between set and run**:

1. `cmd 10` (config) then `cmd 21` (run) with gains G1 — note the values.
2. `cmd 1` (set gains to a clearly different G2) **then `cmd 21` directly** (no cmd 10).

- **Pre-Step-4b:** the run in step 2 used **G1** (stale — the bug).
- **Post-Step-4b:** the run in step 2 uses **G2** (re-applied — the fix).

**Expected pass:** step-2 readings reflect G2. If any ambyte workflow *depended* on the old
stale-config behaviour, flag it — the dirty-mark can be removed from `core_set_gains` /
`core_set_currents` (drop the `adpd_mode = MPF_MODE` line) to restore the exact old binary
behaviour, at the cost of re-diverging from the text path.

---

## 5. Result handling

- **All pass** → mark Step 4b verified: update memory `merge-plan-ambit` ("4b verified on
  HW <date>") and tick MERGE_PLAN §14's ambyte rows.
- **Framing diff (Test A fail)** → contract break; capture the offending command's pre/post
  bytes and fix the adapter so the wire matches.
- **Test C unwanted** → remove the dirty-mark as noted in §4.

---

## 6. Reference — `get_spec_raw` (cmd 35) payload layout

Added in v1.2.0. Request is the standard `0xA0` + 8-byte frame with `cmd_arr[0] = 35`;
the remaining seven bytes are unused. Response is `0xA1` + **32 B** + `0xF0`.

Unlike the cmd 33 structs — which are frozen at 140/48/248 bytes because they are
`memcpy`'d straight out of ESP32-default-aligned C structs — this payload is assembled
byte-explicitly. Every field sits at a naturally aligned offset, so it is padding-free on
both sides without `__attribute__((packed))`. All multi-byte fields are little-endian.

| Off | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | `format` | `= 1`. Bump only on a layout change; a host must reject unknown values rather than guess. |
| 1 | 1 | `atime` | As programmed (99). |
| 2 | 1 | `gain_low` | `as7341_gain_t` 0-10 for the F1-F4/Clear/NIR SMUX bank. `2` = `AS7341_GAIN_2X`. |
| 3 | 1 | `gain_high` | Same enum, F5-F8 bank. Equal to `gain_low` today; the two diverge if autoranging is added. |
| 4 | 2 | `astep` | u16, as programmed (499). |
| 6 | 2 | `flags` | bit0 = at least one channel reached ADC full scale. Other bits reserved, sent as 0. |
| 8 | 20 | `raw[10]` | u16 × 10, **unscaled** counts in wavelength order: F1, F2, F3, F4, F5, F6, F7, F8, NIR, Clear. |
| 28 | 4 | `par` | f32, `spec_coef` applied — the same value cmd 31 reports. |

**Normalisation.** Full scale is `(atime + 1) × (astep + 1)` = 50000, which is why `raw[]`
cannot overflow its `uint16` — that is the whole point of the command versus cmd 31.
Compare exposures with `raw / (gain × (atime + 1) × (astep + 1))`.

**Relation to cmd 31.** cmd 31 reports `raw[i] × Spec_COE_i` saturated into a `uint16`
(weights 12, 10, 11, 10, 10, 9, 7, 4, 1, 1). It remains byte-frozen at 24 B and is the only
spectral command a pre-v1.2.0 image answers.

**Capability gating.** There is no negotiation on this link, and an unknown opcode gets no
reply at all (`default:` in `do_esp_cmd()` writes nothing), so a host that probes pays a
full read timeout. Gate on the firmware version from cmd 33/2 instead: `major.minor >=
1.2` → cmd 35, otherwise cmd 31.

This same harness (Tests A/B) is the reusable conformance gate for any future change that
touches `run_esp.cpp` or `data_utils.cpp`'s FSM (Step 5 onward).

---

## 7. Re-verification log (changes that require re-running this gate)

| Date | Change | Re-run | Status |
|------|--------|--------|--------|
| 2026-06-05 | Step 4b — binary frontend → core (`run_esp.cpp` cmd 1/2/10/20/21) | A + B + C | ✅ PASSED (incl. run(21)/mpf(20) FSM round-trip) |
| 2026-06-05 | Sweep #4 inc 1 — extracted `pam_send_results` from `run_arr_type1`; verbatim relocation of the `fsm_send_esp`/`send_serial` calls | A + B | ✅ PASSED — AMBYTE: run(21) = 5 arrays (idx0 len1 ENV + idx1-4 len30; 730/730ref correctly absent for no-IR), mpf(20) = 7 arrays. (COMPUTER `arrun` half is the same function's other branch — optional spot-check.) |
| 2026-06-05 | **#16 fix** — `serial_read_until`/`flush_serial` no longer echo non-target bytes (the echo injected the ambyte's 2nd `200` as ASCII `'2''0''0'` into the FSM **data phase** → checksum mismatch every run). **Not** a checksum-scheme bug: the amBIT byte-sums correctly (`u32_byte_sum`, data_utils.cpp:200/201/210, via `fsm_send_data`:432). | A + B | ✅ PASSED — array checksums clean, run/mpf complete |
| 2026-06-05 | Sweep #4 inc 2 — routed `run_trigger_spacer` (the mpf / cmd-20 path) through `pam_send_results` (`subsampling=1, has_730=true` → unconditional 7 channels; AMBYTE branch identical to the former inline block, COMPUTER branch inert as this path is never COMPUTER). `MPF` (2-channel idx 0/1), `external_trigger_run`/`_Flash`/`fluor_offset` (no FSM send) don't share this block. | B — the `mpf(20)` row of the Lua | ✅ PASSED — mpf(20) = 7 arrays, no checksum errors |
| 2026-06-05 | Cheap correctness sweep — **#10** init the comma-decl locals in `run_arr_type1`/`run_trigger_spacer`/`fluor_offset` (defensive; each was already assigned before use); **#8** `w`-length `uint8_t`→`uint16_t` + drop pointless `(uint16_t)` casts; **#9** delete 2 uncalled FSM fns (`fsm_wake_up_calls(void)`, `fsm_send_waitesp` — the latter had missing-return UB). | #10 → A+B (measurement path, behavior-neutral, very low risk). #8 text path (`w` over USB), #9 dead-code → no ambyte HW. | ☐ #10 prudent Lua re-run; #8/#9 done |
| 2026-06-08 | **Wire v2 (ambyte variant)** — (1) self-describing length header: the 2 spare bytes now carry `elem_width` (2\|4) + `dtype` (0 unsigned\|1 signed); csum formula unchanged (sum of bytes [0..6], now covers the new bytes); host defaults 4/0 when a byte is 0, so v1 still decodes. (2) ADC channels (Fluo/Fluoref/Sun/Leaf/730/730ref) sent as **uint16** (elem_width 2, dtype 0, each clamped to 0xFFFF); ENV as **int16** centi-°C (elem_width 2, dtype 1). Data-trailer csum = byte-sum of exactly the bytes sent. (3) new **TIMING** block idx 7 = `uint32[tick_begin, tick_end]` (µs from `esp_timer_get_time()`), emitted in `pam_send_results`. (4) **one wake per run**: `data_utils.cpp` FSM split into free `ambyte_wake()` + `dataclass::fsm_send_array(idx,elem_width,dtype)`; `pam_send_results` wakes once then streams each array back-to-back; host loops on the 212 marker, stops at the run's trailing 240. `fsm_send_esp` kept as a legacy wake+one-array(4/0) wrapper (MPF path). Host (ambyte) side updated to match. Both `pio run -e ambyte` and `-e cloud` build clean. | A + B | ☐ pending HW — one wake yields ENV(int16×N), Fluo/Fluoref/Sun/Leaf/730/730ref(uint16×N), TIMING(uint32×2), then 240; every header csum + data byte-sum must verify and each block decodes per `elem_width`/`dtype` |
| 2026-08-05 | **v1.1.1-rc4 host-discovery latch** (`30aaf6f`): UNKNOWN/TEXT remain awake and quiet; only a matched `0xAA` wake latches BINARY and enables light sleep/`0x85`. Binary parser/FSM code was unchanged. Exact rc4 app image (427,264 B, SHA-256 `a3c5e3f6bc9f8b55422db7f618b45e3ae1cfcef14b9512144e30f495f7c51e04`) OTA-flashed to recovered AB48. | Weak A + non-destructive B | ✅ PASSED — cold legacy/JSON first contact 10/10; cold binary wake 5/5; wake after 1.2 s idle 5/5; cmd 33/2 identity, temp/spec/temp-raw, run(21), and mpf(20) length 200 all completed with no timeout/framing/checksum error. MPF returned ENV + six 200-element arrays + TIMING. JSON arrun and malformed-extra-token rejection also passed, as did binary→JSON→binary→text→binary switching on one boot. Mutating baseline/set-metadata tests were deliberately omitted to preserve NVS calibration; no logic analyzer was available, so the strict pre/post byte trace remains unrun. |
| 2026-08-12 | **Spectral overflow fix + `get_spec_raw` (v1.2.0, branch `fix/spec_overflow`).** (1) `get_PAR()`'s `calc_spec[n] = counts × Spec_COE_n` assignment into a `uint16` **wrapped** from ~11 % of ADC full scale (F1 ×12 wraps above 5461 of 50000 counts), so a bright reading looked like a dark one; the ten words now **saturate at 65535** instead. Affects cmd 31 and the text/JSON `PAR`/`get_par` verbs — framing and 24 B length unchanged, *values* change under bright light (see the note in §2). (2) PAR is now summed from the unclamped counts in a `uint32` accumulator, so it no longer collapses when a channel pegs; verified bit-identical to the old result across 100 000 non-overflowing vectors. (3) New additive **cmd 35 `get_spec_raw`** → `0xA1` + 32 B + `0xF0`: unscaled counts plus `format`/`atime`/`astep`/`gain_low`/`gain_high`/`flags` (§6). (4) `spec_meas.h` given an include guard (it now defines a type). Ambyte side deliberately **not** updated — cmd 31 is untouched, so deployed loggers are unaffected. | A + B — A must show cmd 31 framing byte-identical; B adds the cmd 35 rows | ☐ pending HW |

**Focused check for sweep-#4 inc 1** (the send block is shared by both transports, so verify
both halves):

- **AMBYTE path** — drive a `run` (cmd 21) and an `mpf` (cmd 20) from the ambyte: the FSM
  round-trip (array count, each array length, header + byte-sum checksums) must be identical
  to the pre-change build; measurement values within sensor noise.
- **COMPUTER path** — drive `arrun` over the USB/text console: the 7 ASCII blocks
  (`Data:ENV…`, `Fluo`, `Fluoref`, `SUN`, `leaf`, `730`, `730ref`) followed by `Data sent`
  must be byte-identical to pre-change (this exercises the `send_serial` half).

Passing both confirms the extracted `pam_send_results` reproduces the inline block exactly.
The next sweep-#4 increments (`PamBuffers` RAII / `#11` leak fix, then `read_one_frame`)
each get their own row here and must re-run A + B before being trusted.
