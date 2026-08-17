# Command 35 — three-tier AS7341 calibration: spectral characteristics + PAR

Intended reader: an agent or developer implementing this in the **ambit** repo
(`Documents/Git-repo/ambit`). The analysis behind §1–§3 was done in the **miniPar** repo;
everything about ambit was read directly from its source and verified against the commit on
disk. Line references were re-checked on 2026-08-17 against `fix/spec_overflow` @ `a1ff230`.

**The architecture ports. The numbers do not.** ambit has different optics, window and
diffuser, so every coefficient vector must be re-measured on ambit hardware against an
LR1-B (goal A) and a Li-Cor Li-250A (goal B). Do not copy miniPar's fitted vectors into
ambit firmware.

**Status fact that shapes the whole plan: cmd 35 has never been released.** `git tag
--contains a1ff230` is empty, the commit is not an ancestor of `origin/main`, `git show
v1.1.4:src/run_esp.cpp` has no `case 35`, and the Ambyte's `ambit_protocol.h` defines no
opcode 35. No fielded device answers it and no shipped host asks for it. Its 32-byte payload
is therefore **free to redefine**, which is why this plan rewrites cmd 35 in place instead of
adding a cmd 36 alongside it. The only consumer that exists anywhere is one notebook in the
Calibratron repo (§9).

---

## 1. Two goals, one sensor

| | goal | output | fitted against |
|---|---|---|---|
| **A** | spectral characteristics of the light | per-channel irradiance-like values (optionally a 1 nm spectrum, host-side) | LR1-B spectrometer |
| **B** | PAR | one scalar, µmol m⁻² s⁻¹ | Li-Cor Li-250A |

They share the exposure normalisation and diverge immediately after. Goal B is **not**
computed from goal A's output — see §8.

## 2. The three tiers

```
tier 1   x       = raw / (gain × tint)        per sample, no calibration, pure arithmetic
tier 2   PAR_DEF = w · x                      fleet default, 10 floats, fitted once
tier 3   PAR     = a · PAR_DEF + b            per device, 2 floats, from an intensity sweep
```

Why this split, measured on miniPar (462 samples, 6 devices, daylight + canopy + gel
filters + office + LED panels):

| PAR model | R² | median \|err\| |
|---|---|---|
| sum of the 8 visible channels, 1 fitted slope | 0.949 | 17.2% |
| fixed physics weights from the ams matrix, 1 fitted slope | 0.990 | 6.3% |
| 10 channels, freely fitted coefficients | 0.998 | ~2% |

**Tier 2 must be fleet-level, not per-device.** An intensity sweep on one light source
moves the sample along a single ray in 10-space — it identifies a scale, not 10 weights.
Leave-one-device-out, fitting 10 coefficients from one device's own single-source
calibration rows and testing on a different source family:

| | R² | median \|err\| |
|---|---|---|
| 10 coefficients fitted per device | −31 … −7443 | 434 … 779% |
| fleet `w` + 2 per-device params | 0.958 … 0.999 | 0.5 … 3.1% |

The per-device 10-coefficient fit does not degrade gracefully; it produces arbitrary
signs and detonates on any spectrum outside its calibration set.

**Tier 3 source choice.** Both work; daylight calibrates better, LEDs are repeatable on a
bench: daylight-calibrated → 2.4% median on LEDs; LED-calibrated → 4.4% median on
daylight (worst device 9.8%).

## 3. What ambit does today

`get_PAR_raw()` — [src/src/as7341/spec_meas.cpp:207-236](src/src/as7341/spec_meas.cpp):

```cpp
const uint16_t counts = spec[SPEC_RAW_INDEX[n]];      // RAW ADC counts
if (n < 8)       weighted_sum += (uint32_t) counts * SPEC_WEIGHT[n];
else if (n == 8) nir_weighted  = (uint32_t) counts * SPEC_WEIGHT[n];
calc_par = weighted_sum * 0.006 - nir_weighted * 0.0075;
return calc_par * PAR_OFFSET;                          // PAR_OFFSET = 4
```

- **No tier 1 at all.** Nothing in the live firmware divides by gain or by integration time
  anywhere — the only occurrences of the 2.78 µs tick or a gain multiplier in `src/` are
  inside dead driver functions and one commented-out line.
- It survives that only by **pinning the exposure**: `SPEC_ATIME 99`, `SPEC_ASTEP 499`,
  `AS7341_GAIN_2X`, reprogrammed on every read by `dual_exposure()`
  ([spec_meas.cpp:133](src/src/as7341/spec_meas.cpp)). The coefficients are silently
  bound to those settings.
- **Tier 2 is integer weights**, `Spec_COE1..9` = `{12, 10, 11, 10, 10, 9, 7, 4, 1}` in
  [spec_meas.h:19-27](src/src/as7341/spec_meas.h). Normalised to F1 that is
  `1.00, 0.83, 0.92, 0.83, 0.83, 0.75, 0.58, 0.33` — much flatter than the ams matrix's
  PAR weights (`1.00, 0.66, 0.57, 0.52, 0.43, 0.36, 0.46, 0.34`), i.e. close to the
  uniform-sum regime that cost 17% median error on miniPar. F2 and F4 get identical
  weights purely from integer quantisation. The scalars `0.006`, `0.0075` and
  `PAR_OFFSET = 4` have no derivation anywhere in the source.
- **Clear is unused in PAR**; NIR is subtracted with its own scalar. No Clear-based
  stray-light term, which is where most of the miniPar accuracy gain came from.
- **Tier 3 is half-present**: `ambit_calibration_local.spec_coef` (a single scalar,
  validated 0.05–100) multiplies PAR. That is `a` with no `b`.
- No dark-offset vector, no per-channel spectral sensitivity, no storage for either.
- `Adafruit_AS7341::toBasicCounts()` exists at
  [Adafruit_AS7341.cpp:920](src/src/as7341/Adafruit_AS7341.cpp) and uses `2.78 / 1000`
  (**ms**), but it is **dead code** — no call site anywhere. It would also be unsafe here
  as written: it issues live `getGain()`/`getATIME()`/`getASTEP()` reads, so after
  `dual_exposure()` it sees the *high*-bank gain for every channel including F1–F4.

### Cmd 35 is already the right acquisition command

[run_esp.cpp:270](src/run_esp.cpp) returns 32 bytes: `format`, `atime`, `gain_low`,
`gain_high`, `astep`, `flags` (bit0 = full scale hit), `raw[10]`, `par`. Everything a host
needs for tier 1 is already on the wire. Its `par` field is still the legacy path, hence
the TODO at [run_esp.cpp:296](src/run_esp.cpp).

Two defects in what it currently promises, both fixed by this plan:

- The prescribed normalisation `raw / (gain × (atime+1) × (astep+1))` omits the tick
  entirely **and treats the gain byte as a multiplier**. It is an `as7341_gain_t`
  *ordinal* (n → 0.5·2ⁿ): ordinal 0 is 0.5×, so using it literally divides by **zero**;
  ordinals 3, 4, 10 mean 4×, 8×, 512×. Ordinal 2 = multiplier 2 is the enum's only fixed
  point, which is why the formula appears to work at the pinned exposure.
- The payload comment attributes Clear and NIR to the `gain_low` bank. They come from the
  **high** bank — see §8.

---

## 4. Decisions taken

Recorded here so the implementation does not relitigate them.

| # | decision | rationale |
|---|---|---|
| 1 | **Rewrite cmd 35; do not add cmd 36.** | 35 is unreleased and unconsumed; a second near-identical opcode would be permanent duplication bought with nothing. |
| 2 | **The five calibration vectors live *outside* `ambit_calibration_info_t`**, in their own NVS-backed struct with its own byte-explicit read-back. | That struct is blitted raw onto the wire by cmd 33/1 at its full `sizeof` ([run_esp.cpp:231](src/run_esp.cpp#L231)) and is mirrored field-for-field in the Ambyte's `ambit_protocol.h` under an explicit "MUST match ambit-1 nvs1.h — omitting it desyncs the framed read" warning. See §6 for why appending is worse than a clean break. |
| 3 | **Integration time in milliseconds**, tick `2.78e-3`. | The ams constants (dark offsets, reconstruction matrix) are published in ms. Anchor: ATIME 99 / ASTEP 499 → **139 ms**. |
| 4 | **The tier math lives only in the cmd 35 handler.** cmd 31, text `get_par`/`PAR`, and JSON `par`/`par_raw` keep the legacy path untouched. | Cmd 31 is frozen against a shipped Ambyte. Cmd 35 is the future primary spectral command and will supersede the rest once hardware-validated; until then two definitions coexist deliberately, not accidentally. |
| 5 | **The wire carries the gain *ordinal*; the firmware converts internally** via a `gain_multiplier()` helper in `spec_meas`, applied per bank. | The ordinal is compact, already documented as `as7341_gain_t 0-10`, and the host already derives a multiplier (the Calibratron CSV has a `gain_x` column). The helper goes in `spec_meas`, not the vendored Adafruit driver. |
| 6 | **Per-channel saturation and clip masks**, plus board-level status bits. | A payload that divides, offset-corrects and weights must say *which* channels are trustworthy. §5 has the layout. |
| 7 | **`spec_coef` is untouched and is not applied to the new `par`.** | Cmd 31 and deployed devices depend on it. `par_slope` is its successor, not a rename — a scalar on integer-weighted raw counts is a different quantity. |
| 8 | **Default vectors stay unfitted (TODO) until the LR1-B / Li-250A campaign.** `flags` bit2 reports it. | Guessing coefficients is worse than declaring the field uncalibrated. See §11. |

---

## 5. The rewritten cmd 35 payload

Request unchanged: standard `0xA0` + 8-byte frame with `cmd_arr[0] = 35`, remaining seven
bytes unused. Response becomes `0xA1` + **80 B** + `0xF0`.

Byte-explicit and naturally aligned (u16 on even offsets, f32 on multiples of 4), so it is
padding-free without `__attribute__((packed))` and without inheriting the ESP32-default-
alignment coupling that froze the cmd 33 structs at 140/48/248. All fields little-endian.

```
 0  u8   format = 1     bump only for a layout change
 1  u8   atime
 2  u8   gain_low       as7341_gain_t ORDINAL, F1-F4 bank
 3  u8   gain_high      as7341_gain_t ORDINAL, F5-F8 + NIR + Clear bank
 4  u16  astep
 6  u16  flags          bit0 = any channel at digital full scale   (== sat_mask  != 0)
                        bit1 = any channel clipped at its dark offset (== clip_mask != 0)
                        bit2 = PAR uncalibrated (par_weight is all zero)
                        bit3 = analog saturation reported by the AS7341 (ASAT, STATUS2)
                        bit4 = acquisition fault (I2C read failed or sensor absent)
                        bits 5-15 reserved, sent as 0
 8  u16  sat_mask       bit i = channel i reached digital full scale
10  u16  clip_mask      bit i = channel i clipped at its dark offset
12  u16  raw[10]        unscaled counts: F1..F8, NIR, Clear
32  f32  chan[10]       goal A: normalised, offset-corrected, spectrally scaled
72  f32  par            goal B: par_slope * t2 + par_intercept
76  f32  par_tier2      goal B: t2, before slope/intercept
80  end
```

**`format` stays 1.** No host has ever observed the 32-byte form, so value 1 is not burned;
bumping to 2 would permanently reserve a number for a layout that never existed.

Computation, in this order:

```
tint_ms = (atime + 1) * (astep + 1) * 2.78e-3       // MILLISECONDS — decision 3
g(i)    = gain_multiplier(bank(i))                  // ordinal n -> 0.5 * 2^n
x[i]    = raw[i] / (g(i) * tint_ms)                 // tier 1, basic counts
s[i]    = max(0, x[i] - spec_offset[i])             // dark offset, clipped -> clip_mask
A[i]    = s[i] * spec_sens[i]                       // goal A                -> chan[i]
t2      = SUM_i par_weight[i] * s[i]                // goal B tier 2         -> par_tier2
PAR     = par_slope * t2 + par_intercept            // goal B tier 3         -> par
```

`bank(i)`: slots 0–3 (F1–F4) divide by `gain_low`; slots 4–9 (F5–F8, **NIR, Clear**) divide
by `gain_high`. See §8.

- **`raw[]` stays on the wire** so a host can recompute everything and *verify* the firmware
  rather than trust it. That property only holds with the calibration read-back of §6.4.
- **Basic counts get no slot**: the host derives them exactly from `raw[]` plus the four
  exposure fields.
- **`par_tier2`** is exactly the quantity an onboarding intensity sweep regresses against the
  Li-250A, so tier 3 can be fitted without inverting the stored slope/intercept.
- **`clip_mask` matters** because the offset clip is the only nonlinearity in the chain.
- **Full scale is computed from the reported `atime`/`astep`**, not from the compile-time
  `SPEC_FULL_SCALE` macro. The macro is correct only while the exposure stays pinned, and
  reporting the exposure exists precisely so it need not stay pinned.
- **`par` no longer equals cmd 31's `par`.** That equality is currently an explicit
  conformance check at [HW_CONFORMANCE.md:120-123](plans/HW_CONFORMANCE.md) and must be
  replaced with per-tier checks (§9).

---

## 6. Calibration storage

### 6.1 The state

A new struct, deliberately **not** blitted onto any wire:

```c
/* Spectral/PAR calibration. Kept out of ambit_calibration_info_t on purpose: that
 * struct is memcpy'd onto the wire by cmd 33/1 at its full sizeof and is mirrored
 * byte-for-byte in the ambyte's ambit_protocol.h, so it cannot grow without a
 * coordinated reflash of every deployed logger. This one is read back through a
 * byte-explicit, versioned layout (§6.4) and can be extended freely. */
struct ambit_spec_calibration_t {
    float spec_offset[10];      // goal A, dark offset; basic-count units, ms convention
    float spec_sens[10];        // goal A, per-channel sensitivity; 1.0 until measured
    float par_weight[10];       // goal B tier 2; 0.0 until fitted
    float par_slope     = 1.0f; // goal B tier 3
    float par_intercept = 0.0f; // goal B tier 3
};
```

Defaults: `spec_offset` seeded from the ams workbook (§7); everything else identity/zero
until measured. Loaded by a new `load_spec_calibration()` called from
`load_info_from_nvs()`. NVS keys (namespace `config`, ≤15 chars): `spec_off`, `spec_sens`,
`par_w`, `par_slope`, `par_icept`.

### 6.2 NVS I/O — generalise, do not invent

`save_calibration_float()` at [nvs1.cpp:61-85](src/nvs1.cpp) is already a verified blob
writer: `nvs_open` → `nvs_set_blob` → `nvs_commit` → `nvs_get_blob` → size check → `memcmp`
→ `esp_err_t`. It is scalar only because the length is hardcoded. Generalise it to
`save_calibration_blob(const char *key, const void *data, size_t len)` and make the float
version a one-line wrapper, so nothing existing changes. A 40-byte blob is a single NVS
entry, so it is strictly more atomic than the existing six-key `save_adpd_baseline()`.

### 6.3 The loader guard this codebase does not have yet

`Preferences::getBytes` copies a **shorter-than-expected** stored blob and returns the short
length, leaving the tail of the destination untouched. Reading straight into the live struct
would silently blend stored elements with defaults, undetectably afterwards. So:

```c
if (preferences.isKey("spec_off")) {
    float scratch[10];
    if (preferences.getBytes("spec_off", scratch, sizeof scratch) == sizeof scratch &&
        ambit_calibration::valid_spec_offset(scratch)) {
        memcpy(spec_cal.spec_offset, scratch, sizeof scratch);
    }
}
```

Read into scratch → check the returned length → run the predicate → only then adopt. Note
this is also where the handoff's original "appending fields is migration-free" property
stops: it holds for `isKey()`-guarded scalars, but a blob whose element count ever changes
is exactly the short-read case.

### 6.4 Read-back — mandatory, not optional

Because the vectors are off cmd 33/1, nothing else puts them on the wire, and without them
the "host can verify the firmware from `raw[]`" property of §5 does not hold. Add **cmd 33
subtype 4**. This is purely additive: the existing `else` at
[run_esp.cpp:241-243](src/run_esp.cpp) answers a bare `ESP_CMD_END`, so an older image
degrades gracefully instead of going silent.

```
  0  u8   format = 1
  1  u8   reserved
  2  u16  reserved
  4  f32  spec_offset[10]
 44  f32  spec_sens[10]
 84  f32  par_weight[10]
124  f32  par_slope
128  f32  par_intercept
132  end
```

Mirror it as a text verb (`get_spec_cal`, comma-separated) so the Calibratron can confirm a
write landed without speaking binary.

### 6.5 Predicates

Add to [src/calibration_math.h](src/calibration_math.h), which requires every text, binary
and NVS-load path to share one predicate. Note the file's live exception: the adpd baselines
are validated on save and in the text setter but **not** on load
([nvs1.cpp:180-185](src/nvs1.cpp)) — do not copy that. Follow the
`valid_adpd_baseline` / `save_adpd_baseline` pattern instead: validate inside the save
helper, so one call site covers every transport, and validate again on load.

| predicate | bound |
|---|---|
| `valid_spec_offset(const float[10])` | every element finite, `[0, 1)` |
| `valid_spec_sens(const float[10])` | every element finite, `(0, 1000]` |
| `valid_par_weight(const float[10])` | every element finite, `\|e\| <= 1e4` |
| `valid_par_slope(float)` | finite, `(0, 100]` |
| `valid_par_intercept(float)` | finite, `\|b\| <= 500` |

The **element-wise `isfinite` half is the load-bearing part today** — it is what keeps a NaN
out of the tier-1 divide — and it needs no measurements. The numeric ranges are provisional
and get tightened when the campaign produces real vectors (§11). Boundary cases go in
[test/calibration/test_main.cpp](test/calibration/test_main.cpp), which CI compiles and runs
with `-Wall -Wextra -Werror` ([pr.yml:30-36](.github/workflows/pr.yml)).

### 6.6 Setters — text console, not binary 17/18

The earlier draft of this plan routed the vectors through new subtypes on binary cmds 17/18.
Use the text console instead:

- It is the dialect the Calibratron actually speaks (AGENTS.md), and the Calibratron is what
  runs the calibration.
- It already reports accept/reject explicitly (`"PAR coefficient rejected"`), and
  [set_baseline](src/do_command.h#L182-L199) is a working comma-parsed vector setter to copy.
- Binary cmds 17 and 18 write **nothing** on an unrecognised subtype — no `ESP_CMD_DONE`, no
  `ESP_CMD_END`, both replies live entirely inside `if (type == 1)` — so an old image costs
  the host a full read timeout. Worse for cmd 18: `Serial.readBytes` is inside the `if`, so
  the vector payload is never consumed and the stray bytes desync the next header scan.

Proposed verbs, following the existing `set_*` naming: `set_spec_offset`, `set_spec_sens`,
`set_par_weight` (10 comma-separated floats each), `set_par_slope`, `set_par_icept`.
The loose-parse + strict-predicate idiom already used by `set_act`/`set_spec` is fine —
`isfinite` catches `Serial_Input_Double`'s NAN-on-empty.

**If binary subtypes are added later, fix this first:** [run_esp.cpp:423](src/run_esp.cpp)
discards the `readBytes` return value, and the integrity check is a plain u16 sum compared
against the trailing word — so a truncated, zero-filled buffer is self-consistent (`0 == 0`)
and is persisted as zeros. This is live today for the 12-byte actinic vector.
`crc16_ccitt` already exists in the same file at [run_esp.cpp:39-48](src/run_esp.cpp) and is
used by cmd 26; over an all-zero payload it gives 0x84F9 (12 B) / 0x85D9 (40 B), so a zero
payload provably cannot satisfy it.

---

## 7. Concrete constants that *are* portable

The ams dark offsets are device-independent (workbook `AS7341_AD000198_3-00.xlsx`, sheet
`used Correction Values`, row 15), in basic-count units under the **ms** convention.
Reordered into ambit's channel order (F1..F8, **NIR, Clear**):

```c
// basic-count dark offsets, ams convention (tint in ms)
static const float kDefaultSpecOffset[10] = {
    0.00196979f,  // F1  410
    0.00724927f,  // F2  440
    0.00319381f,  // F3  470
    0.001314659f, // F4  510
    0.001468153f, // F5  550
    0.001858105f, // F6  583
    0.001762778f, // F7  620
    0.00521704f,  // F8  670
    0.001f,       // NIR 900
    0.003f,       // Clear 750
};
```

Everything else — `spec_sens`, `par_weight`, `par_slope`, `par_intercept` — must be
measured on ambit hardware (§11).

## 8. Footguns

**Channel order differs from miniPar.** `SPEC_RAW_INDEX = {0,1,2,3,6,7,8,9,11,10}`
([spec_meas.cpp:193](src/src/as7341/spec_meas.cpp)) gives slot 8 = **NIR**, slot 9 =
**Clear**. miniPar's host order ends `..., clear, nir`. Any vector taken from miniPar
tooling needs its last two elements swapped.

**Gain is per bank, and three comments in this repo state it backwards.** Both SMUX
configurations route Clear to ADC4 and NIR to ADC5, so the buffer holds two copies of each:
`spec[4]`/`spec[5]` from the low read and `spec[10]`/`spec[11]` from the high read.
`SPEC_RAW_INDEX` reports slots 8 and 9 from `spec[11]` and `spec[10]` — the **high** read,
taken after `setGain(gain2)` at [spec_meas.cpp:151](src/src/as7341/spec_meas.cpp). So
**F5–F8 *and* NIR *and* Clear divide by `gain_high`.** These three say otherwise and must be
corrected (they echo the SMUX function's *name*, `setup_F1F4_Clear_NIR`, not the data
reported):

- [run_esp.cpp:285-286](src/run_esp.cpp) — cmd 35 payload comment
- [spec_meas.h:50-51](src/src/as7341/spec_meas.h) — `spec_raw_t` field comments
- [HW_CONFORMANCE.md:181-182](plans/HW_CONFORMANCE.md) — §6 reference table

Leave [spec_meas.cpp:189-192](src/src/as7341/spec_meas.cpp) alone — it describes the
*buffer*, where both halves genuinely do contain Clear and NIR, and it is accurate.

Both gains are `AS7341_GAIN_2X` today, so getting this wrong is latent rather than visible —
the payload carries both fields precisely because they may diverge.

**The gain byte is an ordinal, not a multiplier.** `as7341_gain_t` n → 0.5·2ⁿ. Ordinal 0 is
0.5×, so dividing by the raw byte is a division by zero; ordinals 3/4/10 are 4×/8×/512×.
Ordinal 2 = 2× is the enum's only fixed point, which is exactly the pinned value — so a bug
here is invisible in every test at the current exposure.

**Use the ms tick (`2.78e-3`), not seconds.** Three conventions are in circulation:
AN000633 / the workbook / ambit's own `toBasicCounts()` use **ms**; miniPar's *firmware*
`spectrometerGetBasicCountDivisor()` uses **seconds**; cmd 35's old comment prescribed no
tick at all. Decision 3 picks ms, because the published ams constants are expressed in it —
otherwise every vendor constant needs a ×1000 or ÷1000 fixup, in opposite directions for
offsets and weights. Sanity check: ATIME 99 / ASTEP 499 must give `tint = 139 ms`.

**Compute PAR from `s`, never from `A`.** If PAR were `par_weight · A`, then `spec_sens`
would be baked into PAR, and any future improvement to the goal-A spectral calibration would
silently shift PAR on every deployed device with no PAR measurement having changed. Keeping
them separate is the whole reason tier 2 is fitted directly against the Li-250A.
(Mathematically the two are equivalent for *prediction* — a diagonal rescale followed by a
free fit spans the same space — so this is a coupling and maintenance requirement, not an
accuracy one.)

---

## 9. What else must change

**Acquisition** ([spec_meas.cpp](src/src/as7341/spec_meas.cpp)):

- Add `gain_multiplier(as7341_gain_t)` in `spec_meas` — **not** in the vendored
  `Adafruit_AS7341` (AGENTS.md: do not reformat vendored driver libs). The only existing
  ordinal→multiplier switch is inside the dead `toBasicCounts()`.
- Per-channel saturation: replace the single `out->saturated` bool with a 10-bit mask, and
  compare against the reported `(atime+1)*(astep+1)` rather than the compile-time constant.
- Read the AS7341's own ASAT bits from STATUS2 for `flags` bit3. Today the only STATUS2
  access is the AVALID bit in `getIsDataReady()`, so analog saturation below 50000 digital
  counts is invisible.
- Give `dual_exposure()` a return value for `flags` bit4. It is currently `void` and
  discards both I²C results — `low_success` is assigned at
  [spec_meas.cpp:149](src/src/as7341/spec_meas.cpp) then overwritten at :156 and never
  tested — and `check_AS7341()` failure does not abort. A dead I²C bus is currently
  indistinguishable from a dark reading, and the tier-1 divide amplifies whatever comes back.
- Bound `delayForData(0)`, which waits forever. This was previously filed as an unrelated
  out-of-scope observation; it is in scope now that cmd 35 is the primary spectral command.

**[plans/HW_CONFORMANCE.md](plans/HW_CONFORMANCE.md)** — AGENTS.md makes it the regression
gate for any change to `run_esp.cpp`, so it must be rewritten *and re-run*:

- `:70` Test A framing row: 32 B → 80 B.
- `:111-128` Test B checklist: the `format == 1` / `32 B` / `raw[i] <= 50000` assertions,
  plus **`cmd 35's par equals cmd 31's par`, which stops being true** — replace with
  per-tier checks (`par_tier2` reproducible from `raw[]` + the read-back vectors) and
  `sat_mask`/`clip_mask` coverage.
- `:167-199` the whole §6 reference layout, including the wrong `gain_low` bank description
  at `:181` and the `>= 1.2.0` gating text at `:196-199`.
- A new §7 log row. Note the existing v1.2.0 row is still `☐ pending HW` — **cmd 35 has
  never been hardware-verified at all**, so this is a first verification, not a
  re-verification, and there is no passing baseline to preserve. The load-bearing part of
  Test A here is that cmd 31 stays byte-identical.

**`Calibratron/multi_ambit_lr1b_capture.ipynb`** (different repo) — the only consumer of
cmd 35 that exists, and the acceptance harness for this feature, so the firmware change is
untestable until it is updated: `RESP_SPEC_RAW_SIZE` 32 → 80, new `<10f`@32 `chan[]`,
`<f`@72 `par`, `<f`@76 `par_tier2`, and `sat_mask`/`clip_mask` in its saturation reporting.
It is also the harness that produced [data/multi_ambit_spec_lr1b.csv](data/multi_ambit_spec_lr1b.csv).

**Version gating.** Both the cmd 35 comment and HW_CONFORMANCE tell hosts to gate on
cmd 33/2 reporting `major.minor >= 1.2`, but no 1.2.0 exists and a build of this branch
reports **1.1.4** — `tools/version.py` keeps the leading `X.Y.Z` of `git describe`, so every
dev build reports the tag it descends from. The real consumer already abandoned version
gating for exactly this reason and probes the opcode instead. Restate the gate against
whatever version actually ships. Note also that `release.yml` is hard-pinned to
`EXPECTED_VERSION: 1.1.4` with four `vars.AMBIT_V114_*` repository variables asserted by
`test`, so cutting 1.2.0 needs a reviewed release-process change.

**[plans/MERGE_PLAN.md](plans/MERGE_PLAN.md)** — its frozen-element table has no row for
cmd 35 and its command-ID list at `:146` is already stale against `ambit_protocol.h`
(missing 22/23/24 and the OTA opcodes 25–29). AGENTS.md directs readers there before
structural changes.

---

## 10. Non-goals

- **Do not put the 1 nm reconstruction matrix in firmware.** It is 721 × 10 floats
  (~29 kB flash) and a full spectrum is 2.9 kB per reading. The 10 corrected channels in
  `chan[]` are sufficient; the reconstruction is a fixed linear map the host applies.
- **Do not change cmd 31 or `spec_coef`.** Deployed devices cannot negotiate. Cmd 31 is in
  the shipped Ambyte's `ambit_protocol.h`; the rewrite must leave it byte-identical.
- **Do not change the pinned exposure in this work.** Cmd 35 dividing by the *reported*
  exposure is what makes auto-ranging possible later; actually adding auto-ranging is
  separate work. Consequence: `gain_low` and `gain_high` cannot differ today
  (`dual_exposure(AS7341_GAIN_2X, AS7341_GAIN_2X, ...)`, and `dual_exposure` has no
  prototype in the header), so the per-bank divisor cannot be exercised with unequal gains
  on hardware. Get the mapping right by reading §8, not by testing.
- **Do not extend `ambit_calibration_info_t`.** See decision 2 and §6. The struct is not
  sacred — both repos are in development and a coordinated reflash is available — but an
  un-updated Ambyte handed a longer response reads its 140 bytes correctly and then *scans*
  for the `0xF0` terminator (`uart_read_exact` then `uart_scan_byte` in `uart_sensors.c`),
  so it recovers **unless one of the extra bytes happens to equal 0xF0**. For arbitrary
  fitted floats that is ~39% per 128-byte block, and the *defaults* (`0.0f` = `00 00 00 00`,
  `1.0f` = `00 00 80 3F`) contain none — so a bench test passes and the corruption appears
  later, in the field, only on calibrated devices. If the struct is ever changed, the change
  worth making is not appending: it is replacing the raw `memcpy` with a byte-explicit
  versioned layout so that every subsequent extension is free. That is a separate decision,
  on its own merits, and it does not block this work.

## 11. Open TODOs — the measurement campaign

Everything below is deliberately left unfitted; `flags` bit2 tells hosts so.

| quantity | needs | status |
|---|---|---|
| `spec_offset[10]` | none — ams constants (§7) | ready to seed |
| `spec_sens[10]` | LR1-B spectra across multiple sources | 1 session only |
| `par_weight[10]` | Li-250A reference across **many source families** | no data |
| `par_slope`, `par_intercept` | per-device Li-250A intensity sweep | no data |

What exists today: [data/multi_ambit_spec_lr1b.csv](data/multi_ambit_spec_lr1b.csv) — 3 rows,
3 devices, one light condition (`office`), one timestamp, one LR1-B trace. There is no
Li-Cor reference data anywhere in the repo, so tiers 2 and 3 have zero fitting basis.

Two constraints on the campaign, both learned the hard way on miniPar:

- **Tier 2 must be fitted fleet-wide, from many source families.** §2's leave-one-device-out
  table shows what a per-device 10-coefficient fit from a single source does.
- **Expect collinearity.** miniPar's plain-OLS fit has unphysical signs (negative on F3, F5,
  F8; positive on Clear) because the design matrix is collinear (condition number ≈ 451) and
  the dataset is daylight-dominated. A constrained fit (visible channels ≥ 0, Clear/NIR free)
  or shrinkage toward the ams matrix's PAR weights is the recommended fix. Plan for enough
  narrowband LED and gel-filter measurements to break it.

Once real vectors exist, tighten the §6.5 bounds and decide whether `par_weight` should be
seeded fleet-wide in firmware rather than written per device.

## 12. Timing budget — do not micro-optimise the payload

Serial is 115200 8N1 ([src/ambit-1.ino:74](src/ambit-1.ino)) → 86.8 µs per byte.

| component | time |
|---|---|
| 2 × integration, ATIME 99 / ASTEP 499 | **278 ms** |
| SMUX reconfig + 2 × 12-byte I²C reads | ~1–5 ms |
| tier 1 + goal A + goal B math (~50 float ops, 240 MHz FPU) | **< 10 µs** |
| 80-byte payload | 6.9 ms |

Integration is ~98% of the measurement and is inherent to the AS7341's SMUX (the part
cannot present all 10 channels in one integration). The added computation is free, and the
payload growth from 32 to 80 bytes costs ~1.5% (48 extra bytes = 4.2 ms on a ~284 ms
measurement). Include whatever fields are useful.

## 13. Source material in the miniPar repo

| path | contents |
|---|---|
| `CALIBRATION.md` | full write-up of goals A/B, the three tiers, and the evidence |
| `Scripts/regression_PAR_miniPAR.ipynb` | PAR model comparison; fits and exports tier-2 `w` |
| `Scripts/spectralCalibration_miniPAR_LR1-B.ipynb` | goal A, derives per-channel sensitivity vs LR1-B |
| `Scripts/as7341_calibrate.py` | host implementation of tier 1 + goal A |
| `Firmware/docs/AS7341_AD000198_3-00.xlsx` | ams workbook: dark offsets, 721×10 reconstruction matrix |
| `Firmware/docs/AS7341_AN000633_2-00.pdf` | ams app note: §2.1, §2.2, §2.4 |
| `Firmware/src/app/spectrometer_api.cpp:578-584` | miniPar firmware, all three tiers in 3 lines |
