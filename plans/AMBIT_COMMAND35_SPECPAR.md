# Command 35 — three-tier AS7341 calibration: spectral characteristics + PAR

Intended reader: an agent or developer implementing this in the **ambit** repo
(`Documents/Git-repo/ambit`). The analysis behind §1–§3 was done in the **miniPar** repo;
everything about ambit was read directly from its source and verified against the commit on
disk. Line references were re-checked on 2026-08-17 against `fix/spec_overflow` @ `a1ff230`.

**The architecture ports. The numbers are seeds, not calibration.** ambit has different
optics, window and diffuser, so every coefficient vector must eventually be re-measured on
ambit hardware against an LR1-B (goal A) and a Li-Cor Li-250A (goal B). Earlier revisions of
this plan therefore shipped `spec_sens` at 1.0 and `par_weight` at 0.0 — but an all-zero
weight vector makes the computed PAR *identically zero*, which is a worse starting point than
a vector fitted on a different unit of the same sensor. So miniPar's fitted vectors are now
carried in as **defaults** (§7), under conditions spelled out there: they encode miniPar's
optical stack, they are provisional, and they must not be mistaken for an ambit calibration.
§11 tracks what replaces them, and the `flags` calibration byte (§5a) exists to keep the
distinction visible on the wire.

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
| **B** | PAR | one scalar, µmol m⁻² s⁻¹ | tier 2: Li-Cor Li-250A. tier 3: a Li-250A **or** a transfer standard — §1a |

They share the exposure normalisation and diverge immediately after. Goal B is **not**
computed from goal A's output — see §8.

### 1a. Tier 3 does not require a Li-250A. Tier 2 does.

The two tiers have different reference requirements, and conflating them has been costing
this plan clarity.

**Tier 2 needs an absolute PAR reference across many source families.** It is where the
µmol scale enters and where spectral weighting is decided, so nothing but a proper quantum
sensor will do. That stays a Li-250A, and it stays a TODO (§11).

**Tier 3 does not.** It is two per-device parameters fitted at one spectrum, and what it
actually measures is the ratio of this device's optical throughput to whatever the tier-2
vector was fitted on. A **transfer standard** serves that: any instrument that computes PAR
with *the same* `par_weight`. A miniPAR is exactly that today, because §7c's seed *is*
miniPar's fleet vector — so at one spectrum both devices compute `w · s` over identical
weights, the spectral bias is common-mode, and it divides out of the ratio. The fit's
*linearity* is unaffected, which is what makes the host's quality gates still mean something.

The cost is a pure multiplicative bias equal to the reference's own PAR error at the
calibration spectrum: ~2 % in-domain (§7c) but **4.4 % median / 9.8 % worst** by §2's
cross-source figures, which are the honest ones for a halogen bench lamp that is nothing
like miniPar's daylight-dominated set. Against the ~5 % absolute accuracy of a Li-250A that
is ~7 % combined rather than ~5 %.

It is also **recoverable without repeating any bench work**, provided the host stores
`par_tier2` per sweep point: one later Li-250A comparison on a single device yields a scale
correction applicable to every stored sweep. That storage requirement is on the host, and the
Calibratron does it.

So: **tier 3 against a Li-250A is an optional TODO, not a precondition.** A transfer-standard
fit is a conforming tier-3 calibration as long as the record names the reference and carries
the sweep. Two caveats to record with it: the fitted `par_slope` is specific to the
calibration lamp's spectrum, and a halogen lamp shifts colour temperature across an intensity
sweep, so the record should say how far the spectrum moved.

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
| fleet `w` + 2 per-device params | 0.996 … 0.998 | 2.5 … 9.8% |

The per-device 10-coefficient fit does not degrade gracefully; it produces arbitrary
signs and detonates on any spectrum outside its calibration set.

The second row was **re-measured on 2026-08-17** while porting the vectors (§7c), over all
six devices in `miniPar/data/*.csv`: fleet `w` from the other devices, tier 3 from the
held-out device's LED sweep only, scored on daylight rows it never saw. Median across
devices **4.37 %**, worst device 9.82 %. Earlier revisions of this table quoted 0.5–3.1 %
from a narrower run; the wider figure is the one to plan against.

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
| 8 | **`spec_sens` and `par_weight` ship seeded from the miniPar campaign; `par_slope`/`par_intercept` stay identity.** (Revised 2026-08-17 — these were originally 1.0 / 0.0.) | An all-zero weight vector makes PAR identically zero, which is less useful than a fit from another unit of the same sensor. Tier 3 is per-optics and is not seeded. The seeds are miniPar's optical stack and are not an ambit calibration — §7. |
| 9 | **`flags` is split into a negative-polarity condition byte and a positive-polarity calibration byte.** Provisional PAR is reported by *two* bits — bit8 `par_weight` is an ambit fleet fit, bit9 tier 3 stored for this device — not by one. | Decision 8 makes the old `par_weight_is_unset()` test permanently false, which would silently retire the only signal distinguishing a seeded device from a calibrated one. One bit could not express it either: the two halves of the PAR chain are calibrated by different people at different times. Positive polarity makes an all-zero word read as "nothing confirmed". §5a. |

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
 6  u16  flags          TWO ZONES WITH OPPOSITE POLARITY, one per byte — see §5a.
                        low byte = conditions, 1 means attention:
                          bit0 = any channel at digital full scale   (== sat_mask  != 0)
                          bit1 = any channel clipped at its dark offset (== clip_mask != 0)
                          bit2 = analog saturation reported by the AS7341 (ASAT, STATUS2)
                          bit3 = acquisition fault (I2C read failed or sensor absent)
                          bits 4-7 reserved, sent as 0
                        high byte = calibration, 1 means confirmed:
                          bit8 = par_weight is an ambit fleet fit (0 = borrowed seed)
                          bit9 = tier-3 slope/intercept stored for this device
                          bits 10-15 reserved, sent as 0
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

### 5a. Why `flags` has two zones with opposite polarity

*Implemented.* As first written, bit2 was `par_weight_is_unset()` — literally "the tier-2
vector is all zero". That was a sound signal while the vector shipped as zeros, because an
all-zero weight collapsed PAR to `par_intercept` and the flag was the only thing separating
"uncalibrated" from "dark". Seeding `par_weight` (§7c) makes that predicate permanently
false, so left alone it would retire the only warning a host ever gets — and a fresh device
would report a plausible PAR computed on another sensor's optics with nothing saying so.

The replacement inverts the sense: **1 means confirmed calibrated**, and it lives in the high
byte, away from the condition bits.

**Why inverted.** The payload is zero-initialised and reserved bits go out as 0, so a
"provisional = 1" bit reads as *calibrated* in every degenerate case — a truncated frame, a
misparsed offset, a field the firmware failed to set. Doubt should resolve to "not
calibrated": a spurious warning costs a question, while publishing seed-derived PAR as
science data costs a dataset.

**Why zoned rather than mixed.** Bits 0–3 all mean "something is wrong". A single inverted
bit dropped among them is how a host ends up writing `if (flags) warn` and treating
*calibrated* as a fault. Splitting by byte makes the two polarities structural instead of a
comment someone has to notice, and both halves then fail safe in the same direction: an
all-zero `flags` word means "no fault reported, nothing confirmed calibrated".

**Why two bits rather than one.** The two facts are independent and have different lifetimes.
`bit8` is a build property — is the compiled-in tier-2 vector an ambit fleet fit or the
miniPar seed — and is the same on every device running an image. `bit9` is per device: has
this unit been through a tier-3 sweep. Onboarding cares about `bit9` alone; a data consumer
wants both. OR-ing them into one bit would have thrown that away.

```c
// nvs1.h — flip in the commit that lands an ambit-fitted fleet vector (§11).
static constexpr bool AMBIT_PAR_WEIGHT_IS_AMBIT_FIT = false;
extern bool ambit_spec_tier3_stored;   // set from NVS key presence, and by the setters

flags |= (AMBIT_PAR_WEIGHT_IS_AMBIT_FIT &&
          !par_weight_is_unset(...)) << 8;
flags |= ambit_spec_tier3_stored       << 9;
```

`bit9` is keyed on **NVS key presence**, not on comparing floats to the defaults: a fitted
slope of exactly 1.0 is legitimate, and float equality against a constant is a poor test.
Either tier-3 key counts — a sweep through the origin has no intercept to store. `bit8`
additionally requires the live vector not be all-zero, so a build that claims an ambit fit
cannot vouch for a zero vector a host wrote over the setter; that is what keeps
`par_weight_is_unset()` load-bearing.

Today both high bits are **0** on every device: the seed is not an ambit fit, and no unit has
been swept. That is the honest state, and §9's conformance expectation matches it.

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
    float spec_sens[10];        // goal A, per-channel sensitivity; miniPar seed
    float par_weight[10];       // goal B tier 2; miniPar fleet seed
    float par_slope     = 1.0f; // goal B tier 3, per device
    float par_intercept = 0.0f; // goal B tier 3, per device
};
```

Defaults: `spec_offset` from the ams workbook, `spec_sens` and `par_weight` from the miniPar
campaign — all three in §7. `par_slope`/`par_intercept` stay identity because they are
per-device by definition and nothing about miniPar's units transfers. Loaded by a new
`load_spec_calibration()` called from `load_info_from_nvs()`. NVS keys (namespace `config`,
≤15 chars): `spec_off`, `spec_sens`, `par_w`, `par_slope`, `par_icept`.

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

Mirror it as a text verb (`get_spec_cal`) so the Calibratron can confirm a write landed
without speaking binary. *Implemented* in [src/do_command.h](src/do_command.h) as **five
labelled lines**, `<field>:<comma-separated %.9g>`, in the same order as the binary layout:

```
spec_offset:0.00196979,0.00724927,0.00319381,...      10 values
spec_sens:34.9506626,65.2894836,72.6979971,...        10 values
par_weight:333.463542,206.427139,-30.6130743,...      10 values
par_slope:1
par_intercept:0
```

Labelled lines rather than one flat row of 32 floats, for a specific reason: a host parsing
by *position* silently shifts every subsequent value by one when a field is added, removed or
interleaved with a log line — and for these vectors a one-slot shift is undetectable in the
numbers, because every element is a plausible magnitude for its neighbour. Keying on the label
makes that failure impossible instead of merely unlikely. `%.9g` is chosen to round-trip an
IEEE-754 `float` exactly, so the text mirror and cmd 33/4 agree bit for bit and a host may use
either as the read-back of record.

Hosts should read a few more lines than the five and match on labels, since the console can
interleave `ESP_LOG` output at any point.

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

Verbs, following the existing `set_*` naming: `set_spec_offset`, `set_spec_sens`,
`set_par_weight` (10 comma-separated floats each), `set_par_slope`, `set_par_icept`.
The loose-parse + strict-predicate idiom already used by `set_act`/`set_spec` is fine —
`isfinite` catches `Serial_Input_Double`'s NAN-on-empty.

**Reply contract — three outcomes, and hosts must test for the good one.** As implemented
(`report_spec_save` in [src/do_command.h](src/do_command.h)) every setter answers with exactly
one line:

| reply | meaning | state |
|---|---|---|
| `<what> saved and verified` | committed to NVS and read back | **written** |
| `<what> rejected` | failed the predicate | NVS untouched |
| `<what> save failed: <ESP_ERR_…>` | predicate passed, the NVS write did not | NVS untouched |

`<what>` is `Spectral offset`, `Spectral sensitivity`, `PAR weight`, `PAR slope`,
`PAR intercept`.

The third row is the trap, and it is worth stating rather than leaving to be discovered: it
contains no negative keyword at all. A host that tests for `"rejected"` — the obvious reading
of "it already reports accept/reject explicitly" above — treats an NVS failure as a success
and walks away believing it wrote a coefficient it did not write. **So acceptance must be
tested positively, on `saved and verified`.** That single test covers all three outcomes and
any future rewording, because only the confirmation passes it. Recommended for anything
matching these replies, host or firmware test.

Two smaller notes for hosts:

- Validate the value you are about to *transmit*, not the one you fitted. The wire is
  fixed-point, so a value that satisfies §6.5 at full precision can violate it after
  rounding — a `par_slope` below the format's resolution prints as `0.000000`, which
  `valid_par_slope`'s `(0, 100]` correctly refuses. `%.6f` is ample for a slope near 1.0.
- These are five independent NVS commits, with no cross-key atomicity. Writing `par_slope`
  then `par_icept` can therefore be interrupted between them, leaving `a` new and `b` old.
  That is bounded and self-consistent rather than torn, but a host should read both back
  together after writing both, and be able to restore the pair it replaced.

**If binary subtypes are added later, fix this first:** [run_esp.cpp:423](src/run_esp.cpp)
discards the `readBytes` return value, and the integrity check is a plain u16 sum compared
against the trailing word — so a truncated, zero-filled buffer is self-consistent (`0 == 0`)
and is persisted as zeros. This is live today for the 12-byte actinic vector.
`crc16_ccitt` already exists in the same file at [run_esp.cpp:39-48](src/run_esp.cpp) and is
used by cmd 26; over an all-zero payload it gives 0x84F9 (12 B) / 0x85D9 (40 B), so a zero
payload provably cannot satisfy it.

---

## 7. The shipped default vectors

Three of the five come from measurement; two stay identity. **All are listed in ambit's
channel order — F1..F8, NIR, Clear — which is *not* miniPar's order. See §8.**

### 7a. `spec_offset` — ams, genuinely device-independent

Workbook `AS7341_AD000198_3-00.xlsx`, sheet `used Correction Values`, row 15, in
basic-count units under the **ms** convention. This is a property of the silicon, so it
ports without qualification.

```c
// basic-count dark offsets, ams convention (tint in ms)
static const float kDefaultSpecOffset[10] = {
    0.00196979f,  // F1  415
    0.00724927f,  // F2  445
    0.00319381f,  // F3  480
    0.001314659f, // F4  515
    0.001468153f, // F5  555
    0.001858105f, // F6  590
    0.001762778f, // F7  630
    0.00521704f,  // F8  680
    0.001f,       // NIR 910
    0.003f,       // Clear (broadband)
};
```

**Channel centres corrected 2026-08-17.** Earlier revisions of this plan labelled these
410/440/470/510/550/583/620/670 and "Clear 750". Those numbers are not the AS7341's: the
datasheet centres are 415/445/480/515/555/590/630/680 with NIR ≈ 910 and Clear broadband,
as ambit's own vendored driver enum states
([Adafruit_AS7341.h:195-206](src/src/as7341/Adafruit_AS7341.h)) and as miniPar's column
names (`f1_415` … `f8_680`) use. The values were always right; only the comments were wrong.
[src/nvs1.h](src/nvs1.h) still carries the old labels and should be corrected with the
seeding change (§9).

### 7b. `spec_sens` — miniPar §2.4, ports cleanly

From `Scripts/spectralCalibration_miniPAR_LR1-B.ipynb`: the median per-channel ratio
`reference / sensor`, where `reference` is the LR1-B irradiance projected onto the channels
by least-squares inversion of the ams reconstruction matrix, and `sensor` is basic counts
with the ams offsets subtracted. Averaged over 3 devices; per-channel device spread is
**1.4 % – 9.7 %** (Clear worst), which is what justifies one fleet vector instead of a
per-serial lookup.

```c
// miniPar LR1-B campaign, 3 devices, mean of the per-device factors
static const float kDefaultSpecSens[10] = {
    34.950663f,   // F1  415
    65.289484f,   // F2  445
    72.697997f,   // F3  480
    63.273264f,   // F4  515
    56.737110f,   // F5  555
    52.958660f,   // F6  590
    48.706781f,   // F7  630
    42.671670f,   // F8  680
     5.781618f,   // NIR 910      <- miniPar index 9
    31.565986f,   // Clear        <- miniPar index 8
};
```

This one ports **exactly**, because miniPar derived it under the same chain ambit computes:
basic counts with the ms tick, ams offsets subtracted, then scale. `s[i] × spec_sens[i]` in
ambit is the same expression that produced the constant.

### 7c. `par_weight` — miniPar tier 2, ports with one caveat

From `Scripts/regression_PAR_miniPAR.ipynb`. The notebook's export cell refits on **all**
samples and discards the tier-2 intercept; that cell had never been run, so this vector was
regenerated by re-executing its exact code over `miniPar/data/*.csv` (462 samples, 6 devices,
PAR 3.7 – 2033.7 µmol m⁻² s⁻¹). The notebook's stored 80/20 split reproduces bit-for-bit
(R² 0.9988, `f1 = 381.049293`, intercept `5.3691892`), which is the check that the
replication is faithful.

```c
// miniPar Li-250A campaign, OLS on basic counts, all 462 samples,
// tier-2 intercept discarded (tier 3 absorbs it)
static const float kDefaultParWeight[10] = {
     333.463542f,  // F1  415
     206.427134f,  // F2  445
     -30.6130744f, // F3  480
     283.778061f,  // F4  515
    -144.07319f,   // F5  555
      73.852848f,  // F6  590
      38.9285016f, // F7  630
      -6.43585093f,// F8  680
     -37.6580353f, // NIR 910     <- miniPar index 9
      16.9097651f, // Clear       <- miniPar index 8
};
```

**The caveat.** miniPar fitted `w` against `basic_counts()`, which is §2.1 only — it does
**not** subtract the dark offset. ambit applies `s = max(0, x − spec_offset)`. The difference
is `Σ wᵢ·offsetᵢ` = **2.40 µmol m⁻² s⁻¹**, a *constant*, so tier 3's intercept absorbs it
exactly. Verified by running ambit's chain over miniPar's data: R² 0.9983, median |err|
1.96 %, and a subsequent tier-3 fit returns `a = 1.0000, b = 7.983` — i.e. the whole
discrepancy (that 2.40 plus miniPar's discarded 5.61 tier-2 intercept) reappears as a pure
intercept, exactly as the tier composition predicts. Nothing needs rescaling.

**But the constant is only constant while nothing clips — so a clipped point must never be
fitted.** The argument above is what licenses reusing this vector at all, and it rests on
`Σ wᵢ·offsetᵢ` being a fixed offset. Once any channel hits the `max(0, …)` floor, that term
stops depending on the light and the relation stops being affine. Since the clip is the
chain's only nonlinearity (§5), a clipped reading is off-model by construction.

The consequence is sharp and easy to walk into, because the natural place to put a low anchor
in a tier-3 sweep is with the lamp off — and that is exactly where every channel clips. The
ams offsets are **0.28 – 2.02 raw counts** at the pinned 2× / 139 ms (F2 the largest), so the
clip bites only in a light-tight fixture, but there it bites on all ten. A fully clipped
reading has `par_tier2` identically 0 against a reference of ~0, so it sits on the line only
if `par_intercept` is 0. Folding one into the fit drags the intercept from 7.983 to **4.78 —
40 % low — while R² stays above 0.99, so no ordinary fit-quality gate catches it.**

So a tier-3 sweep must exclude any point whose `clip_mask` is non-zero, at *any* lamp drive,
and must include at least one genuinely lit low point to constrain the intercept. Keying the
rule on `clip_mask` rather than on the drive setting handles both cases automatically: a dark
reading that comes back unclipped (ambient light present) is perfectly usable. This is also
the second reason `clip_mask` is on the wire per-channel rather than as one bit — the first
being §5's.

**Read the signs before trusting this vector.** The notebook's own sanity check expects
F1–F8 to lift PAR and Clear/NIR to subtract stray light. This fit violates that on four
channels — F3, F5 and F8 are negative, and Clear is positive:

```
w normalised to f1:   f1  1.000   f2  0.619   f3 -0.092 ←   f4  0.851
                      f5 -0.432 ← f6  0.221   f7  0.117   f8 -0.019 ←
                      clear 0.051 ←           nir -0.113
```

That is collinearity, not physics: the design matrix has condition number ≈ 451 and the
dataset is daylight-dominated, so OLS distributes weight almost arbitrarily among correlated
visible channels. The instability is visible directly — refitting on 80 % of the samples
moves `f1` from 333 to 381, `f3` from −30.6 to −9.2 and `f8` from −6.4 to −12.6, while the
*prediction* barely changes (R² 0.9983 vs 0.9988). The vector predicts well in-domain and
should not be read as a spectral response curve, nor extrapolated to spectra unlike the
calibration set. Fixing it needs a constrained fit (visible ≥ 0, Clear/NIR free) or shrinkage
toward the ams PAR weights — §11.

### 7d. `par_slope` = 1.0, `par_intercept` = 0.0 — deliberately not seeded

Tier 3 is per device and per optical stack. Seeding it from miniPar would import that
sensor's window and diffuser as if they were ambit's.

Be precise about what this leaves: a fresh ambit reports `par = par_tier2` unscaled. It
tracks light correctly and has roughly the right spectral weighting, but its **magnitude is
not meaningful** — `par_slope` is exactly the term that carries optical throughput, and
ambit's aperture and diffuser are not miniPar's, so the scale can be off by a large factor
in either direction. Do not read the number as µmol m⁻² s⁻¹ before a tier-3 sweep. `flags`
bit9 is what says so on the wire.

### 7e. What was checked before porting

| | miniPar | ambit | action |
|---|---|---|---|
| tick | `ASTEP_TICK_MS = 2.78e-3` | `SPEC_TICK_MS = 2.78e-3` | none — both ms |
| gain map | `0.5 if reg==0 else 1<<(reg-1)` | `0.5 × 2ⁿ` | none — identical for all 11 ordinals |
| channel order | `..., clear, nir` | `..., NIR, Clear` | **last two swapped** |
| offset in the fit | not subtracted (`basic_counts`) | subtracted (`s`) | none — constant, absorbed by tier 3 |
| magnitudes | — | `valid_spec_sens` ≤ 1000, `valid_par_weight` ≤ 1e4 | both pass (max 72.7 and 333.5) |

One thing to be aware of when reading the miniPar source: `Scripts/as7341_calibrate.py`
currently has `OFFSET_BASIC` **zeroed out**, with the real ams vector commented out just
above it. So `calibrated_raw()` in that module does not subtract the offset its
`SPECTRAL_COEF` was derived with. That is a miniPar-side inconsistency, worth a look there;
it does not affect anything ported here, because `spec_sens` was taken from the notebook
(which loads the offsets straight from the workbook) and `par_weight` never touches
`OFFSET_BASIC` at all.

## 8. Footguns

**Channel order differs from miniPar.** `SPEC_RAW_INDEX = {0,1,2,3,6,7,8,9,11,10}`
([spec_meas.cpp:193](src/src/as7341/spec_meas.cpp)) gives slot 8 = **NIR**, slot 9 =
**Clear**. miniPar's `CHANNELS` (both notebooks and `as7341_calibrate.py`) ends
`..., 'clear', 'nir'`. Any vector taken from miniPar tooling needs its last two elements
swapped — the §7 listings are **already swapped**; the miniPar sources are not.

This one is unusually easy to get away with and unusually expensive when you don't: NIR and
Clear have the most dissimilar coefficients in every vector (`spec_sens` 5.78 vs 31.57,
`par_weight` −37.7 vs +16.9), so a missed swap does not fail loudly — it just produces a
confidently wrong PAR under any spectrum with appreciable NIR content.

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

**Firmware work from the seeding decision (§7) — complete.** Landed in this order, which is
the safe one: the flags change first (on its own it just reports the truth about a
still-unseeded build, both high bits 0), then the vectors. Reversing it would have left a
window where a device asserted a confident PAR with no provisional signal.

- ~~Redefine `flags` bit2~~ — done; became the two-zone field of §5a.
- ~~Seed `spec_sens` and `par_weight`~~ — done, in `ambit_spec_calibration_t`
  ([src/nvs1.h](src/nvs1.h)), in ambit channel order with the provenance, the sign caveat and
  the offset-convention note recorded at the definition. No predicate changes were needed:
  the largest magnitudes are 72.7 against `valid_spec_sens`'s 1000 and 333.5 against
  `valid_par_weight`'s 1e4.
- ~~Fix the channel-centre comments~~ — done; they now read 415/445/480/515/555/590/630/680,
  NIR 910, Clear broadband (§7a).

Verified against miniPar's 462 samples after the port: the ambit-order vectors applied to
ambit-order columns reproduce the miniPar-order computation to 4.4e-16, and the chain scores
R² 0.9983 / median 1.96 %. A deliberately incorrect NIR/Clear swap scores R² 0.795 / 25.8 %
on the same data — worth knowing as the signature to look for if a future import goes wrong.

**Acquisition** ([spec_meas.cpp](src/src/as7341/spec_meas.cpp)):

- Add `gain_multiplier(as7341_gain_t)` in `spec_meas` — **not** in the vendored
  `Adafruit_AS7341` (AGENTS.md: do not reformat vendored driver libs). The only existing
  ordinal→multiplier switch is inside the dead `toBasicCounts()`.
- Per-channel saturation: replace the single `out->saturated` bool with a 10-bit mask, and
  compare against the reported `(atime+1)*(astep+1)` rather than the compile-time constant.
- Read the AS7341's own ASAT bits from STATUS2 for `flags` bit2 (condition byte). Today the only STATUS2
  access is the AVALID bit in `getIsDataReady()`, so analog saturation below 50000 digital
  counts is invisible.
- Give `dual_exposure()` a return value for `flags` bit3 (condition byte). It is currently `void` and
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

Nothing here is fitted on **ambit** hardware. Three of the five now ship a seed rather than
an identity value (§7), which changes the failure mode from "obviously zero" to "plausible
but wrong" — the `flags` calibration byte (§5a) is what keeps that visible: bit8 stays 0
while `par_weight` is miniPar's, bit9 stays 0 until this device has been swept.

| quantity | shipped default | still needs | ambit data today |
|---|---|---|---|
| `spec_offset[10]` | ams workbook — device-independent | nothing | n/a |
| `spec_sens[10]` | miniPar LR1-B, 3 devices | LR1-B across ambit optics | 1 session |
| `par_weight[10]` | miniPar Li-250A fleet OLS | Li-250A across **many source families** — required, §1a | none |
| `par_slope` | 1.0 | per-device intensity sweep against a Li-250A **or a transfer standard** (§1a); a Li-250A anchor is an optional TODO worth ~4 % | none |
| `par_intercept` | 0.0 | same sweep | none |

What exists today: [data/multi_ambit_spec_lr1b.csv](data/multi_ambit_spec_lr1b.csv) — 3 rows,
3 devices, one light condition (`office`), one timestamp, one LR1-B trace. There is no
Li-Cor reference data anywhere in the ambit repo, so tiers 2 and 3 have no ambit fitting
basis at all. The seeds make the command *useful* before that campaign; they do not make it
*calibrated*, and the two must not be confused in anything reported to a user.

**How to know the seeds have been earned out.** The miniPar port is finished when: an ambit
LR1-B session across several source families replaces §7b; an ambit Li-250A campaign replaces
§7c with a constrained or shrunk fit that has physical signs; `AMBIT_PAR_WEIGHT_IS_AMBIT_FIT`
flips to true (§5a); and each production device gets a tier-3 sweep. Until the third of those,
bit8 stays 0 on every device however well it has been swept — which is correct, and is why a
host must test bit8 and bit9 separately rather than collapsing them.

Two constraints on the campaign, both learned the hard way on miniPar:

- **Tier 2 must be fitted fleet-wide, from many source families.** §2's leave-one-device-out
  table shows what a per-device 10-coefficient fit from a single source does.
- **Expect collinearity.** miniPar's plain-OLS fit has unphysical signs (negative on F3, F5,
  F8; positive on Clear) because the design matrix is collinear (condition number ≈ 451) and
  the dataset is daylight-dominated. A constrained fit (visible channels ≥ 0, Clear/NIR free)
  or shrinkage toward the ams matrix's PAR weights is the recommended fix. Plan for enough
  narrowband LED and gel-filter measurements to break it. §7c shows how unstable the
  unconstrained fit is — coefficients move by a factor of 3 between an 80 % subsample and the
  full set while R² barely changes — so do not treat a high R² as evidence the vector is
  right. The ambit campaign is the chance to do this properly the first time rather than
  inheriting the same problem.

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
| `Scripts/regression_PAR_miniPAR.ipynb` | PAR model comparison; source of §7c's `par_weight`. Its tier-2 export cell has never been run and `par_coeffs_fleet.json` does not exist, so §7c was regenerated by re-executing that cell's code; the stored 80/20 cell reproduces exactly, which is the check that it was done faithfully. |
| `Scripts/spectralCalibration_miniPAR_LR1-B.ipynb` | goal A; source of §7b's `spec_sens`. Note its printed offset output is stale — the label says "same units" while the numbers shown are the old ×1000 seconds-convention ones. The *code* is ms and is what §7b relies on. |
| `Scripts/as7341_calibrate.py` | host implementation of tier 1 + goal A. **`OFFSET_BASIC` is currently zeroed**, with the real ams vector commented out above it, so `calibrated_raw()` skips the offset its own `SPECTRAL_COEF` was fitted with. A miniPar-side inconsistency; it does not affect what was ported here (§7e). |
| `Firmware/docs/AS7341_AD000198_3-00.xlsx` | ams workbook: dark offsets, 721×10 reconstruction matrix |
| `Firmware/docs/AS7341_AN000633_2-00.pdf` | ams app note: §2.1, §2.2, §2.4 |
| `Firmware/src/app/spectrometer_api.cpp:578-584` | miniPar firmware, all three tiers in 3 lines |
