# MLX_CALIBRATION — reliable MLX90632 calibration constants

Branch `fix/mlx-calibration-read` (from `main` @ 7c3395b).

## Symptom (2026-10-06, AD85)

`mlx_init()` reads the sensor's 13 calibration constants from its EEPROM once per boot.
- One boot in ~12 kept a set that put a 21 °C leaf at **317 °C** (object temperature) for
  the whole session.
- The die/ambient temperature stayed correct, so the bad value was among
  Ea Eb Fa Fb Ga Ha Hb Ka.
- The firmware's own `temp` read agreed, so this is the shared calibration, not one read
  path.
- A reboot fixed it.

On good boots the constants were identical every time. Some relations between them looked
suspicious (`Eb == P_R`, `Ea == P_G >> 4`, `Gb == Ka`, `P_T = 0`), but §Root cause shows
they are the sensor's real EEPROM content and consistent with the formula.

## What was wrong in the code

- **The bus callbacks reported success unconditionally.** `mlx90632_i2c_read()` and
  `mlx90632_i2c_write()` returned 0 regardless of the BusIO result, and the 16/32-bit
  helpers ignored it too. A NACK or short read became a silent zero word:
  - at boot, a wrong constant;
  - in a measurement, a DATA_RDY poll that could only end in its 2 s timeout, after which
    `mlx_measure()` computed a temperature from placeholder raw words.
- **The constants were read once, unchecked,** with 4-byte burst reads, and nothing
  verified them.
- **The refresh-rate write used an unverified read.** `mlx90632_set_refresh_rate()` erases
  and rewrites EE_MEDICAL_MEAS1/2 from a value it has just read. One bad read there can
  write garbage into the sensor EEPROM permanently.
- **`mlx_init()` was fire-and-forget.** Its result was ignored at boot, and every call
  leaked an `Adafruit_I2CDevice`.

## Root cause

Measured on two units (2026-10-06):
- **AD85**: MAC 3c:dc:75:0d:fd:58.
- **A second, older unit**: MAC 3c:dc:75:0d:fb:d4, NVS name AmbitV005, previously on
  pre-version-token firmware.

**1. The read path is not systematically wrong.**
- `mlxee` compared, for every constant:
  - the EEPROM words read one checked 16-bit transaction at a time (three passes);
  - the old 4-byte burst read;
  - the set `mlx_init()` kept.
- All three agree on both units, and the three passes are identical, with zero failed
  transactions.
- The odd-looking relations follow from the Melexis formula:
  - `Eb == P_R`: both are the raw ambient reading at 25 °C (×256). P_R is used in the
    ambient temperature, Eb in the object compensation.
  - `Ea == P_G / 16`: the same ambient gain at two fixed-point scales (Ea/2¹⁶ vs P_G/2²⁰).
  - `Gb == Ka` and `P_T = 0`: an equal ambient and IR beta, and no quadratic ambient term.
- The two units share PG, PT, PO, Ea, Fb, Ga, Gb, Ka, Ha and Hb exactly and differ in
  P_R/Eb and Fa. That looks like lot-level constants with a per-unit trim.

**2. Occasionally a boot read returns wrong data without any I2C error.** With the fix's
counters, over 181 software reboots:

| unit | boots | first pass rejected | I2C errors | second pass |
|---|---|---|---|---|
| AD85 | 81 | 3 | 0 | clean every time |
| second unit | 100 | 2 | 0 | clean every time |

- So ~3 % of boots get a set that is inconsistent between the two passes (or fails the
  version / non-zero checks), with every transaction acknowledged.
- The old code read once and kept whatever came back. That matches the 317 °C boot: the
  die stayed right, so one object constant was wrong.
- Checking the I2C result alone would not have caught this. The second, identical pass does.
- Not identified: which word goes wrong, and why only on the first pass after a software
  reset. A plausible but **unverified** cause is a bus left mid-transfer by the reset (no
  I2C bus-recovery clocks at boot).

**3. The refresh-rate EEPROM write was never exercised.** Both units already hold 16 Hz in
EE_MEDICAL_MEAS1/2 (`0x850D` / `0x851D`), so the old unchecked write path did not damage
them. The fix keeps it from ever running on an unverified read.

## Fix

- **Checked bus callbacks.** Every transaction returns -EIO on failure, so the vendor
  driver's existing `ret < 0` paths finally trigger.
- **Verified constants.** Each word is its own checked 16-bit read; 32-bit constants are
  LSW at the address, MSW at address + 1. The full set is read twice and kept only if both
  passes match word for word and the divisors (P_G, Ea, Fa, Ha) are non-zero.
  - Up to 5 attempts, each including the EEPROM-version check.
  - No range checks: units differ, and a wrong bound would disable a good sensor
    permanently.
- **A real "valid" state.** `mlx_calibration_valid()`, and `mlx_measure()` retries
  `mlx_init()` while it is false, so a bad boot heals at the next measurement.
- **Invalid readings are explicit.** With no verified calibration, or a failed read,
  `mlx_measure()` returns `MLX_TEMP_INVALID` (−273.15 °C) for object, ambient and reflected
  temperature. That value is impossible for a real reading and survives every encoding in
  use:
  - `env` / array 0: −27315 centi-°C;
  - cmd 32/34: −2731 deci-°C;
  - JSON: −273.15.

  This is the only change in what hosts can receive, and it replaces values that used to be
  arbitrary.
- **`mlx_coef` is zero without verified constants,** instead of the Melexis placeholder
  set. It feeds the boot banner, the cmd 33 struct and `cal_version`; struct layout
  unchanged.
- **The refresh-rate EEPROM write runs only on a verified read** (both words, read twice,
  equal), only if the rate actually differs, and is confirmed by read-back.
- **Console `mlxee,<reps>`** (read-only) dumps EEPROM 0x2400–0x24FF and compares the old
  burst read with the word read for each 32-bit constant.
- **`mlxfail,<n>`** (fault injection) is built only with `-DAMBIT_DIAG_MLX`.

Known limitation: if the boot init fails and a later measurement re-initialises
successfully, `ambit_calibration_local.mlx_coef` keeps the zeros until the next boot.
Readings are correct; only cmd 33 / banner lag.

## Verification (bench build `-DAMBIT_DIAG_MLX`, 2026-10-06)

| check | AD85 | second unit |
|---|---|---|
| `mlxee` ×3: word reads stable, burst == words == kept | ✅ | ✅ |
| reboots, one constant set, every `temp` plausible | ✅ 50 + 81 | ✅ 10 + 100 |
| first pass rejected / I2C errors | 3 / 0 | 2 / 0 |
| `mlxfail,1` and `mlxfail,3`: retry recovers | ✅ | ✅ |
| `mlxfail,40` and `mlxfail,1000`: `init=0 valid=0` | ✅ | ✅ |
| `temp` while reads fail | −273.15 ×3, ~1 s | −273.15 ×3, ~1 s |
| `mlxfail,0` then `temp`: recovered | ✅ 20.90 °C | ✅ 20.10 °C |
| refresh-rate EEPROM write | not needed (already 16 Hz) | not needed |

`pio run` is clean for both the release and the bench build; `mlxfail` exists only in the
bench binary.

Still open:
- the `plans/HW_CONFORMANCE.md` cmd 32 / 33 / 34 and `env` rows against an Ambyte or a
  host harness (framing is unchanged by construction);
- a field-data check whether `cal_version` / `mlx_coef` has ever changed across boots for
  one `sensor_id`.

**Note for the second unit.** After the reflash it reports the firmware default name
`AmbitV003` instead of `AmbitV005`. Its old firmware stored settings differently; NVS
itself was not erased (same 0x9000 / 20 KB partition). This is a pre-existing
compatibility gap between that old image and current firmware, unrelated to this fix.

## Out of scope

- NVS `Emit`, `temp_offset` and `temp_slope` are loaded and printed but never applied:
  `u_mlx.cpp` keeps its own `mlx_emissivity = 1.0`. Applying them changes reported
  temperatures fleet-wide.
- The `arrun-temperature` dense read (`mlx_raw_to_celsius`) uses the same `mlx_cali_*`
  globals. When it rebases onto this fix it must also check `mlx_calibration_valid()`.
