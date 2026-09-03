# Handoff: load the ADPD6000 factory 960 kHz oscillator trim at init

> Status: **not started.** Found 2026-09-02 on the `deterministic-adpd` bench
> (`plans/DETERMINISTIC_ADPD.md` §8). Independent of that branch: it is an `arrun` / free-run
> bug on `main` and should ship as its own PR, e.g.
> `fix(adpd): load the factory 960 kHz oscillator trim at init`.

## 1. Symptom

The free-run array engine (`run_arr_type1`, binary cmd 21, text `arrun*`, JSON `arrun`) samples
≈ 21 % faster than the requested `freq` on the test unit (`AmbitV003`), at every rate:

| Line | Requested | TIMING span (µs ticks → s) | Implied rate |
|---|---|---|---|
| 200 Hz, N=1000 | 5.00 s | 4.14 s, 4.20 s | ≈ 241 Hz |
| 200 Hz, N=500 | 2.50 s | 2.13 s | ≈ 240 Hz |
| 100 Hz, N=500 | 5.00 s | 4.14 s | ≈ 121 Hz |
| 10 Hz, N=100 | 10.0 s | ≈ 8.3 s | ≈ 12 Hz |

The TIMING block (`d_timing`, cmd 21 array index 7) is the ESP `esp_timer` at run start and end,
so the span is a crystal-derived measurement of how long the chip took to produce N samples. The
triggered engine on the branch, which paces the chip from `esp_timer`, produces exactly N/freq.

Consequences for the fleet: the Ambyte and the openJII pipeline assume within-run point timing
= `1/freq` (see the comment above `PAM_get_env()` in `src/PAM.cpp`); kinetics are therefore
time-scaled by each unit's oscillator error, and each unit also emits ≈ 21 % more LED pulses per
second than requested. The error is unit-specific (RC oscillator process spread).

## 2. Root cause

`ADPD6::run_freq(freq)` → `Driver::set_slot_frequency(960000, freq)` assumes the low-frequency
timer clock is exactly 960 kHz. That clock is the chip's internal RC oscillator
(ADPD6000 DS Rev. 0, "Low Frequency Oscillator"): *"The internal 960 kHz clock frequency is
set using the 10-bit OSC_960K_FREQ_ADJ bits"* (register 0x000B `OSC960K`, bits [9:0], reset
value 0x2B2 → untrimmed). The datasheet also documents a calibration path against an external
reference via the time-stamp feature (`OSC_CAL_ENABLE`, `TIMESTAMP_COUNT_x`).

The ADI reference driver (vendored in the pre-cleanroom repo
`c:/Users/LudovicoCaracciolo/Documents/Git-repo/LEGACY/ambit-IoT`,
`src/src/adpd/lib/ADPD6000/adi_adpd6000_device.c`) contains a factory-fuse load that our
clean-room driver (`src/src/adpd/jii_adpd6000.cpp`) never ported:

```c
int32_t adi_adpd6000_device_cal_960k_osc(adi_adpd6000_device_t *device)
{
    // standby first
    adi_adpd6000_device_enable_slot_operation_mode_go(device, false);
    adi_adpd6000_hal_bf_write(device, 0x00000053, 0x00000104, 1);   // reg 0x53 bit 4 = 1
    adi_adpd6000_hal_reg_write(device, 0x00000044, 7);              // reg 0x44 = 7
    // poll reg 0xD6 bits [2:1] until == 2 (timeout 0x1000 reads)
    //   fuse_code = (reg 0xC8 << 8) | reg 0xC9
    //   write fuse_code to OSC_960K_FREQ_ADJ (reg 0x0B, bits [9:0])   -- BF info 0x0B, 0x0A00
    // else API_ADPD6000_ERROR_FUSE_NOT_DONE
    adi_adpd6000_hal_reg_write(device, 0x00000044, 0);
    adi_adpd6000_hal_bf_write(device, 0x00000053, 0x00000104, 0);
}
```

`adi_adpd6000_hal_bf_write(dev, reg, info, v)`: `info` = `(width << 8) | lsb`, so
`0x00000104` = 1 bit at bit 4, `0x00000A00` = 10 bits at bit 0, `0x00000201` = 2 bits at bit 1.
Registers 0x44, 0x53, 0xC8, 0xC9, 0xD6 are **not in the public datasheet** (test/fuse access);
the sequence is ADI's own, copy it exactly and do not "improve" it. Nothing in LEGACY ever called
this function either, so the fleet has always run untrimmed.

## 3. Implementation notes

- Add a `Driver::load_oscillator_trim()` (or similar) to `src/src/adpd/jii_adpd6000.{h,cpp}`
  implementing the sequence above with the existing `read_register` / `write_register` /
  `update_register`, returning `kOk`, a timeout code, or the I/O error. Keep the clean-room
  licence header; the register numbers are hardware facts, the code is ours.
- Call it once from `ADPD6::begin()` (`src/src/adpd/u_adpd6100.cpp`) after the identify /
  reset step and **in standby** (OP_MODE = 0); the datasheet forbids control-register writes in
  GO mode. Log nothing on UART0 (AGENTS.md: the port is the protocol port). A failure must not
  block boot — fall back to the untrimmed value and expose the status somewhere readable (e.g.
  a field in the cmd 33 fw-info reserved bytes, or the text `check` verb), so the Calibratron
  can tell trimmed from untrimmed units.
- `run_freq()` stays as is: with the trim loaded, 960 kHz is what the divider assumes.
- Do **not** change the wire: cmd 21 bytes, array layout, TIMING semantics all stay. What
  changes is the *actual* sample rate, which is the point. Note it in `plans/HW_CONFORMANCE.md`
  (the byte-diff will show different TIMING spans and, on a living target, different F values
  because fewer pulses arrive per second).

## 4. Verification

Bench, text console at 115200 (the DUT is on COM61 behind the Teensy bridge; close the terminal
first; opening the port may power-cycle the board — wait for the boot banner):

```
arrun2,1,0,1,0,3,232,0,200,0,1     # 200 Hz, N=1000: TIMING span must become 5.00 s (+≈50 ms overhead), was 4.14–4.20 s
arrun2,1,0,1,0,1,244,0,100,0,1     # 100 Hz, N=500:  2.50 s expected (note: 100 Hz free-run may reset the ESP via the BOOT-pin gesture on this bench, see DETERMINISTIC_ADPD.md §8 — use 80 Hz if it does)
arrun2,1,0,1,0,0,100,0,10,0,1      # 10 Hz, N=100:   10.0 s expected
```

TIMING is the last array (`Data:timing,Length:2  t_begin,t_end` in µs). Optional cross-check:
`tseq` on the branch measures the per-sequence time independently of the LF clock and should
not change. A second unit should be measured before and after to confirm the error is
unit-specific and the trim removes it on both.

Residual after the trim should be within the RC's trimmed tolerance (a few %). If a tighter time
base is ever needed, the branch's triggered engine (`arrunt`) is exact by construction.
