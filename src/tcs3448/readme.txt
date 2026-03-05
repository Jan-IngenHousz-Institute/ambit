# TCS3448 Driver Development Guide

This guide summarises the **non‑register** information from the TCS3448 datasheet that is essential for writing a robust C driver. It complements the register‑definition header file and focuses on device behaviour, communication protocols, timing, configuration workflows, and interrupt handling.

---

## 1. I²C Interface

### Slave Address
- **7‑bit address**: `0x59`

### Bus Characteristics
- **Voltage**: 1.2 V or 1.8 V, selected **once** at startup via the GPIO pin.
- **Clock frequency**: Up to **1 MHz** (standard/full‑speed).
- **Pull‑up resistors**: Required on SCL and SDA to the selected I²C bus voltage.

### Transaction Rules
- **Register auto‑increment**: After each byte, the internal register address pointer increments automatically. This pointer persists across STOP conditions.
- **16‑bit register access**:
  - **Write**: Must send **low byte first**, then **high byte**. Writing only one byte will corrupt the register.
  - **Read**: Must read **low byte first**; this action **latches** the whole 16‑bit value. The high byte must be read immediately afterwards.
  - **Burst reads/writes** are recommended for multi‑byte fields.
- **Register banking**: To access registers at addresses `0x58`–`0x66`, set `REG_BANK = 1` in `CFG0 (0xBF)`. Switch back to bank `0` (default) for all other registers (`≥0x80`).

### Startup & I²C Bus Voltage Selection
1. Apply power to `VDD` (1.8 V nominal).
2. **Wait 600 µs** after power‑on reset. During this time:
   - No I²C communication is possible.
   - The device measures the voltage on the **GPIO** pin to decide the I²C I/O level.
3. **GPIO connection rules** (do **not** leave floating):
   - `GPIO = GND`              → I²C = 1.2 V
   - `GPIO = 1.2 V`           → I²C = 1.2 V
   - `GPIO = 1.8 V`           → I²C = 1.8 V
   - `GPIO = 3.3 V`           → I²C = 1.8 V (3 V tolerant)
4. After this measurement, GPIO becomes a normal digital I/O pin.

---

## 2. Device States and Power Management

The device has four main operational states:

| State   | `PON` bit | `ALS_EN` | Oscillator | Typical Current | Description |
|---------|-----------|----------|------------|-----------------|-------------|
| **SLEEP**   | `0`       | –        | Off        | 0.75 µA         | Power‑down. I²C wake‑up temporarily, no measurement. |
| **IDLE**    | `1`       | `0`      | On         | 40–60 µA        | Ready, timers run, no ADC active. |
| **ACTIVE**  | `1`       | `1`      | On         | 210–280 µA      | ADC conversion in progress. |
| **WAIT**    | `1`       | `1`      | On (low)   | Reduced         | Programmed delay between ALS cycles (if `WEN=1`). |

**Initialisation after power‑up**:
- Duration: **~200 µs** (plus the 600 µs GPIO sampling).
- Device sends NACK on I²C and ignores all transactions.
- After init, device enters **SLEEP** (`PON=0`).

**State transitions**:
- `PON = 1` → **IDLE**.
- `ALS_EN = 1` → **ACTIVE** (measurement starts).
- `ALS_EN = 0` → back to **IDLE**.
- `PON = 0` → back to **SLEEP**.

**Sleep‑after‑interrupt (SAI)**:
- If `SAI = 1` in `CFG3`, an interrupt forces the device into **SLEEP** (registers unchanged).
- Exit: clear the interrupt(s) and write `1` to `CLEAR_SAI_ACT` in the `CONTROL` register.

---

## 3. Timing Configuration

### ALS Integration Time
- **Formula**:  
  \( t_{int} [ms] = (ATIME + 1) \times (ASTEP + 1) \times 2.78 \times 10^{-3} \)
- **ADC full scale**: \( (ATIME + 1) \times (ASTEP + 1) \) counts.
- **Constraints**:
  - Both `ATIME` and `ASTEP` cannot be zero simultaneously.
  - `ATIME` = 0…255, `ASTEP` = 0…65535 (16‑bit, two registers).
  - Default `ASTEP` = 999 → step = 2.78 µs, total = (ATIME+1) × 1000 × 2.78 µs.

### Wait Time (between ALS cycles)
- Enabled by `WEN = 1` in `ENABLE` register.
- **Without `WLONG`**:  
  \( t_{wait} [ms] = (WTIME + 1) \times 2.78 \times 10^{-3} \)
- **With `WLONG = 1`** (CFG0): multiply by 16.
- `WTIME` = 0…255.
- **Warning**: If `WTIME` is too short to complete the ALS measurement, the `ALS_TRIG` status bit is set.

### Flicker Detection Timing
- Independent timing via `FD_TIME` (11‑bit) and `FD_GAIN` (same encoding as `AGAIN`).
- \( t_{int\_FD} [ms] = FD\_TIME \times 2.78 \times 10^{-3} \)
- Must not change `FD_TIME` or `FD_GAIN` while `FDEN = 1` and `PON = 1`.

---

## 4. Gain Configuration

### ALS Gain (`AGAIN`)
- 13 settings from 0.5× to 2048× (see header for enum).
- Applied to ADC channels 0‑4.
- Changing gain **during** an active measurement is not recommended.

### Flicker Detection Gain (`FD_GAIN`)
- Independent 5‑bit field in `FD_TIME_2`.
- Uses the same encoding as `AGAIN`.

**Gain ratios** (relative to 128×) are provided in the datasheet; they can be used for compensation calculations.

---

## 5. Measurement Flow

1. **Power up**: Wait 600 µs after POR.
2. **Set I²C voltage** via GPIO (hardware fixed).
3. **Initialise**:
   - Write `PON = 1` (device enters IDLE).
   - Configure `ATIME`, `ASTEP`, `WTIME` (if needed).
   - Set `AGAIN` (and `FD_GAIN` if using flicker).
   - Configure `SMUX` (see §8) or rely on default `auto_smux`.
   - Configure interrupts if used.
4. **Start measurement**: Set `ALS_EN = 1` (and `FDEN = 1` if needed).
5. **Wait for completion**:
   - Poll `AVALID` (STATUS2) or use interrupt.
   - Read `ASTATUS` **first** – this latches **all 18 ADC data registers**.
   - Then read the 16‑bit values from `ADATA0_L` … `ADATA17_H`.
6. **Repeat** or stop: `ALS_EN = 0`.

---

## 6. Interrupt Handling

### Interrupt Pin (`INT`)
- Open‑drain, active low.
- Requires external pull‑up to 1.8 V (not to the I²C bus voltage).
- Multiple interrupt sources; read `STATUS` to determine cause.

### Interrupt Enables (`INTENAB` register)
- `ASIEN`   – ALS/FD saturation interrupt.
- `ALS_IEN` – ALS threshold interrupt (based on CH0 and persistence).
- `F_IEN`   – FIFO buffer interrupt.
- `SIEN`    – System interrupt (flicker status change, SMUX finish).

### Clearing Interrupts
- **`STATUS` register (0x93)** is **self‑clearing by writing back the read value**.
  - Read `STATUS`, handle the event, **write the same byte back** to clear the handled bits.
- Saturation status bits in `STATUS2` and `FD_STATUS` are cleared by writing `1` to the respective bit.

### ALS Threshold Interrupt
- Uses **CH0** by default (selectable via `ALS_TH_CH` in CFG12).
- Set 16‑bit low/high thresholds in `ALS_TH_L` and `ALS_TH_H`.
- **Persistence filter** (`APERS`): number of consecutive out‑of‑range measurements before interrupt. Value mapping:
  - 0 = every cycle
  - 1…14 = 1,2,3,5,10,15,20,25,30,35,40,45,50,55
  - 15 = 60

### FIFO Interrupt
- Triggered when `FIFO_LVL` ≥ the threshold set in `FIFO_TH` (CFG8).
- Interrupt is cleared when FIFO level drops below threshold **and** `FINT` bit is cleared.

### System Interrupt
- Sources: `SINT_FD` (flicker detection status changed) and `SINT_SMUX` (SMUX operation finished).
- Enabled via `SIEN_FD` and `SIEN_SMUX` in `CFG9`.

---

## 7. FIFO Buffer Operation

The FIFO reduces I²C traffic by storing multiple ADC samples.

- **Size**: 256 bytes → **128 entries** (each entry is 2 bytes, except when 8‑bit mode enabled).
- **FIFO_LVL**: indicates number of **entries** waiting. Range 0…128.
- **Overflow**: `FIFO_OV` is set; new data is lost.

### Configuration
- `FIFO_MAP` (0xFC): select which channel data is written to FIFO.
- `FIFO_TH` (CFG8): set trigger level for `FINT`.
- `FD_FIFO_8b` (CFG20): set `1` to store flicker data as **8‑bit** values (saturates at 0xFF if >8 bits).
- `FIFO_WRITE_FD` (FD_CFG0): enable writing flicker raw data to FIFO (ignored if FDEN=0).

### Reading FIFO
- Read from `FDATA_L (0xFE)` and `FDATA_H (0xFF)`.
- **Single byte reads**: Internal FIFO pointer increments **after each low‑byte read**; FIFO_LVL updates accordingly.
- **Block reads**: Pointer increments after each **2‑byte entry**.
- If FIFO_LVL = 0 and read is attempted, device returns 0.
- To fully clear FIFO interrupt, **all data must be read** and then the `FINT` bit cleared.

---

## 8. SMUX – Channel Multiplexer

The SMUX connects photodiodes to the six ADCs. Two operation modes:

### a) Automatic Mode (`auto_smux` in CFG20)
- Recommended for most applications.
- Pre‑defined sequences:
  - **6‑channel**: FZ, FY, FXL, NIR, 2×VIS, FD  (single cycle)
  - **12‑channel**: two cycles (adds F3, F4, F6)
  - **18‑channel**: three cycles (adds F1, F7, F8, F5)
- Data is automatically stored in ADATA0…ADATA17 registers.
- Should be configured **before** starting a measurement.

### b) Manual Mode
- Write desired SMUX configuration to RAM using `SMUX_CMD`.
- Set `SMUXEN = 1` to execute.
- `SINT_SMUX` interrupt indicates completion.
- **Do not change** `SMUX_CMD` during operation.

---

## 9. LED Driver

- Pin `LDR` is a current sink; connect external LED with series resistor.
- **Control**:
  - `LED_ACT = 1` → LED on.
  - `LED_DRIVE[6:0]` sets sink current:
    - 0x00 = 4 mA
    - 0x01 = 6 mA
    - 0x02 = 8 mA
    - … increments of 2 mA up to 0x7F = 258 mA
- Can be synchronised with external signal via GPIO.

---

## 10. Autozero (AZ)

The device periodically resets ADC offsets to compensate for temperature drift.

- **Frequency**: set by `AZ_NTH_ITERATION` (AZ_CONFIG).
  - 0 = never (not recommended)
  - 1 = every integration cycle
  - … 254 = every 254 cycles
  - 255 = only before first measurement
- Autozero takes **~15 ms** and will **interrupt** an ongoing flicker detection measurement if `FDEN = 1`.

---

## 11. Important Notes for Robust Driver Design

1. **Never write to reserved registers** or change reserved bits. Doing so may cause unpredictable behaviour.
2. **Always read `ASTATUS` first** when fetching ALS data – this latches all 16‑bit ADC values consistently.
3. **16‑bit writes** must be performed as low‑byte then high‑byte in one I²C burst, or with two separate writes **without** any other access in between.
4. **Saturation handling**:
   - If `ASAT` or `FDSAT` is set, the measurement data is invalid.
   - Adjust gain (`AGAIN`) or integration time and restart.
5. **Timing errors**:
   - `ALS_TRIG` = `WTIME` too short.
   - `FD_TRIG` = flicker timing misconfigured.
6. **Software reset**: Write `SW_RESET = 1` in `CONTROL` register – forces a full power‑on reset.
7. **GPIO after startup**: The pin becomes a normal I/O; configure it via `GPIO` register if used.

---

## 12. Typical Initialisation Sequence (Pseudo‑code)

```c
// 1. Wait at least 600 µs after VDD stable

// 2. Set I2C voltage via GPIO (hardware), then begin I2C

// 3. Power on
write_register(ENABLE, PON);

// 4. Configure timing
write_register(ATIME, 0x00);      // 1 step
write_register(ASTEP_L, 0xE7);    // default 999 LSB
write_register(ASTEP_H, 0x03);    // default 999 MSB
write_register(WTIME, desired_wait);

// 5. Set gain
write_register(CFG1, AGAIN_128X);

// 6. (Optional) Flicker detection
write_register(ENABLE, read | FDEN);
write_register(FD_TIME_1, fd_time_lsb);
write_register(FD_TIME_2, (fd_gain << 3) | fd_time_msb);

// 7. (Optional) Interrupts
write_register(INTENAB, ALS_IEN | F_IEN);
write_register(ALS_TH_L_LSB, low_th_lsb);
write_register(ALS_TH_L_MSB, low_th_msb);
write_register(ALS_TH_H_LSB, high_th_lsb);
write_register(ALS_TH_H_MSB, high_th_msb);
write_register(PERS, APERS_10);

// 8. (Optional) FIFO
write_register(FIFO_MAP, FIFO_MAP_CH0_DATA | FIFO_MAP_ASTATUS);
write_register(CFG8, FIFO_TH_8 << 6);
write_register(CONTROL, FIFO_CLR);   // clear any stale data

// 9. Start measurement
write_register(ENABLE, read | ALS_EN);

// 10. Wait / interrupt / read data...
```

---

This guide, together with the register‑definition header file, provides all necessary information to write a complete, reliable driver for the TCS3448. For absolute specifications (optical, electrical, mechanical) always refer to the official ams OSRAM datasheet.