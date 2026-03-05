/**
 * @file tcs3448.cpp
 * @brief TCS3448 driver implementation - Low-level hardware interface
 * 
 * Implementation of the TCS3448 14-channel spectral sensor driver.
 * Provides register-level access with automatic bank switching,
 * retry logic, and platform-independent I2C communication.
 * 
 * @par Implementation Details:
 * - **Register Banking**: Automatically switches between Bank 0 (0x80-0xFF) and
 *   Bank 1 (0x20-0x7F) using the REG_BANK bit in CFG0 register
 * - **I2C Retry Logic**: All I2C operations retry up to 10 times before failing
 * - **16-bit Access**: Handles low-byte-first requirement for multi-byte registers
 * - **Error Reporting**: I2C failures reported via Serial (Arduino-only)
 * 
 * @par Register Access Strategy:
 * Low-level register functions (read_reg8, write_reg8, etc.) automatically
 * handle bank switching. This allows transparent access to all registers
 * without manual bank management.
 * 
 * @author TCS3448 Driver Team
 * @version 1.0.0
 * @date 2026
 * 
 * @see tcs3448.h for API documentation
 * @see tcs3448_meas.cpp for high-level measurement interface
 */

#include <Arduino.h>
#include "tcs3448.h"

/**
 * @brief Number of I2C retry attempts for failed transactions
 * 
 * Each I2C read/write operation will retry up to this many times
 * before reporting failure. This improves reliability on noisy
 * I2C buses or with marginal signal integrity.
 */
#define TCS3448_RETRY_COUNT     10

/**
 * @brief Internal I2C write with retry logic
 * 
 * Attempts to write data to a device register, retrying up to
 * TCS3448_RETRY_COUNT times on failure. Reports failures via Serial.
 * 
 * @param[in] dev Pointer to device handle with I2C function pointers
 * @param[in] reg Register address to write to
 * @param[in] data Pointer to data buffer to write
 * @param[in] len Number of bytes to write
 * @return true on success, false if all retries exhausted
 * 
 * @note This is an internal function not exposed in the public API.
 * It wraps the user-provided i2c_write function pointer.
 * 
 * @par Retry Strategy:
 * - Attempts I2C write up to 10 times
 * - Returns immediately on success
 * - Prints error message via Serial after all retries fail
 */
static bool tcs3448_i2c_write(tcs3448_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len) {
    for (int retry = 0; retry < TCS3448_RETRY_COUNT; retry++) {
        if (dev->i2c_write(dev->i2c_addr, reg, data, len) == 0) {
            return true;
        }
    }
    Serial.print("TCS3448 I2C write failed, reg: 0x");
    Serial.println(reg, HEX);
    return false;
}

/**
 * @brief Internal I2C read with retry logic
 * 
 * Attempts to read data from a device register, retrying up to
 * TCS3448_RETRY_COUNT times on failure. Reports failures via Serial.
 * 
 * @param[in] dev Pointer to device handle with I2C function pointers
 * @param[in] reg Register address to read from
 * @param[out] data Pointer to data buffer to store read data
 * @param[in] len Number of bytes to read
 * @return true on success, false if all retries exhausted
 * 
 * @note This is an internal function not exposed in the public API.
 * It wraps the user-provided i2c_read function pointer.
 * 
 * @par Retry Strategy:
 * - Attempts I2C read up to 10 times
 * - Returns immediately on success
 * - Prints error message via Serial after all retries fail
 */
static bool tcs3448_i2c_read(tcs3448_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    for (int retry = 0; retry < TCS3448_RETRY_COUNT; retry++) {
        if (dev->i2c_read(dev->i2c_addr, reg, data, len) == 0) {
            return true;
        }
    }
    Serial.print("TCS3448 I2C read failed, reg: 0x");
    Serial.println(reg, HEX);
    return false;
}

/**
 * @brief Set register bank for subsequent operations
 * 
 * Switches the device between Bank 0 (0x80-0xFF) and Bank 1 (0x20-0x7F)
 * by modifying the REG_BANK bit in CFG0 register.
 * 
 * @param[in] dev Pointer to device handle
 * @param[in] bank Target bank: 0 for Bank 0, 1 for Bank 1
 * 
 * @note This function optimizes by checking current_bank and skipping
 * the write if already in the correct bank.
 * 
 * @par Bank Switching Procedure:
 * 1. Read current CFG0 value
 * 2. Set or clear REG_BANK bit (bit 4)
 * 3. Write back to CFG0
 * 4. Update current_bank field in device handle
 * 
 * @warning CFG0 (0xBF) is always in Bank 0, so this function must
 * operate while in Bank 0. The function handles this internally.
 */
static void tcs3448_set_bank(tcs3448_dev_t *dev, uint8_t bank) {
    if (dev->current_bank == bank) return;
    
    uint8_t cfg0 = tcs3448_read_reg8(dev, TCS3448_REG_CFG0);
    if (bank) {
        cfg0 |= TCS3448_CFG0_REG_BANK;
    } else {
        cfg0 &= ~TCS3448_CFG0_REG_BANK;
    }
    tcs3448_write_reg8(dev, TCS3448_REG_CFG0, cfg0);
    dev->current_bank = bank;
}

/**
 * @brief Implementation of tcs3448_read_reg8()
 * 
 * @copydoc tcs3448_read_reg8
 * 
 * @par Implementation Details:
 * 1. Determine target bank from register address (0x20-0x7F = Bank 1)
 * 2. Call tcs3448_set_bank() to switch if needed
 * 3. Read single byte via tcs3448_i2c_read()
 */
uint8_t tcs3448_read_reg8(tcs3448_dev_t *dev, uint8_t reg) {
    uint8_t value = 0;
    uint8_t bank = (reg >= 0x20 && reg <= 0x7F) ? 1 : 0;
    if (reg == TCS3448_REG_CFG0) {
        tcs3448_i2c_read(dev, reg, &value, 1);
        return value;
    }
    tcs3448_set_bank(dev, bank);
    tcs3448_i2c_read(dev, reg, &value, 1);
    return value;
}

/**
 * @brief Implementation of tcs3448_write_reg8()
 * 
 * @copydoc tcs3448_write_reg8
 * 
 * @par Implementation Details:
 * 1. Determine target bank from register address (0x20-0x7F = Bank 1)
 * 2. Call tcs3448_set_bank() to switch if needed
 * 3. Write single byte via tcs3448_i2c_write()
 */
void tcs3448_write_reg8(tcs3448_dev_t *dev, uint8_t reg, uint8_t value) {
    if (reg == TCS3448_REG_CFG0) {
        tcs3448_i2c_write(dev, reg, &value, 1);
        return;
    }
    uint8_t bank = (reg >= 0x20 && reg <= 0x7F) ? 1 : 0;
    tcs3448_set_bank(dev, bank);
    tcs3448_i2c_write(dev, reg, &value, 1);
}

/**
 * @brief Implementation of tcs3448_read_reg16()
 * 
 * @copydoc tcs3448_read_reg16
 * 
 * @par Implementation Details:
 * Performs 2-byte burst read from reg_l address (low byte).
 * Device automatically increments to reg_l+1 (high byte).
 * Combines bytes: value = low | (high << 8)
 */
uint16_t tcs3448_read_reg16(tcs3448_dev_t *dev, uint8_t reg_l, uint8_t reg_h) {
    (void)reg_h;  // reg_h not used - automatic increment handles it
    uint8_t buf[2];
    tcs3448_i2c_read(dev, reg_l, buf, 2);
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

/**
 * @brief Implementation of tcs3448_write_reg16()
 * 
 * @copydoc tcs3448_write_reg16
 * 
 * @par Implementation Details:
 * Prepares 2-byte buffer with low byte first, then high byte.
 * Performs 2-byte burst write to reg_l address.
 * Device automatically increments to write high byte.
 * Uses static_cast to avoid C++ narrowing warnings.
 */
void tcs3448_write_reg16(tcs3448_dev_t *dev, uint8_t reg_l, uint8_t reg_h, uint16_t value) {
    (void)reg_h;  // reg_h not used - automatic increment handles it
    uint8_t buf[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
    tcs3448_i2c_write(dev, reg_l, buf, 2);
}

/**
 * @brief Implementation of tcs3448_init()
 * 
 * @copydoc tcs3448_init
 * 
 * @par Initialization Sequence:
 * 1. Store function pointers and I2C address
 * 2. Set current_bank = 0 (default after reset)
 * 3. Set initialized = false
 * 4. Delay 1ms for device startup
 * 5. Check device ID via tcs3448_check_id()
 * 6. Set initialized = true on success
 */
bool tcs3448_init(tcs3448_dev_t *dev, tcs3448_i2c_write_fn i2c_write,
                  tcs3448_i2c_read_fn i2c_read, tcs3448_delay_ms_fn delay_ms,
                  uint8_t i2c_addr) {
    dev->i2c_write = i2c_write;
    dev->i2c_read = i2c_read;
    dev->delay_ms = delay_ms;
    dev->i2c_addr = i2c_addr ? i2c_addr : TCS3448_I2C_ADDR;
    dev->current_bank = 0;
    dev->initialized = false;
    
    dev->delay_ms(1);
    
    if (!tcs3448_check_id(dev)) {
        Serial.println("TCS3448: Device ID check failed");
        return false;
    }
    
    dev->initialized = true;
    return true;
}

/**
 * @brief Implementation of tcs3448_check_id()
 * 
 * @copydoc tcs3448_check_id
 * 
 * @par Implementation Details:
 * 1. Switch to Bank 1 (TCS3448_REG_ID is at 0x5A in Bank 1)
 * 2. Read ID register
 * 3. Return true if matches TCS3448_DEVICE_ID (0x81)
 * 
 * @note Bank is switched back automatically by subsequent operations
 */
bool tcs3448_check_id(tcs3448_dev_t *dev) {
    tcs3448_set_bank(dev, 1);
    uint8_t id = tcs3448_read_reg8(dev, TCS3448_REG_ID);
    return (id == TCS3448_DEVICE_ID);
}

/**
 * @brief Implementation of tcs3448_power_on()
 * 
 * @copydoc tcs3448_power_on
 * 
 * @par Implementation Details:
 * Writes TCS3448_ENABLE_PON to ENABLE register (0x80).
 * Waits 1ms for oscillator startup.
 */
void tcs3448_power_on(tcs3448_dev_t *dev) {
    tcs3448_write_reg8(dev, TCS3448_REG_ENABLE, TCS3448_ENABLE_PON);
    dev->delay_ms(1);
}

/**
 * @brief Implementation of tcs3448_power_off()
 * 
 * @copydoc tcs3448_power_off
 * 
 * @par Implementation Details:
 * Writes 0x00 to ENABLE register, clearing all bits including PON.
 */
void tcs3448_power_off(tcs3448_dev_t *dev) {
    tcs3448_write_reg8(dev, TCS3448_REG_ENABLE, 0);
}

/**
 * @brief Implementation of tcs3448_reset()
 * 
 * @copydoc tcs3448_reset
 * 
 * @par Implementation Details:
 * 1. Write TCS3448_CONTROL_SW_RESET to CONTROL register (0xFA)
 * 2. Wait 10ms for reset completion
 * 3. Reset current_bank to 0 (device resets to Bank 0)
 */
void tcs3448_reset(tcs3448_dev_t *dev) {
    tcs3448_write_reg8(dev, TCS3448_REG_CONTROL, TCS3448_CONTROL_SW_RESET);
    dev->delay_ms(10);
    dev->current_bank = 0;
}

/**
 * @brief Implementation of tcs3448_set_timing()
 * 
 * @copydoc tcs3448_set_timing
 * 
 * @par Implementation Details:
 * 1. Write ATIME to register 0x81
 * 2. Write ASTEP (16-bit) to registers 0xD4/0xD5 using tcs3448_write_reg16()
 */
void tcs3448_set_timing(tcs3448_dev_t *dev, uint8_t atime, uint16_t astep) {
    tcs3448_write_reg8(dev, TCS3448_REG_ATIME, atime);
    tcs3448_write_reg16(dev, TCS3448_REG_ASTEP_L, TCS3448_REG_ASTEP_H, astep);
}

/**
 * @brief Implementation of tcs3448_set_gain()
 * 
 * @copydoc tcs3448_set_gain
 * 
 * @par Implementation Details:
 * Writes gain value (masked to 5 bits) to CFG1 register (0xC6).
 * Gain values 0-12 map to 0.5x-2048x.
 */
void tcs3448_set_gain(tcs3448_dev_t *dev, tcs3448_gain_t gain) {
    tcs3448_write_reg8(dev, TCS3448_REG_CFG1, gain & 0x1F);
}

/**
 * @brief Implementation of tcs3448_set_smux_mode()
 * 
 * @copydoc tcs3448_set_smux_mode
 * 
 * @par Implementation Details:
 * 1. Read current CFG20 value
 * 2. Clear bits [6:5] (auto_smux field)
 * 3. Set bits to mode value (shifted to position 5)
 * 4. Write back to CFG20 register (0xD6)
 */
void tcs3448_set_smux_mode(tcs3448_dev_t *dev, tcs3448_smux_mode_t mode) {
    uint8_t cfg20 = tcs3448_read_reg8(dev, TCS3448_REG_CFG20);
    cfg20 &= ~0x60;
    cfg20 |= (mode << 5);
    tcs3448_write_reg8(dev, TCS3448_REG_CFG20, cfg20);
}

/**
 * @brief Implementation of tcs3448_get_num_channels()
 * 
 * @copydoc tcs3448_get_num_channels
 * 
 * @par Implementation Details:
 * Simple switch statement mapping enum values to channel counts:
 * - TCS3448_SMUX_6CH  -> 6
 * - TCS3448_SMUX_12CH -> 12
 * - TCS3448_SMUX_18CH -> 18
 * - default           -> 6 (safe fallback)
 */
uint8_t tcs3448_get_num_channels(tcs3448_smux_mode_t mode) {
    switch (mode) {
        case TCS3448_SMUX_6CH: return 6;
        case TCS3448_SMUX_12CH: return 12;
        case TCS3448_SMUX_18CH: return 18;
        default: return 6;
    }
}

/**
 * @brief Implementation of tcs3448_start_measurement()
 * 
 * @copydoc tcs3448_start_measurement
 * 
 * @par Implementation Details:
 * Writes ENABLE register with PON | ALS_EN bits set:
 * - PON (bit 0): Powers on internal oscillator
 * - ALS_EN (bit 1): Enables ALS measurement engine
 */
void tcs3448_start_measurement(tcs3448_dev_t *dev) {
    uint8_t enable = TCS3448_ENABLE_PON | TCS3448_ENABLE_ALS_EN;
    tcs3448_write_reg8(dev, TCS3448_REG_ENABLE, enable);
}

/**
 * @brief Implementation of tcs3448_wait_for_data()
 * 
 * @copydoc tcs3448_wait_for_data
 * 
 * @par Implementation Details:
 * 1. Record start time using millis()
 * 2. Loop until timeout:
 *    - Read STATUS2 register
 *    - Check AVALID bit (bit 6)
 *    - Return true if set
 *    - Delay 1ms between polls
 * 3. Return false if timeout reached
 */
bool tcs3448_wait_for_data(tcs3448_dev_t *dev, uint32_t timeout_ms) {
    uint32_t start = millis();
    while ((millis() - start) < timeout_ms) {
        uint8_t status2 = tcs3448_read_reg8(dev, TCS3448_REG_STATUS2);
        if (status2 & TCS3448_STATUS2_AVALID) {
            return true;
        }
        dev->delay_ms(1);
    }
    return false;
}

/**
 * @brief Implementation of tcs3448_read_data()
 * 
 * @copydoc tcs3448_read_data
 * 
 * @par Implementation Details:
 * 1. Read ASTATUS register (latches all ADC values)
 * 2. Extract saturation flag (bit 7) and gain (bits 3:0)
 * 3. Loop through all 18 channels:
 *    - Calculate register address: ADATA0_L + (i * 2)
 *    - Read 16-bit value using tcs3448_read_reg16()
 *    - Store in data->channels[i]
 */
bool tcs3448_read_data(tcs3448_dev_t *dev, tcs3448_data_t *data) {
    data->astatus = tcs3448_read_reg8(dev, TCS3448_REG_ASTATUS);
    data->saturated = (data->astatus & 0x80) != 0;
    data->gain = (tcs3448_gain_t)(data->astatus & 0x0F);
    
    for (int i = 0; i < 18; i++) {
        uint8_t reg_l = TCS3448_REG_ADATA0_L + (i * 2);
        data->channels[i] = tcs3448_read_reg16(dev, reg_l, reg_l + 1);
    }
    
    return true;
}

/**
 * @brief Implementation of tcs3448_led_enable()
 * 
 * @copydoc tcs3448_led_enable
 * 
 * @par Implementation Details:
 * 1. Read current LED register value
 * 2. Set or clear LED_ACT bit (bit 7)
 * 3. Write back to LED register (0xCD)
 */
void tcs3448_led_enable(tcs3448_dev_t *dev, bool enable) {
    uint8_t led = tcs3448_read_reg8(dev, TCS3448_REG_LED);
    if (enable) {
        led |= TCS3448_LED_ACT;
    } else {
        led &= ~TCS3448_LED_ACT;
    }
    tcs3448_write_reg8(dev, TCS3448_REG_LED, led);
}

/**
 * @brief Implementation of tcs3448_led_set_current()
 * 
 * @copydoc tcs3448_led_set_current
 * 
 * @par Implementation Details:
 * 1. Clamp current_ma to valid range [4, 258]
 * 2. Calculate register value: drive = (current_ma - 4) / 2
 * 3. Read current LED register
 * 4. Preserve LED_ACT bit, set drive value in bits [6:0]
 * 5. Write back to LED register (0xCD)
 */
void tcs3448_led_set_current(tcs3448_dev_t *dev, uint8_t current_ma) {
    if (current_ma < 4) current_ma = 4;
    if (current_ma > 258) current_ma = 258;
    uint8_t drive = (current_ma - 4) / 2;
    uint8_t led = tcs3448_read_reg8(dev, TCS3448_REG_LED);
    led = (led & 0x80) | (drive & 0x7F);
    tcs3448_write_reg8(dev, TCS3448_REG_LED, led);
}

/**
 * @brief Placeholder implementation of tcs3448_fifo_enable()
 * 
 * @copydoc tcs3448_fifo_enable
 * 
 * @note This is a placeholder. Full FIFO support requires:
 * - Configuring FIFO_MAP register (0xFC)
 * - Setting FIFO threshold (CFG8 register)
 * - Potentially handling FIFO interrupts
 * 
 * Parameters are cast to void to suppress unused parameter warnings.
 */
void tcs3448_fifo_enable(tcs3448_dev_t *dev, bool enable) {
    (void)dev;
    (void)enable;
}

/**
 * @brief Implementation of tcs3448_fifo_clear()
 * 
 * @copydoc tcs3448_fifo_clear
 * 
 * @par Implementation Details:
 * Writes TCS3448_CONTROL_FIFO_CLR to CONTROL register (0xFA).
 * This bit is self-clearing.
 */
void tcs3448_fifo_clear(tcs3448_dev_t *dev) {
    tcs3448_write_reg8(dev, TCS3448_REG_CONTROL, TCS3448_CONTROL_FIFO_CLR);
}

/**
 * @brief Implementation of tcs3448_fifo_get_level()
 * 
 * @copydoc tcs3448_fifo_get_level
 * 
 * @par Implementation Details:
 * Reads FIFO_LVL register (0xFD) which contains the number of
 * entries currently stored in the FIFO buffer (0-128).
 */
uint8_t tcs3448_fifo_get_level(tcs3448_dev_t *dev) {
    return tcs3448_read_reg8(dev, TCS3448_REG_FIFO_LVL);
}

/**
 * @brief Implementation of tcs3448_fifo_read()
 * 
 * @copydoc tcs3448_fifo_read
 * 
 * @par Implementation Details:
 * Reads 'count' entries from FIFO by repeatedly reading FDATA_L (0xFE).
 * Each read pops one 16-bit entry from the FIFO.
 * 
 * @note This implementation reads from FDATA_L register using burst read,
 * which automatically provides both low and high bytes.
 */
bool tcs3448_fifo_read(tcs3448_dev_t *dev, uint16_t *data, uint8_t count) {
    for (int i = 0; i < count; i++) {
        data[i] = tcs3448_read_reg16(dev, TCS3448_REG_FDATA_L, TCS3448_REG_FDATA_L + 1);
    }
    return true;
}
