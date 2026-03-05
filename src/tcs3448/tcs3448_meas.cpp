/**
 * @file tcs3448_meas.cpp
 * @brief High-level measurement interface implementation for Arduino
 * 
 * Implementation of the Arduino-friendly high-level API for TCS3448.
 * This layer provides:
 * - Arduino Wire library integration as default I2C transport
 * - Convenient single-call measurement functions
 * - Automatic Gain Control (AGC) algorithm
 * - LED control wrapper functions
 * - Data formatting and conversion utilities
 * 
 * @par Design Philosophy:
 * This layer abstracts the low-level driver (tcs3448.cpp) to provide
 * a simple, Arduino-idiomatic API. Users don't need to manage device
 * handles or function pointers - everything is handled internally.
 * 
 * @par Global State:
 * This implementation uses a static global device handle (tcs3448_dev)
 * and initialization flag (tcs3448_initialized). This simplifies the API
 * but means only one TCS3448 sensor is supported per program. For multiple
 * sensors, use the low-level API directly.
 * 
 * @par Default I2C Implementation:
 * Uses Arduino Wire library for I2C communication:
 * - Write: Wire.beginTransmission() + Wire.write() + Wire.endTransmission()
 * - Read: Wire.requestFrom() + Wire.read()
 * - Delay: Arduino delay() function
 * 
 * @author TCS3448 Driver Team
 * @version 1.0.0
 * @date 2026
 * 
 * @see tcs3448_meas.h for API documentation
 * @see tcs3448.cpp for low-level implementation
 */

#include "tcs3448_meas.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief Global device handle for the TCS3448 sensor
 * 
 * Static instance used by all high-level functions. Initialized by
 * tcs3448_meas_init() and used internally by all other API functions.
 * 
 * @note This limits the high-level API to a single sensor instance.
 * For multiple sensors, use the low-level tcs3448_ API directly.
 */
static tcs3448_dev_t tcs3448_dev;

/**
 * @brief Initialization flag
 * 
 * Set to true after successful initialization by tcs3448_meas_init().
 * Checked by other functions to prevent operations on uninitialized device.
 */
static bool tcs3448_initialized = false;

/**
 * @brief Gain value lookup table
 * 
 * Maps tcs3448_gain_t enum values to actual gain multipliers.
 * Used for converting raw counts to basic counts.
 * 
 * @par Gain Mapping:
 * | Index | Gain Value |
 * |-------|------------|
 * | 0 (0.5X) | 1 |
 * | 1 (1X) | 2 |
 * | 2 (2X) | 4 |
 * | 3 (4X) | 8 |
 * | 4 (8X) | 16 |
 * | 5 (16X) | 32 |
 * | 6 (32X) | 64 |
 * | 7 (64X) | 128 |
 * | 8 (128X) | 256 |
 * | 9 (256X) | 512 |
 * | 10 (512X) | 1024 |
 * | 11 (1024X) | 1024 |
 * | 12 (2048X) | 2048 |
 * 
 * @note Array index corresponds to tcs3448_gain_t enum values.
 */
static const uint16_t gain_values[] = {
    1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
};

/**
 * @brief Arduino Wire library I2C write wrapper
 * 
 * Implements the tcs3448_i2c_write_fn interface using Arduino Wire library.
 * This is the default I2C write function used when NULL is passed to
 * tcs3448_meas_init().
 * 
 * @param[in] addr 7-bit I2C device address
 * @param[in] reg Register address to write to
 * @param[in] data Pointer to data buffer to write
 * @param[in] len Number of bytes to write
 * @return 0 on success, non-zero on failure (Wire library error code)
 * 
 * @par Implementation:
 * 1. Wire.beginTransmission(addr) - Start I2C transaction
 * 2. Wire.write(reg) - Send register address
 * 3. Wire.write(data[i]) for each byte - Send data
 * 4. Wire.endTransmission() - Complete transaction and return status
 * 
 * @note This function is registered as the I2C write callback when
 * using default initialization (NULL function pointers).
 */
static int tcs3448_wire_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission();
}

/**
 * @brief Arduino Wire library I2C read wrapper
 * 
 * Implements the tcs3448_i2c_read_fn interface using Arduino Wire library.
 * This is the default I2C read function used when NULL is passed to
 * tcs3448_meas_init().
 * 
 * @param[in] addr 7-bit I2C device address
 * @param[in] reg Register address to read from
 * @param[out] data Pointer to data buffer to store read data
 * @param[in] len Number of bytes to read
 * @return 0 on success, 1 on failure
 * 
 * @par Implementation:
 * 1. Wire.beginTransmission(addr) + Wire.write(reg) - Send register address
 * 2. Wire.endTransmission(false) - Repeated START to keep bus active
 * 3. Wire.requestFrom(addr, len) - Request data from device
 * 4. Wire.read() for each byte - Read data into buffer
 * 
 * @note Returns 1 (error) if fewer bytes available than requested.
 * 
 * @note This function is registered as the I2C read callback when
 * using default initialization (NULL function pointers).
 */
static int tcs3448_wire_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 1;
    
    Wire.requestFrom(addr, len);
    for (uint16_t i = 0; i < len; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        } else {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Arduino delay() wrapper
 * 
 * Implements the tcs3448_delay_ms_fn interface using Arduino delay() function.
 * This is the default delay function used when NULL is passed to
 * tcs3448_meas_init().
 * 
 * @param[in] ms Number of milliseconds to delay
 * 
 * @note Simply calls Arduino delay(ms).
 * This function is registered as the delay callback when
 * using default initialization (NULL function pointers).
 */
static void tcs3448_delay_wrapper(uint32_t ms) {
    delay(ms);
}

/**
 * @brief Implementation of tcs3448_meas_init()
 * 
 * @copydoc tcs3448_meas_init
 * 
 * @par Initialization Sequence:
 * 1. If i2c_write is NULL, use tcs3448_wire_write (Arduino Wire)
 * 2. If i2c_read is NULL, use tcs3448_wire_read (Arduino Wire)
 * 3. If delay_ms is NULL, use tcs3448_delay_wrapper (Arduino delay)
 * 4. Call tcs3448_init() with function pointers and address 0 (default)
 * 5. Power on the device with tcs3448_power_on()
 * 6. Set default timing with tcs3448_set_timing()
 * 7. Set tcs3448_initialized = true on success
 */
bool tcs3448_meas_init(tcs3448_i2c_write_fn i2c_write,
                       tcs3448_i2c_read_fn i2c_read,
                       tcs3448_delay_ms_fn delay_ms) {
    if (i2c_write == NULL) i2c_write = tcs3448_wire_write;
    if (i2c_read == NULL) i2c_read = tcs3448_wire_read;
    if (delay_ms == NULL) delay_ms = tcs3448_delay_wrapper;

    
    if (!tcs3448_init(&tcs3448_dev, i2c_write, i2c_read, delay_ms, 0)) {
        return false;
    }

    tcs3448_power_on(&tcs3448_dev);


    tcs3448_set_timing(&tcs3448_dev, TCS3448_DEFAULT_ATIME, TCS3448_DEFAULT_ASTEP);

    tcs3448_set_smux_mode(&tcs3448_dev, TCS3448_SMUX_18CH);
    
    tcs3448_initialized = true;
    return true;
}

/**
 * @brief Implementation of tcs3448_read_spectral()
 * 
 * @copydoc tcs3448_read_spectral
 * 
 * @par Measurement Sequence:
 * 1. Check initialization flag, print error if not initialized
 * 2. Configure SMUX mode with tcs3448_set_smux_mode()
 * 3. Set gain with tcs3448_set_gain()
 * 4. Calculate number of channels based on mode
 * 5. Start measurement with tcs3448_start_measurement()
 * 6. Calculate timeout: integration_time + 10ms margin
 * 7. Wait for data with tcs3448_wait_for_data()
 * 8. Read data with tcs3448_read_data()
 * 
 * @par Timeout Calculation:
 * With default ATIME=99, ASTEP=99:
 * integration_ms = (99+1) * (99+1) * 3 / 1000 + 10 = ~38ms
 * 
 * @note This is a blocking function. Returns false on timeout.
 */
bool tcs3448_read_spectral(tcs3448_data_t *data, tcs3448_gain_t gain, 
                           tcs3448_smux_mode_t mode) {
    if (!tcs3448_initialized) {
        Serial.println("TCS3448 not initialized");
        return false;
    }
    
    tcs3448_set_smux_mode(&tcs3448_dev, mode);
    tcs3448_set_gain(&tcs3448_dev, gain);
    data->num_channels = tcs3448_get_num_channels(mode);
    
    tcs3448_start_measurement(&tcs3448_dev);
    
    uint32_t integration_ms = (uint32_t)(TCS3448_DEFAULT_ATIME + 1) * 
                              (TCS3448_DEFAULT_ASTEP + 1) * 3 / 1000 + 1000;
    
    if (!tcs3448_wait_for_data(&tcs3448_dev, integration_ms)) {
        Serial.println("TCS3448: Timeout waiting for data");
        return false;
    }
    
    return tcs3448_read_data(&tcs3448_dev, data);
}

/**
 * @brief Implementation of tcs3448_read_spectral_agc()
 * 
 * @copydoc tcs3448_read_spectral_agc
 * 
 * @par AGC Algorithm:
 * 1. Measure at TCS3448_GAIN_8X (low gain, good dynamic range)
 * 2. Find maximum value across all channels
 * 3. If max < 1000 (low signal):
 *    a. Measure again at TCS3448_GAIN_256X (high gain)
 *    b. Scale results down by factor of 32 (256/8)
 *    c. Return scaled high-gain data
 * 4. If max >= 1000 (good signal):
 *    a. Return low-gain data directly
 * 
 * @par Scaling Factor:
 * The 32:1 scale factor converts 256X data to equivalent 8X values:
 * - Allows direct comparison between AGC and non-AGC measurements
 * - Prevents overflow when using high gain
 * - Maintains consistent data range regardless of gain used
 * 
 * @note Threshold of 1000 is arbitrary and may need adjustment based on
 * specific application requirements.
 */
bool tcs3448_read_spectral_agc(tcs3448_data_t *data, tcs3448_smux_mode_t mode) {
    tcs3448_data_t low_gain_data, high_gain_data;
    
    if (!tcs3448_read_spectral(&low_gain_data, TCS3448_GAIN_8X, mode)) {
        return false;
    }
    
    uint16_t max_val = 0;
    for (int i = 0; i < low_gain_data.num_channels; i++) {
        if (low_gain_data.channels[i] > max_val) {
            max_val = low_gain_data.channels[i];
        }
    }
    
    if (max_val < 1000) {
        if (!tcs3448_read_spectral(&high_gain_data, TCS3448_GAIN_256X, mode)) {
            return false;
        }
        
        data->num_channels = high_gain_data.num_channels;
        data->astatus = high_gain_data.astatus;
        data->saturated = high_gain_data.saturated;
        data->gain = TCS3448_GAIN_256X;
        
        float scale = 32.0f;
        for (int i = 0; i < data->num_channels; i++) {
            data->channels[i] = (uint16_t)(high_gain_data.channels[i] / scale);
        }
    } else {
        *data = low_gain_data;
    }
    
    return true;
}

/**
 * @brief Implementation of tcs3448_fifo_start()
 * 
 * @copydoc tcs3448_fifo_start
 * 
 * @par Initialization Sequence:
 * 1. Check initialization flag, return silently if not initialized
 * 2. Clear FIFO buffer with tcs3448_fifo_clear()
 * 3. Configure SMUX mode with tcs3448_set_smux_mode()
 * 4. Set gain with tcs3448_set_gain()
 * 5. Start continuous measurement with tcs3448_start_measurement()
 * 
 * @note After calling this function, the sensor continuously measures
 * and stores results in FIFO. Use tcs3448_fifo_read_batch() to retrieve
 * accumulated samples.
 * 
 * @warning Full FIFO support requires proper FIFO_MAP configuration
 * which is not fully implemented in this version.
 */
void tcs3448_fifo_start(tcs3448_smux_mode_t mode, tcs3448_gain_t gain) {
    if (!tcs3448_initialized) return;
    
    tcs3448_fifo_clear(&tcs3448_dev);
    tcs3448_set_smux_mode(&tcs3448_dev, mode);
    tcs3448_set_gain(&tcs3448_dev, gain);
    tcs3448_start_measurement(&tcs3448_dev);
}

/**
 * @brief Implementation of tcs3448_fifo_read_batch()
 * 
 * @copydoc tcs3448_fifo_read_batch
 * 
 * @par Reading Procedure:
 * 1. Check current FIFO level with tcs3448_fifo_get_level()
 * 2. Determine number to read: min(level, max_samples)
 * 3. For each sample:
 *    a. Read 18 channels from FIFO using tcs3448_read_reg16()
 *    b. Store in data[s].channels[]
 * 4. Return number of samples actually read
 * 
 * @note This implementation reads 18 channels for each sample regardless
 * of SMUX mode. Unused channels will contain undefined data.
 * 
 * @warning This is a placeholder implementation. Full FIFO support
 * requires proper FIFO_MAP configuration.
 */
uint8_t tcs3448_fifo_read_batch(tcs3448_data_t *data, uint8_t max_samples) {
    uint8_t level = tcs3448_fifo_get_level(&tcs3448_dev);
    uint8_t to_read = (level < max_samples) ? level : max_samples;
    
    for (uint8_t s = 0; s < to_read; s++) {
        for (int ch = 0; ch < 18; ch++) {
            data[s].channels[ch] = tcs3448_read_reg16(&tcs3448_dev, 
                TCS3448_REG_FDATA_L, TCS3448_REG_FDATA_L + 1);
        }
    }
    
    return to_read;
}

/**
 * @brief Implementation of tcs3448_led_on()
 * 
 * @copydoc tcs3448_led_on
 * 
 * @note Silently does nothing if not initialized.
 * Uses tcs3448_led_enable() from low-level API.
 */
void tcs3448_led_on(void) {
    if (tcs3448_initialized) {
        tcs3448_led_enable(&tcs3448_dev, true);
    }
}

/**
 * @brief Implementation of tcs3448_led_off()
 * 
 * @copydoc tcs3448_led_off
 * 
 * @note Silently does nothing if not initialized.
 * Uses tcs3448_led_enable() from low-level API.
 */
void tcs3448_led_off(void) {
    if (tcs3448_initialized) {
        tcs3448_led_enable(&tcs3448_dev, false);
    }
}

/**
 * @brief Implementation of tcs3448_led_current()
 * 
 * @copydoc tcs3448_led_current
 * 
 * @note Silently does nothing if not initialized.
 * Uses tcs3448_led_set_current() from low-level API.
 */
void tcs3448_led_current(uint8_t current_ma) {
    if (tcs3448_initialized) {
        tcs3448_led_set_current(&tcs3448_dev, current_ma);
    }
}

/**
 * @brief Implementation of tcs3448_counts_to_basic()
 * 
 * @copydoc tcs3448_counts_to_basic
 * 
 * @par Calculation:
 * @code
 * basic_count = raw_count / (gain * integration_factor)
 * where:
 *   gain = gain_values[gain_enum] (lookup from static table)
 *   integration_factor = (ATIME + 1) * (ASTEP + 1)
 * @endcode
 * 
 * @par Example:
 * For raw_count=5000, gain=8X (16), ATIME=99, ASTEP=99:
 * integration_factor = 100 * 100 = 10000
 * basic_count = 5000 / (16 * 10000) = 0.03125
 * 
 * @note Basic counts represent normalized light intensity independent
 * of measurement settings, allowing comparison across different configurations.
 */
float tcs3448_counts_to_basic(uint16_t counts, tcs3448_gain_t gain, 
                               uint8_t atime, uint16_t astep) {
    float integration_factor = (atime + 1) * (astep + 1);
    float gain_val = (float)gain_values[gain];
    return (float)counts / (gain_val * integration_factor);
}

/**
 * @brief Implementation of tcs3448_print_data()
 * 
 * @copydoc tcs3448_print_data
 * 
 * @par Output Format:
 * @code
 * Channels: ch0, ch1, ch2, ..., chN
 * WARNING: Saturation detected (if applicable)
 * @endcode
 * 
 * @par Example Output:
 * @code
 * Channels: 1234, 5678, 9012, 1111, 2222, 3333
 * @endcode
 * 
 * @note Requires Serial to be initialized (Serial.begin()) before use.
 * Only prints num_channels values (6, 12, or 18 based on SMUX mode).
 */
void tcs3448_print_data(const tcs3448_data_t *data) {
    Serial.print("Channels: ");
    for (int i = 0; i < data->num_channels; i++) {
        Serial.print(data->channels[i]);
        if (i < data->num_channels - 1) Serial.print(", ");
    }
    Serial.println();
    
    if (data->saturated) {
        Serial.println("WARNING: Saturation detected");
    }
}



bool tcs3448_read(tcs3448_data_t *data, tcs3448_gain_t gain, 
                           tcs3448_smux_mode_t mode) {
    if (!tcs3448_initialized) {
        Serial.println("TCS3448 not initialized");
        return false;
    }
    
    tcs3448_set_gain(&tcs3448_dev, gain);
    data->num_channels = tcs3448_get_num_channels(mode);
    
    tcs3448_start_measurement(&tcs3448_dev);
    
    uint32_t integration_ms = (uint32_t)(TCS3448_DEFAULT_ATIME + 1) * 
                              (TCS3448_DEFAULT_ASTEP + 1) * 3 / 1000 + 1000;
    
    if (!tcs3448_wait_for_data(&tcs3448_dev, integration_ms)) {
        Serial.println("TCS3448: Timeout waiting for data");
        return false;
    }
    
    return tcs3448_read_data(&tcs3448_dev, data);
}