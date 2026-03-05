/**
 * @file tcs3448.h
 * @brief TCS3448 14-Channel Multi-Spectral Sensor Driver (C implementation)
 * 
 * Based on ams OSRAM TCS3448 datasheet. This driver provides a C-style API
 * for controlling the TCS3448 spectral sensor with automatic register banking,
 * 16-bit register access, and SMUX configuration support.
 * 
 * Key features:
 * - Register banking (Bank 0: 0x80-0xFF, Bank 1: 0x20-0x7F) - automatic switching
 * - 18 ADC channels with auto SMUX modes (6CH/12CH/18CH)
 * - 16-bit register access with low-byte-first requirement
 * - LED driver control (4-258mA)
 * - FIFO buffer support
 * - Retry mechanism for I2C communication (10 attempts)
 * 
 * @author TCS3448 Driver Team
 * @version 1.0.0
 * @date 2026
 * 
 * @note This driver uses function pointers for I2C communication to remain
 * platform-independent. Users must provide their own I2C read/write functions.
 * 
 * @par Example Usage:
 * @code
 * tcs3448_dev_t dev;
 * 
 * // Initialize with custom I2C functions
 * if (tcs3448_init(&dev, my_i2c_write, my_i2c_read, my_delay_ms, 0)) {
 *     tcs3448_power_on(&dev);
 *     tcs3448_set_timing(&dev, 99, 99);  // Default integration time
 *     tcs3448_set_gain(&dev, TCS3448_GAIN_8X);
 *     tcs3448_set_smux_mode(&dev, TCS3448_SMUX_18CH);
 *     
 *     tcs3448_start_measurement(&dev);
 *     if (tcs3448_wait_for_data(&dev, 100)) {
 *         tcs3448_data_t data;
 *         tcs3448_read_data(&dev, &data);
 *         // Process data.channels[0..17]
 *     }
 * }
 * @endcode
 */

#ifndef TCS3448_H
#define TCS3448_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Default 7-bit I2C slave address for TCS3448 */
#define TCS3448_I2C_ADDR                0x59

/** @brief Expected device ID from TCS3448_REG_ID register */
#define TCS3448_DEVICE_ID               0x81

/**
 * @brief I2C write function pointer type
 * 
 * Platform-independent I2C write callback. Must be provided by the user
 * to enable communication with the TCS3448 sensor.
 * 
 * @param[in] addr 7-bit I2C device address
 * @param[in] reg Register address to write to
 * @param[in] data Pointer to data buffer to write
 * @param[in] len Number of bytes to write
 * @return 0 on success, non-zero error code on failure
 * 
 * @note This function should handle the complete I2C transaction including
 * START condition, address byte, register byte, data bytes, and STOP condition.
 * 
 * @par Example Implementation (Arduino Wire):
 * @code
 * int my_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
 *     Wire.beginTransmission(addr);
 *     Wire.write(reg);
 *     for (uint16_t i = 0; i < len; i++) {
 *         Wire.write(data[i]);
 *     }
 *     return Wire.endTransmission();
 * }
 * @endcode
 */
typedef int (*tcs3448_i2c_write_fn)(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief I2C read function pointer type
 * 
 * Platform-independent I2C read callback. Must be provided by the user
 * to enable communication with the TCS3448 sensor.
 * 
 * @param[in] addr 7-bit I2C device address
 * @param[in] reg Register address to read from
 * @param[out] data Pointer to data buffer to store read data
 * @param[in] len Number of bytes to read
 * @return 0 on success, non-zero error code on failure
 * 
 * @note This function should write the register address first (with repeated START),
 * then read the requested number of bytes into the data buffer.
 * 
 * @par Example Implementation (Arduino Wire):
 * @code
 * int my_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
 *     Wire.beginTransmission(addr);
 *     Wire.write(reg);
 *     Wire.endTransmission(false);  // Repeated START
 *     
 *     Wire.requestFrom(addr, len);
 *     for (uint16_t i = 0; i < len; i++) {
 *         if (Wire.available()) {
 *             data[i] = Wire.read();
 *         } else {
 *             return 1;  // Error: not enough data
 *         }
 *     }
 *     return 0;
 * }
 * @endcode
 */
typedef int (*tcs3448_i2c_read_fn)(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Delay function pointer type
 * 
 * Platform-independent millisecond delay callback. Used for timing-sensitive
 * operations like waiting for device initialization or measurement completion.
 * 
 * @param[in] ms Number of milliseconds to delay
 * 
 * @par Example Implementation:
 * @code
 * void my_delay_ms(uint32_t ms) {
 *     delay(ms);  // Arduino delay function
 * }
 * @endcode
 */
typedef void (*tcs3448_delay_ms_fn)(uint32_t ms);

/*============================================================================
 *                         REGISTER ADDRESSES
 *============================================================================*/

/** @name Bank 0 Registers (0x80-0xFF, default, no bank switching needed)
 *  Main operational registers for sensor control and data reading.
 *  These registers are always accessible when REG_BANK bit is 0.
 *  @{ */
#define TCS3448_REG_ENABLE              0x80    /**< Main enable register (PON, ALS_EN, FDEN) */
#define TCS3448_REG_ATIME               0x81    /**< ALS integration time steps (0-255) */
#define TCS3448_REG_WTIME               0x83    /**< Wait time between measurements (0-255) */
#define TCS3448_REG_STATUS              0x93    /**< Main status register (interrupt flags) */
#define TCS3448_REG_STATUS2             0x90    /**< Status 2 (AVALID, saturation flags) */
#define TCS3448_REG_ASTATUS             0x94    /**< ADC status (must read first to latch data) */
#define TCS3448_REG_ADATA0_L            0x95    /**< ADC channel 0 data LSB (start of 18 channels) */
#define TCS3448_REG_CFG0                0xBF    /**< Configuration 0 (REG_BANK bit for switching) */
#define TCS3448_REG_CFG1                0xC6    /**< Configuration 1 (gain setting) */
#define TCS3448_REG_CFG20               0xD6    /**< Configuration 20 (auto SMUX mode selection) */
#define TCS3448_REG_ASTEP_L             0xD4    /**< Integration step size LSB (16-bit, low byte first) */
#define TCS3448_REG_ASTEP_H             0xD5    /**< Integration step size MSB */
#define TCS3448_REG_LED                 0xCD    /**< LED control register (enable + current) */
#define TCS3448_REG_CONTROL             0xFA    /**< Control register (SW reset, FIFO clear) */
#define TCS3448_REG_FIFO_LVL            0xFD    /**< FIFO level (number of entries available) */
#define TCS3448_REG_FDATA_L             0xFE    /**< FIFO data LSB (read to pop from FIFO) */
/** @} */

/** @name Bank 1 Registers (0x20-0x7F, require REG_BANK=1 in CFG0)
 *  Secondary registers for device identification and advanced configuration.
 *  Accessing these registers requires setting CFG0[4] = 1 (REG_BANK bit).
 *  The driver handles this automatically via tcs3448_set_bank().
 *  @{ */
#define TCS3448_REG_AUXID               0x58    /**< Auxiliary ID (banked) */
#define TCS3448_REG_ID                  0x5A    /**< Part number ID (banked) - should read 0x81 */
/** @} */

/*============================================================================
 *                         BIT DEFINITIONS
 *============================================================================*/

/** @name ENABLE Register (0x80) Bit Definitions
 *  Control bits for power management and measurement modes.
 *  @{ */
#define TCS3448_ENABLE_PON              (1 << 0)    /**< Power ON - enables internal oscillator */
#define TCS3448_ENABLE_ALS_EN           (1 << 1)    /**< ALS measurement enable - starts ADC conversions */
#define TCS3448_ENABLE_WEN              (1 << 3)    /**< Wait time enable - adds delay between measurements */
#define TCS3448_ENABLE_SMUXEN           (1 << 4)    /**< SMUX command enable - triggers SMUX configuration */
#define TCS3448_ENABLE_FDEN             (1 << 6)    /**< Flicker detection enable (not implemented in this driver) */
/** @} */

/** @name CFG0 Register (0xBF) Bit Definitions
 *  Configuration register 0 with bank switching control.
 *  @{ */
#define TCS3448_CFG0_REG_BANK           (1 << 4)    /**< Register bank select: 0=Bank 0 (0x80-0xFF), 1=Bank 1 (0x20-0x7F) */
#define TCS3448_CFG0_WLONG              (1 << 2)    /**< Wait time long: multiplies WTIME by 16 when set */
/** @} */

/** @name STATUS2 Register (0x90) Bit Definitions
 *  Status flags for measurement validity and saturation detection.
 *  @{ */
#define TCS3448_STATUS2_AVALID          (1 << 6)    /**< ALS measurement valid - data ready to read */
#define TCS3448_STATUS2_ASAT_ANALOG     (1 << 3)    /**< ALS analog saturation detected */
/** @} */

/** @name LED Register (0xCD) Bit Definitions
 *  LED driver control for external illumination.
 *  @{ */
#define TCS3448_LED_ACT                 (1 << 7)    /**< LED activation: 1=on, 0=off */
/** @} */

/** @name CONTROL Register (0xFA) Bit Definitions
 *  Software reset and FIFO control.
 *  @{ */
#define TCS3448_CONTROL_FIFO_CLR        (1 << 1)    /**< Clear FIFO buffer (self-clearing bit) */
#define TCS3448_CONTROL_SW_RESET        (1 << 3)    /**< Software reset - resets all registers to defaults */
/** @} */

/*============================================================================
 *                         ENUMERATIONS
 *============================================================================*/

/**
 * @brief Gain settings for ALS (Ambient Light Sensing)
 * 
 * Controls the analog gain applied to photodiode signals before ADC conversion.
 * Higher gain increases sensitivity but reduces dynamic range and may cause saturation.
 * 
 * @par Gain Values:
 * | Enum Value | Optical Gain | Use Case |
 * |------------|--------------|----------|
 * | TCS3448_GAIN_0_5X | 0.5x | Very bright light |
 * | TCS3448_GAIN_1X | 1x | Bright light |
 * | TCS3448_GAIN_2X | 2x | Normal indoor light |
 * | TCS3448_GAIN_4X | 4x | Dim light |
 * | TCS3448_GAIN_8X | 8x | Low light (recommended default) |
 * | TCS3448_GAIN_16X | 16x | Very low light |
 * | TCS3448_GAIN_32X | 32x | Dark conditions |
 * | TCS3448_GAIN_64X | 64x | Very dark |
 * | TCS3448_GAIN_128X | 128x | Extremely dark |
 * | TCS3448_GAIN_256X | 256x | Near darkness (AGC high range) |
 * | TCS3448_GAIN_512X | 512x | Near darkness |
 * | TCS3448_GAIN_1024X | 1024x | Near darkness |
 * | TCS3448_GAIN_2048X | 2048x | Pitch black |
 * 
 * @note Changing gain during active measurement is not recommended.
 * Always stop measurement (ALS_EN=0), change gain, then restart.
 * 
 * @warning High gain settings (>256x) may introduce more noise.
 * Use AGC (tcs3448_read_spectral_agc()) for automatic gain selection.
 */
typedef enum {
    TCS3448_GAIN_0_5X = 0,  /**< 0.5x gain - for very bright light */
    TCS3448_GAIN_1X,        /**< 1x gain - for bright light */
    TCS3448_GAIN_2X,        /**< 2x gain - for normal indoor light */
    TCS3448_GAIN_4X,        /**< 4x gain - for dim light */
    TCS3448_GAIN_8X,        /**< 8x gain - for low light (recommended default) */
    TCS3448_GAIN_16X,       /**< 16x gain - for very low light */
    TCS3448_GAIN_32X,       /**< 32x gain - for dark conditions */
    TCS3448_GAIN_64X,       /**< 64x gain - for very dark conditions */
    TCS3448_GAIN_128X,      /**< 128x gain - for extremely dark conditions */
    TCS3448_GAIN_256X,      /**< 256x gain - for near darkness (AGC high range) */
    TCS3448_GAIN_512X,      /**< 512x gain - for near darkness */
    TCS3448_GAIN_1024X,     /**< 1024x gain - for near darkness */
    TCS3448_GAIN_2048X      /**< 2048x gain - for pitch black conditions */
} tcs3448_gain_t;

/**
 * @brief Auto SMUX (Sensor Multiplexer) channel configuration modes
 * 
 * Controls how many spectral channels are measured in each integration cycle.
 * The SMUX connects photodiodes to ADCs in different configurations.
 * 
 * @par Channel Mapping by Mode:
 * 
 * **6CH Mode (TCS3448_SMUX_6CH):**
 * | Channel | Photodiode | Description |
 * |---------|------------|-------------|
 * | 0 | FZ | Blue (450nm) |
 * | 1 | FY | Yellow (570nm) |
 * | 2 | FXL | Extra light (550nm) |
 * | 3 | NIR | Near Infrared |
 * | 4 | VIS1 | Clear photodiode 1 |
 * | 5 | VIS2 | Clear photodiode 2 |
 * 
 * **12CH Mode (TCS3448_SMUX_12CH):**
 * Adds F3, F4, F6 to 6CH (channels 6-8)
 * 
 * **18CH Mode (TCS3448_SMUX_18CH):**
 * Adds F1, F7, F8, F5 to 12CH (channels 12-15)
 * Full spectral coverage: F1(410nm), F2(440nm), F3(470nm), F4(515nm), F5(550nm), 
 * F6(590nm), F7(630nm), F8(680nm), plus clear and NIR channels
 * 
 * @note Mode value 1 is reserved and should not be used.
 * 
 * @warning Do not change SMUX mode while ALS_EN is active. Always stop
 * measurement first, configure mode, then restart.
 */
typedef enum {
    TCS3448_SMUX_6CH = 0,   /**< 6 channels: FZ, FY, FXL, NIR, 2xVIS */
    TCS3448_SMUX_12CH = 2,  /**< 12 channels: adds F3, F4, F6 */
    TCS3448_SMUX_18CH = 3   /**< 18 channels: adds F1, F7, F8, F5 (full spectrum) */
} tcs3448_smux_mode_t;

/*============================================================================
 *                         DATA STRUCTURES
 *============================================================================*/

/**
 * @brief TCS3448 device handle structure
 * 
 * Maintains the state of a single TCS3448 sensor instance including
 * I2C communication callbacks, current register bank, and initialization status.
 * 
 * @note This structure must be initialized by tcs3448_init() before use.
 * Do not modify fields directly after initialization.
 * 
 * @par Usage:
 * @code
 * tcs3448_dev_t sensor;
 * if (tcs3448_init(&sensor, i2c_write, i2c_read, delay_ms, 0x59)) {
 *     // Device initialized successfully
 *     // Use sensor handle for all subsequent operations
 * }
 * @endcode
 */
typedef struct {
    uint8_t i2c_addr;                   /**< 7-bit I2C address (default: 0x59) */
    tcs3448_i2c_write_fn i2c_write;     /**< I2C write function pointer */
    tcs3448_i2c_read_fn i2c_read;       /**< I2C read function pointer */
    tcs3448_delay_ms_fn delay_ms;       /**< Millisecond delay function pointer */
    uint8_t current_bank;               /**< Current register bank (0 or 1) - internal use */
    bool initialized;                   /**< Initialization flag - true after successful init */
} tcs3448_dev_t;

/**
 * @brief Spectral channel indices
 * 
 * Defines the mapping between array indices and photodiode channels.
 * These constants correspond to positions in the channels[] array of
 * tcs3448_data_t structure.
 * 
 * @par Channel Details:
 * 
 * **Spectral Channels (F1-F8):**
 * | Channel | Index | Center Wavelength | Color |
 * |---------|-------|-------------------|-------|
 * | F1 | 12 | 410nm | Violet |
 * | F2 | - | 440nm | Blue |
 * | F3 | 6 | 470nm | Blue |
 * | F4 | 7 | 515nm | Cyan |
 * | F5 | 15 | 550nm | Green |
 * | F6 | 8 | 590nm | Yellow |
 * | F7 | 13 | 630nm | Orange-Red |
 * | F8 | 14 | 680nm | Red |
 * 
 * **Other Channels:**
 * | Channel | Index | Description |
 * |---------|-------|-------------|
 * | FZ | 0 | Blue (450nm) |
 * | FY | 1 | Yellow (570nm) |
 * | FXL | 2 | Extra light (550nm) |
 * | NIR | 3 | Near Infrared |
 * | VIS1 | 4 | Clear photodiode 1 |
 * | VIS2 | 5 | Clear photodiode 2 |
 * 
 * @note F2 is not directly accessible - use FZ instead for blue measurement.
 * 
 * @warning Channels 9-11 and 16-17 may contain undefined data depending on SMUX mode.
 * Always check num_channels field after reading.
 */
#define TCS3448_CH_FZ       0   /**< FZ channel - Blue (450nm) */
#define TCS3448_CH_FY       1   /**< FY channel - Yellow (570nm) */
#define TCS3448_CH_FXL      2   /**< FXL channel - Extra light (550nm) */
#define TCS3448_CH_NIR      3   /**< NIR channel - Near Infrared */
#define TCS3448_CH_VIS1     4   /**< VIS1 channel - Clear photodiode 1 */
#define TCS3448_CH_VIS2     5   /**< VIS2 channel - Clear photodiode 2 */
#define TCS3448_CH_F3       6   /**< F3 channel - Blue (470nm) */
#define TCS3448_CH_F4       7   /**< F4 channel - Cyan (515nm) */
#define TCS3448_CH_F6       8   /**< F6 channel - Yellow (590nm) */
#define TCS3448_CH_F1       12  /**< F1 channel - Violet (410nm) */
#define TCS3448_CH_F7       13  /**< F7 channel - Orange-Red (630nm) */
#define TCS3448_CH_F8       14  /**< F8 channel - Red (680nm) */
#define TCS3448_CH_F5       15  /**< F5 channel - Green (550nm) */

/**
 * @brief Spectral measurement data structure
 * 
 * Holds all data from a single spectral measurement including:
 * - Raw ADC values for all 18 channels
 * - Number of valid channels (based on SMUX mode)
 * - Status information (gain, saturation)
 * 
 * @par Data Processing:
 * Raw counts can be converted to basic counts (normalized values) using:
 * basic_count = raw_count / (gain * integration_factor)
 * where integration_factor = (ATIME + 1) * (ASTEP + 1)
 * 
 * @par Example:
 * @code
 * tcs3448_data_t data;
 * tcs3448_read_data(&dev, &data);
 * 
 * // Access specific channel
 * uint16_t blue_value = data.channels[TCS3448_CH_FZ];
 * 
 * // Check for saturation
 * if (data.saturated) {
 *     Serial.println("Warning: Sensor saturation detected");
 * }
 * @endcode
 */
typedef struct {
    uint16_t channels[18];      /**< Raw ADC values for all 18 channels */
    uint8_t num_channels;       /**< Number of valid channels (6, 12, or 18) */
    uint8_t astatus;            /**< ASTATUS register value (gain + saturation info) */
    bool saturated;             /**< Saturation flag - true if any channel saturated */
    tcs3448_gain_t gain;        /**< Gain setting used for this measurement */
} tcs3448_data_t;

/*============================================================================
 *                         PUBLIC API FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize the TCS3448 device
 * 
 * Performs device initialization by:
 * 1. Storing function pointers and I2C address
 * 2. Waiting for device startup (1ms)
 * 3. Checking device ID (should be 0x81)
 * 4. Setting initialization flag on success
 * 
 * @param[out] dev Pointer to device handle structure to initialize
 * @param[in] i2c_write I2C write function pointer (must not be NULL)
 * @param[in] i2c_read I2C read function pointer (must not be NULL)
 * @param[in] delay_ms Delay function pointer (must not be NULL)
 * @param[in] i2c_addr I2C address (use 0 for default TCS3448_I2C_ADDR=0x59)
 * @return true on successful initialization, false if device not found
 * 
 * @note All three function pointers must be valid. The device handle
 * must remain valid for the lifetime of sensor usage.
 * 
 * @warning This function does NOT power on the device. Call tcs3448_power_on()
 * after successful initialization.
 * 
 * @par Example:
 * @code
 * tcs3448_dev_t sensor;
 * if (tcs3448_init(&sensor, my_i2c_write, my_i2c_read, my_delay_ms, 0)) {
 *     Serial.println("TCS3448 found!");
 *     tcs3448_power_on(&sensor);
 * } else {
 *     Serial.println("TCS3448 not found - check wiring");
 * }
 * @endcode
 */
bool tcs3448_init(tcs3448_dev_t *dev, tcs3448_i2c_write_fn i2c_write,
                  tcs3448_i2c_read_fn i2c_read, tcs3448_delay_ms_fn delay_ms,
                  uint8_t i2c_addr);

/**
 * @brief Check if device ID matches expected value
 * 
 * Reads the device ID register from Bank 1 and verifies it matches
 * TCS3448_DEVICE_ID (0x81). Automatically handles bank switching.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @return true if device ID is correct (0x81), false otherwise
 * 
 * @note This is called automatically by tcs3448_init().
 * Can be used for periodic health checks.
 * 
 * @par Example:
 * @code
 * if (tcs3448_check_id(&sensor)) {
 *     Serial.println("Device ID verified");
 * }
 * @endcode
 */
bool tcs3448_check_id(tcs3448_dev_t *dev);

/**
 * @brief Power on the TCS3448 device
 * 
 * Enables the internal oscillator by setting PON bit in ENABLE register.
 * This transitions the device from SLEEP to IDLE state.
 * 
 * @param[in] dev Pointer to initialized device handle
 * 
 * @note After power-on, the device is in IDLE state (oscillator running,
 * timers active, but no ADC conversions). Current consumption: 40-60µA.
 * 
 * @warning Wait at least 1ms after power-on before accessing registers.
 * This function includes a 1ms delay automatically.
 * 
 * @see tcs3448_power_off()
 * 
 * @par Example:
 * @code
 * tcs3448_power_on(&sensor);
 * // Device is now in IDLE state, ready for configuration
 * @endcode
 */
void tcs3448_power_on(tcs3448_dev_t *dev);

/**
 * @brief Power off the TCS3448 device (enter sleep mode)
 * 
 * Clears all ENABLE register bits, shutting down the oscillator and
 * placing the device in SLEEP state (lowest power consumption).
 * 
 * @param[in] dev Pointer to initialized device handle
 * 
 * @note In sleep mode:
 * - Oscillator stopped
 * - No measurements possible
 * - I2C still responsive (wakes temporarily for I2C transactions)
 * - Current consumption: ~0.75µA
 * 
 * @see tcs3448_power_on()
 * 
 * @par Example:
 * @code
 * // Save power between measurements
 * tcs3448_power_off(&sensor);
 * delay(5000);  // Sleep for 5 seconds
 * tcs3448_power_on(&sensor);  // Wake up
 * @endcode
 */
void tcs3448_power_off(tcs3448_dev_t *dev);

/**
 * @brief Software reset the TCS3448 device
 * 
 * Performs a software reset by setting SW_RESET bit in CONTROL register.
 * This resets all registers to their default values and restarts
 * the device state machine.
 * 
 * @param[in] dev Pointer to initialized device handle
 * 
 * @note After reset:
 * - All registers return to defaults
 * - Bank is reset to 0 (Bank 0)
 * - Device enters SLEEP state
 * - Wait at least 10ms before accessing registers
 * 
 * @warning This function includes a 10ms delay. Any pending measurements
 * will be aborted.
 * 
 * @par Example:
 * @code
 * // Reset to recover from unknown state
 * tcs3448_reset(&sensor);
 * tcs3448_power_on(&sensor);  // Need to power on again after reset
 * @endcode
 */
void tcs3448_reset(tcs3448_dev_t *dev);

/**
 * @brief Set ALS integration time parameters
 * 
 * Configures the integration time for spectral measurements using the formula:
 * @code
 * t_int (ms) = (ATIME + 1) * (ASTEP + 1) * 0.00278
 * @endcode
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] atime Integration time steps (0-255). Number of integration cycles.
 * @param[in] astep Integration step size (0-65535). Duration of each step in 2.78µs units.
 * 
 * @note Both ATIME and ASTEP cannot be zero simultaneously.
 * Default values: ATIME=99, ASTEP=99 (recommended starting point).
 * 
 * @warning Changing timing during active measurement may cause unpredictable results.
 * Always stop measurement first.
 * 
 * @par Integration Time Examples:
 * | ATIME | ASTEP | Integration Time | Use Case |
 * |-------|-------|------------------|----------|
 * | 0 | 999 | ~2.8ms | Fast measurements |
 * | 29 | 499 | ~42ms | Balanced |
 * | 99 | 99 | ~28ms | Default |
 * | 255 | 65535 | ~47 seconds | Very low light |
 * 
 * @par Example:
 * @code
 * // Set integration time to ~28ms (default)
 * tcs3448_set_timing(&sensor, 99, 99);
 * 
 * // Set longer integration for low light
 * tcs3448_set_timing(&sensor, 29, 499);  // ~42ms
 * @endcode
 */
void tcs3448_set_timing(tcs3448_dev_t *dev, uint8_t atime, uint16_t astep);

/**
 * @brief Set ALS gain
 * 
 * Configures the analog gain applied to photodiode signals.
 * Higher gain increases sensitivity but reduces dynamic range.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] gain Gain setting from tcs3448_gain_t enumeration
 * 
 * @note The gain value is stored in the device and applied to future measurements.
 * Reading ASTATUS register returns the gain used for the current measurement.
 * 
 * @warning Changing gain during active measurement is not recommended.
 * Stop measurement, change gain, then restart.
 * 
 * @par Gain Selection Guide:
 * - Use TCS3448_GAIN_8X for general purpose (default)
 * - Use TCS3448_GAIN_256X for low light conditions
 * - Use TCS3448_GAIN_0_5X or TCS3448_GAIN_1X for very bright light
 * 
 * @par Example:
 * @code
 * // Set gain for normal indoor lighting
 * tcs3448_set_gain(&sensor, TCS3448_GAIN_8X);
 * 
 * // Set gain for low light
 * tcs3448_set_gain(&sensor, TCS3448_GAIN_256X);
 * @endcode
 */
void tcs3448_set_gain(tcs3448_dev_t *dev, tcs3448_gain_t gain);

/**
 * @brief Configure auto SMUX mode
 * 
 * Sets the automatic channel multiplexing mode which determines how many
 * spectral channels are measured in each integration cycle.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] mode SMUX mode: TCS3448_SMUX_6CH, TCS3448_SMUX_12CH, or TCS3448_SMUX_18CH
 * 
 * @note Mode is written to CFG20 register bits [6:5].
 * Mode value 1 is reserved and must not be used.
 * 
 * @warning Do not change SMUX mode while ALS_EN is active. Always stop
 * measurement first, configure mode, then restart.
 * 
 * @par Mode Selection:
 * | Mode | Channels | Update Rate | Use Case |
 * |------|----------|-------------|----------|
 * | 6CH | 6 | Fastest | Basic color sensing |
 * | 12CH | 12 | Medium | Extended spectrum |
 * | 18CH | 18 | Slowest | Full spectral analysis |
 * 
 * @par Example:
 * @code
 * // Configure for full 18-channel spectral measurement
 * tcs3448_set_smux_mode(&sensor, TCS3448_SMUX_18CH);
 * @endcode
 */
void tcs3448_set_smux_mode(tcs3448_dev_t *dev, tcs3448_smux_mode_t mode);

/**
 * @brief Get number of channels for a given SMUX mode
 * 
 * Helper function to convert SMUX mode enum to channel count.
 * 
 * @param[in] mode SMUX mode from tcs3448_smux_mode_t
 * @return Number of channels: 6, 12, or 18
 * 
 * @par Example:
 * @code
 * uint8_t num_ch = tcs3448_get_num_channels(TCS3448_SMUX_18CH);
 * Serial.print("Reading ");
 * Serial.print(num_ch);
 * Serial.println(" channels");
 * @endcode
 */
uint8_t tcs3448_get_num_channels(tcs3448_smux_mode_t mode);

/**
 * @brief Start spectral measurement
 * 
 * Enables ALS measurements by setting ALS_EN bit in ENABLE register.
 * This transitions the device from IDLE to ACTIVE state and begins
 * ADC conversions according to configured timing and SMUX mode.
 * 
 * @param[in] dev Pointer to initialized device handle
 * 
 * @note The device must be powered on (PON=1) before starting measurement.
 * Measurement continues until stopped or device powered off.
 * 
 * @warning Wait for measurement completion (AVALID=1) before reading data.
 * Use tcs3448_wait_for_data() or poll STATUS2 register.
 * 
 * @par Measurement Sequence:
 * @code
 * tcs3448_set_timing(&sensor, 99, 99);
 * tcs3448_set_gain(&sensor, TCS3448_GAIN_8X);
 * tcs3448_set_smux_mode(&sensor, TCS3448_SMUX_18CH);
 * 
 * tcs3448_start_measurement(&sensor);
 * 
 * // Wait for completion
 * if (tcs3448_wait_for_data(&sensor, 100)) {
 *     tcs3448_data_t data;
 *     tcs3448_read_data(&sensor, &data);
 * }
 * @endcode
 */
void tcs3448_start_measurement(tcs3448_dev_t *dev);

/**
 * @brief Wait for measurement data to be ready
 * 
 * Polls the STATUS2 register waiting for AVALID bit to be set,
 * indicating measurement completion.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] timeout_ms Maximum time to wait in milliseconds
 * @return true if data ready (AVALID=1), false if timeout
 * 
 * @note This is a blocking function that calls delay_ms(1) between polls.
 * For non-blocking operation, poll STATUS2 register directly.
 * 
 * @par Timeout Calculation:
 * Timeout should be at least: t_int + margin
 * where t_int = (ATIME+1)*(ASTEP+1)*2.78µs
 * Recommended: t_int + 10ms margin
 * 
 * @par Example:
 * @code
 * // Wait up to 100ms for data
 * if (tcs3448_wait_for_data(&sensor, 100)) {
 *     // Data is ready
 * } else {
 *     Serial.println("Measurement timeout!");
 * }
 * @endcode
 */
bool tcs3448_wait_for_data(tcs3448_dev_t *dev, uint32_t timeout_ms);

/**
 * @brief Read all spectral channel data
 * 
 * Reads the ASTATUS register first (to latch all ADC values), then reads
 * all 18 channels of ADC data. Results are stored in the provided structure.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[out] data Pointer to data structure to fill with measurement results
 * @return Always returns true (no error checking in current implementation)
 * 
 * @note ASTATUS must be read FIRST to latch all 16-bit ADC values consistently.
 * The driver handles this automatically.
 * 
 * @par Data Fields Set:
 * - channels[0..17]: Raw ADC values
 * - astatus: ASTATUS register (gain used + saturation info)
 * - saturated: true if any channel saturated
 * - gain: Gain setting from ASTATUS
 * - num_channels: Must be set manually based on SMUX mode
 * 
 * @par Example:
 * @code
 * tcs3448_data_t data;
 * tcs3448_read_data(&sensor, &data);
 * 
 * // Print first 6 channels
 * for (int i = 0; i < 6; i++) {
 *     Serial.print(data.channels[i]);
 *     Serial.print(" ");
 * }
 * Serial.println();
 * @endcode
 */
bool tcs3448_read_data(tcs3448_dev_t *dev, tcs3448_data_t *data);

/**
 * @brief Enable or disable LED driver
 * 
 * Controls the LDR pin which acts as a current sink for external LEDs.
 * The LED current must be set separately using tcs3448_led_set_current().
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] enable true to turn LED on, false to turn off
 * 
 * @note LED current range: 4mA to 258mA (set via tcs3448_led_set_current())
 * LDR pin is open-drain with current sink capability.
 * 
 * @warning Ensure proper current limiting resistor is used with external LED.
 * Maximum continuous current is 258mA.
 * 
 * @par Example:
 * @code
 * // Set LED current to 20mA and turn on
 * tcs3448_led_set_current(&sensor, 20);
 * tcs3448_led_enable(&sensor, true);
 * delay(100);  // Illuminate for 100ms
 * tcs3448_led_enable(&sensor, false);
 * @endcode
 */
void tcs3448_led_enable(tcs3448_dev_t *dev, bool enable);

/**
 * @brief Set LED drive current
 * 
 * Configures the current sink value for the external LED.
 * Current is calculated as: I = 4mA + (drive * 2mA)
 * where drive is the 7-bit value written to LED register.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] current_ma Desired LED current in milliamps (4-258)
 * 
 * @note Current is automatically clamped to valid range:
 * - Minimum: 4mA
 * - Maximum: 258mA
 * - Step size: 2mA
 * 
 * @par Current Calculation:
 * | Register Value | Current |
 * |----------------|---------|
 * | 0x00 | 4mA |
 * | 0x01 | 6mA |
 * | 0x02 | 8mA |
 * | ... | ... |
 * | 0x7F | 258mA |
 * 
 * @warning LED is not automatically turned on. Call tcs3448_led_enable()
 * after setting current.
 * 
 * @par Example:
 * @code
 * // Set current to 50mA
 * tcs3448_led_set_current(&sensor, 50);
 * 
 * // Set current to maximum (258mA)
 * tcs3448_led_set_current(&sensor, 258);
 * @endcode
 */
void tcs3448_led_set_current(tcs3448_dev_t *dev, uint8_t current_ma);

/**
 * @brief Enable or disable FIFO mode
 * 
 * Placeholder function for FIFO configuration. Currently not fully implemented.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] enable true to enable FIFO, false to disable
 * 
 * @note This is a placeholder. Full FIFO support requires:
 * - Configuring FIFO_MAP register to select which channels go to FIFO
 * - Setting FIFO threshold (CFG8)
 * - Handling FIFO interrupts
 * 
 * @warning FIFO mode is not fully implemented in this version of the driver.
 * Use polling mode (tcs3448_read_data) for reliable operation.
 */
void tcs3448_fifo_enable(tcs3448_dev_t *dev, bool enable);

/**
 * @brief Clear FIFO buffer
 * 
 * Clears all data from the FIFO buffer by setting FIFO_CLR bit in
 * CONTROL register. This bit is self-clearing.
 * 
 * @param[in] dev Pointer to initialized device handle
 * 
 * @note This should be called before starting measurements in FIFO mode
 * to ensure no stale data is present.
 * 
 * @par Example:
 * @code
 * tcs3448_fifo_clear(&sensor);
 * tcs3448_start_measurement(&sensor);
 * @endcode
 */
void tcs3448_fifo_clear(tcs3448_dev_t *dev);

/**
 * @brief Get current FIFO level
 * 
 * Reads FIFO_LVL register to determine how many data entries are
 * currently stored in the FIFO buffer.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @return Number of entries in FIFO (0-128)
 * 
 * @note FIFO size is 256 bytes = 128 entries (each entry is 2 bytes).
 * FIFO level indicates how many entries are waiting to be read.
 * 
 * @par Example:
 * @code
 * uint8_t level = tcs3448_fifo_get_level(&sensor);
 * if (level >= 10) {
 *     // Read 10 entries from FIFO
 * }
 * @endcode
 */
uint8_t tcs3448_fifo_get_level(tcs3448_dev_t *dev);

/**
 * @brief Read data from FIFO
 * 
 * Reads specified number of entries from the FIFO buffer.
 * Each entry is 2 bytes (16-bit value).
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[out] data Array to store read values
 * @param[in] count Number of entries to read
 * @return Always returns true
 * 
 * @note Reading from FDATA_L pops one entry from FIFO (FIFO level decrements).
 * This function reads count * 2 bytes from the FIFO.
 * 
 * @warning Ensure count does not exceed current FIFO level to avoid reading
 * invalid data. Check FIFO level first with tcs3448_fifo_get_level().
 * 
 * @par Example:
 * @code
 * uint16_t fifo_data[10];
 * uint8_t level = tcs3448_fifo_get_level(&sensor);
 * if (level >= 10) {
 *     tcs3448_fifo_read(&sensor, fifo_data, 10);
 * }
 * @endcode
 */
bool tcs3448_fifo_read(tcs3448_dev_t *dev, uint16_t *data, uint8_t count);

/**
 * @brief Read 8-bit register value
 * 
 * Low-level function to read a single byte from a device register.
 * Automatically handles register bank switching for Bank 1 registers.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] reg Register address to read from
 * @return Register value (0-255)
 * 
 * @note For Bank 1 registers (0x20-0x7F), this function automatically:
 * 1. Sets REG_BANK bit in CFG0
 * 2. Reads the register
 * 3. Restores previous bank setting
 * 
 * @warning Bank switching has overhead. For multiple consecutive reads from
 * Bank 1, consider using direct register access with manual bank management.
 * 
 * @par Example:
 * @code
 * // Read ENABLE register (Bank 0)
 * uint8_t enable = tcs3448_read_reg8(&sensor, TCS3448_REG_ENABLE);
 * 
 * // Read device ID (Bank 1 - automatic switching)
 * uint8_t id = tcs3448_read_reg8(&sensor, TCS3448_REG_ID);
 * @endcode
 */
uint8_t tcs3448_read_reg8(tcs3448_dev_t *dev, uint8_t reg);

/**
 * @brief Write 8-bit register value
 * 
 * Low-level function to write a single byte to a device register.
 * Automatically handles register bank switching for Bank 1 registers.
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] reg Register address to write to
 * @param[in] value Value to write (0-255)
 * 
 * @note For Bank 1 registers (0x20-0x7F), this function automatically:
 * 1. Sets REG_BANK bit in CFG0
 * 2. Writes the register
 * 3. Restores previous bank setting
 * 
 * @warning Bank switching has overhead. For multiple consecutive writes to
 * Bank 1, consider using direct register access with manual bank management.
 * 
 * @par Example:
 * @code
 * // Write to ENABLE register (Bank 0)
 * tcs3448_write_reg8(&sensor, TCS3448_REG_ENABLE, TCS3448_ENABLE_PON);
 * @endcode
 */
void tcs3448_write_reg8(tcs3448_dev_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Read 16-bit register value
 * 
 * Low-level function to read a 16-bit value from two consecutive registers.
 * Reads low byte first, then high byte (low-byte-first requirement).
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] reg_l Register address for low byte (LSB)
 * @param[in] reg_h Register address for high byte (MSB)
 * @return 16-bit register value
 * 
 * @note TCS3448 requires reading low byte first to latch the entire 16-bit value.
 * This is critical for consistent multi-byte register reads.
 * 
 * @note This function performs a 2-byte I2C read from reg_l address with
 * automatic register increment. This is more efficient than two separate
 * 1-byte reads.
 * 
 * @par Example:
 * @code
 * // Read ASTEP register (16-bit at 0xD4/0xD5)
 * uint16_t astep = tcs3448_read_reg16(&sensor, TCS3448_REG_ASTEP_L, TCS3448_REG_ASTEP_H);
 * @endcode
 */
uint16_t tcs3448_read_reg16(tcs3448_dev_t *dev, uint8_t reg_l, uint8_t reg_h);

/**
 * @brief Write 16-bit register value
 * 
 * Low-level function to write a 16-bit value to two consecutive registers.
 * Writes low byte first, then high byte (low-byte-first requirement).
 * 
 * @param[in] dev Pointer to initialized device handle
 * @param[in] reg_l Register address for low byte (LSB)
 * @param[in] reg_h Register address for high byte (MSB) - unused in current implementation
 * @param[in] value 16-bit value to write
 * 
 * @note TCS3448 requires writing low byte first. Writing only one byte of
 * a 16-bit register will corrupt the register value.
 * 
 * @note This function performs a 2-byte I2C write to reg_l address with
 * automatic register increment. The reg_h parameter is kept for API consistency
 * but the actual register increment handles the high byte.
 * 
 * @par Example:
 * @code
 * // Write ASTEP register (16-bit at 0xD4/0xD5)
 * tcs3448_write_reg16(&sensor, TCS3448_REG_ASTEP_L, TCS3448_REG_ASTEP_H, 999);
 * @endcode
 */
void tcs3448_write_reg16(tcs3448_dev_t *dev, uint8_t reg_l, uint8_t reg_h, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* TCS3448_H */
