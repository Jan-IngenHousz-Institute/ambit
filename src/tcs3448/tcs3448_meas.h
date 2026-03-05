/**
 * @file tcs3448_meas.h
 * @brief High-level measurement interface for TCS3448 spectral sensor
 * 
 * Provides simplified Arduino-friendly API for the TCS3448 driver.
 * This layer handles initialization with Arduino Wire library, automatic
 * gain control (AGC), and convenient measurement functions.
 * 
 * @par Key Features:
 * - Arduino Wire library integration (default I2C functions)
 * - Automatic device initialization with default settings
 * - Single-call spectral measurement with configurable gain
 * - Automatic Gain Control (AGC) for varying light conditions
 * - LED control convenience functions
 * - Data conversion and printing utilities
 * 
 * @par Default Settings:
 * - I2C Address: 0x59
 * - Integration Time: ATIME=99, ASTEP=99 (~28ms)
 * - Gain: TCS3448_GAIN_8X (default for single measurement)
 * - SMUX Mode: User configurable
 * 
 * @author TCS3448 Driver Team
 * @version 1.0.0
 * @date 2026
 * 
 * @note This is the recommended API for Arduino users. For advanced use cases
 * or custom I2C implementations, use the low-level tcs3448.h API directly.
 * 
 * @par Quick Start Example:
 * @code
 * #include "tcs3448_meas.h"
 * 
 * void setup() {
 *     Serial.begin(115200);
 *     Wire.begin();
 *     
 *     // Initialize with default settings
 *     if (tcs3448_meas_init(NULL, NULL, NULL)) {
 *         Serial.println("TCS3448 initialized!");
 *     } else {
 *         Serial.println("TCS3448 not found!");
 *         while(1);  // Halt on error
 *     }
 * }
 * 
 * void loop() {
 *     tcs3448_data_t data;
 *     
 *     // Read with automatic gain control
 *     if (tcs3448_read_spectral_agc(&data, TCS3448_SMUX_18CH)) {
 *         tcs3448_print_data(&data);
 *     }
 *     
 *     delay(1000);  // Wait 1 second between measurements
 * }
 * @endcode
 * 
 * @see tcs3448.h for low-level API and register definitions
 */

#ifndef TCS3448_MEAS_H
#define TCS3448_MEAS_H

#include "tcs3448.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                         DEFAULT CONFIGURATION
 *============================================================================*/

/**
 * @brief Default ATIME value for integration timing
 * 
 * Number of integration cycles. Used with ASTEP to calculate total
 * integration time: t_int = (ATIME + 1) * (ASTEP + 1) * 2.78µs
 * 
 * @par Default Value: 99
 * @par Integration Time with ASTEP=99: ~28ms
 */
#define TCS3448_DEFAULT_ATIME   99

/**
 * @brief Default ASTEP value for integration timing
 * 
 * Duration of each integration step in 2.78µs units.
 * Used with ATIME to calculate total integration time.
 * 
 * @par Default Value: 99
 * @par Integration Time with ATIME=99: ~28ms
 * 
 * @note ASTEP is a 16-bit value stored in two registers (ASTEP_L and ASTEP_H)
 */
#define TCS3448_DEFAULT_ASTEP   99

/*============================================================================
 *                         HIGH-LEVEL API FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize TCS3448 with high-level Arduino interface
 * 
 * Performs complete sensor initialization with default settings:
 * 1. Uses Arduino Wire library for I2C (if NULL function pointers provided)
 * 2. Checks device ID
 * 3. Powers on the device
 * 4. Sets default timing (ATIME=99, ASTEP=99)
 * 5. Configures delay function
 * 
 * @param[in] i2c_write Custom I2C write function (NULL to use Arduino Wire)
 * @param[in] i2c_read Custom I2C read function (NULL to use Arduino Wire)
 * @param[in] delay_ms Custom delay function (NULL to use Arduino delay)
 * @return true on successful initialization, false on failure
 * 
 * @note All three parameters can be NULL to use default Arduino implementations:
 * - I2C: Arduino Wire library (Wire.write, Wire.read)
 * - Delay: Arduino delay() function
 * 
 * @warning This function must be called before any other tcs3448_meas_* functions.
 * Only call once at startup.
 * 
 * @par Example - Default Arduino Setup:
 * @code
 * void setup() {
 *     Wire.begin();
 *     if (tcs3448_meas_init(NULL, NULL, NULL)) {
 *         Serial.println("Ready!");
 *     }
 * }
 * @endcode
 * 
 * @par Example - Custom I2C Functions:
 * @code
 * int my_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
 *     // Custom I2C implementation
 *     return 0;  // Success
 }
 * 
 * void setup() {
 *     if (tcs3448_meas_init(my_i2c_write, my_i2c_read, NULL)) {
 *         Serial.println("Ready with custom I2C!");
 *     }
 * }
 * @endcode
 */
bool tcs3448_meas_init(tcs3448_i2c_write_fn i2c_write,
                       tcs3448_i2c_read_fn i2c_read,
                       tcs3448_delay_ms_fn delay_ms);

/**
 * @brief Read spectral data with specified gain and SMUX mode
 * 
 * Performs a complete single measurement cycle:
 * 1. Configures SMUX mode
 * 2. Sets gain
 * 3. Starts measurement
 * 4. Waits for data ready (with timeout)
 * 5. Reads all channels
 * 
 * This is a blocking function that waits for measurement completion.
 * 
 * @param[out] data Pointer to data structure to store results
 * @param[in] gain Gain setting (e.g., TCS3448_GAIN_8X)
 * @param[in] mode SMUX mode (TCS3448_SMUX_6CH, 12CH, or 18CH)
 * @return true on success, false on timeout or error
 * 
 * @note Timeout is calculated automatically based on integration time:
 * timeout = integration_time + 10ms margin
 * Default timeout with ATIME=99, ASTEP=99: ~38ms
 * 
 * @warning This function blocks until measurement completes or timeout occurs.
 * For non-blocking operation, use low-level API directly.
 * 
 * @par Example - Single Measurement:
 * @code
 * tcs3448_data_t data;
 * if (tcs3448_read_spectral(&data, TCS3448_GAIN_8X, TCS3448_SMUX_18CH)) {
 *     // Access channel data
 *     uint16_t blue = data.channels[TCS3448_CH_FZ];
 *     uint16_t red = data.channels[TCS3448_CH_F8];
 * }
 * @endcode
 * 
 * @see tcs3448_read_spectral_agc() for automatic gain selection
 */
bool tcs3448_read_spectral(tcs3448_data_t *data, tcs3448_gain_t gain, 
                           tcs3448_smux_mode_t mode);

/**
 * @brief Read spectral data with Automatic Gain Control (AGC)
 * 
 * Automatically selects optimal gain for current lighting conditions
 * using a dual-exposure algorithm:
 * 
 * 1. First exposure at TCS3448_GAIN_8X (low gain)
 * 2. Check if maximum value < 1000 (arbitrary threshold)
 * 3. If low signal, take second exposure at TCS3448_GAIN_256X (high gain)
 * 4. Scale high gain data down by factor of 32 for consistency
 * 
 * @param[out] data Pointer to data structure to store results
 * @param[in] mode SMUX mode (TCS3448_SMUX_6CH, 12CH, or 18CH)
 * @return true on success, false on error
 * 
 * @note AGC provides good results across varying light conditions:
 * - Bright light: Uses 8X gain (good dynamic range)
 * - Low light: Uses 256X gain (high sensitivity)
 * 
 * @note The returned data is always normalized as if measured at 8X gain.
 * Scaling factor: 256X / 8X = 32
 * 
 * @warning This function performs 1 or 2 measurements, so it takes 1-2x
 * the integration time plus overhead.
 * 
 * @par Example - Automatic Gain Control:
 * @code
 * tcs3448_data_t data;
 * // Automatically selects best gain
 * if (tcs3448_read_spectral_agc(&data, TCS3448_SMUX_18CH)) {
 *     // data contains optimally exposed values
 *     // data.gain indicates which gain was used (8X or 256X)
 * }
 * @endcode
 * 
 * @par Algorithm Details:
 * @code
 * if (max_value < 1000) {
 *     // Low signal detected, use high gain
 *     measure at 256X
 *     scale_results by 1/32
 *     return scaled_data
 * } else {
 *     // Good signal, use low gain
 *     return 8X_data
 * }
 * @endcode
 */
bool tcs3448_read_spectral_agc(tcs3448_data_t *data, tcs3448_smux_mode_t mode);

/**
 * @brief Start continuous FIFO acquisition
 * 
 * Configures the sensor for continuous measurement with FIFO buffering.
 * Clears FIFO, sets SMUX mode and gain, then starts measurement.
 * 
 * @param[in] mode SMUX mode for measurements
 * @param[in] gain Gain setting for measurements
 * 
 * @note This function only starts the acquisition. To read accumulated
 * data, use tcs3448_fifo_read_batch() periodically.
 * 
 * @warning FIFO mode is not fully implemented in this version.
 * This function exists for API compatibility but may not work correctly.
 * Use tcs3448_read_spectral() for reliable operation.
 * 
 * @par Example - FIFO Mode (Experimental):
 * @code
 * tcs3448_fifo_start(TCS3448_SMUX_18CH, TCS3448_GAIN_8X);
 * 
 * // Later, read accumulated samples
 * tcs3448_data_t samples[10];
 * uint8_t num_read = tcs3448_fifo_read_batch(samples, 10);
 * @endcode
 * 
 * @see tcs3448_fifo_read_batch()
 */
void tcs3448_fifo_start(tcs3448_smux_mode_t mode, tcs3448_gain_t gain);

/**
 * @brief Read multiple samples from FIFO buffer
 * 
 * Reads up to max_samples from the FIFO buffer into the provided array.
 * 
 * @param[out] data Array to store read samples
 * @param[in] max_samples Maximum number of samples to read
 * @return Number of samples actually read (0 if FIFO empty)
 * 
 * @note Each sample contains all 18 channel values.
 * Ensure data array has enough space: sizeof(tcs3448_data_t) * max_samples
 * 
 * @warning This is a placeholder implementation. Full FIFO support requires
 * proper FIFO_MAP configuration which is not implemented.
 * 
 * @par Example:
 * @code
 * tcs3448_data_t buffer[10];
 * uint8_t count = tcs3448_fifo_read_batch(buffer, 10);
 * 
 * for (uint8_t i = 0; i < count; i++) {
 *     // Process buffer[i]
 * }
 * @endcode
 */
uint8_t tcs3448_fifo_read_batch(tcs3448_data_t *data, uint8_t max_samples);

/**
 * @brief Turn on LED driver
 * 
 * Convenience function to enable the external LED connected to LDR pin.
 * LED current must be set first with tcs3448_led_current().
 * 
 * @note Uses global device handle initialized by tcs3448_meas_init().
 * 
 * @warning Has no effect if tcs3448_meas_init() was not called successfully.
 * 
 * @par Example:
 * @code
 * tcs3448_led_current(50);  // Set 50mA
 * tcs3448_led_on();         // Turn on
 * delay(100);               // Illuminate for 100ms
 * tcs3448_led_off();        // Turn off
 * @endcode
 * 
 * @see tcs3448_led_off()
 * @see tcs3448_led_current()
 */
void tcs3448_led_on(void);

/**
 * @brief Turn off LED driver
 * 
 * Convenience function to disable the external LED.
 * 
 * @note Uses global device handle initialized by tcs3448_meas_init().
 * 
 * @warning Has no effect if tcs3448_meas_init() was not called successfully.
 * 
 * @par Example:
 * @code
 * tcs3448_led_off();  // Ensure LED is off
 * @endcode
 * 
 * @see tcs3448_led_on()
 */
void tcs3448_led_off(void);

/**
 * @brief Set LED drive current
 * 
 * Convenience function to configure LED current. The LED is not
 * automatically turned on - use tcs3448_led_on() after setting current.
 * 
 * @param[in] current_ma LED current in milliamps (4-258)
 * 
 * @note Uses global device handle initialized by tcs3448_meas_init().
 * 
 * @par Current Calculation:
 * Actual current = 4mA + (register_value * 2mA)
 * Value is clamped: min=4mA, max=258mA
 * 
 * @par Example:
 * @code
 * tcs3448_led_current(20);   // Set to 20mA
 * tcs3448_led_on();          // Turn on
 * @endcode
 * 
 * @see tcs3448_led_on()
 * @see tcs3448_led_off()
 */
void tcs3448_led_current(uint8_t current_ma);

/**
 * @brief Convert raw counts to basic counts
 * 
 * Normalizes raw ADC counts by removing gain and integration time effects.
 * Basic counts represent the normalized light intensity independent of
 * measurement settings.
 * 
 * @param[in] counts Raw ADC count value (0-65535)
 * @param[in] gain Gain setting used during measurement
 * @param[in] atime ATIME value used during measurement
 * @param[in] astep ASTEP value used during measurement
 * @return Basic count value (float, normalized)
 * 
 * @par Formula:
 * @code
 * basic_count = raw_count / (gain_value * integration_factor)
 * where:
 *   gain_value = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, or 2048
 *   integration_factor = (ATIME + 1) * (ASTEP + 1)
 * @endcode
 * 
 * @par Example:
 * @code
 * // Convert measurement to basic counts
 * float basic = tcs3448_counts_to_basic(
 *     data.channels[TCS3448_CH_FZ],
 *     data.gain,
 *     TCS3448_DEFAULT_ATIME,
 *     TCS3448_DEFAULT_ASTEP
 * );
 * @endcode
 * 
 * @note Basic counts allow comparison between measurements with different
 * gain and integration time settings.
 */
float tcs3448_counts_to_basic(uint16_t counts, tcs3448_gain_t gain, 
                               uint8_t atime, uint16_t astep);

/**
 * @brief Print spectral data to Serial
 * 
 * Convenience function to print all channel values and status information
 * to the Arduino Serial port.
 * 
 * @param[in] data Pointer to data structure to print
 * 
 * @note Output format:
 * @code
 * Channels: ch0, ch1, ch2, ..., chN
 * WARNING: Saturation detected (if applicable)
 * @endcode
 * 
 * @par Example:
 * @code
 * tcs3448_data_t data;
 * if (tcs3448_read_spectral(&data, TCS3448_GAIN_8X, TCS3448_SMUX_18CH)) {
 *     tcs3448_print_data(&data);
 * }
 * // Output: Channels: 1234, 5678, 9012, ...
 * @endcode
 * 
 * @warning Requires Serial to be initialized (Serial.begin()) before use.
 */
void tcs3448_print_data(const tcs3448_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* TCS3448_MEAS_H */
