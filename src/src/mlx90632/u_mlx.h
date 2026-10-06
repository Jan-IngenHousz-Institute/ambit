#ifndef __u_MLX_H_
#define __u_MLX_H_

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include "../pin_config.h"
#include "mlx_dev.h"

// Bus callbacks of the vendor driver: 0, or -EIO when the I2C transaction failed.
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value);
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value);
extern bool FLAG_DEICE;

void usleep(int min_range, int max_range);
void msleep(int msecs);


// Reported for every temperature (object, ambient, reflected) when the MLX has no
// verified calibration or a reading failed (bus error, DATA_RDY timeout), instead of a
// value computed from placeholders. Absolute zero: impossible for a real reading, and
// representable in every encoding the callers use (int16 centi-°C -27315, int16 deci-°C
// -2731, JSON double).
constexpr double MLX_TEMP_INVALID = -273.15;

// true only when the 13 calibration constants were read twice, identically, with every
// transaction acknowledged. mlx_measure() retries it on its own while false.
bool mlx_init(void);
bool mlx_calibration_valid(void);
double mlx_measure(double* object, double* ambient);
double mlx_measure(double* object, double* ambient, double* reflect_obj, int16_t* a1, int16_t* a2, int16_t* a3, int16_t* a4);
double mlx_measure();

void mlx_print_paras(double e);
void mlx_read_coe(int32_t*);
// mlxee,<reps>: console dump of the EEPROM calibration block, one checked 16-bit read
// per word, <reps> passes (0 = none); then each 32-bit constant read both ways (word
// pair vs the pre-fix 4-byte burst), the init attempts / bus errors since boot, and the
// constants mlx_init() kept.
void mlx_dump_eeprom(uint8_t reps);
#ifdef AMBIT_DIAG_MLX
// Bench-only fault injection (build flag AMBIT_DIAG_MLX): fail the next <n> register reads.
void mlx_diag_fail_reads(uint16_t n);
uint16_t mlx_diag_fail_reads_left(void);
#endif




#endif