#ifndef _SPEC_MEAS_H_
#define _SPEC_MEAS_H_
/* This header went unguarded while it held only macros, externs and prototypes,
 * all of which tolerate repeated inclusion. spec_raw_t does not, and several
 * translation units pull this in twice (directly and via PAM.h). */
#include "Adafruit_AS7341.h"
#include "../pin_config.h"

#define PAR_COE1 -0.515
#define PAR_COE2 0.801
#define PAR_COE3 -0.385
#define PAR_COE4 -0.202 
#define PAR_COE5 0.301
#define PAR_COE6 0.183
#define PAR_COE7 0.055
#define PAR_COE8 0.081
#define PAR_COE9 -0.033

#define Spec_COE1 12
#define Spec_COE2 10
#define Spec_COE3 11
#define Spec_COE4 10 
#define Spec_COE5 10
#define Spec_COE6 9
#define Spec_COE7 7
#define Spec_COE8 4
#define Spec_COE9 1

#define PAR_OFFSET 4

/* Acquisition parameters dual_exposure() programs for every spectral read.
 * Full-scale ADC count is (SPEC_ATIME + 1) * (SPEC_ASTEP + 1) = 50000 — below
 * the 16-bit register ceiling, which is why *unscaled* counts always fit a
 * uint16 while the Spec_COE*-weighted ones overflow from ~11% of full scale
 * (Spec_COE1 = 12 wraps above 5461 counts). One definition so the parameters
 * reported to the host cannot drift from the ones actually programmed. */
#define SPEC_ATIME 99
#define SPEC_ASTEP 499
#define SPEC_FULL_SCALE (((uint32_t) SPEC_ATIME + 1) * ((uint32_t) SPEC_ASTEP + 1))

/* One spectral read in unscaled counts, plus the settings it was taken under.
 * A host needs gain/atime/astep to normalise counts across exposures, and will
 * need them per-measurement the day autoranging lands. We report what we wrote
 * rather than reading the registers back: getGain()/getATIME()/getASTEP() each
 * cost an I2C transaction to return a value we already know. */
typedef struct {
    uint16_t raw[10];     // F1,F2,F3,F4,F5,F6,F7,F8,NIR,Clear — unscaled counts
    uint16_t astep;
    uint8_t  atime;
    uint8_t  gain_low;    // as7341_gain_t for the F1-F4/Clear/NIR SMUX bank
    uint8_t  gain_high;   // as7341_gain_t for the F5-F8 bank (diverges once
                          // autoranging exists; equal today)
    bool     saturated;   // at least one channel reached SPEC_FULL_SCALE
} spec_raw_t;

extern Adafruit_AS7341 as7341;

extern bool as7341_setup;
bool check_AS7341();
void AS_LED_Current(uint16_t current);
void AS_LED_ON();
void AS_LED_OFF();

void AS_all_channel(uint16_t T1, uint16_t T2, uint16_t *spec);
void AS_all_channel(uint16_t T1, uint16_t T2);
double get_PAR();
double get_PAR(uint16_t *spec);
double get_PAR_raw(spec_raw_t *out);



uint8_t as7431_reg_write(uint8_t reg, uint8_t data);
uint8_t as7431_reg_read(uint8_t reg, uint8_t* data, uint16_t len);
void as7431_blink(uint8_t n, uint8_t intensity);

#endif // _SPEC_MEAS_H_
