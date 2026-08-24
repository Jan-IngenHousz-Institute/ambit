#ifndef _CALIBRATION_MATH_H_
#define _CALIBRATION_MATH_H_

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

namespace ambit_calibration {

enum AdpdChannel : uint8_t {
    S630 = 0,
    R630 = 1,
    SUN = 2,
    LEAF = 3,
    S730 = 4,
    R730 = 5,
    CHANNEL_COUNT = 6,
};

// These limits are part of the calibration contract, not transport-specific
// policy. Keep every text, binary, and NVS-load path on the same predicates so
// a value cannot be accepted over one frontend and rejected after reboot.
constexpr float ACTINIC_COEFFICIENT_MIN = 0.01f;  // exclusive
constexpr float ACTINIC_COEFFICIENT_MAX = 1.0f;   // inclusive
constexpr float SPEC_COEFFICIENT_MIN = 0.05f;     // inclusive
constexpr float SPEC_COEFFICIENT_MAX = 100.0f;    // inclusive
constexpr uint32_t MAX_ADPD_BASELINE = 0xFFFFFFU;
constexpr uint32_t MAX_S630_BASELINE = 400U;

inline bool valid_actinic_coefficient(float value) {
    return isfinite(value) && value > ACTINIC_COEFFICIENT_MIN &&
           value <= ACTINIC_COEFFICIENT_MAX;
}

inline bool valid_spec_coefficient(float value) {
    return isfinite(value) && value >= SPEC_COEFFICIENT_MIN &&
           value <= SPEC_COEFFICIENT_MAX;
}

inline bool valid_adpd_baseline(const uint32_t values[CHANNEL_COUNT]) {
    if (values == nullptr || values[S630] > MAX_S630_BASELINE) return false;
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        if (values[i] > MAX_ADPD_BASELINE) return false;
    }
    return true;
}

// ── Spectral / PAR calibration (the cmd 35 three-tier chain) ────────────────
// Ten AS7341 channels in the reported wavelength order: F1..F8, NIR, Clear.
constexpr uint8_t SPEC_CHANNEL_COUNT = 10;

// These bounds are provisional. They exist to keep obvious garbage out of NVS
// and off the wire; the real ranges are only knowable once the LR1-B /
// Li-250A campaign produces fitted vectors (plans/AMBIT_COMMAND35_SPECPAR.md
// §11). The element-wise isfinite() check is the load-bearing part today: it is
// the only thing standing between a bad write and a NaN propagating through the
// tier-1 divide into every downstream field of the cmd 35 payload.
constexpr float SPEC_OFFSET_MAX = 1.0f;         // exclusive; basic-count units
constexpr float SPEC_SENS_MAX = 1000.0f;        // inclusive
constexpr float PAR_WEIGHT_ABS_MAX = 1.0e4f;
constexpr float PAR_SLOPE_MAX = 100.0f;         // inclusive
constexpr float PAR_INTERCEPT_ABS_MAX = 500.0f;

inline bool valid_spec_offset(const float values[SPEC_CHANNEL_COUNT]) {
    if (values == nullptr) return false;
    for (uint8_t i = 0; i < SPEC_CHANNEL_COUNT; ++i) {
        if (!isfinite(values[i]) || values[i] < 0.0f ||
            values[i] >= SPEC_OFFSET_MAX) {
            return false;
        }
    }
    return true;
}

inline bool valid_spec_sens(const float values[SPEC_CHANNEL_COUNT]) {
    if (values == nullptr) return false;
    for (uint8_t i = 0; i < SPEC_CHANNEL_COUNT; ++i) {
        if (!isfinite(values[i]) || values[i] <= 0.0f ||
            values[i] > SPEC_SENS_MAX) {
            return false;
        }
    }
    return true;
}

// Signs are deliberately unconstrained: the tier-2 fit legitimately produces a
// negative NIR/Clear term (stray-light subtraction), and constraining the
// visible channels is a fitting decision that belongs in the host tooling, not
// in a firmware acceptance test.
inline bool valid_par_weight(const float values[SPEC_CHANNEL_COUNT]) {
    if (values == nullptr) return false;
    for (uint8_t i = 0; i < SPEC_CHANNEL_COUNT; ++i) {
        if (!isfinite(values[i]) || fabsf(values[i]) > PAR_WEIGHT_ABS_MAX) {
            return false;
        }
    }
    return true;
}

inline bool valid_par_slope(float value) {
    return isfinite(value) && value > 0.0f && value <= PAR_SLOPE_MAX;
}

inline bool valid_par_intercept(float value) {
    return isfinite(value) && fabsf(value) <= PAR_INTERCEPT_ABS_MAX;
}

// par_weight ships all-zero until tier 2 is fitted, which would make the
// computed PAR collapse to par_intercept. The cmd 35 handler reports that
// through flags bit2 rather than passing a meaningless number off as a reading.
inline bool par_weight_is_unset(const float values[SPEC_CHANNEL_COUNT]) {
    if (values == nullptr) return true;
    for (uint8_t i = 0; i < SPEC_CHANNEL_COUNT; ++i) {
        if (values[i] != 0.0f) return false;
    }
    return true;
}

// Text calibration input must distinguish a real zero from atoi-style parse
// failures. strtod lets us reject missing, non-numeric, non-finite, fractional,
// and out-of-range values while accepting harmless surrounding whitespace.
inline bool parse_adpd_baseline_value(const char *text, uint32_t *value) {
    if (text == nullptr || value == nullptr) return false;

    errno = 0;
    char *end = nullptr;
    const double parsed = strtod(text, &end);
    if (end == text || errno == ERANGE || !isfinite(parsed) || parsed < 0.0 ||
        parsed > static_cast<double>(MAX_ADPD_BASELINE) ||
        floor(parsed) != parsed) {
        return false;
    }
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end != '\0') return false;

    *value = static_cast<uint32_t>(parsed);
    return true;
}

// Strict float tokeniser for the text calibration vectors. The established
// loose idiom (Serial_Input_Double -> range predicate) is fine where zero is
// already out of range, but par_weight accepts 0.0 as both its default and a
// legitimate fitted value, so a strtod garbage-to-0.0 would be indistinguishable
// from a deliberate write. Same shape as parse_adpd_baseline_value() minus the
// integer clause.
inline bool parse_calibration_float(const char *text, float *value) {
    if (text == nullptr || value == nullptr) return false;

    errno = 0;
    char *end = nullptr;
    const double parsed = strtod(text, &end);
    if (end == text || errno == ERANGE || !isfinite(parsed)) return false;
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end != '\0') return false;

    *value = static_cast<float>(parsed);
    return true;
}

// Unsigned subtraction is defined modulo 2^32, so this remains correct when
// millis() wraps. Timeout must stay below half the uint32_t range, as ours does.
inline bool wrap_safe_timeout_elapsed(uint32_t start, uint32_t now,
                                      uint32_t timeout_ms) {
    return static_cast<uint32_t>(now - start) >= timeout_ms;
}

// Older AMBITs did not store sun/leaf baselines. Keep their established 65000
// correction until a full six-channel calibration has been saved.
inline uint32_t effective_adpd_offset(uint8_t channel,
                                      const uint32_t offsets[CHANNEL_COUNT],
                                      bool has_explicit_offset = false) {
    if (channel >= CHANNEL_COUNT) return 0;
    if (has_explicit_offset || offsets[channel] != 0) return offsets[channel];
    return (channel == SUN || channel == LEAF) ? 65000U : 0U;
}

inline uint32_t apply_adpd_offset(uint8_t channel, uint32_t sample,
                                  const uint32_t offsets[CHANNEL_COUNT],
                                  bool has_explicit_offset = false) {
    const uint32_t offset = effective_adpd_offset(channel, offsets, has_explicit_offset);
    return sample > offset ? sample - offset : 0U;
}

}  // namespace ambit_calibration

#endif  // _CALIBRATION_MATH_H_
