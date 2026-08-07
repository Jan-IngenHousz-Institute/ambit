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
