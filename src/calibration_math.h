#ifndef _CALIBRATION_MATH_H_
#define _CALIBRATION_MATH_H_

#include <stdint.h>

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
