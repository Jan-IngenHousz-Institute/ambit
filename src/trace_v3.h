#ifndef AMBIT_TRACE_V3_H
#define AMBIT_TRACE_V3_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace ambit_trace_v3 {

// Path A does not pass through the uint16 FSM encoder, so it must apply the
// same saturation rule explicitly or the two v3 producers disagree at high
// signal levels.
constexpr uint16_t clamp_count(uint32_t value) {
  return value > 0xFFFFU ? 0xFFFFU : static_cast<uint16_t>(value);
}

inline void format_sensor_id(uint64_t efuse_mac, char output[18]) {
  // ESP.getEfuseMac() and the frozen cmd-33 FW struct store byte zero in the
  // low eight bits. Match the Ambyte inventory formatter exactly so traces
  // join ambit.device/1 on one stable, uppercase colon-separated identity.
  snprintf(output, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           static_cast<unsigned>(efuse_mac & 0xFFU),
           static_cast<unsigned>((efuse_mac >> 8) & 0xFFU),
           static_cast<unsigned>((efuse_mac >> 16) & 0xFFU),
           static_cast<unsigned>((efuse_mac >> 24) & 0xFFU),
           static_cast<unsigned>((efuse_mac >> 32) & 0xFFU),
           static_cast<unsigned>((efuse_mac >> 40) & 0xFFU));
}

inline void format_time(double value, char output[32]) {
  snprintf(output, 32, "%.4f", value);
  size_t end = strlen(output);
  while (end > 0 && output[end - 1] == '0') output[--end] = '\0';
  if (end > 0 && output[end - 1] == '.') output[--end] = '\0';
  if (strcmp(output, "-0") == 0) strcpy(output, "0");
}

inline void format_leaf_temp(int16_t centi_c, char output[16]) {
  snprintf(output, 16, "%.2f", static_cast<double>(centi_c) / 100.0);
}

enum class SeriesClock : uint8_t {
  MAIN,
  AMBIENT,
  REFLECTION,
};

struct Segment {
  uint8_t type;
  uint16_t pulses;
  uint16_t freq;
  uint8_t actinic;
  uint8_t subsampling;
};

struct RunCounts {
  uint32_t main;
  uint32_t ambient;
  uint32_t reflection;
};

inline Segment decode_segment(const uint8_t* line) {
  Segment segment = {
      line[0],
      static_cast<uint16_t>((static_cast<uint16_t>(line[2]) << 8) | line[3]),
      static_cast<uint16_t>((static_cast<uint16_t>(line[4]) << 8) | line[5]),
      line[6],
      line[7],
  };
  return segment;
}

inline bool active(const Segment& segment) {
  return (segment.type == 1 || segment.type == 2) && segment.freq != 0;
}

inline bool checked_add(uint32_t* total, uint32_t value) {
  if (value > UINT32_MAX - *total) return false;
  *total += value;
  return true;
}

inline bool validate_run_protocol(const uint8_t* protocol,
                                  uint8_t segment_count,
                                  uint32_t max_series_count,
                                  RunCounts* output) {
  RunCounts counts = {0, 0, 0};
  if (protocol == NULL || segment_count == 0 || max_series_count == 0) {
    if (output != NULL) *output = counts;
    return false;
  }

  for (uint8_t i = 0; i < segment_count; ++i) {
    const Segment segment =
        decode_segment(protocol + static_cast<uint16_t>(i) * 8U);
    if (segment.type == 0) continue;
    if ((segment.type != 1 && segment.type != 2) || segment.pulses == 0 ||
        segment.freq == 0 || segment.subsampling > 2) {
      if (output != NULL) *output = counts;
      return false;
    }

    const uint32_t optional = segment.subsampling == 0
                                  ? 0U
                                  : (segment.subsampling == 1
                                         ? segment.pulses
                                         : segment.pulses / 8U);
    if (!checked_add(&counts.main, segment.pulses) ||
        !checked_add(&counts.ambient, optional) ||
        (segment.type == 1 &&
         !checked_add(&counts.reflection, optional))) {
      if (output != NULL) *output = counts;
      return false;
    }
  }

  if (output != NULL) *output = counts;
  // dataclass is a fixed non-ring trace store here. Reject at the decoded
  // protocol boundary instead of allowing uint16 aggregation or writes to wrap
  // and silently retain only the tail of the requested measurement.
  return counts.main > 0 && counts.main <= max_series_count &&
         counts.ambient <= max_series_count &&
         counts.reflection <= max_series_count;
}

inline uint16_t point_count(const Segment& segment, SeriesClock clock) {
  if (!active(segment)) return 0;
  if (clock == SeriesClock::MAIN) return segment.pulses;
  if (segment.subsampling == 0) return 0;
  if (clock == SeriesClock::REFLECTION && segment.type != 1) return 0;
  if (segment.subsampling == 1) return segment.pulses;
  if (segment.subsampling == 2) return segment.pulses / 8;
  return 0;
}

inline double base_period(const Segment& segment, double tick_factor) {
  return tick_factor / static_cast<double>(segment.freq);
}

inline double series_period(const Segment& segment, SeriesClock clock,
                            double tick_factor) {
  const double base = base_period(segment, tick_factor);
  return (clock != SeriesClock::MAIN && segment.subsampling == 2) ? 8.0 * base
                                                                 : base;
}

inline uint8_t effective_period_multiplier(const Segment& segment,
                                           SeriesClock clock) {
  return (clock != SeriesClock::MAIN && segment.subsampling == 2) ? 8U : 1U;
}

inline double first_offset(const Segment& segment, SeriesClock clock,
                           double tick_factor) {
  if (clock != SeriesClock::MAIN && segment.subsampling == 2) {
    // The ambient/reflection value is the mean of pulses 0..7, so its time is
    // the centre of that eight-pulse window, not its first or last pulse.
    return 3.5 * base_period(segment, tick_factor);
  }
  return 0.0;
}

template <typename Visitor>
uint16_t for_each_time(const uint8_t* protocol, uint8_t segment_count,
                       SeriesClock clock, double tick_factor,
                       uint16_t value_count, Visitor visitor) {
  uint16_t emitted = 0;
  double segment_start = 0.0;
  for (uint8_t i = 0; i < segment_count; ++i) {
    const Segment segment = decode_segment(protocol + static_cast<uint16_t>(i) * 8U);
    if (!active(segment)) continue;

    const double base = base_period(segment, tick_factor);
    const uint16_t count = point_count(segment, clock);
    const double first = first_offset(segment, clock, tick_factor);
    const double period = series_period(segment, clock, tick_factor);
    for (uint16_t point = 0; point < count && emitted < value_count; ++point) {
      visitor(emitted, segment_start + first + static_cast<double>(point) * period);
      ++emitted;
    }
    segment_start += static_cast<double>(segment.pulses) * base;
    if (emitted == value_count) break;
  }
  return emitted;
}

struct TimeModel {
  uint16_t count;
  double t0;
  double dt;
  bool regular;
};

inline TimeModel analyze_time_model(const uint8_t* protocol, uint8_t segment_count,
                                    SeriesClock clock, double tick_factor,
                                    uint16_t value_count) {
  TimeModel model = {value_count, 0.0, 0.0, true};
  bool have_period = false;
  uint16_t first_frequency = 0;
  uint8_t first_multiplier = 0;
  for (uint8_t i = 0; i < segment_count; ++i) {
    const Segment segment = decode_segment(protocol + static_cast<uint16_t>(i) * 8U);
    if (point_count(segment, clock) == 0) continue;
    const double period = series_period(segment, clock, tick_factor);
    const uint8_t multiplier = effective_period_multiplier(segment, clock);
    if (!have_period) {
      model.dt = period;
      first_frequency = segment.freq;
      first_multiplier = multiplier;
      have_period = true;
    } else if (segment.freq != first_frequency || multiplier != first_multiplier) {
      // Frequency/effective-period identity is categorical. Rendering tolerance
      // belongs only to the continuity check below; using it here can hide a
      // real 200 -> 201 Hz transition and shift a rendered timestamp by 0.1 ms.
      model.regular = false;
    }
  }

  uint16_t seen = 0;
  for_each_time(protocol, segment_count, clock, tick_factor, value_count,
                [&](uint16_t index, double time) {
                  if (index == 0) {
                    model.t0 = time;
                  } else {
                    const double expected = model.t0 + static_cast<double>(index) * model.dt;
                    // The wire renders times to 0.1 ms. Differences below half
                    // that quantum are observationally identical.
                    if (fabs(time - expected) > 0.00005) model.regular = false;
                  }
                  ++seen;
                });
  if (seen != value_count) model.regular = false;
  return model;
}

inline uint32_t duration_ms(int64_t begin_us, int64_t end_us) {
  if (end_us <= begin_us) return 0;
  const uint64_t elapsed_ms = static_cast<uint64_t>(end_us - begin_us) / 1000ULL;
  return elapsed_ms > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed_ms);
}

static_assert(clamp_count(0) == 0, "zero count must survive");
static_assert(clamp_count(65535) == 65535, "maximum wire count must survive");
static_assert(clamp_count(65536) == 65535, "path-A counts must saturate");

}  // namespace ambit_trace_v3

#endif
