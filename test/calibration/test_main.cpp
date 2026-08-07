// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#include <stdint.h>

#include <iostream>
#include <limits>
#include <string>

#include "../../src/calibration_math.h"

namespace {

int failures = 0;

void check(bool condition, const std::string &message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

void test_legacy_defaults() {
  const uint32_t offsets[ambit_calibration::CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::S630, 123, offsets) == 123,
        "legacy fluorescence signal stays unchanged");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::R630, 456, offsets) == 456,
        "legacy fluorescence reference stays unchanged");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::SUN, 65123, offsets) == 123,
        "legacy sun correction remains 65000");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::LEAF, 64999, offsets) == 0,
        "legacy leaf correction saturates at zero");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::S730, 789, offsets) == 789,
        "legacy 730 signal stays unchanged");
}

void test_explicit_zero_offsets_are_not_legacy_defaults() {
  const uint32_t offsets[ambit_calibration::CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
  check(ambit_calibration::apply_adpd_offset(
            ambit_calibration::SUN, 123, offsets, true) == 123,
        "an explicitly stored zero sun baseline must be honored");
  check(ambit_calibration::apply_adpd_offset(
            ambit_calibration::LEAF, 456, offsets, true) == 456,
        "an explicitly stored zero leaf baseline must be honored");
}

void test_saved_six_channel_baseline() {
  const uint32_t offsets[ambit_calibration::CHANNEL_COUNT] = {10, 20, 30, 40, 50, 60};
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::S630, 110, offsets) == 100,
        "saved s_630 baseline is applied");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::R630, 120, offsets) == 100,
        "saved r_630 baseline is applied");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::SUN, 130, offsets) == 100,
        "saved sun baseline replaces the legacy constant");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::LEAF, 140, offsets) == 100,
        "saved leaf baseline replaces the legacy constant");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::S730, 150, offsets) == 100,
        "saved s_730 baseline is applied");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::R730, 160, offsets) == 100,
        "saved r_730 baseline is applied");
}

void test_saturation_and_invalid_channel() {
  const uint32_t offsets[ambit_calibration::CHANNEL_COUNT] = {100, 100, 100, 100, 100, 100};
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::S630, 99, offsets) == 0,
        "under-baseline values saturate at zero");
  check(ambit_calibration::apply_adpd_offset(ambit_calibration::R730, 100, offsets) == 0,
        "baseline equality returns zero");
  check(ambit_calibration::apply_adpd_offset(99, 123, offsets) == 123,
        "unknown channels are not modified");
}

void test_coefficient_bounds() {
  check(!ambit_calibration::valid_actinic_coefficient(0.01f),
        "actinic lower bound is exclusive");
  check(ambit_calibration::valid_actinic_coefficient(0.0101f),
        "actinic value above the lower bound is accepted");
  check(ambit_calibration::valid_actinic_coefficient(1.0f),
        "actinic upper bound is inclusive");
  check(!ambit_calibration::valid_actinic_coefficient(1.0001f),
        "actinic value above the upper bound is rejected");
  check(ambit_calibration::valid_spec_coefficient(0.05f),
        "spectral lower bound is inclusive");
  check(ambit_calibration::valid_spec_coefficient(100.0f),
        "spectral upper bound is inclusive");
  check(!ambit_calibration::valid_spec_coefficient(0.049f),
        "spectral value below the lower bound is rejected");
  check(!ambit_calibration::valid_spec_coefficient(100.01f),
        "spectral value above the upper bound is rejected");
  check(!ambit_calibration::valid_actinic_coefficient(
            std::numeric_limits<float>::quiet_NaN()) &&
            !ambit_calibration::valid_spec_coefficient(
                std::numeric_limits<float>::infinity()),
        "non-finite coefficients are rejected");
}

void test_strict_baseline_parsing() {
  uint32_t value = 777;
  check(ambit_calibration::parse_adpd_baseline_value("0", &value) && value == 0,
        "an explicit zero baseline parses successfully");
  check(ambit_calibration::parse_adpd_baseline_value(" 16777215 ", &value) &&
            value == ambit_calibration::MAX_ADPD_BASELINE,
        "the maximum baseline and surrounding whitespace are accepted");

  const char *invalid[] = {
      "", " ", "not-a-number", "nan", "inf", "-1", "1.5",
      "16777216", "12tail",
  };
  for (const char *text : invalid) {
    value = 777;
    check(!ambit_calibration::parse_adpd_baseline_value(text, &value),
          std::string("invalid baseline token is rejected: '") + text + "'");
    check(value == 777, "a rejected token does not mutate its output");
  }
  check(!ambit_calibration::parse_adpd_baseline_value(nullptr, &value),
        "a missing baseline token is rejected");
  check(!ambit_calibration::parse_adpd_baseline_value("1", nullptr),
        "a missing baseline output is rejected");
}

void test_baseline_vector_validation() {
  uint32_t values[ambit_calibration::CHANNEL_COUNT] = {
      ambit_calibration::MAX_S630_BASELINE, 1, 2, 3, 4,
      ambit_calibration::MAX_ADPD_BASELINE,
  };
  check(ambit_calibration::valid_adpd_baseline(values),
        "baseline vector accepts both inclusive boundaries");
  values[ambit_calibration::S630] = ambit_calibration::MAX_S630_BASELINE + 1U;
  check(!ambit_calibration::valid_adpd_baseline(values),
        "baseline vector rejects fluorescence baseline above 400");
  values[ambit_calibration::S630] = 0;
  values[ambit_calibration::R730] = ambit_calibration::MAX_ADPD_BASELINE + 1U;
  check(!ambit_calibration::valid_adpd_baseline(values),
        "baseline vector rejects any channel above 24 bits");
  check(!ambit_calibration::valid_adpd_baseline(nullptr),
        "missing baseline vector is rejected");
}

void test_wrap_safe_timeout() {
  check(!ambit_calibration::wrap_safe_timeout_elapsed(100U, 149U, 50U),
        "timeout remains pending one millisecond before its boundary");
  check(ambit_calibration::wrap_safe_timeout_elapsed(100U, 150U, 50U),
        "timeout expires at its boundary");
  check(!ambit_calibration::wrap_safe_timeout_elapsed(
            0xFFFFFFF0U, 0x0000000FU, 32U),
        "timeout remains pending across millis wrap");
  check(ambit_calibration::wrap_safe_timeout_elapsed(
            0xFFFFFFF0U, 0x00000010U, 32U),
        "timeout expires correctly across millis wrap");
}

}  // namespace

int main() {
  test_legacy_defaults();
  test_explicit_zero_offsets_are_not_legacy_defaults();
  test_saved_six_channel_baseline();
  test_saturation_and_invalid_channel();
  test_coefficient_bounds();
  test_strict_baseline_parsing();
  test_baseline_vector_validation();
  test_wrap_safe_timeout();
  if (failures != 0) return 1;
  std::cout << "calibration tests passed\n";
  return 0;
}
