// SPDX-License-Identifier: CERN-OHL-S-2.0
// Copyright (c) 2026 Jan IngenHousz Institute

#include <stdint.h>

#include <iostream>
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

}  // namespace

int main() {
  test_legacy_defaults();
  test_saved_six_channel_baseline();
  test_saturation_and_invalid_channel();
  if (failures != 0) return 1;
  std::cout << "calibration tests passed\n";
  return 0;
}
