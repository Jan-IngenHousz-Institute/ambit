#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "trace_v3.h"

using ambit_trace_v3::SeriesClock;

static bool near(double left, double right) {
  return fabs(left - right) < 0.0000001;
}

int main() {
  const uint8_t same_frequency[] = {
      1, 0, 0, 2, 0, 1, 0, 1,
      2, 0, 0, 2, 0, 1, 0, 1,
  };
  ambit_trace_v3::TimeModel main = ambit_trace_v3::analyze_time_model(
      same_frequency, 2, SeriesClock::MAIN, 0.854, 4);
  assert(main.regular);
  assert(near(main.t0, 0.0));
  assert(near(main.dt, 0.854));

  const uint8_t mixed_frequency[] = {
      1, 0, 0, 2, 0, 1, 0, 1,
      1, 0, 0, 2, 0, 2, 0, 1,
  };
  main = ambit_trace_v3::analyze_time_model(
      mixed_frequency, 2, SeriesClock::MAIN, 0.854, 4);
  assert(!main.regular);

  const uint8_t one_point_mixed[] = {
      1, 0, 0, 1, 0, 1, 0, 1,
      1, 0, 0, 1, 0, 2, 0, 1,
  };
  main = ambit_trace_v3::analyze_time_model(
      one_point_mixed, 2, SeriesClock::MAIN, 0.854, 2);
  assert(!main.regular);

  const uint8_t close_mixed_frequency[] = {
      1, 0, 0, 1, 0, 200, 0, 1,
      1, 0, 0, 2, 0, 201, 0, 1,
  };
  main = ambit_trace_v3::analyze_time_model(
      close_mixed_frequency, 2, SeriesClock::MAIN, 0.854, 3);
  assert(!main.regular);
  double close_mixed_times[3] = {0.0, 0.0, 0.0};
  assert(ambit_trace_v3::for_each_time(
             close_mixed_frequency, 2, SeriesClock::MAIN, 0.854, 3,
             [&](uint16_t index, double value) {
               close_mixed_times[index] = value;
             }) == 3);
  char rendered_third[32];
  ambit_trace_v3::format_time(close_mixed_times[2], rendered_third);
  assert(strcmp(rendered_third, "0.0085") == 0);

  const uint8_t subsampled[] = {
      1, 0, 0, 16, 0, 1, 0, 2,
  };
  ambit_trace_v3::TimeModel ambient = ambit_trace_v3::analyze_time_model(
      subsampled, 1, SeriesClock::AMBIENT, 0.854, 2);
  assert(ambient.regular);
  assert(near(ambient.t0, 3.5 * 0.854));
  assert(near(ambient.dt, 8.0 * 0.854));

  ambit_trace_v3::RunCounts counts;
  const uint8_t valid_run[] = {1, 0, 0, 8, 0, 10, 0, 2};
  assert(ambit_trace_v3::validate_run_protocol(valid_run, 1, 1999, &counts));
  assert(counts.main == 8 && counts.ambient == 1 && counts.reflection == 1);

  const uint8_t zero_frequency[] = {1, 0, 0, 1, 0, 0, 0, 0};
  assert(!ambit_trace_v3::validate_run_protocol(
      zero_frequency, 1, 1999, &counts));
  const uint8_t zero_pulses[] = {1, 0, 0, 0, 0, 1, 0, 0};
  assert(!ambit_trace_v3::validate_run_protocol(
      zero_pulses, 1, 1999, &counts));
  const uint8_t invalid_subsampling[] = {1, 0, 0, 1, 0, 1, 0, 3};
  assert(!ambit_trace_v3::validate_run_protocol(
      invalid_subsampling, 1, 1999, &counts));
  const uint8_t unsupported_type[] = {3, 0, 0, 1, 0, 1, 0, 0};
  assert(!ambit_trace_v3::validate_run_protocol(
      unsupported_type, 1, 1999, &counts));
  const uint8_t over_capacity[] = {1, 0, 7, 208, 0, 1, 0, 0};
  assert(!ambit_trace_v3::validate_run_protocol(
      over_capacity, 1, 1999, &counts));

  const uint8_t formerly_wrapping_aggregate[] = {
      1, 0, 255, 255, 0, 1, 0, 0,
      1, 0, 0, 1, 0, 1, 0, 0,
  };
  assert(ambit_trace_v3::validate_run_protocol(
      formerly_wrapping_aggregate, 2, UINT32_MAX, &counts));
  assert(counts.main == 65536U);
  assert(!ambit_trace_v3::validate_run_protocol(
      formerly_wrapping_aggregate, 2, 1999, &counts));

  assert(ambit_trace_v3::duration_ms(1000000, 1500123) == 500);
  assert(ambit_trace_v3::clamp_count(100000) == 65535);
  char sensor_id[18];
  ambit_trace_v3::format_sensor_id(0xD44F4FA89110ULL, sensor_id);
  assert(strcmp(sensor_id, "10:91:A8:4F:4F:D4") == 0);
  char formatted_time[32];
  ambit_trace_v3::format_time(0.8540, formatted_time);
  assert(strcmp(formatted_time, "0.854") == 0);
  ambit_trace_v3::format_time(2.0, formatted_time);
  assert(strcmp(formatted_time, "2") == 0);
  ambit_trace_v3::format_time(42.8, formatted_time);
  assert(strcmp(formatted_time, "42.8") == 0);
  char formatted_temp[16];
  ambit_trace_v3::format_leaf_temp(2460, formatted_temp);
  assert(strcmp(formatted_temp, "24.60") == 0);
  return 0;
}
