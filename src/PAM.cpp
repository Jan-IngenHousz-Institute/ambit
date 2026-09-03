#include "PAM.h"
#include "nvs1.h"
#include "trace_v3.h"

#include <esp_rom_crc.h>
#include <new>
#include <stdio.h>
#include <string.h>

static const char* TAG = "PAM";
static bool measure_temp = true;
static bool PAM_interrupt(bool, bool);

uint8_t adpd_mode = 0;
adpd_current_config_t adpd_current_config;
adpd_gains_config_t adpd_gains_config;


int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);

// use pre-set values
int conf_slow_FR_1(void){
  
  if (adpd_gains_config.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
  if (adpd_current_config.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");
  return conf_slow_FR_1(adpd_current_config.I620, adpd_current_config.I720, adpd_current_config.IR, adpd_gains_config.Fluo, adpd_gains_config.FluoRef, adpd_gains_config.Sun, adpd_gains_config.Leaf, adpd_gains_config.IR, adpd_gains_config.IRRef);
}


// Slow measurements with 4 time slots
// @param I620 measuring light current (0 - 127)
// @param I730 IR reflection current (0 - 127)
// @param I_FR Far-red treatment current (0 - 127)
// @param G_Fluor fluorescence signal gain (0 - 5)
// @param G_FluorRef fluorescence rerefence gain (0 - 5)
// @param G_Sun Sun facing PD gain (0 - 5)
// @param G_IR Leaf facing PD gain (0 - 5)
// @param G_FR IR reflection signal gain (0 - 5)
// @param G_FRref IR reflection reference gain (0 - 5)
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref){

  int32_t config_result = 0;

  // Setup timeslot 1: two ambient light channels, 2 x 3 bytes
  adpd.led_config.driver1_current = 0;
  adpd.led_config.driver2_current = 0;
  adpd.SNR_config.TIA_gain_CH2 = G_IR;       // channel 2: leaf IR reflection
  adpd.SNR_config.TIA_gain_CH1 = G_Sun;      // channel 1: sun vis
  config_result = adpd.preset_config_1(0, 4);
  if (config_result != jii::adpd6000::kOk) return config_result;



  // Setup timeslot 2:  Fluor and Ref channels,  4 x 3 bytes
    // LED 1A = 620nm
  adpd.led_config.driver1_current = I620;
  adpd.led_config.led1_channel = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;
  adpd.SNR_config.TIA_gain_CH1 = G_Fluor;
  adpd.SNR_config.TIA_gain_CH2 = G_FluorRef;
  config_result = adpd.preset_config_2(1, 1);
  if (config_result != jii::adpd6000::kOk) return config_result;


  // Setup timeslot 3:  IR leave reflection, 2 x 3 bytes
     // LED 1A = 620nm
  adpd.led_config.driver1_current = 0;
  adpd.led_config.led1_channel = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = I730;
  adpd.led_config.led2_channel = LED_A;
  adpd.SNR_config.TIA_gain_CH1 = G_FR;
  adpd.SNR_config.TIA_gain_CH2 = G_FRref;

  config_result = adpd.preset_config_3(2, 4);
  if (config_result != jii::adpd6000::kOk) return config_result;

  // Setup timeslot 4-5-6:  Far-red illumination, 0 data
    // LED 1A = 620nm
  adpd.led_config.driver1_current = 0;
  adpd.led_config.led1_channel = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = I_FR;
  adpd.led_config.led2_channel = LED_A;

  //make 6 time slots, ~ 2 ms without repeats, can get 200 repeats ~ 400ms
  for (uint8_t slot = 3; slot <= 8; ++slot){
    config_result = adpd.preset_config_4(slot);
    if (config_result != jii::adpd6000::kOk) return config_result;
  }

  adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;

  return 0;
}


uint32_t arr_line_parse_type1(uint8_t* line, uint8_t* num1, uint16_t* num2, uint16_t* num3, uint8_t* num4, uint8_t* num5, uint16_t* data_count){
  // type 1: Must have measurements: Fluor/Ref
  // optional measurements: leaf IR, sun VIs, Reflect/Ref
  // 0 = no points, 1 = same freq, 2 = every 8
  // data_count 0: fluor, 1: ambient, 2: reflection
  uint8_t type = line[0];                 // run type 1 = steady state, 0 = skip, 2 = no ir
  *num1 = line[1];                        // FR on / off
  *num2 = line[3] + (line[2] << 8);       // sample number
  *num3 = line[5] + (line[4] << 8);       // frequency
  *num4 = line[6];                        // actinic
  *num5 = line[7];                        // sub-sampling factor

  if (data_count == NULL) return 0;

  if ((type == 1) || (type == 2)){
    data_count[0] = *num2;
    if (line[7] == 0){ // no ambient
      data_count[1] = 0;
      data_count[2] = 0;
    }else if (line[7] == 1){ // every
      data_count[1] = *num2;
      data_count[2] = *num2;
    }else if (line[7] == 2){ // every 8
      data_count[1] = (*num2) / 8;
      data_count[2] = (*num2) / 8;
    }else{
      data_count[1] = 0;
      data_count[2] = 0;
    }
    if (type == 2) data_count[2] = 0;

    return data_count[0] * 2 + data_count[1] * 2 + data_count[2] * 2;

  }
  return 0;  
}



// Shared output sink for a completed run. COMPUTER keeps its legacy ASCII
// channels; AMBYTE streams the frozen arrays 0..7 followed by additive env-time
// array 8 when supplied. Wire-touching: re-run HW_CONFORMANCE.md after changing
// this function.
static void pam_send_results(dataclass* d_env, dataclass* d_fluor, dataclass* d_fluoRef,
                             dataclass* d_sun, dataclass* d_leaf, dataclass* d_730,
                             dataclass* d_730Ref, dataclass* d_timing,
                             dataclass* d_env_time, uint8_t subsampling, bool has_730,
                             bool allow_interrupt){
  if (CONNECTION_TYPE == CONNECTION_TYPES::COMPUTER){
    d_env->send_serial("env");
    d_fluor->send_serial("s_630");
    d_fluoRef->send_serial("r_630");
    d_sun->send_serial("sun");
    d_leaf->send_serial("leaf");
    d_730->send_serial("s_730");
    d_730Ref->send_serial("r_730");
    if (d_timing != NULL) d_timing->send_serial("timing");

    Serial.println("Data sent");
  }else if(CONNECTION_TYPE == CONNECTION_TYPES::AMBYTE){
    // Change 4: one wake per run, then every array streams back-to-back as
    // (length header + data + trailer); run_esp.cpp's trailing ESP_CMD_END (240)
    // terminates the stream. If the ambyte never wakes, there is nothing to send.
    if (ambyte_wake(allow_interrupt) != 1) return;

    // Per-array element width / dtype (Change 1 self-describing header, Change 2 widths):
    //   ENV    -> int16  centi-degC          (elem_width 2, dtype 1)
    //   ADC ch -> uint16 (16-bit ADPD regs)  (elem_width 2, dtype 0, clamped to 0xFFFF)
    //   TIMING -> uint32 ticks               (elem_width 4, dtype 0)
    d_env->fsm_send_array(0, 2, 1);
    d_fluor->fsm_send_array(1, 2, 0);
    d_fluoRef->fsm_send_array(2, 2, 0);
    if (subsampling > 0){
      d_sun->fsm_send_array(3, 2, 0);
      d_leaf->fsm_send_array(4, 2, 0);
      if (has_730){
        d_730->fsm_send_array(5, 2, 0);
        d_730Ref->fsm_send_array(6, 2, 0);
      }
    }
    if (d_timing != NULL) d_timing->fsm_send_array(7, 4, 0);   // Change 3: TIMING block
    // Array 8 is deliberately appended after every frozen 0..7 array. Existing
    // hosts already ignore unknown indices, while v3 hosts gain device-recorded
    // env offsets without any byte changing in the shipped arrays.
    if (d_env_time != NULL) d_env_time->fsm_send_array(8, 4, 0);
  }
}

int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist){
  return run_arr_type1(length, arr, led_persist, false);
}

// ── Async result holder (parallel trigger/poll/fetch protocol) ──────────────
// Holds the nine result buffers of one retained run plus the metadata
// pam_send_results() needs, until the host FETCHes them. One run at a time.
struct ambit_async_result_t {
  dataclass *d_env = NULL, *d_fluor = NULL, *d_fluoRef = NULL, *d_sun = NULL,
            *d_leaf = NULL, *d_730 = NULL, *d_730Ref = NULL, *d_timing = NULL,
            *d_env_time = NULL;
  uint8_t subsampling = 0;
  bool    has_730 = false;
  bool    allow_interrupt = false;
  uint8_t state = AMBIT_ASYNC_IDLE;
};
static ambit_async_result_t g_async;

// Every run owns these heap objects until a retained run explicitly transfers
// them to g_async. This guard covers object-allocation and dataclass::init()
// failures without duplicating a fragile nine-pointer cleanup at each return.
struct pam_run_buffers_t {
  dataclass *d_env, *d_fluor, *d_fluoRef, *d_sun, *d_leaf, *d_730,
            *d_730Ref, *d_timing, *d_env_time;

  pam_run_buffers_t()
      : d_env(new (std::nothrow) dataclass),
        d_fluor(new (std::nothrow) dataclass),
        d_fluoRef(new (std::nothrow) dataclass),
        d_sun(new (std::nothrow) dataclass),
        d_leaf(new (std::nothrow) dataclass),
        d_730(new (std::nothrow) dataclass),
        d_730Ref(new (std::nothrow) dataclass),
        d_timing(new (std::nothrow) dataclass),
        d_env_time(new (std::nothrow) dataclass) {}

  ~pam_run_buffers_t() {
    delete d_env;    delete d_fluor;  delete d_fluoRef;
    delete d_sun;    delete d_leaf;   delete d_730;
    delete d_730Ref; delete d_timing; delete d_env_time;
  }

  bool allocated() const {
    return d_env != NULL && d_fluor != NULL && d_fluoRef != NULL &&
           d_sun != NULL && d_leaf != NULL && d_730 != NULL &&
           d_730Ref != NULL && d_timing != NULL && d_env_time != NULL;
  }

  void release() {
    d_env = NULL;    d_fluor = NULL;  d_fluoRef = NULL;
    d_sun = NULL;    d_leaf = NULL;   d_730 = NULL;
    d_730Ref = NULL; d_timing = NULL; d_env_time = NULL;
  }
};

void ambit_async_clear(void){
  delete g_async.d_env;    delete g_async.d_fluor;  delete g_async.d_fluoRef;
  delete g_async.d_sun;    delete g_async.d_leaf;   delete g_async.d_730;
  delete g_async.d_730Ref; delete g_async.d_timing; delete g_async.d_env_time;
  g_async = ambit_async_result_t();   // re-init all pointers to NULL, state IDLE
}

uint8_t ambit_async_get_state(void){ return g_async.state; }


static void print_json_string(const char* value, size_t max_length){
  Serial.print('"');
  for (size_t i = 0; i < max_length && value[i] != '\0'; ++i){
    const uint8_t c = static_cast<uint8_t>(value[i]);
    if (c == '"' || c == '\\'){
      Serial.print('\\');
      Serial.print(static_cast<char>(c));
    }else if (c < 0x20){
      Serial.printf("\\u%04x", c);
    }else{
      Serial.print(static_cast<char>(c));
    }
  }
  Serial.print('"');
}

// Contract time numbers have at most 0.1 ms precision and never use exponent
// notation. Render four decimals, then remove only insignificant trailing zeros.
static void print_trace_time(double value){
  char text[32];
  ambit_trace_v3::format_time(value, text);
  Serial.print(text);
}

static void send_count_values(dataclass* data){
  Serial.print("\"v\":[");
  const uint16_t count = data->get_length();
  for (uint16_t i = 0; i < count; ++i){
    if (i > 0) Serial.print(',');
    Serial.print(ambit_trace_v3::clamp_count(data->pop()));
  }
  Serial.print(']');
}

static void send_series_clock(const uint8_t* protocol, uint8_t segment_count,
                              ambit_trace_v3::SeriesClock clock,
                              uint16_t value_count){
  const double tick_factor = static_cast<double>(ambit_calibration_local.tick_factor);
  const ambit_trace_v3::TimeModel model = ambit_trace_v3::analyze_time_model(
      protocol, segment_count, clock, tick_factor, value_count);
  if (model.regular){
    Serial.print("\"t0\":"); print_trace_time(model.t0);
    Serial.print(",\"dt\":"); print_trace_time(model.dt);
  }else{
    Serial.print("\"t\":[");
    ambit_trace_v3::for_each_time(
        protocol, segment_count, clock, tick_factor, value_count,
        [](uint16_t index, double value){
          if (index > 0) Serial.print(',');
          print_trace_time(value);
        });
    Serial.print(']');
  }
}

static void send_count_series(const char* name, dataclass* data,
                              const uint8_t* protocol, uint8_t segment_count,
                              ambit_trace_v3::SeriesClock clock){
  const uint16_t value_count = data->get_length();
  Serial.print('"'); Serial.print(name); Serial.print("\":{\"u\":\"count\",");
  send_series_clock(protocol, segment_count, clock, value_count);
  Serial.print(',');
  send_count_values(data);
  Serial.print('}');
}

static void send_leaf_temp_series(dataclass* env, dataclass* env_time){
  uint16_t count = env->get_length();
  const uint16_t time_count = env_time->get_length();
  if (time_count < count) count = time_count;

  Serial.print("\"leaf_temp\":{\"u\":\"Cel\",\"t\":[");
  for (uint16_t i = 0; i < count; ++i){
    if (i > 0) Serial.print(',');
    print_trace_time(static_cast<double>(env_time->pop()) / 1000.0);
  }
  Serial.print("],\"v\":[");
  for (uint16_t i = 0; i < count; ++i){
    if (i > 0) Serial.print(',');
    const int16_t centi_c = static_cast<int16_t>(env->pop() & 0xFFFFU);
    char value[16];
    ambit_trace_v3::format_leaf_temp(centi_c, value);
    Serial.print(value);
  }
  Serial.print("]}");
}

static void send_protocol_segments(const uint8_t* protocol, uint8_t segment_count){
  Serial.print("\"segments\":[");
  for (uint8_t i = 0; i < segment_count; ++i){
    if (i > 0) Serial.print(',');
    const ambit_trace_v3::Segment segment =
        ambit_trace_v3::decode_segment(protocol + static_cast<uint16_t>(i) * 8U);
    Serial.printf("{\"pulses\":%u,\"freq\":%u,\"actinic\":%u}",
                  segment.pulses, segment.freq, segment.actinic);
  }
  Serial.print(']');
}

static void send_trace_v3_json(uint8_t segment_count, const uint8_t* protocol,
                               uint32_t duration_ms, dataclass* d_env,
                               dataclass* d_env_time, dataclass* d_fluor,
                               dataclass* d_fluoRef, dataclass* d_sun,
                               dataclass* d_leaf, dataclass* d_730,
                               dataclass* d_730Ref){
  static_assert(sizeof(ambit_calibration_info_t) == 140,
                "cal_version must hash the frozen 140-byte calibration struct");
  const uint32_t cal_version = esp_rom_crc32_le(
      0, reinterpret_cast<const uint8_t*>(&ambit_calibration_local),
      sizeof(ambit_calibration_local));
  char sensor_id[18];
  ambit_trace_v3::format_sensor_id(ESP.getEfuseMac(), sensor_id);

  Serial.print("{\"schema\":\"ambit.trace/3\",\"sensor_id\":");
  print_json_string(sensor_id, sizeof(sensor_id));
  Serial.print(",\"device\":");
  print_json_string(ambit_calibration_local.ambit_name,
                    sizeof(ambit_calibration_local.ambit_name));
  Serial.printf(",\"time\":{\"duration_ms\":%lu},\"protocol\":{",
                static_cast<unsigned long>(duration_ms));
  send_protocol_segments(protocol, segment_count);
  Serial.printf(",\"cal_version\":\"%08lx\",\"tick_factor\":",
                static_cast<unsigned long>(cal_version));
  print_trace_time(ambit_calibration_local.tick_factor);
  Serial.printf(",\"gains\":[%u,%u,%u,%u,%u,%u]},\"series\":{",
                adpd_gains_config.Fluo, adpd_gains_config.FluoRef,
                adpd_gains_config.IR, adpd_gains_config.IRRef,
                adpd_gains_config.Sun, adpd_gains_config.Leaf);

  send_count_series("fluo_630_signal", d_fluor, protocol, segment_count,
                    ambit_trace_v3::SeriesClock::MAIN);
  Serial.print(',');
  send_count_series("fluo_630_ref", d_fluoRef, protocol, segment_count,
                    ambit_trace_v3::SeriesClock::MAIN);
  if (d_sun->available){
    Serial.print(',');
    send_count_series("ambient_sun_vis", d_sun, protocol, segment_count,
                      ambit_trace_v3::SeriesClock::AMBIENT);
  }
  if (d_leaf->available){
    Serial.print(',');
    send_count_series("ambient_leaf_ir", d_leaf, protocol, segment_count,
                      ambit_trace_v3::SeriesClock::AMBIENT);
  }
  if (d_730->available){
    Serial.print(',');
    send_count_series("refl_730_signal", d_730, protocol, segment_count,
                      ambit_trace_v3::SeriesClock::REFLECTION);
  }
  if (d_730Ref->available){
    Serial.print(',');
    send_count_series("refl_730_ref", d_730Ref, protocol, segment_count,
                      ambit_trace_v3::SeriesClock::REFLECTION);
  }
  Serial.print(',');
  send_leaf_temp_series(d_env, d_env_time);
  Serial.print("}}");
}

int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt, bool json_output, bool retain){
  ambit_trace_v3::RunCounts run_counts;
  if (!ambit_trace_v3::validate_run_protocol(
          arr, length, MAX_DATACLASS_SIZE - 1U, &run_counts)) return -1;

  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
    conf_slow_FR_1();
    ESP_LOGW(TAG, "Run array was not configured!");
  }
  // set to max possible bytes
  uint8_t expected_readout = 8;
  uint8_t expected_readout_bytes = expected_readout * 3;
  const uint8_t num_integration = 1;
  const unsigned int start_t0 = millis();
  const int64_t run_tick_begin = esp_timer_get_time();   // Change 3: TIMING - us at run start

  // Validation above computed in uint32_t and proved every value fits the
  // dataclass limit, so these narrowing conversions cannot wrap.
  uint16_t data_count[] = {
      static_cast<uint16_t>(run_counts.main),
      static_cast<uint16_t>(run_counts.ambient),
      static_cast<uint16_t>(run_counts.reflection),
      0,
  };
  ESP_LOGV(TAG, "Sample & Ref: %d, optionals: %d", data_count[0], data_count[1]);

  pam_run_buffers_t buffers;
  if (!buffers.allocated()) return -1;
  dataclass *d_env = buffers.d_env;
  dataclass *d_fluor = buffers.d_fluor;       // fluorescence signal
  dataclass *d_fluoRef = buffers.d_fluoRef;   // fluorescence reference
  dataclass *d_sun = buffers.d_sun;           // sun-side ambient
  dataclass *d_leaf = buffers.d_leaf;         // leaf-side ambient
  dataclass *d_730 = buffers.d_730;           // 730nm reflectance signal
  dataclass *d_730Ref = buffers.d_730Ref;     // 730nm reference
  dataclass *d_timing = buffers.d_timing;     // run begin/end ticks
  dataclass *d_env_time = buffers.d_env_time; // v3 env offsets from run start


  if (!(d_env->init(512))) return -1;
  if (!(d_env_time->init(512))) return -1;
  if (!(d_timing->init(2))) return -1;
  if (!( (d_fluor->init(data_count[0])) && (d_fluoRef->init(data_count[0])) )) return -1;
  if (data_count[1] > 0){
    if (!( (d_sun->init(data_count[1])) && (d_leaf->init(data_count[1])) )) return -1;}
  if (data_count[2] > 0){
    if (!( (d_730->init(data_count[2])) && (d_730Ref->init(data_count[2])) )) return -1;
  }


  ESP_LOGV(TAG, "Memory allocation completed");


  // variables for each trace
  uint8_t pc = 0;
  uint8_t _type = 0;
  uint8_t farred = 0, actinic = 0, subsampling = 0;
  uint16_t num_ptx = 0, freq = 0;

  // data counter and buffer
  // [sun-amb, leaf-ir, lit_leaf-ir, dark_leaf-ir, lit_leaf-ref, dark_leaf-ref]
  uint32_t ret[expected_readout] = {0};
  uint32_t counter = 0, ploter1 = 0, ploter2 = 0;
  uint16_t fifo_c = 0;
  uint8_t watch_dog_timer = 0;
  int32_t tmp_var = 0;
  uint32_t buf_opt[4] = {0};
  uint32_t _tmparr = 0;
  uint8_t _repeats = 1;
  uint32_t light_sleep_time = 1;
  float_t leaf_temp = 0.0;
  unsigned int env_timer1 = millis();
  bool measure_temperature = false;
  bool interrupt_run = false;

  _tmparr = PAM_get_env(4, start_t0);
  d_env->put(_tmparr);
  // The contract anchors the pre-loop sample to run start. Later samples use
  // the device clock directly, preserving their actual irregular cadence.
  d_env_time->put(0);
  leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;


  adpd.STOP();
  while (pc < length){
    if (interrupt_run) break;
    _type = *(arr + pc * 8);   //get line type
    if ((_type == 1) || (_type == 2)){  // all channels
      arr_line_parse_type1((arr + pc * 8), &farred, &num_ptx, &freq, &actinic, &subsampling, NULL);
      adpd.run_freq(freq);
      adpd.clear_fifo();
      light_sleep_time = (1000/freq);
      // Direct/openJII and Ambyte runs deliberately share this cadence. Keeping
      // separate policies made path A report one temperature while the same
      // protocol over the FSM reported the run's irregular env series.
      measure_temperature = (light_sleep_time > 20) && measure_temp && actinic < 50;


      if (_type == 1){ // use IR reflect
        if (farred == 1){ // whether use actinic IR   
          adpd.num_ts(9);
          _repeats = int(400/freq);
          if (freq < 3) _repeats = 250;
          if (_repeats == 0) _repeats = 1;
          for (uint8_t i = 3; i < 9;i++) adpd.repeats_only(i, 1, _repeats);
        }
        else{
          adpd.num_ts(3);
        }
        expected_readout = 8;
        expected_readout_bytes = expected_readout * 3;
      }else{ // NO IR
        adpd.num_ts(2);
        expected_readout = 6;
        expected_readout_bytes = expected_readout * 3;
      }

      if (actinic > 3){
        AS_LED_Current(actinic);
        AS_LED_ON();
      }else{
        AS_LED_OFF();
        AS_LED_Current(0);
      }


      counter = 0;
      for (uint8_t i = 0; i < 4; i++) buf_opt[i] = 0;
      adpd.RUN();
      delay(2);
      while (counter < num_ptx){
        if (interrupt_run) break;
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
          adpd.readfifo(expected_readout, 3, ret);
          fifo_c -= expected_readout_bytes;
          if (counter == num_ptx) break;
          // 0: sun-vis; 1: leaf-ir; 2: fluoS_dark; 3: fluoS_lit; 4: fluoR_dark; 5: fluoR_lit; 6: Reflect_signal; 7: reflect_ref
          // Apply each stored baseline exactly once at the result boundary.
          const uint32_t calibrated_fluor = apply_adpd_calibration(
              ambit_calibration::S630, calc_signal(ret[2], ret[3], num_integration));
          const uint32_t calibrated_fluo_ref = apply_adpd_calibration(
              ambit_calibration::R630, calc_signal(ret[4], ret[5], num_integration));
          const uint32_t calibrated_sun = apply_adpd_calibration(ambit_calibration::SUN, ret[0]);
          const uint32_t calibrated_leaf = apply_adpd_calibration(ambit_calibration::LEAF, ret[1]);
          const uint32_t calibrated_730 = _type == 1
              ? apply_adpd_calibration(ambit_calibration::S730, ret[6]) : 0;
          const uint32_t calibrated_730_ref = _type == 1
              ? apply_adpd_calibration(ambit_calibration::R730, ret[7]) : 0;
          d_fluor->put(calibrated_fluor);
          d_fluoRef->put(calibrated_fluo_ref);


          // save option data
          if (subsampling > 0){
            if (subsampling == 1){ // every point
              d_sun->put(calibrated_sun);
              d_leaf->put(calibrated_leaf);
              if (_type == 1){d_730->put(calibrated_730);d_730Ref->put(calibrated_730_ref);} // ir enabled

            }else if (subsampling == 2){
              buf_opt[0] += calibrated_sun;
              buf_opt[1] += calibrated_leaf;
              if (_type == 1){
                buf_opt[2] += calibrated_730;
                buf_opt[3] += calibrated_730_ref;
              }

              if (counter % 8 == 7){
                d_sun->put(buf_opt[0]/8);
                d_leaf->put(buf_opt[1]/8);
                if(_type == 1){d_730->put(buf_opt[2]/8);d_730Ref->put(buf_opt[3]/8);}                
                for (uint8_t i = 0; i < 4; i++) buf_opt[i] = 0;
              }
            }
          }

          if (CONNECTION_TYPE == CONNECTION_TYPES::PLOTTING){
            ploter1 = calibrated_fluor;
            ploter2 = calibrated_fluo_ref;
            if (ploter2 == 0){
              ploter2 = 1;
              ploter1 = 0;
            }
            if (_type == 1) {
              Serial.printf("T:%2.3f,F:%3.4f,S:%d,R:%d,7:%d,7R:%d,Sun:%d,L:%d\n", leaf_temp, (float)ploter1/(float)ploter2, ploter1, ploter2, calibrated_730, calibrated_730_ref, calibrated_sun, calibrated_leaf);
            }else if (_type == 2){
              Serial.printf("T:%2.3f,F:%3.4f,S:%d,R:%d,Sun:%d,L:%d\n", leaf_temp, (float)ploter1/(float)ploter2, ploter1, ploter2, calibrated_sun, calibrated_leaf);            }
            Serial.flush();
          }
          counter++;
          watch_dog_timer = 0;
        }

        // do light sleep
        //esp_sleep_enable_timer_wakeup(1000);
        if (counter + 10 < num_ptx){  // a lot of measurements
          // do temperature measurement?
          if (measure_temperature && (millis() - env_timer1 > 2000)){
            _tmparr = PAM_get_env(4, start_t0);
            d_env->put(_tmparr);
            d_env_time->put(millis() - start_t0);
            leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;
            env_timer1 = millis();
            esp_sleep_enable_timer_wakeup(1000);
          }
          else esp_sleep_enable_timer_wakeup(light_sleep_time * 8000);

        }else if (counter + 2 < num_ptx){ // not many 
          esp_sleep_enable_timer_wakeup(light_sleep_time * 1000);
        }else{
          esp_sleep_enable_timer_wakeup(1000);
        }
        // run interrupted by serial input "S"
        interrupt_run = PAM_interrupt(allow_interrupt, false);
        if (!interrupt_run) esp_light_sleep_start();
        interrupt_run = PAM_interrupt(allow_interrupt, true);
      }
      
      adpd.STOP();
      if (!led_persist) AS_LED_OFF();
      digitalWrite(1, LOW);
    }
    pc += 1;
  }

  if (interrupt_run){
    digitalWrite(STF_FLASH_PIN, LOW);
    adpd.STOP();
    AS_LED_OFF();
  };


  if (json_output) {
    // Path A emits the canonical v3 measurement object directly. Capture the
    // end tick before serialization so duration measures the sensor run, not
    // UART JSON transfer time, and diff in int64 before narrowing.
    const int64_t run_tick_end = esp_timer_get_time();
    send_trace_v3_json(length, arr,
                       ambit_trace_v3::duration_ms(run_tick_begin, run_tick_end),
                       d_env, d_env_time, d_fluor, d_fluoRef, d_sun, d_leaf,
                       d_730, d_730Ref);
  } else {
    d_timing->put((uint32_t) run_tick_begin);          // Change 3: run start tick (us)
    // Keep the frozen idx-7 capture at its original point. The int64 diff is a
    // path-A concern; moving this call earlier would perturb the binary value.
    d_timing->put((uint32_t) esp_timer_get_time());    //           run end tick (us)
    if (retain){
      // Parallel protocol: transfer ownership of the buffers to the async holder
      // instead of streaming + freeing. The host fetches them later (cmd 24).
      g_async.d_env = d_env; g_async.d_fluor = d_fluor; g_async.d_fluoRef = d_fluoRef;
      g_async.d_sun = d_sun; g_async.d_leaf = d_leaf; g_async.d_730 = d_730;
      g_async.d_730Ref = d_730Ref; g_async.d_timing = d_timing;
      g_async.d_env_time = d_env_time;
      g_async.subsampling = subsampling; g_async.has_730 = (data_count[2] > 0);
      g_async.allow_interrupt = allow_interrupt;
      buffers.release();  // holder owns the buffers now — do NOT delete
      return 0;
    }
    pam_send_results(d_env, d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref,
                     d_timing, d_env_time, subsampling, data_count[2] > 0,
                     allow_interrupt);
  }

  return 0;

}

// ── Async run-start / fetch (parallel trigger/poll/fetch protocol) ──────────
// Blocks to completion (like the synchronous run) but retains the results, so
// the host can trigger every sensor back-to-back and collect them afterwards.
int ambit_async_run_start(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt){
  ambit_async_clear();          // drop any stale, un-fetched result first
  int rc = run_arr_type1(length, arr, led_persist, allow_interrupt, false, true /*retain*/);
  if (rc == 0 && g_async.d_env != NULL){
    g_async.state = AMBIT_ASYNC_DONE;
  } else {
    ambit_async_clear();        // partial/failed run — free whatever stuck
    g_async.state = AMBIT_ASYNC_ERROR;
  }
  return rc;
}

// Stream the retained arrays to the ambyte over the existing AMBYTE FSM, then
// free them. Caller (cmd 24) has already written ESP_CMD_DONE and will write
// ESP_CMD_END after this returns.
int ambit_async_fetch(void){
  if (g_async.state != AMBIT_ASYNC_DONE || g_async.d_env == NULL) return -1;
  uint8_t saved = CONNECTION_TYPE;
  CONNECTION_TYPE = CONNECTION_TYPES::AMBYTE;
  pam_send_results(g_async.d_env, g_async.d_fluor, g_async.d_fluoRef, g_async.d_sun,
                   g_async.d_leaf, g_async.d_730, g_async.d_730Ref, g_async.d_timing,
                   g_async.d_env_time, g_async.subsampling, g_async.has_730,
                   g_async.allow_interrupt);
  CONNECTION_TYPE = saved;
  ambit_async_clear();
  return 0;
}


void adpd_trigger(void){
    digitalWrite(10, HIGH);
    delayMicroseconds(1);
    digitalWrite(10, LOW);
}



int run_trigger_spacer(uint16_t length, uint8_t interval, bool change_act, uint8_t act, bool interrrupt){

  if (length > 3000) return -1;
  adpd.STOP();

  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1) conf_slow_FR_1();

  digitalWrite(10, LOW);
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));
  adpd_mode = ADPD_CONFIG_MODE::ARRAY_SLOW;

  gpio_sleep_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
  gpio_sleep_set_pull_mode(GPIO_NUM_10, GPIO_PULLDOWN_ONLY);
  


  // set to max possible bytes
  uint8_t expected_readout = 8;
  uint8_t expected_readout_bytes = expected_readout * 3;
  const uint32_t _wait_time_ms = interval * 100;
  const uint8_t num_integration = 1;
  const unsigned int start_t0 = millis();
  const int64_t run_tick_begin = esp_timer_get_time();   // Change 3: TIMING - us at run start

    // data counter and buffer
  // [sun-amb, leaf-ir, lit_leaf-ir, dark_leaf-ir, lit_leaf-ref, dark_leaf-ref]
  uint32_t ret[expected_readout] = {0};
  uint16_t fifo_c = 0;
  uint8_t watch_dog_timer = 0;
  int32_t tmp_var = 0;
  uint32_t buf_opt[4] = {0};
  uint32_t _tmparr = 0;
  uint32_t read_fluor, read_fluoRef, read_sun, read_leaf, read_7, read_7Ref;
  float_t leaf_temp = 0.0;

  unsigned int env_timer1 = millis(), trigger_timer = 0, expected_millis = 0;
  int waiting_time = 0;
  bool measure_temperature = false;
  bool interrupt_run = false;
  esp_sleep_enable_timer_wakeup(90000);

  dataclass *d_env = new dataclass; //
  dataclass *d_fluor = new dataclass; // fluorescence signal
  dataclass *d_fluoRef = new dataclass; // fluorescence reference

  dataclass *d_sun = new dataclass; // sun-side ambient
  dataclass *d_leaf = new dataclass;  // leaf-side ambient
  dataclass *d_730 = new dataclass; // 730nm reflectance signal
  dataclass *d_730Ref = new dataclass;  // 730nm reference
  dataclass *d_timing = new dataclass;  // Change 3: [tick_begin, tick_end] controller ticks (us)

  int _func_ret = -1;
  if (!(d_env->init(512))) goto del_classes;
  if (!(d_timing->init(2))) goto del_classes;
  if (!( (d_fluor->init(length)) && (d_fluoRef->init(length)) )) goto del_classes;
  if (!( (d_sun->init(length)) && (d_leaf->init(length)) )) goto del_classes;
  if (!( (d_730->init(length)) && (d_730Ref->init(length)) )) goto del_classes;
  

  _tmparr = PAM_get_env(4, start_t0);
  d_env->put(_tmparr);
  leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;
  env_timer1 = millis();


  adpd.clear_fifo();
  adpd.run_freq(10);
  adpd.num_ts(3);
  adpd.RUN();
  delay(5);

  if(change_act){
    if (act == 0) AS_LED_OFF();
    if (act > 0) {
      AS_LED_Current(act);
      AS_LED_ON();
      }
  }
  
  for (uint16_t n = 0; n < length; n++){
    fifo_c = 0;
    adpd_trigger();
    trigger_timer = millis();
    expected_millis = trigger_timer + _wait_time_ms;
    delay(1);
    while (fifo_c != expected_readout_bytes){
      fifo_c = adpd.fifo_count();
      if (fifo_c >= expected_readout_bytes) break;
      if (millis() - trigger_timer > 100) break;
    }
    if (fifo_c < expected_readout_bytes){
      ESP_LOGE(TAG, "NOT ENOUGH IN FIFO");
      break;
    }
    adpd.readfifo(expected_readout, 3, ret);
    if (fifo_c > expected_readout_bytes){
      adpd.clear_fifo();
      ESP_LOGE(TAG, "Extra %d byte in FIFO", fifo_c - expected_readout_bytes);
    }

    read_fluor = apply_adpd_calibration(ambit_calibration::S630, calc_signal(ret[2], ret[3], num_integration)); d_fluor->put(read_fluor);
    read_fluoRef = apply_adpd_calibration(ambit_calibration::R630, calc_signal(ret[4], ret[5], num_integration)); d_fluoRef->put(read_fluoRef);
    read_sun = apply_adpd_calibration(ambit_calibration::SUN, ret[0]); d_sun->put(read_sun);
    read_leaf = apply_adpd_calibration(ambit_calibration::LEAF, ret[1]); d_leaf->put(read_leaf);
    read_7 = apply_adpd_calibration(ambit_calibration::S730, ret[6]); d_730->put(read_7);
    read_7Ref = apply_adpd_calibration(ambit_calibration::R730, ret[7]); d_730Ref->put(read_7Ref);


    if (CONNECTION_TYPE == CONNECTION_TYPES::PLOTTING){
      Serial.printf("T:%2.3f,F:%3.4f,S:%d,R:%d,7:%d,7R:%d,Sun:%d,L:%d\n", leaf_temp, (float)read_fluor/(float)read_fluoRef, read_fluor, read_fluoRef, read_7, read_7Ref, read_sun, read_leaf);
      Serial.flush();
    }



    if (interrupt_run) break;
    if (millis() > expected_millis) continue; // overdue
    waiting_time = expected_millis - millis();

    if ((n % 8 == 7) && (waiting_time > 100)){
      if (millis() - env_timer1 > 2000){
        _tmparr = PAM_get_env(4, start_t0);
        d_env->put(_tmparr);
        leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;
        env_timer1 = millis();
      }      
    }    

    

    
    waiting_time = expected_millis - millis();
    while (waiting_time > 250){
      esp_sleep_enable_timer_wakeup((waiting_time - 50) * 1000);
      interrupt_run = PAM_interrupt(interrrupt, false);
      if (!interrupt_run) esp_light_sleep_start();
      interrupt_run = PAM_interrupt(interrrupt, true);
      if (interrupt_run) break;
      waiting_time = expected_millis - millis();
    }
    if (interrupt_run) break;
    waiting_time = expected_millis - millis();
    if (waiting_time > 1) delay(waiting_time);     
  } /// End of Loop


  adpd.STOP();
  adpd.clear_fifo();

  // Trigger-spacer always produces the full 7-channel set (subsampling=1, has_730=true),
  // sent unconditionally. This path never runs as COMPUTER, so pam_send_results' COMPUTER
  // branch is inert here; the AMBYTE branch is identical to the former inline block.
  d_timing->put((uint32_t) run_tick_begin);          // Change 3: run start tick (us)
  d_timing->put((uint32_t) esp_timer_get_time());    //           run end tick (us)
  pam_send_results(d_env, d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref,
                   d_timing, NULL, 1, true, interrrupt);

  _func_ret = 0;
  del_classes:
    delete d_fluor;
    delete d_fluoRef;
    delete d_sun;
    delete d_leaf;
    delete d_730;
    delete d_730Ref;
    delete d_env;
    delete d_timing;
    
  adpd.gpio_config.GPIO0_cfg = 0;
  adpd.gpio_config.EXT_SYNC_EN = 0;
  adpd.gpio_setup(&(adpd.gpio_config));
  adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;        
  return _func_ret;
}


int external_trigger_run(void){

  adpd.STOP();
  conf_slow_FR_1();

  adpd.led_config.driver1_current = adpd_current_config.I620;
  adpd.led_config.led1_channel = LED_A;
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;
  adpd.SNR_config.TIA_gain_CH1 = adpd_gains_config.Fluo;
  adpd.SNR_config.TIA_gain_CH2 = adpd_gains_config.FluoRef;
  adpd.preset_config_2(1, 4);


  digitalWrite(10, LOW);
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));


  // set to max possible bytes
  uint8_t expected_readout = 6, num_integration = 4;
  uint8_t expected_readout_bytes = expected_readout * 3;
  
    // data counter and buffer
  // [sun-amb, leaf-ir, lit_leaf-ir, dark_leaf-ir, lit_leaf-ref, dark_leaf-ref]
  uint32_t ret[expected_readout] = {0};
  uint16_t fifo_c = 0;  
  uint32_t read_fluor, read_fluoRef, read_sun, read_leaf;
  float_t leaf_temp = 0.0;

  adpd.clear_fifo();
  adpd.run_freq(10);
  adpd.num_ts(2);
  adpd.RUN();
  delay(5);
  uint8_t unknown_input_counter = 0;

  unsigned int watchdog_timer = millis(), trigger_timer = 0, start_timer = millis(), temp_timer = millis();
  bool keep_running = true, do_measure = false, change_act = false;
  double obj_T, chip_T;
  char c, c1, c2;

  Serial.println("Run");

  while(keep_running){ 
    
    while (Serial.available() == 0){
      if (millis() - watchdog_timer > 30000){
        keep_running = false;
        break;
      }
      delayMicroseconds(200);
    }
    if (!keep_running) break;
    
    while (Serial.available() > 0){
      c = Serial.read();
      if (c == 'G'){
        do_measure = true;
        watchdog_timer = millis();
      }else if(c == 'E'){
        do_measure = false;
        keep_running = false;
      }else if(c == 'A'){
        delay(1);
        c1 = 0;
        if (Serial.available() > 0){
          c1 = Serial.read();
          change_act = true;
        }
      }else if(c == 'T'){
        temp_timer = millis();
        mlx_measure(&obj_T, &chip_T);
        Serial.printf("T:%d,o:%.3f,a:%.3f,d:%d\n", millis() - start_timer, obj_T, chip_T, millis() - temp_timer);
        Serial.flush();
        watchdog_timer = millis();
        continue;
      }
      else{
        unknown_input_counter += 1;
        if (unknown_input_counter > 200) keep_running = false;
      }
    }

    if (!keep_running) break;
    if (change_act){
      change_act = false;
      if (c1 > 3){
        AS_LED_Current(c1);
        AS_LED_ON();
      }else{
        AS_LED_OFF();
      }
    }
    if (!do_measure) continue;


    fifo_c = 0;
    adpd_trigger();
    do_measure = false;
    trigger_timer = millis(); 
    delay(1);
    while (fifo_c != expected_readout_bytes){
      fifo_c = adpd.fifo_count();
      if (fifo_c >= expected_readout_bytes) break;
      if (millis() - trigger_timer > 100) break;
    }
    if (fifo_c < expected_readout_bytes){
      Serial.println("NOT ENOUGH IN FIFO");
      break;
    }
    adpd.readfifo(expected_readout, 3, ret);
    if (fifo_c > expected_readout_bytes){
      adpd.clear_fifo();
      Serial.printf("Extra %d byte in FIFO", fifo_c - expected_readout_bytes);
    }

    read_fluor = apply_adpd_calibration(ambit_calibration::S630, calc_signal(ret[2], ret[3], num_integration));
    read_fluoRef = apply_adpd_calibration(ambit_calibration::R630, calc_signal(ret[4], ret[5], num_integration));
    read_sun = apply_adpd_calibration(ambit_calibration::SUN, ret[0]);
    read_leaf = apply_adpd_calibration(ambit_calibration::LEAF, ret[1]);



    Serial.printf("T:%d,S:%d,R:%d,F:%d,B:%d\n", millis() - start_timer, read_fluor, read_fluoRef, read_sun, read_leaf);
    Serial.flush();    
  } /// End of Loop


  adpd.STOP();
  adpd.clear_fifo();
    
  adpd.gpio_config.GPIO0_cfg = 0;
  adpd.gpio_config.EXT_SYNC_EN = 0;
  adpd.gpio_setup(&(adpd.gpio_config));

  AS_LED_Current(0);
  AS_LED_OFF();

  Serial.println("Stop");

  return 0;
}


int external_trigger_run_Flash(unsigned int gate_time, unsigned int dt, const uint16_t num){
  adpd.STOP();
  const uint8_t _NUM_TS = 8;

  adpd.led_config.driver1_current = 80;
  adpd.led_config.led1_channel = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;
  adpd.SNR_config.TIA_gain_CH1 = 1;
  adpd.SNR_config.TIA_gain_CH2 = 5;
  
  for (uint8_t i = 0; i < 12; i++){
    adpd.preset_config_ext_fast(i, 2);
  }

  digitalWrite(STF_FLASH_PIN, LOW);
  digitalWrite(10, LOW);
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));
  AS_LED_OFF();
  AS_LED_Current(dt);


  // set to max possible bytes
  uint8_t expected_readout = _NUM_TS * 4, num_integration = 2;
  uint8_t expected_readout_bytes = expected_readout * 3;
  
    // data counter and buffer

  uint32_t ret[expected_readout] = {0};
  uint16_t fifo_c = 0;  
  uint32_t read_fluor, read_fluoRef, read_sun, read_leaf;
  float_t leaf_temp = 0.0;

  adpd.clear_fifo();
  adpd.run_freq(10);
  adpd.num_ts(_NUM_TS);
  adpd.RUN();
  delay(5);
  uint16_t _counter = 0;

  unsigned int watchdog_timer = millis(), trigger_timer = 0, start_timer = millis(), temp_timer = millis();
  bool keep_running = true, do_measure = false, change_act = false;
  double obj_T, chip_T;
  char c, c1, c2;

  dataclass *d_fluor = new dataclass; // fluorescence signal
  dataclass *d_fluoRef = new dataclass; // fluorescence reference
  
  

  Serial.println("Run");
  unsigned long timer1 = micros();
  unsigned long timer2 = micros();
  unsigned int flash_duration = expected_readout;
  uint16_t _num_sample = num < 20 ? num * _NUM_TS : 20 * _NUM_TS + (num - 20);



  if (!( (d_fluor->init(_num_sample + 1)) && (d_fluoRef->init(_num_sample + 1)) )) goto del_classes;
  while(keep_running){
    _counter += 1;
    if (_counter > num) break;
    if (!keep_running) break;
    fifo_c = 0;
    //if ((_counter == 5) && (dt > 0)) AS_LED_ON();
    adpd_trigger();    
    timer1 = micros();
    delayMicroseconds(1000);
    fifo_c = adpd.fifo_count();
    while (fifo_c != expected_readout_bytes){
      fifo_c = adpd.fifo_count();
      if (fifo_c >= expected_readout_bytes) break;
      if (micros() - timer1 > 10000) break;
    }
    //Serial.printf("%ld: %d\n", micros() - timer1, fifo_c);

    if (fifo_c < expected_readout_bytes){
      Serial.printf("NOT ENOUGH IN FIFO: %d < %d\n", fifo_c, expected_readout_bytes);
      break;
    }
    

    if ((gate_time > 0) && (flash_duration > 0) && (_counter > 2)) digitalWrite(STF_FLASH_PIN, HIGH);
    //if ((_counter == 15) && (dt > 0)) AS_LED_OFF();

    timer1 = micros();
    for (uint8_t r = 0; r < expected_readout; r++){
      adpd.readfifo(1, 3, ret + r);
      if (flash_duration == r) digitalWrite(STF_FLASH_PIN, LOW);
    }
    //Serial.println(micros() - timer1);
    
    
    if (_counter <= 20){
      for (uint8_t r = 0; r < _NUM_TS; r++){
        d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
            calc_signal(ret[0 + r * 4], ret[1 + r * 4], num_integration)));
        d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
            calc_signal(ret[2 + r * 4], ret[3 + r * 4], num_integration)));
      }
    }else{
      read_fluor = 0; read_fluoRef = 0;
      for (uint8_t r = 0; r < _NUM_TS; r++){
        read_fluor += calc_signal(ret[0 + r * 4], ret[1 + r * 4], num_integration);
        read_fluoRef += calc_signal(ret[2 + r * 4], ret[3 + r * 4], num_integration);
      }
      d_fluor->put(apply_adpd_calibration(ambit_calibration::S630, read_fluor / _NUM_TS));
      d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630, read_fluoRef / _NUM_TS));
      if (_counter == 100) flash_duration = 4;
      if (_counter == 300) flash_duration = 2;
      if (_counter == 500) flash_duration = 1;
      if (_counter == 700) flash_duration = 32;
    }


    if (fifo_c > expected_readout_bytes){
      fifo_c = adpd.fifo_count();
      if (fifo_c > 0){
        adpd.clear_fifo();
        digitalWrite(STF_FLASH_PIN, LOW);
        Serial.printf("Extra %d byte in FIFO\n", fifo_c - expected_readout_bytes);
        }      
    }
    
    // flash_duration = gate_time - (micros() - timer1);    
    // if ((flash_duration > 1) && (flash_duration < 500)) delayMicroseconds(flash_duration - 1);
    // digitalWrite(STF_FLASH_PIN, LOW);
    //

    
    //Serial.printf("T:%d,S:%d,R:%d,F:%d,B:%d\n", millis() - start_timer, read_fluor, read_fluoRef, read_sun-65000, read_leaf-65000);    
    // Serial.printf("T:%d,S:%d,R:%d\n", millis() - start_timer, read_fluor, read_fluoRef);    
    // flash_duration = gate_time - (micros() - timer1);
    // if (flash_duration > 2){
    //   digitalWrite(STF_FLASH_PIN, HIGH);
    //   delayMicroseconds(flash_duration - 1);
    // }



    digitalWrite(STF_FLASH_PIN, LOW);
  } /// End of Loop
  //Serial.printf("%f for %dpoints\n", float(micros() - timer2)/num, num);
  AS_LED_OFF();
  digitalWrite(STF_FLASH_PIN, LOW);
  adpd.STOP();
  adpd.clear_fifo();

  
    
  adpd.gpio_config.GPIO0_cfg = 0;
  adpd.gpio_config.EXT_SYNC_EN = 0;
  adpd.gpio_setup(&(adpd.gpio_config));

  AS_LED_Current(0);
  AS_LED_OFF();
  digitalWrite(STF_FLASH_PIN, LOW);

  for (uint16_t n = 2; n < _num_sample; n++){
    Serial.printf("%d,%d,%.4f\n", d_fluor->arr[n], d_fluoRef->arr[n], ((float) d_fluor->arr[n]) / d_fluoRef->arr[n]);
  }
  
  del_classes:
    delete d_fluor;
    delete d_fluoRef;
  

  return 0;
}




int MPF(uint16_t mode, uint16_t dc_current){
  if (adpd_gains_config.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
  if (adpd_current_config.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");
  return MPF(mode, adpd_current_config.I620, dc_current, adpd_gains_config.Fluo, adpd_gains_config.FluoRef);

}


int MPF(uint16_t mode, uint16_t current, uint16_t dc_current, uint8_t sign_gain, uint8_t ref_gain){

  
  const uint8_t num_integration = 1;
  const uint16_t _data_size = 1080;
  
  ESP_LOGV(TAG, "RUN MPF with mode:%d, pulse current:%d, DC current %d", mode, current, dc_current);

  dataclass *d_fluor = new dataclass; // fluorescence signal
  dataclass *d_fluoRef = new dataclass; // fluorescence reference
  if (!( (d_fluor->init(_data_size)) && (d_fluoRef->init(_data_size)) )) return -1;
  ESP_LOGV(TAG, "Memory allocation completed");


  adpd.STOP();
  adpd.run_freq(10);
  adpd.clear_fifo();
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));

  adpd.led_config.driver1_current = current;
  adpd.led_config.driver2_current = 0;
  adpd.SNR_config.TIA_gain_CH2 = ref_gain;
  adpd.SNR_config.TIA_gain_CH1 = sign_gain;

  adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;


   // data buffers
  uint32_t ret[48] = {0};
  uint16_t fifo_c = 0;
  int32_t tmp_var = 0;
  uint32_t avg_arr1[12] = {0};
  uint32_t avg_buf[4] = {0};
  uint16_t as_current = 255;
  uint8_t expected_readout = 12 * 4;    // 12 timeslots, 4 data per ts
  uint16_t decay_interval = 1;
  // PHASE 0----------------------------------------------
  // Apply a baseline without actinic
  // fixed at 2 Hz x 20 pts
  if (mode == 0){
    AS_LED_OFF();
    AS_LED_Current(as_current);
    adpd.preset_config_ext_fast(0);
    expected_readout = 4;
    adpd.RUN();
    delayMicroseconds(1500);
    adpd_trigger();
    delayMicroseconds(1500);
    ESP_LOGV(TAG, "Phase-0 Started");
    for (uint8_t i = 0; i < 20; i++){
      adpd_trigger(); 
      adpd.readfifo(expected_readout, 3, ret); 
      for (uint8_t m = 0; m < 4; m++){    // sign-dark, sign-lit, ref-dark, ref lit
        avg_buf[m] = ret[m];
      }
      d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
          calc_signal(avg_buf[0], avg_buf[1], num_integration)));
      d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
          calc_signal(avg_buf[2], avg_buf[3], num_integration)));
      delay(500);
    }
    ESP_LOGV(TAG, "Phase-0 Completed");
  }

  
  // PHASE 1---------------------------------------------------
  // 12 timeslots
  // Rapid induction, all timeslot saved
  ESP_LOGV(TAG, "Phase-1 Config");
  expected_readout = 48;
  for (uint8_t i = 0; i < 12; i++){
    adpd.preset_config_ext_fast(i);
  }
  adpd.RUN();
  // AS Light kept until this point
  AS_LED_OFF();
  AS_LED_Current(0);
  delay(2);
  ESP_LOGV(TAG, "Phase-1 LED ON");
  // Light ON
  AS_LED_ON();
  delayMicroseconds(1);  
  adpd_trigger();
  delayMicroseconds(1500);

  for (uint8_t i = 0; i < 4; i++){
    adpd_trigger(); 
    adpd.readfifo(expected_readout, 3, ret); 
    for (uint8_t j = 0; j < 12; j++){

      d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
          calc_signal(ret[0 + j * 4], ret[1 + j * 4], num_integration)));
      d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
          calc_signal(ret[2 + j * 4], ret[3 + j * 4], num_integration)));
    }
    delayMicroseconds(500);
  }
  ESP_LOGV(TAG, "Phase-1 Completed; Phase-2 start");

  // PHASE 2------------------------------------
  // 300ms induction, 200x12ts, 200 final pts
  for (uint8_t i = 0; i < 200; i++){
    adpd_trigger(); 
    adpd.readfifo(expected_readout, 3, ret); 
    // reset avg arrays
    for (uint8_t m = 0; m < 4; m++){    // find median for sign-dark, sign-lit, ref-dark, ref lit
      memset(avg_arr1, 0, sizeof(avg_arr1));
      for (uint8_t n = 0; n < 12; n++){   // put 12 ts into sorting array
        sorted_insert(avg_arr1, 12, ret[m + n * 4]);
      }
      avg_buf[m] = (avg_arr1[5] + avg_arr1[6] + avg_arr1[7]) / 3; // pick the middle 3 numbers and average
    }


    // Serial.printf("P2, %d, %d, %d\n", avg_buf[0], avg_buf[1], calc_signal(avg_buf[0], avg_buf[1], num_integration));
    // Serial.printf("P2, %d, %d, %d\n", avg_buf[2], avg_buf[3], calc_signal(avg_buf[2], avg_buf[3], num_integration));

   
    d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
        calc_signal(avg_buf[0], avg_buf[1], num_integration)));
    // Serial.println(d_fluor->arr[d_fluor->write_ptr - 1]);
    d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
        calc_signal(avg_buf[2], avg_buf[3], num_integration)));
    // Serial.println(d_fluoRef->arr[d_fluoRef->write_ptr - 1]);
    delayMicroseconds(500);
  }

  ESP_LOGV(TAG, "Phase-2 Completed; Phase-3 Start");


  // PHASE 3------------------------------------
  // 150ms induction X cycles, 100x12ts, 50 final pts
  for (uint8_t j = 0; j < 8; j++){
    AS_LED_Current(220 - j * 30);
    ESP_LOGV(TAG, "LED set to %d", 220 - j * 30);
    memset(avg_buf, 0, sizeof(avg_buf));
    for (uint8_t i = 0; i < 40; i++){
      adpd_trigger(); 
      adpd.readfifo(expected_readout, 3, ret);
      // reset avg arrays
      for (uint8_t m = 0; m < 4; m++){    // find median for sign-dark, sign-lit, ref-dark, ref lit
        memset(avg_arr1, 0, sizeof(avg_arr1));
        for (uint8_t n = 0; n < 12; n++){   // put 12 ts into sorting array
          sorted_insert(avg_arr1, 12, ret[m + n * 4]);
        }
        avg_buf[m] += (avg_arr1[5] + avg_arr1[6] + avg_arr1[7]) / 3; // pick the middle 3 numbers and average
      }

      if (i % 2 == 1){
        d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
            calc_signal(avg_buf[0] / 2, avg_buf[1] / 2, num_integration)));
        d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
            calc_signal(avg_buf[2] / 2, avg_buf[3] / 2, num_integration)));
        memset(avg_buf, 0, sizeof(avg_buf));
      }   
      delayMicroseconds(500);
    }    
  }

  ESP_LOGV(TAG, "Phase-3 Completed; Phase-4 Start");
  // PHASE 4------------------------------------
  AS_LED_Current(as_current);
  // prepare for single timeslot decay kinetic
  expected_readout = 4;
  adpd.preset_config_ext_fast(0);
  adpd.clear_fifo();
  adpd.RUN();
  delayMicroseconds(1000);
  adpd_trigger();
  delayMicroseconds(1500);
  memset(avg_buf, 0, sizeof(avg_buf));

  for (uint8_t i = 0; i < 200; i++){
    adpd_trigger(); 
    adpd.readfifo(expected_readout, 3, ret); 
    for (uint8_t m = 0; m < 4; m++){    // sign-dark, sign-lit, ref-dark, ref lit
      avg_buf[m] += ret[m];
    }
    if (i % 4 == 3){



      // Serial.printf("P4, %d, %d, %d\n", avg_buf[0]/4, avg_buf[1]/4, calc_signal(avg_buf[0]/4, avg_buf[1]/4, num_integration));
      // Serial.printf("P4, %d, %d, %d\n", avg_buf[2]/4, avg_buf[3]/4, calc_signal(avg_buf[2]/4, avg_buf[3]/4, num_integration));


      d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
          calc_signal(avg_buf[0] / 4, avg_buf[1] / 4, num_integration)));
      // Serial.println(d_fluor->arr[d_fluor->write_ptr - 1]);
      d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
          calc_signal(avg_buf[2] / 4, avg_buf[3] / 4, num_integration)));
      // Serial.println(d_fluoRef->arr[d_fluoRef->write_ptr - 1]);
      memset(avg_buf, 0, sizeof(avg_buf));
    }
    if (i == 149) {
      if (mode == 0) AS_LED_OFF();        // dark mode, actinic OFF
      if (mode == 1) AS_LED_Current(dc_current);  // light mode, actinic set
    }
    delayMicroseconds(1400);
  }

  ESP_LOGV(TAG, "Phase-4 Completed");
// PHASE -1 ---------------------------
// Dark decay with increasing interval
  if (mode == 0){
    ESP_LOGV(TAG, "Phase-minus1 Started");
    AS_LED_OFF();
    for (uint8_t i = 0; i < 160; i++){
      adpd_trigger(); 
      adpd.readfifo(expected_readout, 3, ret); 
      for (uint8_t m = 0; m < 4; m++){    // sign-dark, sign-lit, ref-dark, ref lit
        avg_buf[m] += ret[m];
      }
      if (i % 4 == 3){
        d_fluor->put(apply_adpd_calibration(ambit_calibration::S630,
            calc_signal(avg_buf[0] / 4, avg_buf[1] / 4, num_integration)));
        d_fluoRef->put(apply_adpd_calibration(ambit_calibration::R630,
            calc_signal(avg_buf[2] / 4, avg_buf[3] / 4, num_integration)));
        memset(avg_buf, 0, sizeof(avg_buf));
        decay_interval += 5;
      }
      delayMicroseconds(1400);
      delay(decay_interval);
    }
  }

  ESP_LOGV(TAG, "Measurement Completed");
  // Completed

  adpd.STOP();

  if (CONNECTION_TYPE == CONNECTION_TYPES::PLOTTING){
    uint16_t l = d_fluor->get_length();
    uint32_t ploter1, ploter2;
    for (uint16_t i = 0; i < l; i++){
      ploter1 = d_fluor->pop();
      ploter2 = d_fluoRef->pop();
      Serial.printf("F:%3.4f,S:%d,R:%d\n", (float)ploter1/(float)ploter2, ploter1, ploter2);      
    }   
  }else if (CONNECTION_TYPE == CONNECTION_TYPES::AMBYTE){
    d_fluor->fsm_send_esp(0);
    d_fluoRef->fsm_send_esp(1);
  }
  else{
    d_fluor->send_serial("s_630");
    d_fluoRef->send_serial("r_630");
    Serial.println("Data sent");
  }

  ESP_LOGV(TAG, "All Completed");


  adpd.gpio_config.GPIO0_cfg = 0;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 0;
  adpd.gpio_setup(&(adpd.gpio_config));
  
  delete d_fluor;
  delete d_fluoRef;
  return 0;
}


/* Leaf temperature as centi-degC (signed int16 in the low 16 bits). The old
 * time/type bit-packing is gone: env arrays sent to the ambyte are now a plain
 * centi-degC series. Absolute time comes from the ambyte RTC; within-run point
 * timing from the run freq. Only mode 4 (leaf temp) is emitted now; mode/t0 are
 * kept so call sites are unchanged. */
uint32_t PAM_get_env(uint8_t mode, unsigned int t0){
  (void) t0;
  if (mode == 4){
    int16_t centi = (int16_t) (mlx_measure() * 100.0);
    return (uint32_t) (uint16_t) centi;
  }
  return 0;
}

uint32_t PAM_retrieve_env(uint32_t r, uint8_t* mode, float_t* data_f, int16_t* data_i){
  int16_t data = (int16_t) (r & 0x00000FFF);
  uint8_t d_type = (uint8_t) ((r & 0x0000F000) >> 12);
  uint32_t t = ((r & 0xFFFF0000) >> 10);

  if (d_type < 4){
    t += data;
    if (mode != NULL) *mode = d_type;
  }else if (d_type == 4){ // temperature
    if (data_f != NULL) *data_f = (data / 20.0 - 20);
    if (mode != NULL) *mode = d_type;  }
  
  
  return t;
}





int sandbox(uint8_t I620, uint8_t g1, uint8_t g2){

  
  // variables for each trace

  // data counter and buffer
  // [sun-amb, leaf-ir, lit_leaf-ir, dark_leaf-ir, lit_leaf-ref, dark_leaf-ref]
  uint32_t ret[16] = {0};
  uint16_t fifo_c = 0;
  uint16_t fifo_c1 = 0;


  adpd.STOP();
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));

  adpd.led_config.driver1_current = I620;
  adpd.led_config.driver2_current = 0;
  adpd.SNR_config.TIA_gain_CH2 = 5;
  adpd.SNR_config.TIA_gain_CH1 = 1;

  adpd.preset_config_ext_fast(0, 1);
  adpd.preset_config_ext_fast(1, 2);
  adpd.preset_config_ext_fast(2, 3);
  adpd.preset_config_ext_fast(3, 4);
  
  adpd.run_freq(10);
  adpd.clear_fifo();

  adpd.RUN();
  delay(1);
  int64_t timer = 0;


  for (uint16_t i = 0; i < 200; i++){
    digitalWrite(10, HIGH);
    delayMicroseconds(1);
    digitalWrite(10, LOW);
    timer = esp_timer_get_time();
    if (i > 0){
      
      adpd.readfifo(16, 3, ret);

      Serial.printf("Data:%d,%d,%d,%d,", ret[0], ret[1], ret[2],ret[3]);
      Serial.printf("%d,%d,%d,%d,", ret[4], ret[5], ret[6],ret[7]);
      Serial.printf("%d,%d,%d,%d,", ret[8], ret[9], ret[10],ret[11]);
      Serial.printf("%d,%d,%d,%d\n", ret[12], ret[13], ret[14],ret[15]);

      
      delay(100);
      
    }else{
      delayMicroseconds(1500);
    }
    //Serial.println(esp_timer_get_time() - timer);

  }
  //Serial.println(adpd.fifo_count());
  adpd.readfifo(16, 3, ret);

  adpd.STOP();   


  return 0;

}



static bool PAM_interrupt(bool enable, bool check_sleep){
  uint8_t ret = 0;
  if (!enable) return false;
  if (check_sleep){ //  after light sleep
    if (esp_sleep_get_wakeup_cause() == 8){ //  wake up serial
      Serial.flush();
      ret = serial_read_until(177, 0, 0, 25, true);
    }
  }else{
    ret = serial_read_until(177, 0, 0, 15, true);
  }
  if (ret == 1) return true;
  return false;
}


void actinic_test(uint8_t act1, uint8_t act2, uint8_t t1, uint8_t t2){
  AS_LED_Current(act1);
  AS_LED_ON();
  delay(t1 * 1000);
  AS_LED_Current(act2);
  delay(3000);
  AS_LED_OFF();            
  AS_LED_Current(0);
  

}




int fluor_offset_test(uint8_t current, uint8_t num_integ, uint8_t lit_offset, uint8_t dark1_offset, uint8_t dark2_offset, uint8_t pulse_offset, uint8_t pulse_duration){

    // Setup timeslot 2:  Fluor and Ref channels,  4 x 3 bytes
    // LED 1A = 620nm
  adpd.led_config.driver1_current = current;
  adpd.led_config.led1_channel = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;
  adpd.SNR_config.TIA_gain_CH1 = 1;
  adpd.SNR_config.TIA_gain_CH2 = 5;
  adpd.preset_config_2x(0, num_integ, lit_offset, dark1_offset, dark2_offset, pulse_offset, pulse_duration);

  adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;

  return 0;
}


int fluor_offset(uint32_t* fret){
  if (fret == nullptr) return FLUOR_OFFSET_INVALID_ARGUMENT;

  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
    const int config_result = conf_slow_FR_1();
    if (config_result != jii::adpd6000::kOk) return FLUOR_OFFSET_CONFIG_ERROR;
    ESP_LOGW(TAG, "Run array was not configured!");
  }

  constexpr uint8_t expected_readout = 8;
  constexpr uint16_t expected_readout_bytes = expected_readout * 3;
  constexpr uint8_t num_integration = 1;
  constexpr uint16_t num_points = 64;
  constexpr uint8_t repeat_count = 4;
  // Four 64-point blocks at 100 Hz normally take about 2.6 seconds. Five
  // seconds tolerates scheduling/FIFO jitter but bounds a missing or failed
  // sensor. wrap_safe_timeout_elapsed() remains correct across millis() wrap.
  constexpr uint32_t acquisition_timeout_ms = 5000U;
  const uint32_t acquisition_start = millis();

  auto fail_after_stop = [](int result) {
    adpd.STOP();
    return result;
  };

  if (adpd.STOP() != jii::adpd6000::kOk) return FLUOR_OFFSET_ADPD_ERROR;
  if (adpd.run_freq(100) != jii::adpd6000::kOk ||
      adpd.num_ts(3) != jii::adpd6000::kOk){
    return fail_after_stop(FLUOR_OFFSET_ADPD_ERROR);
  }
  AS_LED_OFF();
  AS_LED_Current(0);

  uint32_t ret_fluor = 0, ret_fluoRef = 0, ret_sun = 0;
  uint32_t ret_leaf = 0, ret_r730 = 0, ret_r730Ref = 0;

  for (uint8_t repeat = 0; repeat < repeat_count; ++repeat){
    // STOP does not guarantee the FIFO is empty. Clear it for every block so
    // a queued frame from the prior block cannot bias the next average.
    if (adpd.clear_fifo() != jii::adpd6000::kOk ||
        adpd.RUN() != jii::adpd6000::kOk){
      return fail_after_stop(FLUOR_OFFSET_ADPD_ERROR);
    }
    delay(1);

    uint32_t fluor = 0, fluoRef = 0, sun = 0;
    uint32_t leaf = 0, r730 = 0, r730Ref = 0;
    uint16_t counter = 0;

    while (counter < num_points){
      if (ambit_calibration::wrap_safe_timeout_elapsed(
              acquisition_start, millis(), acquisition_timeout_ms)){
        return fail_after_stop(FLUOR_OFFSET_TIMEOUT);
      }

      uint16_t fifo_count = 0;
      if (adpd.fifo_count(&fifo_count) != jii::adpd6000::kOk){
        return fail_after_stop(FLUOR_OFFSET_ADPD_ERROR);
      }
      if (fifo_count < expected_readout_bytes){
        delay(1);
        continue;
      }

      while (fifo_count >= expected_readout_bytes && counter < num_points){
        uint32_t samples[expected_readout] = {0};
        if (adpd.readfifo(expected_readout, 3, samples) != jii::adpd6000::kOk){
          return fail_after_stop(FLUOR_OFFSET_ADPD_ERROR);
        }
        fifo_count -= expected_readout_bytes;

        // 0: sun-vis; 1: leaf-ir; 2/3: fluo signal dark/lit;
        // 4/5: fluo reference dark/lit; 6/7: 730 signal/reference.
        fluor += calc_signal(samples[2], samples[3], num_integration);
        fluoRef += calc_signal(samples[4], samples[5], num_integration);
        r730 += samples[6];
        r730Ref += samples[7];
        sun += samples[0];
        leaf += samples[1];
        ++counter;
      }
    }

    if (adpd.STOP() != jii::adpd6000::kOk) return FLUOR_OFFSET_ADPD_ERROR;
    ret_fluor += fluor / counter;
    ret_fluoRef += fluoRef / counter;
    ret_sun += sun / counter;
    ret_leaf += leaf / counter;
    ret_r730 += r730 / counter;
    ret_r730Ref += r730Ref / counter;
  }

  // Leave the caller-provided vector untouched unless the full acquisition
  // succeeded. Callers can therefore safely gate NVS writes on this result.
  fret[0] = ret_fluor / repeat_count;
  fret[1] = ret_fluoRef / repeat_count;
  fret[2] = ret_sun / repeat_count;
  fret[3] = ret_leaf / repeat_count;
  fret[4] = ret_r730 / repeat_count;
  fret[5] = ret_r730Ref / repeat_count;
  return FLUOR_OFFSET_OK;

}
