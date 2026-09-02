#include "PAM.h"
#include "nvs1.h"

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



int run_preprocess_type1(uint8_t length, uint8_t* arr, uint16_t* data_counter){
    uint8_t pc = 0;
    uint8_t _type = 0;
    uint8_t para1, actinic, subsampling = 0;
    uint16_t num_ptx = 0, freq = 0;
    uint16_t _data_counter[4] = {0};

    while (pc < length){
      _type = *(arr + pc * 8);
      if ((_type == 1) || (_type == 2)){
        arr_line_parse_type1(arr + pc * 8, &para1, &num_ptx, &freq, &actinic, &subsampling, _data_counter);
        ESP_LOGV(TAG, "Run type:%d, number:%d, freq: %d, actinic: %d, subfactor %d", _type, num_ptx, freq, actinic, subsampling);
        for (uint8_t i = 0; i < 4; i++) data_counter[i] += _data_counter[i];
      }
      pc += 1;
    }
    return 0;
}

// Shared output sink for a completed run: stream the 7 channel buffers over the
// active CONNECTION_TYPE. COMPUTER -> ASCII send_serial; AMBYTE -> binary FSM
// (env retried once; sun/leaf/730 only when present). Extracted verbatim from the
// per-run send block so every run path (sweep #4) shares one copy. Wire-touching:
// re-run HW_CONFORMANCE.md after changing it.
static void pam_send_results(dataclass* d_env, dataclass* d_fluor, dataclass* d_fluoRef,
                             dataclass* d_sun, dataclass* d_leaf, dataclass* d_730,
                             dataclass* d_730Ref, dataclass* d_timing, uint8_t subsampling,
                             bool has_730, bool allow_interrupt){
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
  }
}

int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist){
  return run_arr_type1(length, arr, led_persist, false);
}

// ── Async result holder (parallel trigger/poll/fetch protocol) ──────────────
// Holds the eight result buffers of one retained run plus the metadata
// pam_send_results() needs, until the host FETCHes them. One run at a time.
struct ambit_async_result_t {
  dataclass *d_env = NULL, *d_fluor = NULL, *d_fluoRef = NULL, *d_sun = NULL,
            *d_leaf = NULL, *d_730 = NULL, *d_730Ref = NULL, *d_timing = NULL;
  uint8_t subsampling = 0;
  bool    has_730 = false;
  bool    allow_interrupt = false;
  uint8_t state = AMBIT_ASYNC_IDLE;
};
static ambit_async_result_t g_async;

void ambit_async_clear(void){
  delete g_async.d_env;    delete g_async.d_fluor;  delete g_async.d_fluoRef;
  delete g_async.d_sun;    delete g_async.d_leaf;   delete g_async.d_730;
  delete g_async.d_730Ref; delete g_async.d_timing;
  g_async = ambit_async_result_t();   // re-init all pointers to NULL, state IDLE
}

uint8_t ambit_async_get_state(void){ return g_async.state; }


// openJII env channel: fw_new stores leaf temp as centi-degC in the low 16 bits.
// Used only on the json_output path (arrun via the JSON envelope).
static void send_env_json(dataclass* d){
  Serial.print("\"env\":[");
  if (d->available){
    uint16_t n = d->get_length();
    for (uint16_t i = 0; i < n; i++){
      uint32_t raw = d->pop();
      float temp = (int16_t)(raw & 0xFFFF) / 100.0f;
      if (i > 0) Serial.print(',');
      Serial.printf("{\"temp_c\":%.2f}", temp);
    }
  }
  Serial.print(']');
}

// openJII derived channel: fluo = s_630 / r_630 (signal / reference), one float
// per sample. Computed on-device only for the JSON path because the openJII sink
// consumes the JSON verbatim and does no math. MUST run BEFORE d_fluor / d_fluoRef are
// drained by send_json(): it reads arr[] directly and relies on read_ptr == 0 (nothing
// popped yet). den == 0 -> 0 (calc_signal floors the reference at 0 when dark dominates).
static void send_fluo_json(dataclass* num, dataclass* den){
  Serial.print("\"fluo\":[");
  if (num->available && den->available){
    uint16_t n = num->get_length();
    uint16_t m = den->get_length();
    if (m < n) n = m;                     // defensive: emit only paired samples
    for (uint16_t i = 0; i < n; i++){
      if (i > 0) Serial.print(',');
      uint32_t d = den->arr[i];
      if (d == 0) Serial.print('0');
      else Serial.printf("%.5f", (double)num->arr[i] / (double)d);
    }
  }
  Serial.print(']');
}

// ── Shared per-sample store: free-run run_arr_type1 + triggered run_arr_trigger ──
// One readout `ret` (0 sun-vis, 1 leaf-ir, 2/3 fluo signal dark/lit, 4/5 fluo
// reference dark/lit, 6/7 730 signal/reference) -> calibration at the result
// boundary -> channel buffers (+ the every-8 subsampling accumulator `buf_opt`) ->
// optional PLOTTING line. `counter` is the 0-based index of this sample within its
// line. Extracted VERBATIM from run_arr_type1 so the two acquisition engines cannot
// drift apart (plans/DETERMINISTIC_ADPD.md §5 invariant 1). Wire-touching: what is
// stored here is what pam_send_results streams, so HW_CONFORMANCE.md covers it.
static void pam_store_type1_sample(const uint32_t* ret, uint8_t num_integration, uint8_t _type,
                                   uint8_t subsampling, uint32_t counter, float_t leaf_temp,
                                   uint32_t* buf_opt,
                                   dataclass* d_fluor, dataclass* d_fluoRef, dataclass* d_sun,
                                   dataclass* d_leaf, dataclass* d_730, dataclass* d_730Ref){
  uint32_t ploter1 = 0, ploter2 = 0;
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
}

// Shared end-of-run sink for the array engines. Three mutually exclusive outputs,
// unchanged from the inline block they replace: json_output -> one JSON object on
// Serial (no TIMING block: the envelope has no slot for it); retain -> hand the
// eight buffers to the async holder for a later FETCH (cmd 24); otherwise stream
// them now via pam_send_results on the active CONNECTION_TYPE. Returns true when
// buffer ownership moved to g_async — the caller must then NOT delete them.
static bool pam_finish_results(dataclass* d_env, dataclass* d_fluor, dataclass* d_fluoRef,
                               dataclass* d_sun, dataclass* d_leaf, dataclass* d_730,
                               dataclass* d_730Ref, dataclass* d_timing,
                               uint8_t subsampling, bool has_730, bool allow_interrupt,
                               bool json_output, bool retain, int64_t run_tick_begin){
  if (json_output) {            // one JSON object: {"env":[..],"fluo":[..],"s_630":[..],...}
    Serial.print('{');
    send_env_json(d_env);
    // fluo first: it reads d_fluor/d_fluoRef before send_json() drains them.
    Serial.print(','); send_fluo_json(d_fluor, d_fluoRef);
    Serial.print(','); d_fluor->send_json("s_630");
    Serial.print(','); d_fluoRef->send_json("r_630");
    if (d_sun->available)    { Serial.print(','); d_sun->send_json("sun"); }
    if (d_leaf->available)   { Serial.print(','); d_leaf->send_json("leaf"); }
    if (d_730->available)    { Serial.print(','); d_730->send_json("s_730"); }
    if (d_730Ref->available) { Serial.print(','); d_730Ref->send_json("r_730"); }
    Serial.print('}');
    return false;
  }
  d_timing->put((uint32_t) run_tick_begin);          // Change 3: run start tick (us)
  d_timing->put((uint32_t) esp_timer_get_time());    //           run end tick (us)
  if (retain){
    // Parallel protocol: transfer ownership of the buffers to the async holder
    // instead of streaming + freeing. The host fetches them later (cmd 24).
    g_async.d_env = d_env; g_async.d_fluor = d_fluor; g_async.d_fluoRef = d_fluoRef;
    g_async.d_sun = d_sun; g_async.d_leaf = d_leaf; g_async.d_730 = d_730;
    g_async.d_730Ref = d_730Ref; g_async.d_timing = d_timing;
    g_async.subsampling = subsampling; g_async.has_730 = has_730;
    g_async.allow_interrupt = allow_interrupt;
    return true;
  }
  pam_send_results(d_env, d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref,
                   d_timing, subsampling, has_730, allow_interrupt);
  return false;
}

int run_arr_type1(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt, bool json_output, bool retain){
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

  // Run protocol preprecess, get storage size
  uint16_t data_count[] = {0, 0, 0, 0};
  if (run_preprocess_type1(length, arr, data_count) == -1) return -1;    // calculate data counts
  ESP_LOGV(TAG, "Sample & Ref: %d, optionals: %d", data_count[0], data_count[1]);

  dataclass *d_env = new dataclass; //
  dataclass *d_fluor = new dataclass; // fluorescence signal
  dataclass *d_fluoRef = new dataclass; // fluorescence reference

  dataclass *d_sun = new dataclass; // sun-side ambient
  dataclass *d_leaf = new dataclass;  // leaf-side ambient
  dataclass *d_730 = new dataclass; // 730nm reflectance signal
  dataclass *d_730Ref = new dataclass;  // 730nm reference
  dataclass *d_timing = new dataclass;  // Change 3: [tick_begin, tick_end] controller ticks (us)


  if (!(d_env->init(512))) return -1;
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
  uint32_t counter = 0;
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
      // JSON path: exactly ONE env temperature per array, sampled once at the start
      // (the unconditional d_env->put() before this loop). No in-run 2 s resampling, so
      // env is a single {"temp_c":..} for every array — fast or slow. All other paths
      // keep the original cadence: that env stream is part of the FROZEN wire contract
      // with the datalogger, so its bytes must not change. Runtime branch on
      // json_output (was compile-time VARIANT_CLOUD) — same semantics per caller.
      if (json_output) measure_temperature = false;
      else measure_temperature = (light_sleep_time > 20) && measure_temp && actinic < 50;


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
          pam_store_type1_sample(ret, num_integration, _type, subsampling, counter, leaf_temp, buf_opt,
                                 d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref);
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


  if (pam_finish_results(d_env, d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref, d_timing,
                         subsampling, data_count[2] > 0, allow_interrupt,
                         json_output, retain, run_tick_begin)){
    return 0;   // holder owns the buffers now — do NOT delete
  }

  delete d_fluor;
  delete d_fluoRef;
  delete d_sun;
  delete d_leaf;
  delete d_730;
  delete d_730Ref;
  delete d_env;
  delete d_timing;

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
                   g_async.subsampling, g_async.has_730, g_async.allow_interrupt);
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
                   d_timing, 1, true, interrrupt);

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


// ═════════════════════════════════════════════════════════════════════════════
// Exact-N triggered acquisition — plans/DETERMINISTIC_ADPD.md (Phase 1)
// ═════════════════════════════════════════════════════════════════════════════
// run_arr_type1 puts the ADPD6100 in free-run (GO=1, internal TIMESLOT_PERIOD =
// run_freq(freq)) and only STOP()s once num_ptx samples have been READ from the
// FIFO. num_ptx therefore bounds the samples stored, not the timeslot sequences
// executed: between the last useful FIFO read and STOP() the sequencer keeps
// firing LED pulses, and at high freq the light-sleep cadence lets many
// sequences run per wake. The leaf receives an unknown, rate-dependent number
// of extra 630 nm pulses and the emitted count is not reproducible.
//
// Here the same per-line protocol is acquired in EXT_SYNC: one rising edge on
// ESP32 GPIO10 -> ADPD GPIO0 launches exactly one full timeslot sequence and the
// internal period timer is gated off. A software-paced loop fires exactly
// num_ptx edges at period 1/freq, reads one sequence after each, and aborts —
// never stores a partial sample — if an edge is lost. Sequences == edges ==
// stored == N. Storage, calibration, subsampling and sinks are the helpers
// shared with run_arr_type1, so a given line yields the same bytes.

// Per-sample bound from the edge to a full readout. Well above every measured or
// computed sequence time (plan §4.7: ~0.7-2 ms at num_ts=3, ~3 ms far-red); a
// miss means the edge was lost (rate above the ceiling, or a chip fault), and
// the run aborts rather than desyncing (plan §4.5).
static constexpr uint32_t kTrigSampleTimeoutUs = 20000;
// Light-sleep only for gaps longer than this; below it the wake latency would
// overshoot the edge, so the loop busy-waits.
static constexpr int64_t kTrigSleepMinGapUs = 3000;
// Wake at least this early and busy-wait the remainder to the edge. The light-sleep
// timer runs on the ESP's 136 kHz RC slow clock (ESP8685 DS §4.1.3.3): calibrated at
// boot but percent-level over temperature, so the margin also scales with the gap
// (kTrigSleepMarginPct) — a fixed 1.5 ms would be overrun by a 1 s sleep at 1 % error.
static constexpr int64_t kTrigSleepMarginUs = 1500;
static constexpr int64_t kTrigSleepMarginPct = 2;
// EXT_SYNC pulse width. The ADPD's low-frequency state machine runs at 960 kHz
// (ADPD6000 DS "Low Frequency Oscillator"), a 1.04 µs tick; the legacy 1 µs
// adpd_trigger() pulse is one tick wide and can straddle a boundary. Five ticks costs
// nothing at any rate this path can reach (plan §5 invariant 8).
static constexpr uint32_t kTrigPulseUs = 5;
// Settle after RUN() in EXT_SYNC before the first edge (run_trigger_spacer's
// value; Phase 3 measures whether it can go).
static constexpr uint32_t kTrigArmSettleMs = 5;

// Far-red lines run num_ts(9): the FIFO is complete after ts0..ts2, but the six
// illumination-only slots (ts3..ts8, `repeats` pulses each, 0 FIFO bytes) are
// STILL firing. An edge sent into that tail is swallowed (LEGACY bench: exactly
// 50 % timeouts), so the period is floored to the whole sequence and far-red runs
// at min(freq, ceiling) rather than aborting. PROVISIONAL datasheet arithmetic —
// gate V0 (`tseq,500,1,4`) replaces it with the measured full-sequence time.
static inline int64_t trig_farred_sequence_us(uint8_t repeats){
  return 1000 + 2200LL * repeats;
}

// ── EXT_SYNC arm / disarm, shared by the triggered run and the diagnostics ──────
// GPIO0_cfg=1 (input), SYNC_GPIO=0 (GPIO0 is the sync source), EXT_SYNC_EN=1: the
// sequencer runs one full timeslot sequence per rising edge and the internal
// period timer is gated off. GPIO10 (the edge source) is driven LOW first and
// held LOW as an output through light sleep — a floating pin across the
// sleep/wake transition can produce an edge the ADPD would count (plan §4.4).
static void trig_arm_ext_sync(void){
  digitalWrite(10, LOW);
  adpd.gpio_config.GPIO0_cfg = 1;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_config.EXT_SYNC_EN = 1;
  adpd.gpio_setup(&(adpd.gpio_config));
  gpio_sleep_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT);
  gpio_sleep_set_pull_mode(GPIO_NUM_10, GPIO_PULLDOWN_ONLY);
}

// Restore free-run: STOP, drain, EXT_SYNC off, GPIO0 tristate. A leftover
// EXT_SYNC_EN=1 silently breaks the next free-run arrun (it would wait for edges
// that never come), so every exit of every EXT_SYNC user passes through here
// (plan §5 invariant 3).
static void trig_disarm_ext_sync(void){
  adpd.STOP();
  adpd.clear_fifo();
  adpd.gpio_config.GPIO0_cfg = 0;
  adpd.gpio_config.EXT_SYNC_EN = 0;
  adpd.gpio_config.SYNC_GPIO = 0;
  adpd.gpio_setup(&(adpd.gpio_config));
  digitalWrite(10, LOW);
}

// Fire one edge and wait, µs-bounded, for `expected_bytes` in the FIFO. Returns
// true with *fifo_c set when the sequence landed; false on a lost edge.
static bool trig_fire_and_wait(uint8_t expected_bytes, int64_t* t_trig, uint16_t* fifo_c){
  *t_trig = esp_timer_get_time();
  digitalWrite(10, HIGH);
  delayMicroseconds(kTrigPulseUs);
  digitalWrite(10, LOW);
  *fifo_c = 0;
  while ((uint32_t)(esp_timer_get_time() - *t_trig) < kTrigSampleTimeoutUs){
    *fifo_c = adpd.fifo_count();
    if (*fifo_c >= expected_bytes) return true;
  }
  return false;
}

// Non-blocking interrupt poll. PAM_interrupt() spins up to 15 ms when the input
// is idle — harmless in free-run, where the FIFO buffers the stall, fatal to edge
// pacing here. Only consult it once a byte is actually pending.
static bool trig_poll_interrupt(bool allow){
  if (!allow || Serial.available() <= 0) return false;
  return PAM_interrupt(allow, false);
}

// Pacing statistics of the last triggered run, for gate V1j (mean interval, drift,
// sleep-wake overshoot). Recorded on every run — a few integer ops per sample — and
// read back by the AMBIT_DIAG_TRIGGER-only `tstat` console verb.
struct trig_run_stats_t {
  uint32_t samples = 0;        // edges fired
  uint32_t late_count = 0;     // edges fired after their planned instant
  uint32_t max_late_us = 0;    // worst overshoot of the planned instant
  uint32_t residual_count = 0; // FIFO held more than one sequence after an edge (=> abort)
  int      result = 0;         // ArrTriggerResult of the run
};
static trig_run_stats_t g_trig_stats;

// Boundary check for the triggered engine (plan §6 Phase 1). The free-run path
// lets dataclass::init fail on an oversized N and returns -1 leaking its eight
// buffers; here nothing is allocated and the chip is untouched until the whole
// array is known good. freq == 0 would be a zero-rate line (the free-run path
// divides by it).
int run_arr_trigger_validate(uint8_t length, uint8_t* arr){
  uint16_t data_count[4] = {0};
  run_preprocess_type1(length, arr, data_count);
  if (data_count[0] == 0 || data_count[0] >= MAX_DATACLASS_SIZE) return ARR_TRIG_BAD_LINE;
  for (uint8_t pc = 0; pc < length; pc++){
    const uint8_t* line = arr + pc * 8;
    if (line[0] != 1 && line[0] != 2) continue;
    const uint16_t num_ptx = line[3] + (line[2] << 8);
    const uint16_t freq    = line[5] + (line[4] << 8);
    if (num_ptx > 0 && freq == 0) return ARR_TRIG_BAD_LINE;
  }
  return ARR_TRIG_OK;
}

int run_arr_trigger(uint8_t length, uint8_t* arr, bool led_persist, bool allow_interrupt,
                    bool json_output, bool retain){
  const int validation = run_arr_trigger_validate(length, arr);
  g_trig_stats = trig_run_stats_t();
  g_trig_stats.result = validation;
  if (validation != ARR_TRIG_OK) return validation;

  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1) conf_slow_FR_1();

  // Fluor slot on-chip integration is 1 (preset_config_2(1, 1) in conf_slow_FR_1);
  // the calc_signal divisor must stay equal to it (plan §5 invariant 5).
  const uint8_t num_integration = 1;
  const unsigned int start_t0 = millis();
  const int64_t run_tick_begin = esp_timer_get_time();

  uint16_t data_count[] = {0, 0, 0, 0};
  run_preprocess_type1(length, arr, data_count);

  // Every local is declared before the first goto (single-exit cleanup).
  uint8_t pc = 0, _type = 0, farred = 0, actinic = 0, subsampling = 0;
  uint16_t num_ptx = 0, freq = 0;
  uint8_t expected_readout = 8, expected_readout_bytes = 24;
  uint32_t ret[8] = {0};
  uint32_t counter = 0;
  uint16_t fifo_c = 0;
  uint32_t buf_opt[4] = {0};
  uint32_t _tmparr = 0;
  uint8_t _repeats = 1;
  uint32_t period_ms_int = 1;             // run_arr_type1's `light_sleep_time`: gates the env resample
  int64_t period_us = 1000000, next_trigger = 0, t_trig = 0, now_us = 0, farred_floor = 0;
  int64_t gap_us = 0, margin_us = 0;
  float_t leaf_temp = 0.0;
  unsigned int env_timer1 = millis();
  bool measure_temperature = false;
  bool interrupt_run = false;
  bool ext_sync_on = false;
  bool buffers_transferred = false;
  int _func_ret = ARR_TRIG_ABORT;

  dataclass *d_env = new dataclass;
  dataclass *d_fluor = new dataclass;      // fluorescence signal
  dataclass *d_fluoRef = new dataclass;    // fluorescence reference
  dataclass *d_sun = new dataclass;        // sun-side ambient
  dataclass *d_leaf = new dataclass;       // leaf-side ambient
  dataclass *d_730 = new dataclass;        // 730nm reflectance signal
  dataclass *d_730Ref = new dataclass;     // 730nm reference
  dataclass *d_timing = new dataclass;     // [tick_begin, tick_end] controller ticks (us)

  _func_ret = ARR_TRIG_NOMEM;
  if (!(d_env->init(512))) goto cleanup;
  if (!(d_timing->init(2))) goto cleanup;
  if (!((d_fluor->init(data_count[0])) && (d_fluoRef->init(data_count[0])))) goto cleanup;
  if (data_count[1] > 0){
    if (!((d_sun->init(data_count[1])) && (d_leaf->init(data_count[1])))) goto cleanup;
  }
  if (data_count[2] > 0){
    if (!((d_730->init(data_count[2])) && (d_730Ref->init(data_count[2])))) goto cleanup;
  }
  _func_ret = ARR_TRIG_ABORT;

  _tmparr = PAM_get_env(4, start_t0);
  d_env->put(_tmparr);
  leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;
  env_timer1 = millis();

  adpd.STOP();
  trig_arm_ext_sync();
  ext_sync_on = true;
  adpd_mode = ADPD_CONFIG_MODE::ARRAY_SLOW;

  while (pc < length){
    if (interrupt_run) break;
    _type = *(arr + pc * 8);
    if ((_type == 1) || (_type == 2)){
      arr_line_parse_type1((arr + pc * 8), &farred, &num_ptx, &freq, &actinic, &subsampling, NULL);
      // Park the internal period at 100 ms. EXT_SYNC gates the timer (tidle shows
      // zero self-triggers), but if that ever failed the leak would be slow and
      // visible rather than a kHz pulse train (plan §4.3).
      adpd.run_freq(10);
      adpd.clear_fifo();
      period_us = 1000000LL / freq;        // freq > 0: validated above
      period_ms_int = (1000/freq);
      // Env cadence identical to run_arr_type1: the env stream is part of the frozen
      // wire. JSON gets exactly one env per array; the others resample every 2 s on
      // slow (< ~50 Hz), low-actinic lines.
      if (json_output) measure_temperature = false;
      else measure_temperature = (period_ms_int > 20) && measure_temp && actinic < 50;

      if (_type == 1){
        if (farred == 1){
          adpd.num_ts(9);
          _repeats = int(400/freq);
          if (freq < 3) _repeats = 250;
          if (_repeats == 0) _repeats = 1;
          for (uint8_t i = 3; i < 9;i++) adpd.repeats_only(i, 1, _repeats);
        }else{
          adpd.num_ts(3);
        }
        expected_readout = 8;
      }else{ // NO IR
        adpd.num_ts(2);
        expected_readout = 6;
      }
      expected_readout_bytes = expected_readout * 3;
      farred_floor = (farred == 1) ? trig_farred_sequence_us(_repeats) : 0;

      if (actinic > 3){
        AS_LED_Current(actinic);
        AS_LED_ON();
      }else{
        AS_LED_OFF();
        AS_LED_Current(0);
      }

      counter = 0;
      for (uint8_t i = 0; i < 4; i++) buf_opt[i] = 0;
      adpd.RUN();                          // GO=1 in EXT_SYNC: armed, waits for the first edge
      delay(kTrigArmSettleMs);

      next_trigger = esp_timer_get_time();
      while (counter < num_ptx){
        // ── pace to the edge: light-sleep the long part of the gap, busy-wait the rest ──
        now_us = esp_timer_get_time();
        while ((gap_us = next_trigger - now_us) > kTrigSleepMinGapUs){
          margin_us = gap_us * kTrigSleepMarginPct / 100;
          if (margin_us < kTrigSleepMarginUs) margin_us = kTrigSleepMarginUs;
          esp_sleep_enable_timer_wakeup((uint64_t)(gap_us - margin_us));
          interrupt_run = trig_poll_interrupt(allow_interrupt);
          if (!interrupt_run) esp_light_sleep_start();
          interrupt_run = PAM_interrupt(allow_interrupt, true);   // cheap: only acts on a UART wake
          if (interrupt_run) break;
          now_us = esp_timer_get_time();
        }
        if (interrupt_run) break;
        while (esp_timer_get_time() < next_trigger) { /* fine busy-wait to the edge */ }

        // ── one edge, one sequence, one µs-bounded readout ──
        if (!trig_fire_and_wait(expected_readout_bytes, &t_trig, &fifo_c)){
          _func_ret = ARR_TRIG_LOST_TRIGGER;     // never store a partial / desynced sample
          goto cleanup;
        }
        g_trig_stats.samples++;
        if (t_trig > next_trigger){
          g_trig_stats.late_count++;
          if ((uint32_t)(t_trig - next_trigger) > g_trig_stats.max_late_us)
            g_trig_stats.max_late_us = (uint32_t)(t_trig - next_trigger);
        }
        // Pace the next edge from THIS edge, not from the read, so the mean interval
        // is exactly 1/freq; the far-red floor keeps the edge out of the LED tail.
        next_trigger = t_trig + period_us;
        if (next_trigger < t_trig + farred_floor) next_trigger = t_trig + farred_floor;

        // One edge is one sequence is exactly expected_readout_bytes: the poll only
        // exits at >= expected, so anything above it is a sequence we did not ask for
        // (tidle rules out self-triggering) or a FIFO we did not own. That is a
        // desync, and the datasheet specifies CLEAR_FIFO "while not operating", so
        // the run aborts instead of clearing in GO mode (plan §4.1, §9).
        if (fifo_c != expected_readout_bytes){
          g_trig_stats.residual_count++;
          _func_ret = ARR_TRIG_FIFO_DESYNC;
          goto cleanup;
        }
        adpd.readfifo(expected_readout, 3, ret);

        pam_store_type1_sample(ret, num_integration, _type, subsampling, counter, leaf_temp, buf_opt,
                               d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref);
        counter++;

        // In-run env resample: same cadence and guards as run_arr_type1. The MLX read
        // takes a few ms; it only runs on slow lines (period > 20 ms) so it fits the gap.
        if (counter + 10 < num_ptx && measure_temperature && (millis() - env_timer1 > 2000)){
          _tmparr = PAM_get_env(4, start_t0);
          d_env->put(_tmparr);
          leaf_temp = (int16_t) (_tmparr & 0xFFFF) / 100.0;
          env_timer1 = millis();
        }
        interrupt_run = trig_poll_interrupt(allow_interrupt);
      }

      adpd.STOP();
      if (!led_persist) AS_LED_OFF();
      digitalWrite(STF_FLASH_PIN, LOW);
    }
    pc += 1;
  }

  // Interrupted (host sent AMBYTE_INTR): as in run_arr_type1, what was acquired is
  // still delivered — the binary host waits for a reply — and the return is OK.
  if (interrupt_run){
    digitalWrite(STF_FLASH_PIN, LOW);
    adpd.STOP();
    AS_LED_OFF();
  }

  buffers_transferred = pam_finish_results(d_env, d_fluor, d_fluoRef, d_sun, d_leaf, d_730, d_730Ref,
                                           d_timing, subsampling, data_count[2] > 0, allow_interrupt,
                                           json_output, retain, run_tick_begin);
  _func_ret = ARR_TRIG_OK;

cleanup:
  // Single exit for every path: bad alloc, lost edge, desync, interrupt, completion.
  g_trig_stats.result = _func_ret;
  adpd.STOP();
  if (ext_sync_on){
    trig_disarm_ext_sync();
    adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
  }
  if (_func_ret != ARR_TRIG_OK){
    if (!led_persist) AS_LED_OFF();
    digitalWrite(STF_FLASH_PIN, LOW);
  }
  if (!buffers_transferred){
    delete d_fluor;
    delete d_fluoRef;
    delete d_sun;
    delete d_leaf;
    delete d_730;
    delete d_730Ref;
    delete d_env;
    delete d_timing;
  }
  return _func_ret;
}


#ifdef AMBIT_DIAG_TRIGGER
// ═════════════════════════════════════════════════════════════════════════════
// Bench diagnostics — plans/DETERMINISTIC_ADPD.md Phase 0 (gates V0, V1f)
// ═════════════════════════════════════════════════════════════════════════════
// Console-only replies to explicit console verbs (allowed by AGENTS.md), compiled
// out without -DAMBIT_DIAG_TRIGGER. Each one overrides the ambient/730 slot
// integration, so it leaves adpd_mode dirty (MPF_MODE): the next array run
// re-applies conf_slow_FR_1 instead of inheriting the diagnostic's integration.

// Standard optics, then only the COUNTS register of ts0 (ambient) and ts2 (730)
// overridden to `integ`; the fluor slot stays integ=1. Far-red adds ts3..ts8 with
// `frrep` pulses each. Returns the active slot count.
static uint8_t diag_config(bool farred, uint8_t integ, uint8_t frrep){
  adpd.STOP();
  conf_slow_FR_1();
  adpd.repeats_only(0, integ, 1);
  adpd.repeats_only(2, integ, 1);
  uint8_t num_active = 3;
  if (farred){
    num_active = 9;
    for (uint8_t i = 3; i < 9; i++) adpd.repeats_only(i, 1, frrep);
  }
  return num_active;
}

static void diag_arm(uint8_t num_active){
  trig_arm_ext_sync();
  adpd.clear_fifo();
  adpd.run_freq(10);          // 100 ms park: if gating failed, ~10 self-triggers/s, visible
  adpd.num_ts(num_active);
  adpd.RUN();
  delay(kTrigArmSettleMs);
}

static void diag_teardown(void){
  trig_disarm_ext_sync();
  adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;   // dirty: ts0/ts2 integration was overridden
}

// Empty the FIFO by READING it. CLEAR_FIFO is specified "while not operating"
// (ADPD6000 DS, FIFO_STATUS), so a diagnostic that stays in GO between edges drains
// by reads instead.
static void diag_drain_bytes(uint16_t n){
  uint32_t scratch[16];
  while (n > 0){
    const uint16_t chunk = n > 16 ? 16 : n;
    adpd.readfifo(chunk, 1, scratch);
    n -= chunk;
  }
}

// tstat — pacing statistics of the last run_arr_trigger() (gate V1j). Late edges
// come from light-sleep overshoot (RC slow clock) or a requested rate above the
// ceiling; a residual means a FIFO desync (the run aborted with -5).
void print_trig_stats(void){
  Serial.printf("tstat: result=%d samples=%u late=%u max_late_us=%u residual=%u\n",
                g_trig_stats.result, g_trig_stats.samples, g_trig_stats.late_count,
                g_trig_stats.max_late_us, g_trig_stats.residual_count);
  Serial.flush();
}

// tseq,<reps>,<farred>,<integ>[,<frrep>] — per-sequence execution time. One edge
// -> time until the FIFO holds a full type-1 readout, min/mean/max over `reps`.
// 1/t_seq is the hard ceiling on the triggered rate; the far-red number is the
// DATA-slot (head) time only, the illumination tail is not visible in the FIFO
// and must be scoped on the LED line. The measured time includes one
// fifo_count() poll (tens of µs), a slight over-estimate within the V0 ±10 %.
int measure_tseq(uint16_t reps, bool farred, uint8_t integ, uint8_t frrep){
  if (reps == 0)  reps = 200;
  if (integ == 0) integ = 4;     // production value on the ambient/730 slots
  if (frrep == 0) frrep = 1;
  const uint8_t expected_readout = 8, expected_readout_bytes = 24;
  const uint8_t num_active = diag_config(farred, integ, frrep);
  diag_arm(num_active);

  uint32_t ret[8] = {0};
  uint32_t dt = 0, dt_min = 0xFFFFFFFFUL, dt_max = 0;
  uint64_t dt_sum = 0;
  uint16_t ok = 0, timeouts = 0, residuals = 0;
  int64_t t0 = 0;
  uint16_t fifo_c = 0;

  for (uint16_t n = 0; n < reps; n++){
    const bool got = trig_fire_and_wait(expected_readout_bytes, &t0, &fifo_c);
    dt = (uint32_t)(esp_timer_get_time() - t0);
    if (got){
      adpd.readfifo(expected_readout, 3, ret);
      if (fifo_c > expected_readout_bytes){          // more than one sequence: count + drain
        residuals++;
        diag_drain_bytes(fifo_c - expected_readout_bytes);
      }
      if (dt < dt_min) dt_min = dt;
      if (dt > dt_max) dt_max = dt;
      dt_sum += dt;
      ok++;
      // Far-red: let the illumination tail finish before the next edge (plan §4.2).
      if (farred){
        while ((esp_timer_get_time() - t0) < trig_farred_sequence_us(frrep)) { }
      }
    }else{
      timeouts++;
      if (fifo_c > 0) diag_drain_bytes(fifo_c);      // partial sequence: drain by reads
    }
  }
  diag_teardown();

  const uint32_t mean = ok ? (uint32_t)(dt_sum / ok) : 0;
  Serial.printf("tseq: num_ts=%u integ(amb/730)=%u frrep=%u reps=%u ok=%u timeouts=%u residuals=%u\n",
                num_active, integ, farred ? frrep : 0, reps, ok, timeouts, residuals);
  Serial.printf("tseq_us: min=%u mean=%u max=%u max_rate_hz~%u\n",
                (ok ? dt_min : 0), mean, dt_max, (mean ? (uint32_t)(1000000UL / mean) : 0));
  if (farred){
    Serial.printf("tseq_note: far-red dt is DATA-slot (head) time only; the LED tail runs "
                  "~%u us beyond FIFO-ready (provisional floor) - scope the LED line.\n",
                  (unsigned)(trig_farred_sequence_us(frrep) - mean));
  }
  Serial.flush();
  return 0;
}

// tidle,<farred>,<integ>,<gate> — is the ADPD firing sequences we did NOT
// trigger? Arms EXT_SYNC exactly like run_arr_trigger, then sits 2 s sending
// ZERO edges and watches the FIFO. GO_autorun>0: GO=1 ran a sequence on its own.
// idle_max>0: the internal timer self-triggers (real spurious LED pulses). Both 0:
// the engine is edge-gated and any run-time "extra bytes" is a byte-count read
// artifact. gate: 0 = none. gate 1 (GO_SLEEP) needed the vendored ADI driver's
// sleep-mode call; the clean-room driver does not expose it, so it is refused.
int measure_idle(bool farred, uint8_t integ, uint8_t gate){
  if (gate != 0){
    Serial.println("tidle: gate=1 (GO_SLEEP) is not exposed by the clean-room driver; use gate=0");
    return -1;
  }
  if (integ == 0) integ = 1;
  const uint8_t num_active = diag_config(farred, integ, 1);
  diag_arm(num_active);

  const uint16_t initial = adpd.fifo_count();   // GO auto-run sequence(s), if any (no edge sent)
  adpd.clear_fifo();

  const int64_t t0 = esp_timer_get_time();
  uint16_t maxc = 0, c = 0;
  uint32_t polls = 0;
  while ((uint32_t)(esp_timer_get_time() - t0) < 2000000){   // 2 s, NO triggers
    c = adpd.fifo_count();
    if (c > maxc) maxc = c;
    polls++;
  }
  diag_teardown();

  Serial.printf("tidle: gate=none num_ts=%u integ=%u GO_autorun=%u bytes idle_max=%u bytes / 2000ms (%u polls)\n",
                num_active, integ, initial, maxc, polls);
  Serial.printf("tidle_verdict: GO_autorun=%s self_trigger=%s\n",
                initial == 0 ? "NO" : "YES", maxc == 0 ? "NO" : "YES");
  Serial.flush();
  return 0;
}

// tratio,<reps>,<N>,<freq>,<integ> — first-sample ratio check (plan §4.6). Repeats
// a triggered type-1 acquisition `reps` times (fresh RUN() each time so sample 0
// is a genuine first sample) and compares the first sample's fluor s/r to the
// steady-state ratio (samples [skip..N-1]) with a Welford SEM noise floor.
// Common-mode (ratio preserved) -> relative yield unaffected, no warm-up needed;
// channel-specific (ratio shifts) -> AFE settling, warm-up / drop-first warranted.
// Fires reps*N lit pulses: run on a test target, not a precious sample.
int measure_first_ratio(uint16_t reps, uint16_t N, uint16_t freq, uint8_t integ){
  if (reps == 0) reps = 20;
  if (N == 0)    N = 20;
  if (N > 512)   N = 512;
  if (freq == 0) freq = 500;
  if (integ == 0) integ = 1;
  const uint8_t skip = 3;
  if (N < (uint16_t)(skip + 3)) N = skip + 3;

  const uint8_t expected_readout = 8, expected_readout_bytes = 24;
  const int64_t period_us = 1000000LL / freq;
  const uint8_t num_active = diag_config(false, integ, 1);
  trig_arm_ext_sync();

  uint32_t ret[8] = {0};
  double m0 = 0, M2_0 = 0, mS = 0, M2_S = 0;   // Welford: per-run first-sample & steady ratios
  uint32_t n = 0, bad = 0;

  for (uint16_t rep = 0; rep < reps; rep++){
    adpd.clear_fifo();
    adpd.run_freq(10);
    adpd.num_ts(num_active);
    adpd.RUN();
    delay(kTrigArmSettleMs);

    uint32_t s0 = 0, r0 = 0;
    double sumS_s = 0, sumS_r = 0;
    bool run_ok = true;
    int64_t next_trigger = esp_timer_get_time(), t_trig = 0;
    uint16_t fifo_c = 0;
    for (uint16_t k = 0; k < N; k++){
      while (esp_timer_get_time() < next_trigger) { /* pace */ }
      if (!trig_fire_and_wait(expected_readout_bytes, &t_trig, &fifo_c)){ run_ok = false; break; }
      next_trigger = t_trig + period_us;
      adpd.readfifo(expected_readout, 3, ret);
      if (fifo_c > expected_readout_bytes){ run_ok = false; break; }   // desync: discard run
      const uint32_t sig = calc_signal(ret[2], ret[3], 1);
      const uint32_t ref = calc_signal(ret[4], ret[5], 1);
      if (k == 0){ s0 = sig; r0 = ref; }
      else if (k >= skip){ sumS_s += sig; sumS_r += ref; }
    }
    adpd.STOP();
    if (!run_ok || r0 == 0 || sumS_r <= 0){ bad++; continue; }

    const double fr0 = (double)s0 / (double)r0;
    const double frS = sumS_s / sumS_r;
    n++;
    const double d0 = fr0 - m0; m0 += d0 / n; M2_0 += d0 * (fr0 - m0);
    const double dS = frS - mS; mS += dS / n; M2_S += dS * (frS - mS);
  }
  diag_teardown();

  if (n < 2){
    Serial.printf("tratio: only %u good runs (bad=%u) - raise reps or lower freq\n", n, bad);
    Serial.flush();
    return -1;
  }
  const double sem0 = sqrt((M2_0 / (n - 1)) / (double)n);
  const double semS = sqrt((M2_S / (n - 1)) / (double)n);
  const double dev_pct   = (m0 - mS) / mS * 100.0;
  const double noise_pct = 2.0 * sqrt(sem0*sem0 + semS*semS) / mS * 100.0;

  Serial.printf("tratio: runs=%u bad=%u N=%u freq=%u integ=%u skip=%u (lit pulses=%u)\n",
                n, bad, N, freq, integ, skip, (unsigned)reps * N);
  Serial.printf("tratio: first s/r=%.5f (SEM %.5f) steady s/r=%.5f (SEM %.5f)\n", m0, sem0, mS, semS);
  Serial.printf("tratio: deviation=%+.2f%% noise(2SEM)=+/-%.2f%%\n", dev_pct, noise_pct);
  Serial.printf("tratio_verdict: %s\n",
                (fabs(dev_pct) < noise_pct)
                  ? "COMMON-MODE -> relative yield unaffected; no warm-up needed"
                  : "RATIO SHIFTS -> channel-specific settling; warm-up / drop-first warranted");
  Serial.flush();
  return 0;
}
#endif  // AMBIT_DIAG_TRIGGER


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
