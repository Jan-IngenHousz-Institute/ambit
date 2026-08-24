
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "serial.h"
#include "src/mlx90632/u_mlx.h"
#include "src/as7341/spec_meas.h"
#include "data_utils.h"
#include "PAM.h"
#include "core.h"
#include "nvs1.h"
#include <Preferences.h>

int check_connections();
extern Preferences preferences;
extern bool FLAG_DEICE;
static const char* TAG1 = "do_Cmd";

constexpr unsigned hash(const char *string)
{
 return *string == 0 ? 17325 : *string + (*string * hash(string+1));
}

/* Read a ten-element float vector as comma-separated text, the same shape as
 * set_baseline. Strictly parsed: par_weight takes 0.0 as both its shipping
 * default and a legitimate fitted value, so the loose Serial_Input_Double idiom
 * — which cannot separate a deliberate zero from a strtod parse failure — is
 * not good enough here. Consumes all ten tokens before returning either way, so
 * a rejected line does not leave half a vector in the input buffer. */
static bool read_spec_vector(float values[ambit_calibration::SPEC_CHANNEL_COUNT]){
  bool valid = true;
  for (uint8_t i = 0; i < ambit_calibration::SPEC_CHANNEL_COUNT; ++i){
    char token[25] = {0};
    Serial_Input_Chars(token, ",", 10, sizeof(token) - 1);
    if (!ambit_calibration::parse_calibration_float(token, &values[i])) valid = false;
  }
  return valid;
}

static void report_spec_save(const char *what, esp_err_t err){
  if (err == ESP_OK) Serial.printf("%s saved and verified\n", what);
  else Serial.printf("%s save failed: %s\n", what, esp_err_to_name(err));
}

void do_command(char *choose){

    int setting;
  // char choose[50]; //buffer to hold commands

  //Serial_Input_Chars(choose, "+", 500, sizeof(choose) - 1);

  for (unsigned i = 0; i < strlen(choose); ++i) {   // remove ctrl characters
    if (!isprint(choose[i]))
      choose[i] = 0;
  }

  if (strlen(choose) < 1) {         // short or null command, quietly ignore it
    return;
  }

  if (!isalnum(choose[0])) {
    // Serial_Printf("error: not a-n\n");
    return;                         // go read another command
  }

  unsigned val;                     // we accept int or alpha commands
  if (isdigit(choose[0])) {
    val = atoi(choose);
  }
  else {
    val = hash(choose);             // convert alpha command to an int
  }


  //Serial.printf("cmd: %s\n", choose);
  // process single commands
  switch (val) {
    case hash("hello"):
     {
      // "NEW" and "Ready" are the sentinels the openJII app driver and the
      // Calibratron match (substring / \bready\b); the name and FW:<semver>
      // tokens are additive, both hosts tolerate them.
      Serial.print("NEW ");
      Serial.print(ambit_calibration_local.ambit_name);
      Serial.print(" Ready FW:");
      Serial.println(AMBIT_FW_VERSION);
    }
      break;

    int external_trigger_run(void);
    case hash("ww"):
     {
      Serial.println("Start");
      external_trigger_run();
      Serial.println("Exit");

    }                                                                   
      break;

    int external_trigger_run_Flash(unsigned int gate_time, unsigned int dt, const uint16_t num);
    case hash("ff"):
     {
      unsigned int t = Serial_Input_Long(",", 10);
      unsigned int dt = Serial_Input_Long(",", 10);
      unsigned int num = Serial_Input_Long(",", 10);
      external_trigger_run_Flash(t, dt, num);

    }                                                                   
      break;



    case hash("set_currents"):
     {
      uint8_t i620 = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t i720 = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t ir   = (uint8_t) Serial_Input_Long(",", 10);
      core_set_currents(i620, i720, ir);
      Serial.printf("Currents set to %d, %d, %d\n", adpd_current_config.I620, adpd_current_config.I720, adpd_current_config.IR);
     }                                                
    break;  

    case hash("set_gains"):
     {
      uint8_t fluo    = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t fluoRef = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t ir      = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t irRef   = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t sun     = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t leaf    = (uint8_t) Serial_Input_Long(",", 10);
      core_set_gains(fluo, fluoRef, ir, irRef, sun, leaf);
      Serial.printf("Gains set to %d, %d, %d, %d, %d, %d\n", adpd_gains_config.Fluo, adpd_gains_config.FluoRef, adpd_gains_config.IR, adpd_gains_config.IRRef, adpd_gains_config.Sun, adpd_gains_config.Leaf);
     }                                                
    break;  

    case hash("mlx"):
    { 
      unsigned int timer = millis();
      for (uint8_t i = 0; i < 100; i++){
        Serial.println(mlx_measure());
      }
      Serial.printf("Spend %f ms per measurement", (millis() - timer)/100.0);
    }    
    break;

    case hash("temp"):
    { 
      // uint32_t ret;
      // uint8_t mode = 5;
      // float_t temp = 0.0;


      // ret = PAM_get_env(0, 500);
      // Serial.print(PAM_retrieve_env(ret, &mode));
      // Serial.printf(" %d \n", mode);

      // ret = PAM_get_env(4, 600);
      // Serial.println(PAM_retrieve_env(ret, &mode, &temp));
      // Serial.printf(" %d %f \n", mode, temp);
      double obj,amb,obj_r;
      int16_t a1,a2,a3,a4;

      mlx_measure(&obj, &amb, &obj_r, &a1, &a2, &a3, &a4);
      Serial.printf("%.4f\t%.4f\t%.4f\n", obj, amb, obj_r);


    }    
    break;

    case hash("clean_nvs"):
    {
      nvs_flash_erase();
      nvs_flash_init();
      Serial.println("NVS cleaned");
    }
    break;

    case hash("baseline"):
    {
      uint8_t c = Serial_Input_Long(",", 10);
      int acquisition_err = conf_slow_FR_1(100, 20, 0, 1, 5, 5, 1, 5, 5);
      uint32_t ret[6] = {0};
      if (acquisition_err == 0) acquisition_err = fluor_offset(ret);
      if (acquisition_err != 0){
        Serial.printf("Baseline acquisition failed: %d\n", acquisition_err);
        break;
      }
      Serial.printf("%d,%d,%d,%d,%d,%d\n", ret[0], ret[1], ret[2], ret[3], ret[4], ret[5]);
      if (c == 1){
        if (ret[0] > ambit_calibration::MAX_S630_BASELINE){
          Serial.println("Baseline too high");
          break;
        }
        esp_err_t baseline_err = save_adpd_baseline(ret);
        if (baseline_err == ESP_OK) Serial.println("Baseline saved and verified");
        else Serial.printf("Baseline save failed: %s\n", esp_err_to_name(baseline_err));
      }
    }
    break;

    case hash("set_baseline"):
    {
      uint32_t values[ambit_calibration::CHANNEL_COUNT] = {0};
      bool valid = true;
      for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        char token[25] = {0};
        Serial_Input_Chars(token, ",", 10, sizeof(token) - 1);
        if (!ambit_calibration::parse_adpd_baseline_value(token, &values[i])) valid = false;
      }
      if (!valid || !ambit_calibration::valid_adpd_baseline(values)){
        Serial.println("Baseline rejected");
        break;
      }
      esp_err_t baseline_err = save_adpd_baseline(values);
      if (baseline_err == ESP_OK) Serial.println("Baseline saved and verified");
      else Serial.printf("Baseline save failed: %s\n", esp_err_to_name(baseline_err));
    }
    break;

    case hash("tttt"):
      Serial.println(sizeof(ambit_calibration_info_t));
    break;



    case hash("arrun"):
     {
      uint8_t len = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t persist = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t arr[128] = {0};
      uint8_t tmp_8 = 0;
      for (uint8_t i = 0; i < len; i++){
        for (uint8_t j = 0; j < 8; j++){
          arr[i * 8 + j] = (uint8_t) Serial_Input_Long(",", 10);
        }
      }

      core_run_array(16, arr, persist, false);
    }
      break;  

    case hash("arrun1"):
     {
      uint8_t len = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t persist = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t arr[128] = {0};
      uint8_t tmp_8 = 0;
      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;


      for (uint8_t i = 0; i < len; i++){
        for (uint8_t j = 0; j < 8; j++){
          arr[i * 8 + j] = (uint8_t) Serial_Input_Long(",", 10);
        }
      }

      core_run_array(16, arr, persist, false);
    }
      Serial.println("Done");
      break;

    // arrun2 — like arrun1 but COMPUTER mode. Runs the whole trace, then dumps
    // each data array as one ASCII block ("Data:<tag>,Length:N\t v,v,...,")
    // ending with "Data sent". No per-point streaming → no inter-point UART
    // gaps, so it stays in sync on the ambyte (device-to-device) link.
    case hash("arrun2"):
     {
      uint8_t len = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t persist = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t arr[128] = {0};
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;

      for (uint8_t i = 0; i < len; i++){
        for (uint8_t j = 0; j < 8; j++){
          arr[i * 8 + j] = (uint8_t) Serial_Input_Long(",", 10);
        }
      }

      core_run_array(16, arr, persist, false);
    }
      break;


      case hash("q"):
     {
      uint8_t a = (uint8_t) Serial_Input_Long(",", 10);
      
      uint8_t b = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t c = (uint8_t) Serial_Input_Long(",", 10);
      
      uint8_t arr[24] = {a, 0, 1, 0, 0, b, 0, 1, \
                        a, 0, 1, 0, 0, b, c, 1,\
                        a, 0, 1, 0, 0, b, 0, 1};

      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      //CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      core_run_array(3, arr, 0, false);
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;

    case hash("get_par"):
    {      
        uint16_t spec[10];        
        Serial.println(get_PAR(spec));
        Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",spec[0],spec[1],spec[2],spec[3],spec[4],spec[5],spec[6],spec[7],spec[8],spec[9]);      
    }
    break;

    case hash("PAR"):
    {      
        uint16_t spec[10];
        Serial.println(get_PAR(spec) * ambit_calibration_local.spec_coef);
        Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",spec[0],spec[1],spec[2],spec[3],spec[4],spec[5],spec[6],spec[7],spec[8],spec[9]);      
    }
    break;


        case hash("awake"):
    {

      while(1){
        uint16_t spec[10];        
        get_PAR(spec);
      }
      
    }
    break;


  case hash("r"):
     {
      uint8_t a = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t b = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t c = (uint8_t) Serial_Input_Long(",", 10);
      
      uint8_t arr[24] = {a, 0, 1, 0, 0, b, 0, 1, \
                        a, 0, 1, 0, 0, b, c, 1,\
                        a, 0, 1, 0, 0, b, 0, 1};

      CONNECTION_TYPE = CONNECTION_TYPES::AMBYTE;
      //CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      FLAG_DEICE = true;
      core_run_array(3, arr, 0, true);
      FLAG_DEICE = false;
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;

      case hash("w"):
     {
      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      uint16_t length = Serial_Input_Long(",", 10);
      uint8_t interval = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t act = (uint8_t) Serial_Input_Long(",", 10);
      core_run_mpf(length, interval, false, act);      
    }                                                                   
      break;        
      
      
      case hash("a"):
     {
      uint8_t a = Serial_Input_Long(",", 10);
      uint8_t b = Serial_Input_Long(",", 10);

      as7431_blink(a, b);
      
      
    }                                                                   
      break;  

         
      case hash("aa"):
     {
      uint8_t a = Serial_Input_Long(",", 10);
      
      for (uint8_t i = 0; i < 4; i++){
        as7431_blink(i, a);
      }

      for (uint8_t i = 0; i < 4; i++){
        as7431_blink(i, a);
      }

      for (uint8_t i = 0; i < 4; i++){
        as7431_blink(i, a);
      }

      for (uint8_t i = 0; i < 4; i++){
        as7431_blink(i, a);
      }

      
    }                                                                   
      break;  


   


      case hash("set_act"):
      {
        float_t a = (float_t) Serial_Input_Double(",", 10);
        if (ambit_calibration::valid_actinic_coefficient(a)){
          const esp_err_t err = save_actinic_coefficient(a);
          if (err != ESP_OK) Serial.printf("Actinic coefficient save failed: %s\n", esp_err_to_name(err));
        }else{
          Serial.println("Actinic coefficient rejected");
        }
      }                                                                   
      break;

      case hash("set_name"):
      {
        char s[16];

        Serial_Input_Chars(s, ",\n\r", 10, 15);
        preferences.begin("config", false);
        preferences.putString("name", s);
        preferences.end();
      }                                                                  
      break;

      case hash("set_emit"):
      {
        double a = Serial_Input_Double(",", 10);
        preferences.begin("config", false);
        preferences.putDouble("emit", a);
        preferences.end();
      }                                                                   
      break;

      case hash("set_spec"):
      {
          float_t f = (float_t) Serial_Input_Double(",", 10);
          if (ambit_calibration::valid_spec_coefficient(f)){
            const esp_err_t err = save_spec_coefficient(f);
            if (err != ESP_OK) Serial.printf("PAR coefficient save failed: %s\n", esp_err_to_name(err));
          }else{
            Serial.println("PAR coefficient rejected");
          }
      }
      break;

      /* Spectral/PAR calibration — the three-tier chain cmd 35 computes.
       * See plans/AMBIT_COMMAND35_SPECPAR.md.
       *
       * These live on the text console rather than as new binary cmd 17/18
       * subtypes: this is the dialect the Calibratron speaks, and the
       * Calibratron is what runs the calibration. It also reports acceptance,
       * where cmds 17/18 write nothing at all on an unrecognised subtype — no
       * ESP_CMD_DONE, no ESP_CMD_END — so an older image costs the host its full
       * read timeout and, for cmd 18, leaves the unread vector payload behind to
       * desync the next frame.
       *
       * Vectors are ten comma-separated floats in wavelength order:
       * F1..F8, NIR, Clear. Validation and persistence both happen inside the
       * save_* helpers, so every transport shares one predicate. */
      case hash("set_spec_offset"):
      {
        float values[ambit_calibration::SPEC_CHANNEL_COUNT] = {0.0f};
        if (!read_spec_vector(values) || !ambit_calibration::valid_spec_offset(values)){
          Serial.println("Spectral offset rejected");
          break;
        }
        report_spec_save("Spectral offset", save_spec_offset(values));
      }
      break;

      case hash("set_spec_sens"):
      {
        float values[ambit_calibration::SPEC_CHANNEL_COUNT] = {0.0f};
        if (!read_spec_vector(values) || !ambit_calibration::valid_spec_sens(values)){
          Serial.println("Spectral sensitivity rejected");
          break;
        }
        report_spec_save("Spectral sensitivity", save_spec_sens(values));
      }
      break;

      case hash("set_par_weight"):
      {
        float values[ambit_calibration::SPEC_CHANNEL_COUNT] = {0.0f};
        if (!read_spec_vector(values) || !ambit_calibration::valid_par_weight(values)){
          Serial.println("PAR weight rejected");
          break;
        }
        report_spec_save("PAR weight", save_par_weight(values));
      }
      break;

      case hash("set_par_slope"):
      {
        float value = 0.0f;
        char token[25] = {0};
        Serial_Input_Chars(token, ",", 10, sizeof(token) - 1);
        if (!ambit_calibration::parse_calibration_float(token, &value) ||
            !ambit_calibration::valid_par_slope(value)){
          Serial.println("PAR slope rejected");
          break;
        }
        report_spec_save("PAR slope", save_par_slope(value));
      }
      break;

      case hash("set_par_icept"):
      {
        float value = 0.0f;
        char token[25] = {0};
        Serial_Input_Chars(token, ",", 10, sizeof(token) - 1);
        if (!ambit_calibration::parse_calibration_float(token, &value) ||
            !ambit_calibration::valid_par_intercept(value)){
          Serial.println("PAR intercept rejected");
          break;
        }
        report_spec_save("PAR intercept", save_par_intercept(value));
      }
      break;

      /* Text mirror of cmd 33 subtype 4. Onboarding needs to confirm a vector
       * actually landed in NVS, and the Calibratron should not have to speak
       * binary to do it. */
      case hash("get_spec_cal"):
      {
        Serial.print("spec_offset:");
        for (uint8_t i = 0; i < ambit_calibration::SPEC_CHANNEL_COUNT; ++i){
          Serial.printf("%s%.9g", i ? "," : "", ambit_spec_calibration.spec_offset[i]);
        }
        Serial.print("\nspec_sens:");
        for (uint8_t i = 0; i < ambit_calibration::SPEC_CHANNEL_COUNT; ++i){
          Serial.printf("%s%.9g", i ? "," : "", ambit_spec_calibration.spec_sens[i]);
        }
        Serial.print("\npar_weight:");
        for (uint8_t i = 0; i < ambit_calibration::SPEC_CHANNEL_COUNT; ++i){
          Serial.printf("%s%.9g", i ? "," : "", ambit_spec_calibration.par_weight[i]);
        }
        Serial.printf("\npar_slope:%.9g\npar_intercept:%.9g\n",
                      ambit_spec_calibration.par_slope,
                      ambit_spec_calibration.par_intercept);
      }
      break;

       case hash("reboot"):
      {
        ESP.restart();
      }
      break;

      case hash("check"):
      {
        check_connections();
      }
      break;
      
    int optic_test();
      case hash("test"):
      {
        optic_test();
      }
      break;

      int optic_test(uint8_t current, uint8_t num_integ, uint8_t lit_offset, uint8_t dark1_offset, uint8_t dark2_offset, uint8_t pulse_offset, uint8_t pulse_duration);
      
      case hash("test1"):
      {
        uint8_t current = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t num_integ = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t lit_offset = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t dark1_offset = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t dark2_offset = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t pulse_offset = (uint8_t) Serial_Input_Long(",", 10);
        uint8_t pulse_duration = (uint8_t) Serial_Input_Long(",", 10);
        optic_test(current, num_integ, lit_offset, dark1_offset, dark2_offset, pulse_offset, pulse_duration);
      }
      break;
    default:
      Serial.println("BAD COMMAND");
    break;

  }
}
