#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/devices_init.h"
#include "src/mlx90632/u_mlx.h"
static const char* TAG = "PAM";

extern ADPD6 adpd;

#define MAX_MEMORY_ALLOC 25000


int detector_preset_1(uint8_t current, uint8_t gain_fluo, uint8_t gain_ref, uint8_t gain_par_ir, uint8_t gain_par_vis){

  // Setup timeslot 1: two ambient light channels
  adpd.led_config.driver1_current = 0;
  adpd.led_config.driver2_current = 0;
  adpd.SNR_config.TIA_gain_CH2 = gain_par_ir;
  adpd.SNR_config.TIA_gain_CH1 = gain_par_vis;
  adpd.preset_config_1(0, 4);

  // Setup timeslot 2:  Fluor and Ref channels


    // LED 1A = 620nm
  adpd.led_config.driver1_current = current;
  adpd.led_config.driver1_current = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = current;
  adpd.led_config.led2_channel = LED_A;

  adpd.SNR_config.TIA_gain_CH1 = gain_fluo;
  adpd.SNR_config.TIA_gain_CH2 = gain_ref;

  adpd.preset_config_2(1, 4);
  ESP_LOGI(TAG, "Preset 1 set");
  return 0;
}

uint32_t arr_line_parse_type1(uint8_t* line, uint8_t* num1, uint16_t* num2, uint16_t* num3, uint8_t* num4, uint8_t* num5, uint16_t* data_count){
  uint16_t _sub_count = 0;
  uint8_t type = line[0];                 // run type 1 = steady state
  *num1 = line[1];                        // reserved
  *num2 = line[3] + (line[2] << 8);       // sample number
  *num3 = line[5] + (line[4] << 8);       // frequency
  *num4 = line[6];                        // actinic
  *num5 = line[7];                        // sub-sampling factor
  data_count[0] = *num2;
  data_count[1] = *num2;

  if (line[7] == 0){
    _sub_count = 0;
  }
  else{
    _sub_count = ((*num2) / line[7]);
  }
  data_count[2] = _sub_count;
  data_count[3] = _sub_count;
  return _sub_count + _sub_count + *num2 + *num2;
}

int run_preprocess(uint8_t length, uint8_t* arr, uint16_t* data_counter){
    uint8_t pc = 0;
    uint8_t _type = 0;
    uint8_t para1, actinic, subsampling = 0;
    uint16_t num_ptx, freq = 0;
    uint16_t _data_counter[4] = {0};

    while (pc < length){
      _type = *(arr + pc * 8);
      if (_type == 1){
        arr_line_parse_type1(arr + pc * 8, &para1, &num_ptx, &freq, &actinic, &subsampling, _data_counter);
        ESP_LOGV(TAG, "Run type:%d, number:%d, freq: %d, actinic: %d, subfactor %d", _type, num_ptx, freq, actinic, subsampling);
        for (uint8_t i = 0; i < 4; i++) data_counter[i] += _data_counter[i];
      }      
      pc += 1;
    }
    return 0;

}

int data_allocation(void** F_data, void **R_data, void **A_data, void **D_data, uint16_t* data_count){

  if ((data_count[0] + data_count[1] + data_count[2] + data_count[3]) > MAX_MEMORY_ALLOC){
    ESP_LOGE(TAG, "More than %d", MAX_MEMORY_ALLOC*4);
    return -1;
  }


  if ((data_count[0] > 0) and data_count[0] < MAX_MEMORY_ALLOC) *F_data = heap_caps_calloc(data_count[0], sizeof(uint32_t), MALLOC_CAP_32BIT);
  if (*F_data == NULL) goto F_data_failed;
  ESP_LOGV(TAG, "Allocation F_data: %d bytes", data_count[0] * 4);

  if ((data_count[1] > 0) and data_count[1] < MAX_MEMORY_ALLOC) *R_data = heap_caps_calloc(data_count[1], sizeof(uint32_t), MALLOC_CAP_32BIT);
  if (*R_data == NULL) goto R_data_failed;
  ESP_LOGV(TAG, "Allocation R_data: %d bytes", data_count[1] * 4);

  if ((data_count[2] > 0) and data_count[2] < MAX_MEMORY_ALLOC) *A_data = heap_caps_calloc(data_count[2], sizeof(uint32_t), MALLOC_CAP_32BIT);
  if (*A_data == NULL) goto A_data_failed;
  ESP_LOGV(TAG, "Allocation A_data: %d bytes", data_count[2] * 4);

  if ((data_count[3] > 0) and data_count[3] < MAX_MEMORY_ALLOC) *D_data = heap_caps_calloc(data_count[3], sizeof(uint32_t), MALLOC_CAP_32BIT);
  if (*D_data == NULL) goto D_data_failed;
  ESP_LOGV(TAG, "Allocation D_data: %d bytes", data_count[3] * 4);

  

  return 0;

  D_data_failed:
  free(D_data);
  A_data_failed:
  free(A_data);
  R_data_failed:
  free(R_data);
  F_data_failed:
  free(F_data);
  ESP_LOGE(TAG, "Allocation failed");
  return -1;
}



int run_arr(uint8_t length, uint8_t* arr){
  const uint8_t expected_readout = 6;
  const uint8_t expected_readout_bytes = expected_readout * 3;
  const uint8_t num_integration = 4;

  // Run protocol preprecess, get storage size
  uint16_t data_count[] = {0, 0, 0, 0};
  if (run_preprocess(length, arr, data_count) == -1) return -1;    // calculate data counts
  ESP_LOGV(TAG, "Sample: %d, ref: %d, amb: %d, dark: %d", data_count[0], data_count[1], data_count[2], data_count[3]);
  uint32_t *F_data = NULL;
  uint32_t *R_data = NULL;
  uint32_t *A_data = NULL;
  uint32_t *D_data = NULL;
  if (data_allocation((void**)(&F_data), (void**)&R_data, (void**)&A_data, (void**)&D_data, data_count) == -1) return -1;
  F_data[0] = 0;
  ESP_LOGV(TAG, "Memory allocation completed");

  // configure the timeslots
  detector_preset_1(100, 1, 4, 4, 4);

  // variables for each trace
  uint8_t pc = 0;
  uint8_t _type = 0;
  uint8_t para1, actinic, subsampling = 0;
  uint16_t num_ptx, freq = 0;

  // data counter and buffer
  // [sun-amb, leaf-ir, lit_leaf-ir, dark_leaf-ir, lit_leaf-ref, dark_leaf-ref]
  uint32_t ret[expected_readout] = {0};
  uint32_t counter = 0;
  uint16_t fifo_c = 0;
  uint8_t watch_dog_timer = 0;
  int32_t tmp_var = 0;  


  adpd.STOP();
  while (pc < length){
    _type = *(arr + pc * 8);
    if (_type == 1){
      arr_line_parse_type1((arr + pc * 8), &para1, &num_ptx, &freq, &actinic, &subsampling, data_count);
      adpd.run_freq(freq);
      adpd.clear_fifo();
      if (actinic > 4){
        AS_LED_Current(actinic);
        AS_LED_ON();
      }else{
        AS_LED_OFF();
      }
      counter = 0;
      adpd.RUN();

      while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){
          adpd.readfifo(expected_readout, 3, ret);
          fifo_c -= expected_readout_bytes;
          if (counter == num_ptx) break;
          tmp_var = ((int)ret[3] - (int)ret[2] + 250) - (0.006 / (int)num_integration) * ((int)ret[2] - 16384 * num_integration);
          F_data[counter] = 0;
          if ((tmp_var > 0) && (ret[3] > ret[2])) F_data[counter] = tmp_var;
          R_data[counter] = ret[5] - ret[4];
          A_data[counter] = ret[0];
          D_data[counter] = ret[1];

          if (true){
            Serial.print(F_data[counter]);
            Serial.print(",");
            Serial.print(R_data[counter]);
            Serial.print(",");
            Serial.print(A_data[counter]);
            Serial.print(",");
            Serial.print(D_data[counter]);
            Serial.println();
          }
          counter++;
          watch_dog_timer = 0;
        }

        // esp_sleep_enable_timer_wakeup(wait_time * 5000);
        // esp_light_sleep_start();
        delay(1);    
      }
      adpd.STOP();
      AS_LED_OFF();
    }    
    pc += 1;
  }



  free(D_data);
  free(A_data);
  free(R_data);
  free(F_data);
  
  return 0;


}






int steady_state(uint8_t current, uint8_t gain_fluo, uint8_t gain_ref, uint8_t gain_dark, uint16_t freq, uint16_t num, uint16_t extra_info1, uint16_t extra_info2){

  const uint8_t num_integration = 4;

  uint32_t *F_data = NULL;
  uint32_t *R_data = NULL;
  uint32_t *A_data = NULL;
  

  uint16_t wait_time = 1000 / freq;
  if (wait_time > 1000) wait_time = 1000;
  
  F_data = (uint32_t*)heap_caps_calloc(num, sizeof *F_data, MALLOC_CAP_32BIT);
  if (F_data == NULL) return -8;
  R_data = (uint32_t*)heap_caps_calloc(num, sizeof *R_data, MALLOC_CAP_32BIT);
  if (R_data == NULL){
    free(F_data);
    return -9;
  };
  A_data = (uint32_t*)heap_caps_calloc(num, sizeof *A_data, MALLOC_CAP_32BIT);
  if (A_data == NULL){
    free(F_data);
    free(R_data);
    return -10;
  };
  


    // LED 1A = 620nm
  adpd.led_config.driver1_current = 40;
  adpd.led_config.driver1_current = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;

  
  //adpd.SNR_config.TIA_gain_CH1 = gain_dark;

  adpd.led_config.driver2_current = current;
  adpd.SNR_config.TIA_gain_CH1 = gain_fluo;
  adpd.SNR_config.TIA_gain_CH2 = gain_ref;
  adpd.preset_config_2(0, num_integration);


  adpd.run_freq(freq);
  adpd.clear_fifo();
  //AS_LED_OFF();

  unsigned long start_time = millis(); 

  adpd.RUN();
  uint32_t ret[6] = {0};
  uint32_t counter = 0;
  uint16_t fifo_c = 0;
  uint8_t watch_dog_timer = 0;
  int32_t tmp_var = 0;

  
  while (counter < num){
    fifo_c = adpd.fifo_count();
    while (fifo_c > 14){
      adpd.readfifo(5, 3, ret);
      if (counter == num) break;

      tmp_var = (ret[2] - ret[1] + 250) - (0.006 / num_integration) * ((int)ret[1] - 16384 * num_integration);

      F_data[counter] = 0;
      if (tmp_var > 0) F_data[counter] = tmp_var;
      R_data[counter] = ret[4] - ret[3];
      A_data[counter] = ret[0];


      if (true){
        Serial.print(int((float)F_data[counter] / R_data[counter] * 1000));
        Serial.print(",");
        Serial.print(F_data[counter]);
        Serial.print(",");
        Serial.print(R_data[counter]);
        Serial.print(",");
        Serial.println(A_data[counter]);
      }

      // if (printout){
      //   Serial.print(ret[2] - 16384 * num_integration + 1000);
      //   Serial.print(",");
      //   Serial.print(ret[1] - 16384 * num_integration + 1000);
      //   Serial.print(",");
      //   Serial.print(tmp_var);
      //   Serial.print(",");
      //   Serial.println(A_data[counter]);
      // }


      counter++;
      watch_dog_timer = 0;
      fifo_c -= 15;
    }

    // esp_sleep_enable_timer_wakeup(wait_time * 5000);
    // esp_light_sleep_start();
    delay(wait_time);
    watch_dog_timer += 1;
    if (watch_dog_timer > 4){
      Serial.println("RUN timeout");
      break;
    }
    
  }
  adpd.STOP();

  
  
  free(F_data);
  free(R_data);
  free(A_data);

  if (watch_dog_timer > 4) return -8;

  return 1;
}