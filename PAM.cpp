#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/devices_init.h"
#include "src/mlx90632/u_mlx.h"
static const char* TAG = "PAM";

extern ADPD6 adpd;




int detector_preset_1(uint8_t current, uint8_t gain_fluo, uint8_t gain_ref, uint8_t gain_dark){
    // LED 1A = 620nm
  adpd.led_config.driver1_current = 40;
  adpd.led_config.driver1_current = LED_A;
    // LED 2A = 730nm
  adpd.led_config.driver2_current = 0;
  adpd.led_config.led2_channel = LED_A;

  
  adpd.led_config.driver2_current = current;
  adpd.SNR_config.TIA_gain_CH1 = gain_fluo;
  adpd.SNR_config.TIA_gain_CH2 = gain_ref;
  adpd.preset_config_2(0, 4);
  ESP_LOGI(TAG, "Preset 1 set");
  return 0;
}

int arr_line_parse(uint8_t* line, uint8_t* num1, uint16_t* num2, uint16_t* num3){
  uint8_t type = line[0];
  *num1 = line[1];
  *num2 = line[3] + (line[2] << 8);
  *num3 = line[5] + (line[4] << 8);
  return type;
}


int run_arr(uint8_t length, uint8_t* arr){
  uint8_t pc = 0;
  uint8_t _type = 0;
  uint8_t para1 = 0;
  uint16_t para2, para3;
  while (pc < length){
    _type = arr_line_parse(arr + pc * 8, &para1, &para2, &para3);
    ESP_LOGI(TAG, "Parse result: type: %d, para1: %d, para2: %d, para3: %d", _type, para1, para2, para3);
    pc += 1;
  }

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