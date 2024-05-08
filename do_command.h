
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "serial.h"
#include "src/mlx90632/u_mlx.h"
#include "src/as7341/spec_meas.h"
#include "src/wrench.h"
#include "data_utils.h"
#include "PAM.h"

static const char* TAG1 = "do_Cmd";
void do_c(const char* c);

constexpr unsigned hash(const char *string)
{
 return *string == 0 ? 17325 : *string + (*string * hash(string+1));
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


  Serial.printf("cmd: %s\n", choose);
  // process single commands
  switch (val) {
    case hash("C"):{

      char c[500];
      Serial_Input_Chars(c, "?", 10000, 500);
      do_c(c);
      //Serial.println();
    }
    break;


    case hash("hello"):
     {
      Serial.print("NEW Name Here");
      Serial.println(" Ready");
    }                                                                   
      break;

      
    case hash("mpf"):
     {
      uint16_t m = Serial_Input_Long(",", 100);
      uint16_t l = Serial_Input_Long(",", 100);
      uint16_t n = Serial_Input_Long(",", 100);
      uint16_t g1 = Serial_Input_Long(",", 100);
      uint16_t g2 = Serial_Input_Long(",", 100);
      MPF(m, l, n, g1, g2);
    }                                                                   
      break;


    case hash("set_currents"):
     {
      adpd_current_config.I620 = (uint8_t) Serial_Input_Long(",", 10);
      adpd_current_config.I720 = (uint8_t) Serial_Input_Long(",", 10);
      adpd_current_config.IR = (uint8_t) Serial_Input_Long(",", 10);
      adpd_current_config.init = true;
      Serial.printf("Currents set to %d, %d, %d\n", adpd_current_config.I620, adpd_current_config.I720, adpd_current_config.IR);
      adpd_mode = ADPD_CONFIG_MODE::MPF_MODE; // not applied
     }                                                
    break;  

    case hash("set_gains"):
     {
      adpd_gains_config.Fluo = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.FluoRef = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.IR = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.IRRef = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.Sun = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.Leaf = (uint8_t) Serial_Input_Long(",", 10);
      adpd_gains_config.init = true;
      Serial.printf("Gains set to %d, %d, %d, %d, %d, %d\n", adpd_gains_config.Fluo, adpd_gains_config.FluoRef, adpd_gains_config.IR, adpd_gains_config.IRRef, adpd_gains_config.Sun, adpd_gains_config.Leaf);
      adpd_mode = ADPD_CONFIG_MODE::MPF_MODE; // not applied
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
      uint32_t ret;
      uint8_t mode = 5;
      float_t temp = 0.0;


      ret = PAM_get_env(0, 500);
      Serial.print(PAM_retrieve_env(ret, &mode));
      Serial.printf(" %d \n", mode);

      ret = PAM_get_env(4, 600);
      Serial.println(PAM_retrieve_env(ret, &mode, &temp));
      Serial.printf(" %d %f \n", mode, temp);


    }    
    break;





    case hash("arrun"):
     {
      uint8_t len = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t persist = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t arr[64] = {0};
      uint8_t tmp_8 = 0;
      for (uint8_t i = 0; i < len; i++){
        for (uint8_t j = 0; j < 8; j++){
          arr[i * 8 + j] = (uint8_t) Serial_Input_Long(",", 10);
        }
      }

      if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
        conf_slow_FR_1();
        adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
      }
      run_arr_type1(8, arr, persist);
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

      //CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
        conf_slow_FR_1();
        adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
      }
      run_arr_type1(3, arr, 0);
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;

      case hash("w"):
     {
      uint8_t m = (uint8_t) Serial_Input_Long(",", 10);

      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      MPF(m, 0);
      adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;  

   


    //   case hash("sd"):
    //  {
    //    sandbox(pulsed_620_current, gain_fluor, 4);
    //    Serial.println("Cmd Done!");
    // }                                                                   
    //   break;

       case hash("reboot"):
      {
        ESP.restart();
      }
      break;

      case hash("test"):
      {
        Serial.println(mlx_measure());
        Serial.println(get_PAR());
        for (uint8_t i = 0; i < 10; i++){
            AS_LED_OFF();
            AS_LED_Current(20 * i);
            AS_LED_ON();
            delay(10);
            AS_LED_OFF();
        }


      }
      break;

    default:
      Serial.println("BAD COMMAND");
    break;

  }
}