
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "serial.h"
#include "src/mlx90632/u_mlx.h"
#include "src/as7341/spec_meas.h"
#include "src/wrench.h"
#include "data_utils.h"


//#include "src/adpd/u_adpd6100.h"
static const char* TAG1 = "DOCMD";
//extern ADPD6 adpd;

extern uint8_t pulsed_620_current, pulsed_720_current, dc_current;
extern uint8_t gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf;
extern uint8_t status_run_config_set;



dataclass data;
int sandbox(uint8_t I620, uint8_t g1, uint8_t g2);
int MPF(uint16_t mode, uint16_t current, uint16_t dc_current,uint8_t,uint8_t);
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int run_arr_type1(uint8_t length, uint8_t* arr, bool);



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

  void do_c(const char* c);
  void do_E(const char* c);

  Serial.printf("cmd: %s", choose);
  // process single commands
  switch (val) {
    case hash("C"):{

      char c[500];
      Serial_Input_Chars(c, "?", 10000, 500);
      do_c(c);
      //Serial.println();
    }
    break;


    case hash("E"):{

      char c[500];
      Serial_Input_Chars(c, "?", 10000, 500);
      do_E(c);
      //Serial.println();
    }
    break;

    case hash("S"):{
      data.init(100);
      for (uint16_t i = 0; i < 100; i++){
        data.put(i);
      }      
      data.send_esp(0);
      data.clear();

      for (uint16_t i = 0; i < 100; i++){
        data.put(100 - i);
      } 

      data.send_esp(1);
      data.clear();


      for (uint16_t i = 0; i < 100; i++){
        data.put(200 - i);
      }
      data.send_esp(2);
      data.clear();




      data.clean();

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
      pulsed_620_current = (uint8_t) Serial_Input_Long(",", 10);
      pulsed_720_current = (uint8_t) Serial_Input_Long(",", 10);
      dc_current = (uint8_t) Serial_Input_Long(",", 10);
      Serial.printf("Currents set to %d, %d, %d\n", pulsed_620_current, pulsed_720_current, dc_current);
      status_run_config_set = 0;
     }                                                
    break;  

    case hash("set_gains"):
     {
      gain_fluor = (uint8_t) Serial_Input_Long(",", 10);
      gain_fluref = (uint8_t) Serial_Input_Long(",", 10);
      gain_720 = (uint8_t) Serial_Input_Long(",", 10);
      gain_720ref = (uint8_t) Serial_Input_Long(",", 10);
      gain_sun = (uint8_t) Serial_Input_Long(",", 10);
      gain_leaf = (uint8_t) Serial_Input_Long(",", 10);
      Serial.printf("Gains set to %d, %d, %d, %d, %d, %d\n", gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf);
      status_run_config_set = 0;
     }                                                
    break;  





    case hash("arrun"):
     {
      uint8_t checksum = (uint8_t) Serial_Input_Long(",", 10);
      uint8_t arr[64] = {0};
      uint8_t tmp_8 = 0;
      for (uint8_t i = 0; i < 8; i++){
        for (uint8_t j = 0; j < 8; j++){
          arr[i * 8 + j] = (uint8_t) Serial_Input_Long(",", 10);
          tmp_8 += arr[i * 8 + j];
        }
      }

      if (checksum == tmp_8){
        Serial.println("arr checked");
        if (status_run_config_set == 0){
          conf_slow_FR_1(pulsed_620_current, pulsed_720_current, dc_current, gain_fluor, gain_fluref, gain_sun, gain_leaf, gain_720, gain_720ref);
          status_run_config_set = 1;
        }
        run_arr_type1(8, arr, 0);
      }
    }                                                                   
      break;  


      case hash("q"):
     {
      uint8_t arr[24] = {1, 0, 2, 0, 0, 50, 0, 1, \
                        1, 0, 2, 0, 0, 100, 200, 1,\
                        1, 0, 2, 0, 0, 50, 0, 1};

      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      if (status_run_config_set == 0){
        conf_slow_FR_1(pulsed_620_current, pulsed_720_current, dc_current, gain_fluor, gain_fluref, gain_sun, gain_leaf, gain_720, gain_720ref);
        status_run_config_set = 1;
      }
      run_arr_type1(3, arr, 0);
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;

      case hash("w"):
     {
      uint8_t m = (uint8_t) Serial_Input_Long(",", 10);

      CONNECTION_TYPE = CONNECTION_TYPES::PLOTTING;
      MPF(m, pulsed_620_current, 0, gain_fluor, gain_fluref);
      status_run_config_set = 0;
      CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
      
    }                                                                   
      break;  

   


      case hash("sd"):
     {
       sandbox(pulsed_620_current, gain_fluor, 4);
       Serial.println("Cmd Done!");
    }                                                                   
      break;

    //   int detector_preset_1(uint8_t current, uint8_t gain_fluo, uint8_t gain_ref, uint8_t gain_par_ir, uint8_t gain_par_vis);

    //   case hash("config"):
    //  {
    //   uint8_t current = Serial_Input_Long(",", 100);
    //   uint8_t gain_fluo = Serial_Input_Long(",", 100);
    //   uint8_t gain_ref = Serial_Input_Long(",", 100);
    //   uint8_t gain_par_ir = Serial_Input_Long(",", 100);
    //   uint8_t gain_par_vis = Serial_Input_Long(",", 100);
    //   detector_preset_1(current, gain_fluo, gain_ref, gain_par_ir, gain_par_vis);

    // }                                                                   
    //   break;

    //   int run_arr(uint8_t length, uint8_t* arr);

    //   case hash("run"):
    //  {
    //   uint8_t cycles = Serial_Input_Long(",", 100);
    //   ESP_LOGV(TAG1,"cycles:%d", cycles);


    //   uint8_t arr[cycles * 8] = {};
    //   long _tmp = 0;
    //   bool load_arr = false;
    //   uint16_t num = cycles * 8;
    //   for (uint16_t n = 0; n < num; n++){
    //     _tmp = Serial_Input_Long(",", 100);
    //     if ((_tmp >= 0) && (_tmp < 256)){
    //       arr[n] = (uint8_t) _tmp;
    //     }
    //     else{
    //       ESP_LOGV(TAG1,"ARR BAD", cycles);
    //       break;
    //     }
    //     load_arr = true;
    //   }
    //   if (load_arr){

    //     run_arr(cycles, arr);
    //   }
      
    // }                                                                   
    //   break;


       case hash("reboot"):
      {
        ESP.restart();
      }
      break;


      int wait_for_response_clear(const char* s, uint8_t slen, uint8_t timeout);
      void write32(uint32_t v);
      

      case hash("add"):
      {
        uint16_t l = Serial_Input_Long(",", 100);
        data.put(l);
      }
      break;

      case hash("pop"):
      {
        uint32_t l = 0;
        data.pop(&l);
        Serial.println(l);
      }
      break;

      case hash("len"):
      {
        
        Serial.println(data.get_length());
      }
      break;

        case hash("printall"):
      {
        data.print_all();
      }
      break;

      case hash("del"):
      {
        delete &data;
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