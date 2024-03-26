
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

dataclass data;




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

  // process single commands
  switch (val) {
    case hash("C"):{

      char c[500];
      Serial_Input_Chars(c, "?", 10000, 500);
      do_c(c);
      Serial.println();
    }
    break;


    case hash("hello"):
     {
      Serial.print("NEW Name Here");
      Serial.println(" Ready");

    }                                                                   
      break;

    //   case hash("adpd"):
    //  {
    //   adpd.begin();
    // }                                                                   
    //   break;

      int detector_preset_1(uint8_t current, uint8_t gain_fluo, uint8_t gain_ref, uint8_t gain_par_ir, uint8_t gain_par_vis);

      case hash("config"):
     {
      uint8_t current = Serial_Input_Long(",", 100);
      uint8_t gain_fluo = Serial_Input_Long(",", 100);
      uint8_t gain_ref = Serial_Input_Long(",", 100);
      uint8_t gain_par_ir = Serial_Input_Long(",", 100);
      uint8_t gain_par_vis = Serial_Input_Long(",", 100);
      detector_preset_1(current, gain_fluo, gain_ref, gain_par_ir, gain_par_vis);

    }                                                                   
      break;

      int run_arr(uint8_t length, uint8_t* arr);

      case hash("run"):
     {
      uint8_t cycles = Serial_Input_Long(",", 100);
      ESP_LOGV(TAG1,"cycles:%d", cycles);


      uint8_t arr[cycles * 8] = {};
      long _tmp = 0;
      bool load_arr = false;
      uint16_t num = cycles * 8;
      for (uint16_t n = 0; n < num; n++){
        _tmp = Serial_Input_Long(",", 100);
        if ((_tmp >= 0) && (_tmp < 256)){
          arr[n] = (uint8_t) _tmp;
        }
        else{
          ESP_LOGV(TAG1,"ARR BAD", cycles);
          break;
        }
        load_arr = true;
      }
      if (load_arr){

        run_arr(cycles, arr);
        // for (uint16_t n = 0; n < num; n++){
        //   Serial.print(arr[n]);
        //   if (n % 8 == 7){
        //     Serial.println();
        //   }
        //   else{
        //     Serial.print(",");
        //   }
        // }
      }
      
    }                                                                   
      break;


       case hash("reboot"):
      {
        ESP.restart();
      }
      break;


      int wait_for_response_clear(const char* s, uint8_t slen, uint8_t timeout);
      void write32(uint32_t v);

      case hash("init"):
      {
       Serial.println("Wake!");
       if (wait_for_response_clear("Ready", 5, 10) == 0){
        Serial.println("2");
        if (wait_for_response_clear("GO", 2, 10) == 0){
          write32((uint32_t) 1234);
          write32((uint32_t) 6543);
          Serial.println("DONE");
        }
        
       }
      }
      
      break;

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

    default:
      Serial.println("BAD COMMAND");
    break;

  }
}