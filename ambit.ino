#include <Arduino.h>
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "serial.h"
#include "do_command.h"
static const char* TAG = "INO";


ADPD6 adpd;

void setup(){
    Serial.begin(115200);
    delay(2500);
    Serial.println("BOOT");

    init_i2c_bus();
    init_spi_bus();
    adpd.begin();
    as7341.begin();
    mlx_init();
}




void loop(){

    int c = -1;
    for (;;) {
        c = Serial.available();
        if (c > 1){            // received something
            break;        
        }
        delay(10);
    }  
    char choose[50]; //buffer to hold commands    
    Serial_Input_Chars(choose, ",", 500, sizeof(choose) - 1);
    do_command(choose);




}