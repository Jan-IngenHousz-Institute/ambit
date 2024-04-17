#include <Arduino.h>
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "serial.h"
#include "do_command.h"
#include "src/wrench.h"

#include "config.h"

static const char* TAG = "INO";
ADPD6 adpd;
uint8_t CONNECTION_TYPE = 0;


void setup(){
    esp_timer_early_init();
    pinMode(1, OUTPUT);
    pinMode(10, OUTPUT);
    digitalWrite(1, LOW);
    digitalWrite(10, LOW);

    Serial.begin(115200);
    delay(500);
    Serial.println("BOOT");
    Serial.println(esp_timer_get_time());

    digitalWrite(1, HIGH);
    delay(1);
    digitalWrite(1, LOW);
    init_i2c_bus();
    init_spi_bus();
    adpd.begin();
    if (as7341.begin()) ESP_LOGV(TAG, "AS7341 Found");
    check_AS7341();
    mlx_init();
    CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;    
}


int do_esp_cmd();
int c = -1;
char choose[50];

void loop(){
    c = -1;

    
    for (;;) {
        c = Serial.available();
        if (c > 1){            // received something        
            break;        
        }
        delay(10);
    }
    
    c = Serial.peek();

    if (c > 127) {
        do_esp_cmd();
    }else{
        Serial_Input_Chars(choose, ":,", 500, sizeof(choose) - 1);
        do_command(choose);
    }


}