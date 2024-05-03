#include <Arduino.h>
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "serial.h"
#include "do_command.h"
#include "src/wrench.h"
#include "config.h"
#include "driver/uart.h"

static const char* TAG = "INO";
ADPD6 adpd;
uint8_t CONNECTION_TYPE = 0;
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);
uint16_t flush_serial(uint8_t timeout);
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
    Serial.setTimeout(50);


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

    uart_set_wakeup_threshold(UART_NUM_0, 3);
    esp_sleep_enable_uart_wakeup(UART_NUM_0);

    gpio_sleep_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_20, GPIO_PULLUP_ONLY);

    gpio_sleep_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_1, GPIO_PULLDOWN_ONLY);
    Serial.write(133);

    //esp_sleep_enable_timer_wakeup(1000000);
}


int do_esp_cmd();
int c = -1;
char choose[50];

void loop(){
    c = -1;
    int b;
    unsigned int sleep_timer = millis();

    
    for (;;) {
        c = Serial.available();
        if (c > 0){
            sleep_timer = millis();
            c = Serial.peek();
            if (c == 255) Serial.read();
            if (c < 255) break;
        }else{
            if (millis() - sleep_timer > 100){
                ESP_LOGV(TAG, "ambit sleep");
                Serial.flush();
                esp_sleep_enable_timer_wakeup(10000000);
                esp_light_sleep_start();
                sleep_timer = millis();
                Serial.write(133);
                Serial.flush();
            }else{
                delay(10);
            }
        }

    }
    
    c = Serial.peek();
    if (c > 127) { // not from computer
        while (Serial.available() > 0){
            b = serial_read_until(170, 160, 222, 50, false);
            if (b == 1){ // wake up signal
                flush_serial(5);
                Serial.write(128);
            }else if(b == 2){// command
                do_esp_cmd();
                break;
            }else if (b == 3){ // data send reset signal
                ESP_LOGE(TAG, "OUT of sync");
                Serial.read();
                Serial.write(128);
            }else{
                ESP_LOGE(TAG, "Unknown cmd %d", c);
                Serial.read();
                Serial.write(128);
                break;
            }
        }
        
    }else{
        Serial_Input_Chars(choose, ":,", 500, sizeof(choose) - 1);
        do_command(choose);
    }


}