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
#include "driver/gpio.h"

static const char* TAG = "INO";
ADPD6 adpd;
uint8_t CONNECTION_TYPE = 0;
static uint16_t sleep_threshod_ms = 100;


 
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);
uint16_t flush_serial(uint8_t timeout);
void setup(){
    esp_timer_early_init();
    pinMode(STF_FLASH_PIN, OUTPUT);
    pinMode(BOOT_PIN, OUTPUT);
    digitalWrite(STF_FLASH_PIN, LOW);
    digitalWrite(BOOT_PIN, LOW);

    Serial.begin(115200);
    delay(500);
    Serial.println("BOOT");
    Serial.println(esp_timer_get_time());
    Serial.setTimeout(50);


    digitalWrite(STF_FLASH_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STF_FLASH_PIN, LOW);
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

    Serial.printf("Compiled at %s-%s\n",__DATE__, __TIME__);
    Serial.write(AMBIT_BOOT_IDLE);

    esp_sleep_enable_timer_wakeup(10000000);
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
            if (millis() - sleep_timer > sleep_threshod_ms){
                //ESP_LOGE(TAG, "ambit sleep");
                Serial.flush();
                flush_serial(20);
                esp_light_sleep_start();
                c = esp_sleep_get_wakeup_cause();
                sleep_timer = millis();
                if (c == 8){
                    sleep_threshod_ms = 1000;
                }else{
                    sleep_threshod_ms = 200;
                }               
                
                Serial.write(AMBIT_BOOT_IDLE);
                Serial.flush();
                //Serial.read();
            }else{
                delay(10);
                //sleep_threshod_ms = 200;
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
                sleep_threshod_ms = 500;
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
        sleep_threshod_ms = 30000;
        Serial_Input_Chars(choose, ":,", 200, sizeof(choose) - 1);
        do_command(choose);
        
    }


}