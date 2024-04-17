#include <Arduino.h>
#include "data_utils.h"

#define TAG "ESP"

static uint8_t pulsed_620_current, pulsed_720_current, ir_lumination_current;
static uint8_t gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf;
extern uint8_t status_run_config_set;


extern uint8_t CONNECTION_TYPE;
int MPF(uint16_t mode, uint16_t current, uint16_t dc_current,uint8_t,uint8_t);
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int run_arr_type1(uint8_t length, uint8_t* arr, bool);


#define ESP_CMD_HEADER 160
#define ESP_CMD_DONE 161

int do_esp_cmd(){
    uint8_t target, c;
    uint8_t cmd_arr[8];
    ESP_LOGV(TAG, "parse ESP commands");
    Serial.setTimeout(50);

    //search for cmd header
    target = ESP_CMD_HEADER;
    if (!Serial.find(&target, 1)){
        ESP_LOGE(TAG, "ESP cmd header failed");
        return -1;
    }

    CONNECTION_TYPE = CONNECTION_TYPES::AMBYTE;

    // the real cmd will follows
    c = Serial.readBytes(cmd_arr, 8);
    if (c < 8){
        ESP_LOGE(TAG, "ESP cmd parse failed");
        return -1;        
    }
    ESP_LOGV(TAG, "cmd is %d, %d, %d, %d, %d, %d, %d, %d", cmd_arr[0], cmd_arr[1], cmd_arr[2], cmd_arr[3], cmd_arr[4], cmd_arr[5], cmd_arr[6], cmd_arr[7]);

    switch (cmd_arr[0])
    {
    case 1:  // set PD gains
        if ((cmd_arr[1] > 0) && (cmd_arr[1] < 7)) gain_fluor = cmd_arr[1] - 1;
        if ((cmd_arr[2] > 0) && (cmd_arr[2] < 7)) gain_fluref = cmd_arr[2] - 1;
        if ((cmd_arr[3] > 0) && (cmd_arr[3] < 7)) gain_720 = cmd_arr[3] - 1;
        if ((cmd_arr[4] > 0) && (cmd_arr[4] < 7)) gain_720ref = cmd_arr[4] - 1;
        if ((cmd_arr[5] > 0) && (cmd_arr[5] < 7)) gain_sun = cmd_arr[5] - 1;
        if ((cmd_arr[6] > 0) && (cmd_arr[6] < 7)) gain_leaf = cmd_arr[6] - 1; 
        ESP_LOGV(TAG, "gains are %d, %d, %d, %d, %d, %d", gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf);
        Serial.write(ESP_CMD_DONE);
        break;

    case 2:  // set currents
        if ((cmd_arr[1] < 127)) pulsed_620_current = cmd_arr[1];
        if ((cmd_arr[2] < 127)) pulsed_720_current = cmd_arr[2];
        if ((cmd_arr[3] < 127)) ir_lumination_current = cmd_arr[3];
        ESP_LOGV(TAG, "currents are %d, %d, %d", pulsed_620_current, pulsed_720_current, ir_lumination_current);
        Serial.write(ESP_CMD_DONE);

        break;

    
    
    case 10: // array run config
        conf_slow_FR_1(pulsed_620_current, pulsed_720_current, ir_lumination_current, gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf);
        status_run_config_set = 1;
        Serial.write(ESP_CMD_DONE);
        break;

    case 20: // run mpf
        MPF(cmd_arr[1], pulsed_620_current, cmd_arr[2], gain_fluor, gain_fluref);
        status_run_config_set = 0;
        Serial.write(ESP_CMD_DONE);
        break;

    case 21:// run
    {   
        uint8_t arr_length = cmd_arr[1];
        uint8_t led_persist = cmd_arr[2];
        uint8_t cc = 0;
        if ((arr_length == 0) || (arr_length > 7)){
            ESP_LOGE(TAG, "run array wrong length: %d", arr_length);
            break;
        }
        uint8_t run_arr[arr_length * 8];
        cc = Serial.readBytes(run_arr, arr_length * 8);
        if (cc != (arr_length * 8)){
            ESP_LOGE(TAG, "run array elements count %d not match config %d", cc, arr_length);
            break;
        }
        run_arr_type1(arr_length, run_arr, led_persist);
        Serial.write(ESP_CMD_DONE);
    }
    break;



    default:
        break;
    }









    return 0;
}

