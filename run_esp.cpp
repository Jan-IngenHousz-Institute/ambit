#include <Arduino.h>
#include "data_utils.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "PAM.h"
#include <Preferences.h>

#define TAG "ESP"
#define ESP_CMD_HEADER 160
#define ESP_CMD_DONE 161
#define ESP_CMD_END 240
#define ESP_WAKE_FOR_CMD 170
#define MAX_ARR_LEN 16


// keep a local copy of settings for run-time change
static adpd_current_config_t adpd_current_config_local;
static adpd_gains_config_t adpd_gains_config_local;
extern Preferences preferences;

extern uint8_t CONNECTION_TYPE;
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);

extern char ambit_name[];
extern float actinic_offset;
struct ambit_info_t{
    bool loaded = false; 
    int32_t mlx_coef[14] = {0};
    float_t par_offset = 1.0;
    char ambit_name[20] = "ambit";
}ambit_info;




int do_esp_cmd(){
    uint8_t cmd_arr[8], c, ret;
    ESP_LOGV(TAG, "parse ESP commands");

    //search for cmd header
    ret = serial_read_until(ESP_CMD_HEADER, 0, 0, 100, true);
    if (ret != 1){
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
        if ((cmd_arr[1] > 0) && (cmd_arr[1] < 7)) adpd_gains_config_local.Fluo = cmd_arr[1] - 1;
        if ((cmd_arr[2] > 0) && (cmd_arr[2] < 7)) adpd_gains_config_local.FluoRef = cmd_arr[2] - 1;
        if ((cmd_arr[3] > 0) && (cmd_arr[3] < 7)) adpd_gains_config_local.IR = cmd_arr[3] - 1;
        if ((cmd_arr[4] > 0) && (cmd_arr[4] < 7)) adpd_gains_config_local.IRRef = cmd_arr[4] - 1;
        if ((cmd_arr[5] > 0) && (cmd_arr[5] < 7)) adpd_gains_config_local.Sun = cmd_arr[5] - 1;
        if ((cmd_arr[6] > 0) && (cmd_arr[6] < 7)) adpd_gains_config_local.Leaf = cmd_arr[6] - 1;
        adpd_gains_config_local.init = true;
        ESP_LOGV(TAG, "gains are %d, %d, %d, %d, %d, %d", adpd_gains_config_local.Fluo, adpd_gains_config_local.FluoRef, adpd_gains_config_local.IR, adpd_gains_config_local.IRRef, adpd_gains_config_local.Sun, adpd_gains_config_local.Leaf);
        Serial.write(ESP_CMD_DONE);
        break;

    case 2:  // set currents
        if ((cmd_arr[1] < 127)) adpd_current_config_local.I620 = cmd_arr[1];
        if ((cmd_arr[2] < 127)) adpd_current_config_local.I720 = cmd_arr[2];
        if ((cmd_arr[3] < 127)) adpd_current_config_local.IR = cmd_arr[3];
        adpd_current_config_local.init = true;
        ESP_LOGV(TAG, "currents are %d, %d, %d", adpd_current_config_local.I620, adpd_current_config_local.I720, adpd_current_config_local.IR);
        Serial.write(ESP_CMD_DONE);
        break;    
    
    case 10: // array run config
        if (adpd_gains_config_local.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
        if (adpd_current_config_local.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");
        conf_slow_FR_1(adpd_current_config_local.I620, adpd_current_config_local.I720, adpd_current_config_local.IR, 
            adpd_gains_config_local.Fluo, adpd_gains_config_local.FluoRef, adpd_gains_config_local.Sun, adpd_gains_config_local.Leaf, adpd_gains_config_local.IR, adpd_gains_config_local.IRRef);
        adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
        Serial.write(ESP_CMD_DONE);
        break;

    case 20: // run mpf
        if (adpd_gains_config_local.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
        if (adpd_current_config_local.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");
        Serial.write(ESP_CMD_DONE);
        MPF(cmd_arr[1], adpd_current_config_local.I620, cmd_arr[2], adpd_gains_config_local.Fluo, adpd_gains_config_local.FluoRef);
        adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;
        Serial.write(ESP_CMD_END);
        
        break;

    case 21:// run
    {   
        uint8_t arr_length = cmd_arr[1];
        uint8_t led_persist = cmd_arr[2];
        bool allow_interrupt = (bool) cmd_arr[3];

        uint8_t cc = 0;
        if ((arr_length == 0) || (arr_length > MAX_ARR_LEN)){
            ESP_LOGE(TAG, "run array wrong length: %d", arr_length);
            break;
        }
        uint8_t run_arr[arr_length * 8];
        if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
            if (adpd_gains_config_local.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
            if (adpd_current_config_local.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");
            conf_slow_FR_1(adpd_current_config_local.I620, adpd_current_config_local.I720, adpd_current_config_local.IR, 
                adpd_gains_config_local.Fluo, adpd_gains_config_local.FluoRef, adpd_gains_config_local.Sun, adpd_gains_config_local.Leaf, adpd_gains_config_local.IR, adpd_gains_config_local.IRRef);
            adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
        }

        cc = Serial.readBytes(run_arr, arr_length * 8);
        if (cc != (arr_length * 8)){
            ESP_LOGE(TAG, "run array elements count %d not match config %d", cc, arr_length);
            break;
        }
        Serial.write(ESP_CMD_DONE);
        run_arr_type1(arr_length, run_arr, led_persist, allow_interrupt);
        Serial.write(ESP_CMD_END);
        
    }
    break;

    case 31: // get spec
    {
        uint16_t spec[12] = {0};
        uint16_t par10x = (uint16_t) (get_PAR(spec) * 10);
        spec[10] = par10x;
        Serial.write(ESP_CMD_DONE);
        Serial.write((uint8_t*) spec, 24);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 32: // get temp
    {
        double leaf, chip;
        mlx_measure(&leaf, &chip);
        Serial.write(ESP_CMD_DONE);
        int16_t t1 = (int16_t) (leaf * 10);
        int16_t t2 = (int16_t) (chip * 10);
        Serial.write((uint8_t*) (&t1), 2);
        Serial.write((uint8_t*) (&t2), 2);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 33:    // retrieve ambit info
    {
        ambit_info_t* abi = &ambit_info;        
        mlx_read_coe(abi->mlx_coef);
        abi->par_offset = actinic_offset;
        abi->loaded = true;
        strcpy(abi->ambit_name, ambit_name);        

        Serial.write(ESP_CMD_DONE);
        Serial.write((uint8_t*) abi, sizeof(ambit_info_t));
        Serial.write(ESP_CMD_END);
    }
    break;

    
    case 34: // get temp and raw
    {
        double leaf, leaf_1, chip;
        int16_t a1, a2, a3, a4;
        mlx_measure(&leaf, &chip, &leaf_1, &a1, &a2, &a3, &a4);
        Serial.write(ESP_CMD_DONE);
        int16_t t1 = (int16_t) (leaf * 10);
        int16_t t2 = (int16_t) (leaf_1 * 10);
        int16_t t3 = (int16_t) (chip * 10);
        Serial.write((uint8_t*) (&t1), 2);
        Serial.write((uint8_t*) (&t2), 2);
        Serial.write((uint8_t*) (&t3), 2);
        Serial.write((uint8_t*) (&a1), 2);
        Serial.write((uint8_t*) (&a2), 2);
        Serial.write((uint8_t*) (&a3), 2);
        Serial.write((uint8_t*) (&a4), 2);
        Serial.write(ESP_CMD_END);
    }
    break;


    case 4: // try/set actinic
    {
        AS_LED_OFF();
        Serial.write(ESP_CMD_DONE);
        uint8_t type = cmd_arr[1];
        uint8_t var = cmd_arr[2];
        if (type == 1){ // try actinics            
            AS_LED_Current(50);
            AS_LED_ON();
            delay(3000);
            AS_LED_Current(var);
            delay(2000);
            AS_LED_OFF();
            AS_LED_Current(0);
        }else if (type == 2){ // set actinic offset
            actinic_offset = ((float) cmd_arr[2]) / 128.0;
            if ((actinic_offset > 0.0) && (actinic_offset < 1.99)){
                preferences.begin("config", false);
                preferences.putFloat("actinic", actinic_offset);
                preferences.end();
            }else{
                actinic_offset = 1.0;
            }
        }

        Serial.write(ESP_CMD_END);
    }
    break;


    default:
        ESP_LOGE(TAG, "Bad command");
        break;
    }

    return 0;
}

