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
float_t actinic_coef = 1.0;
float_t spec_coef = 1.0;



extern double mlx_emissivity;

struct ambit_info_t{
    bool loaded = false; 
    int32_t mlx_coef[14] = {0};
    float_t actinic_coef = 0.1;
    float_t spec_coef = 1.0;
    float_t mlx_emissivity = 1.0;
    char ambit_name[20] = "ambit";
}ambit_info;



struct ambit_FW_info_t{
    uint64_t MAC = 0;
    uint32_t Size = 0;
    char FW_date[12];
    char reserved[12];
    uint32_t num = 0;   
}ambit_FW_info;


struct metadata_t {
    double lon = 1.0;
    double lat = 1.0;
    float alt = 1.0; 
    uint32_t time = 1.0;
    float acc = 1.0;
    float vacc = 1.0;
    char info1[200] = "NA";
    char info2[240] = "NA";
    uint8_t checksum = 0;
}metadata_epprom, metadata_incoming;


static void _get_FW_info(void){
    ambit_FW_info.MAC = ESP.getEfuseMac();
    ambit_FW_info.Size = ESP.getSketchSize();
    strncpy(ambit_FW_info.FW_date, __DATE__, 12);
    return;
}

static void _load_metadata(void){
    preferences.begin("metadata", true);
    if (preferences.isKey("lon")) metadata_epprom.lon = preferences.getDouble("lon", 1.0);
    if (preferences.isKey("lat")) metadata_epprom.lat = preferences.getDouble("lat", 1.0);
    if (preferences.isKey("alt")) metadata_epprom.alt = preferences.getFloat("alt", 1.0);
    if (preferences.isKey("time")) metadata_epprom.time = (uint32_t)preferences.getInt("time", 1);
    if (preferences.isKey("acc")) metadata_epprom.acc = preferences.getFloat("acc", 1.0);
    if (preferences.isKey("vacc")) metadata_epprom.vacc = preferences.getFloat("vacc", 1.0);
    if (preferences.isKey("info1")) preferences.getString("info1", metadata_epprom.info1, 200);
    if (preferences.isKey("info2")) preferences.getString("info2", metadata_epprom.info2, 240);
    preferences.end();
    return;
}

static void _save_metadata(void){
    preferences.begin("metadata", false);
    preferences.putDouble("lon", metadata_incoming.lon);
    preferences.putDouble("lat", metadata_incoming.lat);
    preferences.putFloat("alt", metadata_incoming.alt);
    preferences.putInt("time", metadata_incoming.time);
    preferences.putFloat("acc", metadata_incoming.acc);
    preferences.putFloat("vacc", metadata_incoming.vacc);
    preferences.putString("info1", metadata_incoming.info1);
    preferences.putString("info2", metadata_incoming.info2);
    preferences.end();

    return;
}



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
    //Serial.printf("cmd is %d, %d, %d, %d, %d, %d, %d, %d\n", cmd_arr[0], cmd_arr[1], cmd_arr[2], cmd_arr[3], cmd_arr[4], cmd_arr[5], cmd_arr[6], cmd_arr[7]);

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
    {
        if (adpd_gains_config_local.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
        if (adpd_current_config_local.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");

        uint16_t length = (((uint16_t) cmd_arr[1]) << 7) + cmd_arr[2];
        uint8_t interval = cmd_arr[3];
        bool change_act = (bool) cmd_arr[4];
        uint8_t act = cmd_arr[5];

        //Serial.printf("%d, %d, %d, %d\n", length, interval, change_act, act);
        
        Serial.write(ESP_CMD_DONE);
        run_trigger_spacer(length, interval, change_act, act, true);
        Serial.write(ESP_CMD_END);
    }    
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
        float par = get_PAR(spec);
        memcpy(spec + 10, &par, 4);
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
        abi->actinic_coef = actinic_coef;
        abi->spec_coef = spec_coef;
        abi->mlx_emissivity = mlx_emissivity;
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

    case 35:    // retrieve firmware info
    {
        if (ambit_FW_info.Size == 0){
            _get_FW_info();
        }
        Serial.write(ESP_CMD_DONE);
        Serial.write((uint8_t*) (&ambit_FW_info), sizeof(ambit_FW_info_t));
        Serial.write(ESP_CMD_END);
    }
    break;

    case 36:    // retrieve metadata
    {
        _load_metadata();        
        Serial.write(ESP_CMD_DONE);
        Serial.write((uint8_t*) (&metadata_epprom), sizeof(metadata_t));
        Serial.write(ESP_CMD_END);
    }
    break;

    case 37:    // set metadata
    {
        
        Serial.write(ESP_CMD_DONE);
        Serial.readBytes((uint8_t*) (&metadata_incoming), sizeof(metadata_t));
        Serial.write(ESP_CMD_END);
        _save_metadata();
    }
    break;



    case 4: // try/set actinic
    {
        AS_LED_OFF();
        Serial.write(ESP_CMD_DONE);
        uint8_t type = cmd_arr[1];
        uint8_t var = cmd_arr[2];
        float_t _factor = 1.0;
        if (type == 1){ // try actinics
            AS_LED_Current(50);
            AS_LED_ON();
            delay(3000);
            AS_LED_Current(var);
            delay(3000);
            AS_LED_OFF();            
            AS_LED_Current(0);
        }else if (type == 2){ // set actinic offset
            _factor = *((float *) &(cmd_arr[3]));
            if ((_factor > 0.01) && (_factor < 1.01)){
                preferences.begin("config", false);
                preferences.putFloat("actinic", _factor);
                preferences.end();
                actinic_coef = _factor;
            }
        }else if (type == 4){ // set actinic offset
            _factor = *((float *) &(cmd_arr[3]));
            if ((_factor > 0.05) && (_factor < 100.01)){
                //preferences.begin("config", false);
                //preferences.putFloat("spec", _factor);
                //preferences.end();
                spec_coef = _factor;
            }
        }
        Serial.write(ESP_CMD_END);
    }
    break;

    
    case 5:
    {
        uint8_t ambit_id = cmd_arr[1];
        uint8_t intensity = cmd_arr[2];
        Serial.write(ESP_CMD_DONE);
        if ((ambit_id < 4) && (intensity > 4) && (intensity < 254)) as7431_blink(ambit_id, intensity);
        Serial.write(ESP_CMD_END);
    }
    break;



    default:
        ESP_LOGE(TAG, "Bad command");
        break;
    }

    return 0;
}

