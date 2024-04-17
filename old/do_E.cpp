
#include <Arduino.h>
#define MAX_E_COMMAND_LEN 20


extern uint8_t pulsed_620_current, pulsed_720_current, dc_current;
extern uint8_t gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf;
extern uint8_t status_run_config_set;


int MPF(uint16_t mode, uint16_t current, uint16_t dc_current,uint8_t,uint8_t);
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int run_arr_type1(uint8_t length, uint8_t* arr, bool);



static constexpr unsigned hash(const char *string)
{
 return *string == 0 ? 17325 : *string + (*string * hash(string+1));
}

static const char* TAG = "DO_E";


static uint8_t get_cmd(const char* s, uint16_t* ptr, char *cmd){
    char n;
    uint8_t counter = 0;
    for (uint8_t i = 0; i < MAX_E_COMMAND_LEN; i++){
        n = s[i + *ptr];

        if ((n == ',') || (n == '\0')){
            cmd[counter] = '\0';
            if (i > 0){
                if (n == ','){
                    *ptr += i + 1;
                    return 1;
                }else{
                    *ptr += i;
                    return 2;
                }
            }
            return 0;
        }else if (s[i + *ptr] > 31){
            if ((s[i + *ptr] == 32) && (counter == 0)) continue;
            cmd[counter] = s[i + *ptr];
            counter += 1;
        }
    }
    return 0;
}
 
static uint8_t get_int_from_string(const char* s, uint16_t* ptr, int* number){
    char n;
    char integer[MAX_E_COMMAND_LEN];
    uint8_t counter = 0;

    for (uint8_t i = 0; i < MAX_E_COMMAND_LEN; i++){
        n = s[i + *ptr];

        if (((n > 47) && (n < 58)) || (n == 45)){
            integer[counter] = n;
            counter += 1;
        }else if ((n == ',')||(n == '\0')){
            if (counter == 0) return 0;
            integer[counter] = '\0';
            *number = atoi(integer);
            if (n == ','){
                *ptr += i + 1;                
                return 1;                
            }
            *ptr += i;
            return 2;           
        }else if (n < 32) return 0;
    }
    return 0;
}


static uint16_t get_int_array(const char* s, const uint16_t ptr, uint8_t* arr){
    uint16_t data_counter = 0;
    int ii = 0;
    uint16_t ptr_used = ptr;

    uint8_t ret = get_int_from_string(s, &ptr_used, &ii);
    while (ret > 0){
        if (arr != NULL) arr[data_counter] = ii;        
        data_counter += 1;
        if (ret == 2) break;
        ret = get_int_from_string(s, &ptr_used, &ii);
    }

    return data_counter;
}

void do_E(const char* c){
    char cmd[MAX_E_COMMAND_LEN];
    uint16_t ptr = 0;
    int ii;

    if (get_cmd(c, &ptr, cmd) == 0){
        ESP_LOGE(TAG, "Command cannot parse: %s", c);
        return;
    }
    ESP_LOGV(TAG, "Command is: %s", cmd);

    unsigned int val = hash(cmd);

    switch (val)
    {
        case hash("hello"):
            Serial.println("Ready");
            break;

        case hash("SC"):
        {
            uint8_t arr[3];
            uint8_t counter = get_int_array(c, ptr, arr);
            ESP_LOGV(TAG, "set currents cmd gets %d numbers:", counter);
            if (counter == 3){
                pulsed_620_current = arr[0];
                pulsed_720_current = arr[1];
                dc_current = arr[2];
            }else if (counter == 0){
                Serial.printf("Currents are: %d,%d,%d", pulsed_620_current,pulsed_720_current,dc_current);
            }else{
                ESP_LOGE(TAG, "set currents cmd gets %d numbers:", counter);
            }
        }
        break;

        case hash("SG"):
        {
            uint8_t arr[6];
            uint8_t counter = get_int_array(c, ptr, arr);
            ESP_LOGV(TAG, "set Gains cmd gets %d numbers:", counter);
            if (counter == 6){
                gain_fluor = arr[0];
                gain_fluref = arr[1];
                gain_720 = arr[2];
                gain_720ref = arr[3];
                gain_sun = arr[4];
                gain_leaf = arr[5];
            }else if (counter == 0){
                Serial.printf("Gains are: %d,%d,%d,%d,%d,%d", gain_fluor,gain_fluref,gain_720,gain_720ref,gain_sun,gain_leaf);
            }
            
            else
            {
                ESP_LOGE(TAG, "set Gains cmd gets %d numbers:", counter);
            }
        }
        break;

        case hash("config1"):
              if (status_run_config_set == 0){
                conf_slow_FR_1(pulsed_620_current, pulsed_720_current, dc_current, gain_fluor, gain_fluref, gain_sun, gain_leaf, gain_720, gain_720ref);
                status_run_config_set = 1;
                }            
        break;

        case hash("arr"):
            {
                uint8_t counter = get_int_array(c, ptr, NULL);
                ESP_LOGV(TAG, "arr cmd gets %d numbers:", counter);
                //expect: num lines, led_persist + N x 8 elements, checksum
                if ((counter - 3)%8 > 0){
                    ESP_LOGE(TAG, "arr not right");
                    break;
                }
                uint8_t line_counts = (counter - 3)/8;
                if (line_counts == 0){
                    ESP_LOGE(TAG, "arr is zero");
                    break;
                }

                uint8_t arr[counter] = {0};
                uint8_t checksum = 0;
                get_int_array(c, ptr, arr);

                if (line_counts != arr[0]){
                    ESP_LOGE(TAG, "arr length doesn't match");
                    break;
                }

                for (uint8_t i = 0; i < line_counts; i++){
                    for (uint8_t j = 0; j < 8; j++){
                        Serial.printf("%d,", arr[i * 8 + j + 2]);
                    }
                    Serial.println();
                }
                run_arr_type1(line_counts, arr + 2, arr[1]);
            }        
        break;


        
        default:
            Serial.printf("Bad command:%s", cmd);
    }





  




}