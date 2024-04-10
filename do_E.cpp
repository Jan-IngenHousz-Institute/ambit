
#include <Arduino.h>
#define MAX_E_COMMAND_LEN 20

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
 
static uint8_t get_int(const char* s, uint16_t* ptr, int* number){
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

        case hash("arr"):{
            int len = 0;
            if (get_int(c, &ptr, &ii) > 0) len = ii;
            if ((len == 0)||(len > 7)){
                ESP_LOGE(TAG, "Bad length %d", len);
                return;
            }

            
            uint8_t arr[8 * len];
            for (uint8_t i = 0; i < 8 * len; i++){
                if (get_int(c, &ptr, &ii) > 0){
                    arr[i] = (uint8_t)ii;
                }else{
                    ESP_LOGE(TAG, "No integer found in %s after %d", c, i);
                    return;
                }                
            }

            for (uint8_t i = 0; i < 8 * len; i++){
                Serial.printf("%d,", arr[i]);
            }
        }
            
        
        break;


        
        default:
            Serial.printf("Bad command:%s", cmd);
    }





  




}