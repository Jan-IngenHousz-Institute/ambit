#include <cJSON.h>
#include <Arduino.h>

#include "config.h"
#include "PAM.h"





void run_json_array(cJSON *json){
    cJSON *item;    
    item = cJSON_GetObjectItem(json, "array");

    if ((item == NULL) || (!cJSON_IsArray(item))){
        Serial.println("'array' not found or is not an array in JSON");
        return;
    }

    int array_size = cJSON_GetArraySize(item);
    // Serial.print("Array size: ");
    if ((array_size%3 != 0) || (array_size == 0) || (array_size > 30)){
        Serial.println("Invalid array size");
        return;
    }
    cJSON *array_item;
    int arr[30] = {0};

    for (int i = 0; i < array_size; i++){
        array_item = cJSON_GetArrayItem(item, i);
        if (cJSON_IsNumber(array_item)){
            arr[i] = array_item->valueint;
        }else{
            Serial.print("Non-number item at index ");
            Serial.println(i);
            return;
        }
    }


    uint8_t run_arr[128] = {0};

    // Serial.println("Array contents:");

    uint16_t num, freq;
    uint8_t actinic;
    uint8_t idx = 0;

    //[2, 0, num//256, num%256, freq//256, freq%256, actinic, 1]
    for (int i = 0; i < array_size/3; i++){

        num = arr[i*3 + 0];
        freq = arr[i*3 + 1];
        actinic = (uint8_t) arr[i*3 + 2];

        if ((num > 2000) || (num == 0) || (freq < 1)){
            continue;
        }

        run_arr[idx*8 + 0] = (uint8_t) 2;
        run_arr[idx*8 + 1] = 0;
        run_arr[idx*8 + 2] = (uint8_t) (num >> 8);
        run_arr[idx*8 + 3] = (uint8_t) (num & 0xFF);
        run_arr[idx*8 + 4] = (uint8_t) (freq >> 8);
        run_arr[idx*8 + 5] = (uint8_t) (freq & 0xFF);
        run_arr[idx*8 + 6] = actinic;
        run_arr[idx*8 + 7] = 1;

        // Serial.printf("Run %d: Num=%d, Freq=%d, Actinic=%d\n", i+1, num, freq, actinic);
        idx += 1;

    }

    CONNECTION_TYPE = CONNECTION_TYPES::JSON;

    char* buf[50];

    if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
        conf_slow_FR_1();
        adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
      }

    serial_print_with_crc("{\"ambit_start\":");
    snprintf((char*)buf, 50, "%u,", millis());
    serial_print_with_crc((char*)buf);

    run_arr_type1(idx, run_arr, 0);


    serial_print_with_crc(",\"ambit_stopped\":");
    snprintf((char*)buf, 50, "%u}", millis());
    serial_print_with_crc((char*)buf);



    Serial_Print_CRC();


}


void read_json_from_serial(){
    char c;
    char json_buffer[1024] = {0};
    size_t idx = 0;

    bool full_json_received = false;

    c = Serial.read();
    if (c != '[') return;



    while ((Serial.available() > 0) && (idx < sizeof(json_buffer) - 1))
    {
        c = Serial.read();
        if (idx == 0 && c != '{') {
            // Invalid start of JSON object
            continue;
        }
        // Serial.write(c); // echo back
        json_buffer[idx++] = c;
        json_buffer[idx] = '\0';
        if (c == '}'){
            full_json_received = true;
            break;
        }

        if (Serial.available() == 0){
            delay(10); // wait for more data
        }
    }

    if (full_json_received){
        // Serial.println();
        // Serial.println("Full JSON received:");
        // Serial.println(json_buffer);

        cJSON *json = cJSON_Parse(json_buffer);
        if (json == NULL){
            Serial.println("Error parsing JSON");
            return;
        }

        cJSON *item = cJSON_GetObjectItem(json, "ambit");
        if (item != NULL){
            // Serial.print("Value of 'cmd': ");
            // Serial.println(item->valuestring);

        }else{
            Serial.println("'ambit' not found in JSON");
        }




        run_json_array(json);



        

        // Process JSON as needed
        cJSON_Delete(json);
    }else{
        Serial.println();
        Serial.println("Incomplete JSON received.");
    }

    while ((Serial.available() > 0)){
        Serial.read(); // clear buffer
    }


    
}