#include <cJSON.h>
#include <Arduino.h>


void read_json_from_serial(){
    char c;
    char json_buffer[1024] = {0};
    size_t idx = 0;

    bool full_json_received = false;



    while ((Serial.available() > 0) && (idx < sizeof(json_buffer) - 1))
    {
        c = Serial.read();
        Serial.write(c); // echo back
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
        Serial.println();
        Serial.println("Full JSON received:");
        Serial.println(json_buffer);

        cJSON *json = cJSON_Parse(json_buffer);
        if (json == NULL){
            Serial.println("Error parsing JSON");
            return;
        }

        cJSON *item = cJSON_GetObjectItem(json, "cmd");
        if (item != NULL){
            Serial.print("Value of 'cmd': ");
            Serial.println(item->valuestring);
        }else{
            Serial.println("'cmd' not found in JSON");
        }

        item = cJSON_GetObjectItem(json, "array");
        if (item != NULL && cJSON_IsArray(item)){
            Serial.println("Values in 'array':");
            int array_size = cJSON_GetArraySize(item);
            for (int i = 0; i < array_size; i++){
                cJSON *array_item = cJSON_GetArrayItem(item, i);
                if (cJSON_IsNumber(array_item)){
                    Serial.println(array_item->valueint);
                }else if (cJSON_IsArray(array_item)){
                    Serial.print("Sub-array at index ");
                    Serial.print(i);
                    Serial.println(":");
                    int sub_array_size = cJSON_GetArraySize(array_item);
                    for (int j = 0; j < sub_array_size; j++){
                        cJSON *sub_array_item = cJSON_GetArrayItem(array_item, j);
                        if (cJSON_IsNumber(sub_array_item)){
                            Serial.println(sub_array_item->valueint);
                        }else{
                            Serial.println("  Non-number item in sub-array");
                        }
                    }
                }
            }
        }else{
            Serial.println("'array' not found or is not an array in JSON");
        }




        // Process JSON as needed
        cJSON_Delete(json);
    }else{
        Serial.println();
        Serial.println("Incomplete JSON received.");
    }
}