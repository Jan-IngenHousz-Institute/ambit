#include "data_transfer.h"


static const char* TAG = "DATA_TRANS";

DataPtk::DataPtk(void){

}

DataPtk::~DataPtk(void){
    for (uint8_t i = 0; i < NUM_ALL_ARR; i++){
        if (DataPtk::arr[i] != NULL){
            free(DataPtk::arr[i]);
            ESP_LOGV(TAG, "memory arr[%d] freed.", i);
        }
    }
}

int DataPtk::get_arr_len(){
    const uint16_t timeout = 50;
    unsigned long start_t = millis();
    uint8_t b, counter = 0;
    uint8_t arr[4];

    while (((millis() - start_t)  < timeout) && (counter < 10)){
        if (Serial.available() > (NUM_ALL_ARR * 4 - 1)){
            b = Serial.peek();
            if (b == ID_ARR_LENGTH) break;
            counter += 1;
        }
        else{
            delay(1);
        }
    }

    if (b == ID_ARR_LENGTH){
        for (uint8_t i = 0; i < NUM_ALL_ARR; i++){
            Serial.readBytes(arr, 4);
            if (arr[0] == ID_ARR_LENGTH){
                DataPtk::arr_length[i] = arr[2] * 256 + arr[3];
            }
            else{
                ESP_LOGE(TAG, "GET Array length id unmatched");
                return -1;
            }
        }
        ESP_LOGV(TAG, "GET Array length %d, %d, %d, %d, %d, %d", DataPtk::arr_length[0], DataPtk::arr_length[1], DataPtk::arr_length[2], DataPtk::arr_length[3], DataPtk::arr_length[4], DataPtk::arr_length[5]);
        return 0;        
    }

    ESP_LOGE(TAG, "TIMEOUT before getting ID");
    return -1;
}

int DataPtk::allocate(uint8_t id){
    if ((DataPtk::arr[id]) != NULL){
        free(DataPtk::arr[id]);
        ESP_LOGV(TAG, "output array %d freed", id);
    }
    if (DataPtk::arr_length > 0){
        ESP_LOGV(TAG, "Allocation %d bytes", (DataPtk::arr_length[id]) * 4);        
       (DataPtk::arr[id]) = (uint32_t*) heap_caps_calloc((DataPtk::arr_length[id]), sizeof(uint32_t), MALLOC_CAP_32BIT);
       if ((DataPtk::arr[id]) == NULL){
            ESP_LOGE(TAG, "Allocation failed");
            return -1;
        }
    }
    return 0;
}



int DataPtk::get_data_arr(uint8_t id){
    if (DataPtk::arr_length[id] == 0){
        ESP_LOGE(TAG, "array expected length is 0");
        return -1;
    }

    if (DataPtk::arr[id] == NULL){
        DataPtk::allocate(id);
    }
    
    const uint16_t timeout = 50;
    unsigned long start_t = millis();
    uint8_t b, counter = 0;
    uint8_t arr[4];



    while (((millis() - start_t)  < timeout)){
        if (Serial.available() > 4){
            b = Serial.peek();
            if (b == ID_ARR_DATA) break;
        }
    }



    ESP_LOGE(TAG, "TIMEOUT before getting ID");
    return -1;
}