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



int DataPtk::_get_serial_arr(const uint8_t id, const uint8_t len, uint8_t* out){
    unsigned long start_t = millis();
    const uint8_t c_max = len + 10;
    uint8_t counter, b = 0;

    while (((millis() - start_t)  < this->timeout) && (counter < c_max)){
        if (Serial.available() > (len - 1)){
            b = Serial.peek();
            if (b == id){
                Serial.readBytes(out, len);
                //ESP_LOGV(TAG, "Got %d extra bytes with id %d in %d ms", counter, id, millis() - start_t);
                return 0;
            }
            else{
                b = Serial.read();
                counter += 1;
            }
        }
        else{
            delay(1);
        }
    }
    ESP_LOGE(TAG, "Get serial timeout without id, but got %d bytes", counter);
    return -1;
}


// wait for array type and data length

int DataPtk::get_arr_len(){
    const uint8_t pkg_size = 6;//id, #arr, size1, size2, res1, checksum

    uint8_t arr[pkg_size] = {3, 4, 2, 3, 5, 1};//something rnd

    unsigned long start_t = millis();
    uint8_t arr_num, counter = 0;
    uint16_t arr_length = 0;
       

    this->_flush_serial();
    Serial.write(RDY_LENGTH);
    if (this->_get_serial_arr(ID_ARR_LENGTH, pkg_size, arr) == -1) return -1;
    arr_num = arr[1];
    arr_length = arr[2] * 256 + arr[3];
    this->arr_length[arr_num] = arr_length;
    if (this->allocate(arr_num) == -1) return -1;
    this->active_arr = arr_num;

    return 0;
}

int DataPtk::allocate(uint8_t id){
    if ((this->arr[id]) != NULL){
        free(this->arr[id]);
        ESP_LOGV(TAG, "output array %d freed", id);
    }

    if (this->arr_length > 0){
        ESP_LOGV(TAG, "Allocation %d bytes", (this->arr_length[id]) * 4);        
        (this->arr[id]) = (uint32_t*) heap_caps_calloc((this->arr_length[id]), sizeof(uint32_t), MALLOC_CAP_32BIT);
        if ((this->arr[id]) == NULL){
            ESP_LOGE(TAG, "Allocation failed");
            return -1;
        }
    }
    return 0;
}



int DataPtk::get_data_arr(uint8_t id){


    if (this->arr_length[id] == 0){
        ESP_LOGE(TAG, "array expected length is 0");
        return -1;
    }

    if (this->arr[id] == NULL){
        ESP_LOGE(TAG, "Array not initialized");
    }
    
    unsigned long start_t,start_t1 = 0;
    uint8_t b = 0;
    uint8_t arr[4] = {4, 3, 2, 1};
    uint16_t counter = 0;

    // Send ready for data
    this->_flush_serial();
    Serial.write(RDY_DATA);
    // Get data header
    if (this->_get_serial_arr(ID_ARR_DATA, 4, arr) == -1) return -1;
      
  

    // get data
    counter = 0;
    start_t1 = millis();
    for (uint16_t n = 0; n < this->arr_length[id]; n++){
        start_t = millis();
        //------ get a data point and save to arr
        b = 0;
        while (((millis() - start_t) < 5)){
            if (Serial.available() > 3){
                Serial.read(arr, 4);
                this->arr[id][n] = arr[0] * 0 + arr[1] * 65536 + arr[2] * 256 + arr[3];
                b = 1;
                counter += 1;
                break;
            }else{
                delayMicroseconds(1);
            }
        }
        if (b == 0){
            ESP_LOGE(TAG, "Data point 5ms timeout, got %d pts", counter);
            break;
        }
        //-----
    }
    ESP_LOGV(TAG, "Transfered %d data in %d ms", counter, millis() - start_t1);
    ESP_LOGV(TAG, "flushed extra %d", DataPtk::_flush_serial());
    if (counter == DataPtk::arr_length[id]) return 0;
    


    
    return -1;
}














void DataPtk::_print_all(){
    for (uint8_t i = 0; i < NUM_ALL_ARR; i++){
        if (this->arr[i] == NULL){
            Serial.printf("Arr %d not init\n", i);
            continue;;
        }
        Serial.printf("data arr %d:  ", i);
        for (uint16_t j = 0; j < this->arr_length[i]; j++){
            Serial.printf("%d,", *(this->arr[i] + j));            
        }
        Serial.println();
    }
}









uint16_t DataPtk::_flush_serial(){
    if (Serial.available() == 0) return 0;
    uint16_t c = 0;
    unsigned long start_t = millis();
    

    while (((millis() - start_t) < 5)){
        if (Serial.available() > 0){
            Serial.read();
            c += 1;
        }else{
            break;
        }
    }
    return c;
}