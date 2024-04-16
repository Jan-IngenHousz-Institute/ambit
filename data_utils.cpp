#include "data_utils.h"


static const char* TAG = "DATA_UTIL";

dataclass::dataclass(){
    dataclass::available = false;
    dataclass::write_available = false;
    dataclass::read_available = false;
    ESP_LOGV(TAG, "DATACLASS Created");
    return;
};

dataclass::~dataclass(void){
    ESP_LOGV(TAG, "DATACLASS destroied");    
    if (dataclass::available) dataclass:clean();  
    return;
};

void dataclass::clean(void){
    if (dataclass::available) free((void*) (dataclass::arr));
    ESP_LOGV(TAG, "Memory freed");
    dataclass::arr = NULL;
    dataclass::available = false;
    dataclass::write_available = false;
    dataclass::read_available = false;
    return;
};


bool dataclass::init(uint16_t length){
    if ((length > 0) and length < MAX_DATACLASS_SIZE){
        ESP_LOGV(TAG, "Allocation %d bytes", length * 4);
        (dataclass::arr) = (uint32_t*) heap_caps_calloc(length, sizeof(uint32_t), MALLOC_CAP_32BIT);
        if (dataclass::arr == NULL){
            ESP_LOGE(TAG, "Allocation %d bytes failed", length * 4);
            return false;
        }
        dataclass::_length = length;
        dataclass::available = true;
        dataclass::clear();
        dataclass::write_available = true;
        dataclass::write_ptr = 0;
        dataclass::read_ptr = 0;
        dataclass::read_available = false;
        dataclass::_overwritten_counter = 0;
        dataclass::loop_ahead = false;
        return true;
    }
    else{
        ESP_LOGE(TAG, "Out-of-range: %d bytes ", length * 4);
        return false;
    }    
    return false;
};

// set data array to 0
void dataclass::clear(void){
    if (dataclass::available){
        for(uint16_t i = 0; i < dataclass::_length; i++) (dataclass::arr)[i] = 0;
        dataclass::write_ptr = 0;
        dataclass::read_ptr = 0;
        dataclass::loop_ahead = false;
        ESP_LOGV(TAG, "ARR reset to 0");
    }else{
        ESP_LOGE(TAG,"RESET failed, not initialized");
    }
}

// add the next point to the data array
// update loop_over
void dataclass::put(uint32_t data){
    if (!(dataclass::available)){
        ESP_LOGE(TAG, "Add failed, not initialized");
        return;
    }

    // write point cycling around and reach read pointer
    if ((dataclass::write_ptr >= dataclass::read_ptr) && (dataclass::loop_ahead)){
        dataclass::read_ptr = dataclass::write_ptr + 1;
        dataclass::length -= dataclass::write_ptr - dataclass::read_ptr + 1;
        if (dataclass::read_ptr >= dataclass::_length) dataclass::read_ptr = 0;
        dataclass::_overwritten_counter += 1;
        ESP_LOGW(TAG, "Data over-written, total: %d", dataclass::_overwritten_counter);
    }

    dataclass::arr[write_ptr] = data;
    dataclass::length += 1;
    dataclass::write_ptr += 1;

    dataclass::read_available = true;
    if (dataclass::write_ptr >= dataclass::_length){
        dataclass::write_ptr = 0;
        dataclass::loop_ahead = true;
    }

}

uint16_t dataclass::get_length(void){
    if (!(dataclass::available)){
        ESP_LOGE(TAG, "Add failed, not initialized");
        return 0;
    }
    if (!(dataclass::read_available)){
        ESP_LOGW(TAG, "No data available");
        return 0;
    }
    if (dataclass::loop_ahead){
        if (dataclass::write_ptr <= dataclass::read_ptr) return
                    dataclass::_length - dataclass::read_ptr + dataclass::write_ptr;
        ESP_LOGW(TAG, "write_ptr > read_ptr After loop around!!!!");
        return 0;
    }else{
        if (dataclass::write_ptr > dataclass::read_ptr) return (dataclass::write_ptr - dataclass::read_ptr);
        if (dataclass::write_ptr == dataclass::read_ptr){
            dataclass::read_available = false;
            return 0;
        }
        ESP_LOGW(TAG, "write_ptr < read_ptr WITHOUT loop around!!!!");
        return 0;
    }
    return 0;  // should not reach here
}



uint32_t dataclass::send(void (*func) (uint32_t*, uint16_t)){
    if (dataclass::get_length() == 0){
        dataclass::read_available = false;
        ESP_LOGW(TAG, "No data available");
        return 0;
    }

    if (dataclass::loop_ahead){
        if (dataclass::write_ptr <= dataclass::read_ptr){
            func(this->arr + this->read_ptr, this->_length - this->read_ptr);
            func(this->arr, this->read_ptr);
            return 1;
        }
        ESP_LOGW(TAG, "write_ptr > read_ptr After loop around!!!!");
        return 0;
    }else{
        if (dataclass::write_ptr > dataclass::read_ptr){
            func(this->arr + this->read_ptr, this->write_ptr - this->read_ptr);
            return 1;}

        if (dataclass::write_ptr == dataclass::read_ptr){
            dataclass::read_available = false;
            return 0;
        }
        ESP_LOGW(TAG, "write_ptr < read_ptr WITHOUT loop around!!!!");
        return 0;
    }
    return 0;   
}

bool dataclass::pop(uint32_t* data){
    if (!(dataclass::available)){
        ESP_LOGE(TAG, "Add failed, not initialized");
        return false;
    }
    if (!(dataclass::read_available)){
        ESP_LOGW(TAG, "No data available");
        return false;
    }
    
    if (dataclass::get_length() == 0){
        dataclass::read_available = false;
        ESP_LOGW(TAG, "No data available");
        return false;
    }

    if (dataclass::read_ptr >= dataclass::_length){
        ESP_LOGE(TAG, "Read_pointer outside range");
        return false;
    }

    *data = dataclass::arr[dataclass::read_ptr];
    dataclass::read_ptr += 1;
    dataclass::length -= 1;
    if (dataclass::read_ptr >= dataclass::_length){
        dataclass::read_ptr = 0;
        dataclass::loop_ahead = false;
    }
    return true;   
}

uint32_t dataclass::pop(void){
    uint32_t a;
    if (dataclass::pop(&a)) return a;
    return 0xABCDEF01;
}

void dataclass::send_serial(const char* tag){
    uint16_t tmp_var = dataclass::get_length();
    Serial.printf("Data:%s,Length:%d\t", tag, tmp_var);
    if (tmp_var == 0){
        Serial.println();
        return;
    }
    for (uint16_t i = 0; i < tmp_var; i++){
        Serial.printf("%d,", dataclass::pop());
    }
    Serial.print("\n");
    return;
}

enum AMBYTE_STATUS {
    IDEL,
    AWAKE,
    WAIT_FOR_DATA,
};

static void send_binary_array(uint32_t* arr, uint16_t len){
    Serial.write((uint8_t*) arr, len * 4);
}


void dataclass::send_esp(uint8_t arr_num){

    uint8_t ambyte_status, target = 0;
    unsigned int timer1 = 0;
    Serial.setTimeout(20);
    uint16_t tmp_var = dataclass::get_length();    
    
    //--- step one, wake up ambyte
    timer1 = millis();
    target = AMBYTE_AWAKE;
    for (uint8_t i = 0; i <5; i++){
        Serial.write(WAKE_AMBYTE);
        if (Serial.find(&target, 1)){
            ambyte_status = AMBYTE_STATUS::AWAKE;
            ESP_LOGV(TAG, "Wake response in %d ms", millis() - timer1);
            break;
        };
    }

    if (ambyte_status != AMBYTE_STATUS::AWAKE){
        ESP_LOGE(TAG, "Awake failed");
        return;
    }

    //------------------------------
    //--- step two, send array type and length
    // 150, array number, sizeH, sizeL, 0, checksum
    uint8_t arr[6] = {150, arr_num, 0, 0, 0, 0};
    arr[2] = ((tmp_var >> 8) & 0xFF);
    arr[3] = (((tmp_var) & 0xFF));
    for (uint8_t i = 0; i < 5; i++) arr[5] += arr[i];
    
    timer1 = millis();
    target = AMBYTE_READY_FOR_ARRAY;
    
    for (uint8_t i = 0; i < 3; i++){
        Serial.write(arr, 6);
        if (Serial.find(&target, 1)){
            ambyte_status = AMBYTE_STATUS::WAIT_FOR_DATA;
            ESP_LOGV(TAG, "ready for data %d", millis() - timer1);
            break;
        };
    }

    if (ambyte_status != AMBYTE_STATUS::WAIT_FOR_DATA){
        ESP_LOGE(TAG, "Not available for data");
        return;
    }

    //--------------------------------
    // --- step three, send data
    // 151, res, res, res 4 bytes array
    memset(arr, 0, sizeof(arr));
    arr[0] = 151;
    timer1 = millis();
    Serial.write(arr, 4);
    this->send(send_binary_array);
    ESP_LOGV(TAG, "Spend %d ms sending %d data", millis() - timer1, tmp_var);

    return;
}



#ifdef DEBUG_CODES

void dataclass::print_all(void){

    if (!(dataclass::available)){
        ESP_LOGE(TAG, "Add failed, not initialized");
        return;
    }
    for (uint16_t i = 0; i < dataclass::_length; i++){
        Serial.print(dataclass::arr[i]);
        Serial.print(",");
    }
    Serial.println();
    return;
}

#endif


int wait_for_response_clear(const char* s, uint8_t slen, uint8_t timeout){
    unsigned long timer1 = millis();
    uint16_t n = 0;
    Serial.setTimeout(timeout);

    if (Serial.find(s, slen)){
        //ESP_LOGI(TAG, "Responsed in %d ms\n",  millis() - timer1);
        while(Serial.available()){
            Serial.read();
            n += 1;
        }
        if (n > 0) ESP_LOGW(TAG, "received %d unexpected bytes after %s", n, s);
        return 0;
    }
    return -1;
}

bool send_and_wait_rsp(const char *s, const char *r, uint8_t rlen, uint8_t timeout){
    Serial.println(s);
    uint8_t i = wait_for_response_clear(r, rlen, timeout);
    if (i == 0) return true;
    return false;
}

void write32(uint32_t v){
    uint8_t a = (v) & 0xFF;
    uint8_t b = ((v) >> 8) & 0xFF;
    uint8_t c = ((v) >> 16) & 0xFF;
    uint8_t d = ((v) >> 24) & 0xFF;
    Serial.write(d);
    Serial.write(c);
    Serial.write(b);
    Serial.write(a);
    return;
}

void send_data(uint32_t* arr, uint16_t len){

    char c[10];

    // Wake up sleep device
    if (send_and_wait_rsp("Wake!", "Ready", 5, 10)){        
        // send data size
        sprintf(c, "%d", len);
        if (send_and_wait_rsp(c, "GO", 2, 10)){
          uint32_t checksum = 0;
          for (uint16_t n = 0; n < len; n++){
            write32((uint32_t) arr[n]);
            checksum += arr[n];
          }
          write32(checksum);          
          send_and_wait_rsp("DONE", "Check", 5, 10);
        }               
       }
}

/*
    insert a number to an array with order
    large number will insert towards the end
    used for get median
    @param arr: array with sorted data
    @param length: length of the array
    @param c: new data to be inserted
*/
void sorted_insert(uint32_t arr[], uint16_t length, uint32_t c){
  uint16_t n = 0;
  uint16_t nM = length - 1;

  while (n < nM){
    if (arr[n] == 0){
      arr[n] = c;
      break;
    }
    else if (c < arr[n]){
      for (uint8_t g = nM; g > n; g--){
        arr[g] = arr[g - 1];
      }
      arr[n] = c;
      break;
    } else if (arr[n + 1] == 0){
      arr[n + 1] = c;
      break;
    }else{
      n += 1;
    }
  }
  return;
}


uint32_t calc_signal(const uint32_t dark, const uint32_t lit, const uint8_t num){
    if (lit + 250 < dark) return 0;
    uint32_t a = (lit - dark + 250);
    int32_t b = a + 98 - dark * 0.006 / num;

    //int32_t tmp_var = (lit - dark + 250) - (0.006 / p) * (dark - 16384UL * p);
    if (b > 0) return b;
    return 0;
}

