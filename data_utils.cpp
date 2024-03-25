#define MAX_DATACLASS_SIZE 2000
static const char* TAG = "DATA_UTIL";


class dataclass{

    public:
    uint32_t* arr = NULL;                  // data array handle
    uint16_t write_ptr = 0;         // pointer to next input location
    uint16_t read_ptr = 0;          // pointer to next read location
    bool available = false;         // indicator of memory allocation
    bool write_available = false;   // available to add data
    bool read_available = false;    // available to read data
    uint16_t length = 0;

    


    dataclass();
    ~dataclass();
    void clear(void);
    void put(uint32_t data);
    uint16_t get_length(void);
    bool pop(uint32_t* data);
    bool init(uint16_t length);


    private:
    bool loop_ahead = false;
    uint16_t _overwritten_counter = 0;
    uint16_t _length = 0;            // size of data inside

};

dataclass::dataclass(){
    dataclass::available = false;
    dataclass::write_available = false;
    dataclass::read_available = false;
    return;
};

dataclass::~dataclass(void){
    if (dataclass::available) free(dataclass::arr);
    return;
};

bool dataclass::init(uint16_t length){
    if ((length > 0) and length < MAX_DATACLASS_SIZE){
        ESP_LOGV(TAG, "Allocation %d bytes", length * 4);
        (dataclass::arr) = (uint32_t*) (length, sizeof(uint32_t), MALLOC_CAP_32BIT);
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