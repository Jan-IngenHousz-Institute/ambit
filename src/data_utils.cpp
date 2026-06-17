#include "data_utils.h"


static const char* TAG = "DATA_UTIL";
extern bool FLAG_DEICE;


// return 1, 2, 3
// or -1
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false){
    unsigned long start_t = millis();
    uint8_t counter, b = 0;

    while (((millis() - start_t) < timeout)){
        if (Serial.available() > 0){
            b = Serial.peek();
            if (b == target1){
                if (remove) Serial.read();
                return 1;
            } else if ((target2 > 0) && (b == target2)){
                if (remove) Serial.read();
                return 2;
            } else if ((target3 > 0) && (b == target3)){
                if (remove) Serial.read();
                return 3;
            }
        
            b = Serial.read();   // non-target byte: discard (do NOT echo onto the link —
                                 // an echo here injects bytes into the binary FSM stream)
        }
        else{
            delayMicroseconds(10);
        }
    }
    return -1;
}

uint16_t flush_serial(uint8_t timeout){    
    uint16_t c = 0;
    uint8_t r = 0;
    delay(1);
    unsigned long start_t = millis();
    if (Serial.available() < 1) return 0;

    while (((millis() - start_t) < timeout)){
        if (Serial.available() > 0){
            r = Serial.read();   // discard (do NOT echo onto the link — see serial_read_until)
            c += 1;
        }else{
            break;
        }
    }
    return c;
}



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



/* Sum of the 4 little-endian bytes of v. The data checksum is a byte-sum of
 * the payload (matching the bytes send_binary_array transmits and the ambyte's
 * verification) — summing the uint32 value into a uint8 would only cover the
 * low byte. */
static inline uint8_t u32_byte_sum(uint32_t v)
{
    return (uint8_t)v + (uint8_t)(v >> 8) + (uint8_t)(v >> 16) + (uint8_t)(v >> 24);
}

// Pack `len` uint32 values into the wire format, write them to Serial, and return the
// byte-sum of exactly the bytes written (defined below). elem_width 2|4, dtype 0|1.
static uint8_t send_binary_array(uint32_t* arr, uint16_t len, uint8_t elem_width, uint8_t dtype);

uint32_t dataclass::send(uint8_t elem_width, uint8_t dtype, uint8_t* c){
    uint8_t checksum = 0;
    if (dataclass::get_length() == 0){
        dataclass::read_available = false;
        ESP_LOGW(TAG, "No data available");
        return 0;
    }

    if (dataclass::loop_ahead){
        if (dataclass::write_ptr <= dataclass::read_ptr){
            checksum += send_binary_array(this->arr + this->read_ptr, this->_length - this->read_ptr, elem_width, dtype);
            checksum += send_binary_array(this->arr, this->read_ptr, elem_width, dtype);
            *c = checksum;
            return 1;
        }
        ESP_LOGW(TAG, "write_ptr > read_ptr After loop around!!!!");
        return 0;
    }else{
        if (dataclass::write_ptr > dataclass::read_ptr){
            checksum += send_binary_array(this->arr + this->read_ptr, this->write_ptr - this->read_ptr, elem_width, dtype);
            *c = checksum;
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

    if (!this->available) return; 
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

// "tag":[v0,v1,...] — channel array for the cloud/openJII JSON measurement sink.
void dataclass::send_json(const char* tag){
    if (!this->available) return;
    uint16_t n = dataclass::get_length();
    Serial.printf("\"%s\":[", tag);
    for (uint16_t i = 0; i < n; i++){
        if (i > 0) Serial.print(',');
        Serial.print(dataclass::pop());
    }
    Serial.print(']');
}

// Pack each value into elem_width bytes (little-endian low bytes), write to Serial, and
// return the byte-sum of exactly those bytes (Change 1: the data-trailer checksum is over
// the bytes actually sent). elem_width 4 = full uint32 (legacy/v1); 2 = low 16 bits with
// unsigned values (dtype 0) CLAMPED to 0xFFFF (Change 2 — calc_signal can exceed 16 bits
// near saturation; clamp, don't wrap). Signed (dtype 1) passes the stored int16 bit-pattern
// through unchanged; the host interprets the 2 bytes per dtype.
static uint8_t send_binary_array(uint32_t* arr, uint16_t len, uint8_t elem_width, uint8_t dtype){
    uint8_t checksum = 0;
    if (elem_width >= 4){
        Serial.write((uint8_t*) arr, (size_t) len * 4);
        for (uint16_t i = 0; i < len; i++) checksum += u32_byte_sum(arr[i]);
        return checksum;
    }
    // elem_width == 2: low 16 bits, batched into a small buffer to avoid per-byte writes.
    uint8_t buf[128];
    uint16_t bi = 0;
    for (uint16_t i = 0; i < len; i++){
        uint32_t v = arr[i];
        uint16_t w = ((dtype == 0) && (v > 0xFFFF)) ? 0xFFFF : (uint16_t) v;
        uint8_t lo = (uint8_t) w;
        uint8_t hi = (uint8_t) (w >> 8);
        buf[bi++] = lo;
        buf[bi++] = hi;
        checksum += lo + hi;
        if (bi >= sizeof(buf)){
            Serial.write(buf, bi);
            bi = 0;
        }
    }
    if (bi > 0) Serial.write(buf, bi);
    return checksum;
}

// Link-level wake handshake (Change 4: one wake per run, then arrays stream back-to-back).
// Sends WAKE_AMBYTE until the ambyte acks AMBYTE_AWAKE / AMBYTE_CALLS / AMBYTE_CALLFORRESET.
// Power saving preserved: light-sleep between pulses after the first 10 fast tries (unless
// FLAG_DEICE forces the busy "power wasting" path). Returns 1 awake, ERR_LOST_SYNC on
// AMBYTE_CALLS, 0 on AMBYTE_CALLFORRESET, ERR_NO_DATA_REQUEST on timeout. The leftover
// AMBYTE_AWAKE byte is left in the buffer; the first fsm_send_array flushes it.
int ambyte_wake(bool interrupt){
    (void) interrupt;   // abort byte handled by the run loop, not the wake; kept for the API
    unsigned int timer1 = millis();
    int ret = -1;
    uint8_t wake_up_reason, cmd_wait_time;
    int16_t wake_call_counter = 0;
    int16_t max_wake_try = 2000;
    cmd_wait_time = 15;
    flush_serial(10);
    // sending wake up calls, first 10 in 10ms, remaining pulse every second w/ light sleep
    while (wake_call_counter < max_wake_try){
        Serial.write(WAKE_AMBYTE);
        Serial.flush();
        ret = serial_read_until(AMBYTE_AWAKE, AMBYTE_CALLS, AMBYTE_CALLFORRESET, cmd_wait_time, false);
        if (ret > 0) break;
        // after first 10 fast tries, pulse every second with light sleep
        if (wake_call_counter == 10){
            esp_sleep_enable_timer_wakeup(1000000); //  set up light sleep timer
            cmd_wait_time = 100;
        }
        if (wake_call_counter > 10){
            wake_up_reason = 0;

            if (!FLAG_DEICE){  // Power saving mode
                esp_light_sleep_start();
                wake_up_reason = esp_sleep_get_wakeup_cause();
            }else{ // Power wasting mode
                for (uint8_t j = 0; j < 10; j++){
                    ret = serial_read_until(AMBYTE_AWAKE, AMBYTE_CALLS, AMBYTE_CALLFORRESET, 100, false);
                    if (ret > 0) break;
                }
            }

            if (wake_up_reason == 8){ // wake up by uart
                max_wake_try -= wake_call_counter;
                wake_call_counter = 0;
            }
        }
        wake_call_counter += 1;
        if (millis() - timer1 > 3600000) break;
    }

    if (ret == 1){ // Normal: ambyte awake
        ESP_LOGV(TAG, "Ambyte awake in %d ms with %d tries", millis() - timer1, wake_call_counter);
        return 1;
    }else if (ret == 2){ // Lost sync
        ESP_LOGE(TAG, "ERR_LOST_SYNC");
        Serial.read();
        return ERR_LOST_SYNC;
    }else if (ret == 3){
        ESP_LOGE(TAG, "Ambyte calls for re-start");
        Serial.read();
        return 0;
    }else{ // no response within timeout
        return ERR_NO_DATA_REQUEST;
    }
}

// Stream one array to an already-awake ambyte (Change 4): length header + data + trailer.
// Header (Change 1, self-describing): {212, 150, arr_idx, lenHi, lenLo, elem_width, dtype,
// csum}, csum = sum of bytes [0..6]. Then length*elem_width data bytes, then the trailer
// {212, 0, 0, csum} whose csum is the byte-sum of exactly those data bytes. The host loops
// on the 212 marker and stops when run_esp.cpp emits the run's trailing 240 instead.
// Returns 0 on success, ERR_CHECKSUM_FAILED if the host keeps rejecting the data.
int dataclass::fsm_send_array(uint8_t arr_idx, uint8_t elem_width, uint8_t dtype){
    if (!this->available) return 1;

    uint16_t n = dataclass::get_length();

    // --- length header ---
    uint8_t hdr[8] = {212, 150, arr_idx, 0, 0, elem_width, dtype, 0};
    hdr[3] = ((n >> 8) & 0xFF);
    hdr[4] = (n & 0xFF);
    for (uint8_t i = 0; i < 7; i++) hdr[7] += hdr[i];
    flush_serial(5);          // also clears the leftover AMBYTE_AWAKE from the wake
    Serial.write(hdr, 8);
    Serial.flush();

    int ret = serial_read_until(AMBYTE_READY_FOR_ARRAY, 0, AMBYTE_CALLFORRESET, 200, true);
    if (ret != 1){
        ESP_LOGE(TAG, "Array %d: no READY_FOR_ARRAY (got %d)", arr_idx, ret);
        return 0;
    }

    // --- data + trailer (retry while the host requests a resend) ---
    for (uint8_t attempt = 0; attempt < 6; attempt++){
        uint8_t trailer[4] = {212, 0, 0, 0};
        flush_serial(5);
        this->send(elem_width, dtype, &trailer[3]);   // write data, fill the trailer checksum
        Serial.flush();
        Serial.write(trailer, 4);
        Serial.flush();

        ret = serial_read_until(AMBYTE_DATA_PASS, AMBYTE_READY_FOR_ARRAY, AMBYTE_CALLFORRESET, 500, true);
        if (ret == 1){ // pass
            ESP_LOGV(TAG, "Array %d sent: %d elems x %d B", arr_idx, n, elem_width);
            return 0;
        }else if (ret == 2){ // host asked to resend (sent READY_FOR_ARRAY again)
            ESP_LOGW(TAG, "Array %d: resend requested", arr_idx);
            continue;
        }else if (ret == 3){ // call for restart
            ESP_LOGE(TAG, "Ambyte calls for re-start");
            return 0;
        }else{
            ESP_LOGE(TAG, "Array %d: no DATA_PASS", arr_idx);
            return 0;
        }
    }
    return ERR_CHECKSUM_FAILED;
}

int dataclass::fsm_send_esp(uint8_t arr_idx){
    return this->fsm_send_esp(arr_idx, false);
}

// Legacy single-array convenience: wake the link then send one uint32 array (elem_width 4,
// dtype 0 -> host decodes as the v1 format). The primary run path (pam_send_results) instead
// wakes once via ambyte_wake() and streams every array with fsm_send_array().
int dataclass::fsm_send_esp(uint8_t arr_idx, bool use_interrupt){
    if (!this->available) return 1;
    if (ambyte_wake(use_interrupt) != 1) return ERR_NO_DATA_REQUEST;
    this->fsm_send_array(arr_idx, 4, 0);
    return 1;
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

