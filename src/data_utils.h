#ifndef _DU_H
#define _DU_H

#include <Arduino.h>
#include "config.h"


#define MAX_DATACLASS_SIZE 2000
#define WAKE_AMBYTE 211
#define AMBYTE_AWAKE 210
#define AMBYTE_CALLS 170
#define AMBYTE_CALLFORRESET 222
#define AMBYTE_INTR 177

#define AMBYTE_READY_FOR_ARRAY 200
#define AMBYTE_DATA_PASS 180

#define ERR_CHECKSUM_FAILED -10
#define ERR_TOO_MANY_WKUP -4
#define ERR_TOO_MANY_RETRY -9
#define ERR_LOST_SYNC -2
#define ERR_NO_DATA_REQUEST -5

class dataclass{

    public:
    uint32_t *arr = NULL;           // data array handle
    uint16_t write_ptr = 0;         // pointer to next input location
    uint16_t read_ptr = 0;          // pointer to next read location
    uint16_t peek_ptr = 0;          // pointer to next read location
    bool available = false;         // indicator of memory allocation
    bool write_available = false;   // available to add data
    bool read_available = false;    // available to read data
    uint16_t length = 0;

    uint16_t num_retry = 0;




    dataclass();
    ~dataclass();

    void clean(void);
    void clear(void);
    void put(uint32_t data);
    uint16_t get_length(void);
    bool pop(uint32_t* data);
    uint32_t pop(void);
    // Pack each stored uint32 into elem_width (2|4) bytes — low bytes, unsigned values
    // clamped to the field — write them to Serial and return the byte-sum of exactly
    // those bytes in *c. dtype (0 unsigned | 1 signed) is wire metadata for the host.
    uint32_t send(uint8_t elem_width, uint8_t dtype, uint8_t* c);
    bool init(uint16_t length);
    void print_all();
    void send_serial(const char*);
    void send_json(const char*);   // "tag":[v0,v1,...] for the cloud/openJII sink

    //-- ambyte binary protocol --//
    // Stream one array to an already-awake ambyte: length header + data + trailer.
    // elem_width = 2|4 bytes/elem, dtype = 0 unsigned | 1 signed (Change 1/2/4).
    int fsm_send_array(uint8_t arr_idx, uint8_t elem_width, uint8_t dtype);
    // Legacy convenience: wake the link then send one uint32 array (elem_width 4, dtype 0).
    int fsm_send_esp(uint8_t arr_idx);
    int fsm_send_esp(uint8_t arr_idx, bool interrupt);


    private:
    bool loop_ahead = false;
    uint16_t _overwritten_counter = 0;
    uint16_t _length = 0;            // size of data inside

};

void sorted_insert(uint32_t arr[], uint16_t length, uint32_t c);
uint32_t calc_signal(uint32_t, uint32_t, uint8_t);

// Link-level wake handshake for the ambyte binary protocol (Change 4: one wake per
// run, then arrays stream back-to-back). Returns 1 when the ambyte is awake.
int ambyte_wake(bool interrupt);


#endif