#ifndef _DU_H
#define _DU_H

#include <Arduino.h>
#include "config.h"


#define MAX_DATACLASS_SIZE 2000


class dataclass{

    public:
    uint32_t *arr = NULL;           // data array handle
    uint16_t write_ptr = 0;         // pointer to next input location
    uint16_t read_ptr = 0;          // pointer to next read location
    bool available = false;         // indicator of memory allocation
    bool write_available = false;   // available to add data
    bool read_available = false;    // available to read data
    uint16_t length = 0;

    


    dataclass();
    ~dataclass();

    void clean(void);
    void clear(void);
    void put(uint32_t data);
    uint16_t get_length(void);
    bool pop(uint32_t* data);
    uint32_t pop(void);
    bool init(uint16_t length);
    void print_all();


    private:
    bool loop_ahead = false;
    uint16_t _overwritten_counter = 0;
    uint16_t _length = 0;            // size of data inside

};

bool send_and_wait_rsp(const char *s, const char *r, uint8_t rlen, uint8_t timeout);
void send_data(uint32_t* arr, uint16_t len);


#endif