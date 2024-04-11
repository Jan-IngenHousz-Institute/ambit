#ifndef _DT_H
#define _DT_H

#include <Arduino.h>
#include "config.h"


#define NUM_ALL_ARR 8

#define ID_ARR_LENGTH 150
#define ID_ARR_DATA 151

#define RDY_DATA 200
#define GOT_DATA 201

#define RDY_LENGTH 202
#define GOT_LENGTH 203

enum ARRAY_TYPES {
    INT_ARR1, 
    INT_ARR2, 
    INT_ARR3, 
    INT_ARR4, 
    INT_ARR5, 
    INT_ARR6, 
};

class DataPtk{
    public:
    DataPtk();
    ~DataPtk();

    
    int get_arr_len();
    int allocate(uint8_t);
    int get_data_arr(uint8_t);
    void _print_all();
    int _get_serial_arr(uint8_t, uint8_t, uint8_t*);
    


    int8_t active_arr = -1;
    uint8_t *run_arr = NULL;
    const uint16_t timeout = 50;


    uint16_t arr_length[NUM_ALL_ARR] = {0};
    uint32_t* arr[NUM_ALL_ARR] = {NULL};



    

    private:
    bool _sender = true;
    uint16_t _flush_serial();
    
    
};


#endif