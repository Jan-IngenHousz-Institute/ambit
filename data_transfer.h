#ifndef _DT_H
#define _DT_H

#include <Arduino.h>
#include "config.h"

#define ID_ARR_LENGTH 72
#define ID_ARR_DATA 42
#define NUM_ALL_ARR 6




class DataPtk{
    public:
    DataPtk();
    ~DataPtk();

    
    int get_arr_len();
    int allocate(uint8_t);
    int get_data_arr(uint8_t);

    


    uint8_t run_type = 0;
    uint8_t *run_arr = NULL;


    uint16_t arr_length[NUM_ALL_ARR] = {0};
    uint32_t* arr[NUM_ALL_ARR] = {NULL};



    

    private:
    bool _sender = true;
    
};


#endif