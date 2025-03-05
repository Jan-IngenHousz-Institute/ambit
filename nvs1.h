#ifndef _NVS1_H_
#define _NVS1_H_
#include <Arduino.h>

#define MAJOR_VERSION 0
#define MINOR_VERSION 0
#define BATCH_VERSION 1


struct ambit_calibration_info_t{
    char ambit_name[20] = "AmbitV0.0";
    int32_t mlx_coef[14] = {0};
    float_t temp_offset = 0.0;
    float_t temp_slope = 1.0;
    float_t actinic_coef = 0.1;
    float_t spec_coef = 1.0;
    float_t spec_offset1 = 0.0;
    float_t spec_offset2 = 0.0;
    float_t mlx_emissivity = 1.0;
    float_t sun_coef = 1.0;   
};

extern struct ambit_calibration_info_t ambit_calibration_local, ambit_calibration_income;



struct ambit_FW_info_t{
    uint64_t MAC = 0;
    uint8_t Major = MAJOR_VERSION;
    uint8_t Minor = MINOR_VERSION;
    uint8_t Batch = BATCH_VERSION;
    uint8_t Pad = 0; 
    uint32_t Size = 0;
    char FW_date[12];
    char reserved[12];      
};

extern struct ambit_FW_info_t ambit_FW_info;

struct metadata_t {
    double lon = 1.0;
    double lat = 1.0;
    float alt = 1.0; 
    float acc = 1.0;
    float vacc = 1.0;

    uint32_t time = 1.0;
    float x = 1.0;
    float y = 1.0;
    float z = 1.0;

    char info1[200] = "NA";
};

extern struct metadata_t metadata_epprom, metadata_incoming;

void load_info_from_nvs(bool print);
void save_metadata(void);












#endif // _NVS1_H_