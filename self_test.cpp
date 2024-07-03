#include <Arduino.h>
#include "PAM.h"
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"


extern ADPD6 adpd;
static char STR_SUC[] = "Success";
static char STR_FAIL[] = "Failed";


char* str_results[3] = {STR_FAIL, STR_SUC, STR_FAIL};


static int check_adpd(){
    uint8_t ret = (uint8_t)adpd.begin();
    if (ret < 2) Serial.printf("ADPD check %s", str_results[ret]);
    return ret;
}

static int check_spec(){
    uint8_t ret = (uint8_t)check_AS7341();
    if (ret < 2) Serial.printf("AS7341 check %s", str_results[ret]);
    return ret;
}


static int check_mlx(){
    uint8_t ret = (uint8_t)mlx_init();
    if (ret < 2) Serial.printf("MLX90632 check %s", str_results[ret]);
    return ret;
}


static int test_optic_path(){
    adpd.STOP();
    conf_slow_FR_1(40, 20, 0, 5, 5, 5, 5, 5, 3);

    adpd.num_ts(3);
    uint8_t expected_readout_bytes = 24;
    uint8_t expected_readout = 8;
    uint32_t ret[expected_readout] = {0};
    uint16_t fifo_c = 0;
    uint32_t counter = 0;
    uint32_t num_ptx = 500;

    AS_LED_OFF();
    AS_LED_Current(50);
    adpd.run_freq(50);

    adpd.RUN();
    while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
            adpd.readfifo(expected_readout, 3, ret);
            fifo_c -= expected_readout_bytes;
            if (counter == num_ptx) break;
            Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", ret[0],ret[1],ret[2],ret[3],ret[4],ret[5],ret[6],ret[7]);
            counter++;
            if (counter == 120) AS_LED_ON();
            if (counter == 360) AS_LED_OFF();
        }        
    }

    adpd.STOP();
    AS_LED_OFF();
    AS_LED_Current(0);
    return 0;
}




int check_connections(){
    check_adpd();
    check_spec();
    check_mlx();
    test_optic_path();
    return 0;
}