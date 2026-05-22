#include <Arduino.h>
#include "PAM.h"
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "driver/temperature_sensor.h"

extern ADPD6 adpd;
static char STR_SUC[] = "Success";
static char STR_FAIL[] = "Failed";


char* str_results[3] = {STR_FAIL, STR_SUC, STR_FAIL};


static int check_adpd(){
    Serial.print("Checking ADPD\t\t");
    uint8_t ret = (uint8_t)adpd.begin();
    return ret;
}

static int check_spec(){
    Serial.print("Checking AS7341\t\t");
    uint8_t ret = (uint8_t)check_AS7341();
    uint16_t spec[12] = {0};
    AS_all_channel(99, 199, spec);




    if (ret < 2) Serial.printf("%s", str_results[ret]);
    if (ret != 1){
        Serial.println("");
        return ret;
    }
    Serial.printf("\t%d,%d,%d,%d,%d,%d,%d,%d\n", spec[0], spec[1], spec[2], spec[3], spec[4], spec[5], spec[6], spec[7]);


    return ret;
}


static int check_mlx(){
    Serial.print("Checking MLX90632\t");
    uint8_t ret = (uint8_t)mlx_init();
    double obj, board;
    unsigned int start = millis();
    mlx_measure(&obj, &board);
    unsigned int time_used = millis() - start;
    if (ret < 2) Serial.printf("%s\t%d\t%2.2f\t%2.2f\n", str_results[ret], time_used, obj, board);
    return ret;
}


static int test_optic_path(){
    adpd.STOP();
    conf_slow_FR_1(40, 8, 0, 5, 5, 5, 5, 5, 5);

    adpd.num_ts(3);
    uint8_t expected_readout_bytes = 24;
    uint8_t expected_readout = 8;
    uint32_t ret[expected_readout] = {0};
    uint16_t fifo_c = 0;
    uint32_t counter = 0;
    uint32_t num_ptx = 64+64+64;

    AS_LED_OFF();
    AS_LED_Current(15);
    adpd.run_freq(128);

    uint32_t dark1[8] = {0};
    uint32_t dark2[8] = {0};
    uint32_t lit[8] = {0};

    adpd.RUN();
    while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
            adpd.readfifo(expected_readout, 3, ret);
            fifo_c -= expected_readout_bytes;
            if (counter == num_ptx) break;
            for (int i=0; i<8; i++){
                if (i < 2) ret[i] -= 65000;
                else if (i < 6) ret[i] -= 16000;
                if (counter < 64) dark1[i] += ret[i];
                else if (counter >= 128) dark2[i] += ret[i];
                else lit[i] += ret[i];
                // Serial.print(ret[i]);
                // if (i < 7) Serial.print(",");
            }
            // Serial.println("");
            counter++;
            if (counter == 64) AS_LED_ON();
            if (counter == 128) AS_LED_OFF();
        }        
    }

    adpd.STOP();
    AS_LED_OFF();
    AS_LED_Current(0);

    for (int i=0; i<8; i++){
        dark1[i] /= 64;
        dark2[i] /= 64;
        lit[i] /= 64;

        if (i < 6) Serial.printf("%d:[%d,%d,%d]\n",i + 1, (int)dark1[i]-300, (int)lit[i]-300, (int)dark2[i]-300);
        else Serial.printf("%d:[%d,%d,%d]\n",i + 1, dark1[i], lit[i], dark2[i]);

    }

    return 0;
}




static int test_optic_path2(){
    adpd.STOP();
    adpd.led_config.driver1_current = 0;
    adpd.led_config.driver2_current = 0;

    for (int i = 0; i < 5; i++){
        adpd.SNR_config.TIA_gain_CH2 = i + 1;       // channel 2: leaf IR reflection
        adpd.SNR_config.TIA_gain_CH1 = i + 1;      // channel 1: sun vis
        adpd.preset_config_1(i, 1);
    }


    adpd.led_config.driver1_current = 0;
    adpd.led_config.led1_channel = LED_A;
    adpd.led_config.led2_channel = LED_A;
    adpd.SNR_config.TIA_gain_CH1 = 5;
    adpd.SNR_config.TIA_gain_CH2 = 5;

    for (int i = 0; i < 5; i++){
        adpd.led_config.driver2_current = i * 10 + 10;
        adpd.preset_config_2(i + 5, 1);
    }


    uint8_t expected_readout_bytes = 2 * 3 * 5 + 4 * 3 * 5;
    uint8_t expected_readout = expected_readout_bytes / 3;
    uint32_t ret[expected_readout] = {0};
    uint16_t fifo_c = 0;
    uint32_t counter = 0;
    uint32_t num_ptx = 64;



    AS_LED_OFF();
    adpd.run_freq(128);

    uint32_t sun[5] = {0};
    uint32_t leaf[5] = {0};
    uint32_t sig[5] = {0};
    uint32_t ref[5] = {0};


    adpd.RUN();
    while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
            adpd.readfifo(expected_readout, 3, ret);
            fifo_c -= expected_readout_bytes;
            if (counter == num_ptx) break;

            for (uint8_t i = 0; i < 5; i++){
                sun[i] += ret[i * 2] - 16300;
                leaf[i] += ret[i * 2 + 1] - 16300;

                sig[i] += ret[10 + i * 4 + 1] - ret[10 + i * 4];
                ref[i] += ret[10 + i * 4 + 3] - ret[10 + i * 4 + 2];



            }

            counter++;  
        }        
    }

    adpd.STOP();
    AS_LED_OFF();

    Serial.printf("Sun PD\t\t%d\t%d\t%d\t%d\t%d\n", sun[4]/64,sun[3]/64,sun[2]/64,sun[1]/64,sun[0]/64);
    Serial.printf("Leaf PD\t\t%d\t%d\t%d\t%d\t%d\n", leaf[4]/64,leaf[3]/64,leaf[2]/64,leaf[1]/64,leaf[0]/64);
    Serial.printf("Signal\t\t%d\t%d\t%d\t%d\t%d\n", sig[0]/64,sig[1]/64,sig[2]/64,sig[3]/64,sig[4]/64);
    Serial.printf("Ref\t\t%d\t%d\t%d\t%d\t%d\n", ref[0]/64,ref[1]/64,ref[2]/64,ref[3]/64,ref[4]/64);

    return 0;
}



int check_connections(){
    int ret1 = check_adpd();
    int ret2 = check_spec();
    int ret3 = check_mlx();

    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(15, 55);
    float tsens_out;

    temperature_sensor_install(&temp_sensor_config, &temp_handle);
    temperature_sensor_enable(temp_handle);
    temperature_sensor_get_celsius(temp_handle, &tsens_out);
    printf("ESP32Temp\t\t%2.2f\n", tsens_out);
    temperature_sensor_disable(temp_handle);
    temperature_sensor_uninstall(temp_handle);


    Serial.println("ADPD_readings:");
    test_optic_path2();
    test_optic_path();
    Serial.println("Done!!");

    // if ((ret1 == 1) && (ret2 == 1) && (ret3 == 1)){
    //     AS_LED_OFF();
    //     AS_LED_Current(4);
    //     AS_LED_ON();
    //     delay(300);
    //     AS_LED_OFF();
    // } 
  




    return 0;
}


int optic_test(){
    adpd.STOP();
    conf_slow_FR_1(100, 20, 0, 1, 5, 5, 5, 5, 1);

    adpd.num_ts(3);
    uint8_t expected_readout_bytes = 24;
    uint8_t expected_readout = 8;
    uint32_t ret[expected_readout] = {0};
    uint16_t fifo_c = 0;
    uint32_t counter = 0;
    uint32_t num_ptx = 2000;
    uint32_t sig = 0;

    AS_LED_OFF();
    AS_LED_Current(50);
    adpd.run_freq(25);

    adpd.RUN();
    while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
            adpd.readfifo(expected_readout, 3, ret);
            fifo_c -= expected_readout_bytes;
            if (counter == num_ptx) break;
            sig = calc_signal(ret[2], ret[3], 1);
            Serial.printf("%d,%d,%d,%d,%d,%d,%d\n", ret[0]-65000,ret[1]-65000,ret[2]-16000,ret[3]-16000,ret[4]-16000,ret[5]-16000,sig);
            counter++;
        }        
    }

    adpd.STOP();
    AS_LED_OFF();
    AS_LED_Current(0);
    return 0;
}


int optic_test(uint8_t current, uint8_t num_integ, uint8_t lit_offset, uint8_t dark1_offset, uint8_t dark2_offset, uint8_t pulse_offset, uint8_t pulse_duration){
    adpd.STOP();
    fluor_offset_test(current, num_integ, lit_offset, dark1_offset, dark2_offset, pulse_offset, pulse_duration);

    adpd.num_ts(1);
    uint8_t expected_readout_bytes = 12;
    uint8_t expected_readout = 4;
    uint32_t ret[expected_readout] = {0};
    uint16_t fifo_c = 0;
    uint32_t counter = 0;
    uint32_t num_ptx = 300;
    uint32_t sig = 0;

    AS_LED_OFF();
    AS_LED_Current(100);
    adpd.run_freq(25);

    adpd.RUN();
    while (counter < num_ptx){
        fifo_c = adpd.fifo_count();
        while (fifo_c >= expected_readout_bytes){ // read all bytes from FIFO
            adpd.readfifo(expected_readout, 3, ret);
            fifo_c -= expected_readout_bytes;
            if (counter == num_ptx) break;
            if (counter == 100) AS_LED_ON();
            if (counter == 200) AS_LED_OFF();
            sig = calc_signal(ret[0], ret[1], num_integ);
            Serial.printf("%d,%d,%d,%d,%d\n", ret[0]-16000*num_integ, ret[1]-16000*num_integ,ret[2]-16000*num_integ,ret[3]-16000*num_integ, ret[1] - ret[0]);
            counter++;
        }        
    }

    adpd.STOP();
    AS_LED_OFF();
    AS_LED_Current(0);
    return 0;
}

