#include <Arduino.h>
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "serial.h"
#include "do_command.h"
#include "src/wrench.h"

#include "config.h"

static const char* TAG = "INO";
ADPD6 adpd;






void setup(){
    esp_timer_early_init();
    pinMode(1, OUTPUT);
    pinMode(10, OUTPUT);
    digitalWrite(1, LOW);
    digitalWrite(10, LOW);

    Serial.begin(115200);
    delay(500);
    Serial.println("BOOT");
    Serial.println(esp_timer_get_time());

    digitalWrite(1, HIGH);
    delay(1);
    digitalWrite(1, LOW);
    init_i2c_bus();
    init_spi_bus();
    adpd.begin();
    if (as7341.begin()) ESP_LOGV(TAG, "AS7341 Found");
    check_AS7341();
    mlx_init();

}




void loop(){

    int c = -1;
    for (;;) {
        c = Serial.available();
        if (c > 1){            // received something
            break;        
        }
        delay(10);
    }  
    char choose[50]; //buffer to hold commands    
    Serial_Input_Chars(choose, ":,", 500, sizeof(choose) - 1);
    do_command(choose);


    
    // WRState* w = wr_newState(); // create the state

	// wr_registerFunction( w, "print", print ); // bind a function

	// unsigned char* outBytes; // compiled code is alloc'ed
	// int outLen;

	// int err = wr_compile( wrenchCode, strlen(wrenchCode), &outBytes, &outLen ); // compile it
	// if ( err == 0 )
	// {
	// 	wr_run( w, outBytes, outLen ); // load and run the code!
	// 	delete[] outBytes; // clean up 
	// }

	// wr_destroyState( w );





}