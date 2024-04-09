#include "src/wrench.h"
#include "src/mlx90632/u_mlx.h"
#include "src/as7341/spec_meas.h"

#define WR_MAX_ARR 640
static const char* TAG = "DO_C";

uint8_t pulsed_620_current, pulsed_720_current, dc_current = 0;
uint8_t gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf = 5;
uint8_t status_run_config_set = 0;


double get_PAR();
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int run_arr_type1(uint8_t length, uint8_t* arr, bool);
int MPF(uint16_t mode, uint16_t current, uint16_t dc_current, uint8_t sign_gain, uint8_t ref_gain);

// Run array operations ---------------------------------------------
uint8_t wr_run_arr[WR_MAX_ARR] = {0};

static void arr_reset(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (argn == 1){
        uint8_t arr_num = (uint8_t) argv[0].asInt() - 1;
        if ((arr_num == 0) || (arr_num > 7)) return;
        for (uint8_t i = 0; i < WR_MAX_ARR; i++) wr_run_arr[i + arr_num * 64] = 0;
        ESP_LOGV(TAG, "array %d reset", arr_num + 1);
    }else{
        ESP_LOGE(TAG, "array reset with bad param");
    }
}

    

static void arr_set(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    // line num, linetype, sample num, frequency, actinic, subsampling
    if (argn == 8){
        uint8_t arr_num = (uint8_t) argv[0].asInt() - 1;
        uint8_t line_num = (uint8_t) argv[1].asInt() - 1;
        uint8_t ir_on = (uint8_t) argv[2].asInt();
        uint8_t line_type = (uint8_t) argv[3].asInt();
        uint16_t sample_num = (uint16_t) argv[4].asInt();
        uint16_t freq = (uint16_t) argv[5].asInt();
        uint8_t actinic = (uint8_t) argv[6].asInt();
        uint8_t subsampling = (uint8_t) argv[7].asInt();
        ESP_LOGV(TAG,"Set arr%d, line %d to type %d with %d x %dHz samples, actinic:%d, sub:%d, IR:%d", arr_num, line_num, line_type, sample_num, freq, actinic, subsampling, ir_on);

        
        if (arr_num > 7){
            ESP_LOGV(TAG, "arr number > 8");
            return;
        }

        if (line_num > 7){
            ESP_LOGV(TAG, "line number > 8");
            return;
        }

        uint16_t base_num = arr_num * 64 + line_num * 8;


        wr_run_arr[base_num + 0] = (uint8_t) line_type;
        wr_run_arr[base_num + 1] = (uint8_t) ir_on;

        wr_run_arr[base_num + 2] = (uint8_t) (sample_num >> 8);
        wr_run_arr[base_num + 3] = (uint8_t) (sample_num & 0x00FF);

        wr_run_arr[base_num + 4] = (uint8_t) (freq >> 8);
        wr_run_arr[base_num + 5] = (uint8_t) (freq & 0x00FF);

        wr_run_arr[base_num + 6] = actinic;
        wr_run_arr[base_num + 7] = subsampling;
    }
    else{
        ESP_LOGE(TAG, "ARR_SET with length %d", argn);
    } 
}

static void run(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (status_run_config_set == 0){
        conf_slow_FR_1(pulsed_620_current, pulsed_720_current, dc_current, gain_fluor, gain_fluref, gain_sun, gain_leaf, gain_720, gain_720ref);
        status_run_config_set = 1;
    }

    if (argn == 2){
        uint8_t arr_num = (uint8_t) argv[0].asInt() - 1;
        bool led_persist = (bool) argv[1].asInt();
        if ((arr_num < 8)){
            run_arr_type1(8, wr_run_arr + arr_num * 64, led_persist);
        }
        else{
            ESP_LOGE(TAG, "run with bad arr num %d", arr_num + 1);
        }        
    }else{
        ESP_LOGE(TAG, "BAD run array parameter");
    }    
}

// ultilities------------------------------------------------------
static void print( WRContext* c, const WRValue* argv, const int argn, WRValue& retVal, void* usr )
{
	char buf[1024];
    
	for( int i=0; i<argn; ++i )
	{
		Serial.printf( "%s", argv[i].asString(buf, 1024) );
	}
    Serial.println('\r');
}

static void disp(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    for (uint8_t i = 0; i < (WR_MAX_ARR/8); i++){
        for (uint8_t j = 0; j < 8; j++){
            Serial.print(wr_run_arr[i * 8 + j]);
            Serial.print(",");
        }
        Serial.print("\n");    
    }
}




// ambit functions----------------------------------------------
static void wr_get_par(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    float par = get_PAR();
    wr_makeFloat(&retVal, par);
}

static void wr_get_leaf_temp(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    float temp = mlx_measure();
    wr_makeFloat(&retVal, temp);
}

static void detector_preset(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (argn == 9){
        conf_slow_FR_1((uint8_t)argv[0].asInt(), (uint8_t)argv[1].asInt(), (uint8_t)argv[2].asInt(), (uint8_t)argv[3].asInt(), \
        (uint8_t)argv[4].asInt(), (uint8_t)argv[5].asInt(), (uint8_t)argv[6].asInt(), (uint8_t)argv[7].asInt(), (uint8_t)argv[8].asInt());
        status_run_config_set = 1;
        ESP_LOGV(TAG,"ADPD detector preset with current: %d", (uint8_t) argv[0].asInt());
        return;
    }
    else if(argn == 0){
        conf_slow_FR_1(pulsed_620_current, pulsed_720_current, dc_current, gain_fluor, gain_fluref, gain_sun, gain_leaf, gain_720, gain_720ref);
        status_run_config_set = 1;
        ESP_LOGV(TAG, "ADPD detector preset with stored values");
    }
    else{
        ESP_LOGE(TAG, "ADPD detector preset, Get: %d of arguments", argn);
    }
}


static void set_currents(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (argn == 0) return;
    if (argn > 0) pulsed_620_current = (uint8_t)argv[0].asInt();
    if (argn > 1) pulsed_720_current = (uint8_t)argv[1].asInt();
    if (argn > 2) dc_current = (uint8_t)argv[2].asInt();
    ESP_LOGV(TAG, "set pulsed-620 to: %d, set pulsed-720 to: %d, set far_red to: %d", pulsed_620_current, pulsed_720_current, dc_current);
    return;
}

static void set_gains(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (argn == 0) return;
    if (argn > 0) gain_fluor = (uint8_t)argv[0].asInt();
    if (argn > 1) gain_fluref = (uint8_t)argv[1].asInt();
    if (argn > 2) gain_720 = (uint8_t)argv[2].asInt();
    if (argn > 3) gain_720ref = (uint8_t)argv[3].asInt();
    if (argn > 4) gain_sun = (uint8_t)argv[4].asInt();
    if (argn > 5) gain_leaf = (uint8_t)argv[5].asInt();
    ESP_LOGV(TAG, "Fluor Gain:%d, Fluo ref Gain:%d, 730 Gain:%d, 730ref Gain:%d, Sun Gain:%d, Leaf Gain:%d", gain_fluor, gain_fluref, gain_720, gain_720ref, gain_sun, gain_leaf);
    return;
}

static void run_mpf(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    if (argn == 2){
        uint8_t mode = (uint8_t)argv[0].asInt();
        uint8_t actinic = (uint8_t)argv[1].asInt();
        MPF(mode, pulsed_620_current, actinic, gain_fluor, gain_fluref);
        status_run_config_set= 0;
    }else{
        ESP_LOGE(TAG, "MPF, Get: %d of arguments", argn);
    }
}

static void idle(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    AS_LED_OFF();    
}



void do_c(const char* c){

    WRState* w = wr_newState(); // create the state
    wr_loadMathLib( w );

    wr_registerFunction( w, "print", print ); // bind a function
    wr_registerFunction( w, "disp", disp ); // bind a function

    wr_registerFunction( w, "get_par", wr_get_par ); // bind a function
    wr_registerFunction( w, "get_temp", wr_get_leaf_temp ); // bind a function

    wr_registerFunction( w, "reset", arr_reset ); // bind a function
    wr_registerFunction( w, "config", detector_preset); // bind a function
    wr_registerFunction( w, "set_arr", arr_set ); // bind a function
    wr_registerFunction( w, "run", run ); // bind a function
    wr_registerFunction( w, "set_currents", set_currents ); // bind a function
    wr_registerFunction( w, "set_gains", set_gains ); // bind a function
    wr_registerFunction( w, "run_mpf", run_mpf ); // bind a function
    


      

    unsigned char* outBytes; // compiled code is alloc'ed
    int outLen;

    int err = wr_compile( c, strlen(c), &outBytes, &outLen ); // compile it
    if ( err == 0 )
    {
    wr_run( w, outBytes, outLen ); // load and run the code!
    delete[] outBytes; // clean up 
    }

    wr_destroyState( w );
    Serial.println("Cmd Done!");



}
