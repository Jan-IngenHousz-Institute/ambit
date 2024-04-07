#include "src/wrench.h"
#include "src/mlx90632/u_mlx.h"
#include "src/as7341/spec_meas.h"

#define WR_MAX_ARR 64
static const char* TAG = "DO_C";


double get_PAR();
int conf_slow_FR_1(uint8_t I620, uint8_t I730, uint8_t I_FR, uint8_t G_Fluor, uint8_t G_FluorRef, uint8_t G_Sun, uint8_t G_IR, uint8_t G_FR, uint8_t G_FRref);
int run_arr_type1(uint8_t length, uint8_t* arr);

uint8_t wr_run_arr[WR_MAX_ARR] = {0};

static void arr_reset(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    for (uint8_t i = 0; i < WR_MAX_ARR; i++) wr_run_arr[i] = 0;
}

static void wr_get_par(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    float par = get_PAR();
    wr_makeFloat(&retVal, par);
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

static void arr_set(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    // line num, linetype, sample num, frequency, actinic, subsampling
    if (argn == 7){
        uint8_t line_num = (uint8_t) argv[0].asInt() - 1;
        uint8_t ir_on = (uint8_t) argv[1].asInt();
        uint8_t line_type = (uint8_t) argv[2].asInt();
        uint16_t sample_num = (uint16_t) argv[3].asInt();
        uint16_t freq = (uint16_t) argv[4].asInt();
        uint8_t actinic = (uint8_t) argv[5].asInt();
        uint8_t subsampling = (uint8_t) argv[6].asInt();
        ESP_LOGV(TAG,"Set line %d to type %d with %d x %dHz samples, actinic:%d, sub:%d, IR:%d", line_num, line_type, sample_num, freq, actinic, subsampling, ir_on);
        if (line_num >= (WR_MAX_ARR / 8)) return;
        wr_run_arr[line_num * 8 + 0] = (uint8_t) line_type;
        wr_run_arr[line_num * 8 + 1] = (uint8_t) ir_on;

        wr_run_arr[line_num * 8 + 2] = (uint8_t) (sample_num >> 8);
        wr_run_arr[line_num * 8 + 3] = (uint8_t) (sample_num & 0x00FF);

        wr_run_arr[line_num * 8 + 4] = (uint8_t) (freq >> 8);
        wr_run_arr[line_num * 8 + 5] = (uint8_t) (freq & 0x00FF);

        wr_run_arr[line_num * 8 + 6] = actinic;
        wr_run_arr[line_num * 8 + 7] = subsampling;
    }
    else{
        ESP_LOGE(TAG, "ARR_SET with length %d", argn);
    } 
}

static void run(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    run_arr_type1(8, wr_run_arr);
}


void print( WRContext* c, const WRValue* argv, const int argn, WRValue& retVal, void* usr )
{
	char buf[1024];
    
	for( int i=0; i<argn; ++i )
	{
		Serial.printf( "%s", argv[i].asString(buf, 1024) );
	}
    Serial.println('\r');
}



void detector_preset(WRContext* c,const WRValue* argv,const int argn, WRValue& retVal, void* usr){
    Serial.println(argn);
    if (argn == 9){
        conf_slow_FR_1((uint8_t)argv[0].asInt(), (uint8_t)argv[1].asInt(), (uint8_t)argv[2].asInt(), (uint8_t)argv[3].asInt(), \
        (uint8_t)argv[4].asInt(), (uint8_t)argv[5].asInt(), (uint8_t)argv[6].asInt(), (uint8_t)argv[7].asInt(), (uint8_t)argv[8].asInt());
        ESP_LOGV(TAG,"ADPD detector preset with current: %d", (uint8_t) argv[0].asInt());
        return;
    }
    else{
        ESP_LOGE(TAG, "ADPD detector preset, Get: %d of arguments", argn);
    }
}




void do_c(const char* c){

     WRState* w = wr_newState(); // create the state
      wr_registerFunction( w, "print", print ); // bind a function
      wr_registerFunction( w, "config", detector_preset ); // bind a function
      wr_registerFunction( w, "set_arr", arr_set ); // bind a function
      wr_registerFunction( w, "disp", disp ); // bind a function
      wr_registerFunction( w, "run", run ); // bind a function
      wr_registerFunction( w, "reset", arr_reset ); // bind a function
      wr_registerFunction( w, "get_par", wr_get_par ); // bind a function
      wr_loadMathLib( w );

      unsigned char* outBytes; // compiled code is alloc'ed
      int outLen;

      int err = wr_compile( c, strlen(c), &outBytes, &outLen ); // compile it
      if ( err == 0 )
      {
        wr_run( w, outBytes, outLen ); // load and run the code!
        delete[] outBytes; // clean up 
      }

      wr_destroyState( w );



}
