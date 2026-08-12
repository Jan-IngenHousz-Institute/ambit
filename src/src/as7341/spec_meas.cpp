#include "spec_meas.h"
#include "Adafruit_AS7341.h"
static const char* TAG = "SPEC";


Adafruit_AS7341 as7341;


bool as7341_setup = false;

bool check_AS7341(){
    if (as7341.begin()) {
        as7341_setup = true;
        return true;
    }
    else{
        ESP_LOGE(TAG, "AS7341 init failed");
        as7341_setup = false;
        return false;
    }
}

void AS_LED_ON(){
    as7341.enableLED(true);
}

void AS_LED_OFF(){
    as7341.enableLED(false);
}

void AS_LED_Current(uint16_t current){
    as7341.setLEDCurrent(current);  // set actinic LED current
    if (current == 0) as7341.enableLED(false);
}

	

void AS_all_channel(uint16_t T1, uint16_t T2, uint16_t *spec){
    if (!as7341_setup){
         check_AS7341();
    }
    //(T1 + 1) * (T2 + 1) * 2.78 / 1000

    as7341.setATIME(T1);
    as7341.setASTEP(T2);
    as7341.setGain(AS7341_GAIN_2X);

    if (!as7341.readAllChannels(spec)){
        Serial.println("Error reading all channels!");
    }    
}

void AS_all_channel(uint16_t T1, uint16_t T2){
    uint16_t spec[12];
    AS_all_channel(T1, T2, spec);
    for (uint8_t i = 0; i < 11; i++){
        Serial.print(spec[i]);
        Serial.print(",");
    }
    Serial.print(spec[11]);
    Serial.println("");
}

// double get_PAR(uint16_t *spec){
//     if (!as7341_setup){
//         check_AS7341();
//     }    
//     double par = 0;
    
//     as7341.setATIME(29);
//     as7341.setASTEP(499);
//     as7341.setGain(AS7341_GAIN_2X);

//     if (!as7341.readAllChannels(spec)){
//         Serial.println("Error reading all channels!");
//         return 0;
//     }
    
//     par = spec[0] + spec[1] + spec[2] + spec[3] +  spec[6] + spec[7] + spec[8] + spec[9]/2;
//     if (par < 100){
//         as7341.setGain(AS7341_GAIN_256X);
//         if (!as7341.readAllChannels(spec)){
//             Serial.println("Error reading all channels!");
//             return 0;
//         }
//         par = par * 128;
//         par = (par + spec[0] + spec[1] + spec[2] + spec[3] +  spec[6] + spec[7] + spec[8] + spec[9]/2)/2;
//     }else{
//         par = par * 128;
//     }
//     return par/36;
// }

// double raw_PAR(uint16_t *spec, as7341_gain_t gain){
//     if (!as7341_setup){
//         check_AS7341();
//     }    
//     double par = 0;

//     as7341.setATIME(29);
//     as7341.setASTEP(499);
//     as7341.setGain(gain);

//     if (!as7341.readAllChannels(spec)){
//         Serial.println("Error reading all channels!");
//         return 0;
//     }

//     uint16_t ir = (spec[5] + spec[11])/16;
//     uint16_t clear = (spec[4] + spec[10])/8;
    
//     par = spec[0] * PAR_COE1 + spec[1] * PAR_COE2 + spec[2] * PAR_COE3 + spec[3] * PAR_COE4 +  spec[6] * PAR_COE5 + spec[7] * PAR_COE6 + \
//             spec[8] * PAR_COE7 + spec[9] * PAR_COE8 + clear * PAR_COE9 + ir * PAR_COE10;
            
//     return par;
// }


// double get_PAR(uint16_t *spec){
//     double par = raw_PAR(spec, AS7341_GAIN_8X);
//     if (par < 200){
//         par = raw_PAR(spec, AS7341_GAIN_256X)/32;
//         for (uint8_t n = 0; n < 12; n++) spec[n] = spec[n]/32;
//     }
//     return par * PAR_OFFSET;    
// }

// double get_PAR(){
//     uint16_t spec[12];
//     return get_PAR(spec);    
// }

void dual_exposure(as7341_gain_t gain1,as7341_gain_t gain2, uint16_t readings_buffer[]){
    if (!as7341_setup){
        check_AS7341();
    }
    

    as7341.setATIME(SPEC_ATIME);
    as7341.setASTEP(SPEC_ASTEP);
    as7341.setGain(gain1);

    as7341.setSMUXLowChannels(true);        // Configure SMUX to read low channels
    as7341.enableSpectralMeasurement(true); // Start integration
    as7341.delayForData(0);                 // I'll wait for you for all time

    Adafruit_BusIO_Register channel_data_reg =
      Adafruit_BusIO_Register(as7341.i2c_dev, AS7341_CH0_DATA_L, 2);
    bool low_success = channel_data_reg.read((uint8_t *)readings_buffer, 12);

    as7341.setGain(gain2);
    as7341.setSMUXLowChannels(false);        // Configure SMUX to read low channels
    as7341.enableSpectralMeasurement(true); // Start integration
    as7341.delayForData(0);                 // I'll wait for you for all time

     low_success = channel_data_reg.read((uint8_t *)&readings_buffer[6], 12);
}

void cali_PAR(){
    if (!as7341_setup){
        check_AS7341();
    }
    uint16_t spec[12];
    uint16_t calc_spec[10];

    dual_exposure(AS7341_GAIN_8X, AS7341_GAIN_8X, spec);

    calc_spec[0] = spec[0] * Spec_COE1;
    calc_spec[1] = spec[1] * Spec_COE2;
    calc_spec[2] = spec[2] * Spec_COE3;
    calc_spec[3] = spec[3] * Spec_COE4;
    calc_spec[4] = spec[6] * Spec_COE5;
    calc_spec[5] = spec[7] * Spec_COE6;
    calc_spec[6] = spec[8] * Spec_COE7;
    calc_spec[7] = spec[9] * Spec_COE8;
    calc_spec[8] = spec[11] * Spec_COE9;
    calc_spec[9] = spec[10];

    for (uint8_t n = 0; n < 10; n++){
        Serial.print(calc_spec[n]);
        Serial.print(",");
    }

    Serial.println();

    return;
}

/* dual_exposure() returns the two SMUX banks back to back: the low bank lands in
 * spec[0..5] (F1-F4, Clear, NIR) and the high bank in spec[6..11] (F5-F8, Clear,
 * NIR). Every frontend reports the ten channels in wavelength order, so map once
 * here instead of restating the index arithmetic at each call site. */
static const uint8_t SPEC_RAW_INDEX[10] = {0, 1, 2, 3, 6, 7, 8, 9, 11, 10};

/* Per-channel response weights. calc_spec[9] (Clear) has always been reported
 * unweighted; it is spelled 1 here rather than left implicit. */
static const uint8_t SPEC_WEIGHT[10] = {
    Spec_COE1, Spec_COE2, Spec_COE3, Spec_COE4, Spec_COE5,
    Spec_COE6, Spec_COE7, Spec_COE8, Spec_COE9, 1};

/* Unscaled acquisition. This is where PAR is actually computed, deliberately
 * from the raw counts in a uint32 accumulator: the weighted sum peaks at
 * 50000 * 73 = 3.65e6, which the old float accumulator handled fine but the
 * uint16 calc_spec[] it summed did not — a wrapped channel used to drag PAR
 * down with it. Sum, coefficients and operation order are otherwise unchanged,
 * so PAR is bit-identical to the pre-fix value whenever nothing overflowed. */
double get_PAR_raw(spec_raw_t *out){
    if (!as7341_setup){
        check_AS7341();
    }
    uint16_t spec[12];
    float calc_par = 0;

    dual_exposure(AS7341_GAIN_2X, AS7341_GAIN_2X, spec);

    out->atime = SPEC_ATIME;
    out->astep = SPEC_ASTEP;
    out->gain_low = AS7341_GAIN_2X;
    out->gain_high = AS7341_GAIN_2X;
    out->saturated = false;

    uint32_t weighted_sum = 0;
    uint32_t nir_weighted = 0;

    for (uint8_t n = 0; n < 10; n++){
        const uint16_t counts = spec[SPEC_RAW_INDEX[n]];
        out->raw[n] = counts;
        if (counts >= SPEC_FULL_SCALE) out->saturated = true;
        if (n < 8) weighted_sum += (uint32_t) counts * SPEC_WEIGHT[n];
        else if (n == 8) nir_weighted = (uint32_t) counts * SPEC_WEIGHT[n];
    }

    calc_par = weighted_sum;
    calc_par = calc_par * 0.006 - nir_weighted * 0.0075;
    return calc_par * PAR_OFFSET;
}

/* Scaled view for the frozen 16-bit channel words (binary cmd 31, text PAR /
 * get_par, JSON par / par_raw). counts * Spec_COE* exceeds 16 bits from ~11% of
 * ADC full scale, and the old assignment let it wrap silently to near zero — a
 * bright-light reading looked like a dark one. Saturate instead: a pegged 65535
 * is monotonic and visibly at the rail, and it cannot be mistaken for darkness.
 * The returned PAR comes from the unclamped counts, so it stays correct past the
 * point where these words peg. Hosts wanting unclamped data use cmd 35. */
double get_PAR(uint16_t *calc_spec){
    spec_raw_t raw;
    const double par = get_PAR_raw(&raw);

    for (uint8_t n = 0; n < 10; n++){
        const uint32_t scaled = (uint32_t) raw.raw[n] * SPEC_WEIGHT[n];
        calc_spec[n] = (scaled > 0xFFFFU) ? 0xFFFFU : (uint16_t) scaled;
    }
    return par;
}


double get_PAR(){
    uint16_t spec[10];
    return get_PAR(spec);
}



uint8_t as7431_reg_write(uint8_t reg, uint8_t data){
    Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
}


uint8_t as7431_reg_read(uint8_t reg, uint8_t* data, uint16_t len){
    Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)AS7341_I2CADDR_DEFAULT, (uint8_t)len, (uint8_t)0);
    for (uint16_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return 0;
}


void as7431_blink(uint8_t n, uint8_t intensity){

    while (Serial.available()) Serial.print(Serial.read());

    uint8_t reg = intensity >> 1;
    if (intensity < 4) reg = 1;
    reg |= 0b10000000;

    as7431_reg_write(0xA9, 0b00010000);
    as7431_reg_write(0x70, 0b00001000);
    as7431_reg_write(0x74, 0x00);



    unsigned long timer = millis();
    uint8_t a,b;
    a = 18 - n * 2;
    b = 30 - a;

    while(millis() - timer < 60000){
        for (uint8_t z = 0; z < 128; z++){
            as7431_reg_write(0x74, reg);
            delayMicroseconds(a * 1024);
            as7431_reg_write(0x74, 0x00);
            delayMicroseconds(b * 1024);
        }
        if (Serial.available() > 1) break;
    }
    as7431_reg_write(0x74, 0x00);
    as7431_reg_write(0xA9, 0x0);
    
    return;   
}