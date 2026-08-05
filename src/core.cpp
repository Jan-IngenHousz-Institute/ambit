#include "core.h"
#include "PAM.h"

void core_set_currents(uint8_t i620, uint8_t i720, uint8_t ir){
    adpd_current_config.I620 = i620;
    adpd_current_config.I720 = i720;
    adpd_current_config.IR   = ir;
    adpd_current_config.init = true;
    adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;   // dirty: next array run re-applies
}

void core_set_gains(uint8_t fluo, uint8_t fluo_ref, uint8_t ir, uint8_t ir_ref,
                    uint8_t sun, uint8_t leaf){
    adpd_gains_config.Fluo    = fluo;
    adpd_gains_config.FluoRef = fluo_ref;
    adpd_gains_config.IR      = ir;
    adpd_gains_config.IRRef   = ir_ref;
    adpd_gains_config.Sun     = sun;
    adpd_gains_config.Leaf    = leaf;
    adpd_gains_config.init = true;
    adpd_mode = ADPD_CONFIG_MODE::MPF_MODE;   // dirty
}

void core_config_detector(void){
    conf_slow_FR_1();                         // reads the global config
    adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
}

void core_ensure_array_config(void){
    if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1){
        conf_slow_FR_1();
        adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
    }
}

int core_run_array(uint8_t len, uint8_t* arr, uint8_t persist, bool allow_interrupt){
    core_ensure_array_config();
    return run_arr_type1(len, arr, persist, allow_interrupt);
}

int core_run_mpf(uint16_t length, uint8_t interval, bool change_act, uint8_t act){
    return run_trigger_spacer(length, interval, change_act, act, true);
}
