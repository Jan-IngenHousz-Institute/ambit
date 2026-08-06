#include "nvs1.h"
#include "src/mlx90632/u_mlx.h"
#include <Preferences.h>
#include "nvs.h"
#include <cmath>

extern Preferences preferences;
struct ambit_calibration_info_t ambit_calibration_local, ambit_calibration_income;
struct ambit_FW_info_t ambit_FW_info;
struct metadata_t metadata_epprom, metadata_incoming;

static const char *const ADPD_BASELINE_KEYS[ambit_calibration::CHANNEL_COUNT] = {
    "adpd_dark", "adpd_lit", "adpd_sun", "adpd_leaf", "adpd_730", "adpd_730r"
};
static bool adpd_baseline_present[ambit_calibration::CHANNEL_COUNT] = {false};

uint32_t apply_adpd_calibration(uint8_t channel, uint32_t sample){
    const bool present = channel < ambit_calibration::CHANNEL_COUNT
        ? adpd_baseline_present[channel] : false;
    return ambit_calibration::apply_adpd_offset(
        channel, sample, ambit_calibration_local.adpd, present);
}

esp_err_t save_adpd_baseline(const uint32_t values[ambit_calibration::CHANNEL_COUNT]){
    if (values == nullptr) return ESP_ERR_INVALID_ARG;
    if (values[ambit_calibration::S630] > 400U) return ESP_ERR_INVALID_ARG;
    for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        if (values[i] > 0xFFFFFFU) return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        err = nvs_set_u32(handle, ADPD_BASELINE_KEYS[i], values[i]);
        if (err != ESP_OK){
            nvs_close(handle);  // uncommitted writes are discarded
            return err;
        }
    }
    err = nvs_commit(handle);  // one atomic NVS commit for the complete vector
    if (err != ESP_OK){
        nvs_close(handle);
        return err;
    }

    uint32_t readback[ambit_calibration::CHANNEL_COUNT] = {0};
    for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        err = nvs_get_u32(handle, ADPD_BASELINE_KEYS[i], &readback[i]);
        if (err != ESP_OK || readback[i] != values[i]){
            nvs_close(handle);
            return err != ESP_OK ? err : ESP_ERR_INVALID_STATE;
        }
    }
    nvs_close(handle);
    memcpy(ambit_calibration_local.adpd, readback, sizeof(readback));
    memcpy(ambit_calibration_income.adpd, readback, sizeof(readback));
    for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        adpd_baseline_present[i] = true;
    }
    return ESP_OK;
}



static void get_FW_info(void){
    ambit_FW_info.MAC = ESP.getEfuseMac();
    ambit_FW_info.Size = ESP.getSketchSize();
    strncpy(ambit_FW_info.FW_date, __DATE__, 12);
    preferences.begin("config", true);
    ambit_FW_info.hw_rev = preferences.getUChar("hw_rev", 0);
    preferences.end();
    ambit_FW_info.Checksum = 0xFF & ((MAJOR_VERSION << 4) | (MINOR_VERSION << 2) | (BATCH_VERSION << 0));

    return;
}


static void load_metadata(void){
    preferences.begin("metadata", true);
    if (preferences.isKey("lon")) metadata_epprom.lon = preferences.getDouble("lon", 1.0);
    if (preferences.isKey("lat")) metadata_epprom.lat = preferences.getDouble("lat", 1.0);
    if (preferences.isKey("alt")) metadata_epprom.alt = preferences.getFloat("alt", 1.0);
    if (preferences.isKey("time")) metadata_epprom.time = preferences.getUInt("time", 1);
    if (preferences.isKey("acc")) metadata_epprom.acc = preferences.getFloat("acc", 1.0);
    if (preferences.isKey("vacc")) metadata_epprom.vacc = preferences.getFloat("vacc", 1.0);
    if (preferences.isKey("info1")) preferences.getString("info1", metadata_epprom.info1, 200);
    if (preferences.isKey("x")) metadata_epprom.x = preferences.getFloat("x", 0.0);
    if (preferences.isKey("y")) metadata_epprom.y = preferences.getFloat("y", 0.0);
    if (preferences.isKey("z")) metadata_epprom.z = preferences.getFloat("z", 0.0);
    preferences.end();
    memcpy(&metadata_incoming, &metadata_epprom, sizeof(metadata_t));
    return;
}

void save_metadata(void){
    //QC
    if (metadata_incoming.lon < -180.0 || metadata_incoming.lon > 360.0) return;
    if (metadata_incoming.lat < -90.0 || metadata_incoming.lat > 90.0) return;
    if (metadata_incoming.alt < -500.0 || metadata_incoming.alt > 30000.0) return;
    // Save metadata to NVS
    preferences.begin("metadata", false);
    preferences.putDouble("lon", metadata_incoming.lon);
    preferences.putDouble("lat", metadata_incoming.lat);
    preferences.putFloat("alt", metadata_incoming.alt);
    preferences.putUInt("time", metadata_incoming.time);
    preferences.putFloat("acc", metadata_incoming.acc);
    preferences.putFloat("vacc", metadata_incoming.vacc);
    preferences.putString("info1", metadata_incoming.info1);
    preferences.putFloat("x", metadata_incoming.x);
    preferences.putFloat("y", metadata_incoming.y);
    preferences.putFloat("z", metadata_incoming.z);
    preferences.end();
    return;
}


static void load_calibration_info(){
    preferences.begin("config", true);
    for (uint8_t i = 0; i < ambit_calibration::CHANNEL_COUNT; ++i){
        adpd_baseline_present[i] = preferences.isKey(ADPD_BASELINE_KEYS[i]);
    }
    if (preferences.isKey("actinic")){
        const float value = preferences.getFloat("actinic", 0.1);
        if (std::isfinite(value) && value > 0.01f && value <= 1.0f) ambit_calibration_local.actinic_coef = value;
    }
    if (preferences.isKey("spec")){
        const float value = preferences.getFloat("spec", 1.0);
        if (std::isfinite(value) && value >= 0.05f && value <= 100.0f) ambit_calibration_local.spec_coef = value;
    }
    if (preferences.isKey("emit")) ambit_calibration_local.mlx_emissivity = preferences.getDouble("emit", 1.0);
    if (preferences.isKey("sun")) ambit_calibration_local.sun_coef = preferences.getFloat("sun", 1.0);
    if (preferences.isKey("name")) preferences.getString("name", ambit_calibration_local.ambit_name, 20);
    if (preferences.isKey("temp_offset")) ambit_calibration_local.temp_offset = preferences.getFloat("temp_offset", 0.0);
    if (preferences.isKey("temp_slope")) ambit_calibration_local.temp_slope = preferences.getFloat("temp_slope", 1.0);

    if (preferences.isKey("act_50")) ambit_calibration_local.act_50 = preferences.getUShort("act_50", 1);
    if (preferences.isKey("act_100")) ambit_calibration_local.act_100 = preferences.getUShort("act_100", 2);
    if (preferences.isKey("act_150")) ambit_calibration_local.act_150 = preferences.getUShort("act_150", 3);
    if (preferences.isKey("act_200")) ambit_calibration_local.act_200 = preferences.getUShort("act_200", 4);
    if (preferences.isKey("act_250")) ambit_calibration_local.act_250 = preferences.getUShort("act_250", 5);

    if (preferences.isKey("adpd_lit")) ambit_calibration_local.adpd[1] = preferences.getUInt("adpd_lit", 0);
    if (preferences.isKey("adpd_dark")) ambit_calibration_local.adpd[0] = preferences.getUInt("adpd_dark", 0);
    if (preferences.isKey("adpd_sun")) ambit_calibration_local.adpd[2] = preferences.getUInt("adpd_sun", 0);
    if (preferences.isKey("adpd_leaf")) ambit_calibration_local.adpd[3] = preferences.getUInt("adpd_leaf", 0);
    if (preferences.isKey("adpd_730")) ambit_calibration_local.adpd[4] = preferences.getUInt("adpd_730", 0);
    if (preferences.isKey("adpd_730r")) ambit_calibration_local.adpd[5] = preferences.getUInt("adpd_730r", 0);
    preferences.end();
    mlx_read_coe(ambit_calibration_local.mlx_coef);
    memcpy(&ambit_calibration_income, &ambit_calibration_local, sizeof(ambit_calibration_info_t));
}





void load_info_from_nvs(bool print){
    load_metadata();
    if (print){
        Serial.printf("Metadata: lon:%f\tlat:%f\talt:%f\ttime:%d\tacc:%f\tvacc:%f\tinfo1:%s\tx:%f\ty:%f\tz:%f\n", metadata_epprom.lon, metadata_epprom.lat, metadata_epprom.alt, metadata_epprom.time, metadata_epprom.acc, metadata_epprom.vacc, metadata_epprom.info1, metadata_epprom.x, metadata_epprom.y, metadata_epprom.z);
    }
    load_calibration_info();
    if (print){
        // print all ambit_calibration_local
        Serial.printf("Calibration: Name:%s\tActinic:%f\tSpec:%f\tEmit:%f\tSun:%f\tTemp_offset:%f\tTemp_slope:%f\n", ambit_calibration_local.ambit_name, ambit_calibration_local.actinic_coef, ambit_calibration_local.spec_coef, ambit_calibration_local.mlx_emissivity, ambit_calibration_local.sun_coef, ambit_calibration_local.temp_offset, ambit_calibration_local.temp_slope);
        Serial.printf("Calibration: Act_50:%d\tAct_100:%d\tAct_150:%d\tAct_200:%d\tAct_250:%d\n", ambit_calibration_local.act_50, ambit_calibration_local.act_100, ambit_calibration_local.act_150, ambit_calibration_local.act_200, ambit_calibration_local.act_250);
        Serial.printf("Calibration: ADPD: %d\t%d\t%d\t%d\t%d\t%d\n", ambit_calibration_local.adpd[0], ambit_calibration_local.adpd[1], ambit_calibration_local.adpd[2], ambit_calibration_local.adpd[3], ambit_calibration_local.adpd[4], ambit_calibration_local.adpd[5]);
      
        Serial.printf("MLX: ");
        for (int i = 0; i < 14; i++){
            Serial.printf("%d\t", ambit_calibration_local.mlx_coef[i]);
        }
        Serial.printf("\n");
    }
    get_FW_info();
    if (print){
        Serial.printf("FW: MAC:%012llx\tSize:%d\tDate:%s\n", ambit_FW_info.MAC, ambit_FW_info.Size, ambit_FW_info.FW_date);
        Serial.printf("FW: %d.%d.%d\n", ambit_FW_info.Major, ambit_FW_info.Minor, ambit_FW_info.Batch);
    }
    return;
}
