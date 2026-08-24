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

struct ambit_spec_calibration_t ambit_spec_calibration;
bool ambit_spec_tier3_stored = false;

// NVS keys are capped at 15 characters.
static const char *const SPEC_OFFSET_KEY = "spec_off";
static const char *const SPEC_SENS_KEY = "spec_sens";
static const char *const PAR_WEIGHT_KEY = "par_w";
static const char *const PAR_SLOPE_KEY = "par_slope";
static const char *const PAR_INTERCEPT_KEY = "par_icept";

// Largest blob this module reads or writes: one ten-element float vector.
// Bounds the readback buffer in save_calibration_blob() so it can stay on stack.
#define SPEC_CALIBRATION_VECTOR_BYTES \
    (sizeof(float) * ambit_calibration::SPEC_CHANNEL_COUNT)
#define SPEC_CALIBRATION_BLOB_MAX SPEC_CALIBRATION_VECTOR_BYTES

uint32_t apply_adpd_calibration(uint8_t channel, uint32_t sample){
    const bool present = channel < ambit_calibration::CHANNEL_COUNT
        ? adpd_baseline_present[channel] : false;
    return ambit_calibration::apply_adpd_offset(
        channel, sample, ambit_calibration_local.adpd, present);
}

esp_err_t save_adpd_baseline(const uint32_t values[ambit_calibration::CHANNEL_COUNT]){
    if (!ambit_calibration::valid_adpd_baseline(values)) return ESP_ERR_INVALID_ARG;

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

// Generalised from the scalar version: same commit-then-verify contract, any
// length. Preferences::putFloat stores a four-byte NVS blob, so a single float
// written through here stays byte-compatible with existing installations.
// A vector is one blob rather than N keys, which makes it strictly more atomic
// than save_adpd_baseline()'s six-key loop — nvs_set_blob + nvs_commit is
// all-or-nothing by construction.
static esp_err_t save_calibration_blob(const char *key, const void *data, size_t len){
    if (key == nullptr || data == nullptr || len == 0) return ESP_ERR_INVALID_ARG;
    if (len > SPEC_CALIBRATION_BLOB_MAX) return ESP_ERR_INVALID_SIZE;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, key, data, len);
    if (err == ESP_OK) err = nvs_commit(handle);

    uint8_t readback[SPEC_CALIBRATION_BLOB_MAX] = {0};
    size_t readback_size = len;
    if (err == ESP_OK){
        err = nvs_get_blob(handle, key, readback, &readback_size);
        if (err == ESP_OK &&
            (readback_size != len || memcmp(readback, data, len) != 0)){
            err = ESP_ERR_INVALID_STATE;
        }
    }
    nvs_close(handle);
    return err;
}

static esp_err_t save_calibration_float(const char *key, float value){
    return save_calibration_blob(key, &value, sizeof(value));
}

esp_err_t save_actinic_coefficient(float value){
    if (!ambit_calibration::valid_actinic_coefficient(value)) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = save_calibration_float("actinic", value);
    if (err == ESP_OK) ambit_calibration_local.actinic_coef = value;
    return err;
}

esp_err_t save_spec_coefficient(float value){
    if (!ambit_calibration::valid_spec_coefficient(value)) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = save_calibration_float("spec", value);
    if (err == ESP_OK) ambit_calibration_local.spec_coef = value;
    return err;
}

/* Spectral/PAR calibration setters. Validation lives here rather than in the
 * callers, following save_adpd_baseline(): one call site covers the text
 * console, any future binary subtype, and anything else that ever writes these,
 * which is what calibration_math.h's shared-predicate rule asks for. The runtime
 * copy moves only after NVS has confirmed the write. */
static esp_err_t save_spec_vector(const char *key, const float *values, float *destination){
    const esp_err_t err = save_calibration_blob(key, values, SPEC_CALIBRATION_VECTOR_BYTES);
    if (err == ESP_OK) memcpy(destination, values, SPEC_CALIBRATION_VECTOR_BYTES);
    return err;
}

esp_err_t save_spec_offset(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]){
    if (!ambit_calibration::valid_spec_offset(values)) return ESP_ERR_INVALID_ARG;
    return save_spec_vector(SPEC_OFFSET_KEY, values, ambit_spec_calibration.spec_offset);
}

esp_err_t save_spec_sens(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]){
    if (!ambit_calibration::valid_spec_sens(values)) return ESP_ERR_INVALID_ARG;
    return save_spec_vector(SPEC_SENS_KEY, values, ambit_spec_calibration.spec_sens);
}

esp_err_t save_par_weight(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]){
    if (!ambit_calibration::valid_par_weight(values)) return ESP_ERR_INVALID_ARG;
    return save_spec_vector(PAR_WEIGHT_KEY, values, ambit_spec_calibration.par_weight);
}

esp_err_t save_par_slope(float value){
    if (!ambit_calibration::valid_par_slope(value)) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = save_calibration_float(PAR_SLOPE_KEY, value);
    if (err == ESP_OK){
        ambit_spec_calibration.par_slope = value;
        ambit_spec_tier3_stored = true;   // cmd 35 flags bit9
    }
    return err;
}

esp_err_t save_par_intercept(float value){
    if (!ambit_calibration::valid_par_intercept(value)) return ESP_ERR_INVALID_ARG;
    const esp_err_t err = save_calibration_float(PAR_INTERCEPT_KEY, value);
    if (err == ESP_OK){
        ambit_spec_calibration.par_intercept = value;
        ambit_spec_tier3_stored = true;   // cmd 35 flags bit9
    }
    return err;
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
        if (ambit_calibration::valid_actinic_coefficient(value)) ambit_calibration_local.actinic_coef = value;
    }
    if (preferences.isKey("spec")){
        const float value = preferences.getFloat("spec", 1.0);
        if (ambit_calibration::valid_spec_coefficient(value)) ambit_calibration_local.spec_coef = value;
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


/* Read one ten-element float vector, adopting it only if it is both the right
 * size and valid.
 *
 * The size check is the part that is easy to skip and expensive to debug:
 * Preferences::getBytes copies a SHORTER-than-expected stored blob and returns
 * the short length, leaving the tail of the destination untouched. Reading
 * straight into the live struct would silently blend stored elements with
 * compiled-in defaults, with nothing afterwards able to tell which is which. So
 * read into scratch, check the returned length, run the predicate, then adopt.
 * (getBytes already calls getBytesLength internally and returns the real count,
 * so comparing its return value is the single-call form of that check.)
 *
 * Re-validating on load is calibration_math.h's rule: a value must not be
 * accepted over one frontend and rejected after a reboot. Note the adpd
 * baselines in load_calibration_info() above are the standing exception —
 * validated on save but read back unchecked — which is a bug to fix, not a
 * pattern to copy. */
static void load_spec_vector(const char *key, float *destination,
                             bool (*predicate)(const float *)){
    if (!preferences.isKey(key)) return;

    float scratch[ambit_calibration::SPEC_CHANNEL_COUNT] = {0.0f};
    if (preferences.getBytes(key, scratch, sizeof(scratch)) != sizeof(scratch)) return;
    if (!predicate(scratch)) return;

    memcpy(destination, scratch, sizeof(scratch));
}

static void load_spec_calibration(){
    preferences.begin("config", true);

    load_spec_vector(SPEC_OFFSET_KEY, ambit_spec_calibration.spec_offset,
                     ambit_calibration::valid_spec_offset);
    load_spec_vector(SPEC_SENS_KEY, ambit_spec_calibration.spec_sens,
                     ambit_calibration::valid_spec_sens);
    load_spec_vector(PAR_WEIGHT_KEY, ambit_spec_calibration.par_weight,
                     ambit_calibration::valid_par_weight);

    /* Tier 3 counts as stored if either key was written and survived validation.
     * Either alone is a legitimate calibration — a sweep through the origin has
     * no intercept to store — so this is an OR, not an AND. */
    ambit_spec_tier3_stored = false;
    if (preferences.isKey(PAR_SLOPE_KEY)){
        const float value = preferences.getFloat(PAR_SLOPE_KEY, 1.0f);
        if (ambit_calibration::valid_par_slope(value)){
            ambit_spec_calibration.par_slope = value;
            ambit_spec_tier3_stored = true;
        }
    }
    if (preferences.isKey(PAR_INTERCEPT_KEY)){
        const float value = preferences.getFloat(PAR_INTERCEPT_KEY, 0.0f);
        if (ambit_calibration::valid_par_intercept(value)){
            ambit_spec_calibration.par_intercept = value;
            ambit_spec_tier3_stored = true;
        }
    }

    preferences.end();
}





void load_info_from_nvs(bool print){
    load_metadata();
    if (print){
        Serial.printf("Metadata: lon:%f\tlat:%f\talt:%f\ttime:%d\tacc:%f\tvacc:%f\tinfo1:%s\tx:%f\ty:%f\tz:%f\n", metadata_epprom.lon, metadata_epprom.lat, metadata_epprom.alt, metadata_epprom.time, metadata_epprom.acc, metadata_epprom.vacc, metadata_epprom.info1, metadata_epprom.x, metadata_epprom.y, metadata_epprom.z);
    }
    load_calibration_info();
    load_spec_calibration();
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
