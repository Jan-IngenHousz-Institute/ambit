#ifndef _NVS1_H_
#define _NVS1_H_
#include <Arduino.h>
#include "nvs_flash.h"
#include "calibration_math.h"

// Version is build-injected by tools/version.py (semantic-release tag in CI,
// git describe locally). These fallbacks only apply if the pre-script did not
// run. A post-OTA cmd 33/2 version read confirms an update landed.
#ifndef AMBIT_FW_VERSION
#define AMBIT_FW_VERSION "0.0.0-dev"
#endif
#ifndef AMBIT_FW_MAJOR
#define AMBIT_FW_MAJOR 0
#endif
#ifndef AMBIT_FW_MINOR
#define AMBIT_FW_MINOR 0
#endif
#ifndef AMBIT_FW_BATCH
#define AMBIT_FW_BATCH 0
#endif
// Legacy names, kept because the frozen cmd 33/2 wire struct fields use them.
#define MAJOR_VERSION AMBIT_FW_MAJOR
#define MINOR_VERSION AMBIT_FW_MINOR
#define BATCH_VERSION AMBIT_FW_BATCH

struct ambit_calibration_info_t
{
    char ambit_name[20] = "AmbitV003";
    int32_t mlx_coef[14] = {0};
    uint32_t adpd[6] = {0};
    float_t temp_offset = 0.0;
    float_t temp_slope = 1.0;
    float_t actinic_coef = 0.1;
    float_t spec_coef = 1.0;
    uint16_t act_50 = 5;
    uint16_t act_100 = 4;
    uint16_t act_150 = 3;
    uint16_t act_200 = 2;
    uint16_t act_250 = 1;
    float_t mlx_emissivity = 1.0;
    float_t sun_coef = 1.0;
    float_t tick_factor = 0.854;  // PAM point-period scale (ms tick -> s); surfaced to the ambyte
};

extern struct ambit_calibration_info_t ambit_calibration_local, ambit_calibration_income;

struct ambit_FW_info_t
{
    uint8_t Major = MAJOR_VERSION;
    uint8_t Minor = MINOR_VERSION;
    uint8_t Batch = BATCH_VERSION;
    uint32_t Size = 0;
    uint64_t MAC = 0;
    char FW_date[12];
    // hw_rev occupies the first byte of what was reserved[12]: same offsets, same
    // 48-byte total, so the frozen cmd 33/2 wire layout is unchanged (old readers
    // never looked at these bytes). 0 = unknown; NVS "config"/"hw_rev" once boards
    // start being programmed with a hardware revision.
    uint8_t hw_rev = 0;
    char reserved[11];
    uint8_t Checksum = 0;
};

extern struct ambit_FW_info_t ambit_FW_info;

struct metadata_t
{
    double lon = 1.0;
    double lat = 1.0;
    float alt = 1.0;
    float acc = 1.0;
    float vacc = 1.0;
    uint32_t time = 0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    char info1[200] = "New_Ambit";
    uint16_t EOF_MARK = 2025; // end of file marker, used to check if the metadata is valid
};

extern struct metadata_t metadata_epprom, metadata_incoming;

void load_info_from_nvs(bool print);
void save_metadata(void);
esp_err_t save_adpd_baseline(const uint32_t values[ambit_calibration::CHANNEL_COUNT]);
esp_err_t save_actinic_coefficient(float value);
esp_err_t save_spec_coefficient(float value);
uint32_t apply_adpd_calibration(uint8_t channel, uint32_t sample);

#endif // _NVS1_H_
