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

/* Spectral/PAR calibration — the three-tier chain cmd 35 computes.
 * See plans/AMBIT_COMMAND35_SPECPAR.md.
 *
 * Deliberately NOT part of ambit_calibration_info_t. That struct is memcpy'd
 * onto the wire at its full sizeof() by cmd 33 subtype 1, and is mirrored
 * field-for-field in the ambyte's ambit_protocol.h under an explicit "MUST match
 * ambit-1 nvs1.h — omitting it desyncs the framed read" warning. Growing it by
 * 128 bytes would not even fail cleanly: an un-updated ambyte reads its 140
 * bytes correctly and then *scans* for the 0xF0 terminator, so it recovers
 * unless one of the extra bytes happens to be 0xF0 — which the all-zero/1.0f
 * defaults never are and real fitted coefficients often are. A bench test would
 * pass and the field would corrupt later, only on calibrated devices.
 *
 * This struct is read back through a byte-explicit, versioned layout instead
 * (cmd 33 subtype 4), so it can be extended without touching a deployed logger.
 *
 * Units: tier 1 normalises to basic counts with tint in MILLISECONDS
 * (SPEC_TICK_MS), which is the convention every published ams constant uses.
 * The coefficients are meaningless under any other one. */
struct ambit_spec_calibration_t
{
    // ams workbook AS7341_AD000198_3-00, sheet "used Correction Values" row 15,
    // reordered into ambit's channel order. Device-independent, so unlike the
    // rest of this struct these ship as real values rather than placeholders.
    float spec_offset[ambit_calibration::SPEC_CHANNEL_COUNT] = {
        0.00196979f,   // F1  410
        0.00724927f,   // F2  440
        0.00319381f,   // F3  470
        0.001314659f,  // F4  510
        0.001468153f,  // F5  550
        0.001858105f,  // F6  583
        0.001762778f,  // F7  620
        0.00521704f,   // F8  670
        0.001f,        // NIR 900
        0.003f,        // Clear 750
    };
    // 1.0 until measured against an LR1-B on ambit optics. miniPar's vectors do
    // not port — different window and diffuser.
    float spec_sens[ambit_calibration::SPEC_CHANNEL_COUNT] = {
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // 0.0 until fitted against a Li-Cor Li-250A across several source families.
    // All-zero makes the computed PAR collapse to par_intercept, which cmd 35
    // reports as flags bit2 rather than passing off as a reading.
    float par_weight[ambit_calibration::SPEC_CHANNEL_COUNT] = {0.0f};
    // Tier 3, per device, from an intensity sweep. par_slope is spec_coef's
    // successor, not a rename: spec_coef scales a raw-count integer-weighted PAR
    // and stays with cmd 31.
    float par_slope = 1.0f;
    float par_intercept = 0.0f;
};

extern struct ambit_spec_calibration_t ambit_spec_calibration;

/* Is the compiled-in par_weight an ambit fleet fit, or a borrowed seed?
 *
 * Flip to true in the same commit that lands a par_weight fitted against a
 * Li-250A on ambit optics (plans/AMBIT_COMMAND35_SPECPAR.md §11). Until then cmd
 * 35 must not claim tier 2 is calibrated: a seeded vector produces a PAR of the
 * right order that was measured on a different window and diffuser, and the
 * whole point of the flag is that this is not silently indistinguishable from
 * the real thing. */
static constexpr bool AMBIT_PAR_WEIGHT_IS_AMBIT_FIT = false;

/* Has this device been through a tier-3 intensity sweep? Set by
 * load_spec_calibration() from NVS key presence and by the tier-3 setters on a
 * successful write — key presence, not a float comparison against the default,
 * because a fitted slope of exactly 1.0 is legitimate. */
extern bool ambit_spec_tier3_stored;

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
esp_err_t save_spec_offset(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]);
esp_err_t save_spec_sens(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]);
esp_err_t save_par_weight(const float values[ambit_calibration::SPEC_CHANNEL_COUNT]);
esp_err_t save_par_slope(float value);
esp_err_t save_par_intercept(float value);
uint32_t apply_adpd_calibration(uint8_t channel, uint32_t sample);

#endif // _NVS1_H_
