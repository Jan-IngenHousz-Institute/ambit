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
/* CHANNEL ORDER, for every vector below: F1..F8, NIR, Clear — ambit's reported
 * order, set by SPEC_RAW_INDEX. miniPar's tooling (both notebooks and
 * as7341_calibrate.py) ends "..., clear, nir", so every vector imported from
 * there has ALREADY had its last two elements swapped here. Getting this wrong
 * does not fail loudly: NIR and Clear carry the most dissimilar coefficients in
 * each vector (5.78 vs 31.57, -37.7 vs +16.9), so a missed swap just produces a
 * confidently wrong answer under any spectrum with NIR content. */
struct ambit_spec_calibration_t
{
    // ams workbook AS7341_AD000198_3-00, sheet "used Correction Values" row 15.
    // A property of the silicon, not of the optics, so this one is a real
    // calibration rather than a seed.
    float spec_offset[ambit_calibration::SPEC_CHANNEL_COUNT] = {
        0.00196979f,   // F1  415
        0.00724927f,   // F2  445
        0.00319381f,   // F3  480
        0.001314659f,  // F4  515
        0.001468153f,  // F5  555
        0.001858105f,  // F6  590
        0.001762778f,  // F7  630
        0.00521704f,   // F8  680
        0.001f,        // NIR 910
        0.003f,        // Clear (broadband)
    };

    /* SEED, not an ambit calibration. From the miniPar LR1-B campaign
     * (Scripts/spectralCalibration_miniPAR_LR1-B.ipynb): median per-channel
     * reference/sensor ratio over 3 devices, where reference is the LR1-B
     * irradiance projected onto the channels through the ams reconstruction
     * matrix. Per-channel device spread 1.4-9.7%, which is what justifies one
     * fleet vector rather than a per-serial lookup.
     *
     * Ports exactly: miniPar derived it under the same chain computed here —
     * basic counts on the ms tick, ams offsets subtracted, then scale — so
     * s[i] * spec_sens[i] is the expression that produced the constant. What
     * does NOT port is ambit's window and diffuser; replace after an LR1-B
     * session on ambit optics. */
    float spec_sens[ambit_calibration::SPEC_CHANNEL_COUNT] = {
        34.950663f,    // F1  415
        65.289484f,    // F2  445
        72.697997f,    // F3  480
        63.273264f,    // F4  515
        56.737110f,    // F5  555
        52.958660f,    // F6  590
        48.706781f,    // F7  630
        42.671670f,    // F8  680
         5.781618f,    // NIR 910
        31.565986f,    // Clear (broadband)
    };

    /* SEED, not an ambit calibration. From the miniPar Li-250A campaign
     * (Scripts/regression_PAR_miniPAR.ipynb): OLS of measured PAR on basic
     * counts, all 462 samples over 6 devices, tier-2 intercept discarded because
     * composing the tiers gives PAR = a*(w.x) + (a*b0 + b), so tier 3's intercept
     * absorbs it. Leave-one-device-out median 4.4%, worst device 9.8%.
     *
     * Two things to know before trusting these numbers:
     *
     * (1) The signs are not physical. F3, F5 and F8 are negative and Clear is
     *     positive, which is backwards — the visible channels should lift PAR and
     *     Clear/NIR should subtract stray light. That is collinearity, not
     *     physics: condition number ~451 on a daylight-dominated set, so OLS
     *     spreads weight arbitrarily across correlated channels. Refitting on 80%
     *     of the samples moves F1 by 15% and F3 by a factor of 3 while R^2 barely
     *     moves. Good in-domain predictor; NOT a spectral response curve, and not
     *     safe to extrapolate to spectra unlike daylight.
     *
     * (2) w was fitted against basic counts with NO dark offset subtracted,
     *     while this chain computes s = max(0, x - spec_offset). The difference
     *     is sum(w*offset) = 2.40 umol m-2 s-1, a CONSTANT, so par_intercept
     *     absorbs it exactly — verified by running this chain over miniPar's data
     *     (R^2 0.9983) and refitting tier 3, which returns a = 1.0000 and an
     *     intercept holding precisely that constant plus miniPar's discarded b0.
     *     Nothing needs rescaling here.
     *
     * Replace after a Li-250A campaign on ambit hardware, using a constrained or
     * shrunk fit rather than plain OLS, then set AMBIT_PAR_WEIGHT_IS_AMBIT_FIT. */
    float par_weight[ambit_calibration::SPEC_CHANNEL_COUNT] = {
         333.463542f,  // F1  415
         206.427134f,  // F2  445
         -30.6130744f, // F3  480
         283.778061f,  // F4  515
        -144.07319f,   // F5  555
          73.852848f,  // F6  590
          38.9285016f, // F7  630
          -6.43585093f,// F8  680
         -37.6580353f, // NIR 910
          16.9097651f, // Clear (broadband)
    };

    /* Tier 3, per device, from an intensity sweep against a Li-250A. Deliberately
     * NOT seeded from miniPar: this is the term that carries optical throughput,
     * and ambit's window and diffuser are not miniPar's. Until a sweep is run,
     * par reports par_tier2 unscaled — it tracks light correctly but its
     * MAGNITUDE is not meaningful, which is what flags bit9 exists to say.
     *
     * par_slope is spec_coef's successor, not a rename: spec_coef scales a
     * raw-count integer-weighted PAR and stays with cmd 31. */
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
