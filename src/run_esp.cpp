#include <Arduino.h>
#include "data_utils.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "PAM.h"
#include "core.h"
#include <Preferences.h>
#include "nvs1.h"
#include <Update.h>
#include "esp_ota_ops.h"

#define TAG "ESP"
#define ESP_CMD_HEADER 160
#define ESP_CMD_DONE 161
#define ESP_CMD_END 240
#define ESP_WAKE_FOR_CMD 170
#define MAX_ARR_LEN 16





// Config is the single global owned by PAM (adpd_current_config / adpd_gains_config).
// The binary frontend reads/writes the SAME struct as the text frontend, so a value
// set over one transport is applied by a run triggered over the other.
extern Preferences preferences;

extern uint8_t CONNECTION_TYPE;
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);

// ── OTA-over-UART receiver state (cmds 25-28) ───────────────────────────────
// The ambyte streams a new firmware image in CRC16-checked, sequenced chunks; we
// write each straight into the spare OTA partition via Arduino's Update, then
// reboot into it on OTA_END. There is no HW watchdog here, so multi-ms flash
// writes are safe; USB-Serial-JTAG (GPIO18/19) is the recovery path for a bad
// image. Update verifies the whole image (MD5/size) in end() before switching.
static uint16_t ota_expected_seq = 0;

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* Rollback safety: defer OTA image confirmation. The Arduino core auto-marks a
 * freshly-OTA'd PENDING_VERIFY image valid at boot unless this weak hook returns
 * true. We return true so a new image stays UNCONFIRMED until the ambyte sends
 * OTA_CONFIRM (cmd 29) — i.e. until the ambyte has seen the rebooted image
 * answer. If it never confirms (bad/unreachable image), the bootloader
 * (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) rolls back to the previous image on
 * the next reboot. A USB-flashed image is VALID (not pending), so unaffected. */
extern "C" bool verifyRollbackLater() { return true; }

// extern char ambit_name[];
// float_t actinic_coef = 1.0;
// float_t spec_coef = 1.0;
// extern double mlx_emissivity;


int do_esp_cmd(){
    uint8_t cmd_arr[8], c, ret;
    ESP_LOGV(TAG, "parse ESP commands");

    //search for cmd header
    ret = serial_read_until(ESP_CMD_HEADER, 0, 0, 100, true);
    if (ret != 1){
        ESP_LOGE(TAG, "ESP cmd header failed");
        return -1;
    }

    CONNECTION_TYPE = CONNECTION_TYPES::AMBYTE;

    // the real cmd will follows
    c = Serial.readBytes(cmd_arr, 8);
    if (c < 8){
        ESP_LOGE(TAG, "ESP cmd parse failed");
        return -1;        
    }
    //Serial.printf("cmd is %d, %d, %d, %d, %d, %d, %d, %d\n", cmd_arr[0], cmd_arr[1], cmd_arr[2], cmd_arr[3], cmd_arr[4], cmd_arr[5], cmd_arr[6], cmd_arr[7]);

    switch (cmd_arr[0])
    {
    case 1:  // set PD gains
        // wire convention: gains 1-indexed (1..6), value outside that = "no change"
        core_set_gains(
            (cmd_arr[1] > 0 && cmd_arr[1] < 7) ? cmd_arr[1] - 1 : adpd_gains_config.Fluo,
            (cmd_arr[2] > 0 && cmd_arr[2] < 7) ? cmd_arr[2] - 1 : adpd_gains_config.FluoRef,
            (cmd_arr[3] > 0 && cmd_arr[3] < 7) ? cmd_arr[3] - 1 : adpd_gains_config.IR,
            (cmd_arr[4] > 0 && cmd_arr[4] < 7) ? cmd_arr[4] - 1 : adpd_gains_config.IRRef,
            (cmd_arr[5] > 0 && cmd_arr[5] < 7) ? cmd_arr[5] - 1 : adpd_gains_config.Sun,
            (cmd_arr[6] > 0 && cmd_arr[6] < 7) ? cmd_arr[6] - 1 : adpd_gains_config.Leaf);
        ESP_LOGV(TAG, "gains are %d, %d, %d, %d, %d, %d", adpd_gains_config.Fluo, adpd_gains_config.FluoRef, adpd_gains_config.IR, adpd_gains_config.IRRef, adpd_gains_config.Sun, adpd_gains_config.Leaf);
        Serial.write(ESP_CMD_DONE);
        break;

    case 2:  // set currents
        // wire convention: current byte >= 127 = "no change"
        core_set_currents(
            (cmd_arr[1] < 127) ? cmd_arr[1] : adpd_current_config.I620,
            (cmd_arr[2] < 127) ? cmd_arr[2] : adpd_current_config.I720,
            (cmd_arr[3] < 127) ? cmd_arr[3] : adpd_current_config.IR);
        ESP_LOGV(TAG, "currents are %d, %d, %d", adpd_current_config.I620, adpd_current_config.I720, adpd_current_config.IR);
        Serial.write(ESP_CMD_DONE);
        break;    
    
    case 10: // array run config
        core_config_detector();   // applies the global config + enters array mode
        Serial.write(ESP_CMD_DONE);
        break;

    case 20: // run mpf
    {
        if (adpd_gains_config.init == false) ESP_LOGE(TAG, "Gain preset not initized, use default!");
        if (adpd_current_config.init == false) ESP_LOGE(TAG, "Current preset not initized, use default!");

        uint16_t length = (((uint16_t) cmd_arr[1]) << 7) + cmd_arr[2];
        uint8_t interval = cmd_arr[3];
        bool change_act = (bool) cmd_arr[4];
        uint8_t act = cmd_arr[5];

        //Serial.printf("%d, %d, %d, %d\n", length, interval, change_act, act);
        
        Serial.write(ESP_CMD_DONE);
        core_run_mpf(length, interval, change_act, act);
        Serial.write(ESP_CMD_END);
    }    
        break;

    case 21:// run
    {   
        uint8_t arr_length = cmd_arr[1];
        uint8_t led_persist = cmd_arr[2];
        bool allow_interrupt = (bool) cmd_arr[3];

        uint8_t cc = 0;
        if ((arr_length == 0) || (arr_length > MAX_ARR_LEN)){
            ESP_LOGE(TAG, "run array wrong length: %d", arr_length);
            break;
        }
        uint8_t run_arr[arr_length * 8];
        core_ensure_array_config();

        cc = Serial.readBytes(run_arr, arr_length * 8);
        if (cc != (arr_length * 8)){
            ESP_LOGE(TAG, "run array elements count %d not match config %d", cc, arr_length);
            break;
        }
        Serial.write(ESP_CMD_DONE);
        run_arr_type1(arr_length, run_arr, led_persist, allow_interrupt);
        Serial.write(ESP_CMD_END);

    }
    break;

    // ── Parallel measurement protocol (trigger → poll → fetch) ──────────────
    case 22: // run start (async): ack, then run to completion into retained
             // buffers. Same payload as cmd 21. The host gets CMD_DONE and moves
             // on to the next sensor; no CMD_END (host trigger is ACK_ONLY).
    {
        uint8_t arr_length = cmd_arr[1];
        uint8_t led_persist = cmd_arr[2];
        bool allow_interrupt = (bool) cmd_arr[3];

        if ((arr_length == 0) || (arr_length > MAX_ARR_LEN)){
            ESP_LOGE(TAG, "run start wrong length: %d", arr_length);
            break;
        }
        uint8_t run_arr[arr_length * 8];
        core_ensure_array_config();

        uint8_t cc = Serial.readBytes(run_arr, arr_length * 8);
        if (cc != (arr_length * 8)){
            ESP_LOGE(TAG, "run start elements count %d not match config %d", cc, arr_length);
            break;
        }
        Serial.write(ESP_CMD_DONE);
        Serial.flush();   // ack must reach the host before the line goes quiet for the run
        ambit_async_run_start(arr_length, run_arr, led_persist, allow_interrupt);
    }
    break;

    case 23: // poll async state -> 1 status byte (0 IDLE | 1 DONE | 2 ERROR)
    {
        Serial.write(ESP_CMD_DONE);
        Serial.write(ambit_async_get_state());
        Serial.write(ESP_CMD_END);
    }
    break;

    case 24: // fetch retained async result: stream arrays over the AMBYTE FSM
    {
        Serial.write(ESP_CMD_DONE);
        ambit_async_fetch();   // streams the held arrays, or nothing if none ready
        Serial.write(ESP_CMD_END);
    }
    break;

    case 31: // get spec
    {
        uint16_t spec[12] = {0};
        float par = get_PAR(spec);
        memcpy(spec + 10, &par, 4);
        Serial.write(ESP_CMD_DONE);
        Serial.write((uint8_t*) spec, 24);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 32: // get temp
    {
        double leaf, chip;
        mlx_measure(&leaf, &chip);
        Serial.write(ESP_CMD_DONE);
        int16_t t1 = (int16_t) (leaf * 10);
        int16_t t2 = (int16_t) (chip * 10);
        Serial.write((uint8_t*) (&t1), 2);
        Serial.write((uint8_t*) (&t2), 2);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 33:    // retrieve ambit info
    {
        if (cmd_arr[1] == 1){ // send calibration info
            Serial.write(ESP_CMD_DONE);
            Serial.write((uint8_t*) &ambit_calibration_local, sizeof(ambit_calibration_info_t));
            Serial.write(ESP_CMD_END);
        }else if (cmd_arr[1] == 2){ // send FW info
            Serial.write(ESP_CMD_DONE);
            Serial.write((uint8_t*) &ambit_FW_info, sizeof(ambit_FW_info_t));
            Serial.write(ESP_CMD_END);
        }else if (cmd_arr[1] == 3){ // send metadata
            Serial.write(ESP_CMD_DONE);
            Serial.write((uint8_t*) &metadata_epprom, sizeof(metadata_t));
            Serial.write(ESP_CMD_END);
        }else{
            Serial.write(ESP_CMD_END);
        }
    }
    break;

    

    
    case 34: // get temp and raw
    {
        double leaf, leaf_1, chip;
        int16_t a1, a2, a3, a4;
        mlx_measure(&leaf, &chip, &leaf_1, &a1, &a2, &a3, &a4);
        Serial.write(ESP_CMD_DONE);
        int16_t t1 = (int16_t) (leaf * 10);
        int16_t t2 = (int16_t) (leaf_1 * 10);
        int16_t t3 = (int16_t) (chip * 10);
        Serial.write((uint8_t*) (&t1), 2);
        Serial.write((uint8_t*) (&t2), 2);
        Serial.write((uint8_t*) (&t3), 2);
        Serial.write((uint8_t*) (&a1), 2);
        Serial.write((uint8_t*) (&a2), 2);
        Serial.write((uint8_t*) (&a3), 2);
        Serial.write((uint8_t*) (&a4), 2);
        Serial.write(ESP_CMD_END);
    }
    break;

    
    case 37:    // set metadata
    {        
        Serial.write(ESP_CMD_DONE);
        Serial.readBytes((uint8_t*) (&metadata_incoming), sizeof(metadata_t));
        Serial.write(ESP_CMD_END);
        if (metadata_incoming.EOF_MARK == 2025) save_metadata();
        load_info_from_nvs(false);
    }
    break;



    case 4: // try/set actinic
    {
        AS_LED_OFF();
        Serial.write(ESP_CMD_DONE);
        uint8_t type = cmd_arr[1];
        uint8_t var = cmd_arr[2];
        uint8_t var2 = cmd_arr[3];
        float_t _factor = 1.0;
        if (type == 1){ // try actinics
            AS_LED_Current(50);
            AS_LED_ON();
            delay(3000);
            AS_LED_Current(var);
            delay(3000);
            AS_LED_OFF();            
            AS_LED_Current(0);
        }else if (type == 2){ // set actinic offset
            _factor = *((float *) &(cmd_arr[3]));
            if ((_factor > 0.01) && (_factor < 1.01)){
                preferences.begin("config", false);
                preferences.putFloat("actinic", _factor);
                preferences.end();
                ambit_calibration_local.actinic_coef = _factor;
            }
        }else if (type == 4){ // set actinic offset
            _factor = *((float *) &(cmd_arr[3]));
            if ((_factor > 0.05) && (_factor < 100.01)){
                //preferences.begin("config", false);
                //preferences.putFloat("spec", _factor);
                //preferences.end();
                ambit_calibration_local.spec_coef = _factor;
            }
        }else if (type == 5){
            AS_LED_Current(var);
            AS_LED_ON();
            delay(var2 * 100);
            AS_LED_OFF();
        }
        Serial.write(ESP_CMD_END);
    }
    break;

    
    case 5:
    {
        uint8_t ambit_id = cmd_arr[1];
        uint8_t intensity = cmd_arr[2];
        Serial.write(ESP_CMD_DONE);
        if ((ambit_id < 4) && (intensity > 4) && (intensity < 254)) as7431_blink(ambit_id, intensity);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 6: // do adpd baseline flash
    {
        Serial.write(ESP_CMD_DONE);
        uint32_t ret[6] = {0};
        fluor_offset(ret);
        preferences.begin("config", false);
        preferences.putUInt("adpd_lit", ret[1]);
        preferences.putUInt("adpd_sun", ret[2]);
        preferences.putUInt("adpd_leaf", ret[3]);
        preferences.putUInt("adpd_730", ret[4]);
        preferences.putUInt("adpd_730r", ret[5]);
        preferences.end();
        Serial.write(ESP_CMD_END);
        load_info_from_nvs(false);

    }
    break;

    case 17: // nvs update scalar
    {
        uint8_t type = cmd_arr[1]; // 1: actinic 
        uint8_t dtype = cmd_arr[2]; // 1: float
        if ((type == 1) && (dtype == 1)){ // update actinic coef
            Serial.write(ESP_CMD_DONE);
            float_t _factor = *((float *) &(cmd_arr[3]));
            if ((_factor > 0.01) && (_factor < 1.01)){
                preferences.begin("config", false);
                preferences.putFloat("actinic", _factor);
                preferences.end();
                ambit_calibration_local.actinic_coef = _factor;
                load_info_from_nvs(false);
            }
            Serial.write(ESP_CMD_END);
        }

    }   
    break; 

    case 18: // nvs update array
    {
        uint8_t type = cmd_arr[1]; // 1: actinic linear test
        float_t _factor = 1.0;
        if (type == 1){ // update actinic linear readings
            uint16_t _readingsf[6] = {0};
            Serial.write(ESP_CMD_DONE);
            Serial.readBytes((uint8_t*) _readingsf, 12);
            uint16_t checksum = 0;
            _factor = *((float *) &(cmd_arr[4]));
            for (int i = 0; i < 5; i++){
                checksum += _readingsf[i];
            }
            if (checksum == _readingsf[5]){
                preferences.begin("config", false);
                preferences.putUShort("act_50", _readingsf[0]);
                preferences.putUShort("act_100", _readingsf[1]);
                preferences.putUShort("act_150", _readingsf[2]);
                preferences.putUShort("act_200", _readingsf[3]);
                preferences.putUShort("act_250", _readingsf[4]);
                preferences.putFloat("actinic", _factor);
                preferences.end();
                load_info_from_nvs(false);                
            }
            Serial.write(ESP_CMD_END);
        }

    }   
    break; 






    // ── OTA-over-UART (cmds 25-28): host streams a new image; write via Update ──
    case 25: // OTA_BEGIN: cmd_arr[1..4] = total image size (LE u32)
    {
        uint32_t size = (uint32_t)cmd_arr[1] | ((uint32_t)cmd_arr[2] << 8) |
                        ((uint32_t)cmd_arr[3] << 16) | ((uint32_t)cmd_arr[4] << 24);
        if (Update.isRunning()) Update.abort();
        bool ok = (size > 0) && Update.begin(size);   // selects the next OTA partition + erases it
        ota_expected_seq = 0;
        Serial.write(ESP_CMD_DONE);
        Serial.write(ok ? 0 : 1);                     // 0 ok | 1 begin-fail (size/partition)
        Serial.write(ESP_CMD_END);
    }
    break;

    case 26: // OTA_DATA: cmd_arr[1]=len(1..200), cmd_arr[2..3]=seq(LE u16);
             // extra payload = len data bytes + 2-byte CRC16-CCITT(LE) over the data
    {
        uint8_t  len = cmd_arr[1];
        uint16_t seq = (uint16_t)cmd_arr[2] | ((uint16_t)cmd_arr[3] << 8);
        uint8_t  status;
        if (len == 0 || len > 200) {
            status = 4;                               // bad length (no payload consumed)
        } else {
            uint8_t buf[202];
            size_t  got = Serial.readBytes(buf, (size_t)len + 2);
            if (got != (size_t)len + 2)                  status = 5;   // short read
            else {
                uint16_t rx_crc = (uint16_t)buf[len] | ((uint16_t)buf[len + 1] << 8);
                if (crc16_ccitt(buf, len) != rx_crc)     status = 1;   // CRC fail -> host resends same seq
                else if (seq != ota_expected_seq)        status = 2;   // out-of-order (desync)
                else if (!Update.isRunning())            status = 6;   // no OTA_BEGIN
                else if (Update.write(buf, len) != len)  status = 3;   // flash write fail
                else { ota_expected_seq++;               status = 0; } // accepted
            }
        }
        Serial.write(ESP_CMD_DONE);
        Serial.write(status);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 27: // OTA_END: finalize + verify image + set boot partition, then reboot into it
    {
        bool ok = Update.isRunning() && Update.end();  // end() requires the full image (MD5/size check)
        Serial.write(ESP_CMD_DONE);
        Serial.write(ok ? 0 : 1);                      // 0 ok (about to reboot) | 1 verify-fail
        Serial.write(ESP_CMD_END);
        Serial.flush();                                // ack must reach the host before we restart
        if (ok) { delay(100); ESP.restart(); }         // boots the freshly-written slot; no return
    }
    break;

    case 28: // OTA_ABORT: discard a partial update, stay on the current image
    {
        if (Update.isRunning()) Update.abort();
        ota_expected_seq = 0;
        Serial.write(ESP_CMD_DONE);
        Serial.write(0);
        Serial.write(ESP_CMD_END);
    }
    break;

    case 29: // OTA_CONFIRM: mark the running (PENDING_VERIFY) image valid — cancels
             // the pending rollback. Sent by the ambyte once it has seen this image
             // answer after the OTA reboot. A no-op error if not pending (already valid).
    {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        Serial.write(ESP_CMD_DONE);
        Serial.write(err == ESP_OK ? 0 : 1);
        Serial.write(ESP_CMD_END);
    }
    break;

    default:
        ESP_LOGE(TAG, "Bad command");
        break;
    }

    return 0;
}

