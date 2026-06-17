// frontend_cloud — cloud/app (USB-CDC) variant: wires the device's snapshot
// commands into the openJII protocol module and drives it. Gated on VARIANT_CLOUD
// so it is empty in the ambyte image.
//
// Increment 1: snapshot commands only (hello / temp / get_par / PAR). Measurement
// commands (arrun/q/mpf as one streamed JSON value per envelope position) are
// added once D6 — the app's exact in-envelope measurement shape — is settled.
#ifdef VARIANT_CLOUD

#include <Arduino.h>
#include <ArduinoJson.h>
#include "openjii_proto.h"
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "nvs1.h"
#include "config.h"
#include "PAM.h"
#include "core.h"

extern ADPD6 adpd;   // defined in ambit-1.ino

// ── snapshot command handlers (command-as-root, §2.1) ───────────────────────

static void cmd_hello(const String& args, JsonVariant root) {
  root["device"]  = "Ambit";
  root["version"] = "0.0.3";
  root["name"]    = ambit_calibration_local.ambit_name;
}

static void cmd_temp(const String& args, JsonVariant root) {
  double leaf, ambient, reflect;
  int16_t a1, a2, a3, a4;
  mlx_measure(&leaf, &ambient, &reflect, &a1, &a2, &a3, &a4);
  root["leaf"]    = leaf;
  root["ambient"] = ambient;
  root["reflect"] = reflect;
}

static void cmd_get_par(const String& args, JsonVariant root) {
  uint16_t spec[10];
  float par = get_PAR(spec);
  root["par_raw"] = par;
  JsonArray ch = root["channels"].to<JsonArray>();
  for (int i = 0; i < 10; i++) ch.add(spec[i]);
}

static void cmd_PAR(const String& args, JsonVariant root) {
  uint16_t spec[10];
  float par = get_PAR(spec) * ambit_calibration_local.spec_coef;
  root["par"] = par;
  JsonArray ch = root["channels"].to<JsonArray>();
  for (int i = 0; i < 10; i++) ch.add(spec[i]);
}

// ── entry points called from ambit-1.ino under VARIANT_CLOUD ────────────────

// ── measurement command (one streamed JSON value per envelope slot) ─────────
// n-th comma token of `a` (token 0 is before the first comma).
static long arg_long(const String& a, uint8_t n, long def = 0) {
  int start = 0;
  for (uint8_t i = 0; i < n; i++) {
    start = a.indexOf(',', start);
    if (start < 0) return def;
    start++;
  }
  return a.substring(start).toInt();
}

// Parse comma-separated uint8 values from token index `from` into arr.
static uint8_t arg_array8(const String& a, uint8_t from, uint8_t* arr, uint8_t maxLen) {
  uint8_t count = 0;
  int pos = 0;
  for (uint8_t i = 0; i < from; i++) {
    pos = a.indexOf(',', pos);
    if (pos < 0) return 0;
    pos++;
  }
  while (pos >= 0 && pos < (int)a.length() && count < maxLen) {
    int next = a.indexOf(',', pos);
    if (next < 0) next = a.length();
    arr[count++] = (uint8_t) a.substring(pos, next).toInt();
    pos = next + 1;
    if (next == (int)a.length()) break;
  }
  return count;
}

// arrun,<len>,<persist>,<8*len array bytes> — runs an array-mode measurement and
// emits one JSON object {"env":[..],"s_fluo":[..],...}. run_arr_type1's json_output
// path writes it to Serial, which IS the openJII output stream in the cloud build.
static void cmd_arrun(const String& args, Print& out) {
  uint8_t len     = (uint8_t) arg_long(args, 0);
  uint8_t persist = (uint8_t) arg_long(args, 1);
  uint8_t arr[128] = {0};
  uint8_t parsed = arg_array8(args, 2, arr, len * 8);
  if (len == 0 || parsed == 0) { out.print("{\"error\":\"bad_arrun\"}"); return; }

  if (!adpd_gains_config.init)   adpd_gains_config.init = true;
  if (!adpd_current_config.init) adpd_current_config.init = true;
  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1) {
    conf_slow_FR_1();
    adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
  }
  CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;   // no inline plotting; JSON emitted at the end
  run_arr_type1(16, arr, persist, false, true);   // json_output -> writes the JSON object to Serial
}

// ── config setters (snapshot: ack with the applied values; command-as-root) ──
// 0-indexed values; they take effect on the next run (core marks config dirty).
static void cmd_set_currents(const String& args, JsonVariant root) {
  core_set_currents((uint8_t) arg_long(args, 0), (uint8_t) arg_long(args, 1), (uint8_t) arg_long(args, 2));
  root["I620"] = adpd_current_config.I620;
  root["I720"] = adpd_current_config.I720;
  root["IR"]   = adpd_current_config.IR;
}

static void cmd_set_gains(const String& args, JsonVariant root) {
  core_set_gains((uint8_t) arg_long(args, 0), (uint8_t) arg_long(args, 1), (uint8_t) arg_long(args, 2),
                 (uint8_t) arg_long(args, 3), (uint8_t) arg_long(args, 4), (uint8_t) arg_long(args, 5));
  root["Fluo"]    = adpd_gains_config.Fluo;
  root["FluoRef"] = adpd_gains_config.FluoRef;
  root["IR"]      = adpd_gains_config.IR;
  root["IRRef"]   = adpd_gains_config.IRRef;
  root["Sun"]     = adpd_gains_config.Sun;
  root["Leaf"]    = adpd_gains_config.Leaf;
}

// ── config getters (read back the staged config; command-as-root) ───────────
static void cmd_get_currents(const String& args, JsonVariant root) {
  root["I620"] = adpd_current_config.I620;
  root["I720"] = adpd_current_config.I720;
  root["IR"]   = adpd_current_config.IR;
}

static void cmd_get_gains(const String& args, JsonVariant root) {
  root["Fluo"]    = adpd_gains_config.Fluo;
  root["FluoRef"] = adpd_gains_config.FluoRef;
  root["IR"]      = adpd_gains_config.IR;
  root["IRRef"]   = adpd_gains_config.IRRef;
  root["Sun"]     = adpd_gains_config.Sun;
  root["Leaf"]    = adpd_gains_config.Leaf;
}

// printf reaches the ESP-IDF console (USB-Serial/JTAG) regardless of CDC DTR, so
// these per-step markers show exactly where init reaches / hangs.
#define INIT_STEP(label, expr) do { printf("[init] " label "...\n"); expr; printf("[init] " label " ok\n"); } while (0)

void cloud_setup() {
  // Pin setup + STF strobe pulse BEFORE device init — mirrors fork A's proven
  // app-compatible startup; skipping the strobe can hang the detector init.
  esp_timer_early_init();
  pinMode(STF_FLASH_PIN, OUTPUT);
  digitalWrite(STF_FLASH_PIN, LOW);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);

  Serial.begin(115200);
  Serial.setTimeout(50);
  printf("\n[init] boot (cloud)\n");
  esp_log_level_set("*", ESP_LOG_NONE);

  digitalWrite(STF_FLASH_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(STF_FLASH_PIN, LOW);

  INIT_STEP("i2c_bus",  init_i2c_bus());
  INIT_STEP("spi_bus",  init_spi_bus());
  INIT_STEP("adpd",     adpd.begin());
  INIT_STEP("as7341",   as7341.begin());
  INIT_STEP("check_as", check_AS7341());
  INIT_STEP("led_off",  AS_LED_OFF());
  INIT_STEP("mlx",      mlx_init());
  INIT_STEP("nvs",      load_info_from_nvs(false));

  esp_log_level_set("*", ESP_LOG_NONE);   // UART0 is also the protocol port — keep it clean
  CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;

  ojii::identity({ "Ambit", "0.0.3", 0.003f });
  ojii::on("hello",   cmd_hello);
  ojii::on("temp",    cmd_temp);
  ojii::on("get_par", cmd_get_par);
  ojii::on("PAR",     cmd_PAR);
  ojii::on("set_currents", cmd_set_currents);
  ojii::on("set_gains",    cmd_set_gains);
  ojii::on("get_currents", cmd_get_currents);
  ojii::on("get_gains",    cmd_get_gains);
  ojii::on_stream("arrun", cmd_arrun);

  printf("[init] Ready (cloud)\n");
}

void cloud_loop() {
  ojii::poll(Serial);
}

#endif  // VARIANT_CLOUD
