// frontend_json — wires the device's snapshot commands into the openJII protocol
// module (was frontend_cloud, gated on the deleted VARIANT_CLOUD). In the single
// image the main loop routes only '{'/'[' envelope traffic here; bare printable
// lines keep their legacy do_command console replies (that text console is what
// the openJII app driver and the Calibratron actually parse), and openJII LINE
// mode is intentionally unreachable — it had no live consumer.
//
// Measurement commands beyond arrun (q/mpf as one streamed JSON value per
// envelope position) are added once D6 — the app's exact in-envelope measurement
// shape — is settled.
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
  root["version"] = AMBIT_FW_VERSION;   // build-injected, same source as cmd 33/2 and text hello
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

// Parse comma-separated uint8 values from token index `from` into arr. Return
// capacity + 1 as soon as an extra token is seen so callers can distinguish an
// overlong payload without writing past the destination.
static size_t arg_array8(const String& a, uint8_t from, uint8_t* arr, size_t capacity) {
  size_t count = 0;
  int pos = 0;
  for (uint8_t i = 0; i < from; i++) {
    pos = a.indexOf(',', pos);
    if (pos < 0) return 0;
    pos++;
  }
  while (pos >= 0 && pos < (int)a.length()) {
    if (count >= capacity) return capacity + 1;
    int next = a.indexOf(',', pos);
    if (next < 0) next = a.length();
    arr[count++] = (uint8_t) a.substring(pos, next).toInt();
    pos = next + 1;
    if (next == (int)a.length()) break;
  }
  return count;
}

static constexpr long ARRUN_MIN_LEN = 1;
static constexpr long ARRUN_MAX_LEN = 16;

static constexpr bool valid_arrun_shape(long len, size_t parsed) {
  // Validate before multiplying: `len` comes from the wire and must not be
  // narrowed to uint8_t (or multiplied there) until its range is known.
  return len >= ARRUN_MIN_LEN && len <= ARRUN_MAX_LEN &&
         parsed == static_cast<size_t>(len) * 8U;
}

// There is no host/unit harness in this firmware. Keep the boundary contract
// compiler-checked, including the one-point request that exposed the regression.
static_assert(valid_arrun_shape(1, 8), "one-point arrun must be accepted");
static_assert(valid_arrun_shape(16, 128), "full arrun must be accepted");
static_assert(!valid_arrun_shape(0, 0), "empty arrun must be rejected");
static_assert(!valid_arrun_shape(17, 136), "oversized arrun must be rejected");
static_assert(!valid_arrun_shape(1, 128), "arrun byte count must match len");

// arrun,<len>,<persist>,<8*len array bytes> — runs an array-mode measurement and
// emits one JSON object {"env":[..],"s_630":[..],...}. run_arr_type1's json_output
// path writes it to Serial, which IS the openJII output stream in the cloud build.
static void cmd_arrun(const String& args, Print& out) {
  long requested_len = arg_long(args, 0);
  uint8_t persist = (uint8_t) arg_long(args, 1);
  uint8_t arr[128] = {0};
  if (requested_len < ARRUN_MIN_LEN || requested_len > ARRUN_MAX_LEN) {
    out.print("{\"error\":\"bad_arrun\"}");
    return;
  }
  const uint8_t len = static_cast<uint8_t>(requested_len);
  const size_t expected = static_cast<size_t>(len) * 8U;
  const size_t parsed = arg_array8(args, 2, arr, expected);
  if (!valid_arrun_shape(requested_len, parsed)) {
    out.print("{\"error\":\"bad_arrun\"}");
    return;
  }

  if (!adpd_gains_config.init)   adpd_gains_config.init = true;
  if (!adpd_current_config.init) adpd_current_config.init = true;
  if (adpd_mode != ADPD_CONFIG_MODE::ARRAY_MODE1) {
    conf_slow_FR_1();
    adpd_mode = ADPD_CONFIG_MODE::ARRAY_MODE1;
  }
  CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;   // no inline plotting; JSON emitted at the end
  run_arr_type1(len, arr, persist, false, true);  // json_output -> writes the JSON object to Serial
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

// Register the openJII envelope surface. Called once from setup() in
// ambit-1.ino; the device init itself lives there (single shared boot path).
// No printf/markers here: UART0 is the protocol port, and ASCII in front of a
// binary frame breaks the Ambyte session.
void frontend_json_register() {
  ojii::identity({ "Ambit", AMBIT_FW_VERSION, AMBIT_FW_VERSION });
  ojii::on("hello",   cmd_hello);
  ojii::on("temp",    cmd_temp);
  ojii::on("get_par", cmd_get_par);
  ojii::on("PAR",     cmd_PAR);
  ojii::on("set_currents", cmd_set_currents);
  ojii::on("set_gains",    cmd_set_gains);
  ojii::on("get_currents", cmd_get_currents);
  ojii::on("get_gains",    cmd_get_gains);
  ojii::on_stream("arrun", cmd_arrun);
}
