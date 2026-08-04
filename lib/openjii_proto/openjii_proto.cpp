// openjii_proto implementation — see openjii_proto.h and
// CommunicationProtocolOpenJIISerial.md.
#include "openjii_proto.h"
#include <ctype.h>

namespace ojii {
namespace {

constexpr const char* FRAME_TOKEN = "7A1E3AA1";   // §7.2 (reserved for a checksum)
constexpr size_t   RX_MAX        = 2048;          // §4
constexpr uint32_t JSON_IDLE_MS  = 1000;          // §4
constexpr size_t   MAX_CMDS      = 48;

struct Entry { const char* name; SnapshotFn fn; };
Entry    s_cmds[MAX_CMDS];
size_t   s_ncmds = 0;
struct SEntry { const char* name; StreamFn fn; };
SEntry   s_streams[MAX_CMDS];
size_t   s_nstreams = 0;
Identity s_id = { "device", "0", "0" };

// receiver state
String   s_rx;
enum class Mode { UNKNOWN, LINE, JSON };
Mode     s_mode = Mode::UNKNOWN;
int      s_brace = 0, s_bracket = 0;
bool     s_inStr = false;
char     s_prev = 0;
uint32_t s_last_byte_ms = 0;

void reset_rx() {
  s_rx = "";
  s_mode = Mode::UNKNOWN;
  s_brace = 0; s_bracket = 0; s_inStr = false; s_prev = 0;
}

SnapshotFn find(const String& name) {
  for (size_t i = 0; i < s_ncmds; i++)
    if (name == s_cmds[i].name) return s_cmds[i].fn;
  return nullptr;
}

StreamFn find_stream(const String& name) {
  for (size_t i = 0; i < s_nstreams; i++)
    if (name == s_streams[i].name) return s_streams[i].fn;
  return nullptr;
}

// Split a label into command name + comma-args ("set_currents,1,2,3" -> "set_currents","1,2,3").
void split_label(const String& label, String& cmd, String& args) {
  int comma = label.indexOf(',');
  if (comma < 0) { cmd = label; args = ""; }
  else { cmd = label.substring(0, comma); args = label.substring(comma + 1); }
}

// ── errors (§9) ─────────────────────────────────────────────────────────────
void err(Print& out, const char* code, const char* k1 = nullptr, const String& v1 = String(),
         const char* k2 = nullptr, const String& v2 = String()) {
  out.print("{\"error\":\""); out.print(code); out.print('"');
  if (k1) { out.print(",\""); out.print(k1); out.print("\":\""); out.print(v1); out.print('"'); }
  if (k2) { out.print(",\""); out.print(k2); out.print("\":\""); out.print(v2); out.print('"'); }
  out.print("}\n");
}

// ── serialisers (§5.2 / §11) ────────────────────────────────────────────────
bool bare_string_ok(const char* s) {
  // A bare LINE string must not contain '\n' or edge whitespace (§8).
  size_t n = strlen(s);
  if (n == 0) return true;
  if (s[0] == ' ' || s[n - 1] == ' ') return false;
  for (size_t i = 0; i < n; i++) if (s[i] == '\n' || s[i] == '\r') return false;
  return true;
}

// LINE serialiser: scalar -> bare, container -> strict JSON. No trailing newline.
void render_line_node(JsonVariantConst node, Print& out) {
  if (node.is<JsonObjectConst>() || node.is<JsonArrayConst>()) {
    serializeJson(node, out);
  } else if (node.is<const char*>()) {
    const char* s = node.as<const char*>();
    if (bare_string_ok(s)) out.print(s);     // bare (unquoted)
    else serializeJson(node, out);           // fall back to quoted JSON string
  } else {
    serializeJson(node, out);                // number / bool / null -> bare
  }
}

// ── path resolver (§6) ──────────────────────────────────────────────────────
// Resolve `path` against `root`, render the result (LINE) or a §9 error.
// `full` is the whole request line (echoed in error "path").
void resolve_and_render(JsonVariantConst root, const String& path, const String& full, Print& out) {
  JsonVariantConst node = root;
  int start = 0;
  while (true) {
    int dot = path.indexOf('.', start);
    String seg = (dot < 0) ? path.substring(start) : path.substring(start, dot);

    // trailing method, e.g. keys()
    if (seg.endsWith("()")) {
      String method = seg.substring(0, seg.length() - 2);
      if (method == "keys") {
        if (!node.is<JsonObjectConst>()) { err(out, "not_a_container", "path", full, "at", seg); return; }
        JsonDocument tmp; JsonArray arr = tmp.to<JsonArray>();
        for (JsonPairConst kv : node.as<JsonObjectConst>()) arr.add(kv.key().c_str());
        serializeJson(tmp, out); out.print('\n'); return;
      }
      err(out, "bad_path", "path", full); return;
    }

    if (node.is<JsonObjectConst>()) {
      JsonObjectConst obj = node.as<JsonObjectConst>();
      JsonVariantConst child = obj[seg];
      if (child.isNull()) {                  // absent key (present-null is not used in payloads)
        // build the available-keys list
        String avail = "[";
        bool f = true;
        for (JsonPairConst kv : obj) { if (!f) avail += ','; avail += '"'; avail += kv.key().c_str(); avail += '"'; f = false; }
        avail += ']';
        out.print("{\"error\":\"no_such_key\",\"path\":\""); out.print(full);
        out.print("\",\"at\":\""); out.print(seg);
        out.print("\",\"available\":"); out.print(avail); out.print("}\n");
        return;
      }
      node = child;
    } else if (node.is<JsonArrayConst>()) {
      // segment must be a non-negative index
      bool numeric = seg.length() > 0;
      for (size_t i = 0; i < seg.length(); i++) if (!isdigit((unsigned char)seg[i])) numeric = false;
      JsonArrayConst arr = node.as<JsonArrayConst>();
      long idx = numeric ? seg.toInt() : -1;
      if (!numeric || idx < 0 || (size_t)idx >= arr.size()) { err(out, "index_out_of_range", "path", full, "at", seg); return; }
      node = arr[idx];
    } else {
      // scalar — cannot descend
      err(out, "not_a_container", "path", full, "at", seg); return;
    }

    if (dot < 0) break;
    start = dot + 1;
  }
  render_line_node(node, out);
  out.print('\n');
}

// ── LINE dispatch (§5) ──────────────────────────────────────────────────────
void handle_line(const String& line, Print& out) {
  int dot = line.indexOf('.');
  if (dot >= 0) {                            // query form: cmd.path
    String cmd = line.substring(0, dot);
    SnapshotFn fn = find(cmd);
    if (!fn) { err(out, "unknown_command", "path", line); return; }
    JsonDocument doc; JsonVariant root = doc.to<JsonVariant>();
    fn("", root);
    resolve_and_render(doc.as<JsonVariantConst>(), line.substring(dot + 1), line, out);
  } else {                                   // action form: cmd[,args]
    int comma = line.indexOf(',');
    String cmd  = (comma < 0) ? line : line.substring(0, comma);
    String args = (comma < 0) ? String("") : line.substring(comma + 1);
    StreamFn sfn = find_stream(cmd);
    if (sfn) { sfn(args, out); out.print('\n'); return; }
    SnapshotFn fn = find(cmd);
    if (!fn) { err(out, "unknown_command", "path", line); return; }
    JsonDocument doc; JsonVariant root = doc.to<JsonVariant>();
    fn(args, root);
    render_line_node(doc.as<JsonVariantConst>(), out); out.print('\n');
  }
}

// ── JSON envelope (§7) ──────────────────────────────────────────────────────
// Find the command list: _protocol_set_, an array of {set:[...]}, a bare array,
// or the first array-valued member of an object (§7.1, lenient).
JsonArrayConst extract_list(JsonVariantConst req) {
  if (req["_protocol_set_"].is<JsonArrayConst>()) return req["_protocol_set_"].as<JsonArrayConst>();
  if (req.is<JsonArrayConst>()) {
    JsonArrayConst arr = req.as<JsonArrayConst>();
    for (JsonObjectConst o : arr) if (o["set"].is<JsonArrayConst>()) return o["set"].as<JsonArrayConst>();
    return arr;                              // bare array used directly
  }
  if (req.is<JsonObjectConst>())
    for (JsonPairConst kv : req.as<JsonObjectConst>())
      if (kv.value().is<JsonArrayConst>()) return kv.value().as<JsonArrayConst>();
  return JsonArrayConst();
}

// Emit one command's payload as strict JSON (one flat `set` element, §7.2).
void emit_payload_json(const String& label, Print& out) {
  String cmd, args; split_label(label, cmd, args);
  StreamFn sfn = find_stream(cmd);
  if (sfn) { sfn(args, out); return; }       // measurement writes its own JSON value
  SnapshotFn fn = find(cmd);
  if (!fn) { out.print("{\"error\":\"unknown_command\",\"path\":\""); out.print(label); out.print("\"}"); return; }
  JsonDocument doc; JsonVariant root = doc.to<JsonVariant>();
  fn(args, root);
  serializeJson(doc, out);                   // strict JSON, all strings quoted
}

void handle_json(const String& doc_str, Print& out) {
  JsonDocument req;
  DeserializationError e = deserializeJson(req, doc_str);
  if (e) { err(out, "json_parse", "detail", e.c_str()); return; }

  JsonArrayConst list = extract_list(req);

  out.print("{\"device_name\":\""); out.print(s_id.device);
  out.print("\",\"device_version\":\""); out.print(s_id.version);
  out.print("\",\"device_battery\":0,\"device_firmware\":\""); out.print(s_id.firmware);
  out.print("\",\"sample\":[{\"protocol_id\":\"NaN\",\"set\":[");
  bool first = true;
  for (JsonObjectConst entry : list) {
    const char* label = entry["label"];
    if (!label) continue;                    // §7.1 entries without a label are skipped
    uint16_t repeats = entry["protocol_repeats"] | 1;
    if (repeats == 0) repeats = 1;
    for (uint16_t r = 0; r < repeats; r++) { // flat: N consecutive elements (§7.2)
      if (!first) out.print(',');
      first = false;
      emit_payload_json(String(label), out);
    }
  }
  out.print("]}]}"); out.print(FRAME_TOKEN); out.print('\n');
}

}  // namespace

// ── public API ──────────────────────────────────────────────────────────────
void on(const char* name, SnapshotFn fn) {
  if (s_ncmds < MAX_CMDS) s_cmds[s_ncmds++] = { name, fn };
}

void on_stream(const char* name, StreamFn fn) {
  if (s_nstreams < MAX_CMDS) s_streams[s_nstreams++] = { name, fn };
}

void identity(const Identity& id) { s_id = id; }

bool busy(void) { return s_mode != Mode::UNKNOWN || s_rx.length() > 0; }

void reset(void) { reset_rx(); }

void poll(Stream& io) {
  while (io.available() > 0) {
    char c = (char)io.read();
    if (c == '\r') continue;                 // §4: CR ignored

    if (s_mode == Mode::UNKNOWN) {
      // drop leading framing/boot junk before a mode is locked
      unsigned char uc = (unsigned char)c;
      bool printable = (uc >= 0x20 && uc < 0x7F);
      if (!printable && c != '\n') continue;
    }

    s_rx += c;
    s_last_byte_ms = millis();

    if (s_mode == Mode::UNKNOWN) {            // §4: first non-ws byte selects mode
      int i = 0;
      while (i < (int)s_rx.length() && isspace((unsigned char)s_rx[i])) i++;
      if (i < (int)s_rx.length())
        s_mode = (s_rx[i] == '{' || s_rx[i] == '[') ? Mode::JSON : Mode::LINE;
    }

    if (s_mode == Mode::LINE) {
      if (c == '\n') {
        s_rx.trim();
        if (s_rx.length() > 0) { handle_line(s_rx, io); io.flush(); }   // empty line -> no response (§4)
        reset_rx();
      }
    } else if (s_mode == Mode::JSON) {        // string-aware brace/bracket depth
      if (c == '"' && s_prev != '\\') s_inStr = !s_inStr;
      if (!s_inStr) {
        if (c == '{') s_brace++; else if (c == '}') s_brace--;
        else if (c == '[') s_bracket++; else if (c == ']') s_bracket--;
      }
      s_prev = c;
      if (!s_inStr && s_brace == 0 && s_bracket == 0) {
        s_rx.trim();
        if (s_rx.length() > 0) { handle_json(s_rx, io); io.flush(); }
        reset_rx();
      }
    }

    if (s_rx.length() > RX_MAX) { io.print("{\"error\":\"rx_overflow\"}\n"); reset_rx(); }
  }

  // §4: incomplete JSON idle > 1 s
  if (s_mode == Mode::JSON && s_rx.length() > 0 && (millis() - s_last_byte_ms) > JSON_IDLE_MS) {
    io.print("{\"error\":\"json_timeout\"}\n");
    reset_rx();
  }
}

}  // namespace ojii
