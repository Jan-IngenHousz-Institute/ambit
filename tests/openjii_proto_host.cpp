// Host regression for lib/openjii_proto: the generic command object
// (`{"command":"INFO"}`) that the openJII app's identifyDevice() sends as its
// second discovery probe must come back as ONE bare JSON line with a "status"
// key and no frame token, and must not disturb the envelope shape.
//
// Build: see test/README ("openjii_proto"). ArduinoJson is compiled in its
// Arduino flavour (-DARDUINO) against tests/host_shim/Arduino.h, with PROGMEM
// support off since the shim has no flash-string types.
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "openjii_proto.h"

uint32_t g_host_millis = 0;

// A Stream whose RX side is fed by the test and whose TX side is captured.
struct FakeSerial : public Stream {
  std::string rx;
  std::string tx;
  size_t pos = 0;
  int available() override { return (int)(rx.size() - pos); }
  int read() override { return pos < rx.size() ? (unsigned char)rx[pos++] : -1; }
  int peek() override { return pos < rx.size() ? (unsigned char)rx[pos] : -1; }
  void flush() override {}
  size_t write(uint8_t c) override { tx += (char)c; return 1; }
  void feed(const char* s) { rx = s; pos = 0; tx.clear(); }
};

static void info_handler(const String&, JsonVariant root) {
  root["device_name"] = "AmbitV003";
  root["device_type"] = "ambit";
  root["device_id"] = "10:91:A8:4F:4F:C0";
  root["firmware_version"] = "1.2.0";
}

static void hello_handler(const String&, JsonVariant root) {
  root["device"] = "Ambit";
}

static void failing_handler(const String&, JsonVariant root) {
  root["error"] = "bad_arrun";
}

static void stream_handler(const String& args, Print& out) {
  out.print("{\"schema\":\"ambit.trace/3\",\"args\":\"");
  out.print(args);
  out.print("\"}");
}

static std::vector<std::string> lines(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == '\n') { out.push_back(cur); cur.clear(); }
    else cur += c;
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

static void run(FakeSerial& io, const char* input) {
  io.feed(input);
  ojii::poll(io);
  assert(!ojii::busy());
}

int main() {
  ojii::identity({ "Ambit", "1.2.0", "1.2.0" });
  ojii::on("INFO", info_handler);
  ojii::on("hello", hello_handler);
  ojii::on("failing", failing_handler);
  ojii::on_stream("arrun", stream_handler);

  FakeSerial io;

  // 1. The identification probe: exactly one line, parseable, status + data.
  run(io, "{\"command\":\"INFO\"}\n");
  {
    std::vector<std::string> l = lines(io.tx);
    assert(l.size() == 1);
    assert(io.tx.back() == '\n');
    assert(io.tx.find("7A1E3AA1") == std::string::npos);
    JsonDocument doc;
    assert(deserializeJson(doc, l[0].c_str()) == DeserializationError::Ok);
    assert(doc["status"] == "success");
    assert(doc["data"]["device_type"] == "ambit");
    assert(doc["data"]["device_id"] == "10:91:A8:4F:4F:C0");
    assert(doc["data"]["firmware_version"] == "1.2.0");
  }

  // 2. Unknown command: one error line the host can parse.
  run(io, "{\"command\":\"nope\"}\n");
  {
    std::vector<std::string> l = lines(io.tx);
    assert(l.size() == 1);
    JsonDocument doc;
    assert(deserializeJson(doc, l[0].c_str()) == DeserializationError::Ok);
    assert(doc["status"] == "error");
    assert(doc["error"] == "unknown_command");
    assert(doc["command"] == "nope");
  }

  //    ... and a hostile name is escaped, not spliced.
  run(io, "{\"command\":\"no\\\"pe\"}\n");
  {
    std::vector<std::string> l = lines(io.tx);
    assert(l.size() == 1);
    JsonDocument doc;
    assert(deserializeJson(doc, l[0].c_str()) == DeserializationError::Ok);
    assert(doc["command"] == "no\"pe");
  }

  // 3. Handler-reported error surfaces as the contract's status.
  run(io, "{\"command\":\"failing\"}\n");
  {
    JsonDocument doc;
    assert(deserializeJson(doc, lines(io.tx)[0].c_str()) == DeserializationError::Ok);
    assert(doc["status"] == "error");
    assert(doc["error"] == "bad_arrun");
  }

  // 4. Stream handlers are wrapped too, params pass through as comma-args.
  run(io, "{\"command\":\"arrun\",\"params\":\"1,0\"}\n");
  {
    JsonDocument doc;
    assert(deserializeJson(doc, lines(io.tx)[0].c_str()) == DeserializationError::Ok);
    assert(doc["status"] == "success");
    assert(doc["data"]["schema"] == "ambit.trace/3");
    assert(doc["data"]["args"] == "1,0");
  }

  // 5. The envelope is untouched: bare array and {"set":[...]} object forms
  //    still get the header + frame token, even though the object form has
  //    an array-valued key (no collision with the command-object detection).
  run(io, "[{\"label\":\"hello\"}]\n");
  {
    assert(io.tx.find("\"device_name\":\"Ambit\"") != std::string::npos);
    assert(io.tx.find("\"set\":[{\"device\":\"Ambit\"}]") != std::string::npos);
    assert(io.tx.size() > 9);
    assert(io.tx.compare(io.tx.size() - 9, 9, "7A1E3AA1\n") == 0);
  }
  run(io, "{\"set\":[{\"label\":\"hello\"}]}\n");
  assert(io.tx.compare(io.tx.size() - 9, 9, "7A1E3AA1\n") == 0);

  // 6. A non-string "command" member is not the command contract; it falls
  //    through to the (lenient) envelope path rather than crashing.
  run(io, "{\"command\":7}\n");
  assert(io.tx.compare(io.tx.size() - 9, 9, "7A1E3AA1\n") == 0);

  // 7. A stray trailing '\n' (the probe's terminator) leaves the receiver idle.
  run(io, "\n{\"command\":\"INFO\"}\n\n");
  assert(lines(io.tx).size() == 1);
  assert(!ojii::busy());

  printf("openjii_proto host tests passed\n");
  return 0;
}
