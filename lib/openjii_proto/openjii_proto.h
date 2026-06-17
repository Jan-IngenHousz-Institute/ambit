// openjii_proto — device-agnostic openJII serial protocol (LINE + JSON envelope).
//
// Implements CommunicationProtocolOpenJIISerial.md: first-byte mode detection,
// LINE mode (bare scalars / JSON containers, dot-path queries, keys()), and the
// JSON "openJII envelope" (flat `set`, frame token), with the §9 error taxonomy.
//
// A device registers per-command SNAPSHOT handlers that build a value tree
// (command-as-root); the framework owns the serializers, the path resolver, the
// envelope writer, and the receiver. Measurement/STREAM commands (one streamed
// JSON value per envelope position) are a separate API added once D6 is settled.
//
// Whole module is gated on VARIANT_CLOUD so the ambyte image links no JSON.
#pragma once
#ifdef VARIANT_CLOUD

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ojii {

// Snapshot handler: populate `root` as this command's command-as-root payload
// (§2.1). `args` is the comma-separated remainder after the command name
// ("set_currents,1,2,3" -> "1,2,3"; "" if none). On error set root["error"]
// to a §9 code (the framework still serialises whatever the handler produced).
using SnapshotFn = void (*)(const String& args, JsonVariant root);

// Stream/measurement handler: write ONE well-formed JSON value directly to `out`
// (for commands whose output is too large to buffer as a tree, e.g. a run). In an
// envelope it occupies one `set` slot; in LINE mode it is the bare response.
using StreamFn = void (*)(const String& args, Print& out);

struct Identity {
  const char* device;    // e.g. "Ambit"   -> hello.device, envelope device_name
  const char* version;   // e.g. "0.0.3"
  float       firmware;  // e.g. 0.003f     -> envelope device_firmware
};

// Register a snapshot command (name is matched case-sensitively).
void on(const char* name, SnapshotFn fn);

// Register a stream/measurement command.
void on_stream(const char* name, StreamFn fn);

// Set device identity used by the envelope header and the built-in hello.
void identity(const Identity& id);

// Drive the protocol: consume available bytes from `io`, and when a full request
// is framed, dispatch it and write the response to `io`. Call every loop().
void poll(Stream& io);

}  // namespace ojii

#endif  // VARIANT_CLOUD
