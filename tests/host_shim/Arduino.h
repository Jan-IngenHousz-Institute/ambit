// Minimal host-side stand-in for <Arduino.h>, just enough to compile
// lib/openjii_proto on a PC (see tests/openjii_proto_host.cpp and test/README).
// It provides the four Arduino types the protocol module and ArduinoJson's
// Arduino integration touch: String, Print, Stream and millis(). Nothing here
// is meant to be faithful beyond what those call sites use.
#pragma once

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <string>

class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(char c) : s_(1, c) {}
  String(const std::string& s) : s_(s) {}

  String& operator=(const char* s) { s_ = s ? s : ""; return *this; }
  String& operator+=(char c) { s_ += c; return *this; }
  String& operator+=(const char* s) { if (s) s_ += s; return *this; }
  String& operator+=(const String& o) { s_ += o.s_; return *this; }
  bool concat(const char* s) { if (s) s_ += s; return true; }

  bool operator==(const char* s) const { return s && s_ == s; }
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator!=(const char* s) const { return !(*this == s); }

  char operator[](unsigned i) const { return i < s_.size() ? s_[i] : 0; }
  unsigned length() const { return (unsigned)s_.size(); }
  const char* c_str() const { return s_.c_str(); }

  int indexOf(char c, unsigned from = 0) const {
    std::string::size_type p = s_.find(c, from);
    return p == std::string::npos ? -1 : (int)p;
  }
  String substring(unsigned from) const { return from < s_.size() ? String(s_.substr(from)) : String(); }
  String substring(unsigned from, unsigned to) const {
    if (from >= s_.size() || to <= from) return String();
    return String(s_.substr(from, to - from));
  }
  bool endsWith(const char* suf) const {
    size_t n = strlen(suf);
    return s_.size() >= n && s_.compare(s_.size() - n, n, suf) == 0;
  }
  void trim() {
    size_t b = 0, e = s_.size();
    while (b < e && isspace((unsigned char)s_[b])) b++;
    while (e > b && isspace((unsigned char)s_[e - 1])) e--;
    s_ = s_.substr(b, e - b);
  }
  long toInt() const { return atol(s_.c_str()); }

 private:
  std::string s_;
};

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; i++) write(buf[i]);
    return n;
  }
  size_t print(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(const String& s) { return print(s.c_str()); }
  size_t println(const char* s) { size_t n = print(s); return n + write('\n'); }
};

// ArduinoJson's Arduino flavour converts ::Printable values; never used here.
class Printable {
 public:
  virtual ~Printable() {}
  virtual size_t printTo(Print& p) const = 0;
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
  size_t readBytes(char* buf, size_t n) {
    size_t i = 0;
    while (i < n && available() > 0) buf[i++] = (char)read();
    return i;
  }
};

// Test-controlled clock so the idle-timeout branch is deterministic.
extern uint32_t g_host_millis;
inline uint32_t millis() { return g_host_millis; }
