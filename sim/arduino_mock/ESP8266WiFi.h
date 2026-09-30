// ESP8266WiFi.h (simulation mock) - soft access point only.
#pragma once
#include "Arduino.h"

class IPAddress : public Printable {
 public:
  IPAddress(uint8_t a = 0, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) { o_[0] = a; o_[1] = b; o_[2] = c; o_[3] = d; }
  size_t printTo(Print& p) const override {
    size_t n = 0;
    for (int i = 0; i < 4; ++i) { n += p.print((int)o_[i]); if (i < 3) n += p.print('.'); }
    return n;
  }
  String toString() const {
    String s;
    for (int i = 0; i < 4; ++i) { s += (int)o_[i]; if (i < 3) s += '.'; }
    return s;
  }
 private:
  uint8_t o_[4];
};

enum WiFiMode_t { WIFI_OFF = 0, WIFI_STA = 1, WIFI_AP = 2, WIFI_AP_STA = 3 };

class ESP8266WiFiClass {
 public:
  bool mode(WiFiMode_t m) { (void)m; return true; }
  bool softAP(const char* ssid, const char* pass = nullptr, int channel = 1, int hidden = 0, int max_conn = 4);
  IPAddress softAPIP() { return IPAddress(192, 168, 4, 1); }
  int softAPgetStationNum();
  void persistent(bool) {}
};
extern ESP8266WiFiClass WiFi;
