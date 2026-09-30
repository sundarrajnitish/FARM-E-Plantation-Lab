// mock.cpp - the Arduino/ESP8266 API on top of sim::Rig. Every function acts
// on the board whose sketch called it (sim::g_mcu) and charges that board the
// time the call takes on real hardware.
#include "arduino_mock/Arduino.h"
#include "arduino_mock/Servo.h"
#include "arduino_mock/SoftwareSerial.h"
#include "arduino_mock/ESP8266WebServer.h"
#include "arduino_mock/DHTStable.h"
#include "arduino_mock/EEPROM.h"
#include "arduino_mock/avr/wdt.h"
#include "rig.h"

using sim::g_mcu;
using sim::g_rig;
using sim::Mcu;

HardwareSerial Serial;
EEPROMClass EEPROM;
ESP8266WiFiClass WiFi;

namespace {
inline bool uno() { return g_mcu->board == sim::UNO; }
inline uint64_t cost(uint64_t uno_us, uint64_t esp_us) { return uno() ? uno_us : esp_us; }
inline void spend(uint64_t us) { g_mcu->spend(us); }

// Wait (idle) until `deadline`, but wake whenever something could have changed
// for this board: the next byte on its UART, or the other board's clock.
void wait_step(uint64_t deadline) {
  Mcu* m = g_mcu;
  const Mcu& o = g_rig->mcu[1 - m->board];
  uint64_t byte_us = m->ss_baud ? 10000000ULL / m->ss_baud : sim::QUANTUM_US;
  uint64_t next = deadline;
  uint64_t arr = m->next_rx_arrival();
  if (arr < next) next = arr;
  if (o.now + byte_us < next) next = o.now + byte_us;
  if (next <= m->now) next = m->now + 1;
  m->advance_to(next);
}
}  // namespace

// ------------------------------------------------------------------ pins
void pinMode(uint8_t pin, uint8_t mode) {
  Mcu* m = g_mcu;
  spend(cost(4, 1));
  if (pin >= 40) return;
  m->mode[pin] = mode == OUTPUT ? 1 : 0;
  if (mode == INPUT_PULLUP) m->level[pin] = 1;
  if (uno()) g_rig->sync_uno_outputs();
}

void digitalWrite(uint8_t pin, uint8_t val) {
  Mcu* m = g_mcu;
  spend(cost(5, 1));
  if (pin >= 40) return;
  bool was = m->level[pin];
  m->level[pin] = val ? 1 : 0;
  if (uno()) {
    g_rig->sync_uno_outputs();
    if (pin == sim::PIN_TRIG && m->mode[pin] == 1 && was != (bool)m->level[pin])
      g_rig->world.trigger_edge(m->level[pin], m->now);
  }
}

static int echo_level(uint64_t t) {
  uint64_t r, f;
  if (!g_rig->world.echo_window(&r, &f)) return 0;
  return t >= r && t < f;
}

int digitalRead(uint8_t pin) {
  Mcu* m = g_mcu;
  spend(cost(4, 1));
  if (pin >= 40) return 0;
  if (uno() && pin == sim::PIN_ECHO) return echo_level(m->now);
  return m->level[pin];
}

int analogRead(uint8_t pin) {
  spend(cost(112, 100));
  if (!uno()) return 0;
  if (pin < 14) pin += 14;
  if (pin == sim::PIN_BATT) {
    g_rig->world.step_to(g_mcu->now);
    return g_rig->battery_divider ? g_rig->world.battery_adc() : 0;
  }
  return 0;
}

void analogWrite(uint8_t pin, int val) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, val > 127 ? HIGH : LOW);
}

unsigned long millis() { spend(cost(2, 1)); return (unsigned long)(uint32_t)(g_mcu->now / 1000ULL); }
unsigned long micros() { spend(cost(2, 1)); return (unsigned long)(uint32_t)g_mcu->now; }
void delay(unsigned long ms) { g_mcu->advance_to(g_mcu->now + (uint64_t)ms * 1000ULL); }
void delayMicroseconds(unsigned int us) { g_mcu->advance_to(g_mcu->now + us); }
void yield() { spend(cost(1, 5)); }

// pulseIn: like the AVR core, the timeout covers the whole measurement
// (waiting for a previous pulse to end, for the start, and the pulse itself).
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
  Mcu* m = g_mcu;
  spend(cost(3, 1));
  uint64_t start = m->now, deadline = start + timeout;
  if (!(uno() && pin == sim::PIN_ECHO && state == HIGH)) {
    m->advance_to(deadline);
    return 0;
  }
  g_rig->world.step_to(m->now);
  uint64_t r, f;
  if (!g_rig->world.echo_window(&r, &f) || m->now >= r) {  // no pulse ahead of us
    m->advance_to(deadline);
    return 0;
  }
  if (f > deadline) { m->advance_to(deadline); return 0; }
  m->advance_to(f + 2);
  return (unsigned long)(f - r);
}

// ------------------------------------------------------------------ Stream (AVR core 1.8)
int Stream::timedRead() {
  unsigned long start = millis();
  do {
    int c = read();
    if (c >= 0) return c;
    wait_step(((uint64_t)start + timeout_) * 1000ULL);
  } while (millis() - start < timeout_);
  return -1;
}

int Stream::timedPeek() {
  unsigned long start = millis();
  do {
    int c = peek();
    if (c >= 0) return c;
    wait_step(((uint64_t)start + timeout_) * 1000ULL);
  } while (millis() - start < timeout_);
  return -1;
}

int Stream::peekNextDigit(LookaheadMode lookahead, bool detectDecimal) {
  int c;
  while (1) {
    c = timedPeek();
    if (c < 0 || c == '-' || (c >= '0' && c <= '9') || (detectDecimal && c == '.')) return c;
    switch (lookahead) {
      case SKIP_NONE: return -1;
      case SKIP_WHITESPACE:
        switch (c) {
          case ' ': case '\t': case '\r': case '\n': break;
          default: return -1;
        }
      case SKIP_ALL: break;
    }
    read();
  }
}

long Stream::parseInt(LookaheadMode lookahead, char ignore) {
  bool isNegative = false;
  long value = 0;
  int c = peekNextDigit(lookahead, false);
  if (c < 0) return 0;
  do {
    if (c == ignore) {}
    else if (c == '-') isNegative = true;
    else if (c >= '0' && c <= '9') value = value * 10 + c - '0';
    read();
    c = timedPeek();
  } while ((c >= '0' && c <= '9') || c == ignore);
  return isNegative ? -value : value;
}

float Stream::parseFloat(LookaheadMode lookahead, char ignore) {
  bool isNegative = false, isFraction = false;
  long value = 0;
  float fraction = 1.0f;
  int c = peekNextDigit(lookahead, true);
  if (c < 0) return 0;
  do {
    if (c == ignore) {}
    else if (c == '-') isNegative = true;
    else if (c == '.') isFraction = true;
    else if (c >= '0' && c <= '9') {
      value = value * 10 + c - '0';
      if (isFraction) fraction *= 0.1f;
    }
    read();
    c = timedPeek();
  } while ((c >= '0' && c <= '9') || (c == '.' && !isFraction) || c == ignore);
  if (isNegative) value = -value;
  if (isFraction) return value * fraction;
  return (float)value;
}

size_t Stream::readBytesUntil(char terminator, char* buffer, size_t length) {
  size_t index = 0;
  while (index < length) {
    int c = timedRead();
    if (c < 0 || c == terminator) break;
    *buffer++ = (char)c;
    index++;
  }
  return index;
}

// ------------------------------------------------------------------ hardware UART (log)
void HardwareSerial::begin(unsigned long baud) {
  spend(cost(20, 50));
  g_mcu->hw_baud = (uint32_t)baud;
}

size_t HardwareSerial::write(uint8_t c) {
  Mcu* m = g_mcu;
  if (!m->hw_baud) return 1;
  uint64_t byte_us = 10000000ULL / m->hw_baud;
  // TX buffer full: wait for the ISR to drain one byte
  if (m->hw_busy_until > m->now) {
    uint64_t queued = (m->hw_busy_until - m->now) / byte_us;
    if (queued >= (uint64_t)m->hw_buf) m->advance_to(m->hw_busy_until - (uint64_t)(m->hw_buf - 1) * byte_us);
  }
  m = g_mcu;
  uint64_t base = m->hw_busy_until > m->now ? m->hw_busy_until : m->now;
  m->hw_busy_until = base + byte_us;
  m->log_char((char)c);
  spend(cost(3, 1));
  return 1;
}

void HardwareSerial::flush() {
  if (g_mcu->hw_busy_until > g_mcu->now) g_mcu->advance_to(g_mcu->hw_busy_until);
}

// ------------------------------------------------------------------ software UART (the link)
void SoftwareSerial::begin(long baud) {
  Mcu* m = g_mcu;
  spend(cost(30, 30));
  m->ss_on = true;
  m->ss_baud = (uint32_t)baud;
  m->rx_head = m->rx_n = 0;
  (void)rx_; (void)tx_;
}

void SoftwareSerial::end() { g_mcu->ss_on = false; }

bool SoftwareSerial::overflow() { bool o = g_mcu->rx_overflow > 0; g_mcu->rx_overflow = 0; return o; }

int SoftwareSerial::available() {
  spend(cost(2, 1));
  g_mcu->pump_rx();
  return g_mcu->rx_n;
}

int SoftwareSerial::read() {
  spend(cost(3, 1));
  Mcu* m = g_mcu;
  m->pump_rx();
  if (!m->rx_n) return -1;
  int c = m->rx[m->rx_head];
  m->rx_head = (m->rx_head + 1) % 64;
  --m->rx_n;
  return c;
}

int SoftwareSerial::peek() {
  spend(cost(2, 1));
  Mcu* m = g_mcu;
  m->pump_rx();
  return m->rx_n ? m->rx[m->rx_head] : -1;
}

uint64_t SoftwareSerial::nextArrival() { return g_mcu->next_rx_arrival(); }

size_t SoftwareSerial::write(uint8_t c) {
  Mcu* m = g_mcu;
  if (!m->ss_on || !m->ss_baud) return 0;
  uint64_t byte_us = 10000000ULL / m->ss_baud;
  uint64_t s = m->now;
  const Mcu& o = g_rig->mcu[1 - m->board];
  bool garbled = !o.ss_on || o.ss_baud != m->ss_baud;
  m->out->push(c, s + byte_us, garbled);
  m->note_tx(s, s + byte_us);
  spend(byte_us + cost(4, 2));  // bit-banged with interrupts off: the CPU is busy
  return 1;
}

// ------------------------------------------------------------------ Servo
uint8_t Servo::attach(int pin) {
  Mcu* m = g_mcu;
  spend(cost(10, 5));
  if (ch_ < 0) {
    for (int i = 0; i < 4; ++i) if (m->servo_pin[i] < 0) { ch_ = i; break; }
    if (ch_ < 0) return 0;
  }
  pin_ = pin;
  m->servo_pin[ch_] = pin;
  m->servo_deg[ch_] = deg_;
  if (uno() && pin == sim::PIN_SERVO) { g_rig->world.step_to(m->now); g_rig->world.set_servo(deg_, true); }
  return (uint8_t)ch_;
}

void Servo::detach() {
  Mcu* m = g_mcu;
  if (ch_ < 0) return;
  if (uno() && pin_ == sim::PIN_SERVO) { g_rig->world.step_to(m->now); g_rig->world.set_servo(deg_, false); }
  m->servo_pin[ch_] = -1;
  ch_ = -1;
}

void Servo::write(int value) {
  if (value >= 544) { writeMicroseconds(value); return; }
  if (value < 0) value = 0;
  if (value > 180) value = 180;
  Mcu* m = g_mcu;
  spend(cost(8, 3));
  deg_ = value;
  if (ch_ < 0) return;  // not attached: the library stores the value, no pulses
  m->servo_deg[ch_] = value;
  if (uno()) {
    g_rig->world.step_to(m->now);
    if (pin_ == sim::PIN_SERVO) g_rig->world.set_servo(value, true);
    else if (pin_ == sim::PIN_SWEEP1) g_rig->world.set_sweep(0, value);
    else if (pin_ == sim::PIN_SWEEP2) g_rig->world.set_sweep(1, value);
  }
}

void Servo::writeMicroseconds(int us) {
  if (us < 544) us = 544;
  if (us > 2400) us = 2400;
  write((int)((long)(us - 544) * 180 / (2400 - 544)));
}

// ------------------------------------------------------------------ DHT11
int DHTStable::read11(uint8_t pin) {
  (void)pin;
  g_mcu->advance_to(g_mcu->now + 23000);
  int t, h;
  if (!g_rig->world.dht(g_mcu->now, &t, &h)) {
    humidity_ = DHTLIB_INVALID_VALUE;
    temperature_ = DHTLIB_INVALID_VALUE;
    return DHTLIB_ERROR_CHECKSUM;
  }
  humidity_ = (float)h; temperature_ = (float)t;
  return DHTLIB_OK;
}

// ------------------------------------------------------------------ EEPROM / watchdog
uint8_t EEPROMClass::read(int addr) {
  spend(cost(2, 1));
  return (addr >= 0 && addr < 1024) ? g_mcu->eeprom[addr] : 0xFF;
}
void EEPROMClass::write(int addr, uint8_t v) {
  if (addr < 0 || addr >= 1024) return;
  g_mcu->eeprom[addr] = v;
  g_mcu->advance_to(g_mcu->now + 3300);
}

static const uint32_t WDT_MS[] = {15, 30, 60, 120, 250, 500, 1000, 2000, 4000, 8000};
void wdt_enable(uint8_t t) {
  g_mcu->wdt_on = true;
  g_mcu->wdt_ms = WDT_MS[t < 10 ? t : 9];
  g_mcu->wdt_last = g_mcu->now;
}
void wdt_disable() { g_mcu->wdt_on = false; }
void wdt_reset() { g_mcu->wdt_last = g_mcu->now; }

// ------------------------------------------------------------------ Wi-Fi and web server
bool ESP8266WiFiClass::softAP(const char* ssid, const char* pass, int channel, int hidden, int max_conn) {
  (void)ssid; (void)channel; (void)hidden; (void)max_conn;
  g_mcu->advance_to(g_mcu->now + 60000);
  if (pass && *pass && strlen(pass) < 8) return false;  // WPA2 needs 8..63 characters
  g_rig->ap_up_us = g_mcu->now + 1500000;  // the phone joins ~1.5 s later
  return true;
}

int ESP8266WiFiClass::softAPgetStationNum() {
  spend(5);
  return g_mcu->now >= g_rig->ap_up_us ? 1 : 0;
}

void ESP8266WebServer::begin() { spend(200); begun_ = true; }

void ESP8266WebServer::on(const char* uri, HTTPMethod m, THandlerFunction fn) {
  spend(5);
  if (nroutes_ >= 16) return;
  Route& r = routes_[nroutes_++];
  size_t n = strlen(uri); if (n > sizeof(r.uri) - 1) n = sizeof(r.uri) - 1;
  memcpy(r.uri, uri, n); r.uri[n] = 0;
  r.m = m; r.fn = fn;
}

static const void* memchr_(const void* s, int c, size_t n) {
  const char* p = (const char*)s;
  for (size_t i = 0; i < n; ++i) if (p[i] == (char)c) return p + i;
  return nullptr;
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static void url_decode(const char* s, size_t n, char* out, size_t cap) {
  size_t o = 0;
  for (size_t i = 0; i < n && o + 1 < cap; ++i) {
    char c = s[i];
    if (c == '+') c = ' ';
    else if (c == '%' && i + 2 < n && hexv(s[i + 1]) >= 0 && hexv(s[i + 2]) >= 0) { c = (char)(hexv(s[i + 1]) * 16 + hexv(s[i + 2])); i += 2; }
    out[o++] = c;
  }
  out[o] = 0;
}

void ESP8266WebServer::handleClient() {
  Mcu* m = g_mcu;
  spend(25);
  if (!begun_ || m->now < g_rig->ap_up_us) return;
  sim::HttpReq* best = nullptr;
  for (int i = 0; i < sim::Rig::MAX_HTTP; ++i) {
    sim::HttpReq& r = g_rig->http[i];
    if (r.id && r.state == 0 && r.t_us <= m->now && (!best || r.t_us < best->t_us || (r.t_us == best->t_us && r.id < best->id))) best = &r;
  }
  if (!best) return;
  best->state = 2;
  int id = best->id;
  // parse "path?a=1&b=2"
  const char* url = best->url;
  const char* q = strchr(url, '?');
  size_t plen = q ? (size_t)(q - url) : strlen(url);
  url_decode(url, plen, path_, sizeof(path_));
  nargs_ = 0;
  if (q) {
    const char* s = q + 1;
    while (*s && nargs_ < 8) {
      const char* amp = strchr(s, '&');
      size_t len = amp ? (size_t)(amp - s) : strlen(s);
      const char* eq = (const char*)memchr_(s, '=', len);
      size_t nl = eq ? (size_t)(eq - s) : len;
      url_decode(s, nl, names_[nargs_], sizeof(names_[0]));
      if (eq) url_decode(eq + 1, len - nl - 1, values_[nargs_], sizeof(values_[0]));
      else values_[nargs_][0] = 0;
      ++nargs_;
      if (!amp) break;
      s = amp + 1;
    }
  }
  g_rig->world.event_at(m->now, sim::EV_HTTP, id, 0);
  spend(1200);  // accept, receive and parse the request
  cur_ = id;
  answered_ = false;
  THandlerFunction fn = nullptr;
  for (int i = 0; i < nroutes_; ++i) if (strcmp(routes_[i].uri, path_) == 0) { fn = routes_[i].fn; break; }
  if (fn) fn();
  else if (not_found_) not_found_();
  else send(404, "text/plain", "Not found");
  sim::HttpReq* r = g_rig->http_find(id);
  if (r && !answered_) { r->state = 1; r->code = 0; r->done_us = g_mcu->now; }
  cur_ = -1;
}

String ESP8266WebServer::arg(const char* name) {
  spend(5);
  for (int i = 0; i < nargs_; ++i) if (strcmp(names_[i], name) == 0) return String(values_[i]);
  return String("");
}

bool ESP8266WebServer::hasArg(const char* name) {
  for (int i = 0; i < nargs_; ++i) if (strcmp(names_[i], name) == 0) return true;
  return false;
}

void ESP8266WebServer::send_raw(int code, const char* ctype, const char* body, size_t len) {
  Mcu* m = g_mcu;
  spend(800 + len);  // headers + body through lwIP at ~1 MB/s
  sim::HttpReq* r = g_rig->http_find(cur_);
  if (!r || answered_) return;
  answered_ = true;
  sim::Rig* rig = g_rig;
  if (len > sim::Rig::BODY_CAP) len = sim::Rig::BODY_CAP;
  if (rig->body_used + len > sim::Rig::BODY_CAP) { rig->body_used = 0; ++rig->body_gen; }
  memcpy(rig->body + rig->body_used, body, len);
  r->body_gen = rig->body_gen; r->body_off = rig->body_used; r->body_len = (uint32_t)len;
  rig->body_used += (uint32_t)len;
  size_t cl = strlen(ctype); if (cl > sizeof(r->ctype) - 1) cl = sizeof(r->ctype) - 1;
  memcpy(r->ctype, ctype, cl); r->ctype[cl] = 0;
  r->code = code; r->state = 1; r->done_us = m->now;
  rig->world.event_at(m->now, sim::EV_HTTP, r->id, 1, code);
}
