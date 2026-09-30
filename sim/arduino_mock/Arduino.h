// Arduino.h (simulation mock) - enough of the Arduino AVR core and the
// ESP8266 core for the four sketches in this repository. Every call is routed
// to the board the calling sketch runs on (sim::g_mcu) and costs the time it
// costs on that board, so blocking code blocks here as it would in hardware.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../rt.h"

typedef uint8_t byte;
typedef bool boolean;
typedef unsigned int word;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define DEC 10
#define HEX 16
#define LSBFIRST 0
#define MSBFIRST 1

// Arduino UNO analog pins
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define LED_BUILTIN 13

// NodeMCU silk-screen names -> ESP8266 GPIO numbers
static const uint8_t D0 = 16, D1 = 5, D2 = 4, D3 = 0, D4 = 2, D5 = 14, D6 = 12, D7 = 13, D8 = 15;

#define PROGMEM
#define PGM_P const char*
#define PSTR(s) (s)
#define F(s) (s)
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define strlen_P strlen
#define memcpy_P memcpy

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define isnan(x) __builtin_isnan(x)
#define NAN __builtin_nanf("")

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL);
void yield();

class String;
class Printable;

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char* s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; }
  size_t write(const uint8_t* b, size_t len) { size_t n = 0; while (len--) n += write(*b++); return n; }
  size_t print(const char* s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v, int base = DEC) { return printUnsigned(v, base); }
  size_t print(int v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned int v, int base = DEC) { return printUnsigned(v, base); }
  size_t print(long v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned long v, int base = DEC) { return printUnsigned(v, base); }
  size_t print(double v, int digits = 2) { return printFloat(v, digits); }
  size_t print(const String& s);
  size_t print(const Printable& p);
  size_t println() { return write((uint8_t)'\r') + write((uint8_t)'\n'); }
  template <typename T> size_t println(const T& v) { size_t n = print(v); return n + println(); }
  template <typename T> size_t println(const T& v, int b) { size_t n = print(v, b); return n + println(); }

  size_t printUnsigned(unsigned long long v, int base) {
    char buf[24]; int i = 0;
    do { int d = (int)(v % base); buf[i++] = (char)(d < 10 ? '0' + d : 'A' + d - 10); v /= base; } while (v);
    size_t n = 0; while (i) n += write((uint8_t)buf[--i]);
    return n;
  }
  size_t printSigned(long long v, int base) {
    if (v < 0 && base == DEC) { write((uint8_t)'-'); return 1 + printUnsigned((unsigned long long)(-v), base); }
    return printUnsigned((unsigned long long)v, base);
  }
  // Print::printFloat from the Arduino core, same algorithm
  size_t printFloat(double number, int digits) {
    if (number != number) return print("nan");
    if (number > 4294967040.0 || number < -4294967040.0) return print("ovf");
    size_t n = 0;
    if (number < 0.0) { n += print('-'); number = -number; }
    double rounding = 0.5;
    for (int i = 0; i < digits; ++i) rounding /= 10.0;
    number += rounding;
    unsigned long long int_part = (unsigned long long)number;
    double remainder = number - (double)int_part;
    n += printUnsigned(int_part, DEC);
    if (digits > 0) n += print('.');
    while (digits-- > 0) {
      remainder *= 10.0;
      unsigned int d = (unsigned int)remainder;
      n += printUnsigned(d, DEC);
      remainder -= d;
    }
    return n;
  }
};

class Printable {
 public:
  virtual ~Printable() {}
  virtual size_t printTo(Print& p) const = 0;
};

enum LookaheadMode { SKIP_ALL, SKIP_NONE, SKIP_WHITESPACE };
#define NO_IGNORE_CHAR '\x01'

// Stream, as in the Arduino AVR core 1.8.
class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
  void setTimeout(unsigned long ms) { timeout_ = ms; }
  unsigned long getTimeout() { return timeout_; }
  long parseInt(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);
  float parseFloat(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);
  size_t readBytesUntil(char terminator, char* buffer, size_t length);

 protected:
  unsigned long timeout_ = 1000;
  int timedRead();
  int timedPeek();
  int peekNextDigit(LookaheadMode lookahead, bool detectDecimal);
  // time to wait for the next byte that could appear on this stream
  virtual uint64_t nextArrival() { return ~0ULL; }
};

class HardwareSerial : public Stream {
 public:
  void begin(unsigned long baud);
  void end() {}
  size_t write(uint8_t c) override;
  using Print::write;
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override;
  explicit operator bool() const { return true; }
};
extern HardwareSerial Serial;

#include "WString.h"
