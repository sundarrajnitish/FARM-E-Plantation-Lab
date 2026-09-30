// SoftwareSerial.h (simulation mock) - bit-banged UART on any two pins, as
// on the UNO (SoftwareSerial) and the ESP8266 (EspSoftwareSerial). Both are
// half-duplex in practice: while a byte is shifted out with interrupts off,
// an incoming byte is lost. Transmitting blocks for the whole byte.
#pragma once
#include "Arduino.h"

class SoftwareSerial : public Stream {
 public:
  SoftwareSerial(uint8_t rx, uint8_t tx, bool inverse = false) : rx_(rx), tx_(tx) { (void)inverse; }
  void begin(long baud);
  void end();
  bool listen() { return true; }
  bool isListening() { return true; }
  bool overflow();
  int available() override;
  int read() override;
  int peek() override;
  size_t write(uint8_t c) override;
  using Print::write;
  explicit operator bool() const { return true; }

 protected:
  uint64_t nextArrival() override;

 private:
  uint8_t rx_, tx_;
};
