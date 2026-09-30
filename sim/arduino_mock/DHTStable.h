// DHTStable.h (simulation mock) - Rob Tillaart's DHTStable library (the
// portable successor of DHTlib), as the NodeMCU firmware uses it.
#pragma once
#include "Arduino.h"

#ifndef DHTLIB_OK
#define DHTLIB_OK 0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT -2
#define DHTLIB_INVALID_VALUE -999
#endif

class DHTStable {
 public:
  int read11(uint8_t pin);
  float getHumidity() const { return humidity_; }
  float getTemperature() const { return temperature_; }

 private:
  float humidity_ = DHTLIB_INVALID_VALUE;
  float temperature_ = DHTLIB_INVALID_VALUE;
};
