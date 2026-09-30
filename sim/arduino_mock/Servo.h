// Servo.h (simulation mock) - the Arduino Servo library. The UNO's servo on
// pin 9 drives the seed-hopper gate in the simulated world; pins 10 and 11
// are the optional sweep arms.
#pragma once
#include "Arduino.h"

class Servo {
 public:
  uint8_t attach(int pin);
  uint8_t attach(int pin, int min_us, int max_us) { (void)min_us; (void)max_us; return attach(pin); }
  void detach();
  void write(int value);            // degrees (values >= 544 are microseconds)
  void writeMicroseconds(int us);
  int read() const { return deg_; }
  bool attached() const { return ch_ >= 0; }

 private:
  int ch_ = -1;
  int pin_ = -1;
  int deg_ = 90;                    // the library's default pulse is 1500 us
};
