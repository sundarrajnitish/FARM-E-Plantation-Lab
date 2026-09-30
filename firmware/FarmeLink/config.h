// config.h - FarmeLink (NodeMCU 1.0, ESP-12E) wiring and build options.
#pragma once

#define PIN_LINK_RX D5   // SoftwareSerial from the UNO's A1 (through a 1k/2k divider)
#define PIN_LINK_TX D6   // SoftwareSerial to the UNO's A0 (3.3 V is a valid HIGH for the UNO)
#define PIN_DHT D2       // DHT11 data (GPIO4)
#define PIN_STATUS_LED D4  // on-board blue LED, active LOW

#define LINK_BAUD 19200
#define USB_BAUD 115200
#define DHT_PERIOD_MS 2500   // the DHT11 cannot be read faster than once a second
