// config.h - FarmeDrive (Arduino UNO) wiring and build options.
#pragma once

// L298N-style dual H-bridge, enable jumpers fitted (full speed only)
#define PIN_LM1 2   // left track IN1
#define PIN_LM2 4   // left track IN2
#define PIN_RM1 6   // right track IN3
#define PIN_RM2 7   // right track IN4

#define PIN_GATE 9      // SG90 on the seed-hopper gate
#define PIN_SWEEP1 10   // optional sweep-arm servos
#define PIN_SWEEP2 11
#define PIN_PUMP 12     // water pump driver (logic HIGH = on)
#define PIN_LED 13      // status LED

#define PIN_LINK_RX A0  // SoftwareSerial from the NodeMCU's D6
#define PIN_LINK_TX A1  // SoftwareSerial to the NodeMCU's D5 (through a 1k/2k divider: 5 V -> 3.3 V)
#define PIN_BATT A3     // battery through 10k (top) / 3.3k (bottom)
#define PIN_TRIG A4     // HC-SR04, mounted looking down just ahead of the tracks
#define PIN_ECHO A5

#define LINK_BAUD 19200UL
#define USB_BAUD 115200UL

#define BATTERY_DIVIDER_FITTED 1     // 0: no divider on A3, speed is not battery-compensated
#define SWEEPS_FITTED 0              // 1: servos on D10/D11
#define PING_INTERVAL_MS 30          // HC-SR04 needs ~29 ms between pings
#define PING_TIMEOUT_US 6000UL       // ~1 m: anything further is "no ground"

// Battery divider: V_batt = V_adc * (10k + 3.3k) / 3.3k
#define BATT_DIVIDER_NUM 133L
#define BATT_DIVIDER_DEN 33L
