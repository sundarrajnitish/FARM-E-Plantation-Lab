// FarmeDrive - FARM-E, Arduino UNO side: tracks, seed gate, pump and the
// downward-looking ultrasonic sensor. All decisions live in src/drive_core;
// this file only moves data between the pins and the controller.
//
// The loop never blocks for long: the longest step is one echo wait
// (PING_TIMEOUT_US, 6 ms) or one telemetry line on the software UART
// (~23 ms at 19200 baud). A 1 s watchdog restarts the board if anything hangs.
#include <Servo.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "config.h"
#include "src/farme_proto.h"
#include "src/drive_core.h"

SoftwareSerial nodemcu(PIN_LINK_RX, PIN_LINK_TX);
Servo gate;
Servo sweep1, sweep2;
farme::Drive drive;
farme::Parser parser;

uint32_t last_ping_ms = 0, last_batt_ms = 0;
uint16_t batt_mV = 0;
int8_t out_l = 99, out_r = 99;   // what is on the pins now (99 = not written yet)
int out_gate = -1, out_sweep1 = -1, out_sweep2 = -1;
bool out_pump = true;

// ---- settings in EEPROM: magic, version, 10 values, CRC-8
static const uint8_t EE_MAGIC = 0xFA, EE_VERSION = 2;
struct Stored {
  uint8_t magic, version;
  int16_t v[farme::P_COUNT];
  uint8_t crc;
};

static uint8_t stored_crc(const Stored& s) { return farme::crc8((const uint8_t*)&s, sizeof(Stored) - 1); }

void load_config(farme::DriveConfig& cfg) {
  Stored s;
  EEPROM.get(0, s);
  if (s.magic != EE_MAGIC || s.version != EE_VERSION || s.crc != stored_crc(s)) return;  // defaults
  for (uint8_t p = 1; p < farme::P_COUNT; ++p) cfg.set(p, s.v[p]);  // range-checked
}

void save_config(const farme::DriveConfig& cfg) {
  Stored s;
  s.magic = EE_MAGIC;
  s.version = EE_VERSION;
  s.v[0] = 0;
  for (uint8_t p = 1; p < farme::P_COUNT; ++p) s.v[p] = (int16_t)cfg.get(p);
  s.crc = stored_crc(s);
  EEPROM.put(0, s);  // update(): only bytes that changed are written
}

// ---- pins
void set_track(uint8_t in1, uint8_t in2, int8_t cmd) {
  digitalWrite(in1, (cmd == 1 || cmd == 2) ? HIGH : LOW);
  digitalWrite(in2, (cmd == -1 || cmd == 2) ? HIGH : LOW);
}

void apply(const farme::Outputs& o) {
  if (o.left != out_l) { set_track(PIN_LM1, PIN_LM2, o.left); out_l = o.left; }
  if (o.right != out_r) { set_track(PIN_RM1, PIN_RM2, o.right); out_r = o.right; }
  if (o.pump != out_pump) { digitalWrite(PIN_PUMP, o.pump ? HIGH : LOW); out_pump = o.pump; }
  if (o.gate_deg != out_gate) { gate.write(o.gate_deg); out_gate = o.gate_deg; }
#if SWEEPS_FITTED
  if (o.sweep[0] != out_sweep1) { sweep1.write(o.sweep[0]); out_sweep1 = o.sweep[0]; }
  if (o.sweep[1] != out_sweep2) { sweep2.write(o.sweep[1]); out_sweep2 = o.sweep[1]; }
#endif
}

// One HC-SR04 ping: 10 us trigger, echo width -> millimetres, speed of sound
// from the NodeMCU's DHT11 temperature when it is known.
int16_t ping_mm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, PING_TIMEOUT_US);
  if (us == 0) return -1;
  int32_t t_dC = drive.temp_dC();
  if (t_dC < -300 || t_dC > 600) t_dC = 250;
  int32_t c_mm_s = 331300L + 606L * t_dC / 10;           // speed of sound in mm/s
  return (int16_t)((int32_t)us * (c_mm_s / 100) / 20000L);  // round trip: mm = us * c / 2 / 1e6
}

void log_state() {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F("] "));
  Serial.print(farme::Drive::state_name(drive.state()));
  Serial.print(F(" hills="));
  Serial.print(drive.hills());
  Serial.print(F(" row="));
  Serial.print(drive.row());
  Serial.print(F(" ground="));
  Serial.print(drive.ground_mm());
  Serial.print(F("/"));
  Serial.println(drive.baseline_mm());
}

void setup() {
  // park every output before it becomes an output
  const uint8_t outs[] = {PIN_LM1, PIN_LM2, PIN_RM1, PIN_RM2, PIN_PUMP, PIN_TRIG, PIN_LED};
  for (uint8_t i = 0; i < sizeof(outs); ++i) { digitalWrite(outs[i], LOW); pinMode(outs[i], OUTPUT); }
  pinMode(PIN_ECHO, INPUT);
  out_l = out_r = 0;
  out_pump = false;

  Serial.begin(USB_BAUD);
  nodemcu.begin(LINK_BAUD);

  farme::DriveConfig cfg;
  cfg.sweeps_fitted = SWEEPS_FITTED;
  load_config(cfg);
  drive.begin(cfg, millis());

  gate.attach(PIN_GATE);
  gate.write(cfg.gate_closed_deg);
  out_gate = cfg.gate_closed_deg;
#if SWEEPS_FITTED
  sweep1.attach(PIN_SWEEP1);
  sweep2.attach(PIN_SWEEP2);
#endif
  Serial.println(F("FARM-E drive"));
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  uint32_t now = millis();

  while (nodemcu.available()) {
    if (parser.feed((uint8_t)nodemcu.read())) drive.on_frame(parser.frame(), now);
  }

  farme::Inputs in;
  in.now_ms = now;
  if (now - last_ping_ms >= PING_INTERVAL_MS) {
    last_ping_ms = now;
    in.ping = true;
    in.ground_mm = ping_mm();
  }
#if BATTERY_DIVIDER_FITTED
  if (now - last_batt_ms >= 200) {
    last_batt_ms = now;
    int32_t mv = (int32_t)analogRead(PIN_BATT) * 5000L / 1023L * BATT_DIVIDER_NUM / BATT_DIVIDER_DEN;
    batt_mV = (uint16_t)mv;
  }
  in.batt_mV = batt_mV;
#endif

  drive.tick(in);
  apply(drive.out());
  digitalWrite(PIN_LED, drive.mode() == farme::MODE_FAULT ? ((now / 150) & 1) : (drive.mode() != farme::MODE_IDLE));

  if (drive.take_config_changed()) save_config(drive.config());
  if (drive.take_state_changed()) log_state();

  if (drive.telemetry_due(now)) {
    char line[farme::MAX_LINE + 8];
    size_t n = farme::encode(drive.telemetry(now), line, sizeof(line));
    nodemcu.write((const uint8_t*)line, n);
  }
}
