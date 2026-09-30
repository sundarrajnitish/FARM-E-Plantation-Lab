// Compiles firmware/FarmeDrive/FarmeDrive.ino, unmodified, against the mocks.
#include "arduino_mock/Arduino.h"
#include "arduino_mock/Servo.h"
#include "arduino_mock/SoftwareSerial.h"
#include "arduino_mock/EEPROM.h"
#include "arduino_mock/avr/wdt.h"
#include "../firmware/FarmeDrive/src/farme_proto.h"
#include "../firmware/FarmeDrive/src/drive_core.h"
#include "sketch.h"

namespace drive_unit {
#include "../firmware/FarmeDrive/FarmeDrive.ino"

static void reset_globals() {
  nodemcu = SoftwareSerial(PIN_LINK_RX, PIN_LINK_TX);
  gate = Servo(); sweep1 = Servo(); sweep2 = Servo();
  drive = farme::Drive();
  parser = farme::Parser();
  last_ping_ms = 0; last_batt_ms = 0; batt_mV = 0;
  out_l = 99; out_r = 99;
  out_gate = -1; out_sweep1 = -1; out_sweep2 = -1;
  out_pump = true;
}

static double probe(int what) {
  switch (what) {
    case sim::PR_MODE: return drive.mode();
    case sim::PR_STATE: return drive.state();
    case sim::PR_CMD: return drive.last_seq();
    case sim::PR_DIST_MM: return drive.ground_mm();
    case sim::PR_HILLS: return drive.hills();
    case sim::PR_ROW: return drive.row();
    case sim::PR_TANK_ML: return drive.tank_mL();
    case sim::PR_FLAGS: return drive.flags();
    case sim::PR_LINK_OK: return (drive.flags() & farme::FL_LINK) ? 1 : 0;
    case sim::PR_FRAMES_OK: return parser.ok;
    case sim::PR_FRAMES_BAD: return parser.bad_crc + parser.bad_syntax + parser.overlong;
    case sim::PR_TEMP_C: return drive.temp_dC() / 10.0;
    case sim::PR_SEEDS_EST: return drive.hills();
    case sim::PR_BATT_MV: return drive.batt_mV();
    default: return -1;
  }
}
static const char* no_ui() { return ""; }
}  // namespace drive_unit

const sim::SketchIface sim::SK_DRIVE = {"FarmeDrive", drive_unit::reset_globals, drive_unit::setup, drive_unit::loop,
                                        drive_unit::probe, drive_unit::no_ui};
