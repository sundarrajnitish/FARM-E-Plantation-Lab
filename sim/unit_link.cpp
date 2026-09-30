// Compiles firmware/FarmeLink/FarmeLink.ino, unmodified, against the mocks.
#include "arduino_mock/Arduino.h"
#include "arduino_mock/SoftwareSerial.h"
#include "arduino_mock/ESP8266WiFi.h"
#include "arduino_mock/ESP8266WebServer.h"
#include "arduino_mock/DHTStable.h"
#include "../firmware/FarmeLink/src/farme_proto.h"
#include "../firmware/FarmeLink/src/link_core.h"
#include "sketch.h"
#include "rig.h"

namespace link_unit {
#include "../firmware/FarmeLink/FarmeLink.ino"

static void reset_globals() {
  server = ESP8266WebServer(80);
  uno = SoftwareSerial(PIN_LINK_RX, PIN_LINK_TX);
  dht11 = DHTStable();
  bridge = farme::Link();
  parser = farme::Parser();
  last_dht_ms = 0;
}

static double probe(int what) {
  switch (what) {
    case sim::PR_MODE: return bridge.telemetry().mode;
    case sim::PR_STATE: return bridge.telemetry().state;
    case sim::PR_CMD: return bridge.last_code;
    case sim::PR_LINK_OK: return bridge.link_ok(sim::g_rig ? (uint32_t)(sim::g_rig->mcu[sim::ESP].now / 1000ULL) : 0) ? 1 : 0;
    case sim::PR_FRAMES_OK: return parser.ok;
    case sim::PR_FRAMES_BAD: return parser.bad_crc + parser.bad_syntax + parser.overlong;
    case sim::PR_TEMP_C: return dht11.getTemperature();
    case sim::PR_HUM: return dht11.getHumidity();
    default: return -1;
  }
}
static const char* ui() { return WEB_UI; }
}  // namespace link_unit

const sim::SketchIface sim::SK_LINK = {"FarmeLink", link_unit::reset_globals, link_unit::setup, link_unit::loop,
                                       link_unit::probe, link_unit::ui};
