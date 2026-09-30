// FarmeLink - FARM-E, NodeMCU side: Wi-Fi access point, the phone page,
// the JSON API, the DHT11, and reliable delivery of commands to the UNO.
//
// Nothing here blocks for long: the web server answers from cached values
// and the UART link is fed one frame at a time.
//
// Endpoints
//   /                     the control page (src/web_ui.h)
//   /api/status           JSON: link, mode, state, hills, row, tank, battery, air...
//   /api/cmd?c=...        auto | pause | manual | stop | clear | seed | water&ms= |
//                         refill&ml= | drive&d=F|B|L|R|S&ms= | set&p=&v= | sweep&arm=&deg=
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <DHTStable.h>
#include "config.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h not found: using secrets.example.h - set your own Wi-Fi password"
#include "secrets.example.h"
#endif
#include "src/farme_proto.h"
#include "src/link_core.h"
#include "src/web_ui.h"

ESP8266WebServer server(80);
SoftwareSerial uno(PIN_LINK_RX, PIN_LINK_TX);
DHTStable dht11;
farme::Link bridge;
farme::Parser parser;
uint32_t last_dht_ms = 0;

// ---- UART
void pump_link() {
  farme::Frame f;
  uint32_t now = millis();
  while (uno.available()) {
    if (parser.feed((uint8_t)uno.read())) bridge.on_frame(parser.frame(), now);
  }
  if (bridge.next_tx(&f, now)) {
    char line[farme::MAX_LINE + 8];
    size_t n = farme::encode(f, line, sizeof(line));
    uno.write((const uint8_t*)line, n);
  }
}

// ---- HTTP helpers
static int32_t arg_int(const char* name, int32_t dflt) {
  if (!server.hasArg(name)) return dflt;
  return (int32_t)server.arg(name).toInt();
}

static void reply_json(bool ok, uint16_t seq) {
  char b[48];
  size_t p = farme::json_put(b, 0, sizeof(b), "{\"ok\":");
  p = farme::json_put(b, p, sizeof(b), ok ? "true" : "false");
  p = farme::json_put(b, p, sizeof(b), ",\"seq\":");
  p = farme::json_int(b, p, sizeof(b), seq);
  farme::json_put(b, p, sizeof(b), "}");
  server.send(ok ? 200 : 400, "application/json", b);
}

static uint8_t param_id(const String& p) {
  static const char* const NAMES[] = {"", "spacing", "rows", "rowgap", "water", "seedms",
                                      "speed", "turn", "edge", "tank", "pump"};
  for (uint8_t i = 1; i < farme::P_COUNT; ++i) if (p == NAMES[i]) return i;
  return 0;
}

static uint8_t dir_id(const String& d) {
  if (d == "F") return farme::DIR_FWD;
  if (d == "B") return farme::DIR_BACK;
  if (d == "L") return farme::DIR_LEFT;
  if (d == "R") return farme::DIR_RIGHT;
  return farme::DIR_STOP;
}

// ---- handlers
void handle_root() { server.send_P(200, "text/html", WEB_UI); }

void handle_status() {
  char b[400];
  bridge.status_json(b, sizeof(b), millis());
  server.send(200, "application/json", b);
}

void handle_cmd() {
  String c = server.arg("c");
  uint32_t now = millis();
  uint16_t seq = 0;
  if (c == "auto") seq = bridge.command(farme::CMD_AUTO, 0, 0, now);
  else if (c == "pause") seq = bridge.command(farme::CMD_PAUSE, 0, 0, now);
  else if (c == "manual") seq = bridge.command(farme::CMD_MANUAL, 0, 0, now);
  else if (c == "stop") seq = bridge.command(farme::CMD_STOP, 0, 0, now);
  else if (c == "clear") seq = bridge.command(farme::CMD_CLEAR, 0, 0, now);
  else if (c == "seed") seq = bridge.command(farme::CMD_SEED, 0, 0, now);
  else if (c == "water") seq = bridge.command(farme::CMD_WATER, arg_int("ms", 1500), 0, now);
  else if (c == "refill") seq = bridge.command(farme::CMD_REFILL, arg_int("ml", 0), 0, now);
  else if (c == "drive") seq = bridge.command(farme::CMD_DRIVE, dir_id(server.arg("d")), arg_int("ms", 450), now);
  else if (c == "set") {
    uint8_t p = param_id(server.arg("p"));
    if (p) seq = bridge.command(farme::CMD_SET, p, arg_int("v", -1), now);
  } else if (c == "sweep") seq = bridge.command(farme::CMD_SWEEP, arg_int("arm", 1), arg_int("deg", 90), now);
  reply_json(seq != 0, seq);
  pump_link();  // put the frame on the wire now rather than at the next loop
}

void handle_not_found() { server.send(404, "text/plain", "Not found"); }

// ---- sensors
void read_dht(uint32_t now) {
  if (last_dht_ms && now - last_dht_ms < DHT_PERIOD_MS) return;
  bool ok = dht11.read11(PIN_DHT) == DHTLIB_OK;
  last_dht_ms = now ? now : 1;
  if (ok) bridge.set_env((int16_t)(dht11.getTemperature() * 10), (int16_t)dht11.getHumidity(), true);
}

void setup() {
  digitalWrite(PIN_STATUS_LED, HIGH);  // off (active LOW)
  pinMode(PIN_STATUS_LED, OUTPUT);
  Serial.begin(USB_BAUD);
  uno.begin(LINK_BAUD);
  bridge.begin(millis());

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(FARME_AP_SSID, FARME_AP_PASSWORD)) Serial.println(F("softAP failed: password must be 8-63 characters"));
  Serial.print(F("FARM-E link at http://"));
  Serial.println(WiFi.softAPIP());

  server.on("/", handle_root);
  server.on("/api/status", handle_status);
  server.on("/api/cmd", handle_cmd);
  server.onNotFound(handle_not_found);
  server.begin();
}

void loop() {
  uint32_t now = millis();
  server.handleClient();
  pump_link();
  read_dht(now);
  digitalWrite(PIN_STATUS_LED, bridge.link_ok(now) ? LOW : ((now / 500) & 1));
}
