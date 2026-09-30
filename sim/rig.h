// rig.h - one simulated FARM-E: the physical world, an Arduino UNO and a
// NodeMCU (ESP8266) joined by a two-wire software UART, and a Wi-Fi client
// (the phone) that sends HTTP requests to the NodeMCU.
//
// Each board runs its sketch on its own fiber with its own clock. A board's
// clock moves only when the sketch spends time (a delay(), a digitalWrite(),
// a byte shifted out of a software UART, a pulseIn() waiting for an echo...),
// so blocking code blocks exactly as it would on the real hardware. The
// scheduler always resumes the board that is furthest behind, and a board
// hands control back as soon as it gets more than QUANTUM_US ahead of the
// other one. With QUANTUM_US much shorter than one UART byte, no board can
// ever read a byte "before" the other board has sent it.
#pragma once
#include <stdint.h>
#include "world.h"
#include "sketch.h"

namespace sim {

static const uint64_t QUANTUM_US = 200;
enum BoardId { UNO = 0, ESP = 1 };

// One direction of the UART wire: bytes with their arrival time at the receiver.
struct Wire {
  struct B { uint64_t at; uint8_t v; uint8_t garbled; };
  static const uint32_t CAP = 2048;
  B q[CAP];
  uint32_t head = 0, count = 0;
  uint32_t sent = 0;
  void clear() { head = count = 0; sent = 0; }
  void push(uint8_t v, uint64_t at, bool garbled) {
    if (count == CAP) { head = (head + 1) % CAP; --count; }
    q[(head + count) % CAP] = B{at, v, (uint8_t)garbled};
    ++count; ++sent;
  }
};

struct Mcu {
  int board = UNO;
  int slot = 0;
  uint64_t now = 0;
  uint8_t mode[40] = {}, level[40] = {};

  // software UART (the link between the two boards)
  bool ss_on = false;
  uint32_t ss_baud = 0;
  uint32_t ss_timeout_ms = 1000;
  uint8_t rx[64]; int rx_head = 0, rx_n = 0;
  Wire* in = nullptr;
  Wire* out = nullptr;
  uint64_t tx_iv_s[8] = {}, tx_iv_e[8] = {}; int tx_iv_n = 0;  // recent transmit windows
  uint32_t rx_bytes = 0, rx_overflow = 0, rx_collide = 0, rx_garbled = 0;

  // hardware UART: what a Serial Monitor on this board would show
  static const uint32_t LOG_CAP = 32768;
  char log[LOG_CAP];
  uint32_t log_total = 0;
  uint32_t hw_baud = 0;
  uint64_t hw_busy_until = 0;
  int hw_buf = 64;

  // servos (Servo library): up to 4 channels
  int servo_pin[4] = {-1, -1, -1, -1};
  double servo_deg[4] = {};

  // watchdog, EEPROM, resets
  bool wdt_on = false; uint32_t wdt_ms = 0; uint64_t wdt_last = 0;
  bool reset_pending = false; uint32_t resets = 0;
  uint8_t eeprom[1024];

  // bookkeeping
  uint64_t loops = 0, loop_start = 0, loop_max_us = 0;
  uint64_t cpu_busy_us = 0;

  void spend(uint64_t us);          // the sketch used `us` of CPU time
  void advance_to(uint64_t t);      // idle until board time t (yields as needed)
  void pump_rx();                   // move arrived UART bytes into the RX buffer
  uint64_t next_rx_arrival() const; // earliest byte still on the wire (or ~0)
  bool transmitting_during(uint64_t s, uint64_t e) const;
  void note_tx(uint64_t s, uint64_t e);
  void log_char(char c);
};

struct HttpReq {
  int id;
  uint64_t t_us, done_us;
  int state;       // 0 queued, 1 answered
  int code;
  uint32_t body_gen, body_off, body_len;
  char url[192];
  char ctype[40];
};

struct Rig {
  World world;
  Mcu mcu[2];
  Wire to_uno, to_esp;
  const SketchIface* sk[2] = {nullptr, nullptr};
  int index = 0;
  uint64_t run_until = 0;
  bool booted = false;
  bool battery_divider = true;   // A3 divider fitted
  uint64_t ap_up_us = ~0ULL;     // when the phone can reach the access point

  static const int MAX_HTTP = 256;
  HttpReq http[MAX_HTTP];
  int nhttp = 0, next_id = 1;
  static const uint32_t BODY_CAP = 196608;
  char body[BODY_CAP];
  uint32_t body_used = 0, body_gen = 1;

  void power_on(const WorldParams& p, uint64_t seed);
  void run(uint64_t until_us);
  int http_enqueue(const char* url, uint64_t at_us);
  HttpReq* http_find(int id);
  void cpu_reset(int board);
  void sync_uno_outputs();  // push the UNO's pin state into the world
};

extern Rig* g_rig;
extern Mcu* g_mcu;
Rig* rig_at(int h);
int rig_count();

}  // namespace sim
