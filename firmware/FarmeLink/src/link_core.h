// link_core.h - FARM-E's NodeMCU side: turns web requests into commands for
// the UNO, delivers them reliably over the software UART, and keeps the
// latest telemetry for the phone.
//
// Delivery is stop-and-wait: one command is in flight at a time, carrying a
// sequence number. The UNO acknowledges by echoing the number in its next
// telemetry frame (it sends one within ~20 ms of any command); if no
// acknowledgement arrives within RETRY_MS the same frame is sent again, and the
// UNO recognises and ignores repeats. A STOP jumps the queue.
//
// Plain C++ with no Arduino calls, like drive_core: tested on a PC and in the
// simulator.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "farme_proto.h"

namespace farme {

struct Telemetry {
  bool valid = false;
  uint16_t ack = 0;
  uint8_t state = 0, mode = 0, fault = 0, row = 0;
  uint16_t flags = 0, hills = 0;
  int16_t ground_mm = -1, tank_mL = 0;
  uint16_t batt_dV = 0;
};

class Link {
 public:
  static const uint8_t QUEUE = 8;
  static const uint32_t RETRY_MS = 150, HEARTBEAT_MS = 300, LINK_TIMEOUT_MS = 1000;
  static const uint8_t MAX_TRIES = 8;

  void begin(uint32_t now);

  // Queue a command; returns its sequence number (0 if the queue is full).
  // A DRIVE replaces any DRIVE still waiting; a STOP clears the queue first.
  uint16_t command(uint8_t code, int32_t a, int32_t b, uint32_t now);

  // A frame from the UNO
  void on_frame(const Frame& fr, uint32_t now);

  // Next frame to put on the wire now, if any
  bool next_tx(Frame* out, uint32_t now);

  void set_env(int16_t temp_dC, int16_t hum, bool ok) { temp_dC_ = temp_dC; hum_ = hum; env_ok_ = ok; }

  size_t status_json(char* out, size_t cap, uint32_t now) const;
  bool link_ok(uint32_t now) const { return tel_.valid && now - last_rx_ <= LINK_TIMEOUT_MS; }
  const Telemetry& telemetry() const { return tel_; }
  uint16_t pending() const { return n_; }
  uint32_t sent = 0, retries = 0, acked = 0, gave_up = 0, frames_in = 0;
  uint32_t last_ack_latency_ms = 0;
  uint8_t last_code = 0;

 private:
  struct Pending { uint16_t seq; uint8_t code; int32_t a, b; uint32_t queued, last_sent; uint8_t tries; };
  Pending q_[QUEUE];
  uint8_t n_ = 0;
  uint16_t seq_ = 0;
  uint32_t last_rx_ = 0, last_hb_ = 0;
  Telemetry tel_;
  int16_t temp_dC_ = -999, hum_ = -1;
  bool env_ok_ = false;
  void pop();
};

// JSON helpers (no printf on the ESP8266 path)
size_t json_put(char* out, size_t pos, size_t cap, const char* s);
size_t json_int(char* out, size_t pos, size_t cap, int32_t v);

}  // namespace farme
