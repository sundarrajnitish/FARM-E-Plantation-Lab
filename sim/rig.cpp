// rig.cpp - boards, the UART wire, the phone's HTTP queue and the scheduler.
#include "rig.h"
#include "fiber.h"
#include "rt.h"

namespace sim {

Rig* g_rig = nullptr;
Mcu* g_mcu = nullptr;

static const int MAX_RIGS = 4;
Rig g_rigs[MAX_RIGS];

// ------------------------------------------------------------------ Mcu
void Mcu::spend(uint64_t us) {
  now += us;
  cpu_busy_us += us;
  if (wdt_on && now - wdt_last > (uint64_t)wdt_ms * 1000ULL) reset_pending = true;
  const Mcu& o = g_rig->mcu[1 - board];
  if (reset_pending || now >= g_rig->run_until || now > o.now + QUANTUM_US) fiber_yield();
}

void Mcu::advance_to(uint64_t t) {
  if (t <= now) return;
  uint64_t d = t - now;
  cpu_busy_us -= 0;  // idle time is not CPU time
  now += d;
  if (wdt_on && now - wdt_last > (uint64_t)wdt_ms * 1000ULL) reset_pending = true;
  const Mcu& o = g_rig->mcu[1 - board];
  if (reset_pending || now >= g_rig->run_until || now > o.now + QUANTUM_US) fiber_yield();
}

bool Mcu::transmitting_during(uint64_t s, uint64_t e) const {
  int n = tx_iv_n < 8 ? tx_iv_n : 8;
  for (int i = 0; i < n; ++i)
    if (tx_iv_s[i] < e && s < tx_iv_e[i]) return true;
  return false;
}

void Mcu::note_tx(uint64_t s, uint64_t e) {
  int k = tx_iv_n % 8;
  tx_iv_s[k] = s; tx_iv_e[k] = e;
  ++tx_iv_n;
  if (tx_iv_n > 1000000) tx_iv_n = 8 + tx_iv_n % 8;
}

uint64_t Mcu::next_rx_arrival() const {
  return (in && in->count) ? in->q[in->head].at : ~0ULL;
}

void Mcu::pump_rx() {
  if (!in) return;
  while (in->count && in->q[in->head].at <= now) {
    Wire::B b = in->q[in->head];
    in->head = (in->head + 1) % Wire::CAP;
    --in->count;
    if (!ss_on) continue;
    uint64_t byte_us = ss_baud ? 10000000ULL / ss_baud : 0;
    if (transmitting_during(b.at > byte_us ? b.at - byte_us : 0, b.at)) {
      ++rx_collide;
      g_rig->world.event_at(now, EV_LINK_DROP, board, 1);
      continue;
    }
    uint8_t v = b.v;
    if (b.garbled) { ++rx_garbled; v = (uint8_t)(0x80 | (v * 37 + 11)); }
    if (rx_n >= 64) {
      ++rx_overflow;
      g_rig->world.event_at(now, EV_LINK_DROP, board, 0);
      continue;
    }
    rx[(rx_head + rx_n) % 64] = v;
    ++rx_n; ++rx_bytes;
  }
}

void Mcu::log_char(char c) {
  log[log_total % LOG_CAP] = c;
  ++log_total;
}

// ------------------------------------------------------------------ Rig
static void mcu_main(int slot) {
  Rig* r = &g_rigs[slot / 2];
  Mcu* m = &r->mcu[slot % 2];
  const SketchIface* sk = r->sk[slot % 2];
  // power-on / reset to first instruction of setup()
  m->advance_to(m->now + (m->board == UNO ? 65000 : 120000));
  sk->reset();
  sk->setup();
  const uint64_t overhead = m->board == UNO ? 2 : 30;  // main() loop, serialEventRun / esp yield
  for (;;) {
    m->loop_start = m->now;
    sk->loop();
    m->spend(overhead);
    uint64_t d = m->now - m->loop_start;
    if (d > m->loop_max_us) m->loop_max_us = d;
    ++m->loops;
  }
}

void Rig::power_on(const WorldParams& p, uint64_t seed) {
  world.reset(p, seed);
  to_uno.clear();
  to_esp.clear();
  nhttp = 0; next_id = 1; body_used = 0; ++body_gen;
  for (int i = 0; i < MAX_HTTP; ++i) http[i].id = 0;
  ap_up_us = ~0ULL;
  for (int b = 0; b < 2; ++b) {
    Mcu& m = mcu[b];
    m.board = b;
    m.slot = index * 2 + b;
    m.now = 0;
    m.log_total = 0;
    m.loops = 0; m.loop_max_us = 0; m.cpu_busy_us = 0;
    m.resets = 0;
    for (int i = 0; i < 1024; ++i) m.eeprom[i] = 0xFF;
    m.in = b == UNO ? &to_uno : &to_esp;
    m.out = b == UNO ? &to_esp : &to_uno;
    cpu_reset(b);
    m.resets = 0;
  }
  booted = true;
}

void Rig::cpu_reset(int b) {
  Mcu& m = mcu[b];
  for (int i = 0; i < 40; ++i) { m.mode[i] = 0; m.level[i] = 0; }
  m.ss_on = false; m.ss_baud = 0; m.rx_head = m.rx_n = 0; m.tx_iv_n = 0;
  m.rx_bytes = m.rx_overflow = m.rx_collide = m.rx_garbled = 0;
  m.hw_baud = 0; m.hw_busy_until = 0; m.hw_buf = b == UNO ? 64 : 128;
  for (int i = 0; i < 4; ++i) { m.servo_pin[i] = -1; m.servo_deg[i] = 0; }
  m.wdt_on = false;
  if (m.reset_pending) { ++m.resets; world.event_at(m.now, EV_WDT, b); }
  m.reset_pending = false;
  if (b == UNO) {
    world.step_to(m.now);
    world.set_motors(0, 0);
    world.set_pump(false);
    world.set_servo(world.servo_deg, false);
  } else {
    ap_up_us = ~0ULL;
  }
  fiber_setup(m.slot, mcu_main, m.slot);
}

void Rig::run(uint64_t until) {
  run_until = until;
  for (;;) {
    int b = mcu[UNO].now <= mcu[ESP].now ? UNO : ESP;
    Mcu& m = mcu[b];
    if (m.now >= until) break;
    if (m.reset_pending) cpu_reset(b);
    g_rig = this;
    g_mcu = &m;
    fiber_resume(m.slot);
  }
  g_rig = this;
  world.step_to(until);
  g_mcu = nullptr;
}

void Rig::sync_uno_outputs() {
  Mcu& m = mcu[UNO];
  world.step_to(m.now);
  auto on = [&](int p) { return m.mode[p] == 1 && m.level[p]; };
  auto side = [&](bool a, bool b) { return a && !b ? 1 : (!a && b ? -1 : (a && b ? 2 : 0)); };
  world.set_motors(side(on(PIN_LM1), on(PIN_LM2)), side(on(PIN_RM1), on(PIN_RM2)));
  world.set_pump(on(PIN_PUMP));
}

int Rig::http_enqueue(const char* url, uint64_t at_us) {
  HttpReq& r = http[next_id % MAX_HTTP];
  r.id = next_id++;
  r.t_us = at_us; r.done_us = 0; r.state = 0; r.code = 0;
  r.body_gen = 0; r.body_off = 0; r.body_len = 0;
  size_t n = strlen(url);
  if (n > sizeof(r.url) - 1) n = sizeof(r.url) - 1;
  memcpy(r.url, url, n); r.url[n] = 0;
  r.ctype[0] = 0;
  ++nhttp;
  return r.id;
}

HttpReq* Rig::http_find(int id) {
  if (id <= 0) return nullptr;
  HttpReq& r = http[id % MAX_HTTP];
  return r.id == id ? &r : nullptr;
}

Rig* rig_at(int h) { return (h >= 0 && h < MAX_RIGS) ? &g_rigs[h] : nullptr; }
int rig_count() { return MAX_RIGS; }

}  // namespace sim
