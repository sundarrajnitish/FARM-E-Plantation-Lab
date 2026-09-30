// Host unit tests for the controller cores: protocol, drive controller, link.
// Build and run: make -C firmware/test   (g++, -Wall -Werror, ASan + UBSan)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include "../FarmeDrive/src/drive_core.h"
#include "../FarmeDrive/src/farme_proto.h"
#include "../FarmeLink/src/link_core.h"

using namespace farme;

static int g_fail = 0, g_checks = 0;
#define CHECK(c)                                                         \
  do {                                                                   \
    ++g_checks;                                                          \
    if (!(c)) { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } \
  } while (0)
#define TEST(name) static void name()
#define RUN(name) do { std::printf("%s\n", #name); name(); } while (0)

// ---------------------------------------------------------------- helpers
static bool roundtrip(const Frame& in, Frame* out) {
  char line[96];
  size_t n = encode(in, line, sizeof line);
  if (!n) return false;
  Parser p;
  bool got = false;
  for (size_t i = 0; i < n; ++i) if (p.feed((uint8_t)line[i])) { got = true; *out = p.frame(); }
  return got;
}

static Frame cmd(uint16_t seq, uint8_t code, int32_t a = 0, int32_t b = 0) {
  Frame f; f.type = 'C'; f.n = 4; f.f[0] = seq; f.f[1] = code; f.f[2] = a; f.f[3] = b;
  return f;
}

// A tiny "world" for the drive controller: 1-D position along a bed that
// ends at `edge_x`; the sensor sits 130 mm ahead of the centre, 55 mm up.
struct Bench {
  Drive d;
  uint32_t t = 0;
  double x = 160, speed = 240, v = 0;  // mm, mm/s
  double edge_x = 2400;
  uint16_t seq = 0;
  bool ground_on = true;
  uint16_t batt = 11700;
  void begin(DriveConfig c = DriveConfig()) { d.begin(c, 0); }
  void send(uint8_t code, int32_t a = 0, int32_t b = 0) { seq = seq_next(seq); d.on_frame(cmd(seq, code, a, b), t); }
  void step(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
      ++t;
      Inputs in;
      in.now_ms = t;
      if (t % 30 == 0) {
        in.ping = true;
        double sx = x + 130;
        in.ground_mm = !ground_on ? -1 : (sx < edge_x ? 55 : 805);
      }
      in.batt_mV = batt;
      d.tick(in);
      // tracks with the simulator's spin-up (80 ms) and stopping (20-50 ms) lags
      const Outputs& o = d.out();
      double target = (o.left == 1 && o.right == 1) ? speed : ((o.left == -1 && o.right == -1) ? -speed : 0);
      double tau = target != 0 ? 80.0 : (o.left == 2 ? 20.0 : 50.0);
      v += (target - v) / tau;
      x += v / 1000.0;
    }
  }
};

// ---------------------------------------------------------------- protocol
TEST(crc_known_vectors) {
  const char* s = "123456789";
  CHECK(crc8((const uint8_t*)s, 9) == 0xF4);  // CRC-8/SMBUS check value
  CHECK(crc8(nullptr, 0) == 0);
}

TEST(encode_parse_roundtrip) {
  Frame f; f.type = 'T'; f.n = 10;
  for (int i = 0; i < 10; ++i) f.f[i] = (i % 2 ? -1 : 1) * i * 12345;
  Frame g;
  CHECK(roundtrip(f, &g));
  CHECK(g.type == 'T' && g.n == 10);
  for (int i = 0; i < 10; ++i) CHECK(g.f[i] == f.f[i]);
  f.n = 0; f.type = 'H';
  CHECK(roundtrip(f, &g) && g.n == 0 && g.type == 'H');
  f.n = 1; f.f[0] = -2147483647;
  CHECK(roundtrip(f, &g) && g.f[0] == -2147483647);
}

TEST(parser_rejects_corruption) {
  Frame f = cmd(7, CMD_DRIVE, DIR_FWD, 400);
  char line[96];
  size_t n = encode(f, line, sizeof line);
  // flip every bit of every payload byte: the CRC must catch every single-bit error
  for (size_t i = 1; i + 4 < n; ++i) {
    for (int b = 0; b < 8; ++b) {
      char bad[96]; std::memcpy(bad, line, n);
      bad[i] ^= (char)(1 << b);
      if (bad[i] == '$' || bad[i] == '\n' || bad[i] == '\r') continue;
      Parser p; bool got = false;
      for (size_t k = 0; k < n; ++k) got |= p.feed((uint8_t)bad[k]);
      CHECK(!got);
    }
  }
  Parser p;
  const char* junk = "garbage$C,1,2*00\n$$$\n$C,1*ZZ\n";
  bool got = false;
  for (const char* c = junk; *c; ++c) got |= p.feed((uint8_t)*c);
  CHECK(!got);
  CHECK(p.bad_crc + p.bad_syntax >= 2);
}

TEST(parser_resyncs_after_noise_and_overlong) {
  Parser p;
  for (int i = 0; i < 200; ++i) p.feed('x');
  p.feed('$');
  for (int i = 0; i < 100; ++i) p.feed('1');
  CHECK(p.overlong == 1);
  char line[96];
  size_t n = encode(cmd(3, CMD_STOP), line, sizeof line);
  bool got = false;
  for (size_t i = 0; i < n; ++i) got |= p.feed((uint8_t)line[i]);
  CHECK(got && p.frame().f[1] == CMD_STOP);
}

TEST(parser_fuzz_never_crashes) {
  std::mt19937 rng(42);
  Parser p;
  for (int i = 0; i < 200000; ++i) p.feed((uint8_t)(rng() & 0xFF));
  CHECK(p.ok < 5);  // a random valid frame needs a valid CRC and syntax
}

TEST(encode_refuses_small_buffer) {
  Frame f; f.type = 'T'; f.n = 10;
  for (int i = 0; i < 10; ++i) f.f[i] = 2000000000;
  char small[20];
  CHECK(encode(f, small, sizeof small) == 0);
}

// ---------------------------------------------------------------- drive
TEST(boot_is_safe) {
  Bench b; b.begin();
  b.step(2000);
  CHECK(b.d.mode() == MODE_IDLE);
  CHECK(b.d.out().left == 0 && b.d.out().right == 0 && !b.d.out().pump);
  CHECK(b.x == 160);  // nothing moves until asked
  CHECK(b.d.baseline_mm() == 55);
}

TEST(auto_plants_evenly_spaced_hills_and_stops_at_edge) {
  Bench b; DriveConfig c; c.rows = 1; b.begin(c);
  b.step(500);
  b.send(CMD_AUTO);
  double last = -1; int hills = 0;
  double gaps[64]; int ng = 0;
  for (int i = 0; i < 60000 && b.d.state() != ST_DONE; ++i) {
    uint16_t h = b.d.hills();
    b.step(1);
    if (b.d.hills() != h) { if (last >= 0 && ng < 64) gaps[ng++] = b.x - last; last = b.x; ++hills; }
  }
  CHECK(b.d.state() == ST_DONE);
  CHECK(hills >= 8);
  for (int i = 0; i < ng; ++i) CHECK(gaps[i] > 245 && gaps[i] < 255);
  CHECK(b.x + 130 < b.edge_x + 20);  // stopped with the sensor within 2 cm past the edge
  CHECK(b.d.flags() & FL_EDGE);
}

TEST(water_dose_and_tank_accounting) {
  Bench b; DriveConfig c; c.rows = 1; c.water_mL = 20; c.tank_mL = 50; b.begin(c);
  b.step(500);
  b.send(CMD_AUTO);
  uint32_t pump_ms = 0;
  for (int i = 0; i < 60000 && b.d.state() != ST_DONE; ++i) { b.step(1); if (b.d.out().pump) ++pump_ms; }
  // 20 + 20 + 10 mL (the remaining 10 mL is still given), then nothing
  CHECK(pump_ms > 2700 && pump_ms < 2850);
  CHECK(b.d.tank_mL() == 0);
  CHECK(b.d.flags() & FL_TANK_LOW);
  b.send(CMD_REFILL, 0);
  CHECK(b.d.tank_mL() == 50 && !(b.d.flags() & FL_TANK_LOW));
}

TEST(manual_deadman_and_edge_guard) {
  Bench b; b.begin();
  b.step(500);
  b.send(CMD_DRIVE, DIR_FWD, 400);
  CHECK(b.d.mode() == MODE_MANUAL);
  b.step(300);
  CHECK(b.d.out().left == 1);
  b.step(200);
  CHECK(b.d.out().left == 0);  // hold time ran out: stops by itself
  // drive toward the edge with a stream of refreshes, like a held button
  b.x = b.edge_x - 400;
  for (int i = 0; i < 40; ++i) { b.send(CMD_DRIVE, DIR_FWD, 450); b.step(200); }
  CHECK(b.x + 130 < b.edge_x + 25);
  CHECK(b.d.flags() & FL_GUARD);
  b.send(CMD_DRIVE, DIR_BACK, 450);
  b.step(100);
  CHECK(b.d.out().left == -1);  // backing away is always allowed
}

TEST(duplicate_commands_are_ignored) {
  Bench b; b.begin();
  b.step(500);
  b.d.on_frame(cmd(5, CMD_MANUAL), b.t);
  b.d.on_frame(cmd(6, CMD_SEED), b.t);
  b.step(500);
  CHECK(b.d.hills() == 1);
  b.d.on_frame(cmd(6, CMD_SEED), b.t);  // retry of the same frame
  b.step(500);
  CHECK(b.d.hills() == 1);
  CHECK(b.d.last_seq() == 6);
}

TEST(sensor_loss_faults_calibration_and_stops_motion) {
  Bench b; b.ground_on = false; b.begin();
  b.step(300);
  b.send(CMD_AUTO);
  b.step(2000);
  CHECK(b.d.mode() == MODE_FAULT && b.d.fault() == FAULT_SENSOR);
  CHECK(b.d.out().left == 0);
  b.send(CMD_AUTO);  // refused until cleared
  CHECK(b.d.mode() == MODE_FAULT);
  b.send(CMD_CLEAR);
  CHECK(b.d.mode() == MODE_IDLE);

  // echo lost while driving: stops within two pings
  Bench c; c.begin(); c.step(500);
  DriveConfig cfg; cfg.rows = 1;
  c.send(CMD_AUTO);
  c.step(3000);
  CHECK(c.d.state() == ST_DRIVE || c.d.state() == ST_SETTLE || c.d.state() == ST_SEED || c.d.state() == ST_WATER);
  while (c.d.state() != ST_DRIVE) c.step(1);
  c.step(100);
  double x0 = c.x;
  c.ground_on = false;
  c.step(200);
  CHECK(c.x - x0 < 30);
  CHECK(c.d.state() == ST_BACKOFF || c.d.state() == ST_DONE || c.d.state() == ST_TURN1);
}

TEST(serpentine_rows_and_pause_resume) {
  Bench b; b.begin(); b.step(500);
  b.send(CMD_AUTO);
  int max_row = 0;
  bool paused = false;
  for (int i = 0; i < 300000 && b.d.state() != ST_DONE; ++i) {
    b.step(1);
    if (b.d.row() > max_row) max_row = b.d.row();
    // in 1-D the "turns" do nothing; after a turn we are facing the same
    // edge, so move the bench edge back to emulate a fresh row
    if (b.d.state() == ST_TURN1) b.x = 1000;  // turned away from the edge
    if (b.d.state() == ST_TURN2) b.x = 160;
    if (!paused && b.d.hills() == 3 && b.d.state() == ST_DRIVE) {
      b.send(CMD_PAUSE); b.step(1000);
      CHECK(b.d.state() == ST_PAUSED && b.d.out().left == 0);
      b.send(CMD_AUTO); paused = true;
      CHECK(b.d.state() == ST_DRIVE);
    }
  }
  CHECK(paused);
  CHECK(max_row == 2);
  CHECK(b.d.state() == ST_DONE && (b.d.flags() & FL_DONE));
}

TEST(low_battery_pauses_auto) {
  Bench b; b.begin(); b.step(500);
  b.send(CMD_AUTO);
  b.step(1000);
  b.batt = 10200;
  b.step(4000);
  CHECK(b.d.state() == ST_PAUSED && (b.d.flags() & FL_LOW_BATT));
  b.send(CMD_AUTO);
  CHECK(b.d.state() == ST_PAUSED);  // cannot resume on a flat battery
}

TEST(battery_compensation) {
  Drive d; DriveConfig c; d.begin(c, 0);
  Inputs in; in.now_ms = 1; in.batt_mV = 11700; d.tick(in);
  CHECK(d.speed_mm_s() == 240);
  in.now_ms = 2; in.batt_mV = 12600; d.tick(in);
  CHECK(d.speed_mm_s() > 255 && d.speed_mm_s() < 265);
  in.now_ms = 3; in.batt_mV = 10800; d.tick(in);
  CHECK(d.speed_mm_s() > 215 && d.speed_mm_s() < 225);
}

TEST(config_ranges) {
  DriveConfig c;
  CHECK(c.set(P_SPACING_MM, 300) && c.spacing_mm == 300);
  CHECK(!c.set(P_SPACING_MM, 20));
  CHECK(!c.set(P_ROWS, 0));
  CHECK(!c.set(99, 1));
  CHECK(!c.set(0, 1));
  for (uint8_t p = 1; p < P_COUNT; ++p) CHECK(c.set(p, c.get(p)));
}

TEST(manual_actions_do_not_interrupt_auto) {
  Bench b; b.begin(); b.step(500);
  b.send(CMD_AUTO); b.step(100);
  b.send(CMD_DRIVE, DIR_LEFT, 500);
  CHECK(b.d.mode() == MODE_AUTO);
  b.send(CMD_STOP);
  CHECK(b.d.mode() == MODE_IDLE);
  b.step(50);
  CHECK(b.d.out().left == 0 && !b.d.out().pump);
}

// ---------------------------------------------------------------- link
static Frame tel(uint16_t ack) {
  Frame t; t.type = 'T'; t.n = 10; t.f[0] = ack;
  return t;
}

TEST(link_stop_and_wait_with_retries) {
  Link l; l.begin(0);
  uint16_t s1 = l.command(CMD_AUTO, 0, 0, 0);
  uint16_t s2 = l.command(CMD_SEED, 0, 0, 0);
  CHECK(s1 && s2 && s1 != s2);
  Frame f;
  CHECK(l.next_tx(&f, 0) && f.type == 'C' && f.f[0] == s1);
  CHECK(!l.next_tx(&f, 10));          // waiting for the ack
  CHECK(l.next_tx(&f, 200) && f.f[0] == s1 && l.retries == 1);
  l.on_frame(tel(s1), 210);
  CHECK(l.acked == 1);
  CHECK(l.next_tx(&f, 211) && f.f[0] == s2);
  l.on_frame(tel(s2), 230);
  CHECK(l.pending() == 0);
  CHECK(l.next_tx(&f, 600) && f.type == 'H');
}

TEST(link_gives_up_after_max_tries) {
  Link l; l.begin(0);
  l.command(CMD_AUTO, 0, 0, 0);
  Frame f;
  uint32_t t = 0;
  for (int i = 0; i < 40; ++i) { l.next_tx(&f, t); t += 200; }
  CHECK(l.gave_up == 1 && l.pending() == 0);
}

TEST(link_drive_coalesces_and_stop_jumps_queue) {
  Link l; l.begin(0);
  l.command(CMD_AUTO, 0, 0, 0);
  Frame f;
  l.next_tx(&f, 0);                    // AUTO in flight
  uint16_t d1 = l.command(CMD_DRIVE, DIR_FWD, 450, 1);
  uint16_t d2 = l.command(CMD_DRIVE, DIR_LEFT, 450, 2);
  CHECK(d1 == d2 && l.pending() == 2);
  l.command(CMD_STOP, 0, 0, 3);
  CHECK(l.pending() == 1);
  CHECK(l.next_tx(&f, 4) && f.f[1] == CMD_STOP);
}

TEST(link_status_json) {
  Link l; l.begin(0);
  char b[400];
  l.status_json(b, sizeof b, 0);
  CHECK(std::strstr(b, "\"link\":false") != nullptr);
  Frame t = tel(0); t.f[1] = ST_DRIVE; t.f[2] = MODE_AUTO; t.f[5] = 4; t.f[8] = 118;
  l.on_frame(t, 100);
  l.set_env(283, 61, true);
  l.status_json(b, sizeof b, 150);
  CHECK(std::strstr(b, "\"state\":\"DRIVE\"") && std::strstr(b, "\"hills\":4") && std::strstr(b, "\"temp_C\":28"));
  CHECK(std::strstr(b, "\"link\":true"));
  l.status_json(b, 30, 150);  // truncated, still terminated
  CHECK(std::strlen(b) < 30);
}

// Drive and Link talking through a lossy channel
TEST(end_to_end_over_lossy_channel) {
  std::mt19937 rng(7);
  Link l; l.begin(0);
  Bench b; b.begin();
  Parser to_drive, to_link;
  int delivered_cmds = 0;
  l.command(CMD_MANUAL, 0, 0, 0);
  int queued = 0;
  for (uint32_t t = 1; t < 60000; ++t) {
    Frame f;
    if (queued < 20 && l.pending() < Link::QUEUE - 1) { l.command(CMD_SEED, 0, 0, t); l.command(CMD_REFILL, 100 + queued, 0, t); ++queued; }
    if (l.next_tx(&f, t)) {
      char line[96]; size_t n = encode(f, line, sizeof line);
      for (size_t i = 0; i < n; ++i) {
        uint8_t c = (uint8_t)line[i];
        if (rng() % 100 < 2) c ^= 0x10;  // 2 % of bytes corrupted
        if (to_drive.feed(c)) { b.d.on_frame(to_drive.frame(), b.t); ++delivered_cmds; }
      }
    }
    b.step(1);
    if (b.d.telemetry_due(b.t)) {
      char line[96]; size_t n = encode(b.d.telemetry(b.t), line, sizeof line);
      for (size_t i = 0; i < n; ++i) {
        uint8_t c = (uint8_t)line[i];
        if (rng() % 100 < 2) c ^= 0x04;
        if (to_link.feed(c)) l.on_frame(to_link.frame(), t);
      }
    }
  }
  std::printf("  sent %u retries %u acked %u gave_up %u hills %u tank %d pend %u\n", l.sent, l.retries, l.acked, l.gave_up, b.d.hills(), b.d.tank_mL(), l.pending());
  CHECK(l.pending() == 0 && l.gave_up == 0);
  CHECK(l.retries > 0);
  CHECK(b.d.hills() == 20);  // every SEED executed exactly once despite retries
  CHECK(b.d.tank_mL() == 119);
}

int main() {
  RUN(crc_known_vectors);
  RUN(encode_parse_roundtrip);
  RUN(parser_rejects_corruption);
  RUN(parser_resyncs_after_noise_and_overlong);
  RUN(parser_fuzz_never_crashes);
  RUN(encode_refuses_small_buffer);
  RUN(boot_is_safe);
  RUN(auto_plants_evenly_spaced_hills_and_stops_at_edge);
  RUN(water_dose_and_tank_accounting);
  RUN(manual_deadman_and_edge_guard);
  RUN(duplicate_commands_are_ignored);
  RUN(sensor_loss_faults_calibration_and_stops_motion);
  RUN(serpentine_rows_and_pause_resume);
  RUN(low_battery_pauses_auto);
  RUN(battery_compensation);
  RUN(config_ranges);
  RUN(manual_actions_do_not_interrupt_auto);
  RUN(link_stop_and_wait_with_retries);
  RUN(link_gives_up_after_max_tries);
  RUN(link_drive_coalesces_and_stop_jumps_queue);
  RUN(link_status_json);
  RUN(end_to_end_over_lossy_channel);
  std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
