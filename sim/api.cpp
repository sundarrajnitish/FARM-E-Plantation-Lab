// api.cpp - C interface to the simulator, shared by the Python analysis
// (ctypes, native build) and the website (WebAssembly build).
//
// Up to four rigs run side by side, each a complete robot running
// firmware/FarmeDrive (UNO) and firmware/FarmeLink (NodeMCU).
// Parameter, field and action names are exported so the Python and
// JavaScript wrappers look indices up instead of hard-coding them.
#include "rig.h"
#include "rt.h"

#define API extern "C" __attribute__((visibility("default")))

using namespace sim;

namespace {

Rig* rig(int h) {
  Rig* r = rig_at(h);
  if (!r) return nullptr;
  r->index = h;
  g_rig = r;
  return r;
}

WorldParams g_params[4];
char g_scratch[262144];

// ---- world parameters (set before sim_init)
struct ParamDef { const char* name; double WorldParams::*field; };
const ParamDef PARAMS[] = {
    {"bed_len", &WorldParams::bed_len}, {"bed_w", &WorldParams::bed_w}, {"drop", &WorldParams::drop},
    {"start_x", &WorldParams::start_x}, {"start_y", &WorldParams::start_y}, {"start_heading", &WorldParams::start_heading},
    {"v_nom", &WorldParams::v_nom}, {"track_noise", &WorldParams::track_noise}, {"skid", &WorldParams::skid},
    {"soc0", &WorldParams::soc0}, {"sensor_fwd", &WorldParams::sensor_fwd}, {"sensor_h", &WorldParams::sensor_h},
    {"echo_dropout", &WorldParams::echo_dropout}, {"hopper_seeds", &WorldParams::hopper_seeds},
    {"seed_rate", &WorldParams::seed_rate}, {"tank_mL", &WorldParams::tank_mL}, {"pump_mL_s", &WorldParams::pump_mL_s},
    {"air_T", &WorldParams::air_T}, {"air_RH", &WorldParams::air_RH}, {"dht_fail", &WorldParams::dht_fail},
};
const int NPARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// ---- hills: seeds grouped along the robot's path
struct HillStats {
  int hills = 0;
  double seeds_mean = 0, seeds_sd = 0, extent_mean = 0, spacing_mean = 0, spacing_sd = 0;
  double water_on_hills = 0, water_frac = 0;
  int hills_watered = 0;
};
static const int MAX_HILLS = 512;
float g_hill_x[MAX_HILLS], g_hill_y[MAX_HILLS], g_hill_n[MAX_HILLS], g_hill_ext[MAX_HILLS], g_hill_w[MAX_HILLS];

HillStats hill_stats(const World& w) {
  HillStats s;
  int h = -1;
  double sx = 0, sy = 0; int n = 0;
  float fx = 0, fy = 0;
  auto close = [&]() {
    if (h < 0 || h >= MAX_HILLS) return;
    g_hill_x[h] = (float)(sx / n); g_hill_y[h] = (float)(sy / n); g_hill_n[h] = (float)n;
  };
  for (int i = 0; i < w.nseeds; ++i) {
    const Seed& sd = w.seeds[i];
    if (!w.on_bed(sd.x, sd.y)) continue;
    double dx = sd.x - fx, dy = sd.y - fy;
    if (h < 0 || dx * dx + dy * dy > 0.05 * 0.05) {  // more than 5 cm from the previous seed: new hill
      close();
      if (h + 1 >= MAX_HILLS) break;
      ++h; sx = sy = 0; n = 0;
      g_hill_ext[h] = 0; g_hill_w[h] = 0;
    }
    sx += sd.x; sy += sd.y; ++n;
    fx = sd.x; fy = sd.y;
  }
  close();
  s.hills = h + 1;
  if (s.hills <= 0) return s;
  // extent: furthest seed from the centroid, times two
  for (int i = 0; i < w.nseeds; ++i) {
    const Seed& sd = w.seeds[i];
    if (!w.on_bed(sd.x, sd.y)) continue;
    int best = 0; double bd = 1e9;
    for (int j = 0; j < s.hills; ++j) {
      double dx = sd.x - g_hill_x[j], dy = sd.y - g_hill_y[j], d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = j; }
    }
    double e = 2 * hm::sqrt_(bd);
    if (e > g_hill_ext[best]) g_hill_ext[best] = (float)e;
  }
  double sum = 0, sum2 = 0, ext = 0;
  for (int j = 0; j < s.hills; ++j) { sum += g_hill_n[j]; sum2 += g_hill_n[j] * g_hill_n[j]; ext += g_hill_ext[j]; }
  s.seeds_mean = sum / s.hills;
  s.seeds_sd = hm::sqrt_(hm::max_(0.0, sum2 / s.hills - s.seeds_mean * s.seeds_mean));
  s.extent_mean = ext / s.hills;
  // spacing between consecutive hills (skip jumps between rows)
  double gs = 0, gs2 = 0; int ng = 0;
  for (int j = 1; j < s.hills; ++j) {
    double dx = g_hill_x[j] - g_hill_x[j - 1], dy = g_hill_y[j] - g_hill_y[j - 1];
    if (hm::fabs_(dy) > 0.12) continue;
    double d = hm::sqrt_(dx * dx + dy * dy);
    gs += d; gs2 += d * d; ++ng;
  }
  if (ng) { s.spacing_mean = gs / ng; s.spacing_sd = hm::sqrt_(hm::max_(0.0, gs2 / ng - s.spacing_mean * s.spacing_mean)); }
  // water that landed within 5 cm of a hill
  double total = 0;
  for (int i = 0; i < w.nwet; ++i) {
    const Wet& wt = w.wet[i];
    total += wt.mL;
    for (int j = 0; j < s.hills; ++j) {
      double dx = wt.x - g_hill_x[j], dy = wt.y - g_hill_y[j];
      if (dx * dx + dy * dy <= 0.05 * 0.05) { s.water_on_hills += wt.mL; g_hill_w[j] += wt.mL; break; }
    }
  }
  s.water_frac = total > 0 ? s.water_on_hills / total : 0;
  for (int j = 0; j < s.hills; ++j) if (g_hill_w[j] >= 5) ++s.hills_watered;
  return s;
}

// ---- readable state
const char* const FIELDS[] = {
    "t_s", "x", "y", "th", "vl", "vr", "cmd_l", "cmd_r", "pump", "servo_deg", "gate_open", "fell", "fell_t",
    "batt_V", "soc", "current_A", "energy_Wh", "seeds_left", "tank_mL", "water_used", "water_on_bed",
    "pump_on_s", "pump_dry_s", "seeds_dropped", "seeds_off_bed", "dist", "far_overrun", "near_overrun",
    "pings", "pings_lost", "nseeds", "nwet", "ntrail", "events_total", "uno_loops", "esp_loops",
    "uno_loop_max_ms", "esp_loop_max_ms", "uno_log_total", "esp_log_total", "uno_resets", "esp_resets",
    "uno_rx_bytes", "esp_rx_bytes", "link_collide", "link_overflow", "link_garbled", "ap_up",
    "sensor_x", "sensor_y", "bed_len", "bed_w", "drop", "echo_mode", "air_T", "sweep1", "sweep2",
    "hills", "seeds_per_hill", "seeds_per_hill_sd", "hill_extent", "spacing_mean", "spacing_sd",
    "water_on_hills", "water_frac", "hills_watered", "uno_cpu_busy", "esp_cpu_busy",
};
const int NFIELDS = sizeof(FIELDS) / sizeof(FIELDS[0]);

// ---- things an experimenter can do to a running rig
const char* const ACTIONS[] = {"echo_mode", "refill_tank", "refill_seeds", "place", "heading", "soc", "battery_divider", "drop"};
const int NACTIONS = sizeof(ACTIONS) / sizeof(ACTIONS[0]);

}  // namespace

API int sim_rig_count() { return rig_count(); }
API int sim_param_count() { return NPARAMS; }
API const char* sim_param_name(int i) { return (i >= 0 && i < NPARAMS) ? PARAMS[i].name : ""; }
API int sim_field_count() { return NFIELDS; }
API const char* sim_field_name(int i) { return (i >= 0 && i < NFIELDS) ? FIELDS[i] : ""; }
API int sim_action_count() { return NACTIONS; }
API const char* sim_action_name(int i) { return (i >= 0 && i < NACTIONS) ? ACTIONS[i] : ""; }
API char* sim_scratch() { return g_scratch; }
API int sim_scratch_size() { return (int)sizeof(g_scratch); }

// idx -1 resets every parameter of rig h to its default
API void sim_param(int h, int idx, double v) {
  if (h < 0 || h >= 4) return;
  if (idx < 0) { g_params[h] = WorldParams(); return; }
  if (idx < NPARAMS) g_params[h].*(PARAMS[idx].field) = v;
}
API double sim_param_get(int h, int idx) {
  if (h < 0 || h >= 4 || idx < 0 || idx >= NPARAMS) return 0;
  return g_params[h].*(PARAMS[idx].field);
}

API int sim_init(int h, double seed) {
  Rig* r = rig(h);
  if (!r) return -1;
  r->sk[UNO] = &SK_DRIVE;
  r->sk[ESP] = &SK_LINK;
  r->battery_divider = true;
  r->power_on(g_params[h], (uint64_t)seed);
  r->run_until = 0;
  return 0;
}

API void sim_run(int h, double ms) {
  Rig* r = rig(h);
  if (!r || !r->booted || ms <= 0) return;
  r->run(r->run_until + (uint64_t)(ms * 1000.0 + 0.5));
}

API double sim_time(int h) { Rig* r = rig(h); return r ? (double)r->run_until * 1e-6 : 0; }

API void sim_set(int h, int action, double v) {
  Rig* r = rig(h);
  if (!r) return;
  World& w = r->world;
  switch (action) {
    case 0: w.p.echo_mode = (int)v; break;
    case 1: w.tank_left = v > 0 ? v : w.p.tank_mL; break;
    case 2: w.seeds_left = v > 0 ? v : w.p.hopper_seeds; break;
    case 3: w.x = v; w.y = 0; w.fell = false; w.vl = w.vr = 0; break;   // put the robot back on the bed at x
    case 4: w.th = v; break;
    case 5: w.soc = v; break;
    case 6: r->battery_divider = v != 0; break;
    case 7: w.p.drop = v; break;
  }
}

API double sim_get(int h, int f) {
  Rig* r = rig(h);
  if (!r || f < 0 || f >= NFIELDS) return 0;
  const World& w = r->world;
  const Mcu& u = r->mcu[UNO];
  const Mcu& e = r->mcu[ESP];
  const char* name = FIELDS[f];
  static HillStats hs;
  static uint64_t hs_t = ~0ULL; static int hs_h = -1; static int hs_n = -1, hs_w = -1;
  if (f >= 57 && f <= 65 && (hs_t != w.t_us || hs_h != h || hs_n != w.nseeds || hs_w != w.nwet)) {
    hs = hill_stats(w); hs_t = w.t_us; hs_h = h; hs_n = w.nseeds; hs_w = w.nwet;
  }
  (void)name;
  switch (f) {
    case 0: return (double)w.t_us * 1e-6;
    case 1: return w.x;
    case 2: return w.y;
    case 3: return w.th;
    case 4: return w.vl;
    case 5: return w.vr;
    case 6: return w.cmd_l;
    case 7: return w.cmd_r;
    case 8: return w.pump_pin;
    case 9: return w.servo_deg;
    case 10: return w.gate_open;
    case 11: return w.fell;
    case 12: return w.fell_t;
    case 13: return w.batt_V;
    case 14: return w.soc;
    case 15: return w.current_A;
    case 16: return w.energy_Wh;
    case 17: return w.seeds_left;
    case 18: return w.tank_left;
    case 19: return w.water_used;
    case 20: return w.water_on_bed;
    case 21: return w.pump_on_s;
    case 22: return w.pump_dry_s;
    case 23: return w.seeds_dropped;
    case 24: return w.seeds_off_bed;
    case 25: return w.dist;
    case 26: return w.max_far_overrun;
    case 27: return w.max_near_overrun;
    case 28: return w.pings;
    case 29: return w.pings_lost;
    case 30: return w.nseeds;
    case 31: return w.nwet;
    case 32: return w.ntrail;
    case 33: return w.events_total;
    case 34: return (double)u.loops;
    case 35: return (double)e.loops;
    case 36: return (double)u.loop_max_us / 1000.0;
    case 37: return (double)e.loop_max_us / 1000.0;
    case 38: return u.log_total;
    case 39: return e.log_total;
    case 40: return u.resets;
    case 41: return e.resets;
    case 42: return u.rx_bytes;
    case 43: return e.rx_bytes;
    case 44: return u.rx_collide + e.rx_collide;
    case 45: return u.rx_overflow + e.rx_overflow;
    case 46: return u.rx_garbled + e.rx_garbled;
    case 47: return r->run_until >= r->ap_up_us ? 1 : 0;
    case 48: return w.sensor_x();
    case 49: return w.sensor_y();
    case 50: return w.p.bed_len;
    case 51: return w.p.bed_w;
    case 52: return w.p.drop;
    case 53: return w.p.echo_mode;
    case 54: return w.p.air_T;
    case 55: return w.sweep_deg[0];
    case 56: return w.sweep_deg[1];
    case 57: return hs.hills;
    case 58: return hs.seeds_mean;
    case 59: return hs.seeds_sd;
    case 60: return hs.extent_mean;
    case 61: return hs.spacing_mean;
    case 62: return hs.spacing_sd;
    case 63: return hs.water_on_hills;
    case 64: return hs.water_frac;
    case 65: return hs.hills_watered;
    case 66: return (double)u.cpu_busy_us * 1e-6;
    case 67: return (double)e.cpu_busy_us * 1e-6;
    default: return 0;
  }
}

// What the firmware itself believes (see sim::Probe)
API double sim_probe(int h, int board, int what) {
  Rig* r = rig(h);
  if (!r || board < 0 || board > 1 || !r->sk[board]) return -1;
  return r->sk[board]->probe(what);
}

// ---- the phone: queue a GET for the URL in the scratch buffer, at the rig's current time
API int sim_http(int h) {
  Rig* r = rig(h);
  if (!r) return 0;
  g_scratch[sizeof(g_scratch) - 1] = 0;
  return r->http_enqueue(g_scratch, r->run_until);
}
API int sim_http_at(int h, double t_ms) {
  Rig* r = rig(h);
  if (!r) return 0;
  return r->http_enqueue(g_scratch, (uint64_t)(t_ms * 1000.0 + 0.5));
}
// -1 unknown, 0 waiting, 1 answered
API int sim_http_state(int h, int id) {
  Rig* r = rig(h);
  HttpReq* q = r ? r->http_find(id) : nullptr;
  if (!q) return -1;
  return q->state == 1 ? 1 : 0;
}
API int sim_http_code(int h, int id) { Rig* r = rig(h); HttpReq* q = r ? r->http_find(id) : nullptr; return q ? q->code : -1; }
API double sim_http_latency_ms(int h, int id) {
  Rig* r = rig(h); HttpReq* q = r ? r->http_find(id) : nullptr;
  return (q && q->state == 1) ? (double)(q->done_us - q->t_us) / 1000.0 : -1;
}
// copies the response body into the scratch buffer; returns its length (-1 if gone)
API int sim_http_body(int h, int id) {
  Rig* r = rig(h); HttpReq* q = r ? r->http_find(id) : nullptr;
  if (!q || q->state != 1 || q->body_gen != r->body_gen) return -1;
  uint32_t n = q->body_len < sizeof(g_scratch) - 1 ? q->body_len : (uint32_t)sizeof(g_scratch) - 1;
  memcpy(g_scratch, r->body + q->body_off, n);
  g_scratch[n] = 0;
  return (int)n;
}

// ---- serial monitor of a board: bytes from index `from` (see *_log_total)
API int sim_log(int h, int board, double from) {
  Rig* r = rig(h);
  if (!r || board < 0 || board > 1) return 0;
  const Mcu& m = r->mcu[board];
  uint32_t f = (uint32_t)from;
  if (m.log_total > Mcu::LOG_CAP && f < m.log_total - Mcu::LOG_CAP) f = m.log_total - Mcu::LOG_CAP;
  uint32_t n = 0;
  for (uint32_t i = f; i < m.log_total && n < sizeof(g_scratch) - 1; ++i) g_scratch[n++] = m.log[i % Mcu::LOG_CAP];
  g_scratch[n] = 0;
  return (int)n;
}

// ---- world data as raw float arrays (valid until the next sim_run)
API float* sim_seeds(int h) { Rig* r = rig(h); return r ? &r->world.seeds[0].x : nullptr; }
API float* sim_wet(int h) { Rig* r = rig(h); return r ? &r->world.wet[0].x : nullptr; }
API float* sim_trail(int h) { Rig* r = rig(h); return r ? &r->world.trail[0].t : nullptr; }
API float* sim_hills(int h, int which) {
  Rig* r = rig(h); if (!r) return nullptr;
  (void)sim_get(h, 57);
  switch (which) { case 0: return g_hill_x; case 1: return g_hill_y; case 2: return g_hill_n; case 3: return g_hill_ext; default: return g_hill_w; }
}
// events since total index `from`, packed as 5 floats each (t, type, a, b, c) into scratch
API int sim_events(int h, double from) {
  Rig* r = rig(h);
  if (!r) return 0;
  const World& w = r->world;
  uint32_t total = w.events_total, f = (uint32_t)from;
  if (total > (uint32_t)World::MAX_EVENTS && f < total - World::MAX_EVENTS) f = total - World::MAX_EVENTS;
  float* out = (float*)g_scratch;
  int n = 0, cap = (int)(sizeof(g_scratch) / sizeof(float) / 5);
  for (uint32_t i = f; i < total && n < cap; ++i) {
    // event i lives at ring slot i % MAX_EVENTS
    const Event& e = w.events[i % World::MAX_EVENTS];
    out[n * 5 + 0] = e.t; out[n * 5 + 1] = e.type; out[n * 5 + 2] = e.a; out[n * 5 + 3] = e.b; out[n * 5 + 4] = e.c;
    ++n;
  }
  return n;
}

// the page the NodeMCU serves at "/", copied into scratch
API int sim_ui_html(int h) {
  Rig* r = rig(h);
  if (!r || !r->sk[ESP]) return 0;
  const char* s = r->sk[ESP]->ui_html();
  size_t n = strlen(s);
  if (n > sizeof(g_scratch) - 1) n = sizeof(g_scratch) - 1;
  memcpy(g_scratch, s, n);
  g_scratch[n] = 0;
  return (int)n;
}
