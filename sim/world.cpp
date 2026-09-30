// world.cpp - see world.h
#include "world.h"

namespace sim {
using namespace hm;

void World::reset(const WorldParams& params, uint64_t seed) {
  // plain member-wise re-initialisation (no libc in the wasm build)
  p = params;
  uint64_t k = seed * 0x9E3779B97F4A7C15ULL + 12345;
  rng_track = Rng(k); rng_seed = Rng(k ^ 0x5eedULL); rng_echo = Rng(k ^ 0xec40ULL); rng_dht = Rng(k ^ 0xd47ULL);
  t_us = 0;
  x = p.start_x; y = p.start_y; th = p.start_heading;
  vl = vr = 0; nl = nr = 0; cmd_l = cmd_r = 0;
  fell = false; fell_t = -1;
  soc = p.soc0; current_A = 0; energy_Wh = 0;
  batt_V = 3.0 * (3.6 + 0.6 * soc);
  pump_pin = false; servo_deg = 0; servo_target = 0; servo_attached = false;
  sweep_deg[0] = sweep_deg[1] = 0; gate_open = false;
  seeds_left = p.hopper_seeds; tank_left = p.tank_mL;
  water_used = water_on_bed = pump_on_s = pump_dry_s = 0;
  wet_accum = 0; wet_ax = wet_ay = 0;
  seeds_dropped = seeds_off_bed = 0;
  dist = 0; max_far_overrun = -1e9; max_near_overrun = -1e9;
  sensor_past_far = sensor_past_near = false;
  echo_rise_us = echo_fall_us = 0; echo_armed = false; trig_high_us = 0;
  pings = pings_lost = 0;
  nseeds = nwet = ntrail = nevents = 0; events_total = 0;
  next_trail_us = 0;
}

void World::event_at(uint64_t t, int16_t type, int a, int b, int c) {
  Event& e = events[nevents % MAX_EVENTS];
  e.t = (float)((double)t * 1e-6);
  e.type = type; e.a = (int16_t)a; e.b = (int16_t)b; e.c = (int16_t)c;
  ++nevents; ++events_total;
  if (nevents >= 2 * MAX_EVENTS) nevents -= MAX_EVENTS;  // keep the ring index bounded
}

void World::set_motors(int l, int r) {
  if (l == cmd_l && r == cmd_r) return;
  cmd_l = l; cmd_r = r;
  event(EV_MOTOR, l, r);
}

void World::set_pump(bool on) {
  if (on == pump_pin) return;
  pump_pin = on;
  event(EV_PUMP, on);
}

void World::set_servo(double deg, bool attached) {
  servo_target = clamp_(deg, 0, 180);
  servo_attached = attached;
}

void World::set_sweep(int i, double deg) { if (i >= 0 && i < 2) sweep_deg[i] = deg; }

void World::trigger_edge(bool high, uint64_t at_us) {
  if (high) { trig_high_us = at_us; return; }
  // HC-SR04: fires on the falling edge of a >= 10 us trigger pulse
  if (at_us < trig_high_us + 10 || trig_high_us == 0) return;
  trig_high_us = 0;
  if (p.echo_mode == 1) { echo_armed = false; return; }  // echo wire open: pin stays low
  step_to(at_us);
  ++pings;
  double d;
  if (fell) {
    d = 0.03;  // lying on its side against the floor
  } else {
    double sx = sensor_x(), sy = sensor_y();
    d = p.sensor_h + (on_bed(sx, sy) ? 0.0 : p.drop);
  }
  d += p.echo_noise_m * rng_echo.gauss();
  double c = 331.3 + 0.606 * p.air_T;
  double width_us = 2.0 * d / c * 1e6;
  bool lost = p.echo_mode == 2 || rng_echo.uniform() < p.echo_dropout || d > 4.0;
  if (lost) { width_us = 38000; ++pings_lost; }  // module times out and drops ECHO after ~38 ms
  echo_rise_us = at_us + 460;                     // 8-cycle 40 kHz burst + setup
  echo_fall_us = echo_rise_us + (uint64_t)(width_us + 0.5);
  echo_armed = true;
}

bool World::echo_window(uint64_t* rise, uint64_t* fall) const {
  if (!echo_armed) return false;
  *rise = echo_rise_us; *fall = echo_fall_us;
  return true;
}

int World::battery_adc() const {
  double v = batt_V * 3.3 / 13.3;
  int a = (int)(v / 5.0 * 1023.0 + 0.5);
  return a < 0 ? 0 : (a > 1023 ? 1023 : a);
}

bool World::dht(uint64_t at_us, int* T, int* RH) {
  double t = (double)at_us * 1e-6;
  if (rng_dht.uniform() < p.dht_fail) return false;
  double tt = p.air_T + 0.8 * sin_(TWO_PI * t / 900.0);
  double hh = p.air_RH - 3.0 * sin_(TWO_PI * t / 900.0);
  *T = (int)(tt + 0.5); *RH = (int)(hh + 0.5);  // DHT11: whole degrees, whole %RH
  return true;
}

void World::step_to(uint64_t target_us) {
  while (t_us + 1000 <= target_us) step_1ms();
}

void World::step_1ms() {
  const double dt = 0.001;
  t_us += 1000;
  double t = (double)t_us * 1e-6;

  // ---- battery
  double i_motor = (cmd_l == 1 || cmd_l == -1 ? 0.75 : 0) + (cmd_r == 1 || cmd_r == -1 ? 0.75 : 0);
  bool servo_moving = servo_attached && fabs_(servo_target - servo_deg) > 0.5;
  current_A = 0.19 + i_motor + (pump_pin ? (tank_left > 0 ? 0.35 : 0.22) : 0) + (servo_moving ? 0.25 : 0);
  soc -= current_A * dt / 3600.0 / (p.batt_mAh / 1000.0);
  if (soc < 0) soc = 0;
  double ocv = 3.0 * (3.6 + 0.6 * soc);
  batt_V = ocv - current_A * p.r_int;
  energy_Wh += batt_V * current_A * dt / 3600.0;

  // ---- tracks
  double a = exp_(-dt / p.track_noise_tau), b = sqrt_(1 - a * a);
  nl = nl * a + b * rng_track.gauss();
  nr = nr * a + b * rng_track.gauss();
  double vmax = p.v_nom * max_(0.0, batt_V - p.driver_drop) / (p.v_batt_nom - p.driver_drop);
  auto target = [&](int cmd, double n) {
    if (cmd == 1) return vmax * (1 + p.track_noise * n);
    if (cmd == -1) return -vmax * (1 + p.track_noise * n);
    return 0.0;
  };
  double tl = target(cmd_l, nl), tr = target(cmd_r, nr);
  if (fell) { tl = tr = 0; }
  double kl = (cmd_l == 0 ? 0.05 : cmd_l == 2 ? 0.02 : p.motor_tau);  // coasting / braking stop quicker in soil
  double kr = (cmd_r == 0 ? 0.05 : cmd_r == 2 ? 0.02 : p.motor_tau);
  vl += (tl - vl) * (1 - exp_(-dt / kl));
  vr += (tr - vr) * (1 - exp_(-dt / kr));
  if (!fell) {
    double v = 0.5 * (vl + vr), w = (vr - vl) / (p.gauge * p.skid);
    double c = cos_(th), s = sin_(th);
    x += v * c * dt; y += v * s * dt; th = wrap_pi(th + w * dt);
    dist += fabs_(v) * dt;
    if (!on_bed(x, y)) {
      fell = true; fell_t = t;
      event(EV_FALL, (int)(x * 1000), (int)(y * 1000));
      vl = vr = 0;
    }
    double sx = sensor_x();
    if (sx - p.bed_len > max_far_overrun) max_far_overrun = sx - p.bed_len;
    if (-sx > max_near_overrun) max_near_overrun = -sx;
    bool far = sx > p.bed_len, near = sx < 0;
    if (far != sensor_past_far) { sensor_past_far = far; if (far) event(EV_EDGE, 1); }
    if (near != sensor_past_near) { sensor_past_near = near; if (near) event(EV_EDGE, 0); }
  }

  // ---- gate servo and seeds
  if (servo_attached) {
    double step = p.servo_dps * dt;
    double d = servo_target - servo_deg;
    servo_deg += d > step ? step : (d < -step ? -step : d);
  }
  double f = clamp_((servo_deg - p.gate_open_deg) / (p.gate_full_deg - p.gate_open_deg), 0, 1);
  if (f > 0 && seeds_left >= 1) {
    if (rng_seed.uniform() < p.seed_rate * f * dt) {
      seeds_left -= 1;
      ++seeds_dropped;
      double c = cos_(th), s = sin_(th), v = 0.5 * (vl + vr);
      double lx = x - p.tube_back * c + v * c * 0.12 + p.seed_scatter * rng_seed.gauss();
      double ly = y - p.tube_back * s + v * s * 0.12 + p.seed_scatter * rng_seed.gauss();
      if (!on_bed(lx, ly) || fell) ++seeds_off_bed;
      if (nseeds < MAX_SEEDS) seeds[nseeds++] = Seed{(float)lx, (float)ly, (float)t};
    }
  }
  if ((f > 0) != gate_open) { gate_open = f > 0; event(EV_GATE, gate_open); }

  // ---- pump
  if (pump_pin) {
    pump_on_s += dt;
    if (tank_left > 0) {
      double q = min_(tank_left, p.pump_mL_s * dt);
      tank_left -= q; water_used += q;
      if (tank_left <= 0) { tank_left = 0; event(EV_TANK_EMPTY); }
      double c = cos_(th), s = sin_(th);
      float nx = (float)(x - p.nozzle_back * c), ny = (float)(y - p.nozzle_back * s);
      if (on_bed(nx, ny) && !fell) water_on_bed += q;
      wet_accum += q; wet_ax += nx * (float)q; wet_ay += ny * (float)q;
      if (wet_accum >= 1.0) {  // one wet spot per millilitre delivered
        if (nwet < MAX_WET) wet[nwet++] = Wet{wet_ax / (float)wet_accum, wet_ay / (float)wet_accum, (float)wet_accum, (float)t};
        wet_accum = 0; wet_ax = wet_ay = 0;
      }
    } else {
      pump_dry_s += dt;
    }
  }

  // ---- trail every 50 ms
  if (t_us >= next_trail_us) {
    next_trail_us = t_us + 50000;
    if (ntrail < MAX_TRAIL) ntrail++;
    else { for (int i = 1; i < MAX_TRAIL; ++i) trail[i - 1] = trail[i]; }
    trail[ntrail - 1] = TrailPt{(float)t, (float)x, (float)y, (float)th, (float)vl, (float)vr};
  }
}

}  // namespace sim
