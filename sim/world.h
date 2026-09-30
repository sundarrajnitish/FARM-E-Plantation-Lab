// world.h - the physical side of the simulation: a tracked robot on a soil
// bed, its battery, drive tracks, seed hopper and gate servo, water tank and
// pump, the downward-looking ultrasonic sensor and the DHT11's air.
//
// Units: metres, seconds, volts, millilitres. The field frame has x along the
// planting row (the bed runs from x = 0 to x = bed_len), y across the bed
// (|y| <= bed_w / 2). The robot starts near x = 0 facing +x.
//
// The numbers are plausible for the prototype (3S 2200 mAh LiPo,
// 12 V 200 rpm geared motors on rubber tracks, SG90 gate servo, 5 V mini
// pump, HC-SR04) but they are assumptions, not measurements. Both firmwares
// run against the same world, so comparisons between them are fair even
// where the absolute numbers are uncertain.
#pragma once
#include <stdint.h>
#include "hmath.h"

namespace sim {

// UNO pins (firmware/FarmeDrive/config.h)
enum : uint8_t {
  PIN_LM1 = 2, PIN_LM2 = 4, PIN_RM1 = 6, PIN_RM2 = 7,
  PIN_SERVO = 9, PIN_SWEEP1 = 10, PIN_SWEEP2 = 11, PIN_PUMP = 12, PIN_LED = 13,
  PIN_A0 = 14, PIN_A1 = 15, PIN_A2 = 16, PIN_A3 = 17, PIN_A4 = 18, PIN_A5 = 19,
  PIN_TRIG = PIN_A4, PIN_ECHO = PIN_A5, PIN_BATT = PIN_A3,
};

struct WorldParams {
  // field
  double bed_len = 2.4, bed_w = 1.2;
  double drop = 0.75;              // how far the ground falls beyond the bed (tabletop: 0.75)
  double start_x = 0.16, start_y = -0.35, start_heading = 0.0;
  // drive
  double v_nom = 0.24;             // track speed in soil at the nominal battery voltage (m/s)
  double v_batt_nom = 11.7, driver_drop = 2.0;   // L298N-style bridge loses ~2 V
  double motor_tau = 0.08;         // s, first-order spin-up
  double track_noise = 0.02;       // relative, per track, slowly varying
  double track_noise_tau = 1.2;    // s
  double gauge = 0.17, skid = 1.5; // skid-steer: yaw rate = (vR - vL) / (gauge * skid)
  // battery (3S LiPo)
  double batt_mAh = 2200, soc0 = 0.85, r_int = 0.06;
  // geometry
  double sensor_fwd = 0.13, sensor_h = 0.055;  // ultrasonic ahead of the centre, above the soil
  double tube_back = 0.02;          // seed tube behind the centre
  double nozzle_back = 0.05;        // water nozzle behind the centre
  double half_len = 0.11;           // track contact patch half-length
  // ultrasonic
  int echo_mode = 0;               // 0 wired, 1 echo wire disconnected, 2 sensor sees nothing
  double echo_dropout = 0.01;      // chance a ping gets no echo (soil absorbs it)
  double echo_noise_m = 0.002;
  // seeds
  double hopper_seeds = 400, seed_rate = 30;  // seeds/s through a fully open gate
  double gate_open_deg = 12, gate_full_deg = 42;
  double seed_scatter = 0.008;
  double servo_dps = 600;          // SG90, no load
  // water
  double tank_mL = 350, pump_mL_s = 18;
  // air (DHT11)
  double air_T = 28.0, air_RH = 62.0, dht_fail = 0.02;
};

struct Seed { float x, y, t; };
struct Wet { float x, y, mL, t; };
struct TrailPt { float t, x, y, th, vl, vr; };
struct Event { float t; int16_t type, a, b, c; };

enum EventType : int16_t {
  EV_MOTOR = 1,      // a = left cmd, b = right cmd (-1, 0, 1; 2 = brake)
  EV_PUMP = 2,       // a = on
  EV_GATE = 3,       // a = open
  EV_FALL = 4,       // robot left the bed
  EV_EDGE = 5,       // sensor passed over the bed's far (a = 1) or near (a = 0) end
  EV_TANK_EMPTY = 6,
  EV_HTTP = 7,       // a = request id, b = 0 received / 1 answered, c = HTTP status
  EV_WDT = 8,        // a = board
  EV_LINK_DROP = 9,  // a = board that lost a byte, b = 0 overflow / 1 collision
};

struct World {
  WorldParams p;
  hm::Rng rng_track{1}, rng_seed{2}, rng_echo{3}, rng_dht{4};
  uint64_t t_us = 0;

  // body
  double x = 0, y = 0, th = 0;
  double vl = 0, vr = 0;            // track speeds
  double nl = 0, nr = 0;            // slow load noise on each track
  int cmd_l = 0, cmd_r = 0;         // -1, 0, 1, 2 = brake
  bool fell = false;
  double fell_t = -1;
  // power
  double soc = 0.85, batt_V = 12.0, current_A = 0, energy_Wh = 0;
  // actuators
  bool pump_pin = false;
  double servo_deg = 0, servo_target = 0;
  bool servo_attached = false;
  bool gate_open = false;
  double sweep_deg[2] = {0, 0};
  // consumables
  double seeds_left = 0, tank_left = 0;
  double water_used = 0, water_on_bed = 0, pump_on_s = 0, pump_dry_s = 0;
  double wet_accum = 0; float wet_ax = 0, wet_ay = 0;
  uint32_t seeds_dropped = 0, seeds_off_bed = 0;
  // odometry truth
  double dist = 0, max_far_overrun = -1e9, max_near_overrun = -1e9;
  bool sensor_past_far = false, sensor_past_near = false;
  // ultrasonic: echo window of the last ping
  uint64_t echo_rise_us = 0, echo_fall_us = 0;
  bool echo_armed = false;
  uint64_t trig_high_us = 0;
  uint32_t pings = 0, pings_lost = 0;

  static const int MAX_SEEDS = 6000, MAX_WET = 6000, MAX_TRAIL = 24000, MAX_EVENTS = 8000;
  Seed seeds[MAX_SEEDS]; int nseeds = 0;
  Wet wet[MAX_WET]; int nwet = 0;
  TrailPt trail[MAX_TRAIL]; int ntrail = 0;
  Event events[MAX_EVENTS]; int nevents = 0; uint32_t events_total = 0;
  uint64_t next_trail_us = 0;

  void reset(const WorldParams& params, uint64_t seed);
  void step_to(uint64_t target_us);
  void event(int16_t type, int a = 0, int b = 0, int c = 0) { event_at(t_us, type, a, b, c); }
  // events raised by a board carry that board's clock (it may run ahead of the physics)
  void event_at(uint64_t t, int16_t type, int a = 0, int b = 0, int c = 0);

  // inputs from the UNO's pins
  void set_motors(int l, int r);
  void set_pump(bool on);
  void set_servo(double deg, bool attached);
  void set_sweep(int i, double deg);
  void trigger_edge(bool high, uint64_t at_us);  // HC-SR04 TRIG pin changed
  // the echo pin's pulse after the last trigger: returns false if none
  bool echo_window(uint64_t* rise, uint64_t* fall) const;
  int battery_adc() const;  // A3 through a 10k/3.3k divider, 5 V reference

  // DHT11 reading at time t: returns false on a checksum error
  bool dht(uint64_t at_us, int* T, int* RH);

  bool on_bed(double px, double py) const {
    return px >= 0 && px <= p.bed_len && py >= -p.bed_w / 2 && py <= p.bed_w / 2;
  }
  double sensor_x() const { return x + p.sensor_fwd * hm::cos_(th); }
  double sensor_y() const { return y + p.sensor_fwd * hm::sin_(th); }
  bool moving() const { return hm::fabs_(vl) > 0.002 || hm::fabs_(vr) > 0.002; }

 private:
  void step_1ms();
};

}  // namespace sim
