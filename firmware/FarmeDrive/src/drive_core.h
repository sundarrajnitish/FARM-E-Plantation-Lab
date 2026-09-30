// drive_core.h - FARM-E's motion and planting controller (runs on the UNO).
//
// Plain C++ with no Arduino calls: the sketch feeds it the time, ground
// distance pings and battery voltage, hands it the frames that arrive from
// the NodeMCU, and copies its Outputs onto the pins. That keeps every
// decision testable on a PC (firmware/test) and in the simulator (sim/).
//
// Automatic planting, one hill at a time:
//
//   CALIBRATE -> SETTLE -> SEED -> WATER -> DRIVE --spacing--> SETTLE ...
//                                             |
//                                           edge (ground drops away ahead)
//                                             v
//             BACKOFF -> TURN1 -> SHIFT (row gap) -> TURN2 -> SETTLE ...   (serpentine)
//                   \-> DONE after the last row
//
// The ultrasonic sensor looks down just ahead of the tracks. The controller
// learns the ground distance while standing still and treats a reading more
// than edge_mm above it (or three missing echoes in a row, or no ping for
// 150 ms) as "no ground ahead": forward motion stops at once, in automatic
// and in manual mode.
#pragma once
#include <stdint.h>
#include "farme_proto.h"

namespace farme {

enum Mode : uint8_t { MODE_IDLE = 0, MODE_MANUAL = 1, MODE_AUTO = 2, MODE_FAULT = 3 };

enum State : uint8_t {
  ST_IDLE = 0, ST_MANUAL, ST_CALIBRATE, ST_DRIVE, ST_SETTLE, ST_SEED, ST_WATER,
  ST_BACKOFF, ST_TURN1, ST_SHIFT, ST_TURN2, ST_DONE, ST_PAUSED, ST_FAULT, ST_COUNT
};

enum Fault : uint8_t { FAULT_NONE = 0, FAULT_NO_GROUND, FAULT_SENSOR, FAULT_LOW_BATT };

enum Flag : uint16_t {
  FL_EDGE = 1,          // stopped at an edge (automatic) since the last start
  FL_TANK_LOW = 2,      // not enough water for a full dose
  FL_LOW_BATT = 4,      // battery below low_batt_mV
  FL_LINK = 8,          // frames from the NodeMCU are arriving
  FL_CAL = 16,          // ground distance learned
  FL_DONE = 32,         // automatic run finished
  FL_SENSOR_WARN = 64,  // echoes missing
  FL_PAUSED = 128,
  FL_GUARD = 256,       // manual forward refused: no ground ahead
};

struct DriveConfig {
  // user-settable (CMD_SET), stored in EEPROM by the sketch
  int16_t spacing_mm = 250;   // distance between hills
  int16_t rows = 3;           // rows per run (serpentine)
  int16_t row_gap_mm = 300;   // distance between rows
  int16_t water_mL = 20;      // water per hill (0 = no watering)
  int16_t seed_ms = 80;       // gate held open per hill (after ~80 ms of travel)
  int16_t speed_mm_s = 240;   // ground speed at batt_nominal_mV (calibrate on your soil)
  int16_t turn_ms_90 = 910;   // pivot time for 90 degrees at batt_nominal_mV
  int16_t edge_mm = 40;       // a reading this much above the learned ground = edge
  int16_t tank_mL = 350;      // tank capacity
  int16_t pump_mL_s10 = 180;  // pump flow in 0.1 mL/s
  // fixed by the build
  int16_t gate_open_deg = 40, gate_closed_deg = 0;
  int16_t batt_nominal_mV = 11700, driver_drop_mV = 2000, low_batt_mV = 10500;
  int16_t backoff_mm = 80;
  bool sweeps_fitted = false;

  int32_t get(uint8_t p) const;
  bool set(uint8_t p, int32_t v);  // false if p unknown or v out of range
};

struct Inputs {
  uint32_t now_ms = 0;
  bool ping = false;        // a new ground reading is in ground_mm
  int16_t ground_mm = -1;   // -1: no echo within range
  uint16_t batt_mV = 0;     // 0: not measured
};

struct Outputs {
  int8_t left = 0, right = 0;   // -1 back, 0 coast, 1 forward, 2 brake
  uint8_t gate_deg = 0;
  bool pump = false;
  uint8_t sweep[2] = {90, 90};
};

class Drive {
 public:
  void begin(const DriveConfig& cfg, uint32_t now);
  void on_frame(const Frame& fr, uint32_t now);
  void tick(const Inputs& in);
  const Outputs& out() const { return out_; }

  bool telemetry_due(uint32_t now) const;
  Frame telemetry(uint32_t now);

  Mode mode() const { return mode_; }
  State state() const { return state_; }
  Fault fault() const { return fault_; }
  uint16_t flags() const;
  int16_t ground_mm() const { return ground_; }
  int16_t baseline_mm() const { return baseline_; }
  uint16_t hills() const { return hills_; }
  uint8_t row() const { return row_; }
  int16_t tank_mL() const { return (int16_t)(tank_uL_ / 1000); }
  uint16_t batt_mV() const { return batt_mV_; }
  uint16_t last_seq() const { return last_seq_; }
  uint32_t frames() const { return frames_; }
  int16_t temp_dC() const { return temp_dC_; }
  int32_t speed_mm_s() const;          // battery-compensated estimate
  const DriveConfig& config() const { return cfg_; }
  bool take_config_changed() { bool c = cfg_changed_; cfg_changed_ = false; return c; }
  bool take_state_changed() { bool c = state_changed_; state_changed_ = false; return c; }
  static const char* state_name(uint8_t s);

 private:
  void enter(State s, uint32_t now);
  void command(uint8_t code, int32_t a, int32_t b, uint32_t now);
  void start_auto(uint32_t now);
  void ground_update(const Inputs& in);
  bool ground_ahead(uint32_t now) const;
  void motors(int8_t l, int8_t r) { out_.left = l; out_.right = r; }
  void run_auto(uint32_t now, uint32_t dt);
  void run_manual(uint32_t now);
  void pump_account(uint32_t dt);
  uint32_t turn_ms() const;
  uint32_t ms_for(int32_t mm) const;

  DriveConfig cfg_;
  Outputs out_;
  Mode mode_ = MODE_IDLE;
  State state_ = ST_IDLE, paused_from_ = ST_IDLE;
  Fault fault_ = FAULT_NONE;
  uint32_t t_enter_ = 0, t_last_ = 0, phase_t_ = 0;
  uint8_t phase_ = 0;
  bool state_changed_ = false, cfg_changed_ = false;

  // ground sensing
  int16_t ground_ = -1, baseline_ = -1;
  int16_t win_[7] = {};
  uint8_t win_n_ = 0;
  uint8_t edge_run_ = 0, invalid_run_ = 0;
  uint32_t last_ping_ = 0, still_since_ = 0;
  bool edge_flag_ = false, guard_ = false;

  // planting progress
  int32_t dist_um_ = 0;   // distance driven in this segment (micrometres)
  uint16_t hills_ = 0;
  uint8_t row_ = 0;
  bool turn_left_ = true;
  bool done_ = false, tank_low_ = false;
  int32_t tank_uL_ = 350000;
  uint32_t dose_ms_ = 0;

  // manual
  uint8_t drive_dir_ = DIR_STOP;
  uint32_t drive_until_ = 0, pump_until_ = 0;
  uint8_t mseed_phase_ = 0, mseed_queued_ = 0;
  uint32_t mseed_t_ = 0;

  // power and link
  uint16_t batt_mV_ = 0;
  uint32_t low_since_ = 0;
  bool low_batt_ = false;
  uint32_t link_seen_ = 0;
  bool link_ever_ = false;
  uint32_t frames_ = 0;
  int16_t temp_dC_ = -999;
  uint16_t last_seq_ = 0;
  bool ack_now_ = false;
  uint32_t last_tel_ = 0;
};

}  // namespace farme
