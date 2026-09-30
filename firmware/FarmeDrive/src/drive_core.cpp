// drive_core.cpp - see drive_core.h
// Written for the ATmega328P: int is 16 bits there, so anything that can
// exceed 32767 is int32_t, and there is no floating point.
#include "drive_core.h"

namespace farme {

// ------------------------------------------------------------------ config
struct Range { int16_t lo, hi; };
static const Range RANGES[P_COUNT] = {
    {0, 0},        // unused
    {100, 800},    // P_SPACING_MM
    {1, 8},        // P_ROWS
    {150, 600},    // P_ROW_GAP_MM
    {0, 60},       // P_WATER_ML
    {40, 600},     // P_SEED_MS
    {60, 600},     // P_SPEED_MM_S
    {200, 3000},   // P_TURN_MS_90
    {15, 200},     // P_EDGE_MM
    {50, 2000},    // P_TANK_ML
    {20, 600},     // P_PUMP_ML_S10
};

int32_t DriveConfig::get(uint8_t p) const {
  switch (p) {
    case P_SPACING_MM: return spacing_mm;
    case P_ROWS: return rows;
    case P_ROW_GAP_MM: return row_gap_mm;
    case P_WATER_ML: return water_mL;
    case P_SEED_MS: return seed_ms;
    case P_SPEED_MM_S: return speed_mm_s;
    case P_TURN_MS_90: return turn_ms_90;
    case P_EDGE_MM: return edge_mm;
    case P_TANK_ML: return tank_mL;
    case P_PUMP_ML_S10: return pump_mL_s10;
    default: return -1;
  }
}

bool DriveConfig::set(uint8_t p, int32_t v) {
  if (p == 0 || p >= P_COUNT || v < RANGES[p].lo || v > RANGES[p].hi) return false;
  int16_t x = (int16_t)v;
  switch (p) {
    case P_SPACING_MM: spacing_mm = x; break;
    case P_ROWS: rows = x; break;
    case P_ROW_GAP_MM: row_gap_mm = x; break;
    case P_WATER_ML: water_mL = x; break;
    case P_SEED_MS: seed_ms = x; break;
    case P_SPEED_MM_S: speed_mm_s = x; break;
    case P_TURN_MS_90: turn_ms_90 = x; break;
    case P_EDGE_MM: edge_mm = x; break;
    case P_TANK_ML: tank_mL = x; break;
    case P_PUMP_ML_S10: pump_mL_s10 = x; break;
  }
  return true;
}

// ------------------------------------------------------------------ helpers
static const char* const STATE_NAMES[ST_COUNT] = {
    "IDLE", "MANUAL", "CALIBRATE", "DRIVE", "SETTLE", "SEED", "WATER",
    "BACKOFF", "TURN1", "SHIFT", "TURN2", "DONE", "PAUSED", "FAULT"};

const char* Drive::state_name(uint8_t s) { return s < ST_COUNT ? STATE_NAMES[s] : "?"; }

static const uint32_t SETTLE_MS = 250, BRAKE_MS = 80, GATE_TRAVEL_MS = 80, GATE_CLOSE_MS = 150;
static const uint32_t PING_STALE_MS = 150, STILL_MS = 150, LINK_LOST_MS = 1500;
static const uint32_t CALIBRATE_TIMEOUT_MS = 1500, LOW_BATT_MS = 3000;
// the tracks take ~80 ms to reach speed and ~20 ms to stop: count distance
// from 60 ms after the motors start so a segment ends where it should
static const uint32_t DRIVE_LAG_MS = 60;

int32_t Drive::speed_mm_s() const {
  int32_t v = cfg_.speed_mm_s;
  if (batt_mV_ >= 6000) {
    int32_t num = (int32_t)batt_mV_ - cfg_.driver_drop_mV;
    int32_t den = (int32_t)cfg_.batt_nominal_mV - cfg_.driver_drop_mV;
    int32_t c = v * num / den;
    if (c < v / 2) c = v / 2;
    if (c > v * 3 / 2) c = v * 3 / 2;
    v = c;
  }
  return v;
}

uint32_t Drive::ms_for(int32_t mm) const { return (uint32_t)(mm * 1000L / speed_mm_s()); }

uint32_t Drive::turn_ms() const {
  return (uint32_t)((int32_t)cfg_.turn_ms_90 * cfg_.speed_mm_s / speed_mm_s());
}

// ------------------------------------------------------------------ lifecycle
void Drive::begin(const DriveConfig& cfg, uint32_t now) {
  *this = Drive();
  cfg_ = cfg;
  tank_uL_ = (int32_t)cfg_.tank_mL * 1000L;
  out_.gate_deg = (uint8_t)cfg_.gate_closed_deg;
  t_last_ = now;
  still_since_ = now;
  last_tel_ = now;
  enter(ST_IDLE, now);
}

void Drive::enter(State s, uint32_t now) {
  state_ = s;
  t_enter_ = now;
  phase_ = 0;
  state_changed_ = true;
}

uint16_t Drive::flags() const {
  uint16_t f = 0;
  if (edge_flag_) f |= FL_EDGE;
  if (tank_low_) f |= FL_TANK_LOW;
  if (low_batt_) f |= FL_LOW_BATT;
  if (link_ever_ && t_last_ - link_seen_ <= LINK_LOST_MS) f |= FL_LINK;
  if (baseline_ >= 0) f |= FL_CAL;
  if (done_) f |= FL_DONE;
  if (invalid_run_ >= 2) f |= FL_SENSOR_WARN;
  if (state_ == ST_PAUSED) f |= FL_PAUSED;
  if (guard_) f |= FL_GUARD;
  return f;
}

// ------------------------------------------------------------------ frames
void Drive::on_frame(const Frame& fr, uint32_t now) {
  ++frames_;
  link_seen_ = now;
  link_ever_ = true;
  if (fr.type == 'H') {
    temp_dC_ = (int16_t)fr.get(0, -999);
    return;
  }
  if (fr.type != 'C' || fr.n < 2) return;
  uint16_t seq = (uint16_t)fr.get(0);
  ack_now_ = true;
  if (seq == last_seq_ && seq != 0) return;  // a retry of something already done
  last_seq_ = seq;
  command((uint8_t)fr.get(1), fr.get(2), fr.get(3), now);
}

void Drive::start_auto(uint32_t now) {
  mode_ = MODE_AUTO;
  hills_ = 0; row_ = 0; turn_left_ = true;
  done_ = false; edge_flag_ = false; guard_ = false;
  drive_until_ = 0; pump_until_ = 0; mseed_phase_ = 0; mseed_queued_ = 0;
  enter(ST_CALIBRATE, now);
}

void Drive::command(uint8_t code, int32_t a, int32_t b, uint32_t now) {
  bool faulted = mode_ == MODE_FAULT;
  switch (code) {
    case CMD_AUTO:
      if (faulted) break;
      if (state_ == ST_PAUSED) {
        if (low_batt_) break;
        mode_ = MODE_AUTO;
        State s = paused_from_;
        if (s == ST_SEED) s = ST_SETTLE;       // dispense that hill again from the start
        else if (s == ST_WATER) s = ST_DRIVE;  // skip the rest of the interrupted dose
        enter(s, now);
      } else {
        start_auto(now);
      }
      break;
    case CMD_MANUAL:
      if (faulted) break;
      mode_ = MODE_MANUAL;
      drive_until_ = 0;
      enter(ST_MANUAL, now);
      break;
    case CMD_STOP:
      if (faulted) break;
      mode_ = MODE_IDLE;
      drive_until_ = 0; pump_until_ = 0; mseed_phase_ = 0; mseed_queued_ = 0;
      enter(ST_IDLE, now);
      break;
    case CMD_DRIVE:
    case CMD_SEED:
    case CMD_WATER:
      if (faulted || mode_ == MODE_AUTO) break;  // manual actions never interrupt a run: STOP first
      if (mode_ != MODE_MANUAL) { mode_ = MODE_MANUAL; enter(ST_MANUAL, now); }
      if (code == CMD_DRIVE) {
        int32_t hold = b < 0 ? 0 : (b > 1000 ? 1000 : b);
        drive_dir_ = (a >= DIR_STOP && a <= DIR_RIGHT) ? (uint8_t)a : (uint8_t)DIR_STOP;
        drive_until_ = drive_dir_ == DIR_STOP ? 0 : now + (uint32_t)hold;
      } else if (code == CMD_SEED) {
        if (mseed_queued_ < 5) ++mseed_queued_;  // each request is one hill, in order
      } else {
        int32_t ms = a < 0 ? 0 : (a > 3000 ? 3000 : a);
        pump_until_ = now + (uint32_t)ms;
      }
      break;
    case CMD_REFILL: {
      int32_t ml = (a <= 0 || a > cfg_.tank_mL) ? cfg_.tank_mL : a;
      tank_uL_ = ml * 1000L;
      tank_low_ = false;
      break;
    }
    case CMD_SET:
      if (cfg_.set((uint8_t)a, b)) cfg_changed_ = true;
      break;
    case CMD_SWEEP:
      if (cfg_.sweeps_fitted && (a == 1 || a == 2)) out_.sweep[a - 1] = (uint8_t)(b < 0 ? 0 : (b > 180 ? 180 : b));
      break;
    case CMD_PAUSE:
      if (mode_ == MODE_AUTO && state_ != ST_PAUSED && state_ != ST_DONE) {
        paused_from_ = state_;
        enter(ST_PAUSED, now);
      }
      break;
    case CMD_CLEAR:
      if (faulted) {
        fault_ = FAULT_NONE;
        mode_ = MODE_IDLE;
        enter(ST_IDLE, now);
      }
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ sensing
void Drive::ground_update(const Inputs& in) {
  uint32_t now = in.now_ms;
  bool moving = out_.left == 1 || out_.left == -1 || out_.right == 1 || out_.right == -1;
  if (moving) still_since_ = now;
  if (!in.ping) return;
  last_ping_ = now;
  int16_t g = in.ground_mm;
  ground_ = g;
  if (g < 0) {
    if (invalid_run_ < 255) ++invalid_run_;
    return;
  }
  invalid_run_ = 0;
  if (baseline_ >= 0 && g > baseline_ + cfg_.edge_mm) {
    if (edge_run_ < 255) ++edge_run_;
  } else {
    edge_run_ = 0;
  }
  // learn the ground distance while standing still on consistent readings
  bool still = now - still_since_ >= STILL_MS;
  if (!still || edge_run_ || g < 15 || g > 300) { win_n_ = 0; return; }
  if (win_n_ < 7) win_[win_n_++] = g;
  else { for (uint8_t i = 1; i < 7; ++i) win_[i - 1] = win_[i]; win_[6] = g; }
  if (win_n_ < 7) return;
  int16_t s[7];
  for (uint8_t i = 0; i < 7; ++i) s[i] = win_[i];
  for (uint8_t i = 1; i < 7; ++i)  // insertion sort
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; --j) { int16_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
  if (s[6] - s[0] > 15) return;
  int16_t med = s[3];
  if (baseline_ < 0 || (med - baseline_ <= 30 && baseline_ - med <= 30)) baseline_ = med;
}

bool Drive::ground_ahead(uint32_t now) const {
  // Two readings beyond the threshold mean the ground has dropped away. A lone
  // missing echo happens now and then on soft soil, so it takes three in a row.
  return baseline_ >= 0 && edge_run_ < 2 && invalid_run_ < 3 && now - last_ping_ <= PING_STALE_MS;
}

void Drive::pump_account(uint32_t dt) {
  if (!out_.pump) return;
  tank_uL_ -= (int32_t)cfg_.pump_mL_s10 * (int32_t)dt / 10;
  if (tank_uL_ < 0) tank_uL_ = 0;
}

// ------------------------------------------------------------------ control
void Drive::tick(const Inputs& in) {
  uint32_t now = in.now_ms;
  uint32_t dt = now - t_last_;
  if (dt > 200) dt = 200;
  t_last_ = now;

  if (in.batt_mV) {
    batt_mV_ = in.batt_mV;
    if ((int32_t)batt_mV_ < (int32_t)cfg_.low_batt_mV) {
      if (!low_since_) low_since_ = now ? now : 1;
      if (now - low_since_ >= LOW_BATT_MS) low_batt_ = true;
    } else {
      low_since_ = 0;
      if ((int32_t)batt_mV_ > (int32_t)cfg_.low_batt_mV + 300) low_batt_ = false;
    }
  }

  pump_account(dt);  // for the dt that just ended, with the pump state it had
  ground_update(in);

  switch (mode_) {
    case MODE_AUTO:
      run_auto(now, dt);
      break;
    case MODE_MANUAL:
      run_manual(now);
      break;
    case MODE_IDLE:
    case MODE_FAULT:
      motors(0, 0);
      out_.pump = false;
      out_.gate_deg = (uint8_t)cfg_.gate_closed_deg;
      break;
  }
  if (mode_ == MODE_IDLE && state_ != ST_IDLE && state_ != ST_DONE) enter(ST_IDLE, now);
}

void Drive::run_manual(uint32_t now) {
  if (link_ever_ && now - link_seen_ > LINK_LOST_MS) drive_until_ = 0;  // phone gone: stop
  int8_t l = 0, r = 0;
  guard_ = false;
  if (drive_until_ && (int32_t)(drive_until_ - now) > 0) {
    switch (drive_dir_) {
      case DIR_FWD:
        if (ground_ahead(now)) { l = 1; r = 1; }
        else { l = 2; r = 2; guard_ = true; }
        break;
      case DIR_BACK: l = -1; r = -1; break;
      case DIR_LEFT: l = -1; r = 1; break;
      case DIR_RIGHT: l = 1; r = -1; break;
      default: break;
    }
  } else {
    drive_until_ = 0;
  }
  motors(l, r);

  out_.gate_deg = (uint8_t)cfg_.gate_closed_deg;
  if (mseed_phase_ == 0 && mseed_queued_) { --mseed_queued_; mseed_phase_ = 1; mseed_t_ = now; }
  if (mseed_phase_ == 1) {
    out_.gate_deg = (uint8_t)cfg_.gate_open_deg;
    if (now - mseed_t_ >= GATE_TRAVEL_MS + (uint32_t)cfg_.seed_ms) { mseed_phase_ = 2; mseed_t_ = now; }
  } else if (mseed_phase_ == 2) {
    if (now - mseed_t_ >= GATE_CLOSE_MS) { mseed_phase_ = 0; ++hills_; }
  }
  out_.pump = pump_until_ && (int32_t)(pump_until_ - now) > 0 && tank_uL_ > 0;
  if (!out_.pump) pump_until_ = 0;
}

void Drive::run_auto(uint32_t now, uint32_t dt) {
  uint32_t in_state = now - t_enter_;
  if (low_batt_ && state_ != ST_PAUSED && state_ != ST_DONE) {
    paused_from_ = state_;
    enter(ST_PAUSED, now);
  }
  out_.pump = false;
  out_.gate_deg = (uint8_t)cfg_.gate_closed_deg;
  int32_t v = speed_mm_s();
  int8_t turn_l = turn_left_ ? -1 : 1, turn_r = turn_left_ ? 1 : -1;

  switch (state_) {
    case ST_CALIBRATE:
      motors(0, 0);
      if (ground_ahead(now)) {
        dist_um_ = 0;
        enter(ST_SETTLE, now);
      } else if (in_state >= CALIBRATE_TIMEOUT_MS) {
        fault_ = invalid_run_ >= 2 ? FAULT_SENSOR : FAULT_NO_GROUND;
        mode_ = MODE_FAULT;
        enter(ST_FAULT, now);
      }
      break;

    case ST_SETTLE:
      if (in_state < BRAKE_MS) motors(2, 2); else motors(0, 0);
      if (in_state >= SETTLE_MS) enter(ST_SEED, now);
      break;

    case ST_SEED:
      motors(0, 0);
      if (phase_ == 0) {
        out_.gate_deg = (uint8_t)cfg_.gate_open_deg;
        if (in_state >= GATE_TRAVEL_MS + (uint32_t)cfg_.seed_ms) { phase_ = 1; phase_t_ = now; }
      } else if (now - phase_t_ >= GATE_CLOSE_MS) {
        ++hills_;
        enter(ST_WATER, now);
        // size the dose now, from what the tank can still give
        int32_t want_uL = (int32_t)cfg_.water_mL * 1000L;
        int32_t give_uL = want_uL <= tank_uL_ ? want_uL : tank_uL_;
        tank_low_ = want_uL > 0 && tank_uL_ < want_uL;
        if (give_uL < 2000) give_uL = 0;  // not worth a squirt
        dose_ms_ = (uint32_t)(give_uL * 10L / cfg_.pump_mL_s10);
      }
      break;

    case ST_WATER:
      motors(0, 0);
      if (in_state < dose_ms_) out_.pump = true;
      else { dist_um_ = 0; enter(ST_DRIVE, now); }
      break;

    case ST_DRIVE:
      if (!ground_ahead(now)) {
        edge_flag_ = true;
        motors(2, 2);
        enter(ST_BACKOFF, now);
        break;
      }
      motors(1, 1);
      if (in_state > DRIVE_LAG_MS) dist_um_ += v * (int32_t)dt;
      if (dist_um_ >= (int32_t)cfg_.spacing_mm * 1000L) { dist_um_ = 0; enter(ST_SETTLE, now); }
      break;

    case ST_BACKOFF:
      if (phase_ == 0) {
        motors(2, 2);
        if (in_state >= 150) { phase_ = 1; phase_t_ = now; }
      } else {
        motors(-1, -1);
        if (now - phase_t_ >= ms_for(cfg_.backoff_mm)) {
          motors(2, 2);
          if (row_ + 1 < cfg_.rows) enter(ST_TURN1, now);
          else { done_ = true; mode_ = MODE_IDLE; enter(ST_DONE, now); }
        }
      }
      break;

    case ST_TURN1:
      if (in_state < BRAKE_MS) { motors(2, 2); break; }
      motors(turn_l, turn_r);
      if (in_state >= BRAKE_MS + turn_ms()) { motors(2, 2); dist_um_ = 0; enter(ST_SHIFT, now); }
      break;

    case ST_SHIFT:
      if (in_state < BRAKE_MS) { motors(2, 2); break; }
      if (!ground_ahead(now)) {  // the side of the bed: no room for another row
        motors(2, 2);
        done_ = true; mode_ = MODE_IDLE;
        enter(ST_DONE, now);
        break;
      }
      motors(1, 1);
      if (in_state > BRAKE_MS + DRIVE_LAG_MS) dist_um_ += v * (int32_t)dt;
      if (dist_um_ >= (int32_t)cfg_.row_gap_mm * 1000L) { motors(2, 2); enter(ST_TURN2, now); }
      break;

    case ST_TURN2:
      if (in_state < BRAKE_MS) { motors(2, 2); break; }
      motors(turn_l, turn_r);
      if (in_state >= BRAKE_MS + turn_ms()) {
        motors(2, 2);
        ++row_;
        turn_left_ = !turn_left_;
        dist_um_ = 0;
        enter(ST_SETTLE, now);
      }
      break;

    case ST_PAUSED:
    case ST_DONE:
    default:
      motors(0, 0);
      break;
  }
}

// ------------------------------------------------------------------ telemetry
bool Drive::telemetry_due(uint32_t now) const {
  uint32_t since = now - last_tel_;
  return ack_now_ ? since >= 20 : since >= 200;
}

Frame Drive::telemetry(uint32_t now) {
  last_tel_ = now;
  ack_now_ = false;
  Frame t;
  t.type = 'T';
  t.n = 10;
  t.f[0] = last_seq_;
  t.f[1] = state_;
  t.f[2] = mode_;
  t.f[3] = flags();
  t.f[4] = ground_;
  t.f[5] = hills_;
  t.f[6] = row_;
  t.f[7] = tank_uL_ / 1000;
  t.f[8] = batt_mV_ / 100;
  t.f[9] = fault_;
  return t;
}

}  // namespace farme
