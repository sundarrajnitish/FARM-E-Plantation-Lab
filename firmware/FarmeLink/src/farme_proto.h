// farme_proto.h - the text protocol between the NodeMCU (FarmeLink) and the
// UNO (FarmeDrive). Shared verbatim by both sketches (tests check the copies
// are identical).
//
// One frame per line, readable in any serial monitor:
//
//     $C,17,4,1,400*5B\n
//     | |  | | |   |
//     | |  | | |   +-- CRC-8 (poly 0x07, init 0) of everything between '$' and '*', 2 hex digits
//     | |  +-+-+------ integer fields (up to MAX_FIELDS, decimal, may be negative)
//     | +------------- frame type (one letter)
//     +--------------- start of frame
//
// Anything that does not parse (bad CRC, stray bytes, a line longer than
// MAX_LINE) is dropped, and the parser resynchronises on the next '$'.
//
// Frames, NodeMCU -> UNO
//   H  heartbeat       temp_dC, humidity_pct            (every 300 ms; temp -999 = unknown)
//   C  command         seq, code, a, b                   (stop-and-wait: one in flight)
// Frames, UNO -> NodeMCU
//   T  telemetry       ack_seq, state, mode, flags, ground_mm, hills, row, tank_mL, batt_dV, fault
#ifndef FARME_PROTO_H
#define FARME_PROTO_H
#include <stddef.h>
#include <stdint.h>

namespace farme {

static const uint8_t MAX_FIELDS = 10;
static const uint8_t MAX_LINE = 72;

struct Frame {
  char type = 0;
  uint8_t n = 0;
  int32_t f[MAX_FIELDS] = {};
  int32_t get(uint8_t i, int32_t dflt = 0) const { return i < n ? f[i] : dflt; }
};

// Command codes (field 1 of a 'C' frame)
enum Cmd : uint8_t {
  CMD_AUTO = 1,     // start or resume automatic planting
  CMD_MANUAL = 2,   // manual mode (drive with the pad)
  CMD_STOP = 3,     // stop everything, back to idle
  CMD_DRIVE = 4,    // a = Dir, b = hold time in ms (motion stops when it runs out)
  CMD_SEED = 5,     // dispense one hill of seed (manual)
  CMD_WATER = 6,    // a = pump time in ms (manual, <= 3000)
  CMD_REFILL = 7,   // a = mL now in the tank (0 = full)
  CMD_SET = 8,      // a = Param, b = value
  CMD_SWEEP = 9,    // a = arm (1 or 2), b = angle
  CMD_PAUSE = 10,   // pause automatic planting
  CMD_CLEAR = 11,   // clear a fault
};

enum Dir : uint8_t { DIR_STOP = 0, DIR_FWD = 1, DIR_BACK = 2, DIR_LEFT = 3, DIR_RIGHT = 4 };

enum Param : uint8_t {
  P_SPACING_MM = 1, P_ROWS, P_ROW_GAP_MM, P_WATER_ML, P_SEED_MS, P_SPEED_MM_S, P_TURN_MS_90,
  P_EDGE_MM, P_TANK_ML, P_PUMP_ML_S10, P_COUNT
};

// CRC-8/SMBUS
uint8_t crc8(const uint8_t* d, size_t n, uint8_t crc = 0);

// Writes "$T,f0,...*CC\n" into out (NUL-terminated). Returns the length, or 0 if it does not fit.
size_t encode(const Frame& fr, char* out, size_t cap);

class Parser {
 public:
  // Feed one received byte; returns true when a complete, valid frame is ready.
  bool feed(uint8_t c);
  const Frame& frame() const { return frame_; }
  uint32_t ok = 0, bad_crc = 0, bad_syntax = 0, overlong = 0;

 private:
  bool parse();
  char buf_[MAX_LINE + 1];
  uint8_t len_ = 0;
  bool in_ = false;
  Frame frame_;
};

// Sequence numbers wrap at 16 bits
inline uint16_t seq_next(uint16_t s) { return (uint16_t)(s + 1 == 0 ? 1 : s + 1); }

}  // namespace farme

#endif  // FARME_PROTO_H
