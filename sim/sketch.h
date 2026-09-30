// sketch.h - uniform handle on each sketch compiled into the simulator.
#pragma once
#include <stdint.h>

namespace sim {

// What a sketch can report about itself (firmware's own view of the world).
enum Probe {
  PR_MODE = 0,      // controller mode
  PR_STATE,         // controller state
  PR_CMD,           // drive: last sequence number; link: last command code
  PR_DIST_MM,       // last ground distance the firmware measured
  PR_HILLS,         // hills planted
  PR_ROW,           // current row
  PR_TANK_ML,       // water left, as estimated by the firmware
  PR_FLAGS,         // status flags
  PR_LINK_OK,       // link healthy
  PR_FRAMES_OK,     // valid frames received
  PR_FRAMES_BAD,    // frames rejected (checksum, length)
  PR_TEMP_C,        // air temperature
  PR_HUM,           // humidity
  PR_SEEDS_EST,     // seeds dispensed, as estimated
  PR_BATT_MV,       // battery as measured
  PR_COUNT
};

struct SketchIface {
  const char* name;
  void (*reset)();   // re-run the sketch's global initialisers (fresh flash)
  void (*setup)();
  void (*loop)();
  double (*probe)(int what);
  const char* (*ui_html)();  // the page the board serves at "/", if any
};

extern const SketchIface SK_DRIVE;   // firmware/FarmeDrive (UNO)
extern const SketchIface SK_LINK;    // firmware/FarmeLink (NodeMCU)

}  // namespace sim
