// fiber.h - cooperative threads for the co-simulator.
//
// Each simulated microcontroller runs its sketch on its own fiber, so any
// sketch, including one that blocks inside loop() for as long as it likes, can
// be paused when its board's clock gets ahead of the other board and resumed
// later exactly where it stopped.
//
//   native build : ucontext (makecontext / swapcontext)
//   wasm build   : Binaryen Asyncify (unwind / rewind the wasm stack), with a
//                  private C shadow stack per fiber
#pragma once
#include <stdint.h>

namespace sim {

typedef void (*FiberEntry)(int arg);

static const int MAX_FIBERS = 8;

// (Re)initialise fiber `slot` so that the next resume starts entry(arg) from
// the top. Any suspended state of that slot is discarded.
void fiber_setup(int slot, FiberEntry entry, int arg);

// Run fiber `slot` until it yields or its entry function returns.
// Returns false once the entry function has returned.
bool fiber_resume(int slot);

// Called from inside the running fiber: hand control back to fiber_resume().
void fiber_yield();

int fiber_current();  // slot of the running fiber, -1 on the scheduler

}  // namespace sim
