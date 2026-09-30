// fiber.cpp - see fiber.h
#include "fiber.h"

namespace sim {
namespace {
struct Slot {
  FiberEntry entry = nullptr;
  int arg = 0;
  bool started = false;
  bool finished = false;
};
Slot g_slots[MAX_FIBERS];
int g_current = -1;
}  // namespace

int fiber_current() { return g_current; }
}  // namespace sim

#if defined(__wasm__)
// ------------------------------------------------------------------ Asyncify
// The yield is an import (env.fiber_suspend) implemented by the JavaScript or
// Node host in four lines:
//   fiber_suspend(p) { if (asyncify_get_state() === 2) asyncify_stop_rewind();
//                      else asyncify_start_unwind(p); }
// Everything else (rewind start, unwind stop, stack switching) happens here.
extern "C" {
__attribute__((import_module("asyncify"), import_name("stop_unwind"))) void asyncify_stop_unwind(void);
__attribute__((import_module("asyncify"), import_name("start_rewind"))) void asyncify_start_rewind(void*);
__attribute__((import_module("env"), import_name("fiber_suspend"))) void fiber_suspend(void*);
}

namespace sim {
namespace {
static const uint32_t ASY_BYTES = 16384;
static const uint32_t STACK_BYTES = 65536;
struct WasmFiber {
  uint32_t asy_cur, asy_end;  // Asyncify data header: [current, end)
  uint32_t saved_sp;
  uint8_t asy_buf[ASY_BYTES];
  alignas(16) uint8_t stack[STACK_BYTES];
};
WasmFiber g_wf[MAX_FIBERS];
bool g_unwinding = false;

inline uint32_t get_sp() {
  uint32_t r;
  __asm__ volatile(".globaltype __stack_pointer, i32\n global.get __stack_pointer\n local.set %0" : "=r"(r));
  return r;
}
inline void set_sp(uint32_t v) {
  __asm__ volatile(".globaltype __stack_pointer, i32\n local.get %0\n global.set __stack_pointer" ::"r"(v));
}

// On the unwind path, so Asyncify instruments it.
__attribute__((noinline)) void trampoline(int slot) {
  Slot& s = g_slots[slot];
  s.entry(s.arg);
}
}  // namespace

void fiber_setup(int slot, FiberEntry entry, int arg) {
  g_slots[slot].entry = entry;
  g_slots[slot].arg = arg;
  g_slots[slot].started = false;
  g_slots[slot].finished = false;
}

// Listed in asyncify-removelist (sim/Makefile): never instrumented.
extern "C" __attribute__((noinline)) int sim_fiber_resume(int slot) {
  Slot& s = g_slots[slot];
  WasmFiber& f = g_wf[slot];
  if (s.finished) return 0;
  uint32_t sched_sp = get_sp();
  g_current = slot;
  g_unwinding = false;
  if (!s.started) {
    s.started = true;
    set_sp((uint32_t)(uintptr_t)(f.stack + STACK_BYTES));
  } else {
    set_sp(f.saved_sp);
    asyncify_start_rewind(&f.asy_cur);  // header left as the unwind wrote it
  }
  trampoline(slot);
  if (g_unwinding) {
    asyncify_stop_unwind();
    f.saved_sp = get_sp();
  } else {
    s.finished = true;
  }
  set_sp(sched_sp);
  g_current = -1;
  return s.finished ? 0 : 1;
}

bool fiber_resume(int slot) { return sim_fiber_resume(slot) != 0; }

void fiber_yield() {
  WasmFiber& f = g_wf[g_current];
  f.asy_cur = (uint32_t)(uintptr_t)f.asy_buf;
  f.asy_end = f.asy_cur + ASY_BYTES;
  g_unwinding = true;
  fiber_suspend(&f.asy_cur);
  // resumed here after a rewind
  g_unwinding = false;
}
}  // namespace sim

#else
// ------------------------------------------------------------------ ucontext
#include <stdlib.h>
#include <ucontext.h>

namespace sim {
namespace {
static const size_t STACK_BYTES = 512 * 1024;
ucontext_t g_sched;
ucontext_t g_ctx[MAX_FIBERS];
void* g_stack[MAX_FIBERS];

void trampoline(int slot) {
  Slot& s = g_slots[slot];
  s.entry(s.arg);
  s.finished = true;
  // returning resumes uc_link (the scheduler)
}
}  // namespace

void fiber_setup(int slot, FiberEntry entry, int arg) {
  g_slots[slot].entry = entry;
  g_slots[slot].arg = arg;
  g_slots[slot].started = false;
  g_slots[slot].finished = false;
  if (!g_stack[slot]) g_stack[slot] = malloc(STACK_BYTES);
}

bool fiber_resume(int slot) {
  Slot& s = g_slots[slot];
  if (s.finished) return false;
  if (!s.started) {
    s.started = true;
    getcontext(&g_ctx[slot]);
    g_ctx[slot].uc_stack.ss_sp = g_stack[slot];
    g_ctx[slot].uc_stack.ss_size = STACK_BYTES;
    g_ctx[slot].uc_link = &g_sched;
    makecontext(&g_ctx[slot], (void (*)())trampoline, 1, slot);
  }
  g_current = slot;
  swapcontext(&g_sched, &g_ctx[slot]);
  g_current = -1;
  return !s.finished;
}

void fiber_yield() {
  int slot = g_current;
  swapcontext(&g_ctx[slot], &g_sched);
}
}  // namespace sim
#endif
