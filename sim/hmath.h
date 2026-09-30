// hmath.h - tiny deterministic maths for the simulator.
//
// The same C++ is compiled natively (Python analysis, tests) and to
// freestanding WebAssembly (the website). WebAssembly has no libm, and both
// builds must produce bit-identical runs, so the few transcendental functions
// the robot model needs are implemented here with plain IEEE-754 arithmetic
// (build with -ffp-contract=off so no fused multiply-add sneaks in).
#pragma once
#include <stdint.h>

namespace hm {

static const double PI = 3.14159265358979323846;
static const double TWO_PI = 6.28318530717958647692;

inline double fabs_(double x) { return x < 0 ? -x : x; }
inline double sqrt_(double x) { return __builtin_sqrt(x); }  // one IEEE instruction
inline double min_(double a, double b) { return a < b ? a : b; }
inline double max_(double a, double b) { return a > b ? a : b; }
inline double clamp_(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double floor_(double x) {
  double t = (double)(int64_t)x;
  return t > x ? t - 1.0 : t;
}

inline double bits_to_double(uint64_t b) { double d; __builtin_memcpy(&d, &b, 8); return d; }

// e^x, relative error < 1e-15 over the range used here.
inline double exp_(double x) {
  const double LN2 = 0.69314718055994530942;
  if (x > 709.0) return 8.2e307;
  if (x < -708.0) return 0.0;
  double kd = x / LN2;
  int k = (int)(kd >= 0 ? kd + 0.5 : kd - 0.5);
  double r = x - (double)k * LN2;
  double term = 1.0, sum = 1.0;
  for (int i = 1; i <= 18; ++i) { term *= r / (double)i; sum += term; }
  return sum * bits_to_double((uint64_t)(k + 1023) << 52);
}

inline double wrap_pi(double x) {
  double k = floor_((x + PI) / TWO_PI);
  return x - k * TWO_PI;
}

inline double sin_(double x) {
  x = wrap_pi(x);
  double term = x, sum = x, x2 = x * x;
  for (int i = 1; i <= 12; ++i) { term *= -x2 / (double)((2 * i) * (2 * i + 1)); sum += term; }
  return sum;
}
inline double cos_(double x) { return sin_(x + PI / 2); }

// splitmix64: small, fast, identical on every platform
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed) {}
  uint64_t next() {
    uint64_t z = (s += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
  }
  double uniform() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
  // Irwin-Hall approximation of N(0,1): no transcendental calls needed
  double gauss() {
    double s12 = 0;
    for (int i = 0; i < 12; ++i) s12 += uniform();
    return s12 - 6.0;
  }
};

}  // namespace hm
