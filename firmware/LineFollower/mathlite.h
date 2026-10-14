#pragma once
#include <stdint.h>

namespace mlite {

constexpr float kPi = 3.14159265f;
constexpr float kHalfPi = 1.57079633f;

inline float fabs(float x) { return x < 0.0f ? -x : x; }
inline float sqrt(float x) { return x <= 0.0f ? 0.0f : __builtin_sqrtf(x); }
inline float fmax(float a, float b) { return a > b ? a : b; }
inline float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline float wrapAngle(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

inline float sin(float x) {
  x = wrapAngle(x);
  if (x > kHalfPi) x = kPi - x;
  else if (x < -kHalfPi) x = -kPi - x;
  const float x2 = x * x;
  return x * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f - x2 / 5040.0f)));
}

inline float cos(float x) { return sin(x + kHalfPi); }

}
