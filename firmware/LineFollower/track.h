#pragma once
#include <stdint.h>

struct Segment {
  float length;
  float curvature;
  bool line;
};

constexpr Segment Straight(float len) { return {len, 0.0f, true}; }
constexpr Segment Gap(float len) { return {len, 0.0f, false}; }
constexpr Segment LeftArc(float degrees, float radius) {
  return {degrees * 0.0174533f * radius, 1.0f / radius, true};
}
constexpr Segment RightArc(float degrees, float radius) {
  return {degrees * 0.0174533f * radius, -1.0f / radius, true};
}

struct Track {
  const Segment* segs;
  uint8_t count;
  float length;
};

void trackInit(Track* t, const Segment* segs, uint8_t count);
float trackCurvatureAt(const Track* t, float s);
