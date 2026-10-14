#pragma once
// Map geometry. A map is a list of straights and arcs describing the painted
// line's centre line. The same description drives the planner, the controllers
// and the simulator.
#include <stdint.h>

struct Segment {
  float length;     // metres of centre line
  float curvature;  // 1/m: positive turns left, negative turns right, 0 is straight
  bool line;        // false marks a gap in the painted line
};

// Route-style helpers so a map reads like a set of directions.
constexpr Segment Straight(float len) { return {len, 0.0f, true}; }
constexpr Segment Gap(float len) { return {len, 0.0f, false}; }
constexpr Segment LeftArc(float degrees, float radius) {
  return {degrees * 0.0174533f * radius, 1.0f / radius, true};
}
constexpr Segment RightArc(float degrees, float radius) {
  return {degrees * 0.0174533f * radius, -1.0f / radius, true};
}

class Track {
 public:
  void init(const Segment* segments, uint8_t count);

  float length() const { return length_; }
  uint8_t segmentCount() const { return count_; }
  const Segment& segment(uint8_t i) const { return segs_[i]; }

  float curvatureAt(float s) const;  // 1/m at arc length s
  bool lineAt(float s) const;        // is the painted line present at arc length s?

 private:
  const Segment* segs_;
  uint8_t count_;
  float length_;
};
