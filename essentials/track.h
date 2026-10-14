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

class Track {
 public:
  void init(const Segment* segments, uint8_t count);

  float length() const { return length_; }
  uint8_t segmentCount() const { return count_; }
  const Segment& segment(uint8_t i) const { return segs_[i]; }

  float curvatureAt(float s) const;
  bool lineAt(float s) const;

 private:
  const Segment* segs_;
  uint8_t count_;
  float length_;
};
