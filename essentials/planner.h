#pragma once
// Racing-line planner for the Full map modes. Given a whole map it produces the
// fastest speed profile the robot's grip and motor limits allow, sampled along
// the line. A line follower cannot cut corners, so "racing line" here means the
// best speed at every point of the route.
#include <stdint.h>
#include "track.h"

constexpr float kPlanSpacing = 0.02f;       // target m between plan samples
constexpr uint16_t kMaxPlanSamples = 410;   // covers 8 m of track

class RacingPlan {
 public:
  // Returns false if the map is longer than the plan buffer.
  bool build(const Track& track, float topSpeed);

  float speedAt(float s) const;   // m/s planned at arc length s
  float spacing() const { return spacing_; }  // actual m between samples; they span the map exactly
  float length() const { return length_; }
  float idealLapTime() const { return lapTime_; }  // seconds, if the plan is followed exactly

 private:
  float v_[kMaxPlanSamples];
  uint16_t count_;
  float spacing_;
  float length_;
  float lapTime_;
};
