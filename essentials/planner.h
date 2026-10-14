#pragma once
#include <stdint.h>
#include "track.h"

constexpr float kPlanSpacing = 0.02f;
constexpr uint16_t kMaxPlanSamples = 410;

class RacingPlan {
 public:
  bool build(const Track& track);

  float speedAt(float s) const;
  float spacing() const { return spacing_; }
  float length() const { return length_; }
  float idealLapTime() const { return lapTime_; }

 private:
  float v_[kMaxPlanSamples];
  uint16_t count_;
  float spacing_;
  float length_;
  float lapTime_;
};
