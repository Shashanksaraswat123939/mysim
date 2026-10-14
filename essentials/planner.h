#pragma once
#include <stdint.h>
#include "track.h"

constexpr float kPlanSpacing = 0.02f;
constexpr uint16_t kMaxPlanSamples = 410;

struct RacingPlan {
  float v[kMaxPlanSamples];
  uint16_t count;
  float spacing;
  float length;
  float lapTime;
};

bool planBuild(RacingPlan* p, const Track* track);
float planSpeedAt(const RacingPlan* p, float s);
