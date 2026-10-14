#include "planner.h"
#include "config.h"
#include "mathlite.h"

bool RacingPlan::build(const Track& track) {
  const float len = track.length();
  const float samples = len / kPlanSpacing;
  if (samples > (float)kMaxPlanSamples) return false;
  uint16_t n = (uint16_t)samples;
  if ((float)n * kPlanSpacing < len) ++n;
  n += 1;
  if (n > kMaxPlanSamples || n < 2) return false;
  count_ = n;
  length_ = len;
  spacing_ = len / (float)(n - 1);

  for (uint16_t i = 0; i < n; ++i) {
    float k = mlite::fabs(track.curvatureAt(i * spacing_));
    if (i > 0) k = mlite::fmax(k, mlite::fabs(track.curvatureAt((i - 1) * spacing_)));
    if (i + 1 < n) k = mlite::fmax(k, mlite::fabs(track.curvatureAt((i + 1) * spacing_)));
    float v = config::kPlanVMax;
    if (k > 1e-4f) {
      const float corner = mlite::sqrt(config::kPlanLatAccel / k);
      if (corner < v) v = corner;
    }
    v_[i] = v;
  }
  v_[0] = config::kLaunchSpeed;
  v_[n - 1] = config::kLaunchSpeed;

  for (uint16_t i = 1; i < n; ++i) {
    const float limit = mlite::sqrt(v_[i - 1] * v_[i - 1] + 2.0f * config::kPlanAccel * spacing_);
    if (v_[i] > limit) v_[i] = limit;
  }

  for (uint16_t i = n - 1; i-- > 0;) {
    const float limit = mlite::sqrt(v_[i + 1] * v_[i + 1] + 2.0f * config::kPlanBrake * spacing_);
    if (v_[i] > limit) v_[i] = limit;
  }

  float t = 0.0f;
  for (uint16_t i = 0; i + 1 < n; ++i) {
    t += spacing_ / (0.5f * (v_[i] + v_[i + 1]));
  }
  lapTime_ = t;
  return true;
}

float RacingPlan::speedAt(float s) const {
  if (s <= 0.0f) return v_[0];
  const float idx = s / spacing_;
  if (idx >= (float)(count_ - 1)) return v_[count_ - 1];
  const uint16_t i = (uint16_t)idx;
  const float f = idx - (float)i;
  return v_[i] + (v_[i + 1] - v_[i]) * f;
}
