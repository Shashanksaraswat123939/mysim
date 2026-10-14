#include "planner.h"
#include "config.h"
#include "mathlite.h"

bool planBuild(RacingPlan* p, const Track* track) {
  const float len = track->length;
  const float samples = len / kPlanSpacing;
  if (samples > (float)kMaxPlanSamples) return false;
  uint16_t n = (uint16_t)samples;
  if ((float)n * kPlanSpacing < len) ++n;
  n += 1;
  if (n > kMaxPlanSamples || n < 2) return false;
  p->count = n;
  p->length = len;
  p->spacing = len / (float)(n - 1);
  float* v = p->v;

  for (uint16_t i = 0; i < n; ++i) {
    float k = mlite::fabs(trackCurvatureAt(track, i * p->spacing));
    if (i > 0) k = mlite::fmax(k, mlite::fabs(trackCurvatureAt(track, (i - 1) * p->spacing)));
    if (i + 1 < n) k = mlite::fmax(k, mlite::fabs(trackCurvatureAt(track, (i + 1) * p->spacing)));
    float top = config::kPlanVMax;
    if (k > 1e-4f) {
      const float corner = mlite::sqrt(config::kPlanLatAccel / k);
      if (corner < top) top = corner;
    }
    v[i] = top;
  }
  v[0] = config::kLaunchSpeed;
  v[n - 1] = config::kLaunchSpeed;

  for (uint16_t i = 1; i < n; ++i) {
    const float limit = mlite::sqrt(v[i - 1] * v[i - 1] + 2.0f * config::kPlanAccel * p->spacing);
    if (v[i] > limit) v[i] = limit;
  }

  for (uint16_t i = n - 1; i-- > 0;) {
    const float limit = mlite::sqrt(v[i + 1] * v[i + 1] + 2.0f * config::kPlanBrake * p->spacing);
    if (v[i] > limit) v[i] = limit;
  }

  float t = 0.0f;
  for (uint16_t i = 0; i + 1 < n; ++i) {
    t += p->spacing / (0.5f * (v[i] + v[i + 1]));
  }
  p->lapTime = t;
  return true;
}

float planSpeedAt(const RacingPlan* p, float s) {
  if (s <= 0.0f) return p->v[0];
  const float idx = s / p->spacing;
  if (idx >= (float)(p->count - 1)) return p->v[p->count - 1];
  const uint16_t i = (uint16_t)idx;
  const float f = idx - (float)i;
  return p->v[i] + (p->v[i + 1] - p->v[i]) * f;
}
