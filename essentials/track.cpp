#include "track.h"

void trackInit(Track* t, const Segment* segs, uint8_t count) {
  t->segs = segs;
  t->count = count;
  t->length = 0.0f;
  for (uint8_t i = 0; i < count; ++i) t->length += segs[i].length;
}

float trackCurvatureAt(const Track* t, float s) {
  float start = 0.0f;
  for (uint8_t i = 0; i < t->count; ++i) {
    if (s < start + t->segs[i].length) return t->segs[i].curvature;
    start += t->segs[i].length;
  }
  return 0.0f;
}
