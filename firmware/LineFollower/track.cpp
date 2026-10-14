#include "track.h"

void Track::init(const Segment* segments, uint8_t count) {
  segs_ = segments;
  count_ = count;
  length_ = 0.0f;
  for (uint8_t i = 0; i < count_; ++i) length_ += segs_[i].length;
}

float Track::curvatureAt(float s) const {
  float start = 0.0f;
  for (uint8_t i = 0; i < count_; ++i) {
    if (s < start + segs_[i].length) return segs_[i].curvature;
    start += segs_[i].length;
  }
  return 0.0f;
}

bool Track::lineAt(float s) const {
  float start = 0.0f;
  for (uint8_t i = 0; i < count_; ++i) {
    if (s < start + segs_[i].length) return segs_[i].line;
    start += segs_[i].length;
  }
  return false;
}
