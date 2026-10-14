#include "world.h"
#include "mathlite.h"

bool World::start(uint8_t mapIndex, Mode mode, float pidSpeed, const PlantParams& plant, uint32_t seed) {
  const MapInfo& map = mapInfo(mapIndex);
  track_.init(map.segments, map.count);
  if (!plan_.build(track_)) return false;
  buildCentre();

  Tuning tuning = defaultTuning();
  tuning.pidSpeed = pidSpeed;
  ctrl_.begin(mode, &track_, &plan_, tuning);

  plant_ = plant;
  x_ = plant.startX;
  y_ = plant.startY;
  th_ = plant.startHeading;
  wl_ = 0.0f;
  wr_ = 0.0f;
  cmdLeft_ = 0.0f;
  cmdRight_ = 0.0f;
  robotIdx_ = 0;
  trueOffset_ = 0.0f;
  trueS_ = 0.0f;
  derailTimer_ = 0.0f;
  time_ = 0.0f;
  lapTime_ = 0.0f;
  pending_ = 0.0f;
  trailTimer_ = 0.0f;
  trailCount_ = 0;
  outcome_ = Outcome::Running;
  rng_ = seed;
  sense();
  recordTrail();
  return true;
}

void World::buildCentre() {
  float x = 0.0f, y = 0.0f, th = 0.0f, s = 0.0f;
  uint16_t n = 0;
  centre_[n++] = {x, y, th, track_.segment(0).curvature, s, track_.segment(0).line};
  for (uint8_t i = 0; i < track_.segmentCount(); ++i) {
    const Segment& seg = track_.segment(i);
    uint16_t steps = (uint16_t)(seg.length / sim::kCentreStep + 0.999f);
    if (steps < 1) steps = 1;
    const float ds = seg.length / steps;
    for (uint16_t j = 0; j < steps && n < sim::kMaxCentre; ++j) {
      const float mid = th + 0.5f * seg.curvature * ds;  // midpoint rule for the arc
      x += ds * mlite::cos(mid);
      y += ds * mlite::sin(mid);
      th += seg.curvature * ds;
      s += ds;
      centre_[n++] = {x, y, th, seg.curvature, s, seg.line};
    }
  }
  centreCount_ = n;
}

uint16_t World::nearest(float x, float y, uint16_t hint) const {
  const uint16_t lo = hint > sim::kSearchWindow ? hint - sim::kSearchWindow : 0;
  uint16_t hi = hint + sim::kSearchWindow;
  if (hi >= centreCount_) hi = centreCount_ - 1;
  uint16_t best = lo;
  float bestD2 = 1e30f;
  for (uint16_t i = lo; i <= hi; ++i) {
    const float dx = centre_[i].x - x;
    const float dy = centre_[i].y - y;
    const float d2 = dx * dx + dy * dy;
    if (d2 < bestD2) {
      bestD2 = d2;
      best = i;
    }
  }
  return best;
}

float World::uniform() {
  rng_ = rng_ * 1664525u + 1013904223u;
  return (float)(rng_ >> 8) * (1.0f / 16777216.0f);
}

float World::gauss() {
  // Sum of four uniforms, rescaled to unit variance: close enough for sensor noise.
  return (uniform() + uniform() + uniform() + uniform() - 2.0f) * 1.7320508f;
}

void World::sense() {
  const float c = mlite::cos(th_);
  const float s = mlite::sin(th_);
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    const float lat = ((config::kSensorCount - 1) * 0.5f - i) * config::kSensorSpacing;
    const float px = x_ + config::kSensorForward * c - lat * s;
    const float py = y_ + config::kSensorForward * s + lat * c;
    sensorX_[i] = px;
    sensorY_[i] = py;

    // Reflectance falls off over 6 mm either side of the tape's edge.
    const uint16_t j = nearest(px, py, robotIdx_);
    const CentrePoint& p = centre_[j];
    const float dx = px - p.x;
    const float dy = py - p.y;
    const float d = mlite::sqrt(dx * dx + dy * dy);
    float reflect = 0.0f;
    if (p.line) reflect = mlite::clamp(1.0f - (d - 0.5f * config::kLineWidth) / 0.006f, 0.0f, 1.0f);
    sensors_[i] = mlite::clamp(reflect + plant_.sensorNoise * gauss(), 0.0f, 1.0f);
  }
}

void World::actuate(float leftMps, float rightMps, float dt) {
  cmdLeft_ = leftMps;
  cmdRight_ = rightMps;

  // The real motors lag, mismatch and slip.
  const float a = mlite::clamp(dt / sim::kMotorTau, 0.0f, 1.0f);
  const float slip = plant_.wheelNoise * mlite::sqrt(dt);
  wl_ += (leftMps * plant_.leftGain - wl_) * a + slip * gauss();
  wr_ += (rightMps * plant_.rightGain - wr_) * a + slip * gauss();

  const float v = 0.5f * (wl_ + wr_);
  const float w = (wr_ - wl_) / config::kWheelBase;
  th_ += w * dt;
  x_ += v * mlite::cos(th_) * dt;
  y_ += v * mlite::sin(th_) * dt;

  // Ground truth: how far the robot really is from the line (+ve: line is left of robot).
  robotIdx_ = nearest(x_, y_, robotIdx_);
  const CentrePoint& p = centre_[robotIdx_];
  const float dx = x_ - p.x;
  const float dy = y_ - p.y;
  trueOffset_ = mlite::sin(p.heading) * dx - mlite::cos(p.heading) * dy;
  trueS_ = p.s;
  time_ += dt;
  updateTrail(dt);
}

void World::placeRobot(float x, float y, float heading) {
  x_ = x;
  y_ = y;
  th_ = heading;
  wl_ = 0.0f;
  wr_ = 0.0f;
  robotIdx_ = 0;
  sense();
}

void World::judge(RunStatus controllerStatus, float dt) {
  if (outcome_ != Outcome::Running) return;
  if (trueS_ >= track_.length() - sim::kFinishSlack) {
    outcome_ = Outcome::Finished;
    lapTime_ = time_;
    return;
  }
  derailTimer_ = mlite::fabs(trueOffset_) > sim::kDerailOffset ? derailTimer_ + dt : 0.0f;
  if (derailTimer_ > sim::kDerailTime) {
    outcome_ = Outcome::Derailed;
  } else if (controllerStatus != RunStatus::Driving) {
    outcome_ = Outcome::Stopped;
  } else if (time_ >= sim::kRunLimit) {
    outcome_ = Outcome::TimedOut;
  }
}

void World::tick() {
  const float dt = config::kControlPeriod;
  sense();
  const Command cmd = ctrl_.update(sensors_, dt);
  actuate(cmd.left, cmd.right, dt);
  judge(cmd.status, dt);
}

void World::updateTrail(float dt) {
  trailTimer_ += dt;
  if (trailTimer_ >= sim::kTrailInterval) {
    trailTimer_ = 0.0f;
    recordTrail();
  }
}

void World::recordTrail() {
  if (trailCount_ >= sim::kTrailCap) return;
  trail_[trailCount_ * 2] = x_;
  trail_[trailCount_ * 2 + 1] = y_;
  ++trailCount_;
}

void World::advance(float seconds) {
  pending_ += seconds;
  while (pending_ >= config::kControlPeriod && outcome_ == Outcome::Running) {
    tick();
    pending_ -= config::kControlPeriod;
  }
  if (outcome_ != Outcome::Running) pending_ = 0.0f;
}

void World::runToEnd() {
  while (outcome_ == Outcome::Running) tick();
}

float World::progress() const {
  const float len = track_.length();
  return len > 0.0f ? mlite::clamp(trueS_ / len, 0.0f, 1.0f) : 0.0f;
}

const char* outcomeName(Outcome outcome) {
  switch (outcome) {
    case Outcome::Running: return "Driving";
    case Outcome::Finished: return "Finished";
    case Outcome::Derailed: return "Derailed: drifted off the line";
    case Outcome::Stopped: return "Stopped: gave up on the line";
    case Outcome::TimedOut: return "Timed out";
  }
  return "";
}

const char* outcomeShort(Outcome outcome) {
  switch (outcome) {
    case Outcome::Running: return "running";
    case Outcome::Finished: return "finished";
    case Outcome::Derailed: return "derailed";
    case Outcome::Stopped: return "stopped";
    case Outcome::TimedOut: return "timed out";
  }
  return "";
}
