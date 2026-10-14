#include "world.h"
#include "mathlite.h"

namespace {

// tanh by a continued-fraction (Lambert) form, accurate to about 1e-6 for |z| < 5.
// The simulator needs it for the tyre curves; the firmware never does.
float tanhApprox(float z) {
  if (z > 4.97f) return 1.0f;
  if (z < -4.97f) return -1.0f;
  const float z2 = z * z;
  const float num = z * (135135.0f + z2 * (17325.0f + z2 * (378.0f + z2)));
  const float den = 135135.0f + z2 * (62370.0f + z2 * (3150.0f + z2 * 28.0f));
  return num / den;
}

}  // namespace

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
  vx_ = 0.0f;
  vy_ = 0.0f;
  om_ = 0.0f;
  wL_ = 0.0f;
  wR_ = 0.0f;
  slipL_ = 0.0f;
  slipR_ = 0.0f;
  fxL_ = 0.0f;
  fxR_ = 0.0f;
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
  // Two rows: the front bar ahead of the axle, and the rear bar behind it. Both use the same sensors.
  const float c = mlite::cos(th_);
  const float s = mlite::sin(th_);
  for (uint8_t row = 0; row < 2; ++row) {
    const float fwd = row == 0 ? config::kSensorForward : -config::kRearRowDistance;
    for (uint8_t i = 0; i < config::kSensorCount; ++i) {
      const uint8_t k = (uint8_t)(row * config::kSensorCount + i);
      const float lat = ((config::kSensorCount - 1) * 0.5f - i) * config::kSensorSpacing;
      const float px = x_ + fwd * c - lat * s;
      const float py = y_ + fwd * s + lat * c;
      sensorX_[k] = px;
      sensorY_[k] = py;

      // Reflectance falls off over 6 mm either side of the tape's edge.
      const uint16_t j = nearest(px, py, robotIdx_);
      const CentrePoint& p = centre_[j];
      const float dx = px - p.x;
      const float dy = py - p.y;
      const float d = mlite::sqrt(dx * dx + dy * dy);
      float reflect = 0.0f;
      if (p.line) reflect = mlite::clamp(1.0f - (d - 0.5f * config::kLineWidth) / 0.006f, 0.0f, 1.0f);
      sensors_[k] = mlite::clamp(reflect + plant_.sensorNoise * gauss(), 0.0f, 1.0f);
    }
  }
}

// One drive wheel. The motor pushes with a force that falls as the wheel speeds up (back-EMF),
// and the tyre pushes back through a slip-ratio curve capped at friction x normal force.
// The wheel has very little inertia, so its speed is the one where motor and tyre forces balance.
// That is found with a few Newton steps. Returns the longitudinal tyre force.
float World::wheelForce(float duty, float surfaceSpeed, float groundSpeed, float normalForce,
                        float& wheelSpeed, float& slip) const {
  const float stall = plant_.stallForce;
  const float a = duty * stall;                  // motor force at zero wheel speed
  const float b = stall / plant_.freeSpeed;      // force lost per m/s of wheel speed
  const float muN = plant_.friction * normalForce;
  const float ref = mlite::fmax(mlite::fabs(groundSpeed), sim::kRefSpeed);
  const float limit = (plant_.freeSpeed + mlite::fabs(groundSpeed)) * 3.0f;

  float w = surfaceSpeed;
  for (int it = 0; it < 6; ++it) {
    const float z = (w - groundSpeed) / (sim::kSlipScale * ref);
    const float t = tanhApprox(z);
    const float f = a - b * w - muN * t;                                  // motor force minus tyre force
    const float df = -b - muN * (1.0f - t * t) / (sim::kSlipScale * ref); // always negative
    w -= f / df;
    w = mlite::clamp(w, -limit, limit);
  }
  wheelSpeed = w;
  slip = (w - groundSpeed) / ref;
  return mlite::clamp(a - b * w, -muN, muN);
}

void World::actuate(float leftMps, float rightMps, float dt) {
  cmdLeft_ = leftMps;
  cmdRight_ = rightMps;

  const float mass = plant_.mass;
  const float g = sim::kGravity;
  const float normal = 0.5f * mass * g;  // weight shared equally by the two drive wheels
  const float half = 0.5f * config::kWheelBase;

  // Motor duty: the firmware's m/s command as a fraction of free speed, times motor effectiveness.
  const float dutyL = mlite::clamp(leftMps / plant_.freeSpeed, -1.0f, 1.0f) * plant_.leftGain;
  const float dutyR = mlite::clamp(rightMps / plant_.freeSpeed, -1.0f, 1.0f) * plant_.rightGain;

  // Ground speed under each contact patch. Yaw makes the two wheels move at different speeds.
  const float uL = vx_ - om_ * half;
  const float uR = vx_ + om_ * half;
  fxL_ = wheelForce(dutyL, wL_, uL, normal, wL_, slipL_);
  fxR_ = wheelForce(dutyR, wR_, uR, normal, wR_, slipR_);

  // Sideways: the tyre's slip angle makes a side force, limited by the friction circle.
  const float refV = mlite::fmax(mlite::fabs(vx_), sim::kRefSpeed);
  const float fyDes = -plant_.cornerStiffness * (vy_ / refV);
  const float muN = plant_.friction * normal;
  const float fyLim = mlite::sqrt(mlite::fmax(0.0f, muN * muN - fxL_ * fxL_));
  const float fyLimR = mlite::sqrt(mlite::fmax(0.0f, muN * muN - fxR_ * fxR_));
  const float fyL = mlite::clamp(fyDes, -fyLim, fyLim);
  const float fyR = mlite::clamp(fyDes, -fyLimR, fyLimR);

  // Rolling resistance opposes forward motion.
  const float fRoll = plant_.rollingResistance * mass * g * tanhApprox(vx_ / sim::kRollingSpeed);

  // Rigid body. The wheels' own inertia adds a little effective mass.
  const float mEff = mass + 2.0f * sim::kWheelInertia / (plant_.wheelRadius * plant_.wheelRadius);
  const float fxTot = fxL_ + fxR_ - fRoll;
  const float fyTot = fyL + fyR;
  const float torque = half * (fxR_ - fxL_);  // only the longitudinal forces make a yaw torque
  const float ax = fxTot / mEff + vy_ * om_;
  const float ay = fyTot / mEff - vx_ * om_;
  const float alphaZ = torque / plant_.yawInertia;

  vx_ += ax * dt;
  vy_ += ay * dt;
  om_ += alphaZ * dt;
  th_ += om_ * dt;
  x_ += (vx_ * mlite::cos(th_) - vy_ * mlite::sin(th_)) * dt;
  y_ += (vx_ * mlite::sin(th_) + vy_ * mlite::cos(th_)) * dt;

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
  vx_ = 0.0f;
  vy_ = 0.0f;
  om_ = 0.0f;
  wL_ = 0.0f;
  wR_ = 0.0f;
  slipL_ = 0.0f;
  slipR_ = 0.0f;
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
  ctrl_.setWheelEncoders(wL_, wR_);  // encoders read the wheels' rolling speed, slip included
  const Command cmd = ctrl_.update(sensors_, dt, sensors_ + config::kSensorCount);
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
