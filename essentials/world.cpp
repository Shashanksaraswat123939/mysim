#include "world.h"
#include "mathlite.h"

static float tanhApprox(float z) {
  if (z > 4.97f) return 1.0f;
  if (z < -4.97f) return -1.0f;
  const float z2 = z * z;
  const float num = z * (135135.0f + z2 * (17325.0f + z2 * (378.0f + z2)));
  const float den = 135135.0f + z2 * (62370.0f + z2 * (3150.0f + z2 * 28.0f));
  return num / den;
}

static void recordTrail(World* w) {
  if (w->trailCount >= sim::kTrailCap) return;
  w->trail[w->trailCount * 2] = w->x;
  w->trail[w->trailCount * 2 + 1] = w->y;
  ++w->trailCount;
}

static void updateTrail(World* w, float dt) {
  w->trailTimer += dt;
  if (w->trailTimer >= sim::kTrailInterval) {
    w->trailTimer = 0.0f;
    recordTrail(w);
  }
}

static void buildCentre(World* w) {
  float x = 0.0f, y = 0.0f, th = 0.0f, s = 0.0f;
  uint16_t n = 0;
  w->centre[n++] = {x, y, th, w->track.segs[0].curvature, s, w->track.segs[0].line};
  for (uint8_t i = 0; i < w->track.count; ++i) {
    const Segment& seg = w->track.segs[i];
    uint16_t steps = (uint16_t)(seg.length / sim::kCentreStep + 0.999f);
    if (steps < 1) steps = 1;
    const float ds = seg.length / steps;
    for (uint16_t j = 0; j < steps && n < sim::kMaxCentre; ++j) {
      const float mid = th + 0.5f * seg.curvature * ds;
      x += ds * mlite::cos(mid);
      y += ds * mlite::sin(mid);
      th += seg.curvature * ds;
      s += ds;
      w->centre[n++] = {x, y, th, seg.curvature, s, seg.line};
    }
  }
  w->centreCount = n;
}

static uint16_t nearest(const World* w, float x, float y, uint16_t hint) {
  const uint16_t lo = hint > sim::kSearchWindow ? hint - sim::kSearchWindow : 0;
  uint16_t hi = hint + sim::kSearchWindow;
  if (hi >= w->centreCount) hi = w->centreCount - 1;
  uint16_t best = lo;
  float bestD2 = 1e30f;
  for (uint16_t i = lo; i <= hi; ++i) {
    const float dx = w->centre[i].x - x;
    const float dy = w->centre[i].y - y;
    const float d2 = dx * dx + dy * dy;
    if (d2 < bestD2) {
      bestD2 = d2;
      best = i;
    }
  }
  return best;
}

static float uniform(World* w) {
  w->rng = w->rng * 1664525u + 1013904223u;
  return (float)(w->rng >> 8) * (1.0f / 16777216.0f);
}

static float gauss(World* w) {
  return (uniform(w) + uniform(w) + uniform(w) + uniform(w) - 2.0f) * 1.7320508f;
}

static float wheelForce(const PlantParams& plant, float duty, float surfaceSpeed, float groundSpeed,
                        float normalForce, float& wheelSpeed, float& slip) {
  const float stall = plant.stallForce;
  const float a = duty * stall;
  const float b = stall / plant.freeSpeed;
  const float muN = plant.friction * normalForce;
  const float ref = mlite::fmax(mlite::fabs(groundSpeed), sim::kRefSpeed);
  const float limit = (plant.freeSpeed + mlite::fabs(groundSpeed)) * 3.0f;

  float w = surfaceSpeed;
  for (int it = 0; it < 6; ++it) {
    const float z = (w - groundSpeed) / (sim::kSlipScale * ref);
    const float t = tanhApprox(z);
    const float f = a - b * w - muN * t;
    const float df = -b - muN * (1.0f - t * t) / (sim::kSlipScale * ref);
    w -= f / df;
    w = mlite::clamp(w, -limit, limit);
  }
  wheelSpeed = w;
  slip = (w - groundSpeed) / ref;
  return mlite::clamp(a - b * w, -muN, muN);
}

static void tick(World* w) {
  const float dt = config::kControlPeriod;
  worldSense(w);
  controllerSetEncoders(&w->ctrl, w->wL, w->wR);
  const Command cmd = controllerUpdate(&w->ctrl, w->sensors, dt, w->sensors + config::kSensorCount);
  worldActuate(w, cmd.left, cmd.right, dt);
  worldJudge(w, cmd.status, dt);
}

bool worldStart(World* w, uint8_t mapIndex, Mode mode, float pidSpeed, const PlantParams& plant, uint32_t seed) {
  const MapInfo& map = mapInfo(mapIndex);
  trackInit(&w->track, map.segments, map.count);
  if (!planBuild(&w->plan, &w->track)) return false;
  buildCentre(w);

  Tuning tuning = defaultTuning();
  tuning.pidSpeed = pidSpeed;
  controllerBegin(&w->ctrl, mode, &w->track, &w->plan, tuning);

  w->plant = plant;
  w->x = plant.startX;
  w->y = plant.startY;
  w->th = plant.startHeading;
  w->vx = 0.0f;
  w->vy = 0.0f;
  w->om = 0.0f;
  w->wL = 0.0f;
  w->wR = 0.0f;
  w->slipL = 0.0f;
  w->slipR = 0.0f;
  w->fxL = 0.0f;
  w->fxR = 0.0f;
  w->cmdLeft = 0.0f;
  w->cmdRight = 0.0f;
  w->robotIdx = 0;
  w->trueOffset = 0.0f;
  w->trueS = 0.0f;
  w->derailTimer = 0.0f;
  w->time = 0.0f;
  w->lapTime = 0.0f;
  w->pending = 0.0f;
  w->trailTimer = 0.0f;
  w->trailCount = 0;
  w->outcome = Outcome::Running;
  w->rng = seed;
  worldSense(w);
  recordTrail(w);
  return true;
}

void worldSense(World* w) {
  const float c = mlite::cos(w->th);
  const float s = mlite::sin(w->th);
  for (uint8_t row = 0; row < 2; ++row) {
    const float fwd = row == 0 ? config::kSensorForward : -config::kRearRowDistance;
    for (uint8_t i = 0; i < config::kSensorCount; ++i) {
      const uint8_t k = (uint8_t)(row * config::kSensorCount + i);
      const float lat = ((config::kSensorCount - 1) * 0.5f - i) * config::kSensorSpacing;
      const float px = w->x + fwd * c - lat * s;
      const float py = w->y + fwd * s + lat * c;
      w->sensorX[k] = px;
      w->sensorY[k] = py;

      const uint16_t j = nearest(w, px, py, w->robotIdx);
      const CentrePoint& p = w->centre[j];
      const float dx = px - p.x;
      const float dy = py - p.y;
      const float d = mlite::sqrt(dx * dx + dy * dy);
      float reflect = 0.0f;
      if (p.line) reflect = mlite::clamp(1.0f - (d - 0.5f * config::kLineWidth) / 0.006f, 0.0f, 1.0f);
      w->sensors[k] = mlite::clamp(reflect + w->plant.sensorNoise * gauss(w), 0.0f, 1.0f);
    }
  }
}

void worldActuate(World* w, float leftMps, float rightMps, float dt) {
  w->cmdLeft = leftMps;
  w->cmdRight = rightMps;

  const float mass = w->plant.mass;
  const float g = sim::kGravity;
  const float normal = 0.5f * mass * g;
  const float half = 0.5f * config::kWheelBase;

  const float dutyL = mlite::clamp(leftMps / w->plant.freeSpeed, -1.0f, 1.0f) * w->plant.leftGain;
  const float dutyR = mlite::clamp(rightMps / w->plant.freeSpeed, -1.0f, 1.0f) * w->plant.rightGain;

  const float uL = w->vx - w->om * half;
  const float uR = w->vx + w->om * half;
  w->fxL = wheelForce(w->plant, dutyL, w->wL, uL, normal, w->wL, w->slipL);
  w->fxR = wheelForce(w->plant, dutyR, w->wR, uR, normal, w->wR, w->slipR);

  const float refV = mlite::fmax(mlite::fabs(w->vx), sim::kRefSpeed);
  const float fyDes = -w->plant.cornerStiffness * (w->vy / refV);
  const float muN = w->plant.friction * normal;
  const float fyLim = mlite::sqrt(mlite::fmax(0.0f, muN * muN - w->fxL * w->fxL));
  const float fyLimR = mlite::sqrt(mlite::fmax(0.0f, muN * muN - w->fxR * w->fxR));
  const float fyL = mlite::clamp(fyDes, -fyLim, fyLim);
  const float fyR = mlite::clamp(fyDes, -fyLimR, fyLimR);

  const float fRoll = w->plant.rollingResistance * mass * g * tanhApprox(w->vx / sim::kRollingSpeed);

  const float mEff = mass + 2.0f * sim::kWheelInertia / (w->plant.wheelRadius * w->plant.wheelRadius);
  const float fxTot = w->fxL + w->fxR - fRoll;
  const float fyTot = fyL + fyR;
  const float torque = half * (w->fxR - w->fxL);
  const float ax = fxTot / mEff + w->vy * w->om;
  const float ay = fyTot / mEff - w->vx * w->om;
  const float alphaZ = torque / w->plant.yawInertia;

  w->vx += ax * dt;
  w->vy += ay * dt;
  w->om += alphaZ * dt;
  w->th += w->om * dt;
  w->x += (w->vx * mlite::cos(w->th) - w->vy * mlite::sin(w->th)) * dt;
  w->y += (w->vx * mlite::sin(w->th) + w->vy * mlite::cos(w->th)) * dt;

  w->robotIdx = nearest(w, w->x, w->y, w->robotIdx);
  const CentrePoint& p = w->centre[w->robotIdx];
  const float dx = w->x - p.x;
  const float dy = w->y - p.y;
  w->trueOffset = mlite::sin(p.heading) * dx - mlite::cos(p.heading) * dy;
  w->trueS = p.s;
  w->time += dt;
  updateTrail(w, dt);
}

void worldPlaceRobot(World* w, float x, float y, float heading) {
  w->x = x;
  w->y = y;
  w->th = heading;
  w->vx = 0.0f;
  w->vy = 0.0f;
  w->om = 0.0f;
  w->wL = 0.0f;
  w->wR = 0.0f;
  w->slipL = 0.0f;
  w->slipR = 0.0f;
  w->robotIdx = 0;
  worldSense(w);
}

void worldJudge(World* w, RunStatus controllerStatus, float dt) {
  if (w->outcome != Outcome::Running) return;
  if (w->trueS >= w->track.length - sim::kFinishSlack) {
    w->outcome = Outcome::Finished;
    w->lapTime = w->time;
    return;
  }
  w->derailTimer = mlite::fabs(w->trueOffset) > sim::kDerailOffset ? w->derailTimer + dt : 0.0f;
  if (w->derailTimer > sim::kDerailTime) {
    w->outcome = Outcome::Derailed;
  } else if (controllerStatus != RunStatus::Driving) {
    w->outcome = Outcome::Stopped;
  } else if (w->time >= sim::kRunLimit) {
    w->outcome = Outcome::TimedOut;
  }
}

void worldAdvance(World* w, float seconds) {
  w->pending += seconds;
  while (w->pending >= config::kControlPeriod && w->outcome == Outcome::Running) {
    tick(w);
    w->pending -= config::kControlPeriod;
  }
  if (w->outcome != Outcome::Running) w->pending = 0.0f;
}

void worldRunToEnd(World* w) {
  while (w->outcome == Outcome::Running) tick(w);
}

float worldProgress(const World* w) {
  const float len = w->track.length;
  return len > 0.0f ? mlite::clamp(w->trueS / len, 0.0f, 1.0f) : 0.0f;
}

float worldTrueCurvature(const World* w) {
  return w->centre[w->robotIdx].curvature;
}

float worldPlanSpeedAt(const World* w, float s) {
  return planSpeedAt(&w->plan, s);
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
