#include "line_controller.h"
#include "config.h"
#include "mathlite.h"

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::PidOnly: return "Normal PID";
    case Mode::FullMapBlind: return "Full map (blind)";
    case Mode::FullMapIr: return "Full map + IR correction";
    case Mode::DoubleIr: return "Double IR (two rows)";
  }
  return "unknown";
}

const char* modeSummary(Mode mode) {
  switch (mode) {
    case Mode::PidOnly:
      return "Steers only from the IR sensors. No map, so it cannot slow down before a corner.";
    case Mode::FullMapBlind:
      return "Knows the whole map and replays the racing speed profile open loop. The sensors are ignored.";
    case Mode::FullMapIr:
      return "Replays the racing profile for speed and steering, and uses the IR sensors to correct drift and bridge gaps.";
    case Mode::DoubleIr:
      return "Two sensor rows, ahead of and behind the axle. Measures the curve it is in, and slows for tight bends and speeds up on wide ones. No map.";
  }
  return "";
}

Tuning defaultTuning() {
  Tuning t;
  t.pidSpeed = config::kPidSpeed;
  t.kp = config::kKp;
  t.ki = config::kKi;
  t.kd = config::kKd;
  return t;
}

LineEstimate estimateLine(const float* sensors) {
  float sum = 0.0f;
  float moment = 0.0f;
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    const float x = ((config::kSensorCount - 1) * 0.5f - i) * config::kSensorSpacing;
    sum += sensors[i];
    moment += sensors[i] * x;
  }
  LineEstimate e;
  e.seen = sum > 0.4f;
  e.offset = e.seen ? moment / sum : 0.0f;
  return e;
}

float lineHeading(float frontOffset, float rearOffset, float frontDist, float rearDist) {
  return (frontOffset - rearOffset) / (frontDist + rearDist);
}

void controllerBegin(Controller* c, Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning) {
  c->mode = mode;
  c->track = track;
  c->plan = plan;
  c->tuning = tuning;
  c->s = 0.0f;
  c->lostTime = 0.0f;
  c->lastOffset = 0.0f;
  c->integral = 0.0f;
  c->prevError = 0.0f;
  c->dFilter = 0.0f;
  c->kappa = 0.0f;
  c->theta = 0.0f;
  c->thetaRate = 0.0f;
  c->encL = 0.0f;
  c->encR = 0.0f;
  c->haveTheta = false;
  c->speedCmd = config::kLaunchSpeed;
  c->havePrev = false;
  c->seen = false;
}

void controllerSetEncoders(Controller* c, float leftMps, float rightMps) {
  c->encL = leftMps;
  c->encR = rightMps;
}

static Command stopped(RunStatus status) {
  Command cmd = {0.0f, 0.0f, status};
  return cmd;
}

static Command drive(float v, float omega) {
  const float half = 0.5f * config::kWheelBase;
  Command cmd;
  cmd.left = mlite::clamp(v - omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.right = mlite::clamp(v + omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.status = RunStatus::Driving;
  return cmd;
}

static float pidStep(Controller* c, float error, float dt, float kp, float kd) {
  float derivative = 0.0f;
  if (c->havePrev) derivative = (error - c->prevError) / dt;
  const float alpha = dt / (config::kDerivativeTau + dt);
  c->dFilter += alpha * (derivative - c->dFilter);
  c->prevError = error;
  c->havePrev = true;

  c->integral += error * dt;
  const float ki = c->tuning.ki > 0.0f ? c->tuning.ki : 1.0f;
  const float limit = config::kIntegralLimit / ki;
  c->integral = mlite::clamp(c->integral, -limit, limit);

  return kp * error + c->tuning.ki * c->integral + kd * c->dFilter;
}

static Command pidOnlyStep(Controller* c, const LineEstimate& est, float dt) {
  if (c->lostTime > config::kLostTimeout) return stopped(RunStatus::GaveUp);

  const float blend = mlite::clamp((c->tuning.pidSpeed - config::kPidGainSpeedLow) /
                                       (config::kPidGainSpeedHigh - config::kPidGainSpeedLow),
                                   0.0f, 1.0f);
  const float kp = config::kPidKpLow + (config::kPidKpHigh - config::kPidKpLow) * blend;
  const float kd = config::kPidKdLow + (config::kPidKdHigh - config::kPidKdLow) * blend;
  const float slowdown = config::kPidSlowdownLow + (config::kPidSlowdownHigh - config::kPidSlowdownLow) * blend;
  const float offset = est.seen ? est.offset : c->lastOffset;
  const float omega = pidStep(c, offset, dt, kp, kd);
  const float v = c->tuning.pidSpeed / (1.0f + slowdown * mlite::fabs(omega) +
                                        config::kPidOffsetSlowdownHigh * blend * mlite::fabs(offset));
  return drive(v, omega);
}

static Command fullMapStep(Controller* c, const LineEstimate& est, float dt) {
  float v = planSpeedAt(c->plan, c->s);
  if (v < config::kLaunchSpeed) v = config::kLaunchSpeed;
  float omega = v * trackCurvatureAt(c->track, c->s);

  if (c->mode == Mode::FullMapIr) {
    if (c->lostTime > config::kLostTimeout) {
      const RunStatus status = c->s >= c->track->length ? RunStatus::Finished : RunStatus::GaveUp;
      return stopped(status);
    }
    if (est.seen) {
      omega += pidStep(c, est.offset, dt, c->tuning.kp, c->tuning.kd);
    } else {
      c->havePrev = false;
    }
  }

  c->s += v * dt;
  if (c->mode == Mode::FullMapBlind && c->s >= c->track->length) return stopped(RunStatus::Finished);
  return drive(v, omega);
}

static Command doubleIrStep(Controller* c, const LineEstimate& front, const float* rear, float dt) {
  if (c->lostTime > config::kLostTimeout) return stopped(RunStatus::GaveUp);

  if (rear != nullptr && front.seen) {
    const LineEstimate back = estimateLine(rear);
    if (back.seen) {
      const float theta = lineHeading(front.offset, back.offset, config::kSensorForward, config::kRearRowDistance);
      const float a = dt / (config::kCurveTau + dt);
      if (!c->haveTheta) {
        c->theta = theta;
        c->haveTheta = true;
      } else {
        const float prevTheta = c->theta;
        c->theta += a * (theta - c->theta);
        c->thetaRate += a * ((c->theta - prevTheta) / dt - c->thetaRate);
      }
      const float turnRate = (c->encR - c->encL) / config::kWheelBase;
      const float v = mlite::fmax(c->speedCmd, config::kLaunchSpeed);
      c->kappa += a * ((c->thetaRate + turnRate) / v - c->kappa);
    }
  }

  const float absK = mlite::fmax(mlite::fabs(c->kappa), 1e-3f);
  float target = mlite::sqrt(config::kDoubleIrLatAccel / absK);
  if (target > config::kDoubleIrVMax) target = config::kDoubleIrVMax;
  c->speedCmd = mlite::clamp(target, c->speedCmd - config::kPlanBrake * dt, c->speedCmd + config::kPlanAccel * dt);
  if (c->speedCmd < config::kLaunchSpeed) c->speedCmd = config::kLaunchSpeed;

  float omega = c->speedCmd * c->kappa;
  if (front.seen) {
    c->lastOffset = front.offset;
    omega += pidStep(c, front.offset, dt, config::kDoubleIrKp, config::kDoubleIrKd);
  } else {
    c->havePrev = false;
  }
  return drive(c->speedCmd, omega);
}

Command controllerUpdate(Controller* c, const float* sensors, float dt, const float* rearSensors) {
  const LineEstimate est = estimateLine(sensors);
  c->seen = est.seen;
  if (est.seen) {
    c->lostTime = 0.0f;
    c->lastOffset = est.offset;
  } else {
    c->lostTime += dt;
  }

  if (c->mode == Mode::DoubleIr) return doubleIrStep(c, est, rearSensors, dt);
  if (c->mode == Mode::PidOnly) return pidOnlyStep(c, est, dt);
  return fullMapStep(c, est, dt);
}
