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

float modeSpeedDefault(Mode mode) {
  switch (mode) {
    case Mode::PidOnly: return config::kPidSpeed;
    case Mode::FullMapBlind:
    case Mode::FullMapIr: return config::kPlanVMax;
    case Mode::DoubleIr: return config::kDoubleIrVMax;
  }
  return config::kPidSpeed;
}

float modeSpeedMin(Mode) { return config::kSpeedMin; }

float modeSpeedMax(Mode mode) {
  switch (mode) {
    case Mode::PidOnly: return config::kPidSpeedMax;
    case Mode::FullMapBlind:
    case Mode::FullMapIr: return config::kPlanVMax;
    case Mode::DoubleIr: return config::kDoubleIrSpeedMax;
  }
  return config::kPidSpeed;
}

Tuning defaultTuning(Mode mode) {
  Tuning t;
  t.pidSpeed = modeSpeedDefault(mode);
  t.topSpeed = modeSpeedDefault(mode);
  t.kp = config::kKp;
  t.ki = config::kKi;
  t.kd = config::kKd;
  return t;
}

LineEstimate estimateLine(const float* sensors) {
  float sum = 0.0f;
  float moment = 0.0f;
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    // Sensor 0 is the far left, so it has the most positive lateral position.
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
  // yF - yR = theta * (a + b) + kappa * (a^2 - b^2) / 2. The kappa term is zero when a == b.
  return (frontOffset - rearOffset) / (frontDist + rearDist);
}

void LineController::begin(Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning) {
  mode_ = mode;
  track_ = track;
  plan_ = plan;
  tuning_ = tuning;
  s_ = 0.0f;
  lostTime_ = 0.0f;
  lastOffset_ = 0.0f;
  integral_ = 0.0f;
  prevError_ = 0.0f;
  dFilter_ = 0.0f;
  kappa_ = 0.0f;
  theta_ = 0.0f;
  thetaRate_ = 0.0f;
  encL_ = 0.0f;
  encR_ = 0.0f;
  haveTheta_ = false;
  speedCmd_ = config::kLaunchSpeed;
  havePrev_ = false;
  seen_ = false;
}

float LineController::pid(float error, float dt, float kp, float kd) {
  float derivative = 0.0f;
  if (havePrev_) derivative = (error - prevError_) / dt;
  const float alpha = dt / (config::kDerivativeTau + dt);
  dFilter_ += alpha * (derivative - dFilter_);
  prevError_ = error;
  havePrev_ = true;

  integral_ += error * dt;
  const float ki = tuning_.ki > 0.0f ? tuning_.ki : 1.0f;
  const float limit = config::kIntegralLimit / ki;
  integral_ = mlite::clamp(integral_, -limit, limit);

  return kp * error + tuning_.ki * integral_ + kd * dFilter_;
}

Command LineController::update(const float* sensors, float dt, const float* rearSensors) {
  const LineEstimate est = estimateLine(sensors);
  seen_ = est.seen;
  if (est.seen) {
    lostTime_ = 0.0f;
    lastOffset_ = est.offset;
  } else {
    lostTime_ += dt;
  }

  if (mode_ == Mode::DoubleIr) return doubleIr(est, rearSensors, dt);

  float v = 0.0f;
  float omega = 0.0f;

  switch (mode_) {
    case Mode::PidOnly: {
      if (lostTime_ > config::kLostTimeout) return Command{0.0f, 0.0f, RunStatus::GaveUp};
      // Gains and governor are scheduled with the base speed (see config.h).
      const float blend = mlite::clamp((tuning_.pidSpeed - config::kPidGainSpeedLow) /
                                           (config::kPidGainSpeedHigh - config::kPidGainSpeedLow),
                                       0.0f, 1.0f);
      const float kp = config::kPidKpLow + (config::kPidKpHigh - config::kPidKpLow) * blend;
      const float kd = config::kPidKdLow + (config::kPidKdHigh - config::kPidKdLow) * blend;
      const float slowdown = config::kPidSlowdownLow + (config::kPidSlowdownHigh - config::kPidSlowdownLow) * blend;
      const float offset = est.seen ? est.offset : lastOffset_;  // hold the last offset in a gap
      omega = pid(offset, dt, kp, kd);
      v = tuning_.pidSpeed / (1.0f + slowdown * mlite::fabs(omega) +
                              config::kPidOffsetSlowdownHigh * blend * mlite::fabs(offset));
      break;
    }
    case Mode::FullMapBlind:
    case Mode::FullMapIr: {
      // Feed-forward: the planned speed and the map's curvature at the robot's
      // dead-reckoned position along the route.
      v = plan_->speedAt(s_);
      if (v < config::kLaunchSpeed) v = config::kLaunchSpeed;
      omega = v * track_->curvatureAt(s_);

      if (mode_ == Mode::FullMapIr) {
        // The tape ends at the finish, so with sensors the robot can see the end
        // directly: keep following past the dead-reckoned end until the line is gone.
        if (lostTime_ > config::kLostTimeout) {
          const RunStatus status = s_ >= track_->length() ? RunStatus::Finished : RunStatus::GaveUp;
          return Command{0.0f, 0.0f, status};
        }
        if (est.seen) {
          omega += pid(est.offset, dt, tuning_.kp, tuning_.kd);
        } else {
          havePrev_ = false;  // no derivative spike when the line comes back
        }
      }

      s_ += v * dt;
      if (mode_ == Mode::FullMapBlind && s_ >= track_->length()) {
        return Command{0.0f, 0.0f, RunStatus::Finished};  // blind: it can only trust the plan
      }
      break;
    }
    case Mode::DoubleIr:
      break;  // handled above, before the switch
  }

  // Differential drive: the wheel on the inside of the turn runs slower.
  const float half = 0.5f * config::kWheelBase;
  Command cmd;
  cmd.left = mlite::clamp(v - omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.right = mlite::clamp(v + omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.status = RunStatus::Driving;
  return cmd;
}

Command LineController::doubleIr(const LineEstimate& front, const float* rearSensors, float dt) {
  // With no line in sight the robot can only give up, as Normal PID does.
  if (lostTime_ > config::kLostTimeout) return Command{0.0f, 0.0f, RunStatus::GaveUp};

  // 1. Curvature of the line under the robot. The rows give the line's direction relative to the
  //    robot, theta. Between two moments, theta changes by kappa*v*dt minus the robot's own turn,
  //    so kappa = (rate of change of theta + robot turn rate) / v. The robot's turn rate comes
  //    from the encoders. Both pieces are smoothed with the same time constant.
  if (rearSensors != nullptr && front.seen) {
    const LineEstimate back = estimateLine(rearSensors);
    if (back.seen) {
      const float theta = lineHeading(front.offset, back.offset, config::kSensorForward, config::kRearRowDistance);
      const float a = dt / (config::kCurveTau + dt);
      if (!haveTheta_) {
        theta_ = theta;
        haveTheta_ = true;
      } else {
        const float prevTheta = theta_;
        theta_ += a * (theta - theta_);
        thetaRate_ += a * ((theta_ - prevTheta) / dt - thetaRate_);
      }
      const float turnRate = (encR_ - encL_) / config::kWheelBase;  // positive = turning left
      const float v = mlite::fmax(speedCmd_, config::kLaunchSpeed);
      kappa_ += a * ((thetaRate_ + turnRate) / v - kappa_);
    }
  }

  // 2. Speed from the curve: the sideways grip limit gives v = sqrt(aLat / |kappa|), so a wide
  //    bend (small kappa) allows more speed and a tight bend (large kappa) less. The speed moves
  //    no faster than the plan's acceleration and braking limits.
  const float absK = mlite::fmax(mlite::fabs(kappa_), 1e-3f);
  float target = mlite::sqrt(config::kDoubleIrLatAccel / absK);
  if (target > tuning_.topSpeed) target = tuning_.topSpeed;
  speedCmd_ = mlite::clamp(target, speedCmd_ - config::kPlanBrake * dt, speedCmd_ + config::kPlanAccel * dt);
  if (speedCmd_ < config::kLaunchSpeed) speedCmd_ = config::kLaunchSpeed;

  // 3. Steering: feed-forward from the measured curvature, plus PID on the front row's offset.
  float omega = speedCmd_ * kappa_;
  if (front.seen) {
    lastOffset_ = front.offset;
    omega += pid(front.offset, dt, config::kDoubleIrKp, config::kDoubleIrKd);
  } else {
    havePrev_ = false;  // no derivative spike when the line comes back
  }

  const float half = 0.5f * config::kWheelBase;
  Command cmd;
  cmd.left = mlite::clamp(speedCmd_ - omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.right = mlite::clamp(speedCmd_ + omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.status = RunStatus::Driving;
  return cmd;
}
