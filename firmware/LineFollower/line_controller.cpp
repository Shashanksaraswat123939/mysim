#include "line_controller.h"
#include "config.h"
#include "mathlite.h"

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::PidOnly: return "Normal PID";
    case Mode::FullMapBlind: return "Full map (blind)";
    case Mode::FullMapIr: return "Full map + IR correction";
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

Command LineController::update(const float* sensors, float dt) {
  const LineEstimate est = estimateLine(sensors);
  seen_ = est.seen;
  if (est.seen) {
    lostTime_ = 0.0f;
    lastOffset_ = est.offset;
  } else {
    lostTime_ += dt;
  }

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
  }

  // Differential drive: the wheel on the inside of the turn runs slower.
  const float half = 0.5f * config::kWheelBase;
  Command cmd;
  cmd.left = mlite::clamp(v - omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.right = mlite::clamp(v + omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.status = RunStatus::Driving;
  return cmd;
}
