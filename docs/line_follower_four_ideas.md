# Line follower: four ways to drive

The robot is an Arduino Mega 2560 with five IR sensors. The same C++ code runs on the robot and in the browser simulator. This page explains the four ways it can drive, and shows the code for each.

Source: `firmware/LineFollower/` on branch `update-deck-and-cleanup` (commit `d4c8de3`).

## The four ideas

| # | Name | What it knows | How it steers |
|---|---|---|---|
| 1 | Normal PID | Only the five sensors | PID on the line offset. Slows down when it steers hard. |
| 2 | Full map (blind) | The whole route and the best speed at every point | Follows the plan open loop. Ignores the sensors. |
| 3 | Full map + IR | The route and the plan, plus the sensors | Follows the plan, and PID on the sensors corrects drift. |
| 4 | Double IR | Nothing stored. Two sensor rows and wheel encoders | Measures how sharp the bend is, then sets speed and steering from it. |

Every mode returns two wheel speeds, left and right. The Arduino calls `controllerUpdate()` every 2 ms (500 times a second).

## Shared code

### Reading the line

Five sensors give values from 0 (white) to 1 (black). The line position is the weighted average of the sensors that see black.

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

### Turning a speed and a turn rate into wheel speeds

`omega` is the turn rate (rad/s). Positive means turn left. The inner wheel slows down, the outer wheel speeds up.

static Command drive(float v, float omega) {
  const float half = 0.5f * config::kWheelBase;
  Command cmd;
  cmd.left = mlite::clamp(v - omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.right = mlite::clamp(v + omega * half, -config::kWheelMaxSpeed, config::kWheelMaxSpeed);
  cmd.status = RunStatus::Driving;
  return cmd;
}

### PID on the line offset

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

## 1. Normal PID

The robot only uses the sensors. It steers to cancel the line offset, and slows down when the steering effort or the offset is large. If the line is lost for 0.6 s, it stops.

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

## 2. Full map (blind)

The route and speed come from the plan. At each distance `s` along the route, the robot's turn is `speed x curvature`. It counts `s` by adding `speed x time`, and never looks at the sensors.

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

In the blind mode the `if (c->mode == Mode::FullMapIr)` block is skipped, so the sensors play no part.

## 3. Full map + IR

Same plan as blind mode. The sensor correction is added on top: PID on the line offset, with the Full map + IR gains. If the line is lost for longer than 0.6 s, the robot stops. It counts as finished if it has reached the end of the route.

The same `fullMapStep` function covers modes 2 and 3. The `FullMapIr` branch adds the sensor correction.

## 4. Double IR

No stored route. Two rows of sensors (front and rear, 4 cm from the axle) show which way the line is heading. Wheel encoders show how fast the robot is turning. From those, the bend's sharpness (curvature) is estimated. Speed is the grip limit for that bend, capped at 0.4 m/s.

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

## Which function runs for which mode

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

## Settings used

| Name | Value | Meaning |
|---|---|---|
| `kLostTimeout` | 0.60f | seconds without the line before the robot gives up |
| `kPidSpeed` | 0.25f | base speed for Normal PID (m/s) |
| `kKp` | 240.0f | steering gain, Full map + IR |
| `kKi` | 2.0f | integral gain, all PID modes |
| `kKd` | 22.0f | derivative gain, Full map + IR |
| `kDoubleIrKp` | 160.0f | steering gain, Double IR |
| `kDoubleIrKd` | 14.0f | derivative gain, Double IR |
| `kPlanVMax` | 1.00f | top speed on straights (m/s) |
| `kPlanLatAccel` | 1.50f | sideways grip limit for the racing plan (m/s^2) |
| `kDoubleIrLatAccel` | 1.6f | sideways grip limit for Double IR (m/s^2) |
| `kDoubleIrVMax` | 0.40f | speed cap for Double IR (m/s) |
| `kCurveTau` | 0.05f | smoothing time for the curve estimate (s) |

## Files

| File | What it does |
|---|---|
| `LineFollower.ino` | Arduino entry point: `setup()` and `loop()`, choose map and mode |
| `line_controller.cpp/.h` | The four driving modes |
| `planner.cpp/.h` | The racing speed plan for the route |
| `track.cpp/.h` | Route shapes (straights, arcs, gaps) |
| `maps.cpp/.h` | The six routes |
| `hardware.cpp/.h` | Motor driver, sensor reading and calibration, pins |
| `config.h` | All the tuning numbers |
| `body.h` | Turning resistance of the robot (from the part list) |
