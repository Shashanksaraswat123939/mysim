#pragma once
// The three ways the robot can drive a map. All of them output wheel speeds, so the
// same code runs on the Arduino and inside the simulator.
#include <stdint.h>
#include "planner.h"
#include "track.h"

enum class Mode : uint8_t {
  PidOnly = 0,       // Normal PID: IR sensors only, no map
  FullMapBlind = 1,  // Full map: racing plan followed open loop, sensors ignored
  FullMapIr = 2,     // Full map + IR: racing plan, with sensors correcting drift
  DoubleIr = 3,      // Two sensor rows: measures the curve it is on and speeds up or slows down
};
constexpr uint8_t kModeCount = 4;

const char* modeName(Mode mode);
const char* modeSummary(Mode mode);

struct Tuning {
  float pidSpeed;  // m/s base speed for PidOnly
  float topSpeed;  // m/s cap for the map modes and Double IR
  float kp;
  float ki;
  float kd;
};
Tuning defaultTuning(Mode mode);
float modeSpeedDefault(Mode mode);
float modeSpeedMin(Mode mode);
float modeSpeedMax(Mode mode);

enum class RunStatus : uint8_t {
  Driving = 0,
  Finished = 1,  // reached the end of the map
  GaveUp = 2,    // lost the line for too long (or PID ran off the end of the tape)
};

struct Command {
  float left;   // m/s
  float right;  // m/s
  RunStatus status;
};

// Where the painted line sits under the sensor bar.
struct LineEstimate {
  bool seen;
  float offset;  // m, positive = line is to the robot's left
};
LineEstimate estimateLine(const float* sensors);

// The direction of the line relative to the robot, from two sensor rows. Along the line the
// lateral position is y(x) = y0 + theta * x + kappa * x^2 / 2, where x is distance ahead of the
// axle. Subtracting the rear row from the front row cancels y0, which is the position of the
// axle relative to the line and is unknown. The curvature term is cancelled only when the rows
// are the same distance from the axle (frontDist == rearDist), so keep them equal.
// Returns theta in radians: positive when the line runs off to the left ahead of the robot.
float lineHeading(float frontOffset, float rearOffset, float frontDist, float rearDist);

class LineController {
 public:
  void begin(Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning);

  // sensors: kSensorCount values, 0 = white, 1 = black, ordered far left to far right.
  // rearSensors: the second row, same layout. Only Double IR uses it; pass nullptr otherwise.
  Command update(const float* sensors, float dt, const float* rearSensors = nullptr);

  // Wheel speeds from the encoders, m/s, the wheel's rolling speed. Double IR turns them into the
  // robot's turn rate. Call before update() each tick.
  void setWheelEncoders(float leftMps, float rightMps) {
    encL_ = leftMps;
    encR_ = rightMps;
  }

  float progress() const { return s_; }             // m along the map (dead-reckoned)
  float lineOffset() const { return lastOffset_; }  // m, last good sensor estimate
  bool lineSeen() const { return seen_; }
  float estimatedCurvature() const { return kappa_; }  // 1/m, Double IR only

 private:
  float pid(float error, float dt, float kp, float kd);
  Command doubleIr(const LineEstimate& front, const float* rearSensors, float dt);

  Mode mode_;
  const Track* track_;
  const RacingPlan* plan_;
  Tuning tuning_;

  float s_;
  float lostTime_;
  float lastOffset_;
  float integral_;
  float prevError_;
  float dFilter_;
  float kappa_;      // Double IR: estimated curvature of the line under the robot
  float theta_;      // Double IR: smoothed direction of the line relative to the robot, rad
  float thetaRate_;  // Double IR: smoothed rate of change of theta, rad/s
  float encL_, encR_;  // Double IR: encoder speeds, m/s
  float speedCmd_;   // Double IR: the speed being commanded, rate-limited
  bool haveTheta_;   // Double IR: theta has been measured at least once
  bool havePrev_;
  bool seen_;
};
