#pragma once
#include <stdint.h>
#include "planner.h"
#include "track.h"

enum class Mode : uint8_t {
  PidOnly = 0,
  FullMapBlind = 1,
  FullMapIr = 2,
  DoubleIr = 3,
};
constexpr uint8_t kModeCount = 4;

const char* modeName(Mode mode);
const char* modeSummary(Mode mode);

struct Tuning {
  float pidSpeed;
  float kp;
  float ki;
  float kd;
};
Tuning defaultTuning();

enum class RunStatus : uint8_t {
  Driving = 0,
  Finished = 1,
  GaveUp = 2,
};

struct Command {
  float left;
  float right;
  RunStatus status;
};

struct LineEstimate {
  bool seen;
  float offset;
};
LineEstimate estimateLine(const float* sensors);

float lineHeading(float frontOffset, float rearOffset, float frontDist, float rearDist);

class LineController {
 public:
  void begin(Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning);

  Command update(const float* sensors, float dt, const float* rearSensors = nullptr);

  void setWheelEncoders(float leftMps, float rightMps) {
    encL_ = leftMps;
    encR_ = rightMps;
  }

  float progress() const { return s_; }
  float lineOffset() const { return lastOffset_; }
  bool lineSeen() const { return seen_; }
  float estimatedCurvature() const { return kappa_; }

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
  float kappa_;
  float theta_;
  float thetaRate_;
  float encL_, encR_;
  float speedCmd_;
  bool haveTheta_;
  bool havePrev_;
  bool seen_;
};
