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

struct Controller {
  Mode mode;
  const Track* track;
  const RacingPlan* plan;
  Tuning tuning;

  float s;
  float lostTime;
  float lastOffset;
  float integral;
  float prevError;
  float dFilter;
  float kappa;
  float theta;
  float thetaRate;
  float encL;
  float encR;
  float speedCmd;
  bool haveTheta;
  bool havePrev;
  bool seen;
};

void controllerBegin(Controller* c, Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning);
void controllerSetEncoders(Controller* c, float leftMps, float rightMps);
Command controllerUpdate(Controller* c, const float* sensors, float dt, const float* rearSensors = nullptr);
