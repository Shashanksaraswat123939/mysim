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
};
constexpr uint8_t kModeCount = 3;

const char* modeName(Mode mode);
const char* modeSummary(Mode mode);

struct Tuning {
  float pidSpeed;  // m/s base speed for PidOnly
  float kp;
  float ki;
  float kd;
};
Tuning defaultTuning();

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

class LineController {
 public:
  void begin(Mode mode, const Track* track, const RacingPlan* plan, const Tuning& tuning);

  // sensors: kSensorCount values, 0 = white, 1 = black, ordered far left to far right.
  Command update(const float* sensors, float dt);

  float progress() const { return s_; }             // m along the map (dead-reckoned)
  float lineOffset() const { return lastOffset_; }  // m, last good sensor estimate
  bool lineSeen() const { return seen_; }

 private:
  float pid(float error, float dt);

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
  bool havePrev_;
  bool seen_;
};
