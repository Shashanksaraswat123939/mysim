#pragma once
// The simulated world: a "real" robot with imperfect motors, sensors that see the
// painted line, and ground truth used to decide whether a run succeeded.
// It runs the firmware's own controller (firmware/LineFollower) unchanged.
#include <stdint.h>
#include "config.h"
#include "line_controller.h"
#include "maps.h"
#include "planner.h"
#include "track.h"

namespace sim {
constexpr uint16_t kMaxCentre = 1000;     // 10 m of centre line at 1 cm steps
constexpr uint16_t kTrailCap = 3600;      // 72 s of trail at 50 Hz
constexpr float kCentreStep = 0.01f;
constexpr uint16_t kSearchWindow = 40;    // nearest-point search window, in samples
constexpr float kDerailOffset = 0.035f;   // beyond this the line is out of sensor reach
constexpr float kDerailTime = 0.25f;      // seconds beyond that before the robot is lost
constexpr float kFinishSlack = 0.04f;     // within this of the end counts as finished
constexpr float kRunLimit = 40.0f;        // seconds before a run times out
constexpr float kTrailInterval = 0.02f;
constexpr float kMotorTau = 0.04f;        // motor response lag, seconds
}  // namespace sim

// How the simulated robot differs from the firmware's beliefs about it. The
// controllers are never told any of these values.
struct PlantParams {
  float leftGain;      // left motor delivers this fraction of its command
  float rightGain;     // right motor delivers this fraction of its command
  float wheelNoise;    // m/s per sqrt(s) of random wheel slip
  float sensorNoise;   // reflectance noise, 1 sigma
  float startX;        // start placed slightly off the line, as by hand
  float startY;
  float startHeading;
};

// The imperfect robot used by default: a 0.6% weaker right motor, slip, sensor
// noise and a hand-placed start that is a little off the line.
constexpr PlantParams kRealRobot = {1.000f, 0.994f, 0.03f, 0.04f, -0.03f, 0.008f, 0.03f};
// Perfect hardware, for checking the controllers on their own.
constexpr PlantParams kIdealRobot = {1.000f, 1.000f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

struct CentrePoint {
  float x, y, heading, curvature, s;
  bool line;
};

enum class Outcome : uint8_t {
  Running = 0,
  Finished = 1,
  Derailed = 2,   // drifted out of sensor reach and never came back
  Stopped = 3,    // the controller gave up before the end
  TimedOut = 4,
};

const char* outcomeName(Outcome outcome);   // long text for the status panel
const char* outcomeShort(Outcome outcome);  // short text for the comparison table

class World {
 public:
  // Returns false only if the map does not fit the planner.
  bool start(uint8_t mapIndex, Mode mode, float pidSpeed,
             const PlantParams& plant = kRealRobot, uint32_t seed = 0x2545F491u);
  void advance(float seconds);
  void runToEnd();  // headless: used for the comparison table

  // Lower-level steps, used by the hardware-in-the-loop harness to run the firmware's
  // own setup() and loop() against this robot: sense, then actuate with the wheel speeds
  // the firmware chose, then judge the run.
  void sense();
  void actuate(float leftMps, float rightMps, float dt);
  void judge(RunStatus controllerStatus, float dt);
  void placeRobot(float x, float y, float heading);  // as if picked up and set down by hand

  Outcome outcome() const { return outcome_; }
  float time() const { return time_; }
  float lapTime() const { return lapTime_; }
  float progress() const;       // fraction of the map really covered, 0..1
  float trueOffset() const { return trueOffset_; }
  float estimatedOffset() const { return ctrl_.lineOffset(); }
  float estimatedProgress() const { return ctrl_.progress(); }
  bool lineSeen() const { return ctrl_.lineSeen(); }
  float wheelLeft() const { return cmdLeft_; }
  float wheelRight() const { return cmdRight_; }
  float poseX() const { return x_; }
  float poseY() const { return y_; }
  float poseHeading() const { return th_; }
  float trackLength() const { return track_.length(); }
  float planLapTime() const { return plan_.idealLapTime(); }

  const CentrePoint* centre() const { return centre_; }
  uint16_t centreCount() const { return centreCount_; }
  float planSpeedAt(float s) const { return plan_.speedAt(s); }
  float sensorValue(uint8_t i) const { return sensors_[i]; }
  float sensorX(uint8_t i) const { return sensorX_[i]; }
  float sensorY(uint8_t i) const { return sensorY_[i]; }
  const float* trailXY() const { return trail_; }
  uint16_t trailCount() const { return trailCount_; }

 private:
  void buildCentre();
  void tick();
  void updateTrail(float dt);
  uint16_t nearest(float x, float y, uint16_t hint) const;
  float uniform();
  float gauss();
  void recordTrail();

  Track track_;
  RacingPlan plan_;
  PlantParams plant_;
  LineController ctrl_;
  CentrePoint centre_[sim::kMaxCentre];
  uint16_t centreCount_;

  float x_, y_, th_;          // true pose
  float wl_, wr_;             // actual wheel speeds
  float cmdLeft_, cmdRight_;  // last command from the controller
  float sensors_[config::kSensorCount];
  float sensorX_[config::kSensorCount];
  float sensorY_[config::kSensorCount];
  uint16_t robotIdx_;
  float trueOffset_;
  float trueS_;
  float derailTimer_;
  float time_;
  float lapTime_;
  float pending_;
  float trailTimer_;
  float trail_[sim::kTrailCap * 2];
  uint16_t trailCount_;
  Outcome outcome_;
  uint32_t rng_;
};
