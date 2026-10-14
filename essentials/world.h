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

// Physical constants of the plant model.
constexpr float kGravity = 9.81f;         // m/s^2
constexpr float kWheelInertia = 4e-6f;    // kg m^2 per wheel (about a 20 g wheel of 2 cm radius)
constexpr float kSlipScale = 0.2f;        // slip ratio at which a tyre reaches about 76% of its grip
constexpr float kRefSpeed = 0.2f;         // m/s; keeps slip ratio and slip angle finite near standstill
constexpr float kRollingSpeed = 0.02f;    // m/s; rolling resistance switches sign over this speed
}  // namespace sim

// The physical robot. The controllers are never told any of these values. Every value
// is an assumption: none was measured on a real robot (see docs/PROJECT.md, section 13).
struct PlantParams {
  float mass;               // kg, whole robot
  float yawInertia;         // kg m^2 about the vertical axis
  float wheelRadius;        // m
  float friction;           // tyre-to-tape friction coefficient, mu
  float rollingResistance;  // c_rr: rolling force = c_rr * mass * g
  float stallForce;         // N each motor gives at standstill, full duty
  float freeSpeed;          // m/s, wheel surface speed at full duty with no load
  float cornerStiffness;    // N per radian of tyre slip angle, per wheel
  float leftGain;           // left motor effectiveness (multiplies its duty)
  float rightGain;          // right motor effectiveness
  float sensorNoise;        // reflectance noise, 1 sigma
  float startX;             // start placed slightly off the line, as by hand
  float startY;
  float startHeading;
};

// The physical robot used by default: a 0.3 kg robot on a tape or mat surface (mu 0.6),
// a motor 0.6% weaker on the right, sensor noise and a hand-placed start.
constexpr PlantParams kRealRobot = {
    0.30f,  5e-4f, config::kWheelRadius, 0.6f, 0.03f, 4.0f, 1.20f, 8.0f,
    1.000f, 0.994f, 0.04f, -0.03f, 0.008f, 0.03f};
// Perfect grip and no rolling loss, for checking the controllers on their own.
constexpr PlantParams kIdealRobot = {
    0.30f,  5e-4f, 0.02f, 1000.0f, 0.0f, 4.0f, 1.20f, 8.0f,
    1.0f,   1.0f,  0.0f,  0.0f,    0.0f, 0.0f};

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
  bool start(uint8_t mapIndex, Mode mode, float speed,
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
  float wheelLeft() const { return cmdLeft_; }    // commanded by the firmware, m/s
  float wheelRight() const { return cmdRight_; }
  float actualWheelLeft() const { return wL_; }   // wheel surface speed, m/s
  float actualWheelRight() const { return wR_; }
  float slipLeft() const { return slipL_; }       // slip ratio: 0 = rolling, 1 = fully spinning
  float slipRight() const { return slipR_; }
  float bodySpeed() const { return vx_; }         // forward speed of the robot, m/s
  float slideSpeed() const { return vy_; }        // sideways speed of the axle, m/s
  float poseX() const { return x_; }
  float poseY() const { return y_; }
  float poseHeading() const { return th_; }
  float trackLength() const { return track_.length(); }
  float planLapTime() const { return plan_.idealLapTime(); }

  const CentrePoint* centre() const { return centre_; }
  uint16_t centreCount() const { return centreCount_; }
  float planSpeedAt(float s) const { return plan_.speedAt(s); }
  // Sensors 0..kSensorCount-1 are the front row, kSensorCount..2*kSensorCount-1 the rear row.
  float sensorValue(uint8_t i) const { return sensors_[i]; }
  float sensorX(uint8_t i) const { return sensorX_[i]; }
  float sensorY(uint8_t i) const { return sensorY_[i]; }
  float estimatedCurvature() const { return ctrl_.estimatedCurvature(); }  // 1/m, from the controller
  float trueCurvature() const { return centre_[robotIdx_].curvature; }    // 1/m, ground truth
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
  float wheelForce(float duty, float surfaceSpeed, float groundSpeed, float normalForce,
                   float& wheelSpeed, float& slip) const;

  Track track_;
  RacingPlan plan_;
  PlantParams plant_;
  LineController ctrl_;
  CentrePoint centre_[sim::kMaxCentre];
  uint16_t centreCount_;

  float x_, y_, th_;          // true pose
  float vx_, vy_, om_;        // body velocities: forward, sideways, yaw rate
  float wL_, wR_;             // wheel surface speeds
  float slipL_, slipR_;       // slip ratios
  float fxL_, fxR_;           // longitudinal tyre forces
  float cmdLeft_, cmdRight_;  // last command from the controller
  float sensors_[2 * config::kSensorCount];
  float sensorX_[2 * config::kSensorCount];
  float sensorY_[2 * config::kSensorCount];
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
