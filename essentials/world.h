#pragma once
#include <stdint.h>
#include "config.h"
#include "line_controller.h"
#include "maps.h"
#include "planner.h"
#include "track.h"

namespace sim {
constexpr uint16_t kMaxCentre = 1000;
constexpr uint16_t kTrailCap = 3600;
constexpr float kCentreStep = 0.01f;
constexpr uint16_t kSearchWindow = 40;
constexpr float kDerailOffset = 0.035f;
constexpr float kDerailTime = 0.25f;
constexpr float kFinishSlack = 0.04f;
constexpr float kRunLimit = 40.0f;
constexpr float kTrailInterval = 0.02f;

constexpr float kGravity = 9.81f;
constexpr float kWheelInertia = 4e-6f;
constexpr float kSlipScale = 0.2f;
constexpr float kRefSpeed = 0.2f;
constexpr float kRollingSpeed = 0.02f;
}

struct PlantParams {
  float mass;
  float yawInertia;
  float wheelRadius;
  float friction;
  float rollingResistance;
  float stallForce;
  float freeSpeed;
  float cornerStiffness;
  float leftGain;
  float rightGain;
  float sensorNoise;
  float startX;
  float startY;
  float startHeading;
};

constexpr PlantParams kRealRobot = {
    0.30f,  5e-4f, config::kWheelRadius, 0.6f, 0.03f, 4.0f, 1.20f, 8.0f,
    1.000f, 0.994f, 0.04f, -0.03f, 0.008f, 0.03f};
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
  Derailed = 2,
  Stopped = 3,
  TimedOut = 4,
};

const char* outcomeName(Outcome outcome);
const char* outcomeShort(Outcome outcome);

class World {
 public:
  bool start(uint8_t mapIndex, Mode mode, float pidSpeed,
             const PlantParams& plant = kRealRobot, uint32_t seed = 0x2545F491u);
  void advance(float seconds);
  void runToEnd();

  void sense();
  void actuate(float leftMps, float rightMps, float dt);
  void judge(RunStatus controllerStatus, float dt);
  void placeRobot(float x, float y, float heading);

  Outcome outcome() const { return outcome_; }
  float time() const { return time_; }
  float lapTime() const { return lapTime_; }
  float progress() const;
  float trueOffset() const { return trueOffset_; }
  float estimatedOffset() const { return ctrl_.lineOffset(); }
  float estimatedProgress() const { return ctrl_.progress(); }
  bool lineSeen() const { return ctrl_.lineSeen(); }
  float wheelLeft() const { return cmdLeft_; }
  float wheelRight() const { return cmdRight_; }
  float actualWheelLeft() const { return wL_; }
  float actualWheelRight() const { return wR_; }
  float slipLeft() const { return slipL_; }
  float slipRight() const { return slipR_; }
  float bodySpeed() const { return vx_; }
  float slideSpeed() const { return vy_; }
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
  float estimatedCurvature() const { return ctrl_.estimatedCurvature(); }
  float trueCurvature() const { return centre_[robotIdx_].curvature; }
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

  float x_, y_, th_;
  float vx_, vy_, om_;
  float wL_, wR_;
  float slipL_, slipR_;
  float fxL_, fxR_;
  float cmdLeft_, cmdRight_;
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
