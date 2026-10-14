#pragma once
#include <stdint.h>
#include "body.h"
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
    0.30f,  kBodyYawInertia, config::kWheelRadius, 0.6f, 0.03f, 4.0f, 1.20f, 8.0f,
    1.000f, 0.994f, 0.04f, -0.03f, 0.008f, 0.03f};
constexpr PlantParams kIdealRobot = {
    0.30f,  kBodyYawInertia, 0.02f, 1000.0f, 0.0f, 4.0f, 1.20f, 8.0f,
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

struct World {
  Track track;
  RacingPlan plan;
  PlantParams plant;
  Controller ctrl;
  CentrePoint centre[sim::kMaxCentre];
  uint16_t centreCount;

  float x, y, th;
  float vx, vy, om;
  float wL, wR;
  float slipL, slipR;
  float fxL, fxR;
  float cmdLeft, cmdRight;
  float sensors[2 * config::kSensorCount];
  float sensorX[2 * config::kSensorCount];
  float sensorY[2 * config::kSensorCount];
  uint16_t robotIdx;
  float trueOffset;
  float trueS;
  float derailTimer;
  float time;
  float lapTime;
  float pending;
  float trailTimer;
  float trail[sim::kTrailCap * 2];
  uint16_t trailCount;
  Outcome outcome;
  uint32_t rng;
};

bool worldStart(World* w, uint8_t mapIndex, Mode mode, float pidSpeed,
                const PlantParams& plant = kRealRobot, uint32_t seed = 0x2545F491u);
void worldAdvance(World* w, float seconds);
void worldRunToEnd(World* w);
void worldSense(World* w);
void worldActuate(World* w, float leftMps, float rightMps, float dt);
void worldJudge(World* w, RunStatus controllerStatus, float dt);
void worldPlaceRobot(World* w, float x, float y, float heading);
float worldProgress(const World* w);
float worldTrueCurvature(const World* w);
float worldPlanSpeedAt(const World* w, float s);
