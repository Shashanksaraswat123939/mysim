#pragma once
#include <stdint.h>

namespace config {

constexpr float kWheelBase = 0.090f;
constexpr uint8_t kSensorCount = 5;
constexpr float kSensorSpacing = 0.012f;
constexpr float kSensorForward = 0.040f;
constexpr float kLineWidth = 0.020f;
constexpr float kWheelRadius = 0.020f;
constexpr float kWheelWidth = 0.012f;
constexpr float kBodyLength = 0.064f;
constexpr float kBodyHeight = 0.022f;
constexpr float kBodyClearance = 0.008f;

constexpr float kControlPeriod = 0.002f;
constexpr float kWheelMaxSpeed = 1.20f;
constexpr float kLaunchSpeed = 0.15f;
constexpr float kLostTimeout = 0.60f;

constexpr float kPlanVMax = 1.00f;
constexpr float kPlanLatAccel = 1.50f;
constexpr float kPlanAccel = 2.00f;
constexpr float kPlanBrake = 2.50f;

constexpr float kPidSpeed = 0.25f;
constexpr float kPidGainSpeedLow = 0.25f;
constexpr float kPidGainSpeedHigh = 0.50f;
constexpr float kPidKpLow = 80.0f;
constexpr float kPidKpHigh = 160.0f;
constexpr float kPidKdLow = 8.0f;
constexpr float kPidKdHigh = 14.0f;
constexpr float kPidSlowdownLow = 0.5f;
constexpr float kPidSlowdownHigh = 1.0f;
constexpr float kPidOffsetSlowdownHigh = 80.0f;
constexpr float kKp = 240.0f;
constexpr float kKi = 2.0f;
constexpr float kKd = 22.0f;
constexpr float kDoubleIrKp = 160.0f;
constexpr float kDoubleIrKd = 14.0f;
constexpr float kIntegralLimit = 1.0f;
constexpr float kDerivativeTau = 0.03f;

constexpr float kRearRowDistance = 0.040f;
constexpr float kDoubleIrLatAccel = 1.6f;
constexpr float kDoubleIrVMax = 0.40f;
constexpr float kCurveTau = 0.05f;

static_assert(kSensorForward == kRearRowDistance, "lineHeading assumes both sensor rows are the same distance from the axle");

}
