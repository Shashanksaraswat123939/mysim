#pragma once
// Robot geometry and tuning, shared by the Arduino firmware and the simulator.
// Units: metres, seconds, radians, m/s. These are the robot's *beliefs* about
// itself. The simulated "real" robot (sim/world.cpp) is deliberately imperfect.
#include <stdint.h>

namespace config {

// ---- Geometry ---------------------------------------------------------------
constexpr float kWheelBase = 0.090f;      // centre-to-centre of the drive wheels
constexpr uint8_t kSensorCount = 5;       // reflectance sensors in a row
constexpr float kSensorSpacing = 0.012f;  // distance between neighbouring sensors
constexpr float kSensorForward = 0.040f;  // sensor bar distance ahead of the axle
constexpr float kLineWidth = 0.020f;      // width of the black tape
// Body, for the 3D view and the drawings. The simulator's physics uses the wheel radius too.
constexpr float kWheelRadius = 0.020f;    // drive wheel radius
constexpr float kWheelWidth = 0.012f;     // drive wheel width
constexpr float kBodyLength = 0.064f;     // chassis length, centred on the axle
constexpr float kBodyHeight = 0.022f;     // chassis height
constexpr float kBodyClearance = 0.008f;  // gap between the floor and the chassis underside

// ---- Timing and limits -----------------------------------------------------
constexpr float kControlPeriod = 0.002f;  // 500 Hz control loop
constexpr float kWheelMaxSpeed = 1.20f;   // wheel speed that corresponds to full PWM
constexpr float kLaunchSpeed = 0.15f;     // speed the racing plan starts and ends at
constexpr float kLostTimeout = 0.60f;     // seconds without a line before giving up

// ---- Racing plan (Full map modes) ------------------------------------------
constexpr float kPlanVMax = 1.00f;        // m/s top speed on straights
constexpr float kPlanLatAccel = 1.50f;    // m/s^2 sideways grip limit in corners
constexpr float kPlanAccel = 2.00f;       // m/s^2 forward acceleration limit
constexpr float kPlanBrake = 2.50f;       // m/s^2 braking limit

// ---- Controller tuning -------------------------------------------------------
constexpr float kPidSpeed = 0.25f;        // m/s base speed for Normal PID
// Normal PID gains are scheduled with its base speed: faster runs need a stiffer steering
// loop. The values below are the low-speed set (at kPidGainSpeedLow) and the high-speed set
// (at kPidGainSpeedHigh), blended linearly in between and held at the low set below that.
constexpr float kPidGainSpeedLow = 0.25f;   // m/s
constexpr float kPidGainSpeedHigh = 0.50f;  // m/s
constexpr float kPidKpLow = 80.0f;
constexpr float kPidKpHigh = 160.0f;
constexpr float kPidKdLow = 8.0f;
constexpr float kPidKdHigh = 14.0f;
// The governor slows the robot as steering effort (|omega|) and line offset grow:
// v = speed / (1 + slowdown * |omega| + offsetSlowdown * |offset|).
constexpr float kPidSlowdownLow = 0.5f;
constexpr float kPidSlowdownHigh = 1.0f;
constexpr float kPidOffsetSlowdownHigh = 80.0f;  // per metre of offset, at the high-speed set
constexpr float kKp = 160.0f;             // rad/s of steering per metre of line offset (Full map + IR)
constexpr float kKi = 2.0f;
constexpr float kKd = 14.0f;
constexpr float kIntegralLimit = 1.0f;    // rad/s, anti-windup clamp on the I term
constexpr float kDerivativeTau = 0.03f;   // s, smoothing on the D term so sensor noise is not amplified

// ---- Double IR: a second sensor row, behind the axle --------------------------
// Needs a second bar of sensors (see docs/PROJECT.md). The front bar is kSensorForward ahead of the axle.
constexpr float kRearRowDistance = 0.040f;  // m behind the axle for the rear sensor row
// Sideways grip limit the robot is willing to use. From the simulator's grip data: the sideways
// slide reaches 2 cm/s at 1.6 to 2.5 m/s^2 across radii, and the model's theoretical limit is
// mu*g = 5.9 m/s^2. The lowest value of that range is used. Calibrate it on the real robot.
constexpr float kDoubleIrLatAccel = 1.6f;   // m/s^2
// Smoothing on the curvature estimate, seconds. Shorter follows bends sooner but passes more sensor noise.
// Top speed for Double IR. The front row sees a bend only 4 cm before the axle, so the speed cannot be
// set in advance. Measured in the simulator: capped at 0.40 m/s it completes every map (48 of 48 seeded
// runs); at 0.50 m/s it completes 46 of 48, and uncapped (1.0 m/s) it completes 16 of 48.
constexpr float kDoubleIrVMax = 0.40f;
constexpr float kCurveTau = 0.05f;
// Double IR needs wheel encoders: the rows give the line's direction relative to the robot, and the
// encoders give the robot's own turn rate. Without them the curvature cannot be found.

}  // namespace config
