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
constexpr float kKp = 80.0f;              // rad/s of steering per metre of line offset
constexpr float kKi = 2.0f;
constexpr float kKd = 8.0f;
constexpr float kIntegralLimit = 1.0f;    // rad/s, anti-windup clamp on the I term
constexpr float kDerivativeTau = 0.03f;   // s, smoothing on the D term so sensor noise is not amplified

}  // namespace config
