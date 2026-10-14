// C ABI for the browser page. Every number, name and curve the page shows comes
// from the firmware's own code through these functions.
#include "world.h"

namespace {

World g_world;
float g_centre[sim::kMaxCentre * 5];  // x, y, curvature, planned speed, line present
float g_sensors[2 * config::kSensorCount * 3];  // per sensor: value, x, y. Front row, then rear row.
int g_headlessOutcome = 0;

Mode modeFrom(int m) {
  if (m < 0) m = 0;
  if (m >= (int)kModeCount) m = (int)kModeCount - 1;
  return (Mode)m;
}

}  // namespace

extern "C" {

__attribute__((export_name("map_count"))) int map_count() { return kMapCount; }
__attribute__((export_name("map_name"))) const char* map_name(int i) { return mapInfo((uint8_t)i).name; }
__attribute__((export_name("map_summary"))) const char* map_summary(int i) { return mapInfo((uint8_t)i).summary; }
__attribute__((export_name("mode_count"))) int mode_count() { return kModeCount; }
__attribute__((export_name("mode_name"))) const char* mode_name(int i) { return modeName(modeFrom(i)); }
__attribute__((export_name("mode_summary"))) const char* mode_summary(int i) { return modeSummary(modeFrom(i)); }
__attribute__((export_name("default_pid_speed"))) float default_pid_speed() { return config::kPidSpeed; }
__attribute__((export_name("plan_top_speed"))) float plan_top_speed() { return config::kPlanVMax; }

// Robot and sensor geometry, so the drawing matches the firmware's numbers.
__attribute__((export_name("geometry"))) float geometry(int which) {
  switch (which) {
    case 0: return config::kWheelBase;
    case 1: return config::kSensorForward;
    case 2: return config::kSensorSpacing;
    case 3: return config::kLineWidth;
    case 4: return (float)config::kSensorCount;
    case 5: return sim::kDerailOffset;
    case 6: return config::kRearRowDistance;
    case 7: return config::kWheelRadius;
    case 8: return config::kWheelWidth;
    case 9: return config::kBodyLength;
    case 10: return config::kBodyHeight;
    case 11: return config::kBodyClearance;
  }
  return 0.0f;
}

__attribute__((export_name("outcome_name"))) const char* outcome_name(int i) { return outcomeName((Outcome)i); }
__attribute__((export_name("outcome_short"))) const char* outcome_short(int i) { return outcomeShort((Outcome)i); }

// The physical robot the simulator uses, field by field (see PlantParams in sim/world.h).
__attribute__((export_name("plant_value"))) float plant_value(int which) {
  switch (which) {
    case 0: return kRealRobot.mass;
    case 1: return kRealRobot.friction;
    case 2: return kRealRobot.rollingResistance;
    case 3: return kRealRobot.stallForce;
    case 4: return kRealRobot.freeSpeed;
    case 5: return kRealRobot.cornerStiffness;
    case 6: return kRealRobot.leftGain;
    case 7: return kRealRobot.rightGain;
    case 8: return kRealRobot.sensorNoise;
    case 9: return kRealRobot.startX;
    case 10: return kRealRobot.startY;
    case 11: return kRealRobot.startHeading;
    case 12: return kRealRobot.yawInertia;
  }
  return 0.0f;
}

// Physical state of the robot in the run, for the page's readouts.
__attribute__((export_name("run_wheel_actual_left"))) float run_wheel_actual_left() { return g_world.actualWheelLeft(); }
__attribute__((export_name("run_wheel_actual_right"))) float run_wheel_actual_right() { return g_world.actualWheelRight(); }
__attribute__((export_name("run_slip_left"))) float run_slip_left() { return g_world.slipLeft(); }
__attribute__((export_name("run_slip_right"))) float run_slip_right() { return g_world.slipRight(); }
__attribute__((export_name("run_body_speed"))) float run_body_speed() { return g_world.bodySpeed(); }
__attribute__((export_name("run_slide_speed"))) float run_slide_speed() { return g_world.slideSpeed(); }

__attribute__((export_name("run_start"))) int run_start(int map, int mode, float pidSpeed) {
  return g_world.start((uint8_t)map, modeFrom(mode), pidSpeed) ? 1 : 0;
}
__attribute__((export_name("run_advance"))) void run_advance(float seconds) { g_world.advance(seconds); }
__attribute__((export_name("run_outcome"))) int run_outcome() { return (int)g_world.outcome(); }
__attribute__((export_name("run_time"))) float run_time() { return g_world.time(); }
__attribute__((export_name("run_lap_time"))) float run_lap_time() { return g_world.lapTime(); }
__attribute__((export_name("run_progress"))) float run_progress() { return g_world.progress(); }
__attribute__((export_name("run_true_offset"))) float run_true_offset() { return g_world.trueOffset(); }
__attribute__((export_name("run_estimated_offset"))) float run_estimated_offset() { return g_world.estimatedOffset(); }
__attribute__((export_name("run_estimated_progress"))) float run_estimated_progress() { return g_world.estimatedProgress(); }
__attribute__((export_name("run_line_seen"))) int run_line_seen() { return g_world.lineSeen() ? 1 : 0; }
__attribute__((export_name("run_wheel_left"))) float run_wheel_left() { return g_world.wheelLeft(); }
__attribute__((export_name("run_wheel_right"))) float run_wheel_right() { return g_world.wheelRight(); }
__attribute__((export_name("run_pose_x"))) float run_pose_x() { return g_world.poseX(); }
__attribute__((export_name("run_pose_y"))) float run_pose_y() { return g_world.poseY(); }
__attribute__((export_name("run_pose_heading"))) float run_pose_heading() { return g_world.poseHeading(); }
__attribute__((export_name("run_track_length"))) float run_track_length() { return g_world.trackLength(); }
__attribute__((export_name("run_plan_lap"))) float run_plan_lap() { return g_world.planLapTime(); }

// Centre line of the loaded map: five floats per point.
__attribute__((export_name("run_centre_count"))) int run_centre_count() { return g_world.centreCount(); }
__attribute__((export_name("run_centre"))) float* run_centre() {
  const CentrePoint* pts = g_world.centre();
  for (uint16_t i = 0; i < g_world.centreCount(); ++i) {
    g_centre[i * 5 + 0] = pts[i].x;
    g_centre[i * 5 + 1] = pts[i].y;
    g_centre[i * 5 + 2] = pts[i].curvature;
    g_centre[i * 5 + 3] = g_world.planSpeedAt(pts[i].s);
    g_centre[i * 5 + 4] = pts[i].line ? 1.0f : 0.0f;
  }
  return g_centre;
}

// Sensor readings for the current instant: value (0 white .. 1 black), x, y per sensor.
__attribute__((export_name("run_sensors"))) float* run_sensors() {
  for (uint8_t i = 0; i < 2 * config::kSensorCount; ++i) {
    g_sensors[i * 3 + 0] = g_world.sensorValue(i);
    g_sensors[i * 3 + 1] = g_world.sensorX(i);
    g_sensors[i * 3 + 2] = g_world.sensorY(i);
  }
  return g_sensors;
}

// Double IR: the curvature the robot measured, and the true curvature of the centre line under it (1/m).
__attribute__((export_name("run_estimated_curvature"))) float run_estimated_curvature() { return g_world.estimatedCurvature(); }
__attribute__((export_name("run_true_curvature"))) float run_true_curvature() { return g_world.trueCurvature(); }

__attribute__((export_name("run_trail_count"))) int run_trail_count() { return g_world.trailCount(); }
__attribute__((export_name("run_trail"))) const float* run_trail() { return g_world.trailXY(); }

// Runs one map to its end with no rendering, for the comparison table.
// Returns the lap time in seconds, or -1 if the run did not finish.
__attribute__((export_name("headless_lap"))) float headless_lap(int map, int mode, float pidSpeed) {
  if (!g_world.start((uint8_t)map, modeFrom(mode), pidSpeed)) {
    g_headlessOutcome = (int)Outcome::Stopped;
    return -1.0f;
  }
  g_world.runToEnd();
  g_headlessOutcome = (int)g_world.outcome();
  return g_world.outcome() == Outcome::Finished ? g_world.lapTime() : -1.0f;
}
__attribute__((export_name("headless_outcome"))) int headless_outcome() { return g_headlessOutcome; }

}  // extern "C"
