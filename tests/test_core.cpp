#include <math.h>
#include <stdio.h>
#include "line_controller.h"
#include "maps.h"
#include "mathlite.h"
#include "planner.h"
#include "world.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failures;                                                          \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
    }                                                                        \
  } while (0)

static World g_world;

static void testMathLite() {
  float worst = 0.0f;
  for (float x = -10.0f; x <= 10.0f; x += 0.01f) {
    const float e1 = fabsf(mlite::sin(x) - sinf(x));
    const float e2 = fabsf(mlite::cos(x) - cosf(x));
    if (e1 > worst) worst = e1;
    if (e2 > worst) worst = e2;
  }
  printf("  sin/cos worst error on [-10,10]: %.6f\n", worst);
  CHECK(worst < 5e-4f);
  CHECK(fabsf(mlite::sqrt(2.0f) - sqrtf(2.0f)) < 1e-5f);
}

static void testSensorEstimate() {
  const float centred[5] = {0, 0, 1, 0, 0};
  LineEstimate e = estimateLine(centred);
  CHECK(e.seen);
  CHECK(fabsf(e.offset) < 1e-6f);

  const float leftSide[5] = {1, 1, 0, 0, 0};
  e = estimateLine(leftSide);
  CHECK(e.seen && e.offset > 0.0f);

  const float rightSide[5] = {0, 0, 0, 1, 1};
  e = estimateLine(rightSide);
  CHECK(e.seen && e.offset < 0.0f);

  const float none[5] = {0, 0, 0, 0, 0};
  e = estimateLine(none);
  CHECK(!e.seen);
}

static void testTrackGeometry() {
  for (uint8_t i = 0; i < kMapCount; ++i) {
    Track t;
    const MapInfo& m = mapInfo(i);
    trackInit(&t, m.segments, m.count);
    RacingPlan plan;
    CHECK(planBuild(&plan, &t));
    printf("  %-16s length %.2f m, planned lap %.2f s\n", m.name, t.length, plan.lapTime);
    CHECK(t.length > 1.0f && t.length < 8.0f);
  }
  const Segment left = LeftArc(90, 0.3f);
  const Segment right = RightArc(90, 0.3f);
  CHECK(left.curvature > 0.0f && right.curvature < 0.0f);
  CHECK(fabsf(left.length - 0.3f * 1.5708f) < 1e-3f);
}

static void testPlanLimits() {
  for (uint8_t i = 0; i < kMapCount; ++i) {
    Track t;
    const MapInfo& m = mapInfo(i);
    trackInit(&t, m.segments, m.count);
    RacingPlan plan;
    CHECK(planBuild(&plan, &t));
    const float ds = plan.spacing;
    const float tol = 1e-3f;
    const float L = t.length;
    CHECK(fabsf(planSpeedAt(&plan, 0.0f) - config::kLaunchSpeed) < tol);
    CHECK(fabsf(planSpeedAt(&plan, L) - config::kLaunchSpeed) < tol);
    for (float s0 = 0.0f; s0 + ds <= L; s0 += ds) {
      const float s1 = s0 + ds;
      const float kMax = mlite::fmax(fabsf(trackCurvatureAt(&t, s0)), fabsf(trackCurvatureAt(&t, s1)));
      const float vMid = planSpeedAt(&plan, s0 + 0.5f * ds);
      if (kMax > 1e-3f) CHECK(vMid * vMid * kMax <= config::kPlanLatAccel + 1e-3f);
      const float v0 = planSpeedAt(&plan, s0);
      const float v1 = planSpeedAt(&plan, s1);
      CHECK(v1 * v1 <= v0 * v0 + 2.0f * config::kPlanAccel * ds + 1e-3f);
      CHECK(v0 * v0 <= v1 * v1 + 2.0f * config::kPlanBrake * ds + 1e-3f);
      if (g_failures > 20) return;
    }
  }
  static const Segment straight[] = {Straight(1.0f)};
  Track t;
  trackInit(&t, straight, 1);
  RacingPlan plan;
  CHECK(planBuild(&plan, &t));
  CHECK(fabsf(planSpeedAt(&plan, 0.5f) - config::kPlanVMax) < 1e-3f);
}

static void testControllers() {
  static const Segment straight[] = {Straight(2.0f)};
  Track t;
  trackInit(&t, straight, 1);
  RacingPlan plan;
  planBuild(&plan, &t);

  const float centred[5] = {0, 0, 1, 0, 0};
  const float allWhite[5] = {0, 0, 0, 0, 0};
  Tuning tuning = defaultTuning();
  Command c = {0.0f, 0.0f, RunStatus::Driving};

  Controller blind;
  controllerBegin(&blind, Mode::FullMapBlind, &t, &plan, tuning);
  c = controllerUpdate(&blind, centred, config::kControlPeriod);
  CHECK(c.status == RunStatus::Driving);
  CHECK(fabsf(c.left - c.right) < 1e-6f);
  CHECK(fabsf(c.left - config::kLaunchSpeed) < 1e-3f);

  for (int i = 0; i < 200; ++i) c = controllerUpdate(&blind, allWhite, config::kControlPeriod);
  CHECK(c.status == RunStatus::Driving);

  Controller ir;
  controllerBegin(&ir, Mode::FullMapIr, &t, &plan, tuning);
  for (int i = 0; i < 1000 && c.status == RunStatus::Driving; ++i) {
    c = controllerUpdate(&ir, allWhite, config::kControlPeriod);
  }
  CHECK(c.status == RunStatus::GaveUp);

  const float leftOff[5] = {1, 1, 0, 0, 0};
  Controller pid;
  controllerBegin(&pid, Mode::PidOnly, &t, &plan, tuning);
  c = controllerUpdate(&pid, leftOff, config::kControlPeriod);
  CHECK(c.right > c.left);

  Controller finisher;
  controllerBegin(&finisher, Mode::FullMapBlind, &t, &plan, tuning);
  c.status = RunStatus::Driving;
  for (int i = 0; i < 100000 && c.status == RunStatus::Driving; ++i) {
    c = controllerUpdate(&finisher, centred, config::kControlPeriod);
  }
  CHECK(c.status == RunStatus::Finished);
}

struct SimResult {
  bool finished;
  float lap;
};

static SimResult runSim(uint8_t map, Mode mode, const PlantParams& plant, uint32_t seed) {
  worldStart(&g_world, map, mode, config::kPidSpeed, plant, seed);
  worldRunToEnd(&g_world);
  SimResult r;
  r.finished = g_world.outcome == Outcome::Finished;
  r.lap = r.finished ? g_world.lapTime : -1.0f;
  return r;
}

static void testSimulation() {
  for (uint8_t m = 0; m < kMapCount; ++m) {
    const SimResult pidReal = runSim(m, Mode::PidOnly, kRealRobot, 0x2545F491u);
    const SimResult blindReal = runSim(m, Mode::FullMapBlind, kRealRobot, 0x2545F491u);
    const SimResult irReal = runSim(m, Mode::FullMapIr, kRealRobot, 0x2545F491u);
    const SimResult blindIdeal = runSim(m, Mode::FullMapBlind, kIdealRobot, 0x2545F491u);
    const SimResult irIdeal = runSim(m, Mode::FullMapIr, kIdealRobot, 0x2545F491u);
    const SimResult pidIdeal = runSim(m, Mode::PidOnly, kIdealRobot, 0x2545F491u);
    printf("  %-16s PID %6.2fs  blind %-6s IR %6.2fs | ideal: PID %6.2fs blind %6.2fs IR %6.2fs\n",
           mapInfo(m).name, pidReal.lap, blindReal.finished ? "done" : "fails", irReal.lap,
           pidIdeal.lap, blindIdeal.lap, irIdeal.lap);

    CHECK(irReal.finished);
    for (uint32_t seed = 1; seed <= 8; ++seed) {
      CHECK(runSim(m, Mode::FullMapIr, kRealRobot, seed * 7919u).finished);
    }
    CHECK(blindIdeal.finished);
    CHECK(irIdeal.finished);
    CHECK(pidReal.finished);
    if (pidReal.finished) CHECK(irReal.lap < pidReal.lap);
    CHECK(runSim(m, Mode::FullMapIr, kRealRobot, 0x2545F491u).lap == irReal.lap);
  }
}

static float slideOnArc(float radius, float speed) {
  worldStart(&g_world, 0, Mode::FullMapBlind, config::kPidSpeed, kRealRobot, 1u);
  const float om = speed / radius;
  const float left = speed - om * 0.5f * config::kWheelBase;
  const float right = speed + om * 0.5f * config::kWheelBase;
  float worst = 0.0f;
  for (int i = 0; i < 1000; ++i) {
    worldActuate(&g_world, left, right, config::kControlPeriod);
    if (i > 500) worst = fmaxf(worst, fabsf(g_world.vy));
  }
  return worst;
}

static void testGripPhysics() {
  const float tight = slideOnArc(0.10f, 0.5f);
  const float wide = slideOnArc(0.50f, 0.5f);
  printf("  sideways slide at 0.5 m/s: %.1f mm/s on 10 cm radius, %.1f mm/s on 50 cm radius\n",
         1000.0f * tight, 1000.0f * wide);
  CHECK(tight > 3.0f * wide);
}

static void testDoubleIr() {
  const float a = 0.04f;
  const float theta = 0.07f, kappa = 6.0f, y0 = 0.0023f;
  const float yF = y0 + theta * a + kappa * a * a * 0.5f;
  const float yR = y0 - theta * a + kappa * a * a * 0.5f;
  CHECK(fabsf(lineHeading(yF, yR, a, a) - theta) < 1e-5f);

  for (uint8_t m = 0; m < kMapCount; ++m) {
    worldStart(&g_world, m, Mode::DoubleIr, config::kPidSpeed, kRealRobot, 0x2545F491u);
    static float rel[4000];
    int n = 0;
    while (g_world.outcome == Outcome::Running) {
      worldAdvance(&g_world, config::kControlPeriod);
      const float kt = worldTrueCurvature(&g_world);
      if (fabsf(kt) > 0.5f && n < 4000) rel[n++] = fabsf(g_world.ctrl.kappa - kt) / fabsf(kt);
    }
    CHECK(g_world.outcome == Outcome::Finished);
    for (int i = 1; i < n; ++i) {
      const float x = rel[i];
      int j = i - 1;
      while (j >= 0 && rel[j] > x) { rel[j + 1] = rel[j]; --j; }
      rel[j + 1] = x;
    }
    if (n > 0) CHECK(rel[n / 2] < 0.25f);
    printf("  %-16s Double IR finished %s, median curvature error %.0f%%\n", mapInfo(m).name,
           g_world.outcome == Outcome::Finished ? "yes" : "no", n ? 100.0f * rel[n / 2] : 0.0f);
  }
}

int main() {
  printf("Grip physics\n");
  testGripPhysics();
  printf("Math\n");
  testMathLite();
  printf("Sensor estimate\n");
  testSensorEstimate();
  printf("Track geometry\n");
  testTrackGeometry();
  printf("Racing plan limits\n");
  testPlanLimits();
  printf("Controllers\n");
  testControllers();
  printf("Simulation\n");
  testSimulation();
  printf("Double IR\n");
  testDoubleIr();

  printf("\n%d checks, %d failed\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
