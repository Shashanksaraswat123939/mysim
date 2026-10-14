// Host tests for the firmware core and the simulator. Build and run with scripts/build.sh.
// Math is checked against <math.h> here only; the firmware itself never includes it.
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

// World is large, so keep it out of the stack.
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

  const float leftSide[5] = {1, 1, 0, 0, 0};  // sensor 0 is the far left
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
    t.init(m.segments, m.count);
    RacingPlan plan;
    CHECK(plan.build(t));
    printf("  %-16s length %.2f m, planned lap %.2f s\n", m.name, t.length(), plan.idealLapTime());
    CHECK(t.length() > 1.0f && t.length() < 8.0f);
  }
  // A left arc must curve left, a right arc right.
  const Segment left = LeftArc(90, 0.3f);
  const Segment right = RightArc(90, 0.3f);
  CHECK(left.curvature > 0.0f && right.curvature < 0.0f);
  CHECK(fabsf(left.length - 0.3f * 1.5708f) < 1e-3f);
}

static void testPlanLimits() {
  for (uint8_t i = 0; i < kMapCount; ++i) {
    Track t;
    const MapInfo& m = mapInfo(i);
    t.init(m.segments, m.count);
    RacingPlan plan;
    CHECK(plan.build(t));
    const float ds = plan.spacing();
    const float tol = 1e-3f;
    const float L = t.length();
    CHECK(fabsf(plan.speedAt(0.0f) - config::kLaunchSpeed) < tol);
    CHECK(fabsf(plan.speedAt(L) - config::kLaunchSpeed) < tol);
    // Check the grip limit on each sample interval (and the midpoints inside it),
    // using the worse curvature of the two ends, which is what the planner uses.
    for (float s0 = 0.0f; s0 + ds <= L; s0 += ds) {
      const float s1 = s0 + ds;
      const float kMax = mlite::fmax(fabsf(t.curvatureAt(s0)), fabsf(t.curvatureAt(s1)));
      const float vMid = plan.speedAt(s0 + 0.5f * ds);
      if (kMax > 1e-3f) CHECK(vMid * vMid * kMax <= config::kPlanLatAccel + 1e-3f);
      // Traction and braking between neighbouring samples.
      const float v0 = plan.speedAt(s0);
      const float v1 = plan.speedAt(s1);
      CHECK(v1 * v1 <= v0 * v0 + 2.0f * config::kPlanAccel * ds + 1e-3f);
      CHECK(v0 * v0 <= v1 * v1 + 2.0f * config::kPlanBrake * ds + 1e-3f);
      if (g_failures > 20) return;
    }
  }
  // A long straight must reach the top speed in the middle.
  static const Segment straight[] = {Straight(1.0f)};
  Track t;
  t.init(straight, 1);
  RacingPlan plan;
  CHECK(plan.build(t));
  CHECK(fabsf(plan.speedAt(0.5f) - config::kPlanVMax) < 1e-3f);
}

static void testControllers() {
  static const Segment straight[] = {Straight(2.0f)};
  Track t;
  t.init(straight, 1);
  RacingPlan plan;
  plan.build(t);

  const float centred[5] = {0, 0, 1, 0, 0};
  const float allWhite[5] = {0, 0, 0, 0, 0};
  Tuning tuning = defaultTuning();
  Command c = {0.0f, 0.0f, RunStatus::Driving};

  // Blind on a straight, perfectly centred: both wheels run at the planned speed.
  LineController blind;
  blind.begin(Mode::FullMapBlind, &t, &plan, tuning);
  c = blind.update(centred, config::kControlPeriod);
  CHECK(c.status == RunStatus::Driving);
  CHECK(fabsf(c.left - c.right) < 1e-6f);
  CHECK(fabsf(c.left - config::kLaunchSpeed) < 1e-3f);

  // Blind ignores the sensors entirely: a white field does not stop it.
  for (int i = 0; i < 200; ++i) c = blind.update(allWhite, config::kControlPeriod);
  CHECK(c.status == RunStatus::Driving);

  // With the sensors on, losing the line for longer than the timeout gives up.
  LineController ir;
  ir.begin(Mode::FullMapIr, &t, &plan, tuning);
  for (int i = 0; i < 1000 && c.status == RunStatus::Driving; ++i) {
    c = ir.update(allWhite, config::kControlPeriod);
  }
  CHECK(c.status == RunStatus::GaveUp);

  // Normal PID: a line that is off to the left turns the robot left (right wheel faster).
  const float leftOff[5] = {1, 1, 0, 0, 0};  // sensors 0 and 1 are on the left
  LineController pid;
  pid.begin(Mode::PidOnly, &t, &plan, tuning);
  c = pid.update(leftOff, config::kControlPeriod);
  CHECK(c.right > c.left);

  // Finishing: the full-map controllers report Finished at the end of the map.
  LineController finisher;
  finisher.begin(Mode::FullMapBlind, &t, &plan, tuning);
  c.status = RunStatus::Driving;
  for (int i = 0; i < 100000 && c.status == RunStatus::Driving; ++i) {
    c = finisher.update(centred, config::kControlPeriod);
  }
  CHECK(c.status == RunStatus::Finished);
}

struct SimResult {
  bool finished;
  float lap;
};

static SimResult runSim(uint8_t map, Mode mode, const PlantParams& plant, uint32_t seed) {
  g_world.start(map, mode, config::kPidSpeed, plant, seed);
  g_world.runToEnd();
  SimResult r;
  r.finished = g_world.outcome() == Outcome::Finished;
  r.lap = r.finished ? g_world.lapTime() : -1.0f;
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

    // The mixed mode completes every map with the real robot: the default run the
    // page starts with, and a set of other noise seeds.
    CHECK(irReal.finished);
    for (uint32_t seed = 1; seed <= 8; ++seed) {
      CHECK(runSim(m, Mode::FullMapIr, kRealRobot, seed * 7919u).finished);
    }
    // With perfect hardware the open-loop plan alone completes every map.
    CHECK(blindIdeal.finished);
    CHECK(irIdeal.finished);
    // Normal PID, tuned with its speed governor, completes every map at the default speed.
    CHECK(pidReal.finished);
    // The racing plan is faster than the sensor-only controller wherever both finish.
    if (pidReal.finished) CHECK(irReal.lap < pidReal.lap);
    // Determinism: the same seed gives the same result.
    CHECK(runSim(m, Mode::FullMapIr, kRealRobot, 0x2545F491u).lap == irReal.lap);
  }
}

int main() {
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

  printf("\n%d checks, %d failed\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
