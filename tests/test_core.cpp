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
    CHECK(plan.build(t, config::kPlanVMax));
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
    CHECK(plan.build(t, config::kPlanVMax));
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
  CHECK(plan.build(t, config::kPlanVMax));
  CHECK(fabsf(plan.speedAt(0.5f) - config::kPlanVMax) < 1e-3f);
}

static void testControllers() {
  static const Segment straight[] = {Straight(2.0f)};
  Track t;
  t.init(straight, 1);
  RacingPlan plan;
  plan.build(t, config::kPlanVMax);

  const float centred[5] = {0, 0, 1, 0, 0};
  const float allWhite[5] = {0, 0, 0, 0, 0};
  Tuning tuning = defaultTuning(Mode::FullMapIr);
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
  g_world.start(map, mode, modeSpeedDefault(mode), plant, seed);
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

// The grip physics that Double IR's speed rule relies on: at a fixed speed, the sideways slide is larger
// on a tighter arc. Measured open loop, with the wheels commanded for the arc.
static float slideOnArc(float radius, float speed) {
  g_world.start(0, Mode::FullMapBlind, modeSpeedDefault(Mode::FullMapBlind), kRealRobot, 1u);
  const float om = speed / radius;
  const float left = speed - om * 0.5f * config::kWheelBase;
  const float right = speed + om * 0.5f * config::kWheelBase;
  float worst = 0.0f;
  for (int i = 0; i < 1000; ++i) {  // 2 s, so the robot settles on the arc
    g_world.actuate(left, right, config::kControlPeriod);
    if (i > 500) worst = fmaxf(worst, fabsf(g_world.slideSpeed()));
  }
  return worst;
}

static void testGripPhysics() {
  const float tight = slideOnArc(0.10f, 0.5f);
  const float wide = slideOnArc(0.50f, 0.5f);
  printf("  sideways slide at 0.5 m/s: %.1f mm/s on 10 cm radius, %.1f mm/s on 50 cm radius\n",
         1000.0f * tight, 1000.0f * wide);
  CHECK(tight > 3.0f * wide);  // measured ratio about 5
}

// Double IR: the heading from two rows, and the curvature the robot measures on a real run.
static void testDoubleIr() {
  // With equal row distances the curvature cancels, so the heading is exact for any kappa and y0.
  const float a = 0.04f;
  const float theta = 0.07f, kappa = 6.0f, y0 = 0.0023f;
  const float yF = y0 + theta * a + kappa * a * a * 0.5f;
  const float yR = y0 - theta * a + kappa * a * a * 0.5f;
  CHECK(fabsf(lineHeading(yF, yR, a, a) - theta) < 1e-5f);

  // The Double IR mode finishes every map on the default run, and the curvature it measures on the
  // curved parts stays close to the truth (median error under 25%; measured 6 to 16%).
  for (uint8_t m = 0; m < kMapCount; ++m) {
    g_world.start(m, Mode::DoubleIr, modeSpeedDefault(Mode::DoubleIr), kRealRobot, 0x2545F491u);
    static float rel[4000];
    int n = 0;
    while (g_world.outcome() == Outcome::Running) {
      g_world.advance(config::kControlPeriod);
      const float kt = g_world.trueCurvature();
      if (fabsf(kt) > 0.5f && n < 4000) rel[n++] = fabsf(g_world.estimatedCurvature() - kt) / fabsf(kt);
    }
    CHECK(g_world.outcome() == Outcome::Finished);
    for (int i = 1; i < n; ++i) {  // insertion sort: small, and keeps the test free of library calls
      const float x = rel[i];
      int j = i - 1;
      while (j >= 0 && rel[j] > x) { rel[j + 1] = rel[j]; --j; }
      rel[j + 1] = x;
    }
    if (n > 0) CHECK(rel[n / 2] < 0.25f);
    printf("  %-16s Double IR finished %s, median curvature error %.0f%%\n", mapInfo(m).name,
           g_world.outcome() == Outcome::Finished ? "yes" : "no", n ? 100.0f * rel[n / 2] : 0.0f);
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
