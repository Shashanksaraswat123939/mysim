// Runs every map in every control mode through the simulator and writes the results as
// JSON. scripts/build.sh runs this, embeds the JSON in the page and publishes it with the
// site, so the published numbers come from the same C++ as the firmware.
#include <stdio.h>
#include <initializer_list>
#include "world.h"

namespace {
constexpr uint32_t kSeeds = 40;
constexpr uint32_t kDefaultSeed = 0x2545F491u;
World g_world;
}  // namespace

int main() {
  printf("{\n");
  printf("  \"pidSpeed\": %.2f,\n", config::kPidSpeed);
  printf("  \"seeds\": %u,\n", (unsigned)kSeeds);
  printf("  \"maps\": [\n");
  for (uint8_t m = 0; m < kMapCount; ++m) {
    g_world.start(m, Mode::PidOnly, config::kPidSpeed, kRealRobot, kDefaultSeed);
    printf("    {\"name\": \"%s\", \"planLap\": %.2f, \"modes\": [\n", mapInfo(m).name, g_world.planLapTime());
    for (uint8_t md = 0; md < kModeCount; ++md) {
      // Default run: the one the page starts with.
      g_world.start(m, (Mode)md, config::kPidSpeed, kRealRobot, kDefaultSeed);
      g_world.runToEnd();
      const bool defaultDone = g_world.outcome() == Outcome::Finished;
      const float defaultLap = defaultDone ? g_world.lapTime() : -1.0f;
      const char* defaultOutcome = outcomeShort(g_world.outcome());

      // Seeded runs: sensor and motor noise varied, same robot.
      uint32_t finished = 0;
      double lapSum = 0.0;
      for (uint32_t s = 1; s <= kSeeds; ++s) {
        g_world.start(m, (Mode)md, config::kPidSpeed, kRealRobot, s * 7919u);
        g_world.runToEnd();
        if (g_world.outcome() == Outcome::Finished) {
          ++finished;
          lapSum += g_world.lapTime();
        }
      }
      const float meanLap = finished ? (float)(lapSum / finished) : -1.0f;

      printf("      {\"mode\": \"%s\", \"defaultLap\": %.2f, \"defaultOutcome\": \"%s\", "
             "\"seedsFinished\": %u, \"meanLap\": %.2f}%s\n",
             modeName((Mode)md), defaultLap, defaultOutcome, (unsigned)finished, meanLap,
             md + 1 < kModeCount ? "," : "");
    }
    printf("    ]}%s\n", m + 1 < kMapCount ? "," : "");
  }
  printf("  ]\n}\n");
  return 0;
}
