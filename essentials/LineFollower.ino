#include <Arduino.h>
#include "config.h"
#include "hardware.h"
#include "line_controller.h"
#include "maps.h"
#include "planner.h"
#include "track.h"

constexpr uint8_t kSelectedMap = 0;
constexpr Mode kSelectedMode = Mode::FullMapIr;
static_assert(kSelectedMode != Mode::DoubleIr, "Double IR needs the rear sensor row and wheel encoders");

constexpr uint32_t kTelemetryPeriodMs = 100;

static Track g_track;
static RacingPlan g_plan;
static Controller g_controller;
static bool g_running = false;
static uint32_t g_lastMicros = 0;
static uint32_t g_lastTelemetry = 0;

static void startRun() {
  controllerBegin(&g_controller, kSelectedMode, &g_track, &g_plan, defaultTuning());
  g_running = true;
  g_lastMicros = micros();
  g_lastTelemetry = millis();
  digitalWrite(kPinLed, HIGH);
  Serial.println(F("Go"));
}

static void stopRun(const Command& cmd) {
  stopWheels();
  g_running = false;
  digitalWrite(kPinLed, LOW);
  Serial.println(cmd.status == RunStatus::Finished ? F("Finished") : F("Gave up: line lost"));
}

static void printTelemetry(const Command& cmd) {
  Serial.print(F("s="));
  Serial.print(g_controller.s, 2);
  Serial.print(F(" line="));
  Serial.print(g_controller.seen ? F("yes") : F("no "));
  Serial.print(F(" offset_mm="));
  Serial.print(g_controller.lastOffset * 1000.0f, 1);
  Serial.print(F(" L="));
  Serial.print(cmd.left, 2);
  Serial.print(F(" R="));
  Serial.println(cmd.right, 2);
}

void setup() {
  Serial.begin(115200);
  hardwareInit();

  const MapInfo& map = mapInfo(kSelectedMap);
  trackInit(&g_track, map.segments, map.count);
  if (!planBuild(&g_plan, &g_track)) {
    Serial.println(F("Map is too long for the planner buffer"));
    while (true) {
      digitalWrite(kPinLed, !digitalRead(kPinLed));
      delay(100);
    }
  }
  controllerBegin(&g_controller, kSelectedMode, &g_track, &g_plan, defaultTuning());

  Serial.print(F("Map: "));
  Serial.println(map.name);
  Serial.print(F("Mode: "));
  Serial.println(modeName(kSelectedMode));
  Serial.print(F("Planned lap: "));
  Serial.print(g_plan.lapTime, 2);
  Serial.println(F(" s"));

  Serial.println(F("Calibrating: sensor bar across the line, spinning for 2 s"));
  calibrateSensors();
  Serial.println(F("Ready. Press the button to start."));
}

void loop() {
  if (!g_running) {
    if (buttonPressed()) startRun();
    return;
  }

  const uint32_t now = micros();
  const uint32_t elapsed = now - g_lastMicros;
  if (elapsed < (uint32_t)(config::kControlPeriod * 1000000.0f)) return;
  const float dt = elapsed * 1e-6f;
  g_lastMicros = now;

  float sensors[config::kSensorCount];
  readSensors(sensors);
  const Command cmd = controllerUpdate(&g_controller, sensors, dt);

  if (cmd.status != RunStatus::Driving) {
    stopRun(cmd);
    return;
  }
  driveWheels(cmd.left, cmd.right);

  if (millis() - g_lastTelemetry >= kTelemetryPeriodMs) {
    g_lastTelemetry = millis();
    printTelemetry(cmd);
  }
}
