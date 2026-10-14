// Hardware-in-the-loop check: runs the Arduino sketch's real setup() and loop(),
// unchanged, against the simulated robot. The mock pins read the robot's sensors and
// the motor pins drive its wheels. Pass/fail is judged from the robot's ground truth.
#include <stdio.h>
#include "Arduino.h"
#include "config.h"
#include "hardware.h"
#include "world.h"

SerialClass Serial;

namespace {

World g_world;
uint32_t g_timeUs = 0;
uint8_t g_level[64];
int g_pwm[64];
uint32_t g_buttonAt = 0xFFFFFFFFu;  // when the button is held down, in microseconds

constexpr uint32_t kStepUs = 100;  // physics step, matching the 500 Hz loop's resolution
constexpr uint32_t kBudgetUs = 60000000u;

// Wheel speed the motor driver is currently producing, from its direction and PWM pins.
float wheelMps(uint8_t in1, uint8_t in2, uint8_t pwm) {
  int dir = 0;
  if (g_level[in1] == HIGH && g_level[in2] == LOW) dir = 1;
  if (g_level[in1] == LOW && g_level[in2] == HIGH) dir = -1;
  return dir * (g_pwm[pwm] / 255.0f) * config::kWheelMaxSpeed;
}

void stepRobot() {
  g_timeUs += kStepUs;
  const float dt = kStepUs * 1e-6f;
  g_world.sense();
  const float left = wheelMps(kPinLeftIn1, kPinLeftIn2, kPinLeftPwm);
  const float right = wheelMps(kPinRightIn1, kPinRightIn2, kPinRightPwm);
  g_world.actuate(left, right, dt);
  g_world.judge(RunStatus::Driving, dt);
}

}  // namespace

// ---- Arduino API -----------------------------------------------------------
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value) { g_level[pin] = value; }
int digitalRead(uint8_t pin) {
  if (pin == kPinButton) {
    const bool held = g_timeUs >= g_buttonAt && g_timeUs < g_buttonAt + 60000u;
    return held ? LOW : HIGH;
  }
  return g_level[pin];
}
int analogRead(uint8_t pin) {
  const int index = pin - A0;
  // Black tape reads low (about 100), white floor high (about 900).
  return 900 - (int)(800.0f * g_world.sensorValue((uint8_t)index));
}
void analogWrite(uint8_t pin, int value) { g_pwm[pin] = value; }
unsigned long millis() { return g_timeUs / 1000u; }
unsigned long micros() { return g_timeUs; }
void delay(unsigned long ms) {
  for (unsigned long i = 0; i < ms * 10; ++i) stepRobot();
}

int main() {
  // The sensor bar starts straddling the line for calibration, as the instructions say.
  PlantParams calibrationSpot = kRealRobot;
  calibrationSpot.startX = -0.04f;
  calibrationSpot.startY = 0.0f;
  calibrationSpot.startHeading = 0.0f;
  g_world.start(0, Mode::FullMapIr, config::kPidSpeed, calibrationSpot);

  setup();  // calibrates, then waits for the button

  // Pick the robot up and set it down at the start line, a little off-centre.
  g_world.placeRobot(kRealRobot.startX, kRealRobot.startY, kRealRobot.startHeading);
  g_buttonAt = g_timeUs + 500000u;

  while (g_world.outcome() == Outcome::Running && g_timeUs < kBudgetUs) {
    stepRobot();
    loop();
  }

  // Lap time from the button press, not from power-up (calibration takes two seconds).
  const float lap = g_world.outcome() == Outcome::Finished ? (g_timeUs - g_buttonAt) * 1e-6f : -1.0f;
  printf("HIL result: outcome=%d lap=%.2f s plan=%.2f s true-offset=%+.4f m\n",
         (int)g_world.outcome(), lap, g_world.planLapTime(), g_world.trueOffset());
  return g_world.outcome() == Outcome::Finished ? 0 : 1;
}
