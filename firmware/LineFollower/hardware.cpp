#include "hardware.h"
#include "mathlite.h"

namespace {

uint16_t g_rawBlack[config::kSensorCount];
uint16_t g_rawWhite[config::kSensorCount];

void driveMotor(uint8_t in1, uint8_t in2, uint8_t pwm, float mps) {
  int duty = (int)(mlite::fabs(mps) / config::kWheelMaxSpeed * 255.0f);
  if (duty > 255) duty = 255;
  digitalWrite(in1, mps >= 0.0f ? HIGH : LOW);
  digitalWrite(in2, mps >= 0.0f ? LOW : HIGH);
  analogWrite(pwm, duty);
}

}

void hardwareInit() {
  pinMode(kPinIrEmitter, OUTPUT);
  digitalWrite(kPinIrEmitter, HIGH);
  pinMode(kPinButton, INPUT_PULLUP);
  pinMode(kPinLed, OUTPUT);
  pinMode(kPinLeftIn1, OUTPUT);
  pinMode(kPinLeftIn2, OUTPUT);
  pinMode(kPinLeftPwm, OUTPUT);
  pinMode(kPinRightIn1, OUTPUT);
  pinMode(kPinRightIn2, OUTPUT);
  pinMode(kPinRightPwm, OUTPUT);
  pinMode(kPinStandby, OUTPUT);
  digitalWrite(kPinStandby, HIGH);

  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    g_rawBlack[i] = 0;
    g_rawWhite[i] = 1023;
  }
  stopWheels();
}

void readSensors(float out[config::kSensorCount]) {
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    const float raw = (float)analogRead(kSensorPins[i]);
    const float span = (float)(g_rawWhite[i] - g_rawBlack[i]);
    out[i] = mlite::clamp(((float)g_rawWhite[i] - raw) / (span > 20.0f ? span : 20.0f), 0.0f, 1.0f);
  }
}

void calibrateSensors() {
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    g_rawBlack[i] = 1023;
    g_rawWhite[i] = 0;
  }
  driveWheels(-0.25f, 0.25f);
  const uint32_t start = millis();
  while (millis() - start < 2000) {
    for (uint8_t i = 0; i < config::kSensorCount; ++i) {
      const uint16_t raw = analogRead(kSensorPins[i]);
      if (raw < g_rawBlack[i]) g_rawBlack[i] = raw;
      if (raw > g_rawWhite[i]) g_rawWhite[i] = raw;
    }
    delay(2);
  }
  stopWheels();
}

void driveWheels(float leftMps, float rightMps) {
  driveMotor(kPinLeftIn1, kPinLeftIn2, kPinLeftPwm, leftMps);
  driveMotor(kPinRightIn1, kPinRightIn2, kPinRightPwm, rightMps);
}

void stopWheels() {
  driveWheels(0.0f, 0.0f);
}

bool buttonPressed() {
  static uint32_t lastPress = 0;
  if (digitalRead(kPinButton) != LOW) return false;
  const uint32_t now = millis();
  if (now - lastPress < 400) return false;
  lastPress = now;
  return true;
}
