#pragma once
#include <Arduino.h>
#include "config.h"

constexpr uint8_t kSensorPins[config::kSensorCount] = {A0, A1, A2, A3, A4};
constexpr uint8_t kPinIrEmitter = 27;
constexpr uint8_t kPinButton = 2;
constexpr uint8_t kPinLed = LED_BUILTIN;
constexpr uint8_t kPinLeftIn1 = 22;
constexpr uint8_t kPinLeftIn2 = 23;
constexpr uint8_t kPinLeftPwm = 4;
constexpr uint8_t kPinRightIn1 = 24;
constexpr uint8_t kPinRightIn2 = 25;
constexpr uint8_t kPinRightPwm = 5;
constexpr uint8_t kPinStandby = 26;

void hardwareInit();

void readSensors(float out[config::kSensorCount]);

void calibrateSensors();

void driveWheels(float leftMps, float rightMps);
void stopWheels();

bool buttonPressed();
