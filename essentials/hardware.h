#pragma once
// Arduino-only hardware layer: reflectance sensors, motor driver and button.
// The control logic in the other files knows nothing about these pins.
#include <Arduino.h>
#include "config.h"

// ---- Wiring (Arduino Mega 2560 with a TB6612FNG motor driver) --------------
constexpr uint8_t kSensorPins[config::kSensorCount] = {A0, A1, A2, A3, A4};  // far left .. far right
constexpr uint8_t kPinIrEmitter = 27;   // drives the IR LEDs on the sensor bar
constexpr uint8_t kPinButton = 2;       // momentary switch to GND (uses INPUT_PULLUP)
constexpr uint8_t kPinLed = LED_BUILTIN;
constexpr uint8_t kPinLeftIn1 = 22;
constexpr uint8_t kPinLeftIn2 = 23;
constexpr uint8_t kPinLeftPwm = 4;
constexpr uint8_t kPinRightIn1 = 24;
constexpr uint8_t kPinRightIn2 = 25;
constexpr uint8_t kPinRightPwm = 5;
constexpr uint8_t kPinStandby = 26;

void hardwareInit();

// Fills out[] with 0 (white) .. 1 (black) for each sensor, using the calibration.
// Assumes a reflectance sensor that reads LOW over black tape.
void readSensors(float out[config::kSensorCount]);

// Spins on the spot for two seconds with the sensor bar across the line, learning
// the black and white levels of each sensor.
void calibrateSensors();

// Wheel speeds in m/s, positive forward. Converted to PWM with config::kWheelMaxSpeed.
void driveWheels(float leftMps, float rightMps);
void stopWheels();

// True once per press, with a debounce.
bool buttonPressed();
