#pragma once
#include <stdint.h>
#include <stdio.h>

#define F(text) (text)
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LED_BUILTIN 13
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58

class SerialClass {
 public:
  void begin(unsigned long) {}
  void print(const char* s) { printf("%s", s); }
  void println(const char* s) { printf("%s\n", s); }
  void print(float v, int digits) { printf("%.*f", digits, (double)v); }
  void println(float v, int digits) { printf("%.*f\n", digits, (double)v); }
  void print(int v) { printf("%d", v); }
  void println(int v) { printf("%d\n", v); }
};
extern SerialClass Serial;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);

void setup();
void loop();
