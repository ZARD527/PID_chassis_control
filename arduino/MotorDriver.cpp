#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool inverted)
    : pwmPin_(pwmPin), in1Pin_(in1Pin), in2Pin_(in2Pin), inverted_(inverted) {}

void MotorDriver::begin() {
  pinMode(pwmPin_, OUTPUT);
  pinMode(in1Pin_, OUTPUT);
  pinMode(in2Pin_, OUTPUT);
  coast();
}

void MotorDriver::setPwm(float signedPwm) {
  if (inverted_) signedPwm = -signedPwm;
  int magnitude = (int)constrain(fabs(signedPwm), 0.0f, 255.0f);
  if (magnitude < 2) {
    coast();
    return;
  }
  digitalWrite(in1Pin_, signedPwm >= 0.0f ? HIGH : LOW);
  digitalWrite(in2Pin_, signedPwm >= 0.0f ? LOW : HIGH);
  analogWrite(pwmPin_, magnitude);
}

void MotorDriver::brake() {
  digitalWrite(in1Pin_, HIGH);
  digitalWrite(in2Pin_, HIGH);
  analogWrite(pwmPin_, 255);
}

void MotorDriver::coast() {
  digitalWrite(in1Pin_, LOW);
  digitalWrite(in2Pin_, LOW);
  analogWrite(pwmPin_, 0);
}
