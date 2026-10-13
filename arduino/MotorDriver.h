#pragma once

#include <Arduino.h>

class MotorDriver {
 public:
  MotorDriver(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, bool inverted = false);
  void begin();
  void setPwm(float signedPwm);
  void brake();
  void coast();

 private:
  uint8_t pwmPin_;
  uint8_t in1Pin_;
  uint8_t in2Pin_;
  bool inverted_;
};
