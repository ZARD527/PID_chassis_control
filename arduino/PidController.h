#pragma once

#include <Arduino.h>

class PidController {
 public:
  PidController(float kp = 1.2f, float ki = 0.35f, float kd = 0.01f)
      : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), previousError_(0.0f) {}

  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
  }

  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
  }

  float update(float error, float dt, float outputMin, float outputMax) {
    if (dt <= 0.0f) return 0.0f;
    integral_ += error * dt;
    if (fabs(ki_) > 0.0001f) {
      float limit = max(fabs(outputMin), fabs(outputMax)) / fabs(ki_);
      integral_ = constrain(integral_, -limit, limit);
    } else {
      integral_ = 0.0f;
    }
    float derivative = (error - previousError_) / dt;
    previousError_ = error;
    return constrain(kp_ * error + ki_ * integral_ + kd_ * derivative, outputMin, outputMax);
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_;
  float previousError_;
};
