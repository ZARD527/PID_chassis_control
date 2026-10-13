#pragma once

class Odometry {
 public:
  Odometry() : x_(0.0f), y_(0.0f), heading_(0.0f) {}
  void reset() { x_ = y_ = heading_ = 0.0f; }
  void update(float leftDistanceM, float rightDistanceM, float wheelBaseM);
  float x() const { return x_; }
  float y() const { return y_; }
  float heading() const { return heading_; }

 private:
  float x_;
  float y_;
  float heading_;
};
