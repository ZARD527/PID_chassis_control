#include <Arduino.h>
#include "Odometry.h"

void Odometry::update(float leftDistanceM, float rightDistanceM, float wheelBaseM) {
  if (wheelBaseM <= 0.0f) return;
  float distance = (leftDistanceM + rightDistanceM) * 0.5f;
  float deltaHeading = (rightDistanceM - leftDistanceM) / wheelBaseM;
  float midpointHeading = heading_ + deltaHeading * 0.5f;
  x_ += distance * cos(midpointHeading);
  y_ += distance * sin(midpointHeading);
  heading_ += deltaHeading;
  if (heading_ > PI) heading_ -= TWO_PI;
  if (heading_ < -PI) heading_ += TWO_PI;
}
