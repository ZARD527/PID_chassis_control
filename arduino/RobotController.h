#pragma once

#include <Arduino.h>
#include "Encoder.h"
#include "MotorDriver.h"
#include "Odometry.h"
#include "PidController.h"

enum DriveMode { DRIVE_STOP, DRIVE_WHEEL, DRIVE_CMDVEL };

struct DriveConfig {
  float ticksPerWheelRevolution = 1320.0f;
  float wheelDiameterM = 0.065f;
  float wheelBaseM = 0.155f;
  float maxRpm = 220.0f;
  unsigned long controlPeriodMs = 10;
  unsigned long reportPeriodMs = 50;
  unsigned long commandTimeoutMs = 600;
};

struct WheelTelemetry {
  long ticks = 0;
  long lastTicks = 0;
  float targetRpm = 0.0f;
  float rpm = 0.0f;
  float pwm = 0.0f;
  float error = 0.0f;
};

class RobotController {
 public:
  RobotController();
  void begin();
  void update(unsigned long now);
  void reportIfDue(unsigned long now, Stream &output);
  void handleCommand(String line, Stream &output);
  void printHelp(Stream &output) const;

 private:
  void setWheelTargets(float leftRpm, float rightRpm, DriveMode mode);
  void setCmdVel(float linearMps, float angularRadps);
  void stop(bool brake);
  void resetOdometry();
  void updateWheel(WheelTelemetry &wheel, Encoder &encoder, MotorDriver &motor,
                   PidController &pid, unsigned long elapsedMs);
  void printTelemetry(unsigned long now, Stream &output) const;
  bool setPid(String arguments, Stream &output);
  bool setConfig(String arguments, Stream &output);
  static String nextToken(String &text);
  static bool parseTwoFloats(String text, float &a, float &b);
  static bool parseThreeFloats(String text, float &a, float &b, float &c);
  const char *modeName() const;

  MotorDriver leftMotor_;
  MotorDriver rightMotor_;
  Encoder leftEncoder_;
  Encoder rightEncoder_;
  PidController leftPid_;
  PidController rightPid_;
  Odometry odometry_;
  DriveConfig config_;
  WheelTelemetry left_;
  WheelTelemetry right_;
  DriveMode mode_;
  bool coastWhenStopped_;
  unsigned long lastControlMs_;
  unsigned long lastReportMs_;
  unsigned long lastCommandMs_;
};
