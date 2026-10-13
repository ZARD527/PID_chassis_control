#include "RobotController.h"

namespace {
const uint8_t PIN_STBY = 4;
const uint8_t PIN_LEFT_PWM = 9;
const uint8_t PIN_LEFT_IN1 = 7;
const uint8_t PIN_LEFT_IN2 = 8;
const uint8_t PIN_RIGHT_PWM = 10;
const uint8_t PIN_RIGHT_IN1 = 5;
const uint8_t PIN_RIGHT_IN2 = 6;
const uint8_t PIN_LEFT_ENC_A = 2;
const uint8_t PIN_LEFT_ENC_B = 11;
const uint8_t PIN_RIGHT_ENC_A = 3;
const uint8_t PIN_RIGHT_ENC_B = 12;

const bool LEFT_MOTOR_INVERT = false;
const bool RIGHT_MOTOR_INVERT = false;
const bool LEFT_ENCODER_INVERT = false;
const bool RIGHT_ENCODER_INVERT = false;
}

RobotController::RobotController()
    : leftMotor_(PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2, LEFT_MOTOR_INVERT),
      rightMotor_(PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2, RIGHT_MOTOR_INVERT),
      leftEncoder_(PIN_LEFT_ENC_A, PIN_LEFT_ENC_B, LEFT_ENCODER_INVERT),
      rightEncoder_(PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B, RIGHT_ENCODER_INVERT),
      mode_(DRIVE_STOP), coastWhenStopped_(false), lastControlMs_(0), lastReportMs_(0), lastCommandMs_(0) {}

void RobotController::begin() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  leftMotor_.begin();
  rightMotor_.begin();
  leftEncoder_.begin(0);
  rightEncoder_.begin(1);
  stop(true);
  unsigned long now = millis();
  lastControlMs_ = now;
  lastReportMs_ = now;
  lastCommandMs_ = now;
}

void RobotController::update(unsigned long now) {
  if (mode_ != DRIVE_STOP && now - lastCommandMs_ > config_.commandTimeoutMs) stop(true);
  if (now - lastControlMs_ < config_.controlPeriodMs) return;
  unsigned long elapsedMs = now - lastControlMs_;
  lastControlMs_ = now;
  if (mode_ == DRIVE_STOP) {
    left_.ticks = left_.lastTicks = leftEncoder_.readTicks();
    right_.ticks = right_.lastTicks = rightEncoder_.readTicks();
    left_.rpm = right_.rpm = 0.0f;
    if (coastWhenStopped_) {
      leftMotor_.coast();
      rightMotor_.coast();
    } else {
      leftMotor_.brake();
      rightMotor_.brake();
    }
    left_.pwm = right_.pwm = 0.0f;
    return;
  }
  updateWheel(left_, leftEncoder_, leftMotor_, leftPid_, elapsedMs);
  updateWheel(right_, rightEncoder_, rightMotor_, rightPid_, elapsedMs);
  float metersPerTick = PI * config_.wheelDiameterM / config_.ticksPerWheelRevolution;
  odometry_.update((left_.ticks - left_.lastTicks) * metersPerTick,
                   (right_.ticks - right_.lastTicks) * metersPerTick, config_.wheelBaseM);
}

void RobotController::updateWheel(WheelTelemetry &wheel, Encoder &encoder, MotorDriver &motor,
                                  PidController &pid, unsigned long elapsedMs) {
  wheel.lastTicks = wheel.ticks;
  wheel.ticks = encoder.readTicks();
  long deltaTicks = wheel.ticks - wheel.lastTicks;
  wheel.rpm = deltaTicks * 60000.0f / (config_.ticksPerWheelRevolution * elapsedMs);
  wheel.error = wheel.targetRpm - wheel.rpm;
  wheel.pwm = pid.update(wheel.error, elapsedMs / 1000.0f, -255.0f, 255.0f);
  motor.setPwm(wheel.pwm);
}

void RobotController::setWheelTargets(float leftRpm, float rightRpm, DriveMode mode) {
  left_.targetRpm = constrain(leftRpm, -config_.maxRpm, config_.maxRpm);
  right_.targetRpm = constrain(rightRpm, -config_.maxRpm, config_.maxRpm);
  leftPid_.reset();
  rightPid_.reset();
  mode_ = mode;
  coastWhenStopped_ = false;
  lastCommandMs_ = millis();
}

void RobotController::setCmdVel(float linearMps, float angularRadps) {
  float leftMps = linearMps - angularRadps * config_.wheelBaseM * 0.5f;
  float rightMps = linearMps + angularRadps * config_.wheelBaseM * 0.5f;
  float metersPerRevolution = PI * config_.wheelDiameterM;
  setWheelTargets(leftMps / metersPerRevolution * 60.0f,
                  rightMps / metersPerRevolution * 60.0f, DRIVE_CMDVEL);
}

void RobotController::stop(bool brake) {
  mode_ = DRIVE_STOP;
  coastWhenStopped_ = !brake;
  left_.targetRpm = right_.targetRpm = 0.0f;
  left_.error = right_.error = 0.0f;
  leftPid_.reset();
  rightPid_.reset();
  if (brake) {
    leftMotor_.brake();
    rightMotor_.brake();
  } else {
    leftMotor_.coast();
    rightMotor_.coast();
  }
}

void RobotController::resetOdometry() {
  leftEncoder_.zero();
  rightEncoder_.zero();
  left_.ticks = left_.lastTicks = 0;
  right_.ticks = right_.lastTicks = 0;
  odometry_.reset();
}

void RobotController::reportIfDue(unsigned long now, Stream &output) {
  if (now - lastReportMs_ >= config_.reportPeriodMs) {
    printTelemetry(now, output);
    lastReportMs_ = now;
  }
}

void RobotController::handleCommand(String line, Stream &output) {
  line.trim();
  line.toUpperCase();
  if (line == "HELP") printHelp(output);
  else if (line == "STATUS") printTelemetry(millis(), output);
  else if (line == "STOP" || line == "BRAKE") { stop(true); output.println(F("OK stopped")); }
  else if (line == "FREE") { stop(false); output.println(F("OK coast")); }
  else if (line == "ZERO") { resetOdometry(); output.println(F("OK zero")); }
  else if (line.startsWith("WHEEL ")) {
    float leftRpm, rightRpm;
    if (parseTwoFloats(line.substring(6), leftRpm, rightRpm)) {
      setWheelTargets(leftRpm, rightRpm, DRIVE_WHEEL); output.println(F("OK wheel"));
    } else output.println(F("ERR use: WHEEL left_rpm right_rpm"));
  } else if (line.startsWith("CMDVEL ")) {
    float linear, angular;
    if (parseTwoFloats(line.substring(7), linear, angular)) {
      setCmdVel(linear, angular); output.println(F("OK cmdvel"));
    } else output.println(F("ERR use: CMDVEL linear_mps angular_radps"));
  } else if (line.startsWith("PID ")) setPid(line.substring(4), output);
  else if (line.startsWith("CONFIG ")) setConfig(line.substring(7), output);
  else { output.print(F("ERR unknown command: ")); output.println(line); }
}

bool RobotController::setPid(String arguments, Stream &output) {
  String target = nextToken(arguments);
  float kp, ki, kd;
  if (!parseThreeFloats(arguments, kp, ki, kd)) {
    output.println(F("ERR use: PID SPEED|LEFT|RIGHT kp ki kd")); return false;
  }
  if (target == "SPEED" || target == "BOTH") {
    leftPid_.setGains(kp, ki, kd); rightPid_.setGains(kp, ki, kd);
  } else if (target == "LEFT") leftPid_.setGains(kp, ki, kd);
  else if (target == "RIGHT") rightPid_.setGains(kp, ki, kd);
  else { output.println(F("ERR PID target must be SPEED, LEFT, or RIGHT")); return false; }
  output.println(F("OK pid"));
  return true;
}

bool RobotController::setConfig(String arguments, Stream &output) {
  String name = nextToken(arguments);
  float value = arguments.toFloat();
  bool ok = true;
  if (name == "CPR" && value > 0.0f) config_.ticksPerWheelRevolution = value;
  else if (name == "MAXRPM" && value > 0.0f) config_.maxRpm = value;
  else if (name == "WHEEL_DIAM" && value > 0.0f) config_.wheelDiameterM = value;
  else if (name == "WHEEL_BASE" && value > 0.0f) config_.wheelBaseM = value;
  else if (name == "REPORT" && value >= 20.0f) config_.reportPeriodMs = (unsigned long)value;
  else if (name == "TIMEOUT" && value >= 100.0f) config_.commandTimeoutMs = (unsigned long)value;
  else ok = false;
  output.println(ok ? F("OK config") : F("ERR bad config value or name"));
  return ok;
}

String RobotController::nextToken(String &text) {
  text.trim();
  int separator = text.indexOf(' ');
  if (separator < 0) { String token = text; text = ""; return token; }
  String token = text.substring(0, separator);
  text = text.substring(separator + 1);
  text.trim();
  return token;
}

bool RobotController::parseTwoFloats(String text, float &a, float &b) {
  String first = nextToken(text);
  String second = nextToken(text);
  if (first.length() == 0 || second.length() == 0) return false;
  a = first.toFloat(); b = second.toFloat(); return true;
}

bool RobotController::parseThreeFloats(String text, float &a, float &b, float &c) {
  String first = nextToken(text);
  String second = nextToken(text);
  String third = nextToken(text);
  if (first.length() == 0 || second.length() == 0 || third.length() == 0) return false;
  a = first.toFloat(); b = second.toFloat(); c = third.toFloat(); return true;
}

const char *RobotController::modeName() const {
  if (mode_ == DRIVE_WHEEL) return "WHEEL";
  if (mode_ == DRIVE_CMDVEL) return "CMDVEL";
  return "STOP";
}

void RobotController::printTelemetry(unsigned long now, Stream &output) const {
  float circumference = PI * config_.wheelDiameterM;
  float linear = (left_.rpm + right_.rpm) * circumference / 120.0f;
  float angular = (right_.rpm - left_.rpm) * circumference / (60.0f * config_.wheelBaseM);
  output.print(F("DATA,t=")); output.print(now);
  output.print(F(",mode=")); output.print(modeName());
  output.print(F(",target_l=")); output.print(left_.targetRpm, 2);
  output.print(F(",rpm_l=")); output.print(left_.rpm, 2);
  output.print(F(",pwm_l=")); output.print(left_.pwm, 1);
  output.print(F(",err_l=")); output.print(left_.error, 2);
  output.print(F(",target_r=")); output.print(right_.targetRpm, 2);
  output.print(F(",rpm_r=")); output.print(right_.rpm, 2);
  output.print(F(",pwm_r=")); output.print(right_.pwm, 1);
  output.print(F(",err_r=")); output.print(right_.error, 2);
  output.print(F(",x=")); output.print(odometry_.x(), 4);
  output.print(F(",y=")); output.print(odometry_.y(), 4);
  output.print(F(",theta=")); output.print(odometry_.heading(), 4);
  output.print(F(",v=")); output.print(linear, 3);
  output.print(F(",w=")); output.println(angular, 3);
}

void RobotController::printHelp(Stream &output) const {
  output.println(F("Commands:"));
  output.println(F("  WHEEL left_rpm right_rpm"));
  output.println(F("  CMDVEL linear_mps angular_radps"));
  output.println(F("  PID SPEED|LEFT|RIGHT kp ki kd"));
  output.println(F("  CONFIG CPR|MAXRPM|WHEEL_DIAM|WHEEL_BASE|REPORT|TIMEOUT value"));
  output.println(F("  ZERO | STOP | BRAKE | FREE | STATUS | HELP"));
}
