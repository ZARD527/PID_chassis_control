/*
  Dual-wheel differential-drive controller for Arduino Uno/Nano + TB6612.
  Upload this sketch folder as a whole. Hardware and serial commands are in
  README.md. The implementation is split into small modules beside this file.
*/

#include "RobotController.h"

RobotController robot;
String serialLine;

void setup() {
  Serial.begin(115200);
  serialLine.reserve(96);
  robot.begin();
  Serial.println(F("OK dual-wheel PID controller ready"));
  robot.printHelp(Serial);
}

void loop() {
  robot.update(millis());

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        robot.handleCommand(serialLine, Serial);
        serialLine = "";
      }
    } else if (serialLine.length() < 120) {
      serialLine += c;
    }
  }

  robot.reportIfDue(millis(), Serial);
}
