#pragma once

#include <Arduino.h>

class Encoder {
 public:
  Encoder(uint8_t channelA, uint8_t channelB, bool inverted = false);
  void begin(uint8_t interruptSlot);
  long readTicks() const;
  void zero();

 private:
  static Encoder *instances_[2];
  static void isr0();
  static void isr1();
  void onAChanged();

  uint8_t channelA_;
  uint8_t channelB_;
  bool inverted_;
  volatile long ticks_;
};
