#include "Encoder.h"

Encoder *Encoder::instances_[2] = {nullptr, nullptr};

Encoder::Encoder(uint8_t channelA, uint8_t channelB, bool inverted)
    : channelA_(channelA), channelB_(channelB), inverted_(inverted), ticks_(0) {}

void Encoder::begin(uint8_t interruptSlot) {
  pinMode(channelA_, INPUT_PULLUP);
  pinMode(channelB_, INPUT_PULLUP);
  if (interruptSlot > 1 || digitalPinToInterrupt(channelA_) == NOT_AN_INTERRUPT) return;
  instances_[interruptSlot] = this;
  attachInterrupt(digitalPinToInterrupt(channelA_), interruptSlot == 0 ? isr0 : isr1, CHANGE);
}

long Encoder::readTicks() const {
  noInterrupts();
  long result = ticks_;
  interrupts();
  return result;
}

void Encoder::zero() {
  noInterrupts();
  ticks_ = 0;
  interrupts();
}

void Encoder::isr0() {
  if (instances_[0] != nullptr) instances_[0]->onAChanged();
}

void Encoder::isr1() {
  if (instances_[1] != nullptr) instances_[1]->onAChanged();
}

void Encoder::onAChanged() {
  int step = digitalRead(channelA_) == digitalRead(channelB_) ? 1 : -1;
  ticks_ += inverted_ ? -step : step;
}
