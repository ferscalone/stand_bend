#pragma once

#include <Arduino.h>

class StepperMotor {
 public:
  StepperMotor(uint8_t stepPlusPin, uint8_t stepMinusPin, uint8_t dirPlusPin, uint8_t dirMinusPin);

  void begin();
  void stop();
  void setDirection(bool forward);
  void moveSteps(int32_t steps, uint32_t stepIntervalUs);
  int32_t getExpectedSteps() const;
  float getExpectedDegrees() const;
  void resetExpectedPosition();
  // Инверсия физической линии DIR. Логическое «вперёд» (+шаги) после этого всегда
  // соответствует росту угла энкодера (выставляется автопробой при загрузке).
  void setInvertDirection(bool invert);
  bool isDirectionInverted() const;

 private:
  void pulseStep(uint32_t stepIntervalUs);

  uint8_t stepPlusPin_;
  uint8_t stepMinusPin_;
  uint8_t dirPlusPin_;
  uint8_t dirMinusPin_;
  bool directionForward_ = true;
  bool invertDirection_ = false;
  int32_t expectedSteps_ = 0;
};

