#include "StepperMotor.h"

#include "Config.h"

StepperMotor::StepperMotor(uint8_t stepPlusPin, uint8_t stepMinusPin, uint8_t dirPlusPin, uint8_t dirMinusPin)
    : stepPlusPin_(stepPlusPin),
      stepMinusPin_(stepMinusPin),
      dirPlusPin_(dirPlusPin),
      dirMinusPin_(dirMinusPin) {}

void StepperMotor::begin() {
  pinMode(stepPlusPin_, OUTPUT);
  pinMode(stepMinusPin_, OUTPUT);
  pinMode(dirPlusPin_, OUTPUT);
  pinMode(dirMinusPin_, OUTPUT);

  digitalWrite(stepMinusPin_, LOW);
  digitalWrite(dirMinusPin_, LOW);
  digitalWrite(stepPlusPin_, LOW);
  digitalWrite(dirPlusPin_, HIGH);
}

void StepperMotor::stop() {
  digitalWrite(stepPlusPin_, LOW);
}

void StepperMotor::setDirection(bool forward) {
  directionForward_ = forward;
  digitalWrite(dirPlusPin_, forward ? HIGH : LOW);
}

void StepperMotor::moveSteps(int32_t steps, uint32_t stepIntervalUs) {
  if (steps == 0) {
    return;
  }

  const bool forward = steps > 0;
  setDirection(forward);

  const int32_t count = abs(steps);
  for (int32_t i = 0; i < count; ++i) {
    pulseStep(stepIntervalUs);
    expectedSteps_ += forward ? 1 : -1;
  }
}

int32_t StepperMotor::getExpectedSteps() const {
  return expectedSteps_;
}

float StepperMotor::getExpectedDegrees() const {
  return expectedSteps_ * Config::DEGREES_PER_STEP;
}

void StepperMotor::resetExpectedPosition() {
  expectedSteps_ = 0;
}

void StepperMotor::pulseStep(uint32_t stepIntervalUs) {
  digitalWrite(stepPlusPin_, HIGH);
  delayMicroseconds(stepIntervalUs / 2);
  digitalWrite(stepPlusPin_, LOW);
  delayMicroseconds(stepIntervalUs / 2);
}

