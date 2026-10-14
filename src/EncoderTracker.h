#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

class EncoderTracker {
 public:
  bool begin(TwoWire& wire);
  bool update();
  void captureLogicalZero();

  float getAbsoluteDegrees() const;
  float getRelativeDegrees() const;
  float getRelativeCounts() const;
  int32_t getMultiTurnCounts() const;

  static float normalizeDegrees(float degrees);
  static float shortestDeltaDegrees(float from, float to);

 private:
  bool readRawWithRetry(uint16_t& out);

  AS5600 encoder_{&Wire};
  bool initialized_ = false;
  bool logicalZeroCaptured_ = false;
  int16_t lastRaw_ = -1;
  int32_t multiTurnCounts_ = 0;
  int32_t zeroCounts_ = 0;
};
