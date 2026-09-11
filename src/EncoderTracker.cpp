#include "EncoderTracker.h"

#include "Config.h"

namespace {

// Диагностическое сканирование шины I2C. Печатает найденные адреса в Serial, чтобы при
// неверной проводке SDA/SCL сразу было видно, отвечает ли вообще что-то на шине, а не
// только сухое "AS5600 init failed" без подробностей.
void scanI2CBus(TwoWire& wire) {
  Serial.printf("[ENCODER] I2C scan on SDA=%d SCL=%d ...\n",
                static_cast<int>(Config::PIN_AS5600_SDA), static_cast<int>(Config::PIN_AS5600_SCL));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    wire.beginTransmission(addr);
    if (wire.endTransmission() == 0) {
      Serial.printf("[ENCODER]   found device at 0x%02X%s\n", addr, (addr == 0x36) ? "  <- AS5600" : "");
      ++found;
    }
  }
  if (found == 0) {
    Serial.println("[ENCODER]   no I2C device responded - check SDA/SCL wiring, power and pull-ups "
                    "on the pins set in Config.h (PIN_AS5600_SDA / PIN_AS5600_SCL)");
  }
}

}  // namespace

bool EncoderTracker::begin(TwoWire& wire) {
  wire.begin(Config::PIN_AS5600_SDA, Config::PIN_AS5600_SCL);
  scanI2CBus(wire);

  // Нет физического DIR-пина (только I2C) - используем программный режим направления
  // (аргумент по умолчанию AS5600_SW_DIRECTION_PIN = 255). Раньше здесь стоял вызов
  // begin(4), который ошибочно конфигурировал GPIO4 как "аппаратный" пин направления
  // AS5600 (pinMode+digitalWrite внутри библиотеки) - а GPIO4 это же PIN_STEP_P (импульс
  // ШАГ мотора) из Config.h: два разных модуля дрались за один физический пин.
  if (!encoder_.begin()) {
    Serial.println("[ENCODER] AS5600 (0x36) did not ACK on the I2C bus");
    return false;
  }

  const uint16_t raw = encoder_.rawAngle();
  if (encoder_.lastError() != AS5600_OK) {
    // Раньше здесь стояла проверка "raw > 4095", которая никогда не срабатывала:
    // rawAngle() в библиотеке всегда маскирует результат в диапазон 0..4095 (& 0x0FFF),
    // причём даже при полностью оборванной шине I2C (тогда она просто возвращает 0).
    // Из-за этого init считался "успешным", даже если энкодер физически не отвечает -
    // отсюда и ситуация "контроллер не видит энкодер" без единой ошибки в логе.
    Serial.println("[ENCODER] I2C read error while reading AS5600 angle");
    return false;
  }

  lastRaw_ = static_cast<int16_t>(raw);
  multiTurnCounts_ = raw;
  zeroCounts_ = raw;
  initialized_ = true;
  return true;
}

bool EncoderTracker::update() {
  if (!initialized_) {
    return false;
  }

  const uint16_t raw = encoder_.rawAngle();
  if (encoder_.lastError() != AS5600_OK) {
    // См. комментарий в begin(): "raw > 4095" никогда не срабатывает, т.к. rawAngle()
    // всегда возвращает значение в диапазоне 0..4095, даже при обрыве I2C.
    return false;
  }

  int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(lastRaw_);
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }

  multiTurnCounts_ += delta;
  lastRaw_ = static_cast<int16_t>(raw);
  return true;
}

void EncoderTracker::captureLogicalZero() {
  zeroCounts_ = multiTurnCounts_;
  logicalZeroCaptured_ = true;
}

float EncoderTracker::getAbsoluteDegrees() const {
  const int32_t raw = ((lastRaw_ % 4096) + 4096) % 4096;
  return raw * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
}

float EncoderTracker::getRelativeDegrees() const {
  // До калибровки логического нуля (то есть до первого homing после перепрошивки)
  // zeroCounts_ всё ещё равен позиции при запуске, поэтому multiTurnCounts_ - zeroCounts_
  // равно 0 где бы ни находился вал - что читается как «в домашней позиции», даже когда
  // это не так. До тех пор вместо этого сообщается истинное отклонение от ИЗВЕСТНОГО угла
  // домашней позиции, чтобы отображаемое отклонение отражало реальность (например, вал,
  // припаркованный в мёртвой зоне за домашней позицией, показывает свою реальную угловую
  // ошибку, а не 0). Это значение ТОЛЬКО для отображения; детектирование проскальзывания
  // использует getRelativeCounts() и не затрагивается.
  //
  // Знак ОБЯЗАН совпадать с калиброванным путём (multiTurnCounts_ - zeroCounts_ ≈
  // absDeg - HOME): shortestDeltaDegrees(HOME, absDeg) = absDeg - HOME, так что дуга
  // циферблата поворачивается одинаково до и после homing. Перестановка аргументов меняет
  // знак и зеркалит дугу.
  if (!logicalZeroCaptured_) {
    return shortestDeltaDegrees(Config::HOME_ABSOLUTE_ANGLE_DEG, getAbsoluteDegrees());
  }
  return getRelativeCounts() * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
}

float EncoderTracker::getRelativeCounts() const {
  return static_cast<float>(multiTurnCounts_ - zeroCounts_);
}

int32_t EncoderTracker::getMultiTurnCounts() const {
  return multiTurnCounts_;
}

float EncoderTracker::normalizeDegrees(float degrees) {
  while (degrees >= 360.0f) {
    degrees -= 360.0f;
  }
  while (degrees < 0.0f) {
    degrees += 360.0f;
  }
  return degrees;
}

float EncoderTracker::shortestDeltaDegrees(float from, float to) {
  float delta = normalizeDegrees(to) - normalizeDegrees(from);
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  return delta;
}
