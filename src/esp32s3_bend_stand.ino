#include <Arduino.h>
#include <Wire.h>

#include "ButtonInput.h"
#include "Config.h"
#include "ConfigStore.h"
#include "EncoderTracker.h"
#include "NetworkManager.h"
#include "StepperMotor.h"
#include "SystemState.h"
#include "Telemetry.h"

// ── Под-фаза прогона RUNNING ─────────────────────────────────────────────────
// Программа изгиба - это конечный автомат внутри состояния RUNNING. Один ЦИКЛ -
// полный размах 0 -> 180 -> 0. Порядок за прогон из N циклов:
//   вход:                90 -> 0            (TO_LEFT)     [в счёт не идёт]
//   выдержка в 0:                            (DWELL_LEFT)
//   для i = 1..N:
//       0 -> 180                             (TO_RIGHT)
//       выдержка в 180:                      (DWELL_RIGHT)
//       180 -> 0                             (TO_LEFT)
//       выдержка в 0:                        (DWELL_LEFT)  <- цикл i завершён
//   выход:              0 -> 90 (центр)      -> RETURN_HOME -> IDLE
// Объявлено вверху (рядом с прочими enum), чтобы тип был определён к моменту, где
// сборка Arduino вставляет автогенерируемые прототипы функций.
enum class CyclePhase : uint8_t {
  IDLE,          // прогон не идёт
  ENTER_LEFT,    // 90 -> 0 (вход, не считается)
  DWELL_LEFT,    // стоим в 0
  TO_RIGHT,      // 0 -> 180
  DWELL_RIGHT,   // стоим в 180
  TO_LEFT,       // 180 -> 0 (завершает цикл в момент прихода)
  TO_CENTER      // 0 -> 90 (выход)
};

inline const char* toString(CyclePhase p) {
  switch (p) {
    case CyclePhase::IDLE: return "IDLE";
    case CyclePhase::ENTER_LEFT: return "ENTER_LEFT";
    case CyclePhase::DWELL_LEFT: return "DWELL_LEFT";
    case CyclePhase::TO_RIGHT: return "TO_RIGHT";
    case CyclePhase::DWELL_RIGHT: return "DWELL_RIGHT";
    case CyclePhase::TO_LEFT: return "TO_LEFT";
    case CyclePhase::TO_CENTER: return "TO_CENTER";
    default: return "IDLE";
  }
}

// Прямые объявления процедур, определённых ниже, но вызываемых раньше по тексту
// (motorTask -> runCycleProgram -> moveToRelDeg). Сборка Arduino обычно
// сама генерирует прототипы, но здесь они заданы явно, чтобы порядок определений не
// зависел от этой генерации.
bool moveToRelDeg(float targetRelDeg, uint32_t stepIntervalUs, bool allowAbort, bool* aborted);
bool dwell(uint32_t durationMs);
void runCycleProgram();

struct SharedData {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile SystemState state = SystemState::BOOT;
  volatile bool encoderReady = false;
  volatile float encoderAbsoluteDeg = 0.0f;
  volatile float encoderRelativeDeg = 0.0f;
  volatile float expectedPositionDeg = 0.0f;
  volatile bool encoderFault = false;
  volatile float encoderErrorDeg = 0.0f;
  volatile uint32_t cyclesDone = 0;
  volatile uint32_t cyclesTarget = 0;
  volatile CyclePhase phase = CyclePhase::IDLE;
};

SharedData gShared;

// Изменяемый во время работы конфиг программы циклов (число циклов, выдержка,
// скорость, амплитуда). У него СВОЙ spinlock, чтобы чтение/запись конфигурации
// никогда не конкурировали с горячей критической секцией энкодера/телеметрии.
// Инициализируется значениями по умолчанию из Config.h, затем перекрывается из NVS.
struct RuntimeConfig {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t cycles = Config::CYCLES_DEFAULT;
  volatile uint32_t dwellMs = Config::DWELL_MS_DEFAULT;
  volatile uint32_t speedUs = Config::SPEED_US_DEFAULT;
  volatile float amplitudeDeg = Config::AMPLITUDE_DEG_DEFAULT;
};
RuntimeConfig gConfig;

// Атомарный снимок оперативной конфигурации (одна критическая секция), чтобы
// вызывающий код видел все четыре значения одного момента.
void getConfigSnapshot(CycleConfig& out) {
  portENTER_CRITICAL(&gConfig.mux);
  out.cycles = gConfig.cycles;
  out.dwellMs = gConfig.dwellMs;
  out.speedUs = gConfig.speedUs;
  out.amplitudeDeg = gConfig.amplitudeDeg;
  portEXIT_CRITICAL(&gConfig.mux);
}

// Атомарная запись всех значений конфигурации. Вызывающий код ОБЯЗАН сначала
// проверить их (validateConfig). Безопасно из сетевой задачи (ядро 0) параллельно с
// чтениями задачи мотора (ядро 1) через spinlock.
void setConfig(uint32_t cycles, uint32_t dwellMs, uint32_t speedUs, float amplitudeDeg) {
  portENTER_CRITICAL(&gConfig.mux);
  gConfig.cycles = cycles;
  gConfig.dwellMs = dwellMs;
  gConfig.speedUs = speedUs;
  gConfig.amplitudeDeg = amplitudeDeg;
  portEXIT_CRITICAL(&gConfig.mux);
}

ButtonInput gButton(Config::PIN_BUTTON, Config::BUTTON_DEBOUNCE_MS);
EncoderTracker gEncoder;
StepperMotor gMotor(Config::PIN_STEP_P, Config::PIN_STEP_N, Config::PIN_DIR_P, Config::PIN_DIR_N);

TaskHandle_t gMotorTaskHandle = nullptr;
int32_t gHomeReferenceSteps = 0;

// Устанавливается обработчиком кнопки/сети, чтобы попросить motorTask восстановиться
// из ERROR / выполнить homing. motorTask владеет энкодером/мотором, поэтому homing
// выполняется там, чтобы избежать межъядерной гонки по I2C/мотору.
volatile bool gRehomeRequested = false;
// Устанавливается setup(), чтобы попросить motorTask выполнить ОДНОРАЗОВЫЙ homing при
// загрузке. Отделено от gRehomeRequested, чтобы два намерения оставались различимыми.
volatile bool gBootHomeRequested = false;
// Устанавливается кнопкой/сетью, чтобы запросить старт/останов прогона. Читаются и
// сбрасываются в motorTask, который единолично владеет автоматом циклов.
volatile bool gStartRequested = false;
volatile bool gStopRequested = false;

// ── Аксессоры разделяемого состояния ─────────────────────────────────────────
void setState(SystemState newState) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.state = newState;
  portEXIT_CRITICAL(&gShared.mux);
  // Немедленно отправить переход WS-клиентам дашборда. queueEvent безопасен для
  // задач и неблокирующий; переходы до begin() безопасно отбрасываются.
  NetworkManager::queueEvent(buildEventJson("state", toString(newState)).c_str());
}

SystemState getState() {
  portENTER_CRITICAL(&gShared.mux);
  const SystemState state = gShared.state;
  portEXIT_CRITICAL(&gShared.mux);
  return state;
}

void setPhase(CyclePhase phase) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.phase = phase;
  portEXIT_CRITICAL(&gShared.mux);
}

void setCycleProgress(uint32_t done, uint32_t target) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.cyclesDone = done;
  gShared.cyclesTarget = target;
  portEXIT_CRITICAL(&gShared.mux);
}

void updateEncoderShared(bool ready, float absoluteDeg, float relativeDeg, float expectedDeg, bool encoderFault, float encoderErrorDeg) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.encoderReady = ready;
  gShared.encoderAbsoluteDeg = absoluteDeg;
  gShared.encoderRelativeDeg = relativeDeg;
  gShared.expectedPositionDeg = expectedDeg;
  gShared.encoderFault = encoderFault;
  gShared.encoderErrorDeg = encoderErrorDeg;
  portEXIT_CRITICAL(&gShared.mux);
}

float getEncoderRelativeDeg() {
  portENTER_CRITICAL(&gShared.mux);
  const float deg = gShared.encoderRelativeDeg;
  portEXIT_CRITICAL(&gShared.mux);
  return deg;
}

// ── Преобразование bend-кадр <-> относительный угол энкодера ──────────────────
// bend_angle = BEND_ANCHOR_DEG + BEND_DIR_SIGN * rel  (анкер = логический ноль =
// bend 180°). Знак BEND_DIR_SIGN выбирает СТОРОНУ размаха относительно анкера (см.
// Config.h). Обратно: rel = BEND_DIR_SIGN * (bend - ANCHOR), т.к. SIGN = ±1 и
// 1/SIGN == SIGN.
float bendToRel(float bendDeg) {
  return Config::BEND_DIR_SIGN * (bendDeg - Config::BEND_ANCHOR_DEG);
}
float relToBend(float relDeg) {
  return Config::BEND_ANCHOR_DEG + Config::BEND_DIR_SIGN * relDeg;
}

// Целевые относительные углы левой/правой/центральной точек для заданной амплитуды.
// Центр всегда bend 90° (rel -90). Левая = 90-amp, правая = 90+amp.
float relLeft(float amplitudeDeg) { return bendToRel(Config::BEND_CENTER_DEG - amplitudeDeg); }
float relRight(float amplitudeDeg) { return bendToRel(Config::BEND_CENTER_DEG + amplitudeDeg); }
float relCenter() { return bendToRel(Config::BEND_CENTER_DEG); }

// ── Снимок телеметрии ────────────────────────────────────────────────────────
void getTelemetrySnapshot(TelemetrySnapshot& out) {
  portENTER_CRITICAL(&gShared.mux);
  out.state = gShared.state;
  out.encoderReady = gShared.encoderReady;
  out.encAbsDeg = gShared.encoderAbsoluteDeg;
  out.encRelDeg = gShared.encoderRelativeDeg;
  out.encoderFault = gShared.encoderFault;
  out.cyclesDone = gShared.cyclesDone;
  out.cyclesTarget = gShared.cyclesTarget;
  out.phase = toString(gShared.phase);
  portEXIT_CRITICAL(&gShared.mux);

  // bend-угол выводится из относительного угла того же снимка.
  out.bendAngleDeg = relToBend(out.encRelDeg);

  // Поля конфигурации берутся из gConfig под ЕГО СОБСТВЕННОЙ блокировкой - в
  // отдельной критической секции (никогда не вложенной в gShared.mux).
  portENTER_CRITICAL(&gConfig.mux);
  out.cfgCycles = gConfig.cycles;
  out.cfgDwellMs = gConfig.dwellMs;
  out.cfgSpeedUs = gConfig.speedUs;
  out.cfgAmplitudeDeg = gConfig.amplitudeDeg;
  portEXIT_CRITICAL(&gConfig.mux);
}

// ── Кнопка ───────────────────────────────────────────────────────────────────
// Опрос кнопки и немедленная реакция по текущему состоянию. Вызывается ИЗ motorTask
// (ядро 1) - того же ядра/задачи, что исполняет движение, поэтому во время прогона
// (когда runCycleProgram блокирует ядро 1) кнопка всё равно опрашивается в
// контрольных точках dwell()/moveToRelDeg(), а не теряется. Прежде опрос был только
// в loop() на том же ядре 1, и блокирующий прогон почти не давал ему выполниться -
// нажатие во время цикла терялось. gButton.update() сам делает антидребезг, поэтому
// его нужно звать достаточно часто и ТОЛЬКО отсюда (единый опросчик, без гонки за
// внутренним состоянием антидребезга).
//
//   IDLE    -> START (запуск прогона)
//   RUNNING -> STOP  (прервать -> возврат в центр)
//   ERROR   -> re-home
void pollButton() {
  if (!gButton.update()) {
    return;
  }
  Serial.println("[BUTTON] Press detected");
  switch (getState()) {
    case SystemState::IDLE:
      gStartRequested = true;
      Serial.println("[BUTTON] START requested");
      break;
    case SystemState::RUNNING:
      gStopRequested = true;
      Serial.println("[BUTTON] STOP requested");
      break;
    case SystemState::ERROR:
      gRehomeRequested = true;
      Serial.println("[BUTTON] ERROR -> re-home requested");
      break;
    default:
      // BOOT / HOMING / RETURN_HOME: нажатие не действует.
      break;
  }
}

// ── Диагностика проскальзывания энкодера ─────────────────────────────────────
// Сравнивает ожидаемый (по шагам мотора) и фактический (по энкодеру) относительный
// угол. Обновляет разделяемое состояние. Возвращает true при превышении порога
// рассогласования (используется только как предупреждение во время управляемого
// движения; жёсткий отказ по положению ловит детектор застревания в moveToRel).
bool checkEncoderSlip(float* outErrorDeg = nullptr) {
  const bool encoderOk = gEncoder.update();
  const float expectedDeg = gMotor.getExpectedDegrees();

  if (!encoderOk) {
    updateEncoderShared(false, 0.0f, 0.0f, expectedDeg, true, 0.0f);
    return true;
  }

  const float expectedCounts = static_cast<float>(gMotor.getExpectedSteps()) * Config::ENCODER_COUNTS_PER_STEP;
  const float actualCounts = gEncoder.getRelativeCounts();
  const float errorCounts = actualCounts - expectedCounts;
  const float error = errorCounts * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
  const bool fault = fabsf(error) > Config::ENCODER_POSITION_ERROR_DEG;
  const float actualDeg = gEncoder.getRelativeDegrees();

  if (outErrorDeg != nullptr) {
    *outErrorDeg = error;
  }

  updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), actualDeg, expectedDeg, fault, error);
  return fault;
}

// ── Автоопределение полярности DIR ───────────────────────────────────────────
// Делает DIR_PROBE_STEPS шагов «вперёд» и смотрит, куда сдвинулся энкодер. Если угол
// уменьшился - инвертирует DIR, чтобы логическое «+» всегда совпадало с ростом угла
// энкодера (на этом держится вся замкнутая по энкодеру логика движения). Если вал
// почти не сдвинулся - мотор не крутится: возвращает false с диагностикой.
bool probeMotorDirection() {
  if (!gEncoder.update()) {
    Serial.println("[ERROR] Encoder read failed before direction probe");
    return false;
  }
  const int32_t before = gEncoder.getMultiTurnCounts();
  gMotor.moveSteps(Config::DIR_PROBE_STEPS, Config::HOME_STEP_INTERVAL_US);
  gMotor.stop();
  delay(Config::MOVE_VERIFY_SETTLE_MS);
  if (!gEncoder.update()) {
    Serial.println("[ERROR] Encoder read failed after direction probe");
    return false;
  }
  const float movedDeg = static_cast<float>(gEncoder.getMultiTurnCounts() - before) *
                         (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
  const float expectedDeg = Config::DIR_PROBE_STEPS * Config::DEGREES_PER_STEP;
  Serial.printf("[PROBE] %ld steps forward -> encoder moved %.2f deg (expected ~%.2f)\n",
                static_cast<long>(Config::DIR_PROBE_STEPS), static_cast<double>(movedDeg),
                static_cast<double>(expectedDeg));

  if (fabsf(movedDeg) < Config::DIR_PROBE_MIN_DEG) {
    Serial.println("[ERROR] Motor did not move the shaft: check driver power/enable, "
                   "STEP/DIR wiring (PIN_STEP_P/PIN_DIR_P in Config.h) and that the "
                   "encoder magnet is on the same shaft");
    NetworkManager::queueEvent(buildEventJson("error", "motor_not_moving").c_str());
    return false;
  }
  if (movedDeg < 0.0f) {
    gMotor.setInvertDirection(!gMotor.isDirectionInverted());
    Serial.println("[PROBE] Encoder counts opposite to motor - DIR inverted in software");
  } else {
    Serial.println("[PROBE] Motor/encoder direction OK");
  }
  return true;
}

// ── Homing к абсолютному домашнему углу (захват логического нуля) ─────────────
// Приводит вал к HOME_ABSOLUTE_ANGLE_DEG (= bend 180°, логический ноль), подтверждает
// несколькими отсчётами и захватывает логический ноль. Направление - кратчайшая дуга
// к домашнему углу (нет нагрузки/стенки, поэтому кратчайший путь безопасен).
// Возвращает false при сбое чтения энкодера / превышении хода.
bool runHoming() {
  setState(SystemState::HOMING);
  Serial.println("[STATE] HOMING");

  if (!gEncoder.update()) {
    Serial.println("[ERROR] Encoder read failed during homing");
    return false;
  }

  delay(Config::HOMING_SETTLE_MS);

  // Полярность мотор/энкодер проверяется один раз за загрузку (до первого homing).
  static bool directionProbed = false;
  if (!directionProbed) {
    if (!probeMotorDirection()) {
      return false;
    }
    directionProbed = true;
  }

  uint32_t safetyCounter = 0;
  uint8_t readErrors = 0;
  while (true) {
    // Дать поработать loop() (сеть/[DIAG]) - он на том же ядре 1 с меньшим приоритетом.
    vTaskDelay(1);
    if (!gEncoder.update()) {
      if (++readErrors >= Config::ENCODER_READ_ERROR_LIMIT) {
        Serial.println("[ERROR] Encoder read failed during homing move");
        return false;
      }
      continue;
    }
    readErrors = 0;

    const float absoluteDeg = gEncoder.getAbsoluteDegrees();
    const float homeErrorDeg = EncoderTracker::shortestDeltaDegrees(absoluteDeg, Config::HOME_ABSOLUTE_ANGLE_DEG);

    updateEncoderShared(true, absoluteDeg, gEncoder.getRelativeDegrees(),
                        gMotor.getExpectedDegrees(), false, homeErrorDeg);

    if (fabsf(homeErrorDeg) <= Config::ENCODER_HOME_WINDOW_DEG) {
      gMotor.stop();

      // Не принимать домашнюю позицию по одному удачному отсчёту: после подачи
      // питания драйвера вал может ещё расслабляться. Требуется несколько
      // последовательных отсчётов в пределах окна после короткого успокоения.
      delay(Config::HOMING_VERIFY_SETTLE_MS);

      bool confirmedHome = true;
      for (uint8_t i = 0; i < Config::HOMING_VERIFY_SAMPLES; ++i) {
        if (!gEncoder.update()) {
          Serial.println("[ERROR] Encoder read failed during homing verify");
          return false;
        }
        const float verifyDeg = gEncoder.getAbsoluteDegrees();
        const float verifyErrorDeg = EncoderTracker::shortestDeltaDegrees(verifyDeg, Config::HOME_ABSOLUTE_ANGLE_DEG);
        if (fabsf(verifyErrorDeg) > Config::ENCODER_HOME_WINDOW_DEG) {
          confirmedHome = false;
          break;
        }
        delay(10);
      }

      if (confirmedHome) {
        break;
      }
      continue;
    }

    // Направление - по знаку ошибки, пересчитывается каждую итерацию (самокоррекция
    // при перелёте). Размер шага двухскоростной: вдали - пачка, у анкера - один шаг,
    // чтобы попасть в окно ENCODER_HOME_WINDOW_DEG, а не колебаться вокруг него.
    const float absErr = fabsf(homeErrorDeg);
    const int32_t dirSign = (homeErrorDeg > 0.0f) ? 1 : -1;
    const int32_t chunk = (absErr > Config::MOVE_COARSE_BAND_DEG) ? Config::HOME_STEP_CHUNK : 1;
    gMotor.moveSteps(dirSign * chunk, Config::HOME_STEP_INTERVAL_US);

    if (++safetyCounter > (Config::HOME_MAX_TRAVEL_STEPS / 1)) {
      Serial.println("[ERROR] Homing failed: target home angle not reached");
      return false;
    }
  }

  gEncoder.captureLogicalZero();
  gMotor.resetExpectedPosition();
  gHomeReferenceSteps = 0;
  updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), gEncoder.getRelativeDegrees(), gMotor.getExpectedDegrees(), false, 0.0f);

  Serial.printf("[HOMING] Reached home angle %.2f deg (= bend %.0f), logical zero captured\n",
                Config::HOME_ABSOLUTE_ANGLE_DEG, Config::BEND_ANCHOR_DEG);
  return true;
}

// ── Движение к целевому относительному углу ──────────────────────────────────
// Блокирующе шагает от текущего положения к targetRelDeg (угол от логического нуля),
// пока не окажется в пределах ENCODER_TARGET_WINDOW_DEG, затем подтверждает и
// останавливается. Направление каждого шага выбирается по ЗНАКУ ошибки угла
// энкодера, поэтому движение сходится к цели независимо от полярности проводки.
// Скорость задаётся stepIntervalUs. Детектор застревания: если за
// MOVE_STALL_TIMEOUT_MS угол не продвинулся к цели хотя бы на MOVE_MIN_PROGRESS_DEG,
// вал застрял -> возврат false (вызывающий поднимает ERROR).
//
// allowAbort: если true, движение прерывается (возврат false, *aborted=true) при
// поднятом gStopRequested - для плеч прогона, где STOP/HOME должны действовать
// немедленно. Если false (homing, вход в центр при загрузке / восстановлении),
// движение НЕ прерывается флагами: эти перемещения обязаны довестись до конца, а
// выполняются они в состояниях HOMING/RETURN_HOME (не RUNNING), поэтому прежняя
// проверка «state != RUNNING» ошибочно прерывала их сразу (вал оставался на 180 и
// не приезжал в центр). Возвращает true при достижении цели.
bool moveToRelDeg(float targetRelDeg, uint32_t stepIntervalUs, bool allowAbort, bool* aborted) {
  if (aborted) *aborted = false;

  gEncoder.update();
  float bestErr = fabsf(targetRelDeg - gEncoder.getRelativeDegrees());
  uint32_t lastProgressMs = millis();
  uint32_t safetyCounter = 0;
  uint8_t readErrors = 0;

  while (true) {
    // Дать поработать loop() (сеть/[DIAG]/команды) - он на том же ядре 1 с меньшим
    // приоритетом и без этого почти не получал процессор во время движения.
    vTaskDelay(1);
    // Опрос кнопки в прерываемых плечах прогона: нажатие в RUNNING поднимет
    // gStopRequested, и движение прервётся здесь же (кнопка работает во время цикла).
    if (allowAbort) {
      pollButton();
    }
    // Немедленная реакция на запрос останова (только для прерываемых плеч прогона).
    if (allowAbort && gStopRequested) {
      if (aborted) *aborted = true;
      gMotor.stop();
      return false;
    }

    if (!gEncoder.update()) {
      // Одиночные сбои I2C (помехи от драйвера) пропускаем; отказ - только серия подряд.
      if (++readErrors >= Config::ENCODER_READ_ERROR_LIMIT) {
        Serial.println("[ERROR] Encoder read failed during move");
        gMotor.stop();
        return false;
      }
      continue;
    }
    readErrors = 0;

    const float relDeg = gEncoder.getRelativeDegrees();
    const float errDeg = targetRelDeg - relDeg;
    const float absErr = fabsf(errDeg);

    updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), relDeg,
                        gMotor.getExpectedDegrees(), false, errDeg);

    if (absErr <= Config::ENCODER_TARGET_WINDOW_DEG) {
      gMotor.stop();
      // Подтверждение прихода несколькими отсчётами после короткого успокоения.
      delay(Config::MOVE_VERIFY_SETTLE_MS);
      bool confirmed = true;
      for (uint8_t i = 0; i < Config::MOVE_VERIFY_SAMPLES; ++i) {
        if (!gEncoder.update()) { confirmed = false; break; }
        if (fabsf(targetRelDeg - gEncoder.getRelativeDegrees()) > Config::ENCODER_TARGET_WINDOW_DEG) {
          confirmed = false;
          break;
        }
        delay(10);
      }
      if (confirmed) {
        return true;
      }
      continue;
    }

    // Детектор застревания: отслеживаем лучшее (наименьшее) отклонение; если оно не
    // улучшилось на MIN_PROGRESS_DEG за STALL_TIMEOUT_MS, вал не крутится -> отказ.
    if ((bestErr - absErr) >= Config::MOVE_MIN_PROGRESS_DEG) {
      bestErr = absErr;
      lastProgressMs = millis();
    } else if ((millis() - lastProgressMs) >= Config::MOVE_STALL_TIMEOUT_MS) {
      gMotor.stop();
      Serial.printf("[ERROR] Move stalled: no encoder progress (err=%.2f deg)\n", errDeg);
      return false;
    }

    // Защита по общему ходу.
    if (++safetyCounter > (Config::HOME_MAX_TRAVEL_STEPS / Config::HOME_STEP_CHUNK)) {
      gMotor.stop();
      Serial.println("[ERROR] Move travel limit exceeded");
      return false;
    }

    // Направление - по знаку ошибки, пересчитывается каждую итерацию: если вал
    // перескочил цель, знак ошибки меняется и следующий шаг идёт обратно, поэтому
    // движение самокорректируется без отдельной эвристики разворота. Размер шага
    // двухскоростной: вдали от цели - пачка (быстро), у цели - один шаг, чтобы
    // попасть в окно, а не перескочить его.
    const int32_t dirSign = (errDeg > 0.0f) ? 1 : -1;
    const int32_t chunk = (absErr > Config::MOVE_COARSE_BAND_DEG) ? Config::HOME_STEP_CHUNK : 1;
    gMotor.moveSteps(dirSign * chunk, stepIntervalUs);
  }
}

// Неблокирующая по отношению к сети/кнопке выдержка: удерживает мотор остановленным
// durationMs, но продолжает крутить цикл задачи, чтобы STOP/выход из RUNNING
// срабатывали во время выдержки. Возвращает false, если выдержка прервана
// (останов/смена состояния), true если отстояла полностью.
bool dwell(uint32_t durationMs) {
  const uint32_t startMs = millis();
  while ((millis() - startMs) < durationMs) {
    pollButton();  // опрос кнопки во время выдержки (RUNNING -> STOP)
    if (gStopRequested || getState() != SystemState::RUNNING) {
      return false;
    }
    gMotor.stop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

// ── Задача мотора (ядро 1): homing + автомат циклов ──────────────────────────
void motorTask(void* parameter) {
  (void)parameter;
  uint8_t encoderReadErrorCounter = 0;
  uint32_t lastWarnMs = 0;

  for (;;) {
    // Опрос кнопки в покоящихся состояниях (IDLE/ERROR): здесь motorTask крутится
    // каждые MOTOR_TASK_PERIOD_MS, поэтому нажатие ловится сразу. Во время прогона
    // опрос идёт из dwell()/moveToRelDeg(). Единый опросчик - только motorTask.
    pollButton();

    // Однократный boot homing, запрошенный из setup(). При успехе -> перевод в
    // центр 90° -> IDLE; при неудаче -> ERROR (восстанавливается кнопкой/сетью).
    if (gBootHomeRequested) {
      gBootHomeRequested = false;
      if (runHoming()) {
        // После захвата логического нуля (в bend 180°) перевести в центр 90°.
        // allowAbort=false: движение идёт в состоянии HOMING (не RUNNING) и обязано
        // довестись до центра, а не прерваться сразу.
        setState(SystemState::HOMING);
        if (moveToRelDeg(relCenter(), Config::HOME_STEP_INTERVAL_US, /*allowAbort=*/false, nullptr)) {
          setState(SystemState::IDLE);
          gHomeReferenceSteps = gMotor.getExpectedSteps();
          Serial.println("[STATE] IDLE (at center 90)");
        } else {
          setState(SystemState::ERROR);
          Serial.println("[ERROR] Move to center failed after homing");
          NetworkManager::queueEvent(buildEventJson("error", "center_move_failed").c_str());
        }
      } else {
        setState(SystemState::ERROR);
        Serial.println("[ERROR] Boot homing failed");
        NetworkManager::queueEvent(buildEventJson("error", "boot_home_failed").c_str());
      }
      continue;
    }

    // Повторный homing по запросу кнопки/сети (восстановление из ERROR или
    // идемпотентный home из IDLE). Логический ноль УЖЕ захвачен при загрузке и
    // не теряется (энкодер многооборотный), поэтому повторный homing НЕ ходит к
    // абсолютному анкеру (bend 180) и обратно - он просто едет ПРЯМО в центр 90 по
    // уже известному нулю. Это устраняет бессмысленный крюк 90->180->90 при home,
    // когда вал уже около центра. Выполняется здесь (motorTask владеет
    // энкодером/мотором).
    if (gRehomeRequested) {
      const SystemState rehomeFrom = getState();
      gRehomeRequested = false;
      if (rehomeFrom == SystemState::ERROR || rehomeFrom == SystemState::IDLE) {
        setState(SystemState::HOMING);
        if (moveToRelDeg(relCenter(), Config::HOME_STEP_INTERVAL_US, /*allowAbort=*/false, nullptr)) {
          setState(SystemState::IDLE);
          gHomeReferenceSteps = gMotor.getExpectedSteps();
          Serial.println("[STATE] IDLE (re-homed, at center 90)");
        } else {
          setState(SystemState::ERROR);
          Serial.println("[ERROR] Move to center failed during re-home");
          NetworkManager::queueEvent(buildEventJson("error", "center_move_failed").c_str());
        }
      } else {
        Serial.printf("[NET] re-home dropped: state changed to %s before service\n",
                      toString(rehomeFrom));
      }
      continue;
    }

    // Запрос старта прогона: только из IDLE.
    if (gStartRequested) {
      gStartRequested = false;
      gStopRequested = false;
      if (getState() == SystemState::IDLE) {
        runCycleProgram();
      } else {
        Serial.printf("[NET] START ignored in state %s\n", toString(getState()));
      }
      continue;
    }

    // Диагностика энкодера (чтение) в покое, чтобы телеметрия/[DIAG] жили в IDLE/ERROR.
    float encoderError = 0.0f;
    const bool fault = checkEncoderSlip(&encoderError);

    portENTER_CRITICAL(&gShared.mux);
    const bool encoderReady = gShared.encoderReady;
    portEXIT_CRITICAL(&gShared.mux);

    if (!encoderReady) {
      ++encoderReadErrorCounter;
      if (encoderReadErrorCounter >= Config::ENCODER_READ_ERROR_LIMIT) {
        setState(SystemState::ERROR);
        gMotor.stop();
        Serial.println("[ERROR] Encoder read timeout/failure");
        NetworkManager::queueEvent(buildEventJson("error", "encoder_read_timeout").c_str());
      }
    } else {
      encoderReadErrorCounter = 0;
    }

    if (fault && encoderReady) {
      if ((millis() - lastWarnMs) >= Config::WARN_THROTTLE_MS) {
        lastWarnMs = millis();
        Serial.printf("[WARN] Encoder mismatch %.2f deg\n", encoderError);
      }
    }

    // В покое мотор стоит.
    switch (getState()) {
      case SystemState::BOOT:
      case SystemState::HOMING:
      case SystemState::RETURN_HOME:
      case SystemState::ERROR:
      case SystemState::IDLE:
      case SystemState::RUNNING:  // прогон исполняется в runCycleProgram, сюда не попадает
        gMotor.stop();
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TASK_PERIOD_MS));
  }
}

// ── Исполнение программы циклов ──────────────────────────────────────────────
// Полностью проходит прогон из состояния IDLE: вход 90->0, затем N раз
// [0->180, выдержка, 180->0, выдержка], затем выход 0->90 -> IDLE. Любая неудача
// движения -> ERROR. STOP/HOME в процессе -> аккуратный возврат в центр -> IDLE.
// Выполняется целиком в motorTask (владелец мотора), поэтому это одна блокирующая
// процедура; сеть/кнопка/[DIAG] живут на ядре 0.
void runCycleProgram() {
  CycleConfig cfg;
  getConfigSnapshot(cfg);

  const float leftRel = relLeft(cfg.amplitudeDeg);
  const float rightRel = relRight(cfg.amplitudeDeg);
  const uint32_t target = cfg.cycles;
  const uint32_t speedUs = cfg.speedUs;
  const uint32_t dwellMs = cfg.dwellMs;

  setState(SystemState::RUNNING);
  setCycleProgress(0, target);
  Serial.printf("[STATE] RUNNING: %lu cycles, amp=%.1f, dwell=%lu ms, speed=%lu us\n",
                static_cast<unsigned long>(target), static_cast<double>(cfg.amplitudeDeg),
                static_cast<unsigned long>(dwellMs), static_cast<unsigned long>(speedUs));

  bool aborted = false;
  // Итог прогона: true - штатно завершён/остановлен (возврат в центр -> IDLE),
  // false - отказ движения (-> ERROR). Локальная лямбда сводит все выходы в одну точку.
  auto finishToCenter = [&]() {
    setState(SystemState::RETURN_HOME);
    setPhase(CyclePhase::TO_CENTER);
    Serial.println("[STATE] RETURN_HOME (to center 90)");
    // Возврат в центр НЕ прерывается флагом останова (мы уже завершаем): снимаем
    // gStopRequested и вызываем движение с allowAbort=false, чтобы оно довелось до
    // центра.
    gStopRequested = false;
    if (moveToRelDeg(relCenter(), speedUs, /*allowAbort=*/false, nullptr)) {
      setPhase(CyclePhase::IDLE);
      setState(SystemState::IDLE);
      Serial.println("[STATE] IDLE (back at center 90)");
    } else {
      setPhase(CyclePhase::IDLE);
      setState(SystemState::ERROR);
      Serial.println("[ERROR] Return to center failed");
      NetworkManager::queueEvent(buildEventJson("error", "return_center_failed").c_str());
    }
  };

  auto failToError = [&](const char* reason) {
    gMotor.stop();
    setPhase(CyclePhase::IDLE);
    setState(SystemState::ERROR);
    Serial.printf("[ERROR] Cycle program failed: %s\n", reason);
    NetworkManager::queueEvent(buildEventJson("error", reason).c_str());
  };

  // Вход: 90 -> 0 (левое). Не считается циклом.
  setPhase(CyclePhase::ENTER_LEFT);
  if (!moveToRelDeg(leftRel, speedUs, /*allowAbort=*/true, &aborted)) {
    if (aborted) { finishToCenter(); return; }
    failToError("enter_left_failed");
    return;
  }
  setPhase(CyclePhase::DWELL_LEFT);
  if (!dwell(dwellMs)) { finishToCenter(); return; }

  // N циклов: каждый = 0 -> 180 (выдержка) -> 180 -> 0 (выдержка).
  for (uint32_t i = 0; i < target; ++i) {
    setPhase(CyclePhase::TO_RIGHT);
    if (!moveToRelDeg(rightRel, speedUs, /*allowAbort=*/true, &aborted)) {
      if (aborted) { finishToCenter(); return; }
      failToError("to_right_failed");
      return;
    }
    setPhase(CyclePhase::DWELL_RIGHT);
    if (!dwell(dwellMs)) { finishToCenter(); return; }

    setPhase(CyclePhase::TO_LEFT);
    if (!moveToRelDeg(leftRel, speedUs, /*allowAbort=*/true, &aborted)) {
      if (aborted) { finishToCenter(); return; }
      failToError("to_left_failed");
      return;
    }
    // Приход в левое завершает цикл i.
    setCycleProgress(i + 1, target);
    char detail[24];
    snprintf(detail, sizeof(detail), "%lu/%lu", static_cast<unsigned long>(i + 1),
             static_cast<unsigned long>(target));
    NetworkManager::queueEvent(buildEventJson("cycle", detail).c_str());
    Serial.printf("[CYCLE] %s\n", detail);

    setPhase(CyclePhase::DWELL_LEFT);
    if (!dwell(dwellMs)) { finishToCenter(); return; }
  }

  // Все циклы выполнены штатно.
  NetworkManager::queueEvent(buildEventJson("done", "completed").c_str());
  Serial.println("[CYCLE] program completed");
  finishToCenter();
}

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup() {
  delay(250);
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32-S3 bend stand controller booting");
  Serial.println("[BOOT] Serial online");
  Serial.println("========================================");

  if (strcmp(Config::WIFI_SSID, "CHANGE_ME") == 0) {
    Serial.println("[BOOT][WARN] WiFi SSID is still the placeholder CHANGE_ME - network/dashboard will NOT connect.");
  }

  gButton.begin();
  Serial.println("[BOOT] Stepper driver power applied");
  gMotor.begin();
  Serial.printf("[BOOT] Waiting %lu ms for driver power settle\n", static_cast<unsigned long>(Config::DRIVER_POWER_SETTLE_MS));
  delay(Config::DRIVER_POWER_SETTLE_MS);

  const bool encoderOk = gEncoder.begin(Wire);
  if (!encoderOk) {
    setState(SystemState::ERROR);
    Serial.println("[ERROR] AS5600 init failed");
  } else {
    // Homing запрашивается, а выполняет его motorTask на ядре 1, поэтому setup()
    // возвращается быстро и сеть/[DIAG] поднимаются сразу, оставаясь активными во
    // время homing.
    setState(SystemState::HOMING);
    gBootHomeRequested = true;
    Serial.println("[BOOT] Homing requested (runs in motorTask; logs/WiFi stay live)");
  }

  // Загрузка сохранённого конфига циклов из NVS с откатом к умолчаниям Config.h.
  {
    CycleConfig defaults{Config::CYCLES_DEFAULT, Config::DWELL_MS_DEFAULT,
                         Config::SPEED_US_DEFAULT, Config::AMPLITUDE_DEG_DEFAULT};
    CycleConfig loaded;
    const bool persisted = ConfigStore::load(loaded, defaults);
    setConfig(loaded.cycles, loaded.dwellMs, loaded.speedUs, loaded.amplitudeDeg);
    if (!persisted) {
      ConfigStore::save(loaded);  // заполнить NVS, чтобы значения были стабильны далее
    }
    Serial.printf("[CFG] active cycles=%lu dwell=%lu speed=%lu amp=%.1f (%s)\n",
                  static_cast<unsigned long>(loaded.cycles), static_cast<unsigned long>(loaded.dwellMs),
                  static_cast<unsigned long>(loaded.speedUs), static_cast<double>(loaded.amplitudeDeg),
                  persisted ? "from NVS" : "defaults");
  }

  // motorTask (ядро 1) выполняет boot homing; loop() на ядре 0 сразу начинает
  // выдавать [DIAG] и прокачивать сеть.
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 6144, nullptr, 2, &gMotorTaskHandle, Config::MOTOR_TASK_CORE);

  NetworkManager::begin();
  Serial.println("[BOOT] NetworkManager started");
}

void loop() {
  // Кнопка опрашивается в motorTask (pollButton), а НЕ здесь: loop() и motorTask оба
  // на ядре 1, и блокирующий прогон почти не давал loop() выполниться, из-за чего
  // нажатие во время цикла терялось. Теперь loop() занимается только сетью и [DIAG].

  static uint32_t lastReportMs = 0;
  if ((millis() - lastReportMs) >= Config::SERIAL_REPORT_MS) {
    lastReportMs = millis();

    portENTER_CRITICAL(&gShared.mux);
    const SystemState state = gShared.state;
    const float absDeg = gShared.encoderAbsoluteDeg;
    const float relDeg = gShared.encoderRelativeDeg;
    const bool encFault = gShared.encoderFault;
    const float encErr = gShared.encoderErrorDeg;
    const uint32_t cyclesDone = gShared.cyclesDone;
    const uint32_t cyclesTarget = gShared.cyclesTarget;
    const CyclePhase phase = gShared.phase;
    portEXIT_CRITICAL(&gShared.mux);

    const bool netUp = NetworkManager::isWifiConnected();
    const String netIp = NetworkManager::wifiIp();
    const uint8_t wsClients = NetworkManager::wsClientCount();

    Serial.printf(
        "[DIAG] state=%s phase=%s abs=%.2f rel=%.2f bend=%.2f cyc=%lu/%lu enc_err=%.2f fault=%d net=%s ip=%s ws=%u\n",
        toString(state), toString(phase), absDeg, relDeg, relToBend(relDeg),
        static_cast<unsigned long>(cyclesDone), static_cast<unsigned long>(cyclesTarget),
        encErr, encFault ? 1 : 0, netUp ? "UP" : "DOWN", netIp.c_str(),
        static_cast<unsigned>(wsClients));
  }

  NetworkManager::loop();

  delay(5);
}

// ── Обработчик сетевых команд ────────────────────────────────────────────────
// Не-weak переопределение NetworkManager::onCommand. Управляет автоматом тремя
// командами:
//   START - запустить прогон циклов. Только из IDLE (иначе игнор).
//   STOP  - прервать прогон -> аккуратный возврат в центр -> IDLE. Только из RUNNING.
//   HOME  - из RUNNING эквивалент STOP; из ERROR/IDLE - повторный homing.
// Команды из несовместимого состояния логируются и игнорируются. Каждая команда
// порождает ровно одно событие "ack" ("<cmd>:ok" / "<cmd>:ignored_<STATE>").
void NetworkManager::onCommand(NetCommand::Command cmd) {
  Serial.printf("[NET] cmd=%s\n", NetCommand::commandName(cmd));

  const SystemState state = getState();
  const char* cmdName = NetCommand::commandName(cmd);
  auto ackAccepted = [&]() {
    char detail[48];
    snprintf(detail, sizeof(detail), "%s:ok", cmdName);
    NetworkManager::queueEvent(buildEventJson("ack", detail).c_str());
  };
  auto ackIgnored = [&]() {
    char detail[48];
    snprintf(detail, sizeof(detail), "%s:ignored_%s", cmdName, toString(state));
    NetworkManager::queueEvent(buildEventJson("ack", detail).c_str());
  };

  switch (cmd) {
    case NetCommand::Command::START:
      if (state == SystemState::IDLE) {
        gStartRequested = true;
        Serial.println("[STATE] START requested (net)");
        ackAccepted();
      } else {
        Serial.printf("[NET] START ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::STOP:
      if (state == SystemState::RUNNING) {
        gStopRequested = true;
        Serial.println("[STATE] STOP requested (net)");
        ackAccepted();
      } else {
        Serial.printf("[NET] STOP ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::HOME:
      if (state == SystemState::RUNNING) {
        gStopRequested = true;  // из прогона HOME эквивалентно STOP (возврат в центр)
        Serial.println("[STATE] STOP requested via HOME (net)");
        ackAccepted();
      } else if (state == SystemState::ERROR || state == SystemState::IDLE) {
        gRehomeRequested = true;
        Serial.println("[STATE] re-home requested (net)");
        ackAccepted();
      } else {
        Serial.printf("[NET] HOME ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::NONE:
    default:
      break;
  }
}
