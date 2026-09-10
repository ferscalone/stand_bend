#pragma once

#include <Arduino.h>
#include "SystemState.h"

// Единый согласованный снимок каждого поля телеметрии, которое отображает панель.
// .ino заполняет его в одной критической секции, так что все поля берутся из одного
// момента (без рваного чтения через отдельные блокируемые геттеры).
// buildTelemetryJson() формирует вывод из локальной копии этой структуры.
struct TelemetrySnapshot {
  SystemState state;
  bool encoderReady;
  float encAbsDeg;      // абсолютный угол AS5600
  float encRelDeg;      // относительный (от логического нуля) угол
  bool encoderFault;
  float bendAngleDeg;   // угол в bend-кадре: 180 + encRelDeg (0=лево, 90=центр, 180=право)
  uint32_t cyclesDone;  // завершённых полных циклов в текущем прогоне
  uint32_t cyclesTarget;// заданное число циклов
  // Текущая под-фаза прогона строкой ("IDLE"/"TO_LEFT"/"DWELL_LEFT"/"TO_RIGHT"/
  // "DWELL_RIGHT"/"TO_CENTER"). Указатель на статический литерал; копируется в JSON.
  const char* phase;
  // Действующий runtime-конфиг (перекрываемый через POST /v1/config, хранимый в NVS).
  uint32_t cfgCycles;
  uint32_t cfgDwellMs;
  uint32_t cfgSpeedUs;
  float cfgAmplitudeDeg;
};

// Атомарный доступ: копирует все поля телеметрии из разделяемого состояния под одной
// критической секцией. Определён в .ino (где размещена структура SharedData).
void getTelemetrySnapshot(TelemetrySnapshot& out);

// Формирует JSON-снимок телеметрии из текущего глобального состояния изгибного
// стенда. Строка собирается во внутренний статический буфер и возвращается по
// значению (String).
//
// Отдаваемые поля:
//   state           - текущее SystemState строкой ("IDLE", "RUNNING", ...)
//   phase           - под-фаза прогона строкой (см. выше)
//   bend_angle_deg  - угол в bend-кадре (0=лево, 90=центр/покой, 180=право)
//   encoder_abs_deg - абсолютный угол энкодера (град)
//   encoder_rel_deg - относительный (от логического нуля) угол энкодера (град)
//   encoder_fault   - 1, если установлена ошибка/сбой позиции энкодера
//   cycles_done     - завершённых полных циклов в текущем прогоне
//   cycles_target   - заданное число циклов
//   cfg_cycles / cfg_dwell_ms / cfg_speed_us / cfg_amplitude_deg - действующий конфиг
//   uptime_ms       - millis() с момента запуска
String buildTelemetryJson();

// Формирует компактный JSON-кадр дискретного события для потока WS, например
//   {"event":"state","detail":"RUNNING","uptime_ms":12345}
// `event` - короткий токен типа ("state"/"cycle"/"phase"/"done"/"error"/"ack"),
// `detail` - короткая заведомо безопасная нагрузка. Ни то, ни другое не
// экранируется - вызывающие передают только фиксированные короткие токены.
String buildEventJson(const char* event, const char* detail);
