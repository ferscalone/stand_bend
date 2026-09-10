#pragma once

enum class SystemState : uint8_t {
  BOOT,
  HOMING,       // приведение вала к домашней позиции (bend 90°) и захват логического нуля
  IDLE,         // стоит в центре 90°, ждёт запуска
  RUNNING,      // выполняется программа изгибных циклов (см. CyclePhase в .ino)
  RETURN_HOME,  // возврат в центр 90° по завершении / остановке прогона
  ERROR
};

inline const char* toString(SystemState state) {
  switch (state) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::HOMING: return "HOMING";
    case SystemState::IDLE: return "IDLE";
    case SystemState::RUNNING: return "RUNNING";
    case SystemState::RETURN_HOME: return "RETURN_HOME";
    case SystemState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}
