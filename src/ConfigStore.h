#pragma once

#include <Arduino.h>

#include "Config.h"

// Единый согласованный набор параметров программы циклов, задаваемых оператором из
// веб-панели и сохраняемых в NVS. Отражает оперативное gConfig в .ino; используется
// загрузкой/сохранением ConfigStore и валидатором /v1/config.
//
//   cycles        - число полных изгибных циклов за прогон (0 -> 180 -> 0).
//   dwellMs       - выдержка в крайних точках 0° и 180° (мс).
//   speedUs       - интервал одного импульса шага при повороте (мкс/шаг; больше = медленнее).
//   amplitudeDeg  - половина размаха от центра 90° (90 -> полный ход 0..180).
struct CycleConfig {
  uint32_t cycles;
  uint32_t dwellMs;
  uint32_t speedUs;
  float amplitudeDeg;
};

// Проверяет параметры цикла. Возвращает nullptr при корректных значениях, иначе
// короткую статическую строку-причину, пригодную для "detail" ответа 400. Правила -
// только границы разумности из Config.h (без NaN/inf для float, диапазоны для
// целочисленных).
const char* validateConfig(uint32_t cycles, uint32_t dwellMs, uint32_t speedUs, float amplitudeDeg);

namespace ConfigStore {

// Загружает сохранённый конфиг в out. Если сохранённого корректного blob нет
// (отсутствует ключ / повреждён / не проходит validateConfig), out перезаписывается
// значениями `defaults` и возвращается false. Возвращает true только когда прочитан
// корректный полностью сохранённый конфиг.
bool load(CycleConfig& out, const CycleConfig& defaults);

// Сохраняет весь конфиг как единый blob (один nvs_commit). Вызывать только после
// того, как validateConfig прошла. Выполняется синхронно в служебной задаче
// AsyncTCP внутри обработчика POST; единый blob сокращает время блокировки.
void save(const CycleConfig& cfg);

}  // namespace ConfigStore
