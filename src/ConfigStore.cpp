#include "ConfigStore.h"

#include <Preferences.h>
#include <math.h>

// Пространство имён + ключ NVS (<= 15 символов). Весь конфиг хранится под ОДНИМ
// ключом как упакованный blob, поэтому сохранение - это один nvs_commit вместо
// нескольких; это важно, потому что save() выполняется синхронно в служебной задаче
// AsyncTCP внутри обработчика POST.
namespace {
constexpr const char* kNamespace = "bendcfg";
constexpr const char* kKeyBlob = "cfg";

// Раскладка blob во flash. Обычный POD, копируется memcpy в/из байтов NVS. Приём с
// проверкой РАЗМЕРА (см. load()) отличает корректный blob от отсутствующего/
// повреждённого. НИКОГДА не переставлять поля - при добавлении новых полей в конец
// приём по размеру сохранит совместимость со старым blob.
struct StoredConfig {
  uint32_t cycles;
  uint32_t dwellMs;
  uint32_t speedUs;
  float amplitudeDeg;
};
}  // namespace

const char* validateConfig(uint32_t cycles, uint32_t dwellMs, uint32_t speedUs, float amplitudeDeg) {
  if (isnan(amplitudeDeg) || isinf(amplitudeDeg)) return "amplitude nan/inf";
  if (cycles < Config::CYCLES_MIN || cycles > Config::CYCLES_MAX) return "cycles out of range";
  if (dwellMs > Config::DWELL_MS_MAX) return "dwell out of range";
  if (speedUs < Config::SPEED_US_MIN || speedUs > Config::SPEED_US_MAX) return "speed out of range";
  if (amplitudeDeg < Config::AMPLITUDE_DEG_MIN || amplitudeDeg > Config::AMPLITUDE_DEG_MAX) {
    return "amplitude out of range";
  }
  return nullptr;
}

namespace ConfigStore {

bool load(CycleConfig& out, const CycleConfig& defaults) {
  Preferences prefs;
  prefs.begin(kNamespace, /*readOnly=*/true);
  StoredConfig blob{};
  const size_t got = prefs.getBytes(kKeyBlob, &blob, sizeof(blob));
  prefs.end();

  // Признаётся только blob точного текущего размера. Отсутствующий ключ (got==0),
  // усечённый или больший blob из будущего считаются «не сохранённым».
  if (got != sizeof(StoredConfig)) {
    out = defaults;
    return false;
  }

  out.cycles = blob.cycles;
  out.dwellMs = blob.dwellMs;
  out.speedUs = blob.speedUs;
  out.amplitudeDeg = blob.amplitudeDeg;

  // Отклонение повреждённого сохранённого конфига: откат к значениям по умолчанию,
  // чтобы плохой blob NVS никогда не заклинил прогон в невозможных параметрах.
  if (validateConfig(out.cycles, out.dwellMs, out.speedUs, out.amplitudeDeg) != nullptr) {
    Serial.println("[CFG][WARN] persisted config invalid, using defaults");
    out = defaults;
    return false;
  }
  return true;
}

void save(const CycleConfig& cfg) {
  // ВНИМАНИЕ: выполняется синхронно в служебной задаче AsyncTCP (внутри обработчика
  // POST /v1/config). putBytes делает ОДИН nvs_commit и блокирует эту задачу на
  // ~десятки мс - намеренно допустимо для малонагруженной локальной панели. Не
  // вызывать на горячем пути.
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    Serial.println("[CFG][WARN] NVS open failed, not persisting");
    return;
  }
  StoredConfig blob{cfg.cycles, cfg.dwellMs, cfg.speedUs, cfg.amplitudeDeg};
  prefs.putBytes(kKeyBlob, &blob, sizeof(blob));
  prefs.end();
  Serial.printf("[CFG] persisted cycles=%lu dwell=%lu speed=%lu amp=%.1f\n",
                static_cast<unsigned long>(cfg.cycles), static_cast<unsigned long>(cfg.dwellMs),
                static_cast<unsigned long>(cfg.speedUs), static_cast<double>(cfg.amplitudeDeg));
}

}  // namespace ConfigStore
