#pragma once

#include <Arduino.h>

#if __has_include("Secrets.h")
#include "Secrets.h"
#else
#error "Net fajla src/Secrets.h: skopirujte src/Secrets.example.h v src/Secrets.h i vpishite WiFi"
#endif

namespace Config {

// ── Пины ─────────────────────────────────────────────────────────────────────
// Те же, что на нагрузочном стенде: аппаратная часть идентична (драйвер шагового
// двигателя по step/dir, магнитный энкодер AS5600 по I2C, кнопка). Тензодатчик
// HX711 в этом стенде ОТСУТСТВУЕТ, поэтому его пины здесь не заданы.
constexpr uint8_t PIN_STEP_P = 4;
constexpr uint8_t PIN_STEP_N = 5;
constexpr uint8_t PIN_DIR_P = 6;
constexpr uint8_t PIN_DIR_N = 7;

constexpr uint8_t PIN_AS5600_SCL = 17;
constexpr uint8_t PIN_AS5600_SDA = 18;

constexpr uint8_t PIN_BUTTON = 19;

// ── Двигатель / энкодер ──────────────────────────────────────────────────────
constexpr uint16_t MOTOR_MICROSTEP = 4;
constexpr uint16_t MOTOR_PULSES_PER_REV = 800;
constexpr float ENCODER_COUNTS_PER_REV = 4096.0f;
constexpr float DEGREES_PER_REV = 360.0f;
constexpr float DEGREES_PER_STEP = DEGREES_PER_REV / static_cast<float>(MOTOR_PULSES_PER_REV);
constexpr float ENCODER_COUNTS_PER_STEP = ENCODER_COUNTS_PER_REV / static_cast<float>(MOTOR_PULSES_PER_REV);

// Знак отображения bend-кадра на относительный угол энкодера. Управление ведётся
// ПО ЭНКОДЕРУ (см. moveToRelDeg): знак каждого шага выбирается по знаку ошибки угла
// энкодера, поэтому движение ВСЕГДА сходится к цели - НЕЛЬЗЯ менять физическое
// направление инверсией линии DIR (это разомкнуло бы обратную связь: шаг уводил бы
// угол ОТ цели -> разгон в стенку / вечная дрожь). Правильный способ развернуть
// сторону размаха - поменять, по какую сторону от анкера лежат целевые углы:
//   bend = BEND_ANCHOR_DEG + BEND_DIR_SIGN * rel   <=>   rel = SIGN * (bend - 180).
//   +1: центр 90° = rel -90 (размах идёт в «-», физически «снизу» на этом стенде);
//   -1: центр 90° = rel +90 (размах идёт в «+», физически «сверху» / против часовой).
// Обе полярности сходятся к цели корректно; флаг лишь выбирает сторону дуги. Если
// вал уходит не в ту сторону - переключить знак.
constexpr float BEND_DIR_SIGN = -1.0f;

// ── Геометрия изгиба (bend-кадр) ─────────────────────────────────────────────
// Стенд гнёт кабель поворотом ротора. Рабочая разметка угла ведётся в «bend-кадре»:
//   0°   - крайнее ЛЕВОЕ положение,
//   90°  - ВЕРХ / центр / положение покоя (домашняя позиция цикла),
//   180° - крайнее ПРАВОЕ положение.
// Полный размах одного цикла: 0 -> 180 -> 0 (через центр 90).
constexpr float BEND_CENTER_DEG = 90.0f;   // «вверх», положение покоя и цель homing
constexpr float BEND_LEFT_DEG = 0.0f;      // крайнее левое
constexpr float BEND_RIGHT_DEG = 180.0f;   // крайнее правое

// Привязка bend-кадра к энкодеру. Логический ноль (позиция, захваченная при
// homing на абсолютном угле HOME_ABSOLUTE_ANGLE_DEG) СООТВЕТСТВУЕТ bend 180°:
//   bend_angle = 180 + rel_deg,   где rel_deg - угол от логического нуля.
// Значит центр 90° = rel -90°, левое 0° = rel -180°, правое 180° = rel 0°.
// Весь ход лежит по «-» сторону от анкера (rel <= 0), поэтому механическая стенка
// нагрузочного стенда (~150-160° абс., ВЫШЕ анкера) не задевается. См. bendToRel().
constexpr float BEND_ANCHOR_DEG = 180.0f;  // bend-угол, соответствующий rel 0 (логический ноль)

// Абсолютный угол AS5600, принимаемый за логический ноль (= bend 180°). На стенде,
// где идёт отладка (без второго физического стенда), за это берётся тот же анкер,
// что и у нагрузочного стенда: 106.08° абс. Homing приводит вал в эту точку и
// захватывает логический ноль. Обновить, если механическая домашняя позиция
// переопределена.
constexpr float HOME_ABSOLUTE_ANGLE_DEG = 106.08f;

// ── Параметры цикла ПО УМОЛЧАНИЮ (перекрываются runtime-конфигом из NVS) ──────
// Один ЦИКЛ - это полный размах 0 -> 180 -> 0. Вход 90 -> 0 в начале прогона и
// выход 0 -> 90 в конце в счёт циклов НЕ входят.
constexpr uint32_t CYCLES_DEFAULT = 10;      // сколько циклов за прогон
// Выдержка в крайних точках (0° и 180°): ротор стоит это время перед разворотом.
constexpr uint32_t DWELL_MS_DEFAULT = 1000;
// Скорость поворота: интервал одного импульса шага (мкс/шаг). Больше -> медленнее.
// Было 1200 мкс (~830 шаг/с, старт без разгона) - шаговый двигатель на таком темпе
// с места срывается/пропускает шаги, и детектор застревания уводил стенд в ERROR.
// Рабочий нагрузочный стенд на том же железе никогда не шагает быстрее 9000 мкс.
// 4000 мкс = ~112 °/с (ход 0->180 примерно за 1.6 с). Ускорять из панели осторожно.
constexpr uint32_t SPEED_US_DEFAULT = 4000;
// Амплитуда: половина размаха от центра. 90 -> полный ход 0..180. Для безопасного
// теста на стенде можно уменьшить (например, 30 -> размах 60..120).
constexpr float AMPLITUDE_DEG_DEFAULT = 90.0f;

// Границы разумности runtime-конфига (проверяются в validateConfig). Верхние
// пределы - защита от абсурдных значений, а не жёсткое требование механики.
constexpr uint32_t CYCLES_MIN = 1;
constexpr uint32_t CYCLES_MAX = 1000000;
constexpr uint32_t DWELL_MS_MIN = 0;
constexpr uint32_t DWELL_MS_MAX = 600000;    // 10 минут
constexpr uint32_t SPEED_US_MIN = 1000;      // быстрее рискует пропуском шагов (разгона нет)
constexpr uint32_t SPEED_US_MAX = 60000;
constexpr float AMPLITUDE_DEG_MIN = 5.0f;
constexpr float AMPLITUDE_DEG_MAX = 90.0f;   // центр 90 +- 90 = ход 0..180

// ── Homing / возврат домой ───────────────────────────────────────────────────
// Движение всегда идёт короткими пачками шагов, чтобы после каждой сверяться с
// энкодером (единственный источник истины по положению).
// Движение к целевому углу двухскоростное (грубо/точно):
//   * ВДАЛИ от цели (|ошибка| > MOVE_COARSE_BAND_DEG) шаги идут пачками
//     HOME_STEP_CHUNK - быстро, меньше I2C-чтений на градус;
//   * ВБЛИЗИ цели - по ОДНОМУ шагу, чтобы попасть в окно ENCODER_TARGET_WINDOW_DEG.
// Это критично: пачка из HOME_STEP_CHUNK шагов даёт HOME_STEP_CHUNK*DEGREES_PER_STEP
// градусов за раз; если это БОЛЬШЕ окна, вал перескакивает окно и колеблется вокруг
// цели, а детектор застревания ошибочно уходит в ERROR (наблюдалось: вал замирал в
// ~4-5° от центра). Одиночный шаг у цели гарантирует попадание в окно.
constexpr int32_t HOME_STEP_CHUNK = 8;
// Порог перехода с грубого хода на точный (по одному шагу). Заметно больше пачки
// HOME_STEP_CHUNK*DEGREES_PER_STEP, чтобы торможение начиналось до окна.
constexpr float MOVE_COARSE_BAND_DEG = 6.0f;
// Интервал импульса при homing/переходах в центр. Намеренно медленный (было 1200 мкс -
// двигатель срывался при homing сразу после загрузки).
constexpr uint32_t HOME_STEP_INTERVAL_US = 5000;

// Автоопределение полярности DIR при загрузке (перед homing): мотор делает
// DIR_PROBE_STEPS шагов «вперёд», и по знаку изменения угла энкодера прошивка сама
// решает, нужно ли инвертировать линию DIR. Управление замкнуто по энкодеру, и при
// обратной полярности (другая распайка мотора / магнит энкодера с другой стороны)
// вал уходил бы ОТ цели -> «Move stalled» -> ERROR. Если за пробу вал сдвинулся
// меньше DIR_PROBE_MIN_DEG - мотор не крутится (питание/драйвер/пины), homing
// прерывается с понятной ошибкой в Serial.
constexpr int32_t DIR_PROBE_STEPS = 20;      // 20 * 0.45° = 9°
constexpr float DIR_PROBE_MIN_DEG = 3.0f;

constexpr uint32_t DRIVER_POWER_SETTLE_MS = 800;
constexpr uint32_t HOMING_SETTLE_MS = 400;
constexpr uint32_t HOMING_VERIFY_SETTLE_MS = 250;
constexpr uint8_t HOMING_VERIFY_SAMPLES = 5;
constexpr uint32_t MOVE_VERIFY_SETTLE_MS = 150;
constexpr uint8_t MOVE_VERIFY_SAMPLES = 3;
constexpr uint32_t HOME_MAX_TRAVEL_STEPS = 6400;

// Детектор застревания при движении к целевому углу: если за это время угол
// энкодера не продвинулся к цели хотя бы на MIN_PROGRESS_DEG, вал застрял/
// проскальзывает -> ERROR. Заменяет детекторы стенки нагрузочного стенда: здесь
// нет нагрузки, поэтому единственный отказ - «вал не крутится, как ожидалось».
constexpr uint32_t MOVE_STALL_TIMEOUT_MS = 1500;
constexpr float MOVE_MIN_PROGRESS_DEG = 0.8f;

// Окно, в пределах которого целевой угол считается достигнутым. Заметно больше
// одиночного шага (DEGREES_PER_STEP ~0.45°), чтобы точная фаза гарантированно в него
// попадала, но достаточно мало для нужной точности позиционирования.
constexpr float ENCODER_TARGET_WINDOW_DEG = 1.5f;
constexpr float ENCODER_HOME_WINDOW_DEG = 1.5f;

// ── Диагностика энкодера ─────────────────────────────────────────────────────
// Рассогласование ожидаемого (по шагам) и фактического (по энкодеру) угла.
// Во время управляемого движения это лишь предупреждение; жёсткий отказ по
// положению ловит детектор застревания выше.
constexpr float ENCODER_POSITION_ERROR_DEG = 30.0f;
constexpr uint8_t ENCODER_ERROR_LIMIT = 20;
constexpr uint8_t ENCODER_READ_ERROR_LIMIT = 5;
// Повторы одного чтения AS5600 внутри EncoderTracker::update(). Драйвер шагового
// двигателя наводит помехи на I2C; одиночный сбой чтения не должен сразу ронять
// движение в ERROR (рабочая прошивка ошибки I2C вообще игнорирует).
constexpr uint8_t ENCODER_READ_RETRIES = 3;

// ── Тайминги задач / кнопки / отчётов ────────────────────────────────────────
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t SERIAL_REPORT_MS = 500;
constexpr uint32_t WARN_THROTTLE_MS = 1500;
constexpr uint32_t MOTOR_TASK_PERIOD_MS = 2;

constexpr BaseType_t MOTOR_TASK_CORE = 1;

constexpr uint32_t BUTTON_QUEUE_LENGTH = 8;

// ── Сеть / WiFi / дашборд ────────────────────────────────────────────────────
// Учётные данные WiFi лежат в src/Secrets.h - этот файл не хранится в git (см.
// .gitignore). Шаблон: src/Secrets.example.h.
constexpr const char* WIFI_SSID = Secrets::WIFI_SSID;
constexpr const char* WIFI_PASSWORD = Secrets::WIFI_PASSWORD;
// Имя хоста mDNS БЕЗ суффикса ".local" - MDNS.begin() добавляет ".local" сам.
// Число в начале - номер стенда, "bend" - тип стенда (изгиб). Итоговый домен:
// 1.bend.esp32-s3.local.
constexpr const char* NET_HOSTNAME = "1.bend.esp32-s3";
constexpr uint16_t NET_HTTP_PORT = 80;
constexpr const char* NET_WS_PATH = "/ws";
constexpr uint32_t NET_TELEMETRY_INTERVAL_MS = 200;
constexpr uint8_t NET_CMD_QUEUE_SIZE = 8;
constexpr uint16_t NET_WS_BUFFER_SIZE = 512;
constexpr uint8_t NET_MAX_WS_CLIENTS = 4;

// ── Стабильная адресация ─────────────────────────────────────────────────────
// ПРЕДПОЧТИТЕЛЬНЫЙ путь к предсказуемому адресу: MAC STA у ESP32 стабилен (берётся
// из заводского eFuse), поэтому нужно зарезервировать IP для этого MAC в настройках
// DHCP точки доступа/роутера. MAC печатается один раз при загрузке. Работает в
// любой сети без жёсткого задания адреса.
//
// Статический IP ниже - НЕОБЯЗАТЕЛЬНЫЙ запасной вариант для сетей без
// DHCP-резервирования. По умолчанию ОТКЛЮЧЁН, чтобы жёстко заданный адрес,
// занятый в LAN, не вызвал тихий конфликт. При включении WiFi.config()
// применяется перед WiFi.begin(); при некорректных значениях код пишет
// предупреждение и откатывается на DHCP.
constexpr bool NET_USE_STATIC_IP = false;             // держать false -> DHCP (резервировать по MAC)
constexpr const char* NET_STATIC_IP = "192.168.1.51"; // используется только если NET_USE_STATIC_IP
constexpr const char* NET_STATIC_GATEWAY = "192.168.1.1";
constexpr const char* NET_STATIC_SUBNET = "255.255.255.0";
constexpr const char* NET_STATIC_DNS = "192.168.1.1"; // обычно это шлюз

}  // namespace Config
