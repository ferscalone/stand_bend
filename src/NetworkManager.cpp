#include "NetworkManager.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

#include "Config.h"
#include "ConfigStore.h"
#include "NetCommand.h"
#include "Telemetry.h"

// Определено в .ino (владеет gConfig). Атомарно записывает оперативный конфиг циклов.
extern void setConfig(uint32_t cycles, uint32_t dwellMs, uint32_t speedUs, float amplitudeDeg);

namespace {

AsyncWebServer sServer(Config::NET_HTTP_PORT);
AsyncWebSocket sWs(Config::NET_WS_PATH);

QueueHandle_t sCmdQueue = nullptr;
uint32_t sLastTelemetryMs = 0;
bool sMdnsStarted = false;

// Если связь STA остаётся разорванной дольше этого времени, сторож в loop() заново
// запускает WiFi.begin(). Также это минимальный интервал между попытками
// переподключения.
constexpr uint32_t kWifiReconnectTimeoutMs = 10000;

// Очередь широковещательной рассылки событий по возможности. queueEvent() (любая
// задача) копирует строку JSON в один из этих фиксированных слотов; loop() (сетевая
// задача) вычерпывает их и выполняет собственно sWs.textAll().
struct EventMsg {
  char json[160];
};
QueueHandle_t sEventQueue = nullptr;

// Применяет необязательный статический IP из Config перед WiFi.begin(). Возвращает
// false (и пишет в лог), если какая-либо адресная строка некорректна или
// WiFi.config() отвергнут, чтобы вызывающий откатился на DHCP.
bool applyStaticIpIfEnabled() {
  if (!Config::NET_USE_STATIC_IP) {
    return false;
  }
  IPAddress ip, gw, mask, dns;
  if (!ip.fromString(Config::NET_STATIC_IP) ||
      !gw.fromString(Config::NET_STATIC_GATEWAY) ||
      !mask.fromString(Config::NET_STATIC_SUBNET) ||
      !dns.fromString(Config::NET_STATIC_DNS)) {
    Serial.println("[NET][WARN] static IP config malformed, falling back to DHCP");
    return false;
  }
  if (!WiFi.config(ip, gw, mask, dns)) {
    Serial.println("[NET][WARN] WiFi.config() rejected static IP, falling back to DHCP");
    return false;
  }
  Serial.printf("[NET] static IP configured: %s (gw %s)\n",
                Config::NET_STATIC_IP, Config::NET_STATIC_GATEWAY);
  return true;
}

// Обработчик событий WiFi. Переподключение по событию - основной путь
// восстановления: SDK выдаёт STA_DISCONNECTED в момент разрыва связи, поэтому
// переассоциация происходит сразу. STA_GOT_IP сбрасывает флаг mDNS, чтобы ответчик
// заново анонсировался на свежей связи. Держать неблокирующим.
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[NET] WiFi got IP %s\n", WiFi.localIP().toString().c_str());
      sMdnsStarted = false;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[NET] WiFi disconnected (reason %u), reconnecting\n",
                    info.wifi_sta_disconnected.reason);
      WiFi.reconnect();
      break;
    default:
      break;
  }
}

// Неблокирующая постановка разобранной команды в общую очередь. Возвращает false для
// NONE (нераспознано) или если очередь полна/отсутствует. И обработчик WS, и
// эндпойнт REST /v1/command проходят через это.
bool enqueueCommand(NetCommand::Command cmd) {
  if (cmd == NetCommand::Command::NONE) {
    return false;
  }
  if (!sCmdQueue) {
    return false;
  }
  const NetCommand::Command qCmd = cmd;
  return xQueueSend(sCmdQueue, &qCmd, 0) == pdTRUE;
}

// Отправляет тело JSON с заголовком Access-Control-Allow-Origin:*, чтобы дашборд с
// другого источника мог обращаться к /v1/* через fetch().
void sendJsonCors(AsyncWebServerRequest* request, int code, const String& body) {
  AsyncWebServerResponse* r = request->beginResponse(code, "application/json", body);
  r->addHeader("Access-Control-Allow-Origin", "*");
  request->send(r);
}

// WS onEvent направляет входящие текстовые кадры сюда. Разбор через
// NetCommand::parseCommand, результат ставится в sCmdQueue. Результаты NONE молча
// отбрасываются.
void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)server;
  (void)arg;

  switch (type) {
    case WS_EVT_CONNECT: {
      if (sWs.count() > Config::NET_MAX_WS_CLIENTS) {
        client->close();
      }
      break;
    }
    case WS_EVT_DISCONNECT:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;

    case WS_EVT_DATA: {
      const AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
      if (!info || !info->final || info->opcode != WS_TEXT) {
        break;
      }
      char buf[64];
      const size_t copyLen = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
      memcpy(buf, data, copyLen);
      buf[copyLen] = '\0';

      // Обрезка хвостовых CR/LF/пробелов, чтобы "start\n" из терминала тоже работал.
      size_t end = copyLen;
      while (end > 0 && (buf[end - 1] == '\r' || buf[end - 1] == '\n' ||
                         buf[end - 1] == ' ')) {
        buf[--end] = '\0';
      }

      const NetCommand::Command parsed = NetCommand::parseCommand(buf);
      if (parsed != NetCommand::Command::NONE && !enqueueCommand(parsed)) {
        Serial.println("[NET] cmd queue full, dropped");
      }
      break;
    }
  }
}

}  // namespace

void NetworkManager::begin() {
  sCmdQueue = xQueueCreate(Config::NET_CMD_QUEUE_SIZE, sizeof(NetCommand::Command));
  sEventQueue = xQueueCreate(8, sizeof(EventMsg));

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWifiEvent);

  WiFi.setHostname(Config::NET_HOSTNAME);
  Serial.printf("[NET] STA MAC %s (reserve a DHCP lease for this MAC)\n",
                WiFi.macAddress().c_str());

  applyStaticIpIfEnabled();

  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

  sWs.onEvent(handleWsEvent);
  sServer.addHandler(&sWs);

  // Минимальный корень-заглушка: подтверждает, что сервер жив.
  sServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Bend Stand API");
  });

  // ── REST API (v1) ──────────────────────────────────────────────────────────

  // GET /v1/health - дешёвая проба живости.
  sServer.on("/v1/health", HTTP_GET, [](AsyncWebServerRequest* request) {
    String body = "{\"status\":\"ok\",\"uptime_ms\":";
    body += String(millis());
    body += "}";
    sendJsonCors(request, 200, body);
  });

  // GET /v1/status - полный снимок телеметрии (тот же JSON, что шлёт поток WS).
  sServer.on("/v1/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    sendJsonCors(request, 200, buildTelemetryJson());
  });

  // POST /v1/command - принимает команду из параметра "cmd" (query ?cmd=start или
  // тело form-urlencoded cmd=start). Обработчика сырого тела намеренно НЕТ (иначе
  // спрятанная в _tempObject String утекала бы буфером кучи на каждом запросе).
  sServer.on(
      "/v1/command", HTTP_POST, [](AsyncWebServerRequest* request) {
        String value;
        if (request->hasParam("cmd")) {
          value = request->getParam("cmd")->value();
        } else if (request->hasParam("cmd", true)) {
          value = request->getParam("cmd", true)->value();
        } else {
          sendJsonCors(request, 400, "{\"error\":\"missing cmd\"}");
          return;
        }
        value.trim();

        const NetCommand::Command cmd = NetCommand::parseCommand(value.c_str());
        if (cmd == NetCommand::Command::NONE) {
          sendJsonCors(request, 400, "{\"error\":\"unknown command\"}");
          return;
        }
        if (!enqueueCommand(cmd)) {
          sendJsonCors(request, 503, "{\"error\":\"queue full\"}");
          return;
        }
        String body = "{\"accepted\":\"";
        body += NetCommand::commandName(cmd);
        body += "\"}";
        sendJsonCors(request, 200, body);
      });

  sServer.on("/v1/command", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* r = request->beginResponse(204);
    r->addHeader("Access-Control-Allow-Origin", "*");
    r->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(r);
  });

  // POST /v1/config - задаёт параметры программы циклов во время работы. Параметры
  // ТОЛЬКО через query (?cycles=..&dwell_ms=..&speed_us=..&amplitude_deg=..) или
  // form-urlencoded; обработчика сырого тела НЕТ (во избежание утечки кучи).
  // Требуются все четыре. Проверяется validateConfig. При успехе: атомарно
  // применяется к gConfig + сохраняется в NVS.
  sServer.on("/v1/config", HTTP_POST, [](AsyncWebServerRequest* request) {
    auto getParam = [request](const char* name) -> const AsyncWebParameter* {
      if (request->hasParam(name)) return request->getParam(name);
      if (request->hasParam(name, true)) return request->getParam(name, true);
      return nullptr;
    };
    // Разбор целого без знака: отвергает пустые токены и токены с мусором на конце.
    auto parseU32 = [](const AsyncWebParameter* p, uint32_t* out) -> bool {
      const String& v = p->value();
      char* endp = nullptr;
      const unsigned long n = strtoul(v.c_str(), &endp, 10);
      if (endp == v.c_str() || *endp != '\0') return false;
      *out = static_cast<uint32_t>(n);
      return true;
    };
    auto parseF = [](const AsyncWebParameter* p, float* out) -> bool {
      const String& v = p->value();
      char* endp = nullptr;
      const float f = strtof(v.c_str(), &endp);
      if (endp == v.c_str() || *endp != '\0') return false;
      *out = f;
      return true;
    };

    const AsyncWebParameter* pc = getParam("cycles");
    const AsyncWebParameter* pd = getParam("dwell_ms");
    const AsyncWebParameter* ps = getParam("speed_us");
    const AsyncWebParameter* pa = getParam("amplitude_deg");
    if (!pc || !pd || !ps || !pa) {
      sendJsonCors(request, 400, "{\"error\":\"missing param\"}");
      return;
    }
    uint32_t cycles, dwellMs, speedUs;
    float amplitudeDeg;
    if (!parseU32(pc, &cycles) || !parseU32(pd, &dwellMs) ||
        !parseU32(ps, &speedUs) || !parseF(pa, &amplitudeDeg)) {
      sendJsonCors(request, 400, "{\"error\":\"bad param\"}");
      return;
    }

    const char* reason = validateConfig(cycles, dwellMs, speedUs, amplitudeDeg);
    if (reason != nullptr) {
      String body = "{\"error\":\"invalid\",\"detail\":\"";
      body += reason;
      body += "\"}";
      sendJsonCors(request, 400, body);
      return;
    }

    setConfig(cycles, dwellMs, speedUs, amplitudeDeg);
    CycleConfig applied{cycles, dwellMs, speedUs, amplitudeDeg};
    ConfigStore::save(applied);

    char body[160];
    snprintf(body, sizeof(body),
             "{\"applied\":{\"cycles\":%lu,\"dwell_ms\":%lu,\"speed_us\":%lu,\"amplitude_deg\":%.2f}}",
             static_cast<unsigned long>(cycles), static_cast<unsigned long>(dwellMs),
             static_cast<unsigned long>(speedUs), static_cast<double>(amplitudeDeg));
    sendJsonCors(request, 200, body);
  });

  sServer.on("/v1/config", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* r = request->beginResponse(204);
    r->addHeader("Access-Control-Allow-Origin", "*");
    r->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(r);
  });

  sServer.begin();
}

void NetworkManager::loop() {
  // Вычерпать поставленные в очередь команды и разослать их. Ограниченный цикл, не
  // более одной глубины очереди за вызов.
  if (sCmdQueue) {
    NetCommand::Command cmd;
    UBaseType_t remaining = uxQueueMessagesWaiting(sCmdQueue);
    while (remaining-- > 0) {
      if (xQueueReceive(sCmdQueue, &cmd, 0) == pdTRUE) {
        onCommand(cmd);
      }
    }
  }

  // Сторож переподключения WiFi - МЕДЛЕННАЯ ПОДСТРАХОВКА за onWifiEvent().
  static uint32_t sLastWifiOkMs = 0;
  static bool sWasConnected = false;
  const uint32_t nowMs = millis();

  if (WiFi.status() == WL_CONNECTED) {
    sLastWifiOkMs = nowMs;
    if (!sWasConnected) {
      sMdnsStarted = false;
      sWasConnected = true;
    }
  } else {
    sWasConnected = false;
    if (nowMs - sLastWifiOkMs >= kWifiReconnectTimeoutMs) {
      Serial.println("[NET] WiFi down, re-driving begin()");
      WiFi.disconnect();
      WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
      sLastWifiOkMs = nowMs;
    }
    return;
  }

  // Запустить mDNS теперь, когда WiFi поднят (WiFi.begin() неблокирующий, поэтому в
  // begin() этого сделать нельзя). Защёлкивать sMdnsStarted только при УСПЕХЕ.
  if (!sMdnsStarted) {
    MDNS.end();
    if (MDNS.begin(Config::NET_HOSTNAME)) {
      MDNS.addService("http", "tcp", Config::NET_HTTP_PORT);
      Serial.printf("[NetManager] mDNS responder started: http://%s.local\n",
                    Config::NET_HOSTNAME);
      sMdnsStarted = true;
    } else {
      Serial.println("[NetManager] mDNS start failed, will retry");
    }
  }

  // Вычерпать поставленные в очередь события всем клиентам WS. Делается здесь
  // (сетевая задача), чтобы задача двигателя никогда не платила стоимость мьютекса +
  // кучи у textAll.
  if (sEventQueue && sWs.count() > 0) {
    EventMsg msg;
    UBaseType_t evRemaining = uxQueueMessagesWaiting(sEventQueue);
    while (evRemaining-- > 0) {
      if (xQueueReceive(sEventQueue, &msg, 0) == pdTRUE) {
        sWs.textAll(msg.json);
      }
    }
  }

  if (sWs.count() == 0) {
    return;
  }
  const uint32_t now = millis();
  if (now - sLastTelemetryMs < Config::NET_TELEMETRY_INTERVAL_MS) {
    return;
  }
  sLastTelemetryMs = now;

  const String json = buildTelemetryJson();
  if (json.length() > 0) {
    sWs.textAll(json);
  }
}

void NetworkManager::queueEvent(const char* json) {
  // Безопасно для задач, неблокирующе. Скопировать JSON в фиксированный слот и
  // поставить в очередь; собственно sWs.textAll() происходит в loop().
  if (!sEventQueue || !json) {
    return;
  }
  EventMsg msg;
  strlcpy(msg.json, json, sizeof(msg.json));
  xQueueSend(sEventQueue, &msg, 0);
}

void NetworkManager::broadcastEvent(const String& json) {
  queueEvent(json.c_str());
}

bool NetworkManager::isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

uint8_t NetworkManager::wsClientCount() {
  return static_cast<uint8_t>(sWs.count());
}

String NetworkManager::wifiIp() {
  if (WiFi.status() != WL_CONNECTED) {
    return String("0.0.0.0");
  }
  return WiFi.localIP().toString();
}

// Слабая реализация по умолчанию: просто пишет команду в лог. Ожидается, что .ino
// переопределит её реальной реализацией, управляющей машиной состояний.
__attribute__((weak)) void NetworkManager::onCommand(NetCommand::Command cmd) {
  Serial.printf("[NetManager] cmd=%s (default stub)\n", NetCommand::commandName(cmd));
}
