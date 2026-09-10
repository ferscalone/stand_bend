#include "Telemetry.h"

#include "Config.h"

// ArduinoJson нет в lib_deps, поэтому JSON собирается через snprintf в буфер на
// стеке. Формат фиксированный и компактный; NET_WS_BUFFER_SIZE (512) - буфер
// отправки WebSocket, поэтому запас относительно него делает одиночную отправку
// дешёвой.

String buildTelemetryJson() {
  // Берётся ОДИН атомарный снимок всех разделяемых полей телеметрии, чтобы JSON был
  // внутренне согласован: каждое поле берётся из одного и того же момента.
  TelemetrySnapshot snap;
  getTelemetrySnapshot(snap);

  const uint32_t uptimeMs = millis();

  char buf[416];
  const int written = snprintf(buf, sizeof(buf),
           "{\"state\":\"%s\",\"phase\":\"%s\","
           "\"bend_angle_deg\":%.2f,"
           "\"encoder_abs_deg\":%.2f,\"encoder_rel_deg\":%.2f,\"encoder_fault\":%d,"
           "\"cycles_done\":%lu,\"cycles_target\":%lu,"
           "\"cfg_cycles\":%lu,\"cfg_dwell_ms\":%lu,\"cfg_speed_us\":%lu,\"cfg_amplitude_deg\":%.2f,"
           "\"uptime_ms\":%lu}",
           toString(snap.state),
           snap.phase ? snap.phase : "IDLE",
           static_cast<double>(snap.bendAngleDeg),
           static_cast<double>(snap.encAbsDeg),
           static_cast<double>(snap.encRelDeg),
           snap.encoderFault ? 1 : 0,
           static_cast<unsigned long>(snap.cyclesDone),
           static_cast<unsigned long>(snap.cyclesTarget),
           static_cast<unsigned long>(snap.cfgCycles),
           static_cast<unsigned long>(snap.cfgDwellMs),
           static_cast<unsigned long>(snap.cfgSpeedUs),
           static_cast<double>(snap.cfgAmplitudeDeg),
           static_cast<unsigned long>(uptimeMs));
  if (written < 0 || written >= static_cast<int>(sizeof(buf))) {
    Serial.println("[TELEMETRY][WARN] JSON truncated, buf too small");
  }
  return String(buf);
}

String buildEventJson(const char* event, const char* detail) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"event\":\"%s\",\"detail\":\"%s\",\"uptime_ms\":%lu}",
           event, detail, static_cast<unsigned long>(millis()));
  return String(buf);
}
