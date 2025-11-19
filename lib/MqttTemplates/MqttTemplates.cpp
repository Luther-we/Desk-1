#include <ArduinoJson.h>
#include "MqttClient.h"
#include "MqttConfig.h"
#include "MqttTemplates.h"


void publishButtonState(bool pressed) {
  const char* s = pressed ? "pressed" : "released";
  mqttClient().publish(t_btn_state.c_str(), s, true);   // retained
  Serial.printf("[BTN] physical state=%s\n", s);
}

void publishDiscovery() {
  JsonDocument doc;

  doc["name"]        = "Action touche";
  doc["uniq_id"]     = String(DEVICE_NAME) + "_button_state";
  doc["obj_id"]      = String(DEVICE_NAME) + "_button_state";
  doc["stat_t"]      = t_btn_state;
  doc["pl_on"]       = "pressed";
  doc["pl_off"]      = "released";
  doc["dev_cla"]     = "occupancy";          // purement esthétique
  doc["avty_t"]      = t_avail;
  doc["pl_avail"]    = "online";
  doc["pl_not_avail"]= "offline";

  JsonObject dev = doc["device"].to<JsonObject>();
  dev["name"] = FRIENDLY_NAME;
  dev["mf"]   = "DIY";
  dev["mdl"]  = "ESP32 C3 Button TTP223";
  JsonArray ids = dev["ids"].to<JsonArray>();
  ids.add(DEVICE_NAME);

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqttClient().publish(t_disc_bsens.c_str(), (const uint8_t*)buf, (unsigned int)n, true);
  Serial.println("[DISCOVERY] binary_sensor entity published");
}



