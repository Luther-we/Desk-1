#pragma once
#include <Arduino.h>

struct MqttConfig {
  const char* host;
  uint16_t    port;
  const char* user;
  const char* password;
};

// Config globale
extern const MqttConfig MQTT_CONFIG;

// Identité du device
extern const char* const DEVICE_NAME;
extern const char* const FRIENDLY_NAME;

// Topics MQTT globaux
extern String t_base;
extern String t_btn_state;
extern String t_avail;
extern String t_disc_bsens;

