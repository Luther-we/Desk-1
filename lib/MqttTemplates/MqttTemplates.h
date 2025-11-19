#pragma once
#include <stdint.h>


void publishButtonState(bool pressed);

void publishDiscovery();

// // Publie l'état de la lampe sur t_state
// void mqttPublishLampState(bool on,
//                           uint8_t brightness,
//                           uint8_t r, uint8_t g, uint8_t b,
//                           LampEffect effect);

// // Publie température + humidité sur les topics capteurs
// void mqttPublishDht(float temperature, float humidity);

// // Publie la discovery Home Assistant (light + temp + hum)
// void mqttPublishDiscovery();
