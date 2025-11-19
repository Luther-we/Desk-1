#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "MqttClient.h"
#include "MqttTemplates.h"
#include "MqttConfig.h"
#include "OTA.h"
#include "WifiConfig.h"
#include "WifiConnect.h"


#define BTN_PIN           5            // GPIO où est branché OUT du TTP223
#define BTN_ACTIVE_LEVEL  HIGH         // TTP223 = actif à HIGH
#define BTN_DEBOUNCE_MS   30           // antirebond simple

bool btnPhysicalPressed = false;   // état logique interne (true = touché)


/* ========= WIFI / MQTT ========= */
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

/* ========= ETAT BOUTON ========= */



/* ========= LECTURE TTP223 (poll + antirebond) ========= */
void updateButtonPhysical() {
  static bool lastStable     = false;
  static bool lastRead       = false;
  static uint32_t lastChangeMs = 0;

  // TTP223 actif à HIGH
  bool raw = (digitalRead(BTN_PIN) == BTN_ACTIVE_LEVEL); // true = touché

  // Détection de changement brut (pour l'antirebond)
  if (raw != lastRead) {
    lastRead = raw;
    lastChangeMs = millis();
  }

  // Si l'état est resté stable assez longtemps, on le valide
  if (millis() - lastChangeMs >= BTN_DEBOUNCE_MS && raw != lastStable) {
    lastStable = raw;
    btnPhysicalPressed = raw;

    // Mise à jour du binary_sensor (pressed/released)
    publishButtonState(btnPhysicalPressed);
    Serial.printf("[BTN] physical %s\n",
                  btnPhysicalPressed ? "PRESSED" : "RELEASED");
  }
}

void onConnectPublish() {
  publishDiscovery();
  // Publier l'état physique du bouton à la connexion MQTT
  publishButtonState(btnPhysicalPressed);
}

/* ========= SETUP / LOOP ========= */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n[BOOT] Desk 1");

  pinMode(BTN_PIN, INPUT);   // TTP223

  wifiSetup(WIFI_CONFIG);
  mqttSetup();
  // mqttClient().setCallback(mqttCallback);

  setupOTA();
}

void loop() {
  wifiLoop();

  if (wifiIsConnected()) {
    bool justConnected = mqttEnsureConnected();
    if (justConnected) {
      onConnectPublish();
      
    }
  }

  mqttLoop();
  loopOTA();

  updateButtonPhysical();

  delay(5); 


  // TODO : add cette logique dans wifiConnect
  // static uint32_t lastWifiCheck = 0;
  // static uint32_t lastMqttCheck = 0;

  // uint32_t now = millis();

  // // Vérifier le WiFi toutes les 2 secondes max
  // if (now - lastWifiCheck >= 2000) {
  //   lastWifiCheck = now;
  //   if (WiFi.status() != WL_CONNECTED) {
  //     wifiConnect();
  //   }
  // }
 // laisse le CPU souffler un peu
}
