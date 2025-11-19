#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ========= CONFIG UTILISATEUR ========= */
static const char* WIFI_SSID    = "Livebox-1714";
static const char* WIFI_PASS    = "47PnSkZz7GtYv4QG7m";

/* ===== CONFIG IP STATIQUE ===== */
IPAddress local_IP(192, 168, 1, 20);   // <-- Choisis ici ton XX
IPAddress gateway(192,168,1,1);        // Routeur
IPAddress subnet(255,255,255,0);
IPAddress dns1(1, 1, 1, 1);
IPAddress dns2(8, 8, 8, 8);

static const char* MQTT_HOST    = "192.168.1.31";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_USER    = "mqttuser";
static const char* MQTT_PASSWD  = "aloha22";

/* ========= IDENTIFIANT APPAREIL ========= */
#define ESP_WIFI_NAME  "Desk #1"
#define DEVICE_NAME    "esp_button1"   // préfixe topics + unique_id
#define FRIENDLY_NAME  "ESP Bouton TTP223"

/* ========= TTP223 =========
   - Alimente le module en 3,3 V
   - GND commun avec l'ESP
   - OUT du TTP223 -> GPIO BTN_PIN
   - Par défaut, la plupart des modules : sortie HIGH quand on touche.
*/
#define BTN_PIN           5            // GPIO où est branché OUT du TTP223
#define BTN_ACTIVE_LEVEL  HIGH         // TTP223 = actif à HIGH
#define BTN_DEBOUNCE_MS   30           // antirebond simple

/* ========= TOPICS MQTT ========= */
String t_base      = String(DEVICE_NAME) + "/";
String t_btn_state = t_base + "button/state";      // ESP → HA (binary_sensor)
String t_avail     = t_base + "status";            // disponibilité

// Discovery topics
String t_disc_bsens = "homeassistant/binary_sensor/" + String(DEVICE_NAME) + "_state/config";

/* ========= WIFI / MQTT ========= */
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

/* ========= ETAT BOUTON ========= */
bool btnPhysicalPressed = false;   // état logique interne (true = touché)

/* ========= PUBLISH ÉTAT BOUTON PHYSIQUE ========= */
void publishButtonState(bool pressed) {
  const char* s = pressed ? "pressed" : "released";
  mqtt.publish(t_btn_state.c_str(), s, true);   // retained
  Serial.printf("[BTN] physical state=%s\n", s);
}

/* ========= DISCOVERY HA ========= */
void publishDiscovery() {
  // ----- Binary sensor: reflète l'état du TTP223 -----
  JsonDocument doc;

  doc["name"]        = String(FRIENDLY_NAME) + " État touche";
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
  mqtt.publish(t_disc_bsens.c_str(), (const uint8_t*)buf, (unsigned int)n, true);
  Serial.println("[DISCOVERY] binary_sensor entity published");
}

/* ========= WIFI / MQTT CONNECT ========= */
void wifiConnect() {
  Serial.println("[WiFi] Configuration IP statique...");
  
  if (!WiFi.config(local_IP, gateway, subnet, dns1, dns2)) {
    Serial.println("[WiFi] ERREUR config IP !");
  }
  
  WiFi.mode(WIFI_STA);
  // WiFi.setHostname(ESP_WIFI_NAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  Serial.printf("[WiFi] Connexion %s ...\n", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Hostname: %s, IP: %s\n",
                  WiFi.getHostname(),
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] ECHEC (timeout)");
  }
}

void mqttEnsure() {
  while (!mqtt.connected()) {
    String clientId = String(DEVICE_NAME) + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.printf("[MQTT] Connexion %s:%u ...\n", MQTT_HOST, MQTT_PORT);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWD,
                           t_avail.c_str(), 0, true, "offline");
    if (ok) {
      Serial.println("[MQTT] Connecté");
      mqtt.publish(t_avail.c_str(), "online", true);

      publishDiscovery();
      publishButtonState(false);  // TTP223 au repos au démarrage
    } else {
      Serial.printf("[MQTT] Échec (state=%d), retry...\n", mqtt.state());
      delay(1500);
    }
  }
}

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

/* ========= SETUP / LOOP ========= */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n[BOOT] ESP Button TTP223 C3");

  pinMode(BTN_PIN, INPUT);   // TTP223

  wifiConnect();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(10);

  // ---- OTA ----
  ArduinoOTA.setHostname("esp-button");   // nom réseau (esp-button.local)
  // ArduinoOTA.setPassword("ton_mdp");   // optionnel
  
  ArduinoOTA
    .onStart([]() {
      Serial.println("[OTA] Start");
    })
    .onEnd([]() {
      Serial.println("\n[OTA] End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("[OTA] Error[%u]\n", error);
    });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");
}

void loop() {
  ArduinoOTA.handle();

  static uint32_t lastWifiCheck = 0;
  static uint32_t lastMqttCheck = 0;

  uint32_t now = millis();

  // Vérifier le WiFi toutes les 2 secondes max
  if (now - lastWifiCheck >= 2000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnect();
    }
  }

  // Vérifier le MQTT toutes les 2 secondes max
  if (now - lastMqttCheck >= 2000) {
    lastMqttCheck = now;
    if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
      mqttEnsure();
    }
  }

  if (mqtt.connected()) {
    mqtt.loop();
  }

  updateButtonPhysical();

  delay(5);  // laisse le CPU souffler un peu
}
