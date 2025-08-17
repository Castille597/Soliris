// net.h — gestion connectivité & envoi JSON (Wi-Fi -> 4G -> BLE)
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ====== CONFIG À ADAPTER ======
// Charge un fichier local non commité (secret.h / secrets.h)
#if defined(__has_include)
  #if __has_include("secret.h")
    #include "secret.h"
  #elif __has_include("secrets.h")
    #include "secrets.h"
  #endif
#endif

// Valeurs par défaut (utilisées si aucun secret.* n'est présent)
#ifndef WIFI_SSID
  #define WIFI_SSID      "MonHotspot"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS      "motdepasse"
#endif
#ifndef BACKEND_WIFI
  #define BACKEND_WIFI   "http://192.168.1.26:5050"
#endif

// === Endpoint central pour le backend ===
#ifndef ENDPOINT_PATH
  #define ENDPOINT_PATH  "/telemetry"   // (au lieu de "/ingest")
#endif

// (optionnel 4G, remplacé par secret.h si défini)
#ifndef APN
  #define APN      "orange"
  #define APN_USER ""
  #define APN_PASS ""
#endif

// Option tunnel 4G (pas secret) — tu peux les laisser tel quel
#ifndef USE_CELLULAR_TUNNEL
  #define USE_CELLULAR_TUNNEL   0   // 0 = off (buffer)
#endif
#ifndef BACKEND_TUNNEL
  #define BACKEND_TUNNEL        "https://mon-tunnel-public/ingest"
#endif

// ====== Sélection modem 4G (TinyGSM) ======
// Décommente le bon modèle si tu actives la 4G
// #define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_A7670
// #define TINY_GSM_MODEM_BG95

#if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
  #include <TinyGsmClient.h>
  HardwareSerial Modem(2);     // UART2 (RX/TX à définir selon ton PCB)
  TinyGsm modem(Modem);
  TinyGsmClient gsmClient(modem);
  const char* APN      = "orange";  // ⚠️ APN opérateur
  const char* APN_USER = "";
  const char* APN_PASS = "";
#endif

// ====== BLE (fallback passerelle smartphone) ======
#include <NimBLEDevice.h>    // plus léger que BLEDevice
static NimBLEServer*        bleServer = nullptr;
static NimBLECharacteristic* bleChar  = nullptr;
// UUIDs arbitraires (garde-les constants côté app)
#define BLE_SVC_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ====== États ======
static bool wifiReady = false;
static bool cellReady = false;
static bool bleReady  = false;

static uint32_t lastWiFiAttempt = 0;
static uint32_t lastCellAttempt = 0;

// ====== Buffer offline si 4G sans tunnel ======
#define OFFLINE_MAX  64
static String offlineQ[OFFLINE_MAX];
static int qHead = 0, qTail = 0;

static void offline_enqueue(const String& s) {
  int nxt = (qTail + 1) % OFFLINE_MAX;
  if (nxt == qHead) { qHead = (qHead + 1) % OFFLINE_MAX; } // drop oldest
  offlineQ[qTail] = s; qTail = nxt;
}
static bool offline_dequeue(String& out) {
  if (qHead == qTail) return false;
  out = offlineQ[qHead]; qHead = (qHead + 1) % OFFLINE_MAX; return true;
}

// ====== Wi-Fi ======
static void wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) delay(200);
  wifiReady = (WiFi.status() == WL_CONNECTED);
  Serial.println(wifiReady ? "[NET] Wi-Fi OK" : "[NET] Wi-Fi FAIL");
}

static bool http_post_wifi(const String& json) {
  if (!wifiReady) return false;
  HTTPClient http;
  String url = String(BACKEND_WIFI) + String(ENDPOINT_PATH);  // <-- /telemetry
  if (!http.begin(url)) return false;
  http.addHeader("Content-Type", "application/json");

  int code = http.POST((uint8_t*)json.c_str(), json.length());

  // --- LOG LISIBLE ---
  if (code <= 0) {
    Serial.printf("[NET] POST %s -> ERR %d\n", url.c_str(), code);
  } else {
    Serial.printf("[NET] POST %s -> %d\n", url.c_str(), code);
  }
  // --------------------

  http.end();
  return (code > 0 && code < 400);
}

// ====== Cellulaire (TinyGSM) ======
static void cell_connect() {
#if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
  // ⚠️ adapte RX/TX/PWRKEY si besoin
  Modem.begin(115200, SERIAL_8N1, 16, 17);
  delay(300);
  if (!modem.init())                             { Serial.println("[NET] Modem init FAIL"); cellReady = false; return; }
  if (!modem.waitForNetwork(60000))              { Serial.println("[NET] No network");      cellReady = false; return; }
  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) { Serial.println("[NET] GPRS FAIL");      cellReady = false; return; }
  cellReady = true;
  Serial.println("[NET] Cellular OK");
#else
  cellReady = false;
#endif
}

#if USE_CELLULAR_TUNNEL
  static bool http_post_cell_tunnel(const String& json) {
  #if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
    if (!cellReady) return false;
    // Placeholder simple : implémenter un POST HTTP “manuel” via TinyGsmClient si tunnel HTTP.
    return false;
  #else
    return false;
  #endif
  }
#endif

// ====== BLE (peripheral, notifie le JSON) ======
// Astuce compatibilité: certaines versions de NimBLE changent la signature des callbacks.
// On fournit les 2 variantes SANS 'override' -> plus d'erreur IntelliSense.
class BridgeCallbacks : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer* s) {
    (void)s;
    Serial.println("[NET] BLE central connecté");
  }
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) {
    (void)s; (void)desc;
    Serial.println("[NET] BLE central connecté");
  }
  void onDisconnect(NimBLEServer* s) {
    (void)s;
    Serial.println("[NET] BLE central déconnecté");
  }
  void onDisconnect(NimBLEServer* s, int reason) {
    (void)s; (void)reason;
    Serial.println("[NET] BLE central déconnecté");
  }
};

static void ble_start() {
  NimBLEDevice::init("Soliris-Bridge");
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BridgeCallbacks());
  auto svc = bleServer->createService(BLE_SVC_UUID);
  bleChar = svc->createCharacteristic(
      BLE_CHR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  svc->start();
  bleServer->getAdvertising()->addServiceUUID(BLE_SVC_UUID);
  bleServer->getAdvertising()->start();
  bleReady = true;
  Serial.println("[NET] BLE prêt (advertising).");
}

// ====== API PUBLIQUE ======
static void net_setup() {
  wifi_connect();
#if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
  if (!wifiReady) cell_connect();
#endif
  ble_start(); // prêt pour le pont smartphone
}

static void net_loop() {
  // Re-check Wi-Fi régulièrement si down
  if (!wifiReady && (millis() - lastWiFiAttempt) > 15000) {
    lastWiFiAttempt = millis();
    wifi_connect();
  }
  // Re-check cell si down et Wi-Fi down
#if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
  if (!wifiReady && !cellReady && (millis() - lastCellAttempt) > 30000) {
    lastCellAttempt = millis();
    cell_connect();
  }
#endif

  // Si Wi-Fi revient: flush du buffer offline
  if (wifiReady) {
    String item; int flushed = 0;
    while (offline_dequeue(item) && flushed < 8) {
      if (!http_post_wifi(item)) { offline_enqueue(item); break; }
      flushed++;
    }
  }
}

// Envoi (à appeler après avoir construit ton JSON)
static bool net_send(const String& json) {
  // 1) Wi-Fi direct
  if (wifiReady) {
    if (http_post_wifi(json)) return true;
  }

  // 2) 4G → tunnel (si activé) sinon buffer
#if USE_CELLULAR_TUNNEL
  #if defined(TINY_GSM_MODEM_SIM7600) || defined(TINY_GSM_MODEM_SIM7000) || defined(TINY_GSM_MODEM_A7670) || defined(TINY_GSM_MODEM_BG95)
    if (cellReady) {
      if (http_post_cell_tunnel(json)) return true;
    }
  #endif
#else
  if (!wifiReady) offline_enqueue(json); // on garde localement, on flush quand Wi-Fi revient
#endif

  // 3) BLE → notifie le JSON au téléphone (qui fera le POST)
  if (bleReady && bleChar) {
    // MTU ~180, donc on coupe en tranches si besoin
    size_t n = json.length();
    const size_t CHUNK = 160;
    for (size_t i = 0; i < n; i += CHUNK) {
      String part = json.substring(i, min(i + CHUNK, n));
      bleChar->setValue((uint8_t*)part.c_str(), part.length());
      bleChar->notify();
      delay(15);
    }
    // Marqueur de fin de message
    const char* end = "\n";
    bleChar->setValue((uint8_t*)end, 1);
    bleChar->notify();
  }

  // succès Wi-Fi/4G => true ; via BLE = best-effort
  return false;
}