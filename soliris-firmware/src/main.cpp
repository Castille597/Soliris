#include "net.h"
// ===== SAFE: BME280 + DS18B20 + SGP40 + SCD40 =====
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>
#include <limits.h>
#include <OneWire.h>
#include <Adafruit_NeoPixel.h>
#include <DallasTemperature.h>
DeviceAddress DS_ADDR;
bool ds_parasite = false;
#include <Adafruit_BME280.h>
#include <Adafruit_SGP40.h>
#pragma push_macro("I2C_BUFFER_LENGTH")
#undef I2C_BUFFER_LENGTH
#include <MAX30105.h>
#pragma pop_macro("I2C_BUFFER_LENGTH")
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include "Si115X.h"
Si115X si115(0x53);
bool si_ok = false;

// ==== BLE (seulement sur ESP32) ====
#if defined(ESP_PLATFORM)
  #include <NimBLEDevice.h>
#else
  // stub pour autres cibles: évite les #ifdef dans setup()
  static inline void ble_setup_ctrl() {}
#endif

// Mesures/états "soleil"
uint16_t si_vis = 0, si_ir = 0, si_uv = 0;  // UV restera 0 avec SI1151
float    uv_index   = NAN;                  // SI1151 => NAN (pas d'UV natif)
bool     si_covered = false;                // capteur caché ?
bool     si_outdoor = false;                // dehors ?
float    sun_proxy  = 0.0f;                 // 0..1 (proxy risque soleil)
uint32_t sun_dose   = 0;                    // intégrale

// --- On-wrist robust ---
bool  onWrist = false;
float onwrist_score = 0.0f;   // 0..1 pour debug
float onwrist_dSA   = NAN;    // T_peau - T_air (pour debug)
float onwrist_dTdt  = 0.0f;   // dérivée peau °C/s (pour debug)

const float   G = 9.81f;

#define EASY_TEST 1
#if EASY_TEST
  // permissive thresholds (mattress/desk tests)
  const float    IMPACT_G           = 1.8f;     // impact if |a| > 1.8 g
  const float    NO_MOTION_DYN      = 1.0f;     // |amag-1g| < 1.0 m/s²
  const float    NO_MOTION_GYRO     = 2.0f;     // |gx|+|gy|+|gz| < 2.0 rad/s
  const uint16_t IMPACT_LOCK_MS     = 800;      // anti-double impact
  const uint16_t INACT_AFTER_MS     = 3000;     // inactivity >= 3s
  const uint16_t LYING_CONFIRM_MS   = 1000;     // lying >= 1s
#else
  // stricter (realistic) thresholds
  const float    IMPACT_G           = 2.8f;
  const float    NO_MOTION_DYN      = 0.25f;
  const float    NO_MOTION_GYRO     = 0.50f;
  const uint16_t IMPACT_LOCK_MS     = 1200;
  const uint16_t INACT_AFTER_MS     = 10000;
  const uint16_t LYING_CONFIRM_MS   = 4000;
#endif

// Vitals thresholds (adjust per use case)
const int   HR_BRADY       = 40;  // bpm
const int   HR_TACHY       = 180; // bpm (can be high right after fall; use with care)
const int   SPO2_LOW       = 88;  // %
const uint16_t UNCON_MIN_MS = 20000; // sustain signs for >=20s

// --- Détection "soleil sur la peau" ---
bool  sun_touch = false;   // 1 si soleil direct sur la peau
float sun_score = 0.0f;    // 0..1 = intensité * orientation
const int SUN_AXIS_SIGN = -1; // si c’est inversé, passe à -1

// ==== Ambient correction (BME280) ====
// Calibrations à ajuster chez toi
float T_FIXED_OFFSET = -4.94f;
float RH_GAIN        = 1.00f;
float RH_OFFSET      = 10.0f;

// Heuristiques "cold-floor" + lissage
const float K_UP   = 0.002f;
const float K_DOWN = 0.60f;
const float ALPHA_SMOOTH = 0.30f;

// États internes pour l'algorithme
float T_floor = NAN;
float T_air_est_prev = NAN;

// Sorties corrigées (ce que tu publieras)
float env_c_out = NAN, rh_out_corr = NAN, hpa_out = NAN;

// Utilitaires humidité (Magnus)
static float es_hPa(float T_C) {
  return 6.112f * expf((17.62f * T_C) / (243.12f + T_C));
}
static float rh_retarget(float RH_raw, float Traw_C, float Tcorr_C) {
  float e = (RH_raw / 100.0f) * es_hPa(Traw_C);
  float RHcorr = 100.0f * (e / es_hPa(Tcorr_C));
  if (RHcorr < 0) RHcorr = 0; if (RHcorr > 100) RHcorr = 100;
  return RHcorr;
}

#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
bool mpu_ok = false;
#include <VOCGasIndexAlgorithm.h>  // algo VOC (on force l'include)

VOCGasIndexAlgorithm voc_algo;
static inline int32_t voc_index_from_sraw(int32_t s){ return voc_algo.process(s); }

#include <SensirionI2cScd4x.h>
// make sure that we use the proper definition of NO_ERROR
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0
bool scd_ok = false;

SensirionI2cScd4x sensor;

static char errorMessage[64];
static int16_t error;

// --- Telemetry push control (send every 30 s) ---
static unsigned long last_push_ms = 0;
static const unsigned long PUSH_PERIOD_MS = 30000;   // 30s
static const char* USER_ID = "veronique";

// float -> JSON number or null
static inline String f2json(float v, int d = 2) {
  if (isnan(v)) return "null";
  char b[24]; dtostrf(v, 0, d, b);
  return String(b);
}

// ===== PRIVACY/DEMO FLAGS =====
#define DEMO_MODE 1   // 1 = n'imprime que les champs SAFE (les lignes sensibles restent dans le code mais commentées)
                      // 0 = impression RAW (comportement initial)

#ifndef STRICT_PI
#define STRICT_PI 1    // 1 = AUCUNE donnée perso réelle capturée/émise
#endif

#ifndef SIM_PI
#define SIM_PI 1       // 1 = simuler la PI quand STRICT_PI=1 (facultatif)
#endif

#if STRICT_PI
  #define BIOMETRICS_ENABLED 0
#else
  #define BIOMETRICS_ENABLED 1
#endif     


#ifndef USER_ID
  #define USER_ID "veronique"
#endif

// ===== Demo vs Real location =====
#define DEMO_LOCATION 1             // 1 = force Madrid (demo), 0 = use real GPS later
const double DEMO_LAT = 40.4168;    // Madrid
const double DEMO_LON = -3.7038;

// ===== GPS toggles (privacy) =====
#define GPS_ENABLE   0              // disable GNSS for the demo
#define PRINT_COORDS 0              // keep 0; no real coords printed
// (optional) SIM_CITY no longer used; you can delete it

// ===== Helpers d'arrondi/bucketing =====
static inline float qf(float x, float step){ if (isnan(x)) return NAN; return roundf(x/step)*step; }
static inline int   qi(float x, float step){ if (isnan(x)) return -1; return (int)roundf(x/step)*step; }
static inline int   bucket_ppg_drive(uint8_t drv){
  if (drv <= 0x10) return 0;          // faible
  if (drv <= 0x28) return 1;          // moyen-
  if (drv <= 0x60) return 2;          // moyen+
  return 3;                           // fort
}

// --- Prototypes (forward declarations)
void ppg_init();
void ppg_service();
float read_env_c();
float read_env_rh();
float read_env_hpa();
float read_skin_c();
float read_skin_c_raw();
float read_voc_index(int* srawOut, float tempC=NAN, float rh=NAN);
float read_co2_ppm(float* out_rh);
bool read_mpu(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& amag);
void si115_service();
void onwrist_update_robuste(float skinC, float airC, bool ppg_contact, float dc_ir, int motion);
void ambient_update(float skinC);

void PrintUint64(uint64_t& value) {
    Serial.print("0x");
    Serial.print((uint32_t)(value >> 32), HEX);
    Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

// Cadence de lecture PPG
static uint32_t ppg_next_ms = 0;
const uint8_t  PPG_PERIOD_MS = 10;   // 100 Hz

// Détecteur de pics (sur l’AC IR)
static float    ppg_env = 0.0f;      // enveloppe |AC|
static float    ppg_prev_ac = 0.0f;  // AC au sample précédent
const  float    PPG_ENV_ALPHA = 0.10f;   // lissage enveloppe (~100 ms)
const  float    PPG_THR_RATIO = 0.55f;   // seuil = ratio * enveloppe
const  uint16_t PPG_REFRACT_MS = 280;    // période réfractaire (≈ max 214 BPM)

// Pins
#define SDA1_PIN 7
#define SCL1_PIN 6
#define SDA2_PIN 4
#define SCL2_PIN 5
#define ONEWIRE_PIN 21

// Objets dynamiques (créés dans setup)
OneWire*           ow  = nullptr;
DallasTemperature* ds  = nullptr;
Adafruit_BME280*   bme = nullptr;
Adafruit_SGP40*    sgp = nullptr;

// États
bool ds_ok=false, bme_ok=false, sgp_ok=false;

// ---- Activity / Posture / Steps ----
enum Activity { ACT_STILL=0, ACT_WALK=1, ACT_RUN=2 };
enum Posture  { POST_UNKNOWN=0, POST_STANDING=1, POST_SITTING=2, POST_LYING=3 };

// ===== LED + BUZZER =====
#define RGB_PIN   14         // NeoPixel
#define BUZZ_PIN   2         // Buzzer (passif)

// Interrupteurs commandables par l’app
volatile bool ALERTS_LED_ENABLED  = true;
volatile bool ALERTS_BUZZ_ENABLED = true;

// Objet NeoPixel (1 LED)
Adafruit_NeoPixel led(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Petites helpers
static inline void led_show(uint8_t r,uint8_t g,uint8_t b){
  if (!ALERTS_LED_ENABLED){ led.clear(); led.show(); return; }
  led.fill(led.Color(r,g,b));
  led.show();
}
static inline void buzz_on(uint16_t hz){
  if (ALERTS_BUZZ_ENABLED) tone(BUZZ_PIN, hz);
}
static inline void buzz_off(){
  noTone(BUZZ_PIN);
  // repos en HIGH (utile si ton driver est actif-bas)
  digitalWrite(BUZZ_PIN, HIGH);
}

// Types d’alertes
enum AlertKind { ALERT_NONE, ALERT_HYDR, ALERT_SUN, ALERT_AIR, ALERT_FALL, ALERT_HRT };

// Petit état machine pour jouer les motifs sans bloquer
struct AlertPlay { AlertKind k; uint32_t t0; uint8_t step; bool active; } _ap = {ALERT_NONE,0,0,false};

void alerts_init(){
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);   // idle OFF
  led.begin();
  led.setBrightness(80);
  led.clear(); led.show();
}
bool alerts_isPlaying(){ return _ap.active; }
void alerts_play_kind(AlertKind k){ _ap = {k, millis(), 0, k!=ALERT_NONE}; }

// À appeler dans loop() très souvent (non bloquant)
void alerts_update(){
  if(!_ap.active) return;
  uint32_t now = millis();

  switch(_ap.k){

    case ALERT_HYDR: // LED bleu, 1 vibration lente
      if(_ap.step==0){ led_show(0,0,255); buzz_on(1500); _ap.step=1; _ap.t0=now; }
      else if(_ap.step==1 && now-_ap.t0>600){ buzz_off(); _ap.step=2; _ap.t0=now; }
      else if(_ap.step==2 && now-_ap.t0>400){ led_show(0,0,0); _ap.active=false; }
    break;

    case ALERT_SUN: // LED orange, 1 longue
      if(_ap.step==0){ led_show(255,120,0); buzz_on(1200); _ap.step=1; _ap.t0=now; }
      else if(_ap.step==1 && now-_ap.t0>900){ buzz_off(); _ap.step=2; _ap.t0=now; }
      else if(_ap.step==2 && now-_ap.t0>300){ led_show(0,0,0); _ap.active=false; }
    break;

    case ALERT_AIR: // LED jaune, 2 rapides
      if(_ap.step==0){ led_show(255,255,0); buzz_on(1800); _ap.step=1; _ap.t0=now; }
      else if(_ap.step==1 && now-_ap.t0>250){ buzz_off(); _ap.step=2; _ap.t0=now; }
      else if(_ap.step==2 && now-_ap.t0>150){ buzz_on(1800); _ap.step=3; _ap.t0=now; }
      else if(_ap.step==3 && now-_ap.t0>250){ buzz_off(); _ap.step=4; _ap.t0=now; }
      else if(_ap.step==4 && now-_ap.t0>200){ led_show(0,0,0); _ap.active=false; }
    break;

    case ALERT_FALL: // LED rouge, buzz continu ~3s
      if(_ap.step==0){ led_show(255,0,0); buzz_on(1000); _ap.step=1; _ap.t0=now; }
      else if(_ap.step==1 && now-_ap.t0>3000){ buzz_off(); led_show(0,0,0); _ap.active=false; }
    break;

    case ALERT_HRT: // LED rouge, 2 lentes
      if(_ap.step==0){ led_show(255,0,0); buzz_on(1600); _ap.step=1; _ap.t0=now; }
      else if(_ap.step==1 && now-_ap.t0>350){ buzz_off(); _ap.step=2; _ap.t0=now; }
      else if(_ap.step==2 && now-_ap.t0>250){ buzz_on(1600); _ap.step=3; _ap.t0=now; }
      else if(_ap.step==3 && now-_ap.t0>350){ buzz_off(); _ap.step=4; _ap.t0=now; }
      else if(_ap.step==4 && now-_ap.t0>200){ led_show(0,0,0); _ap.active=false; }
    break;

    default: _ap.active=false; break;
  }
}

// === Contrôle depuis le front (JSON) ===
// Exemples: {"play":"fall"} | {"led":0} | {"buzz":1} | {"led":1,"buzz":0,"play":"sun"}
void apply_control_json(const String& s){
  JsonDocument d;                       // v7 : plus de Static/Dynamic
  DeserializationError err = deserializeJson(d, s);
  if (err) return;

  if (d["led"].is<bool>())  ALERTS_LED_ENABLED  = d["led"].as<bool>();
  if (d["buzz"].is<bool>()) ALERTS_BUZZ_ENABLED = d["buzz"].as<bool>();

  if (d["play"].is<const char*>()){
    const char* k = d["play"];
    if (!strcmp(k,"hydr")) alerts_play_kind(ALERT_HYDR);
    else if (!strcmp(k,"sun"))  alerts_play_kind(ALERT_SUN);
    else if (!strcmp(k,"air"))  alerts_play_kind(ALERT_AIR);
    else if (!strcmp(k,"fall")) alerts_play_kind(ALERT_FALL);
    else if (!strcmp(k,"hrt"))  alerts_play_kind(ALERT_HRT);
  }
}

// --- Contrôle via port série (simulateur) ---
void ctrl_serial_poll() {
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n');  // lit jusqu'au \n envoyé par le backend
    s.trim();
    if (s.length()) {
      // Serial.print("[CTRL SERIAL] "); Serial.println(s); // debug (optionnel)
      apply_control_json(s);  // réutilise ton parser JSON
    }
  }
}

#ifdef ESP_PLATFORM
// ===== BLE (Web Bluetooth côté front) =====
#define UUID_SVC_CTRL  "c0de0001-2bad-4b0b-a3f8-9b3b5f2a0001"
#define UUID_CHR_CTRL  "c0de0002-2bad-4b0b-a3f8-9b3b5f2a0001"

class CtrlCb : public NimBLECharacteristicCallbacks {
 public:
  // Variante "ancienne" (1 argument)
  void onWrite(NimBLECharacteristic* c) {
    std::string v = c->getValue();
    if (!v.empty()) apply_control_json(String(v.c_str()));
  }
  // Variante NimBLE récente (2 arguments)
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*conn*/) {
    onWrite(c);
  }
  // Variante encore plus récente (3 arguments, avec offset)
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& /*conn*/, uint16_t /*offset*/) {
    onWrite(c);
  }
} _ctrlCb;

void ble_setup_ctrl() {
  NimBLEDevice::init("Soliris");
  NimBLEServer* srv = NimBLEDevice::createServer();
  NimBLEService* svc = srv->createService(UUID_SVC_CTRL);
  NimBLECharacteristic* chr =
    svc->createCharacteristic(UUID_CHR_CTRL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  chr->setCallbacks(&_ctrlCb);
  chr->setValue("{}");
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_SVC_CTRL);
  adv->start();
}
#endif

volatile uint32_t step_count = 0;
uint32_t last_step_ms = 0;

uint16_t last_step_interval_ms = 0;  // interval of last detected step
uint32_t t_last_upright_ms = 0;      // last time posture != LYING
static uint32_t still_ms = 0;    

bool  fall_event = false;
bool  unconscious = false;
float unconscious_score = 0.0f;

uint32_t t_impact_ms      = 0;  // last hard impact
uint32_t t_last_motion_ms = 0;  // last time motion was seen
uint32_t t_lying_since_ms = 0;  // since lying started

Activity activity_state = ACT_STILL;
Posture  posture_state  = POST_UNKNOWN;

// Filtres pour gravité & accélération le long de g
bool  g_init=false;
float g_lp_x=0, g_lp_y=0, g_lp_z=0;     // gravité low-pass
float a_par_hp = 0;                     // composante “verticale” (HP) pour pas
const float G_LP_ALPHA = 0.05f; // ~0.4 s like the minimal sketch
const float HP_ALPHA   = 0.30f;

#if EASY_TEST
const float STEP_PEAK = 0.35f;   // comme le sketch minimal
#else
const float STEP_PEAK = 0.60f;
#endif
const uint16_t STEP_MIN_MS = 250;   // ~240 BPM max
const uint16_t STEP_MAX_MS = 1200;  // ~50 BPM min

// ==== IMU service rapide (50 Hz) ====
const uint8_t IMU_PERIOD_MS = 20;   // 50 Hz
static uint32_t imu_next_ms = 0;

// Dernières mesures IMU (partagées avec le JSON 1 Hz)
static float ax_g=NAN, ay_g=NAN, az_g=NAN, gx_g=NAN, gy_g=NAN, gz_g=NAN, amag_g=NAN;
static float gyro_sum_g = 0.0f, face_g = 0.0f;
static int   motion_g   = 0;

// ---- MAX3010x (BPM + SpO2) ----
MAX30105 ppg;
bool ppg_ok = false;
bool ppg_contact = false;

uint8_t ppg_irDrive  = 0x30;         // puissance LED IR (auto-ajustée)
uint8_t ppg_redDrive = 0x30;         // puissance LED Rouge (un peu plus fort pour SpO2)

// Fenêtre DC cible pour l’auto-gain IR (équivalent à tes PPG_DC_LOW/HIGH)
long        PPG_IR_LOW  = 40000;
long        PPG_IR_HIGH = 120000;
const uint8_t PPG_DRIVE_MIN = 0x20, PPG_DRIVE_MAX = 0xB0;

// Buffer AC sur ~1 s pour métriques "human-only"
const int   PPG_SR_HZ = 100;
const int   PPG_WIN   = PPG_SR_HZ;   // ~1 s
long        ppg_acIR[PPG_WIN], ppg_acRED[PPG_WIN];
int         ppg_idx = 0, ppg_filled = 0;

// État de verrouillage (onde plausible peau)
bool        ppg_locked = false;
unsigned long ppg_okSince = 0, ppg_badSince = 0;
unsigned long ppg_lastBeat = 0;

float ppg_bpm = 0;
byte  ppg_rates[8] = {0};
byte  ppg_rateSpot = 0;
int   ppg_rateFilled = 0;            // nb d’entrées valides dans rates[]
int   ppg_bpm_avg  = 0;

// SpO2 (approx. ratio-of-ratios)
float dc_ir = 0, dc_red = 0;
float rms_ir = 0, rms_red = 0;
uint32_t spo2_n = 0;
float   spo2_value = NAN;    // 70..100 %, ou NAN si pas fiable
uint8_t spo2_quality = 0;    // 0..3 (heuristique)

// Seuils (ajustables)
const long  DC_CONTACT_MIN = 20000;   // DC minimale pour considérer un doigt
const long  DC_LOSS_MIN    = 15000;   // en dessous: perte de contact
const float AC_CONTACT_MIN = 120.0f;  // amplitude pulsatile moyenne
const float AC_LOSS_MIN    = 80.0f;   // perte si en-dessous
const int   CONTACT_ON_SAMPLES  = 8;  // ~80 ms @100 Hz
const int   CONTACT_OFF_SAMPLES = 20; // ~200 ms @100 Hz

// ---- Timers SCD40 ----
static uint32_t scd_next_read_ms = 0;

int find_onewire_pin(const int* pins, int n, byte* addr_out) {
  for (int i = 0; i < n; ++i) {
    int p = pins[i];

    pinMode(p, INPUT_PULLUP);   // pull-up interne en secours
    delay(2);

    OneWire testBus(p);
    if (testBus.reset()) {      // présence sur le bus ?
      // tente de lire une adresse ROM
      testBus.reset_search();
      if (testBus.search(addr_out)) {
        return p;               // trouvé !
      }
    }
  }
  return -1;                    // rien trouvé
}

void i2c_scan(TwoWire& bus, const char* name){
  Serial.print("[SCAN] "); Serial.print(name); Serial.print(": ");
  for (uint8_t a=1; a<127; a++){
    bus.beginTransmission(a);
    if (bus.endTransmission()==0){ Serial.print("0x"); Serial.print(a,HEX); Serial.print(" "); }
  }
  Serial.println();
}

void setup() {

  Serial.begin(115200); delay(300);
  Serial.println("SAFE start");

#ifdef ESP_PLATFORM
  randomSeed(esp_random());  // ESP32 / ESP-IDF
#else
  randomSeed(micros());      // autres cibles (Arduino AVR/STM32/etc.)
#endif

   #if DEMO_MODE
    Serial.println("[BUILD] DEMO_MODE=1 (SAFE)");
  #else
    Serial.println("[BUILD] DEMO_MODE=0 (RAW)");
  #endif

  #if STRICT_PI && SIM_PI
  Serial.println("[MODE] LIVE non-PI + SIM PI (aucune PI réelle)");
#elif STRICT_PI
  Serial.println("[MODE] LIVE non-PI + PI OFF");
#else
  Serial.println("[MODE] DEMO PI réelle (sanitisée)");
#endif

  Wire.begin(SDA1_PIN, SCL1_PIN);
  Wire1.begin(SDA2_PIN, SCL2_PIN);
  for (uint8_t a=1; a<127; a++) {
  Wire1.beginTransmission(a);
  if (Wire1.endTransmission()==0) { Serial.print("I2C1 found 0x"); Serial.println(a,HEX); }
}

i2c_scan(Wire,  "Wire  (SDA=7, SCL=6)");
i2c_scan(Wire1, "Wire1 (SDA=4, SCL=5)");

  Wire.setClock(100000);
  Wire.setTimeOut(1000);   // 1s de timeout I2C

  static float    skin_last = NAN;
  static uint32_t skin_last_ms = 0;

  Wire1.setClock(100000);
  Wire1.setTimeOut(1000); 
  Serial.println("I2C OK");

#if BIOMETRICS_ENABLED
  ppg_init();
#else
  ppg_ok = false;   // PPG réel désactivé
#endif

// ===== SI115X (Sunlight Sensor) =====
// Si ta version de lib accepte un TwoWire*, essaye si besoin : si_ok = si115.Begin(&Wire);
if (!si_ok) {
  delay(20);
  si_ok = si115.Begin();
  Serial.println(si_ok ? "SI115X OK (retry)" : "SI115X FAIL");
}

  // ==== PIN HUNTER OneWire ====
const int candidates[] = {8,9,10,11,12,13,14,15,16,17,18,21,33};
byte rom[8] = {0};
int foundPin = find_onewire_pin(candidates, sizeof(candidates)/sizeof(candidates[0]), rom);

if (foundPin > 0) {
  Serial.print(">>> OneWire device on GPIO "); Serial.println(foundPin);
  Serial.print("ROM family: 0x"); Serial.println(rom[0], HEX); // DS18B20 = 0x28
  Serial.print("ROM addr  : ");
  for (uint8_t i=0; i<8; i++){ if (rom[i] < 16) Serial.print("0"); Serial.print(rom[i], HEX); }
  Serial.println();
} else {
  Serial.println(">>> No OneWire device found on tested pins.");
}

  // DS18B20 (OneWire)
#if BIOMETRICS_ENABLED
ow = new OneWire(ONEWIRE_PIN);
ds = new DallasTemperature(ow);
ds->begin();

uint8_t count = ds->getDeviceCount();
Serial.print("DS18B20 count: "); Serial.println(count);

if (count > 0 && ds->getAddress(DS_ADDR, 0)) {
  // affichage adresse (debug)
  Serial.print("DS18B20 addr: ");
  for (uint8_t i=0; i<8; i++){ if (DS_ADDR[i] < 16) Serial.print("0"); Serial.print(DS_ADDR[i], HEX); }
  Serial.print("  parasite? "); Serial.println(ds->isParasitePowerMode() ? "YES" : "NO");

  ds->setResolution(DS_ADDR, 10);   // 10 bits (~190 ms), plus réactif pour la peau
  ds->setWaitForConversion(true);   // attendre la conversion avant de lire
  ds_ok = true;
  Serial.println("DS18B20 OK");
  // Prime: première conversion pour éviter 85°C en début de loop
  ds->requestTemperaturesByAddress(DS_ADDR);
  float t0 = ds->getTempC(DS_ADDR);
  Serial.print("DS18B20 first read: "); Serial.println(t0);
} else {
  ds_ok = false;
  Serial.println("DS18B20 FAIL (pas trouvé)");
}
#else
  ds_ok = false;  // Température peau réelle désactivée en STRICT_PI
#endif

  // BME280 (0x77 sur I2C1)
  bme = new Adafruit_BME280();
  bme_ok = bme->begin(0x77, &Wire);
  Serial.println(bme_ok ? "BME280 OK" : "BME280 FAIL");
  if (bme_ok) {
  bme->setSampling(
    Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1,  // Temp
    Adafruit_BME280::SAMPLING_X1,  // Press
    Adafruit_BME280::SAMPLING_X1,  // Hum
    Adafruit_BME280::FILTER_OFF,
    Adafruit_BME280::STANDBY_MS_1000
  );
}

  // SGP40
  sgp = new Adafruit_SGP40();
  sgp_ok = sgp->begin(&Wire);
  Serial.println(sgp_ok ? "SGP40 OK" : "SGP40 FAIL");

  // Algo VOC
  Serial.println("VOC algo: PRESENT");

  // ---- SCD40 init (robuste + logs) ----
Serial.println("SCD4x: probing...");

// ✅ SCD40 est sur I2C1 -> Wire (SDA=7, SCL=6)
TwoWire& BUS = Wire;

BUS.setClock(100000);
sensor.begin(BUS, 0x62);     // <— adresse SCD40

delay(30);
sensor.wakeUp();
sensor.stopPeriodicMeasurement();
delay(500);
error = sensor.reinit();
delay(20);

// MPU6050 — auto-détection bus + adresse
const char* imu_bus_name = "Wire1";
mpu_ok = mpu.begin(0x68, &Wire1) || mpu.begin(0x69, &Wire1);
if (!mpu_ok) {                     // fallback si le capteur est en fait sur Wire
  imu_bus_name = "Wire";
  mpu_ok = mpu.begin(0x68, &Wire) || mpu.begin(0x69, &Wire);
}
Serial.printf("MPU6050 %s sur %s (essaie 0x68/0x69)\n", mpu_ok ? "OK" : "FAIL", imu_bus_name);

if (mpu_ok) {
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 1ère lecture pour debug
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);
  Serial.printf("[MPU] ax=%.2f ay=%.2f az=%.2f | gx=%.2f gy=%.2f gz=%.2f\n",
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                g.gyro.x, g.gyro.y, g.gyro.z);
}

// Vérif comm + démarrage périodique (~1 mesure / 5 s)
uint64_t serialNumber = 0;
error = sensor.getSerialNumber(serialNumber);
if (error != NO_ERROR) {
  Serial.print("getSerialNumber error: "); Serial.println(error);
  scd_ok = false;
} else {
  Serial.print("SCD4x SN: "); PrintUint64(serialNumber); Serial.println();
  error = sensor.startPeriodicMeasurement();
  scd_next_read_ms = millis() + 6500;   // première lecture dans ~6,5 s
  scd_ok = (error == NO_ERROR);
}
Serial.println(scd_ok ? "SCD40 OK" : "SCD40 FAIL");

  Serial.println("SAFE ready");
  alerts_init();
ALERTS_LED_ENABLED = true;  ALERTS_BUZZ_ENABLED = true;
#if defined(ESP_PLATFORM)
  ble_setup_ctrl();
#endif

  net_setup();
}

// ===== Lectures =====
float read_env_c(){ return bme_ok ? bme->readTemperature() : NAN; }
float read_env_rh(){ return bme_ok ? bme->readHumidity()    : NAN; }
float read_env_hpa(){return bme_ok ? bme->readPressure()/100.0f : NAN; }

float read_skin_c(){
  if (!ds_ok) return NAN;

  ds->requestTemperaturesByAddress(DS_ADDR);   // lance la conversion (bloque ~190 ms)
  float t = ds->getTempC(DS_ADDR);             // lit la valeur

  // Codes d'erreur ou valeur non prête
  if (t == DEVICE_DISCONNECTED_C) return NAN;  // -127.0°C
  if (t == 85.0f) return NAN;                  // 85°C = conversion non faite

  // Garde seulement des valeurs plausibles pour la peau
  if (t < 15 || t > 50) return NAN;
  return t;
}

float read_skin_c_raw(){
  if (!ds_ok) return NAN;
  ds->requestTemperaturesByAddress(DS_ADDR);
  float t = ds->getTempC(DS_ADDR);
  if (t == DEVICE_DISCONNECTED_C) return NAN;  // -127°C
  if (t == 85.0f) return NAN;                  // pas de conversion
  return t;
}

float read_voc_index(int* srawOut, float tempC, float rh) {
  if (!sgp_ok){ if (srawOut) *srawOut=-1; return NAN; }
  uint16_t sraw = sgp->measureRaw(isnan(tempC)?25.0f:tempC, isnan(rh)?50.0f:rh);
  if (srawOut) *srawOut = (int)sraw;
  int32_t idx = voc_index_from_sraw((int32_t)sraw); // 0..500
  if (idx < 0)   idx = 0;
  if (idx > 500) idx = 500;
  return (float)idx;
}

float read_co2_ppm(float* out_rh = nullptr){
  if (!scd_ok) return NAN;

  uint16_t co2 = 0;
  float tC = 0.0f, rH = 0.0f;

  int16_t ret = sensor.readMeasurement(co2, tC, rH);
  if (ret != NO_ERROR) return NAN;  
  if (co2 == 0) return NAN;         

  if (out_rh) *out_rh = rH;
  return (float)co2;
}

bool read_mpu(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& amag){
  if(!mpu_ok) return false;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  ax = a.acceleration.x;  ay = a.acceleration.y;  az = a.acceleration.z;   // m/s^2
  gx = g.gyro.x;          gy = g.gyro.y;          gz = g.gyro.z;           // rad/s
  amag = sqrtf(ax*ax + ay*ay + az*az);                                     // module
  return true;
}

void si115_service() {
  if (!si_ok) return;

  si_vis = si115.ReadVisible();
  si_ir  = si115.ReadIR();
  si_uv  = 0;
  uv_index = NAN;

  uint32_t s = (uint32_t)si_vis + (uint32_t)si_ir;

  // Auto-échelle
  static float floorE = 2.0f, ceilE = 50.0f;
  const float A_FAST = 0.10f, A_SLOW = 0.01f;
  if (s > ceilE)   ceilE  = (1-A_FAST)*ceilE  + A_FAST*s; else ceilE  = (1-A_SLOW)*ceilE  + A_SLOW*s;
  if (s < floorE)  floorE = (1-A_FAST)*floorE + A_FAST*s; else floorE = (1-A_SLOW)*floorE + A_SLOW*s;
  float span = ceilE - floorE; if (span < 10.0f) span = 10.0f;

  float x = (s - floorE) / span;           // 0..1
  if (x < 0) x = 0; if (x > 1) x = 1;
  sun_proxy = x;

  // "Couvert" = noir quasi total ET valeurs brutes très basses
  si_covered = (s <= floorE + 1.5f) && (si_vis < 2) && (si_ir < 2);

  // Dehors = vraiment lumineux ; sinon c'est intérieur/ombre/nuageux
  si_outdoor = (!si_covered && sun_proxy >= 0.55f);

  
  // sunny / cloudy / dim / covered
  const char* ambient = si_covered ? "covered" :
                        (sun_proxy >= 0.55f) ? "sunny" :
                        (sun_proxy >= 0.25f) ? "cloudy" : "dim";
}

void update_steps_activity_posture(float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float amag, uint32_t now_ms) {
  // Guard
  if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
    activity_state = ACT_STILL;
    return;
  }

  if (!(amag > 0.5f) || !isfinite(amag)) {
    amag = sqrtf(ax*ax + ay*ay + az*az);
  }

  // --- Gravity low-pass (estimate g) ---
  if (!g_init) { g_lp_x=ax; g_lp_y=ay; g_lp_z=az; g_init=true; }
  g_lp_x = (1.0f-G_LP_ALPHA)*g_lp_x + G_LP_ALPHA*ax;
  g_lp_y = (1.0f-G_LP_ALPHA)*g_lp_y + G_LP_ALPHA*ay;
  g_lp_z = (1.0f-G_LP_ALPHA)*g_lp_z + G_LP_ALPHA*az;

  float gnorm = sqrtf(g_lp_x*g_lp_x + g_lp_y*g_lp_y + g_lp_z*g_lp_z);
  if (gnorm < 1e-3f) gnorm = 1e-3f;

  // --- Posture (IMPORTANT: match working sketch)
  // In your minimal code: 0° = lying, 90° = standing (because Z aligns with g when lying on your setup)
  float cosZ = fabsf(g_lp_z) / gnorm;
  if (cosZ > 1.0f) cosZ = 1.0f;
  float tilt_deg = acosf(cosZ) * 57.2958f;     // 0..90
  if      (tilt_deg < 35) posture_state = POST_LYING;
  else if (tilt_deg < 65) posture_state = POST_SITTING;
  else                    posture_state = POST_STANDING;

  // Track last time not lying (for "soft-drop")
  if (posture_state != POST_LYING) t_last_upright_ms = now_ms;

  // --- Steps: project along g then high-pass ---
  float gxhat = g_lp_x / gnorm, gyhat = g_lp_y / gnorm, gzhat = g_lp_z / gnorm;
  float a_par     = ax*gxhat + ay*gyhat + az*gzhat; // ~9.81 when static
  float a_par_dyn = a_par - gnorm;
  static float prev_hp = 0.0f;
  a_par_hp = (1.0f-HP_ALPHA)*a_par_hp + HP_ALPHA*a_par_dyn;

  // Peak detection with refractory
  uint16_t dt_ms = now_ms - last_step_ms;
  bool rising = (a_par_hp > STEP_PEAK && prev_hp <= STEP_PEAK);
  if (rising && dt_ms >= STEP_MIN_MS && dt_ms <= STEP_MAX_MS) {
    step_count++;
    last_step_interval_ms = dt_ms;
    last_step_ms = now_ms;
  }
  prev_hp = a_par_hp;

  // --- Robust "moving" vs "still" (like the test sketch) ---
  float dyn = fabsf(amag - G);
  float thr_dyn  = NO_MOTION_DYN;
  float thr_gyro = NO_MOTION_GYRO;
  if (posture_state == POST_LYING) {
    thr_dyn  *= 2.0f;   // ignore mattress/bed rebounds
    thr_gyro *= 3.0f;
  }
  float gyro_sum = fabsf(gx) + fabsf(gy) + fabsf(gz);
  bool moving = (dyn > thr_dyn) || (gyro_sum > thr_gyro);

  // Anti-rebound: 700 ms after an impact, ignore micro-motion
  if (t_impact_ms && (now_ms - t_impact_ms) < 700) {
    moving = false;
  }

  // Update robust inactivity counter (20 ms steps expected from 50 Hz caller)
  if (!moving) {
    still_ms = (still_ms + IMU_PERIOD_MS);
    if (still_ms > 60000) still_ms = 60000;
  } else {
    still_ms = 0;
  }

  // --- Activity classification (same as your working sketch) ---
  uint32_t since_step = now_ms - last_step_ms;
  if (since_step < 2500) {
    if (last_step_interval_ms > 0 && last_step_interval_ms < 450) activity_state = ACT_RUN;
    else                                                           activity_state = ACT_WALK;
  } else if (!moving) {
    activity_state = ACT_STILL;
  } else {
    // moving but no recent steps (wrist/hand movements): fallback to WALK
    activity_state = ACT_WALK;
  }

  // Also maintain t_last_motion_ms for legacy logic elsewhere
  if (moving) t_last_motion_ms = now_ms;
}

void ppg_init(){
  delay(50);
  Wire.setClock(400000);
  if (!ppg.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    // Retry à 100 kHz si pull-up faible
    Wire.setClock(100000);
    if (!ppg.begin(Wire, I2C_SPEED_STANDARD, 0x57)) {
      Serial.println("MAX3010x FAIL (not found)");
      ppg_ok = false;
      return;
    }
  }

  byte sampleAverage = 8;
byte ledMode       = 2;     // <-- mode 2 = Rouge + IR (sans vert)
int  sampleRate    = 100;
int  pulseWidth    = 411;
int  adcRange      = 16384;

  // Démarrage
  ppg.setup(ppg_irDrive, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  ppg.setPulseAmplitudeIR(ppg_irDrive);      // IR auto-ajustée ensuite
  ppg.setPulseAmplitudeRed(ppg_redDrive);    // Rouge un peu plus fort pour SpO2
  ppg.setPulseAmplitudeGreen(0);
  ppg.clearFIFO();

  // Baselines/DC de départ (mieux que baseline “delta”)
  long sIR=0, sRED=0;
  for (int i=0;i<32;i++){ sIR += (long)ppg.getIR(); sRED += (long)ppg.getRed(); delay(5); }
  dc_ir  = (float)(sIR/32);
  dc_red = (float)(sRED/32);
  rms_ir = 0; rms_red = 0; spo2_n = 0;

  ppg_ok = true;
  Serial.println("MAX3010x OK (BPM+SpO2)");

  // On repasse le bus à 100 kHz pour cohabiter avec SCD40/BME/SGP
  Wire.setClock(100000);
}

void ppg_service(){
  if (!ppg_ok) return;

  // Cadence fixe ~100 Hz
  uint32_t now = millis();
  if ((int32_t)(now - ppg_next_ms) < 0) return;
  ppg_next_ms = now + PPG_PERIOD_MS;

  // Lecture directe (comme dans ton sketch qui marche)
  long ir  = (long)ppg.getIR();
  long red = (long)ppg.getRed();

  // Auto-gain IR pour garder la DC dans une fenêtre sûre
  if (ir > PPG_IR_HIGH && ppg_irDrive > 0x08) {
    ppg_irDrive -= 0x08; ppg.setPulseAmplitudeIR(ppg_irDrive);
  } else if (ir < PPG_IR_LOW && ppg_irDrive < 0xF0) {
    ppg_irDrive += 0x08; ppg.setPulseAmplitudeIR(ppg_irDrive);
  }

  // --- Construction du signal AC IR (retire la DC) ---
  const float ALPHA_DC = 0.02f;             // DC suit lentement (~0.5 s)
  dc_ir = (1.0f-ALPHA_DC)*dc_ir + ALPHA_DC*(float)ir;
  float ac_ir = (float)ir - dc_ir;

  // ---- Détection contact ROBUSTE (DC + pulsation) ----
  static float ac_abs_avg = 0.0f;           // moyenne glissante de |AC|
  const float ALPHA_ACABS = 0.10f;
  ac_abs_avg = (1.0f-ALPHA_ACABS)*ac_abs_avg + ALPHA_ACABS*fabsf(ac_ir);

  static int yes_cnt = 0, no_cnt = 0;
  bool cond_on  = (dc_ir > DC_CONTACT_MIN) && (ac_abs_avg > AC_CONTACT_MIN);
  bool cond_off = (dc_ir < DC_LOSS_MIN)    || (ac_abs_avg < AC_LOSS_MIN);

  if (cond_on)  { yes_cnt++; no_cnt = 0; } else { yes_cnt = 0; }
  if (cond_off) { no_cnt++;  yes_cnt = 0; } else { /* keep */ }

  if (!ppg_contact && yes_cnt >= CONTACT_ON_SAMPLES)  ppg_contact = true;   // ~80 ms @100 Hz
  if ( ppg_contact && no_cnt  >= CONTACT_OFF_SAMPLES) {                     // ~200 ms
    ppg_contact = false;
    ppg_rateSpot = 0; ppg_rateFilled = 0; ppg_bpm = 0; ppg_bpm_avg = 0;     // reset propre
  }

  // Option d’économie / anti-faux positifs : baisse le courant IR sans contact
  if (!ppg_contact && ppg_irDrive != PPG_DRIVE_MIN) {
  ppg_irDrive = PPG_DRIVE_MIN;   // 0x20
  ppg.setPulseAmplitudeIR(ppg_irDrive);
}

  // Enveloppe et seuil adaptatif
  ppg_env = (1.0f-PPG_ENV_ALPHA)*ppg_env + PPG_ENV_ALPHA*fabsf(ac_ir);
  float thr = PPG_THR_RATIO * ppg_env;

  // Détection de pic sur montée (crossing thr), avec période réfractaire
  if (ppg_contact &&
      ac_ir > thr && ppg_prev_ac <= thr &&
      (now - ppg_lastBeat) > PPG_REFRACT_MS) {

    unsigned long beatMs = now - ppg_lastBeat;
    ppg_lastBeat = now;

    if (beatMs > 250 && beatMs < 2000) {    // 30..240 BPM
      float inst = 60000.0f / (float)beatMs;
      ppg_bpm = inst;

      ppg_rates[ppg_rateSpot++] = (byte)inst;
      if (ppg_rateSpot >= 8) ppg_rateSpot = 0;
      if (ppg_rateFilled < 8) ppg_rateFilled++;

      int s = 0; for (int i=0;i<ppg_rateFilled;i++) s += ppg_rates[i];
      ppg_bpm_avg = (ppg_rateFilled > 0) ? (s / ppg_rateFilled) : 0;
    }
  }
  ppg_prev_ac = ac_ir;

  // --- SpO2 (approx) : on réutilise ac_ir + on met à jour le canal Rouge ---
  const float ALPHA_RMS = 0.05f;
  dc_red = (1.0f-ALPHA_DC)*dc_red + ALPHA_DC*(float)red;
  float ac_red = (float)red - dc_red;

  rms_ir  = (1.0f-ALPHA_RMS)*rms_ir  + ALPHA_RMS*(ac_ir*ac_ir);
  rms_red = (1.0f-ALPHA_RMS)*rms_red + ALPHA_RMS*(ac_red*ac_red);
  spo2_n++;

  if (spo2_n >= 100) { // ~1 s @ 100 Hz
    float acIR  = sqrtf(fmaxf(rms_ir,  0.0f));
    float acRED = sqrtf(fmaxf(rms_red, 0.0f));

    if (ppg_contact && dc_ir>15000 && dc_red>15000 && acIR>300 && acRED>150) {
      float R  = (acRED/dc_red) / (acIR/dc_ir);
      float sp = 104.0f - 17.0f * R;
      if (sp < 70.0f) sp = 70.0f;
      if (sp > 100.0f) sp = 100.0f;
      spo2_value = sp;

      float snr = (acIR>0) ? (acIR / sqrtf(dc_ir)) : 0.0f;
      spo2_quality = (snr > 50) ? 3 : (snr > 20) ? 2 : 1;
    } else {
      spo2_value = NAN; spo2_quality = 0;
    }
    spo2_n = 0;
  }

  // Reset quand on perd le contact
  if (!ppg_contact) {
    ppg_rateSpot = 0; ppg_rateFilled = 0; ppg_bpm = 0; ppg_bpm_avg = 0;
    ppg_env = 0; ppg_prev_ac = 0;
  }
}

void onwrist_update_robuste(float skinC, float airC, bool ppg_contact, float dc_ir, int motion) {
  static float skin_lp = NAN, skin_lp_prev = NAN;
  static uint32_t t_prev = 0, arm_t0 = 0;
  uint32_t now = millis();

  // Lissage peau + dérivée dT/dt
  if (!isnan(skinC)) skin_lp = isnan(skin_lp) ? skinC : (0.9f*skin_lp + 0.1f*skinC);
  float dTdt = 0.0f;
  if (!isnan(skin_lp)) {
    if (!isnan(skin_lp_prev) && t_prev) {
      float dt = (now - t_prev) / 1000.0f; if (dt < 0.5f) dt = 0.5f;
      dTdt = (skin_lp - skin_lp_prev) / dt;
    }
    skin_lp_prev = skin_lp;
  }
  t_prev = now;

  bool skinOK = !isnan(skin_lp) && skin_lp > 20.0f && skin_lp < 45.0f;
  bool airOK  = !isnan(airC);
  float dSA   = (skinOK && airOK) ? (skin_lp - airC) : NAN; // peau - air
  bool near_skin_ppg = ppg_contact || (dc_ir > 20000);      // proximité via IR

  // Score 0..1 composé
  float s = 0.0f;
  if (skinOK && skin_lp >= 29.0f && skin_lp <= 36.5f) s += 0.35f;             // peau dans plage humaine
  if (!isnan(dSA) && dSA > 3.0f)                    s += 0.35f;               // peau nettement > air
  else if (!isnan(dSA) && dSA > 1.0f)               s += 0.15f;               // petit delta
  if (dTdt > 0.03f)                                 s += 0.15f;               // la peau se réchauffe
  if (near_skin_ppg)                                s += 0.35f;               // capteur très proche
  // (optionnel) si tu exposes bme_heat_up : s += (bme_heat_up > 1.0f) ? 0.15f : 0.0f;
  if (motion)                                       s -= 0.10f;               // évite faux positifs en grand mouvement
  if (s < 0) s = 0; if (s > 1) s = 1;

  // Lissage & hystérésis temporelle
  static float s_lp = 0.0f;
  s_lp = 0.8f*s_lp + 0.2f*s;                         // lissage

  if (!onWrist) {
    if (s_lp >= 0.60f) { if (!arm_t0) arm_t0 = now; if (now - arm_t0 > 15000) onWrist = true; }
    else arm_t0 = 0;
  } else {
    if (s_lp <= 0.35f) { onWrist = false; arm_t0 = 0; }
  }

  // Expose pour debug/JSON
  onwrist_score = s_lp;
  onwrist_dSA   = dSA;
  onwrist_dTdt  = dTdt;
}

void ambient_update(float skinC) {
  if (!bme_ok) { env_c_out=NAN; rh_out_corr=NAN; hpa_out=NAN; return; }

  // Mesure forcée (MODE_FORCED)
  bme->takeForcedMeasurement();
  float T_bme = bme->readTemperature();
  float RH_raw = bme->readHumidity();
  hpa_out = bme->readPressure() / 100.0f;

  // Cold-floor : suit vite quand ça refroidit, monte lentement (auto-échauffement PCB)
  if (isnan(T_floor)) T_floor = T_bme;
  if (T_bme < T_floor) T_floor += K_DOWN * (T_bme - T_floor);
  else                 T_floor += K_UP   * (T_bme - T_floor);
  if (T_floor > T_bme) T_floor = T_bme;

  float T_air_est = T_floor;

  // Lissage d'affichage
  if (isnan(T_air_est_prev)) T_air_est_prev = T_air_est;
  T_air_est = (1.0f - ALPHA_SMOOTH) * T_air_est_prev + ALPHA_SMOOTH * T_air_est;
  T_air_est_prev = T_air_est;

  // AmbientC = estimée + offset fixe
  float T_air_final = T_air_est + T_FIXED_OFFSET;

  // RH final = reprojetée à T_air_final + gain/offset
  float RH_corr  = rh_retarget(RH_raw, T_bme, T_air_final);
  float RH_final = RH_GAIN * RH_corr + RH_OFFSET;
  if (RH_final < 0) RH_final = 0; if (RH_final > 100) RH_final = 100;

  // Remonte les sorties
  env_c_out   = T_air_final;   // => "AmbientC"
  rh_out_corr = RH_final;      // => "RH final"
}

void update_fall_and_unconscious(float amag, float gyro_sum,
                                 Posture posture, int activity,
                                 int bpm_pub, int spo2_pub,
                                 bool ppg_has_contact, uint32_t now) {
  // -------- Impact detection (with lock) ----------
  static uint32_t last_impact_mark = 0;
  bool impact = (amag > IMPACT_G * G) && ((now - last_impact_mark) > IMPACT_LOCK_MS);
  if (impact) {
    t_impact_ms = now;
    last_impact_mark = now;
    Serial.printf("[FALL] impact! amag=%.2f g=%.2f\n", amag, amag/9.81f);
  }

  // -------- Lying tracker ----------
  static Posture prev_post = POST_UNKNOWN;
  if (posture == POST_LYING) {
    if (prev_post != POST_LYING) t_lying_since_ms = now;
  } else {
    t_lying_since_ms = 0;
  }
  prev_post = posture;

  // -------- Decisions (mirror minimal sketch) ----------
  bool had_recent_impact = (t_impact_ms && (now - t_impact_ms) < 8000);
  bool lying_confirmed   = (t_lying_since_ms && (now - t_lying_since_ms) > LYING_CONFIRM_MS);
  bool inactive_enough   = (still_ms >= INACT_AFTER_MS);
  bool soft_drop         = ((now - t_last_upright_ms) < 2000) && lying_confirmed;

  fall_event = ((had_recent_impact || soft_drop) && inactive_enough);

  // -------- Unconsciousness scoring (unchanged, but gate with inactivity) ----------
  float s = 0.0f;
  if (lying_confirmed) s += 0.35f;
  if (inactive_enough) s += 0.35f;

  bool hr_bad   = (bpm_pub <= 0 || bpm_pub < HR_BRADY);
  bool spo2_bad = (spo2_pub >= 0 && spo2_pub < SPO2_LOW);
  if (hr_bad)   s += 0.20f;
  if (spo2_bad) s += 0.35f;

  if (!ppg_has_contact) s -= 0.10f;

  if (s < 0) s = 0; if (s > 1) s = 1;
  unconscious_score = 0.8f * unconscious_score + 0.2f * s;

  static uint32_t t_score_high = 0;
  if (unconscious_score >= 0.75f) {
    if (!t_score_high) t_score_high = now;
    unconscious = ((now - t_score_high) > UNCON_MIN_MS);
  } else {
    t_score_high = 0;
    if (still_ms < 500 || posture != POST_LYING) unconscious = false;
  }

  // -------- 1 Hz debug line (matches your test logs) ----------
  static uint32_t next_dbg = 0;
  if ((int32_t)(now - next_dbg) >= 0) {
    next_dbg = now + 1000;
    Serial.printf("[FALL] hadImp=%d lying=%d inactive=%d | still_ms=%u fall=%d | a_hp=%.2f steps=%lu act=%d\n",
      (int)had_recent_impact, (int)lying_confirmed, (int)inactive_enough,
      (unsigned)still_ms, (int)fall_event, a_par_hp,
      (unsigned long)step_count, (int)activity_state);
  }
}

struct SimPI {
  float skin = 33.2f;
  int   bpm  = 78;
  int   spo2 = 97;
  bool  contact = true;
  int   drive_lvl = 1;
  bool  on_wrist = true;
  float dSA = 3.5f;
  float dTdt = 0.00f;
} simpi;

static inline float rwalk(float v, float step, float lo, float hi){
  v += ((float)random(-100,101)/100.0f)*step;
  if (v<lo) v=lo; if (v>hi) v=hi; return v;
}
void simpi_update(float envC){
  static uint32_t next=0; uint32_t now=millis();
  if ((int32_t)(now-next)>=0){
    next = now + 1000; // 1 Hz
    simpi.skin = rwalk(simpi.skin, 0.06f, 31.0f, 35.0f);
    float air = isnan(envC)?26.0f:envC;
    simpi.dSA  = simpi.skin - air;
    simpi.dTdt = rwalk(simpi.dTdt, 0.005f, -0.05f, 0.05f);
    simpi.on_wrist = true;
    simpi.contact  = true;
    simpi.bpm  = (int)roundf(rwalk(simpi.bpm, 1.2f, 60, 110));
    simpi.spo2 = (int)roundf(rwalk(simpi.spo2, 0.4f, 95, 100));
    simpi.drive_lvl = (simpi.bpm>95)?2:1;
  }
}

void loop() {
  net_loop();
  alerts_update();
  static uint32_t last = 0;
  uint32_t now = millis();

#if BIOMETRICS_ENABLED
  ppg_service();
#endif
  si115_service();

  // ===== Service IMU rapide (50 Hz) =====
  if (mpu_ok && (int32_t)(now - imu_next_ms) >= 0) {
    imu_next_ms = now + IMU_PERIOD_MS;

    // Lecture IMU
    read_mpu(ax_g, ay_g, az_g, gx_g, gy_g, gz_g, amag_g);

    // Mouvements & orientation (face ciel)
    gyro_sum_g = (isnan(gx_g)||isnan(gy_g)||isnan(gz_g)) ? 0.0f
                                                         : (fabsf(gx_g)+fabsf(gy_g)+fabsf(gz_g));
    float accel_dyn_g = (!isnan(amag_g)) ? fabsf(amag_g - 9.81f) : 0.0f;
    motion_g = (accel_dyn_g > 0.6f || gyro_sum_g > 0.6f) ? 1 : 0;

    face_g = 0.0f;
    if (!isnan(amag_g) && amag_g > 5.0f) {
      float cosTheta = (-az_g) / amag_g;   // Z vers le sol sur ton montage
      if (cosTheta < 0) cosTheta = 0;
      if (cosTheta > 1) cosTheta = 1;
      face_g = cosTheta;                   // 0..1
    }

    // Steps / Activity / Posture (50 Hz)
    update_steps_activity_posture(ax_g, ay_g, az_g, gx_g, gy_g, gz_g, amag_g, now);

    // Chute / Inconscience (50 Hz)
    update_fall_and_unconscious(
      amag_g, gyro_sum_g,
      posture_state, activity_state,
      /* bpm_pub  */ ppg_contact ? ((int)roundf(ppg_bpm/5.0f)*5) : 0,
      /* spo2_pub */ (isnan(spo2_value) ? -1 : (int)roundf(spo2_value)),
      /* ppg contact */ ppg_contact,
      now
    );
    static uint32_t last_fall_play = 0;
if (fall_event && (millis() - last_fall_play > 10000)) { // max 1 fois / 10 s
  alerts_play_kind(ALERT_FALL);
  last_fall_play = millis();
}
  }

  // ===== Sorties 1 Hz =====
  if (now - last >= 1000) {
    last = now;

    // 1) Peau d’abord, puis ambiante corrigée
#if STRICT_PI
    float skin_raw = NAN;
    float skin     = NAN;
#else
    float skin_raw = read_skin_c_raw();   // DS18B20
    float skin     = (!isnan(skin_raw) && skin_raw >= 10 && skin_raw <= 50) ? skin_raw : NAN;
#endif

    ambient_update(skin_raw);        // calcule env_c_out & rh_out_corr & hpa_out
    float envC = env_c_out;          // AmbientC
    float rh   = rh_out_corr;        // RH final
    float hpa  = hpa_out;

#if STRICT_PI && SIM_PI
    simpi_update(envC);
#endif

    // 2) VOC (utilise l’ambiante corrigée si dispo)
    int   voc_sraw = -1;
    float voc_idx  = read_voc_index(&voc_sraw, envC, rh);

    // 3) SCD40 (espacé)
    float rh_scd = NAN;
    float co2    = NAN;
    if ((int32_t)(now - scd_next_read_ms) >= 0) {
      float rh_tmp = NAN;
      float co2_new = read_co2_ppm(&rh_tmp);           // NAN si pas prêt
      if (!isnan(co2_new)) { co2 = co2_new; rh_scd = rh_tmp; scd_next_read_ms = now + 5500; }
      else                 { scd_next_read_ms = now + 500; }
    }
    // Fallback RH via SCD40 si BME indispo
    float rh_out = isnan(rh) ? rh_scd : rh;

    // 4) Score & dose soleil (utilise face_g & motion_g mis à jour à 50 Hz)
    sun_score = (!si_covered ? (sun_proxy * face_g) : 0.0f);
    if (motion_g) sun_score *= 0.8f;   // anti-faux positifs
    sun_touch = (sun_score > 0.35f);
    sun_dose += (uint32_t)(sun_score * 100);

    // 5) On-wrist robuste (utilise motion_g)
#if STRICT_PI
    onWrist       = false;
    onwrist_score = NAN;
    onwrist_dSA   = NAN;
    onwrist_dTdt  = 0.0f;
#else
    onwrist_update_robuste(skin_raw, envC, ppg_contact, dc_ir, motion_g);
#endif

    // 6) Impression JSON (DEMO vs RAW)
#if DEMO_MODE
    // ---- Versions SAFE (arrondies/bucketisées) ----
    uint32_t ts_pub   = (now/1000/60)*60;   // arrondi à la minute
    float    envC_pub = qf(envC, 0.5f);
    float    rh_pub   = qf(rh_out, 1.0f);
    float    hpa_pub  = qf(hpa, 1.0f);
    float    skin_pub = qf(skin, 0.5f);
    int      bpm_pub  = ppg_contact ? ((int)roundf(ppg_bpm/5.0f)*5) : 0;
    int      spo2_pub = isnan(spo2_value) ? -1 : (int)qf(spo2_value, 1.0f);
    int      spo2q_pub= (int)spo2_quality;
    int      ppg_drive_lvl = bucket_ppg_drive(ppg_irDrive);
    int      voc_pub  = isnan(voc_idx) ? -1 : (int)qf(voc_idx, 5.0f);
    int      co2_pub  = isnan(co2) ? -1 : (int)(roundf(co2/50.0f)*50);
    float    dSA_pub  = qf(onwrist_dSA, 0.01f);
    float    dTdt_pub = qf(onwrist_dTdt, 0.001f);
    float    sun_proxy_pub = qf(sun_proxy, 0.01f);
    float    sun_score_pub = qf(sun_score, 0.01f);

    Serial.print("{");
    Serial.print("\"ts\":");        Serial.print(ts_pub);                    Serial.print(",");
    Serial.print("\"env_c\":");     Serial.print(isnan(envC_pub)?String("null"):String(envC_pub,2)); Serial.print(",");
    Serial.print("\"rh\":");        Serial.print(isnan(rh_pub)?String("null"):String(rh_pub,0));     Serial.print(",");
    Serial.print("\"hpa\":");       Serial.print(isnan(hpa_pub)?String("null"):String(hpa_pub,0));   Serial.print(",");

    // PI simulée / neutralisée / réelle SAFE
  #if STRICT_PI && SIM_PI
    Serial.print("\"skin_c\":");    Serial.print(simpi.skin,2);  Serial.print(",");
    Serial.print("\"bpm\":");       Serial.print(simpi.bpm);     Serial.print(",");
    Serial.print("\"spo2\":");      Serial.print(simpi.spo2);    Serial.print(",");
    Serial.print("\"spo2_q\":");    Serial.print(2);             Serial.print(",");
    Serial.print("\"ppg_contact\":");Serial.print(simpi.contact?1:0); Serial.print(",");
    Serial.print("\"ppg_drive_lvl\":");Serial.print(simpi.drive_lvl);  Serial.print(",");
  #elif STRICT_PI
    Serial.print("\"skin_c\":null,");
    Serial.print("\"bpm\":0,");
    Serial.print("\"spo2\":null,");
    Serial.print("\"spo2_q\":0,");
    Serial.print("\"ppg_contact\":0,");
  #else
    Serial.print("\"skin_c\":");    Serial.print(isnan(skin_pub)?String("null"):String(skin_pub,2)); Serial.print(",");
    Serial.print("\"bpm\":");       Serial.print(bpm_pub);                         Serial.print(",");
    Serial.print("\"spo2\":");      Serial.print((spo2_pub<0)?String("null"):String(spo2_pub)); Serial.print(",");
    Serial.print("\"spo2_q\":");    Serial.print(spo2q_pub);                       Serial.print(",");
    Serial.print("\"ppg_contact\":");Serial.print(ppg_contact?1:0);                 Serial.print(",");
    Serial.print("\"ppg_drive_lvl\":");Serial.print(ppg_drive_lvl);                 Serial.print(",");
  #endif

    // Air & gaz (agrégés)
    Serial.print("\"voc\":");       Serial.print((voc_pub<0)?String("null"):String(voc_pub)); Serial.print(",");
    Serial.print("\"co2\":");       Serial.print((co2_pub<0)?String("null"):String(co2_pub));

    // IMU non détaillée + lumière & motion (motion_g!)
    Serial.print(",\"motion\":");   Serial.print(motion_g);
    Serial.print(",\"vis\":");      Serial.print(si_ok ? (int)si_vis : -1);
    Serial.print(",\"ir\":");       Serial.print(si_ok ? (int)si_ir  : -1);
    Serial.print(",\"covered\":");  Serial.print(si_covered?1:0);
    Serial.print(",\"outdoor\":");  Serial.print(si_outdoor?1:0);
    Serial.print(",\"sun_proxy\":");Serial.print(sun_proxy_pub,2);
    Serial.print(",\"sun_dose\":"); Serial.print(sun_dose);
    Serial.print(",\"sun_score\":");Serial.print(sun_score_pub,2);
    Serial.print(",\"sun_touch\":");Serial.print(sun_touch?1:0);

    // On-wrist
  #if STRICT_PI && SIM_PI
    Serial.print(",\"on_wrist\":"); Serial.print(simpi.on_wrist?1:0);
    Serial.print(",\"dSA\":");      Serial.print(simpi.dSA,2);
    Serial.print(",\"dTdt\":");     Serial.print(simpi.dTdt,3);
  #elif STRICT_PI
    Serial.print(",\"on_wrist\":0");
  #else
    Serial.print(",\"on_wrist\":"); Serial.print(onWrist?1:0);
    Serial.print(",\"dSA\":");      Serial.print(isnan(dSA_pub)?String("null"):String(dSA_pub,2));
    Serial.print(",\"dTdt\":");     Serial.print(dTdt_pub,3);
  #endif

  #if DEMO_LOCATION
    Serial.print(",\"gps\":{\"lat\":"); Serial.print(DEMO_LAT, 6);
    Serial.print(",\"lon\":");          Serial.print(DEMO_LON, 6);
    Serial.print("}");
    Serial.print(",\"privacy\":{\"use_demo_location\":true}");
  #endif

    // Activity / Posture / Steps
    Serial.print(",\"steps\":");    Serial.print(step_count);
    Serial.print(",\"activity\":\"");
    switch (activity_state) {
      case ACT_WALK: Serial.print("walk"); break;
      case ACT_RUN:  Serial.print("run");  break;
      default:       Serial.print("still"); break;
    }
    Serial.print("\",");

    Serial.print("\"posture\":\"");
    switch (posture_state) {
      case POST_STANDING: Serial.print("standing"); break;
      case POST_SITTING:  Serial.print("sitting");  break;
      case POST_LYING:    Serial.print("lying");    break;
      default:            Serial.print("unknown");  break;
    }
    Serial.print("\"");

    // Fall / Unconscious fields
    Serial.print(",\"fall_event\":");       Serial.print(fall_event ? 1 : 0);
    Serial.print(",\"unconscious\":");      Serial.print(unconscious ? 1 : 0);
    Serial.print(",\"unconscious_score\":");Serial.print(unconscious_score, 2);

    Serial.print(",\"imu_ok\":"); Serial.print(mpu_ok?1:0);

    Serial.println("}");

    // ---- ENVOI SAFE toutes les 30 s (attention à motion_g ci-dessous) ----
    if (millis() - last_push_ms >= PUSH_PERIOD_MS) {
      last_push_ms = millis();

      float skin_to_send =
      #if STRICT_PI && SIM_PI
        simpi.skin;
      #else
        skin_pub;
      #endif

      int hr_to_send =
      #if STRICT_PI && SIM_PI
        simpi.bpm;
      #else
        bpm_pub;
      #endif

      int spo2_to_send =
      #if STRICT_PI && SIM_PI
        simpi.spo2;
      #else
        (spo2_pub < 0 ? 0 : spo2_pub);
      #endif

      double lat_send = DEMO_LAT;
      double lon_send = DEMO_LON;

      String json = String("{")
        + "\"userId\":\"" + String(USER_ID) + "\","
        + "\"hr\":"         + String(hr_to_send) + ","
        + "\"spo2\":"       + String(spo2_to_send) + ","
        + "\"temp_skin\":"  + f2json(skin_to_send) + ","
        + "\"env_c\":"      + f2json(envC) + ","
        + "\"co2\":"        + (co2_pub < 0 ? String("null") : String(co2_pub)) + ","
        + "\"voc\":"        + (voc_pub < 0 ? String("null") : String(voc_pub)) + ","
        + "\"sun_touch\":"  + String(sun_touch ? 1 : 0) + ","
        + "\"sun_proxy\":"  + f2json(sun_proxy_pub, 2) + ","
        + "\"motion\":"     + String(motion_g ? 1 : 0) + ","   // <<<<<< ICI motion_g
        + "\"gps\":{\"lat\":" + String(lat_send, 6) + ",\"lon\":" + String(lon_send, 6) + "}"
        + "}";

      net_send(json);
      ctrl_serial_poll(); 
    }
#else
  // ===== Mode RAW (ton bloc original) =====
  Serial.print("{");

  Serial.print("\"ts\":");        Serial.print(now/1000);                                  Serial.print(",");
  Serial.print("\"env_c\":");     Serial.print(isnan(envC)?String("null"):String(envC,2)); Serial.print(",");
  Serial.print("\"rh\":");        Serial.print(isnan(rh_out)?String("null"):String(rh_out,1)); Serial.print(",");
  Serial.print("\"hpa\":");       Serial.print(isnan(hpa)?String("null"):String(hpa,1));   Serial.print(",");
  Serial.print("\"skin_c\":");    Serial.print(isnan(skin)?String("null"):String(skin,2)); Serial.print(",");
  Serial.print("\"skin_raw\":");  Serial.print(isnan(skin_raw)?String("null"):String(skin_raw,2)); Serial.print(",");

  Serial.print("\"bpm\":");       Serial.print(ppg_contact ? (int)ppg_bpm : 0);           Serial.print(",");
  Serial.print("\"bpm_avg\":");   Serial.print(ppg_contact ? ppg_bpm_avg : 0);            Serial.print(",");
  Serial.print("\"spo2\":");      Serial.print(isnan(spo2_value)?String("null"):String(spo2_value,0)); Serial.print(",");
  Serial.print("\"spo2_q\":");    Serial.print((int)spo2_quality);                        Serial.print(",");
  Serial.print("\"ppg_contact\":");Serial.print(ppg_contact?1:0);                          Serial.print(",");
  Serial.print("\"ppg_ir_dc\":"); Serial.print((int)dc_ir);                                Serial.print(",");
  Serial.print("\"ppg_ir_drv\":");Serial.print(ppg_irDrive, HEX);                          Serial.print(",");

  Serial.print("\"voc\":");       Serial.print(isnan(voc_idx)?String("null"):String(voc_idx,0)); Serial.print(",");
  Serial.print("\"voc_sraw\":");  Serial.print((int)voc_sraw);                             Serial.print(",");

  Serial.print("\"co2\":");       Serial.print(isnan(co2)?String("null"):String(co2,0));

  Serial.print(",\"ax\":");       Serial.print(isnan(ax)?String("null"):String(ax,2));
  Serial.print(",\"ay\":");       Serial.print(isnan(ay)?String("null"):String(ay,2));
  Serial.print(",\"az\":");       Serial.print(isnan(az)?String("null"):String(az,2));
  Serial.print(",\"gx\":");       Serial.print(isnan(gx)?String("null"):String(gx,2));
  Serial.print(",\"gy\":");       Serial.print(isnan(gy)?String("null"):String(gy,2));
  Serial.print(",\"gz\":");       Serial.print(isnan(gz)?String("null"):String(gz,2));
  Serial.print(",\"a_mag\":");    Serial.print(isnan(amag)?String("null"):String(amag,2));

  Serial.print(",\"motion\":");   Serial.print(motion);
  Serial.print(",\"vis\":");      Serial.print(si_ok ? (int)si_vis : -1);
  Serial.print(",\"ir\":");       Serial.print(si_ok ? (int)si_ir  : -1);
  Serial.print(",\"sun_raw\":");  Serial.print((int)si_vis + (int)si_ir);
  Serial.print(",\"covered\":");  Serial.print(si_covered?1:0);
  Serial.print(",\"outdoor\":");  Serial.print(si_outdoor?1:0);
  Serial.print(",\"sun_proxy\":");Serial.print(sun_proxy,2);
  Serial.print(",\"sun_dose\":"); Serial.print(sun_dose);
  Serial.print(",\"sun_score\":");Serial.print(sun_score,2);
  Serial.print(",\"sun_touch\":");Serial.print(sun_touch?1:0);

  Serial.print(",\"on_wrist\":"); Serial.print(onWrist?1:0);
  Serial.print(",\"ow_score\":"); Serial.print(onwrist_score,2);
  Serial.print(",\"dSA\":");      Serial.print(isnan(onwrist_dSA)?String("null"):String(onwrist_dSA,2));
  Serial.print(",\"dTdt\":");     Serial.print(onwrist_dTdt,3);

  Serial.print(",\"fall_event\":");       Serial.print(fall_event ? 1 : 0);
  Serial.print(",\"unconscious\":");      Serial.print(unconscious ? 1 : 0);
  Serial.print(",\"unconscious_score\":");Serial.print(unconscious_score, 2);

  Serial.println("}");
#endif
  }
}