#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

// #define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
  #define DBG(...) Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
#endif

/************** CONFIG LED **************/
#define LED_PIN   16
#define NUM_LEDS  7

const uint8_t SOLO_BRIGHTNESS_MAX   = 25;    // ~10%
const uint8_t CROWD_BRIGHTNESS_MAX  = 200;   // ~80%

const float   PULSE_SPEED_MIN       = 0.35f; // Hz
const float   PULSE_SPEED_MAX       = 0.70f; // Hz
const float   HUE_DRIFT_PER_SEC     = 4.0f;

const uint8_t SATURATION_BASE       = 240;
const uint8_t SATURATION_WARM       = 255;
const float   WARM_BLEND_EXP        = 1.4f;  // fusione colori caldi

/************** PRESENZA / BLE **********/
const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -90;
const int8_t   RSSI_MAX_DBM         = -55;
const float    PRESENCE_ALPHA       = 0.20f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

/************** TIMING **************/
const uint16_t FRAME_INTERVAL_MS    = 30;    // ~33 fps
const uint32_t SCAN_PERIOD_MS       = 1200;  // ogni quanto avviare una scansione
const uint32_t SCAN_WINDOW_MS       = 350;   // durata scansione (bloccante)

/************** ADVERT DATA *************/
const uint16_t COMPANY_ID     = 0xFFFF;  // fittizio
const char     SIG[3]         = {'B','D','G'};
const uint8_t  PROTO_VERSION  = 1;

/************** STRUTTURE ***************/
struct LedState {
  float   pulsePhase;
  float   pulseSpeed;
  uint8_t baseHue8;
  uint8_t warmHue8;
};

struct BadgePeer {
  NimBLEAddress addr;
  int8_t   lastRSSI = 0;
  uint32_t lastSeen = 0;
  bool     valid    = false;
};

static const uint8_t MAX_PEERS = 16;

/************** VARIABILI ***************/
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
LedState  ledStates[NUM_LEDS];
BadgePeer peers[MAX_PEERS];
float     presenceFactor = 0.0f;
uint32_t  lastFrameMs = 0;
uint32_t  lastScanStartMs = 0;
uint8_t   myID = 0;

/************** RANDOM UTILS ************/
uint8_t rand8range(uint8_t a, uint8_t b) {
  return a + (uint8_t)(esp_random() % (b - a + 1));
}
float randFloat(float a, float b) {
  return a + ((float)esp_random() / (float)UINT32_MAX) * (b - a);
}

/************** PEERS *******************/
int findPeerSlot(const NimBLEAddress& a) {
  int freeSlot = -1;
  for (int i=0; i<MAX_PEERS; i++) {
    if (peers[i].valid && peers[i].addr == a) return i;
    if (!peers[i].valid && freeSlot < 0) freeSlot = i;
  }
  return freeSlot;
}

void purgePeers() {
  uint32_t now = millis();
  for (int i=0;i<MAX_PEERS;i++){
    if (peers[i].valid && (now - peers[i].lastSeen) > BEACON_TIMEOUT_MS) {
      peers[i].valid = false;
    }
  }
}

float rssiToWeight(int8_t rssi) {
  if (rssi <= RSSI_MIN_DBM) return 0.0f;
  if (rssi >= RSSI_MAX_DBM) return 1.0f;
  return (float)(rssi - RSSI_MIN_DBM) / (float)(RSSI_MAX_DBM - RSSI_MIN_DBM);
}

float computePresenceRaw() {
  purgePeers();
  float sum = 0.0f;
  for (int i=0;i<MAX_PEERS;i++){
    if (!peers[i].valid) continue;
    sum += rssiToWeight(peers[i].lastRSSI);
  }
  float normalized = sum / (float)PEERS_FOR_FULL_SCALE;
  if (normalized > 1.0f) normalized = 1.0f;
  return normalized;
}

/************** BLE SETUP ***************/
void setupBLE() {
  NimBLEDevice::init("Badge");
  myID = (uint8_t)(esp_random() & 0xFF);

  // Advertising
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  uint8_t payload[7];
  payload[0] = (uint8_t)(COMPANY_ID & 0xFF);
  payload[1] = (uint8_t)(COMPANY_ID >> 8);
  payload[2] = 'B';
  payload[3] = 'D';
  payload[4] = 'G';
  payload[5] = PROTO_VERSION;
  payload[6] = myID;
  adv->setManufacturerData(std::string((char*)payload, sizeof(payload)));
  adv->start();

  // Scan object (userà polling manuale)
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(false); // passivo
  scan->setInterval(160);     // 100 ms
  scan->setWindow(120);       // 75 ms
}

/************** SCAN POLLING ************/
void doScanCycle() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  float durationSec = SCAN_WINDOW_MS / 1000.0f;
  scan->start(durationSec, false, false);

  NimBLEScanResults results = scan->getResults();
#ifdef DEBUG_SERIAL
  Serial.print("Scan results: ");
  Serial.println(results.getCount());
#endif
  for (int i=0; i<results.getCount(); i++) {
    // DIFFERENZA: getDevice(i) restituisce puntatore
    const NimBLEAdvertisedDevice* adv = results.getDevice(i);
    if (!adv) continue;
    if (!adv->haveManufacturerData()) continue;
    std::string m = adv->getManufacturerData();
    if (m.size() < 7) continue;
    const uint8_t* d = (const uint8_t*)m.data();
    uint16_t cid = d[0] | (d[1] << 8);
    if (cid != COMPANY_ID) continue;
    if (d[2] != 'B' || d[3] != 'D' || d[4] != 'G') continue;

    int slot = findPeerSlot(adv->getAddress());
    if (slot < 0) continue;

    peers[slot].addr     = adv->getAddress();
    peers[slot].lastRSSI = adv->getRSSI();
    peers[slot].lastSeen = millis();
    peers[slot].valid    = true;

#ifdef DEBUG_SERIAL
    Serial.print("Peer ");
    Serial.print(peers[slot].addr.toString().c_str());
    Serial.print(" RSSI ");
    Serial.println(peers[slot].lastRSSI);
#endif
  }
  scan->clearResults();
}

/************** LED INIT ***************/
void initLeds() {
  strip.begin();
  strip.clear();
  strip.setBrightness(255); // moduliamo solo V HSV
  strip.show();
  for (int i=0;i<NUM_LEDS;i++){
    ledStates[i].pulsePhase = randFloat(0, TWO_PI);
    float freq = randFloat(PULSE_SPEED_MIN, PULSE_SPEED_MAX);
    ledStates[i].pulseSpeed = freq * TWO_PI;
    ledStates[i].baseHue8   = rand8range(0,255);
    ledStates[i].warmHue8   = rand8range(0,45);
  }
}

/************** HUE BLEND **************/
uint8_t blendHue(uint8_t h1, uint8_t h2, float t) {
  float hf = (1.0f - t) * h1 + t * h2;
  if (hf < 0) hf = 0;
  if (hf > 255) hf = 255;
  return (uint8_t)hf;
}

/************** UPDATE LEDS ************/
void updateLeds(uint32_t nowMs, float dt) {
  float raw = computePresenceRaw();
  presenceFactor += PRESENCE_ALPHA * (raw - presenceFactor);
  if (presenceFactor < 0) presenceFactor = 0;
  if (presenceFactor > 1) presenceFactor = 1;

  float maxBrightF = SOLO_BRIGHTNESS_MAX +
                     (float)(CROWD_BRIGHTNESS_MAX - SOLO_BRIGHTNESS_MAX) *
                     powf(presenceFactor, 0.9f);

  float driftHue8 = fmodf((nowMs / 1000.0f) * HUE_DRIFT_PER_SEC, 255.0f);

  for (int i=0;i<NUM_LEDS;i++){
    auto &ls = ledStates[i];

    ls.pulsePhase += ls.pulseSpeed * dt;
    if (ls.pulsePhase > TWO_PI) ls.pulsePhase -= TWO_PI;

    float wave = (sinf(ls.pulsePhase) + 1.0f) * 0.5f;
    wave = powf(wave, 1.4f);

    float valF = 2.0f + wave * maxBrightF;
    if (valF > 255.0f) valF = 255.0f;
    uint8_t V = (uint8_t)valF;

    uint8_t dynHue = (uint8_t)(ls.baseHue8 + (uint8_t)driftHue8);
    float warmBlend = powf(presenceFactor, WARM_BLEND_EXP);
    uint8_t finalHue = blendHue(dynHue, ls.warmHue8, warmBlend);

    float Sf = SATURATION_BASE +
               (float)(SATURATION_WARM - SATURATION_BASE) * warmBlend;
    if (Sf > 255) Sf = 255;
    uint8_t S = (uint8_t)Sf;

    uint16_t hue16 = (uint16_t)finalHue * 257;
    uint32_t c = strip.ColorHSV(hue16, S, V);
    strip.setPixelColor(i, c);

    if ((nowMs & 0x1FF) == 0) {
      ls.baseHue8 += rand8range(0,2);
    }
  }
  strip.show();
}

/************** SETUP / LOOP ***********/
void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(150);
  DBG("Boot");
#endif
  randomSeed(esp_random());
  initLeds();
  setupBLE();
  lastScanStartMs = 0; // forza una prima scansione
  DBG("BLE ready");
}

void loop() {
  uint32_t now = millis();

  if (now - lastScanStartMs >= SCAN_PERIOD_MS) {
    lastScanStartMs = now;
    doScanCycle();
  }

  if (now - lastFrameMs >= FRAME_INTERVAL_MS) {
    float dt = (lastFrameMs == 0)
               ? (FRAME_INTERVAL_MS / 1000.0f)
               : (now - lastFrameMs) / 1000.0f;
    lastFrameMs = now;
    updateLeds(now, dt);
  }

  delay(1);
}