#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>

// ================== OPZIONI DEBUG ==================
// #define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
  #define DBG(...) Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
#endif

// ================== CONFIG LED ======================
#define LED_PIN   16
#define NUM_LEDS  7

// Luminosità di base (range 0..255)
const uint8_t SOLO_BRIGHTNESS_MAX   = 25;    // ~10%
const uint8_t CROWD_BRIGHTNESS_MAX  = 200;   // ~80%

// Colori / saturazione
const uint8_t SATURATION_BASE       = 240;
const uint8_t SATURATION_WARM       = 255;
const float   WARM_BLEND_EXP        = 1.4f;

// Drift cromatico psichedelico (hue deriva indipendentemente dal firefly)
const float   HUE_DRIFT_PER_SEC     = 4.0f;

// ================== PRESENZA / BLE ==================
const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -90;
const int8_t   RSSI_MAX_DBM         = -55;
const float    PRESENCE_ALPHA       = 0.20f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

// ================== FIREFLY SYNC ====================
const float FIREFLY_BASE_FREQ_HZ    = 0.55f;   // frequenza pulsazione principale
const float FIREFLY_COUPLING        = 0.08f;   // 0.05..0.15 tipico
const float FIREFLY_PHASE_JUMP_ON_FLASH = 0.0f; // opzionale, se vuoi "resettare" (0 = disabilitato)
const bool  FIREFLY_LOCK_WHEN_NEAR  = true;    // accelera sincronizzazione quando presenza>0

// ================== HEAT BLOOM ======================
const uint32_t BLOOM_DURATION_MS    = 600;
const float    BLOOM_AMPLITUDE      = 120.0f;  // extra brightness max
const float    BLOOM_WARM_EXTRA     = 0.35f;   // extra warm blending (0..1)
const float    PRESENCE_SPIKE_THRESHOLD = 0.15f; // delta presence per scatenare bloom

// ================== TIMING ==========================
const uint16_t FRAME_INTERVAL_MS    = 30;
const uint32_t SCAN_PERIOD_MS       = 1200;
const uint32_t SCAN_WINDOW_MS       = 350;
const uint32_t ADV_UPDATE_INTERVAL_MS = 300; // aggiorna phase in advertising

// ================== MANUFACTURER DATA ===============
const uint16_t COMPANY_ID     = 0xFFFF;
const char     SIG[3]         = {'B','D','G'};
const uint8_t  PROTO_VERSION  = 2;  // incrementato (aggiunta fase)

// ================== STRUTTURE DATI ==================
struct LedState {
  // Manteniamo ancora baseHue per varietà per-LED
  uint8_t baseHue8;
  uint8_t warmHue8;
};

struct BadgePeer {
  NimBLEAddress addr;
  int8_t   lastRSSI = 0;
  uint32_t lastSeen = 0;
  bool     valid    = false;
  bool     phaseValid = false;
  float    phase = 0.0f; // 0..1 letto dal peer
};

static const uint8_t MAX_PEERS = 16;

// ================== VARIABILI GLOBALI ===============
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
LedState  ledStates[NUM_LEDS];
BadgePeer peers[MAX_PEERS];

float presenceFactor       = 0.0f;
float previousPresence     = 0.0f;

uint32_t lastFrameMs       = 0;
uint32_t lastScanStartMs   = 0;
uint32_t lastAdvUpdateMs   = 0;

uint8_t  myID              = 0;

// Firefly oscillator
float fireflyPhase         = 0.0f;  // 0..1
float fireflyFreq          = FIREFLY_BASE_FREQ_HZ;

// Media fase vicini (calcolata per ogni scan cycle)
bool   neighborPhaseAvailable = false;
float  neighborMeanPhase      = 0.0f;

// Heat Bloom
uint32_t bloomEndMs        = 0;
bool     bloomActive       = false;

// Conteggio peers (per trigger bloom su nuovo arrivo)
uint8_t  lastPeerCount     = 0;

// Advertising pointer (per aggiornare phase)
NimBLEAdvertising* gAdv    = nullptr;

// ================== RANDOM UTILS ====================
uint8_t rand8range(uint8_t a, uint8_t b) {
  return a + (uint8_t)(esp_random() % (b - a + 1));
}
float randFloat(float a, float b) {
  return a + ((float)esp_random() / (float)UINT32_MAX) * (b - a);
}

// ================== PEER MANAGEMENT =================
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
      peers[i].phaseValid = false;
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

uint8_t countValidPeers() {
  uint8_t c = 0;
  for (int i=0;i<MAX_PEERS;i++) if (peers[i].valid) c++;
  return c;
}

// ================== FIREFLY HELPERS =================
static inline float wrap01(float x) {
  if (x >= 1.0f) return x - (int)x;
  if (x < 0.0f)  return x - floorf(x);
  return x;
}

// differenza fase (media - local) riportata in -0.5..+0.5 (per correzione)
static inline float phaseDiff(float target, float local) {
  float d = target - local;
  while (d >  0.5f) d -= 1.0f;
  while (d < -0.5f) d += 1.0f;
  return d;
}

// ================== HEAT BLOOM TRIGGER ==============
void triggerBloom() {
  bloomEndMs  = millis() + BLOOM_DURATION_MS;
  bloomActive = true;
}

// ================== BLE SETUP =======================
void setupBLE() {
  NimBLEDevice::init("Badge");
  myID = (uint8_t)(esp_random() & 0xFF);

  gAdv = NimBLEDevice::getAdvertising();
  // Primo set manufacturer data (fase iniziale = 0)
  uint8_t payload[8];
  payload[0] = (uint8_t)(COMPANY_ID & 0xFF);
  payload[1] = (uint8_t)(COMPANY_ID >> 8);
  payload[2] = 'B';
  payload[3] = 'D';
  payload[4] = 'G';
  payload[5] = PROTO_VERSION;
  payload[6] = myID;
  payload[7] = (uint8_t)(fireflyPhase * 255.0f);
  gAdv->setManufacturerData(std::string((char*)payload, sizeof(payload)));
  gAdv->start();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(false); // passivo
  scan->setInterval(160);
  scan->setWindow(120);
}

// Aggiorna manufacturer data con nuova fase
void updateAdvertisingPhase() {
  if (!gAdv) return;
  uint32_t now = millis();
  if (now - lastAdvUpdateMs < ADV_UPDATE_INTERVAL_MS) return;
  lastAdvUpdateMs = now;

  uint8_t payload[8];
  payload[0] = (uint8_t)(COMPANY_ID & 0xFF);
  payload[1] = (uint8_t)(COMPANY_ID >> 8);
  payload[2] = 'B';
  payload[3] = 'D';
  payload[4] = 'G';
  payload[5] = PROTO_VERSION;
  payload[6] = myID;
  payload[7] = (uint8_t)(wrap01(fireflyPhase) * 255.0f);

  gAdv->setManufacturerData(std::string((char*)payload, sizeof(payload)));
  // Non serve restart: NimBLE aggiorna l’advertising data in corso
}

// ================== SCAN POLLING =====================
// Calcola anche media di fase dei peer (sin/cos)
void doScanCycle() {
  NimBLEScan* scan = NimBLEDevice::getScan();
  float durationSec = SCAN_WINDOW_MS / 1000.0f;
  scan->start(durationSec, false, false);

  NimBLEScanResults results = scan->getResults();

  float sumSin = 0.0f, sumCos = 0.0f;
  int   phaseCount = 0;

  for (int i=0; i<results.getCount(); i++) {
    const NimBLEAdvertisedDevice* adv = results.getDevice(i);
    if (!adv) continue;
    if (!adv->haveManufacturerData()) continue;
    std::string m = adv->getManufacturerData();
    if (m.size() < 8) continue; // ora richiediamo 8 byte
    const uint8_t* d = (const uint8_t*)m.data();
    uint16_t cid = d[0] | (d[1] << 8);
    if (cid != COMPANY_ID) continue;
    if (d[2] != 'B' || d[3] != 'D' || d[4] != 'G') continue;

    int slot = findPeerSlot(adv->getAddress());
    if (slot < 0) continue;

    peers[slot].addr       = adv->getAddress();
    peers[slot].lastRSSI   = adv->getRSSI();
    peers[slot].lastSeen   = millis();
    peers[slot].valid      = true;
    // Versione d[5] se serve differenziare in futuro
    // myID peer = d[6]
    uint8_t peerPhaseByte  = d[7];
    peers[slot].phase      = (float)peerPhaseByte / 255.0f;
    peers[slot].phaseValid = true;

    float angle = peers[slot].phase * TWO_PI;
    sumSin += sinf(angle);
    sumCos += cosf(angle);
    phaseCount++;
  }

  if (phaseCount > 0) {
    float meanAngle = atan2f(sumSin / phaseCount, sumCos / phaseCount);
    if (meanAngle < 0) meanAngle += TWO_PI;
    neighborMeanPhase = meanAngle / TWO_PI;
    neighborPhaseAvailable = true;
  } else {
    neighborPhaseAvailable = false;
  }

  scan->clearResults();

  // Controllo new peer per bloom
  uint8_t currentPeerCount = countValidPeers();
  if (currentPeerCount > lastPeerCount) {
    triggerBloom();
  }
  lastPeerCount = currentPeerCount;
}

// ================== LED INIT =========================
void initLeds() {
  strip.begin();
  strip.clear();
  strip.setBrightness(255);
  strip.show();
  for (int i=0;i<NUM_LEDS;i++){
    ledStates[i].baseHue8 = rand8range(0,255);
    ledStates[i].warmHue8 = rand8range(0,45); // warm target
  }
}

// ================== HUE BLEND ========================
uint8_t blendHue(uint8_t h1, uint8_t h2, float t) {
  float hf = (1.0f - t) * h1 + t * h2;
  if (hf < 0) hf = 0;
  if (hf > 255) hf = 255;
  return (uint8_t)hf;
}

// ================== UPDATE FIREFLY ===================
void updateFirefly(float dt) {
  // Avanza fase base
  fireflyPhase += fireflyFreq * dt;
  fireflyPhase = wrap01(fireflyPhase);

  // Coupling se disponibile
  if (neighborPhaseAvailable && FIREFLY_COUPLING > 0.0f) {
    // Se vuoi coupling solo quando presenza > X:
    float effectiveCoupling = FIREFLY_COUPLING;
    if (FIREFLY_LOCK_WHEN_NEAR) {
      effectiveCoupling *= (0.4f + 0.6f * presenceFactor); // aumenta con presenza
    }
    float diff = phaseDiff(neighborMeanPhase, fireflyPhase);
    fireflyPhase = wrap01(fireflyPhase + diff * effectiveCoupling);
  }
}

// ================== UPDATE BLOOM =====================
float getBloomBoost() {
  if (!bloomActive) return 0.0f;
  uint32_t now = millis();
  if (now >= bloomEndMs) {
    bloomActive = false;
    return 0.0f;
  }
  uint32_t elapsed = (bloomEndMs - now); // contando al contrario
  uint32_t age = BLOOM_DURATION_MS - elapsed;
  float t = (float)age / (float)BLOOM_DURATION_MS; // 0..1
  // curva ease-out (1 - (1-t)^3)
  float curve = 1.0f - powf(1.0f - t, 3.0f);
  // invertiamo perché vogliamo forte all'inizio e svanisce
  float fade = 1.0f - curve;  // parte 1 → va a 0
  return fade * BLOOM_AMPLITUDE;
}

float getBloomWarmExtra() {
  if (!bloomActive) return 0.0f;
  uint32_t now = millis();
  if (now >= bloomEndMs) {
    bloomActive = false;
    return 0.0f;
  }
  uint32_t age = (millis() - (bloomEndMs - BLOOM_DURATION_MS));
  float t = (float)age / (float)BLOOM_DURATION_MS;
  float fade = 1.0f - t;
  if (fade < 0) fade = 0;
  return fade * BLOOM_WARM_EXTRA;
}

// ================== UPDATE LEDS ======================
void updateLeds(uint32_t nowMs, float dt) {
  // Presence (EMA)
  float raw = computePresenceRaw();
  presenceFactor += PRESENCE_ALPHA * (raw - presenceFactor);
  if (presenceFactor < 0) presenceFactor = 0;
  if (presenceFactor > 1) presenceFactor = 1;

  // Bloom trigger da spike di presence
  float presenceDelta = presenceFactor - previousPresence;
  if (presenceDelta > PRESENCE_SPIKE_THRESHOLD) {
    triggerBloom();
  }
  previousPresence = presenceFactor;

  // Firefly oscillator update
  updateFirefly(dt);

  // Brightness max base
  float maxBrightF = SOLO_BRIGHTNESS_MAX +
                     (float)(CROWD_BRIGHTNESS_MAX - SOLO_BRIGHTNESS_MAX) *
                     powf(presenceFactor, 0.9f);

  // Wave globale (pulsazione sincronizzata)
  float wave = sinf(fireflyPhase * TWO_PI); // -1..1
  wave = (wave + 1.0f) * 0.5f;              // 0..1
  // shaping
  wave = powf(wave, 1.35f);

  // Bloom overlay
  float bloomAdd = getBloomBoost();
  float bloomWarmAdd = getBloomWarmExtra();

  // Drift cromatico globale
  float driftHue8 = fmodf((nowMs / 1000.0f) * HUE_DRIFT_PER_SEC, 255.0f);

  for (int i=0;i<NUM_LEDS;i++){
    auto &ls = ledStates[i];

    // Valore di base
    float valF = 2.0f + wave * maxBrightF + bloomAdd;
    if (valF > 255.0f) valF = 255.0f;
    uint8_t V = (uint8_t)valF;

    uint8_t dynHue = (uint8_t)(ls.baseHue8 + (uint8_t)driftHue8);

    // Warm blend principale + bloom extra
    float warmBlend = powf(presenceFactor, WARM_BLEND_EXP);
    warmBlend += bloomWarmAdd;
    if (warmBlend > 1.0f) warmBlend = 1.0f;

    uint8_t finalHue = blendHue(dynHue, ls.warmHue8, warmBlend);

    float Sf = SATURATION_BASE +
               (float)(SATURATION_WARM - SATURATION_BASE) * warmBlend;
    if (Sf > 255) Sf = 255;
    uint8_t S = (uint8_t)Sf;

    uint16_t hue16 = (uint16_t)finalHue * 257;
    uint32_t c = strip.ColorHSV(hue16, S, V);
    strip.setPixelColor(i, c);

    // Lenta deriva per varietà (ogni ~512 ms)
    if ((nowMs & 0x1FF) == 0) {
      ls.baseHue8 += rand8range(0,2);
    }
  }

  strip.show();
}

// ================== SETUP ============================
void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(150);
  DBG("Boot start");
#endif
  randomSeed(esp_random());
  initLeds();
  setupBLE();
  lastScanStartMs = 0; // forza stanza prima scansione
#ifdef DEBUG_SERIAL
  DBG("Setup complete");
#endif
}

// ================== LOOP =============================
void loop() {
  uint32_t now = millis();

  // Scansione periodica
  if (now - lastScanStartMs >= SCAN_PERIOD_MS) {
    lastScanStartMs = now;
    doScanCycle();
  }

  // Frame update
  if (now - lastFrameMs >= FRAME_INTERVAL_MS) {
    float dt = (lastFrameMs == 0)
               ? (FRAME_INTERVAL_MS / 1000.0f)
               : (now - lastFrameMs) / 1000.0f;
    lastFrameMs = now;
    updateLeds(now, dt);
    updateAdvertisingPhase(); // aggiorna phase nel manufacturer data
  }

  delay(1);
}
