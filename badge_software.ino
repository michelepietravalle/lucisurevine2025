/*
 badge_warm_pulse_v2.ino
 - BLE legacy (advertise + blocking scan)
 - Smooth symmetric pulsing: slow rise and fall
 - Minimum pulse level 2%
 - Peak maps: 1 peer -> 10%, 10+ peers -> 100%, smoothed over time
 - Without peers: cold / psychedelic colors
 - Lightweight [STATUS] logs
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   16
#define NUM_LEDS  7

// Logging
const uint32_t STATUS_LOG_INTERVAL_MS = 1500;

// Visual / behavior params
const uint8_t SOLO_BRIGHTNESS_MAX   = 25;
const uint8_t CROWD_BRIGHTNESS_MAX  = 190;
const uint8_t SATURATION_BASE       = 240;
const uint8_t SATURATION_WARM       = 255;

const float   WARM_BLEND_EXP        = 1.0f;   // linear-ish blend
const float   WARM_SHIFT_STRENGTH   = 1.6f;   // stronger shift toward warm
const float   WARM_SHIFT_EXP        = 0.95f;

const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -100;
const int8_t   RSSI_MAX_DBM         = -45;
const float    PRESENCE_ALPHA       = 0.30f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

const uint8_t PROTO_VERSION         = 4;
const uint8_t WARM_HUE              = 10;   // central warm hue
const uint8_t COLD_MIN_HUE          = 140;
const uint8_t COLD_MAX_HUE          = 200;
const float   GROUP_HUE_COUPLING    = 0.055f;

const uint32_t FRAME_INTERVAL_MS      = 30;
const uint32_t ADV_UPDATE_INTERVAL_MS = 300;
const uint32_t SCAN_PERIOD_MS         = 1500;
const uint32_t SCAN_DURATION_S        = 1; // seconds (int)
const uint16_t COMPANY_ID = 0xFFFF;

// Pulsing specifics
const float SLOW_PULSE_FREQ_HZ = 0.22f; // ~4.5s period
const float PEAK_SMOOTH_ALPHA = 0.02f;  // smoothing alpha per frame for peak
const float MIN_FRACTION = 0.02f;       // 2% minimum

struct LedState {
  uint8_t offsetHue8;   // small random offset
  uint8_t warmHue8;     // per-led warm base
  float   pulsePhase;
  float   pulseSpeed;
};

struct BadgePeer {
  BLEAddress addr;
  int8_t   lastRSSI    = 0;
  uint32_t lastSeen    = 0;
  bool     valid       = false;
  bool     phaseValid  = false;
  float    phase       = 0.0f;
  bool     hueValid    = false;
  uint8_t  groupHue    = 0;
};

static const uint8_t MAX_PEERS = 16;

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
LedState  ledStates[NUM_LEDS];
BadgePeer peers[MAX_PEERS];

float presenceFactor        = 0.0f;
float previousPresence      = 0.0f;

uint32_t lastFrameMs        = 0;
uint32_t lastScanStartMs    = 0;
uint32_t lastAdvUpdateMs    = 0;
uint32_t lastStatusLogMs    = 0;

uint8_t  myID               = 0;

float fireflyPhase          = 0.0f;
float fireflyFreq           = 0.55f;

bool  neighborPhaseAvailable= false;
float neighborMeanPhase     = 0.0f;

uint32_t bloomEndMs         = 0;
bool     bloomActive        = false;

uint8_t  lastPeerCount      = 0;

uint8_t  groupHueLocal      = 170;
uint8_t  groupHueTarget     = 170;
uint32_t lastSoloHueDriftMs = 0;

BLEAdvertising* gAdv        = nullptr;

// smoothing for peak
float smoothedPeak = MIN_FRACTION;
// hue drift for psychedelic mode (no peers)
float hueDriftPhase = 0.0f;

// ---------- utils ----------
uint8_t rand8range(uint8_t a, uint8_t b){ return a + (uint8_t)(esp_random() % (b - a + 1)); }
float randFloat(float a,float b){ return a + ((float)esp_random() / (float)UINT32_MAX)*(b-a); }

int findPeerSlot(const BLEAddress& a){
  int freeSlot=-1;
  for(int i=0;i<MAX_PEERS;i++){
    if (peers[i].valid && peers[i].addr.equals(a)) return i;
    if (!peers[i].valid && freeSlot<0) freeSlot=i;
  }
  return freeSlot;
}

void purgePeers(){
  uint32_t now=millis();
  for(int i=0;i<MAX_PEERS;i++){
    if(peers[i].valid && (now - peers[i].lastSeen) > BEACON_TIMEOUT_MS){
      peers[i].valid=false;
      peers[i].phaseValid=false;
      peers[i].hueValid=false;
    }
  }
}

float rssiToWeight(int8_t r){
  if (r <= RSSI_MIN_DBM) return 0.0f;
  if (r >= RSSI_MAX_DBM) return 1.0f;
  return (float)(r - RSSI_MIN_DBM)/(float)(RSSI_MAX_DBM - RSSI_MIN_DBM);
}

float computePresenceRaw(){
  purgePeers();
  float sum=0;
  for(int i=0;i<MAX_PEERS;i++){
    if(!peers[i].valid) continue;
    sum += rssiToWeight(peers[i].lastRSSI);
  }
  float n = sum / (float)PEERS_FOR_FULL_SCALE;
  if (n>1.0f) n=1.0f;
  return n;
}
uint8_t countValidPeers(){
  uint8_t c=0;
  for(int i=0;i<MAX_PEERS;i++) if(peers[i].valid) c++;
  return c;
}

static inline float wrap01(float x){
  if (x>=1.0f) return x-(int)x;
  if (x<0.0f)  return x-floorf(x);
  return x;
}
static inline float phaseDiff(float target,float local){
  float d=target-local;
  while(d>0.5f)d-=1.0f;
  while(d<-0.5f)d+=1.0f;
  return d;
}
static inline int8_t hueDiffSigned(uint8_t target,uint8_t current){
  int16_t d=(int16_t)target - (int16_t)current;
  if(d>128)d-=256;
  if(d<-128)d+=256;
  return (int8_t)d;
}
uint8_t hueAdd(uint8_t h,int16_t delta){
  int16_t v=(int16_t)h + delta;
  while(v<0)v+=256;
  while(v>255)v-=256;
  return (uint8_t)v;
}
uint8_t clampCold(uint8_t h){
  if(h<COLD_MIN_HUE) return COLD_MIN_HUE;
  if(h>COLD_MAX_HUE) return COLD_MAX_HUE;
  return h;
}

uint8_t applyWarmBias(uint8_t baseHue, float driver){
  if (driver <= 0.001f) return baseHue;
  float t = powf(driver, WARM_SHIFT_EXP)*WARM_SHIFT_STRENGTH;
  if (t>1.0f) t=1.0f;
  int8_t d = hueDiffSigned(WARM_HUE, baseHue);
  float move = d * t;
  int16_t newHue = (int16_t)baseHue + (int16_t)lroundf(move);
  while(newHue<0)newHue+=256;
  while(newHue>255)newHue-=256;
  return (uint8_t)newHue;
}

// ---------- LED init ----------
void initLeds(){
  strip.begin();
  strip.clear();
  strip.setBrightness(255);
  strip.show();
  for(int i=0;i<NUM_LEDS;i++){
    // distinct offsets
    ledStates[i].offsetHue8 = (uint8_t)(i * 18 + (rand8range(0,8)));
    // per-led warm base around WARM_HUE, slightly spread
    ledStates[i].warmHue8   = (uint8_t)((WARM_HUE + rand8range(0,60)) & 0xFF);
    float freq = randFloat(0.18f, 0.35f);
    ledStates[i].pulseSpeed = freq * TWO_PI;
    ledStates[i].pulsePhase = randFloat(0.0f, TWO_PI);
  }
}

// ---------- Advertising ----------
void setupAdvertising(){
  gAdv = BLEDevice::getAdvertising();
  uint8_t payload[9];
  payload[0] = (uint8_t)(COMPANY_ID & 0xFF);
  payload[1] = (uint8_t)(COMPANY_ID >> 8);
  payload[2] = 'B'; payload[3] = 'D'; payload[4] = 'G';
  payload[5] = PROTO_VERSION;
  payload[6] = myID;
  payload[7] = (uint8_t)(wrap01(fireflyPhase)*255.0f);
  payload[8] = groupHueLocal;
  String manuf; manuf.reserve(9);
  for (size_t i=0;i<9;i++) manuf += (char)payload[i];
  BLEAdvertisementData advData; advData.setManufacturerData(manuf);
  gAdv->setAdvertisementData(advData);
  gAdv->start();
}

void updateAdvertisingData(){
  if(!gAdv) return;
  uint32_t now = millis();
  if(now - lastAdvUpdateMs < ADV_UPDATE_INTERVAL_MS) return;
  lastAdvUpdateMs = now;
  uint8_t payload[9];
  payload[0] = (uint8_t)(COMPANY_ID & 0xFF);
  payload[1] = (uint8_t)(COMPANY_ID >> 8);
  payload[2] = 'B'; payload[3] = 'D'; payload[4] = 'G';
  payload[5] = PROTO_VERSION;
  payload[6] = myID;
  payload[7] = (uint8_t)(wrap01(fireflyPhase)*255.0f);
  payload[8] = groupHueLocal;
  String manuf; manuf.reserve(9);
  for (size_t i=0;i<9;i++) manuf += (char)payload[i];
  BLEAdvertisementData advData; advData.setManufacturerData(manuf);
  gAdv->setAdvertisementData(advData);
}

// ---------- Scan ----------
void doScanCycle(){
  BLEScan* scan = BLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(160);
  BLEScanResults* resultsPtr = scan->start(SCAN_DURATION_S, false);
  if(!resultsPtr) return;
  int foundTotal = resultsPtr->getCount();

  float sumSinPhase=0,sumCosPhase=0;
  int phaseCount=0;
  float sumSinHue=0,sumCosHue=0;
  int hueCount=0;

  for(int i=0;i<foundTotal;i++){
    BLEAdvertisedDevice dev = resultsPtr->getDevice(i);
    if(!dev.haveManufacturerData()) continue;
    String m = dev.getManufacturerData();
    if (m.length() < 8) continue;
    uint8_t d0 = (uint8_t)m.charAt(0);
    uint8_t d1 = (uint8_t)m.charAt(1);
    uint16_t cid = d0 | (d1<<8);
    if (cid != COMPANY_ID) continue;
    if ((char)m.charAt(2)!='B' || (char)m.charAt(3)!='D' || (char)m.charAt(4)!='G') continue;

    int slot = findPeerSlot(dev.getAddress());
    if (slot < 0) continue;

    peers[slot].addr     = dev.getAddress();
    peers[slot].lastRSSI = dev.getRSSI();
    peers[slot].lastSeen = millis();
    peers[slot].valid    = true;

    uint8_t phaseByte = (uint8_t)m.charAt(7);
    peers[slot].phase = (float)phaseByte / 255.0f;
    peers[slot].phaseValid = true;

    if (m.length() >= 9){
      uint8_t peerHue = (uint8_t)m.charAt(8);
      peers[slot].groupHue = peerHue;
      peers[slot].hueValid = true;
      float angleH = (peerHue/255.0f)*TWO_PI;
      sumSinHue += sinf(angleH);
      sumCosHue += cosf(angleH);
      hueCount++;
    } else peers[slot].hueValid = false;

    float angleP = peers[slot].phase * TWO_PI;
    sumSinPhase += sinf(angleP);
    sumCosPhase += cosf(angleP);
    phaseCount++;
  }

  if (phaseCount>0){
    float meanAngleP = atan2f(sumSinPhase/phaseCount, sumCosPhase/phaseCount);
    if (meanAngleP < 0) meanAngleP += TWO_PI;
    neighborMeanPhase = meanAngleP / TWO_PI;
    neighborPhaseAvailable = true;
  } else neighborPhaseAvailable = false;

  if (hueCount>0){
    float meanAngleH = atan2f(sumSinHue/hueCount, sumCosHue/hueCount);
    if (meanAngleH < 0) meanAngleH += TWO_PI;
    groupHueTarget = (uint8_t)((meanAngleH / TWO_PI) * 255.0f);
  }

  uint8_t currentPeerCount = countValidPeers();
  if (currentPeerCount > lastPeerCount){
    if (presenceFactor < 0.02f) presenceFactor = 0.15f;
    bloomEndMs = millis() + 600;
    bloomActive = true;
  }
  lastPeerCount = currentPeerCount;

  scan->clearResults();
}

// ---------- Group hue update ----------
void updateGroupHue(uint8_t peerCount){
  if (peerCount == 0){
    // solo drift
    if (millis() - lastSoloHueDriftMs > 1800){
      lastSoloHueDriftMs = millis();
      int8_t step = (int8_t)rand8range(0,2)-1;
      groupHueLocal = clampCold(hueAdd(groupHueLocal, step));
    }
    groupHueTarget = groupHueLocal;
  } else {
    int8_t d = hueDiffSigned(groupHueTarget, groupHueLocal);
    float step = d * GROUP_HUE_COUPLING;
    if (step > 6) step = 6;
    if (step < -6) step = -6;
    groupHueLocal = hueAdd(groupHueLocal, (int16_t)lroundf(step));
  }
}

// ---------- LEDs update ----------
void updateLeds(uint32_t nowMs, float dt){
  float raw = computePresenceRaw();
  presenceFactor += PRESENCE_ALPHA * (raw - presenceFactor);
  if(presenceFactor<0) presenceFactor=0;
  if(presenceFactor>1) presenceFactor=1;

  uint8_t peerCount = countValidPeers();

  float presenceDelta = presenceFactor - previousPresence;
  if (presenceDelta > 0.12f && peerCount>0){
    bloomEndMs = millis() + 600;
    bloomActive = true;
  }
  previousPresence = presenceFactor;

  // firefly phase small coupling
  fireflyPhase += fireflyFreq * dt;
  fireflyPhase = wrap01(fireflyPhase);
  if (peerCount>0 && neighborPhaseAvailable){
    float eff = 0.09f * (0.4f + 0.6f*presenceFactor);
    float diff = phaseDiff(neighborMeanPhase, fireflyPhase);
    fireflyPhase = wrap01(fireflyPhase + diff*eff);
  }

  updateGroupHue(peerCount);

  // Determine target peak fraction:
  float targetPeak;
  if (peerCount == 0) targetPeak = MIN_FRACTION; // 2% when no peers
  else {
    targetPeak = (float)peerCount / 10.0f; // 1->0.1, 10->1.0
    if (targetPeak < 0.1f) targetPeak = 0.1f;
    if (targetPeak > 1.0f) targetPeak = 1.0f;
  }

  // Smooth the peak to avoid jumps
  smoothedPeak += PEAK_SMOOTH_ALPHA * (targetPeak - smoothedPeak);
  if (smoothedPeak < MIN_FRACTION) smoothedPeak = MIN_FRACTION;

  // slow pulse phase update (global)
  static float slowPulsePhase = 0.0f;
  slowPulsePhase += SLOW_PULSE_FREQ_HZ * dt;
  slowPulsePhase = wrap01(slowPulsePhase);
  float slowWave = (sinf(slowPulsePhase * TWO_PI) + 1.0f) * 0.5f; // 0..1 symmetric

  // hue drift for psychedelic when no peers
  hueDriftPhase += dt * 0.06f; // slow hue drift
  hueDriftPhase = wrap01(hueDriftPhase);

  // warm driver used for hue biasing
  float peerDensity = (peerCount==0)?0.0f:(float)peerCount/(float)PEERS_FOR_FULL_SCALE;
  if(peerDensity>1.0f) peerDensity = 1.0f;
  float warmDriver = (presenceFactor > peerDensity) ? presenceFactor : peerDensity;

  uint8_t biasedGroupHue = applyWarmBias(groupHueLocal, warmDriver);

  for(int i=0;i<NUM_LEDS;i++){
    auto &ls = ledStates[i];

    // compute dynamic hue
    uint8_t dynHue = hueAdd(biasedGroupHue, ls.offsetHue8);

    uint8_t finalHue;
    uint8_t S; // saturation
    if (peerCount == 0){
      // psychedelic cold: spread hues across cold range, add small time-varying offset
      // base is cold clamp of groupHueLocal, then per-led offset and drift
      uint8_t base = clampCold(groupHueLocal);
      int16_t driftOffset = (int16_t)lroundf( (sinf(hueDriftPhase * TWO_PI + i) ) * 20.0f );
      finalHue = hueAdd(base, (int16_t)ls.offsetHue8 + driftOffset);
      // more saturated for psychedelic effect
      float Sf = SATURATION_BASE + 8.0f * ((float)i / (float)NUM_LEDS);
      if (Sf > 255.0f) Sf = 255.0f;
      S = (uint8_t)Sf;
    } else {
      // with peers: keep warm blend toward per-led warm base
      float warmBlend = powf(warmDriver, WARM_BLEND_EXP);
      if (bloomActive) { warmBlend += 0.12f; if (warmBlend>1.0f) warmBlend=1.0f; }
      finalHue = blendHueLinear(ls.warmHue8, dynHue, warmBlend);
      float Sf = SATURATION_BASE + (float)(SATURATION_WARM - SATURATION_BASE) * warmBlend;
      if (Sf > 255.0f) Sf = 255.0f;
      S = (uint8_t)Sf;
    }

    // brightness: symmetric slowWave with smoothedPeak, minimum MIN_FRACTION
    float peak = smoothedPeak;
    if (peak < MIN_FRACTION) peak = MIN_FRACTION;
    float valFrac = MIN_FRACTION + slowWave * (peak - MIN_FRACTION); // MIN..peak
    if (valFrac < MIN_FRACTION) valFrac = MIN_FRACTION;
    if (valFrac > 1.0f) valFrac = 1.0f;
    uint8_t V = (uint8_t)lroundf(valFrac * 255.0f);

    uint16_t hue16 = (uint16_t)finalHue * 257;
    uint32_t c = strip.ColorHSV(hue16, S, V);
    strip.setPixelColor(i, c);
  }
  strip.show();

  // decay bloom
  if (bloomActive && millis() > bloomEndMs) bloomActive = false;

  // status log
  if (nowMs - lastStatusLogMs > STATUS_LOG_INTERVAL_MS){
    lastStatusLogMs = nowMs;
    Serial.print("[STATUS] peers=");
    Serial.print(peerCount);
    Serial.print(" presence=");
    Serial.print(presenceFactor,3);
    Serial.print(" gHueLocal=");
    Serial.print(groupHueLocal);
    Serial.print(" gHueTarget=");
    Serial.print(groupHueTarget);
    Serial.print(" biased=");
    Serial.print(biasedGroupHue);
    Serial.print(" peakPct=");
    Serial.println((int)lroundf(smoothedPeak * 100.0f));
  }
}

// ---------- helper blend ----------
uint8_t blendHueLinear(uint8_t h1,uint8_t h2,float t){
  float hf=(1.0f - t)*(float)h1 + t*(float)h2;
  if(hf<0) hf=0;
  if(hf>255) hf=255;
  return (uint8_t)hf;
}

// ---------- setup / loop ----------
void setupBLE_scanConfig(){
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(160);
}

uint8_t deriveMyIDFromBLEAddr(){
  String addr = BLEDevice::getAddress().toString();
  int pos = addr.lastIndexOf(':');
  String last = (pos >= 0) ? addr.substring(pos+1) : addr;
  last.trim();
  char buf[3] = {0,0,0};
  last.toCharArray(buf, 3);
  uint8_t v = (uint8_t)strtoul(buf, NULL, 16);
  if (v == 0) v = (uint8_t)(esp_random() & 0xFF);
  return v;
}

void setup(){
  Serial.begin(115200);
  delay(120);
  BLEDevice::init("Badge");
  myID = deriveMyIDFromBLEAddr();

  Serial.print("PROTO_VERSION=");
  Serial.println(PROTO_VERSION);
  Serial.print("myID=");
  Serial.println(myID);

  initLeds();
  setupAdvertising();
  setupBLE_scanConfig();
  lastScanStartMs = 0;
}

void loop(){
  uint32_t now = millis();

  if (now - lastScanStartMs >= SCAN_PERIOD_MS){
    lastScanStartMs = now;
    doScanCycle();
  }

  if (now - lastFrameMs >= FRAME_INTERVAL_MS){
    float dt = (lastFrameMs == 0) ? (FRAME_INTERVAL_MS/1000.0f) : (now - lastFrameMs)/1000.0f;
    lastFrameMs = now;
    updateLeds(now, dt);
    updateAdvertisingData();
  }

  delay(1);
}
