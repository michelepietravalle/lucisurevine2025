/*
 badge_warm_pulse_sync_ordered_v23.ino

 Obiettivi:
 - Più movimento in presenza di peer:
    * Periodi minimi più rapidi (pulsazione & cambio colori)
    * Shimmer globale & per-pixel (solo peer mode)
    * Micro-sparkle controllato (solo peer mode)
    * Phase drift lento per LED (solo peer mode)
    * Leggero hue wobble (solo peer mode)
    * Cross-fade palette più veloce in peer mode
 - Mantiene fading 0 -> picco -> 0, nessuna baseline luminosa artificiale.
 - Flash bianco post-blackout al 20%.

 NOTE:
 - LED_PHASE_OFFSET_ENABLED forzato a true (richiesta utente).
 - In solo mode tutto rimane morbido e lento come nelle versioni precedenti recenti.

 Tuning rapido:
 - Rendere ancora più veloce: abbassa PULSE_PERIOD_MIN_SEC_PEER / COLOR_CYCLE_PERIOD_MIN_SEC_PEER
 - Più vibrazione colore: aumenta HUE_WOBBLE_MAX
 - Eliminare micro-sparkle: ENABLE_PEER_MICRO_SPARKLE = false
 - Eliminare drift: ENABLE_PEER_PHASE_DRIFT = false
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_NeoPixel.h>

#ifndef COMPANY_ID
#define COMPANY_ID 0xFFFF
#endif

#define PEER_RSSI_DEBUG 0
// #define COLOR_DEBUG 1

#define LED_PIN   16
#define NUM_LEDS  7

// Logging
const uint32_t STATUS_LOG_INTERVAL_MS = 2000;

// Presence / RSSI
const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -100;
const int8_t   RSSI_MAX_DBM         = -45;
const int8_t   PEER_RSSI_CUTOFF_DBM = -70;
const float    PRESENCE_ALPHA       = 0.30f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

// Protocol
const uint8_t  PROTO_VERSION = 6;

// Cold hues (solo mode)
const uint8_t  COLD_MIN_HUE = 140;
const uint8_t  COLD_MAX_HUE = 200;

// Hue coupling
const float GROUP_HUE_COUPLING = 0.055f;

// Vivacità saturazione
const uint8_t MAX_PEER_ACCEL_PEERS = 6;

// Periodi dinamici (SOLO MODE min)
const float PULSE_PERIOD_MAX_SEC_SOLO = 3.0f;
const float PULSE_PERIOD_MIN_SEC_SOLO = 1.20f;
const float COLOR_CYCLE_PERIOD_MAX_SEC_SOLO = 3.0f;
const float COLOR_CYCLE_PERIOD_MIN_SEC_SOLO = 1.50f;

// Periodi dinamici (PEER MODE min più veloci)
const float PULSE_PERIOD_MIN_SEC_PEER       = 0.75f;
const float COLOR_CYCLE_PERIOD_MIN_SEC_PEER = 0.80f;

// Smoothing
const float PULSE_PERIOD_SMOOTH_ALPHA       = 0.25f;
const float COLOR_CYCLE_PERIOD_SMOOTH_ALPHA = 0.25f;

// Frame / adv
const uint32_t FRAME_INTERVAL_MS      = 30;
const uint32_t ADV_UPDATE_INTERVAL_MS = 100;

// Peak smoothing
const float PEAK_SMOOTH_ALPHA           = 0.06f;
const float PEAK_SMOOTH_ALPHA_FALL_MULT = 2.6f;

// Peak targets
const float ZERO_PEER_PEAK = 0.40f;
const float PEAK_SCALE_MAX = 1.25f;

// Shimmer & movement toggles
const bool  ENABLE_PEER_GLOBAL_SHIMMER = true;
const float PEER_GLOBAL_SHIMMER_DEPTH  = 0.09f;

const bool  ENABLE_PEER_PIXEL_SHIMMER  = true;
const float PEER_PIXEL_SHIMMER_DEPTH   = 0.18f;
const float SHIMMER_PHASE_SPREAD       = 1.7f;
const float SHIMMER_FREQ_MULT          = 2.15f;

// Micro-sparkle
const bool  ENABLE_PEER_MICRO_SPARKLE = true;
const float PEER_MICRO_SPARKLE_MAX    = 0.10f;
const float SPARKLE_WAVE_THRESHOLD    = 0.15f;

// Phase drift per LED
const bool     ENABLE_PEER_PHASE_DRIFT = true;
const uint32_t PHASE_DRIFT_INTERVAL_MS = 1800;
const float    PHASE_DRIFT_MAX_DELTA   = 0.035f; // massimo offset addizionale
// Hue wobble
const bool  ENABLE_HUE_WOBBLE = true;
const uint8_t HUE_WOBBLE_MAX  = 6; // ±6

// Bloom
const uint32_t BLOOM_DURATION_MS = 600;

// Cross-fade palette (solo / peer)
const uint16_t CROSS_FADE_MS_SOLO = 150;
const uint16_t CROSS_FADE_MS_PEER = 90;

// Blackout
const uint8_t  BLACKOUT_PROB        = 85;  // ~33%
const uint16_t BLACKOUT_DURATION_MS = 120;

// Blackout nuovo peer
#define BLACKOUT_NEW_PEER_ENABLED 1
const uint16_t BLACKOUT_NEW_PEER_DURATION_MS = 150;
const bool     NEW_PEER_BLACKOUT_EXTEND      = true;

// Flash bianco post-blackout
#define POST_FLASH_ENABLED 1
const uint16_t POST_FLASH_DURATION_MS     = 130;
const float    POST_FLASH_FIXED_INTENSITY = 0.20f; // 20%

// Phase offset per LED: utente vuole attivo
const bool LED_PHASE_OFFSET_ENABLED = true;

// Cold color jitter solo mode
const uint32_t ZERO_PEER_BASE_INTERVAL_MS = 3000;
const uint32_t ZERO_PEER_COLOR_JITTER_MS  = 900;

// Sequenza
static const int SEQ[NUM_LEDS] = {0,2,4,6,1,3,5};

// Strutture
struct LedState {
  float baseOffset;
  float driftOffset;    // aggiornato nel tempo
  uint32_t nextDriftUpdate;
  uint32_t seed;        // per wobble e drift
};

struct BadgePeer {
  BLEAddress addr;
  int8_t   lastRSSI=0;
  uint32_t lastSeen=0;
  bool     valid=false;
  bool     phaseValid=false;
  float    phase=0.0f;
  bool     cycleValid=false;
  uint16_t colorCycle=0;
  uint8_t  peerID=0;
};

static const uint8_t MAX_PEERS=16;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Stato
LedState  ledStates[NUM_LEDS];
BadgePeer peers[MAX_PEERS];

uint8_t  coldHueCurrent[NUM_LEDS];
uint32_t coldHueNextChangeMs[NUM_LEDS];

float presenceFactor=0.0f;
float previousPresence=0.0f;

uint32_t lastFrameMs=0;
uint32_t lastAdvUpdateMs=0;
uint32_t lastStatusLogMs=0;

uint8_t  myID=0;

float fireflyPhase=0.0f;
float pulsePeriodCurrentSec = PULSE_PERIOD_MAX_SEC_SOLO;
float fireflyFreq=1.0f / PULSE_PERIOD_MAX_SEC_SOLO;

uint32_t warmColorCycle=0;
float    colorCyclePeriodCurrentSec = COLOR_CYCLE_PERIOD_MAX_SEC_SOLO;
uint32_t lastColorCycleTickMs=0;

bool  neighborPhaseAvailable=false;
float neighborMeanPhase=0.0f;

uint32_t bloomEndMs=0;
bool     bloomActive=false;

uint8_t  groupHueLocal=170;
uint8_t  groupHueTarget=170;
uint32_t lastSoloHueDriftMs=0;

BLEAdvertising* gAdv=nullptr;

float smoothedPeak=0.0f;

uint8_t currentLeaderID=0;
bool    isLeader=true;

// Palette
static const uint8_t basePalette[NUM_LEDS]={210,28,85,52,0,245,230};
uint8_t prevCycleHue[NUM_LEDS];
uint8_t targetCycleHue[NUM_LEDS];
uint8_t currentHueDisplay[NUM_LEDS];
uint32_t lastCycleChangeMs=0;
bool fadeActive=false;

// PRNG
static uint32_t prngState=0;
inline void prngSeed(uint32_t s){
  prngState = s ^ 0x9E3779B9u;
  for(int i=0;i<3;i++){
    prngState ^= prngState << 13;
    prngState ^= prngState >> 17;
    prngState ^= prngState << 5;
  }
}
inline uint32_t prngNext(){
  uint32_t x=prngState; x^=x<<13; x^=x>>17; x^=x<<5; prngState=x; return x;
}

// Blackout / flash
bool     blackoutScheduled=false;
float    blackoutPhase=0.0f;
bool     blackoutActive=false;
uint32_t blackoutEndMs=0;

bool     postFlashActive=false;
uint32_t postFlashStartMs=0;
uint32_t postFlashEndMs=0;

// --- Utility ---
uint8_t rand8range(uint8_t a,uint8_t b){ return a + (uint8_t)(esp_random() % (b - a + 1)); }

int findPeerSlot(const BLEAddress& a){
  int freeSlot=-1;
  for(int i=0;i<MAX_PEERS;i++){
    if(peers[i].valid && peers[i].addr.equals(a)) return i;
    if(!peers[i].valid && freeSlot<0) freeSlot=i;
  }
  return freeSlot;
}

void purgePeers(){
  uint32_t now=millis();
  for(int i=0;i<MAX_PEERS;i++){
    if(peers[i].valid && (now - peers[i].lastSeen)>BEACON_TIMEOUT_MS){
      peers[i].valid=false; peers[i].phaseValid=false; peers[i].cycleValid=false;
    }
  }
}

float rssiToWeight(int8_t r){
  if(r<=RSSI_MIN_DBM) return 0.0f;
  if(r>=RSSI_MAX_DBM) return 1.0f;
  return (float)(r - RSSI_MIN_DBM)/(float)(RSSI_MAX_DBM - RSSI_MIN_DBM);
}

float computePresenceRaw(){
  purgePeers();
  float sum=0;
  for(int i=0;i<MAX_PEERS;i++) if(peers[i].valid) sum += rssiToWeight(peers[i].lastRSSI);
  float n=sum/(float)PEERS_FOR_FULL_SCALE;
  return (n>1.0f)?1.0f:n;
}

uint8_t countValidPeers(){
  uint8_t c=0; for(int i=0;i<MAX_PEERS;i++) if(peers[i].valid)c++; return c;
}

static inline float wrap01(float x){
  if(x>=1.0f) return x-(int)x;
  if(x<0.0f)  return x - floorf(x);
  return x;
}

static inline float phaseDiff(float target,float local){
  float d=target-local;
  while(d>0.5f)d-=1.0f;
  while(d<-0.5f)d+=1.0f;
  return d;
}

static inline int8_t hueDiffSigned(uint8_t t,uint8_t c){
  int16_t d=(int16_t)t-(int16_t)c;
  if(d>128)d-=256;
  if(d<-128)d+=256;
  return (int8_t)d;
}

uint8_t hueAdd(uint8_t h,int16_t delta){
  int16_t v=(int16_t)h+delta;
  v = (v % 256 + 256) % 256;
  return (uint8_t)v;
}

uint8_t clampCold(uint8_t h){
  if(h<COLD_MIN_HUE)return COLD_MIN_HUE;
  if(h>COLD_MAX_HUE)return COLD_MAX_HUE;
  return h;
}

uint8_t hueLerpCircular(uint8_t a,uint8_t b,float t){
  if(t<=0) return a;
  if(t>=1) return b;
  int16_t da=(int16_t)b-(int16_t)a;
  if(da>127) da-=256;
  if(da<-128) da+=256;
  int16_t v = (int16_t)a + (int16_t)lroundf(da*t);
  if(v<0) v+=256;
  if(v>255) v-=256;
  return (uint8_t)v;
}

// --- Palette ---
void computeTargetPalette(){
  uint8_t idx[NUM_LEDS];
  for(int i=0;i<NUM_LEDS;i++) idx[i]=i;
  prngSeed(warmColorCycle * 2654435761u);
  for(int i=NUM_LEDS-1;i>0;i--){
    uint32_t r=prngNext()%(i+1);
    uint8_t tmp=idx[i]; idx[i]=idx[r]; idx[r]=tmp;
  }
  for(int i=0;i<NUM_LEDS;i++) targetCycleHue[i]=basePalette[idx[i]];
}

uint16_t currentCrossFadeMs(uint8_t peerCount){
  if(peerCount==0) return CROSS_FADE_MS_SOLO;
  float factor = (peerCount >= MAX_PEER_ACCEL_PEERS)?1.0f:(float)peerCount/MAX_PEER_ACCEL_PEERS;
  // Interp tra solo e peer
  return (uint16_t)lroundf(CROSS_FADE_MS_SOLO + (CROSS_FADE_MS_PEER - CROSS_FADE_MS_SOLO)*factor);
}

// --- Blackout scheduling ---
void scheduleBlackoutForCycle(){
  uint32_t h = warmColorCycle * 2654435761u;
  h ^= (h >> 13);
  uint8_t prob = (uint8_t)(h & 0xFF);
  if(prob < BLACKOUT_PROB){
    blackoutScheduled=true;
    blackoutPhase=((h>>8)&0xFF)/255.0f;
  } else {
    blackoutScheduled=false;
  }
  blackoutActive=false;
}

// --- Blackout nuovo peer ---
void triggerNewPeerBlackout(uint32_t now){
#if BLACKOUT_NEW_PEER_ENABLED
  if(!blackoutActive){
    blackoutActive=true;
    blackoutEndMs=now + BLACKOUT_NEW_PEER_DURATION_MS;
  } else if(NEW_PEER_BLACKOUT_EXTEND){
    uint32_t proposed=now + BLACKOUT_NEW_PEER_DURATION_MS;
    if(proposed>blackoutEndMs) blackoutEndMs=proposed;
  }
#endif
}

// --- Post Flash ---
void startPostFlash(uint32_t now){
#if POST_FLASH_ENABLED
  postFlashActive=true;
  postFlashStartMs=now;
  postFlashEndMs=now + POST_FLASH_DURATION_MS;
#endif
}
void updatePostFlash(uint32_t now){
#if POST_FLASH_ENABLED
  if(postFlashActive && now>=postFlashEndMs) postFlashActive=false;
#endif
}

// --- Cycle transitions ---
void startNewCycleFade(uint32_t nowMs){
  for(int i=0;i<NUM_LEDS;i++) prevCycleHue[i]=currentHueDisplay[i];
  computeTargetPalette();
  scheduleBlackoutForCycle();
  lastCycleChangeMs=nowMs;
  fadeActive=true;
}

void advanceColorCycle(uint32_t nowMs){
  warmColorCycle++;
  startNewCycleFade(nowMs);
  lastAdvUpdateMs=0;
}

void adoptColorCycle(uint32_t newCycle,uint32_t nowMs){
  if(newCycle==warmColorCycle) return;
  warmColorCycle=newCycle;
  for(int i=0;i<NUM_LEDS;i++) prevCycleHue[i]=currentHueDisplay[i];
  computeTargetPalette();
  scheduleBlackoutForCycle();
  lastCycleChangeMs=nowMs;
  fadeActive=true;
  lastAdvUpdateMs=0;
}

void updateFade(uint32_t nowMs, uint8_t peerCount){
  if(!fadeActive){
    for(int i=0;i<NUM_LEDS;i++) currentHueDisplay[i]=targetCycleHue[i];
    return;
  }
  uint16_t cf = currentCrossFadeMs(peerCount);
  uint32_t elapsed=nowMs - lastCycleChangeMs;
  float t = (cf==0)?1.0f:(float)elapsed/(float)cf;
  if(t>=1.0f){
    fadeActive=false;
    for(int i=0;i<NUM_LEDS;i++) currentHueDisplay[i]=targetCycleHue[i];
    return;
  }
  for(int i=0;i<NUM_LEDS;i++){
    currentHueDisplay[i]=hueLerpCircular(prevCycleHue[i], targetCycleHue[i], t);
  }
}

// --- Periodi dinamici ---
void updateDynamicPeriods(uint8_t peerCount){
  float factor=(peerCount >= MAX_PEER_ACCEL_PEERS)?1.0f:(float)peerCount/MAX_PEER_ACCEL_PEERS;

  // Pulsazione
  float pulseMin = (peerCount==0) ? PULSE_PERIOD_MIN_SEC_SOLO : PULSE_PERIOD_MIN_SEC_PEER;
  float pulseTarget = PULSE_PERIOD_MAX_SEC_SOLO + (pulseMin - PULSE_PERIOD_MAX_SEC_SOLO)*factor;
  pulsePeriodCurrentSec += PULSE_PERIOD_SMOOTH_ALPHA*(pulseTarget - pulsePeriodCurrentSec);
  if(pulsePeriodCurrentSec < pulseMin) pulsePeriodCurrentSec=pulseMin;
  if(pulsePeriodCurrentSec > PULSE_PERIOD_MAX_SEC_SOLO) pulsePeriodCurrentSec=PULSE_PERIOD_MAX_SEC_SOLO;
  fireflyFreq = 1.0f / pulsePeriodCurrentSec;

  // Colori
  float colorMin = (peerCount==0) ? COLOR_CYCLE_PERIOD_MIN_SEC_SOLO : COLOR_CYCLE_PERIOD_MIN_SEC_PEER;
  float cycleTarget = COLOR_CYCLE_PERIOD_MAX_SEC_SOLO + (colorMin - COLOR_CYCLE_PERIOD_MAX_SEC_SOLO)*factor;
  colorCyclePeriodCurrentSec += COLOR_CYCLE_PERIOD_SMOOTH_ALPHA*(cycleTarget - colorCyclePeriodCurrentSec);
  if(colorCyclePeriodCurrentSec < colorMin) colorCyclePeriodCurrentSec = colorMin;
  if(colorCyclePeriodCurrentSec > COLOR_CYCLE_PERIOD_MAX_SEC_SOLO) colorCyclePeriodCurrentSec=COLOR_CYCLE_PERIOD_MAX_SEC_SOLO;
}

// --- Leader re-election ---
void reevaluateLeaderIfNeeded(uint32_t nowMs){
  bool leaderValid=false;
  if(currentLeaderID==myID) leaderValid=true;
  else {
    for(int i=0;i<MAX_PEERS;i++){
      if(peers[i].valid && peers[i].peerID==currentLeaderID){ leaderValid=true; break; }
    }
  }
  if(leaderValid) return;

  uint8_t newLeader=myID;
  for(int i=0;i<MAX_PEERS;i++){
    if(peers[i].valid && peers[i].peerID < newLeader)
      newLeader=peers[i].peerID;
  }
  if(newLeader != currentLeaderID){
    currentLeaderID=newLeader;
    isLeader=(myID==currentLeaderID);
    uint32_t dynMs=(uint32_t)(colorCyclePeriodCurrentSec*1000.0f);
    if(isLeader && millis()-lastCycleChangeMs > (2*dynMs)){
      advanceColorCycle(nowMs);
      lastColorCycleTickMs=nowMs;
    }
  } else {
    isLeader=(myID==currentLeaderID);
  }
}

// --- LED init ---
void initLeds(){
  strip.begin(); strip.clear(); strip.setBrightness(255); strip.show();
  uint32_t now=millis();
  for(int k=0;k<NUM_LEDS;k++){
    int i=SEQ[k];
    if(LED_PHASE_OFFSET_ENABLED)
      ledStates[i].baseOffset = (float)k/(float)NUM_LEDS;
    else
      ledStates[i].baseOffset = 0.0f;
    ledStates[i].driftOffset = 0.0f;
    ledStates[i].nextDriftUpdate = now + PHASE_DRIFT_INTERVAL_MS + (esp_random()%700);
    ledStates[i].seed = esp_random();
    coldHueCurrent[i]=rand8range(COLD_MIN_HUE,COLD_MAX_HUE);
    coldHueNextChangeMs[i]=now + ZERO_PEER_BASE_INTERVAL_MS +
                           (esp_random() % (ZERO_PEER_COLOR_JITTER_MS+1));
  }
  computeTargetPalette();
  scheduleBlackoutForCycle();
  for(int i=0;i<NUM_LEDS;i++){
    currentHueDisplay[i]=targetCycleHue[i];
    prevCycleHue[i]=targetCycleHue[i];
  }
  fadeActive=false;
  lastCycleChangeMs=now;
  lastColorCycleTickMs=now;
}

// --- Advertising ---
void buildAdvPayload(uint8_t p[12]){
  p[0]=(uint8_t)(COMPANY_ID & 0xFF);
  p[1]=(uint8_t)(COMPANY_ID >> 8);
  p[2]='B'; p[3]='D'; p[4]='G';
  p[5]=PROTO_VERSION;
  p[6]=myID;
  p[7]=(uint8_t)(wrap01(fireflyPhase)*255.0f);
  p[8]=groupHueLocal;
  p[9]=(uint8_t)(warmColorCycle & 0xFF);
  p[10]=(uint8_t)((warmColorCycle>>8)&0xFF);
  p[11]=(isLeader?0x01:0x00);
}
void setupAdvertising(){
  uint8_t payload[12]; buildAdvPayload(payload);
  String s; s.reserve(12); for(int i=0;i<12;i++) s+=(char)payload[i];
  BLEAdvertisementData adv; adv.setManufacturerData(s);
  gAdv=BLEDevice::getAdvertising();
  gAdv->setAdvertisementData(adv);
  gAdv->start();
}
void updateAdvertisingData(bool force=false){
  if(!gAdv) return;
  uint32_t now=millis();
  if(!force && (now - lastAdvUpdateMs) < ADV_UPDATE_INTERVAL_MS) return;
  lastAdvUpdateMs=now;
  uint8_t payload[12]; buildAdvPayload(payload);
  String s; s.reserve(12); for(int i=0;i<12;i++) s+=(char)payload[i];
  BLEAdvertisementData adv; adv.setManufacturerData(s);
  gAdv->setAdvertisementData(adv);
}

// --- Timer color cycle ---
void leaderColorCycleTimer(uint32_t nowMs,bool peerMode){
  if(!peerMode) return;
  if(!isLeader) return;
  uint32_t intervalMs=(uint32_t)(colorCyclePeriodCurrentSec*1000.0f);
  if(nowMs - lastColorCycleTickMs >= intervalMs){
    lastColorCycleTickMs += intervalMs;
    if(nowMs - lastColorCycleTickMs > intervalMs) lastColorCycleTickMs=nowMs;
    advanceColorCycle(nowMs);
  }
  if(nowMs - lastCycleChangeMs > (2*intervalMs)){
    lastColorCycleTickMs=nowMs;
    advanceColorCycle(nowMs);
  }
}

// --- Group hue ---
void updateGroupHue(uint8_t peerCount){
  if(peerCount==0){
    if(millis()-lastSoloHueDriftMs > 1800){
      lastSoloHueDriftMs=millis();
      int8_t step=(int8_t)rand8range(0,2)-1;
      groupHueLocal=clampCold(hueAdd(groupHueLocal, step));
    }
    groupHueTarget=groupHueLocal;
  }else{
    int8_t d=hueDiffSigned(groupHueTarget,groupHueLocal);
    float step=d*GROUP_HUE_COUPLING;
    if(step>6) step=6; if(step<-6) step=-6;
    groupHueLocal=hueAdd(groupHueLocal,(int16_t)lroundf(step));
  }
}

// --- Solo cold update ---
void updateSoloColdColors(uint32_t now){
  for(int i=0;i<NUM_LEDS;i++){
    if(now >= coldHueNextChangeMs[i]){
      coldHueCurrent[i]=rand8range(COLD_MIN_HUE,COLD_MAX_HUE);
      coldHueNextChangeMs[i]= now + ZERO_PEER_BASE_INTERVAL_MS +
                              (esp_random() % (ZERO_PEER_COLOR_JITTER_MS+1));
    }
  }
}

// --- Phase drift update ---
void updatePhaseDrift(uint32_t now, float factor){
  if(!ENABLE_PEER_PHASE_DRIFT || factor<=0.0f) {
    for(int i=0;i<NUM_LEDS;i++) ledStates[i].driftOffset=0.0f;
    return;
  }
  for(int i=0;i<NUM_LEDS;i++){
    if(now >= ledStates[i].nextDriftUpdate){
      // nuova deriva piccola
      float sign = ((ledStates[i].seed ^ prngNext()) & 1)? 1.0f : -1.0f;
      float mag  = ((prngNext() & 0xFFFF) / 65535.0f) * PHASE_DRIFT_MAX_DELTA * factor;
      ledStates[i].driftOffset = sign * mag;
      ledStates[i].nextDriftUpdate = now + PHASE_DRIFT_INTERVAL_MS +
                                     (prngNext() % 900);
    }
  }
}

// --- Hue wobble ---
uint8_t applyHueWobble(uint8_t baseHue, float phase, uint32_t seed, float factor){
  if(!ENABLE_HUE_WOBBLE || factor<=0.0f) return baseHue;
  float w = sinf(phase* TWO_PI + (seed & 0xFF)*0.0245436926f); // seed-based
  int8_t wob = (int8_t) lroundf(w * (HUE_WOBBLE_MAX * factor));
  return (uint8_t)(baseHue + wob); // warp (wrap automatic in rendering via 8 bit)
}

// --- RSSI logging (optional) ---
#if PEER_RSSI_DEBUG
void printPeerRSSI(){
  uint8_t count=countValidPeers();
  if(count==0){ Serial.println("[PEERS] none"); return; }
  for(int i=0;i<MAX_PEERS;i++){
    if(!peers[i].valid) continue;
    Serial.print("#"); Serial.print(i);
    Serial.print(" RSSI="); Serial.print(peers[i].lastRSSI);
    Serial.print(" ID="); Serial.print(peers[i].peerID);
    Serial.print(" MAC="); Serial.println(peers[i].addr.toString().c_str());
  }
}
#endif

// --- Update LEDs ---
void updateLeds(uint32_t nowMs, float dt){
  float raw=computePresenceRaw();
  reevaluateLeaderIfNeeded(nowMs);

  presenceFactor += PRESENCE_ALPHA*(raw - presenceFactor);
  presenceFactor = constrain(presenceFactor, 0.0f, 1.0f);

  uint8_t peerCount = countValidPeers();
  bool soloMode = (peerCount==0);

  updateDynamicPeriods(peerCount);

  float presenceDelta=presenceFactor - previousPresence;
  if(presenceDelta > 0.12f && !soloMode){
    bloomEndMs=nowMs + BLOOM_DURATION_MS;
    bloomActive=true;
  }
  previousPresence=presenceFactor;

  float prevPhase=fireflyPhase;
  fireflyPhase += fireflyFreq * dt;
  fireflyPhase = wrap01(fireflyPhase);
  if(!soloMode && neighborPhaseAvailable){
    float eff=0.18f*(0.4f + 0.6f*presenceFactor);
    float diff=phaseDiff(neighborMeanPhase, fireflyPhase);
    fireflyPhase=wrap01(fireflyPhase + diff*eff);
  }

  leaderColorCycleTimer(nowMs, !soloMode);
  updateGroupHue(peerCount);
  if(soloMode) updateSoloColdColors(nowMs);

  updateFade(nowMs, peerCount);

  // Blackout ciclico
  if(!soloMode){
    if(blackoutScheduled && !blackoutActive){
      bool crossed=false;
      if(prevPhase <= blackoutPhase){
        if(fireflyPhase >= blackoutPhase) crossed=true;
      } else {
        if(fireflyPhase >= blackoutPhase || fireflyPhase < prevPhase) crossed=true;
      }
      if(crossed){
        blackoutActive=true;
        blackoutEndMs=nowMs + BLACKOUT_DURATION_MS;
      }
    }
    if(blackoutActive && nowMs >= blackoutEndMs){
      blackoutActive=false;
      startPostFlash(nowMs);
    }
  } else {
    if(blackoutActive){ blackoutActive=false; startPostFlash(nowMs); }
  }
  updatePostFlash(nowMs);

  // Peak target
  float factor=(peerCount >= MAX_PEER_ACCEL_PEERS)?1.0f:(float)peerCount/MAX_PEER_ACCEL_PEERS;
  float targetPeak = soloMode ? ZERO_PEER_PEAK : (factor * PEAK_SCALE_MAX);
  if(targetPeak > 1.35f) targetPeak=1.35f;

  float alpha=PEAK_SMOOTH_ALPHA;
  if(targetPeak < smoothedPeak) alpha *= PEAK_SMOOTH_ALPHA_FALL_MULT;
  smoothedPeak += alpha*(targetPeak - smoothedPeak);
  smoothedPeak = constrain(smoothedPeak, 0.0f, 1.4f);

  // Phase drift & wobble updates
  updatePhaseDrift(nowMs, factor);

  // Global shimmer
  float globalShimmer = 1.0f;
  if(!soloMode && ENABLE_PEER_GLOBAL_SHIMMER){
    float sg = sinf(fireflyPhase * TWO_PI);
    globalShimmer = 1.0f + PEER_GLOBAL_SHIMMER_DEPTH * sg;
    if(globalShimmer < 0) globalShimmer=0;
  }

  // Render
  for(int i=0;i<NUM_LEDS;i++){
    uint32_t out=0;
    if(blackoutActive){
      out=0;
    } else if(postFlashActive){
      uint8_t v=(uint8_t)lroundf(POST_FLASH_FIXED_INTENSITY * 255.0f);
      out = strip.Color(v,v,v);
    } else {
      float phase_i = wrap01(fireflyPhase + ledStates[i].baseOffset + ledStates[i].driftOffset);
      float wave = (sinf(phase_i * TWO_PI) + 1.0f)*0.5f; // 0..1
      float valFrac = wave * smoothedPeak * globalShimmer;

      if(!soloMode && ENABLE_PEER_PIXEL_SHIMMER){
        float sh=sinf((fireflyPhase*TWO_PI*SHIMMER_FREQ_MULT) + i*SHIMMER_PHASE_SPREAD);
        float pixelShimmer = 1.0f + PEER_PIXEL_SHIMMER_DEPTH * sh * factor;
        if(pixelShimmer<0) pixelShimmer=0;
        valFrac *= pixelShimmer;
      }

      if(!soloMode && ENABLE_PEER_MICRO_SPARKLE && wave > SPARKLE_WAVE_THRESHOLD){
        uint32_t r=esp_random();
        float jitter = ((r & 0x3FF)/1023.0f) - 0.5f; // -0.5..0.5
        valFrac += jitter * (factor * PEER_MICRO_SPARKLE_MAX);
      }

      if(valFrac<0) valFrac=0;
      if(valFrac>1) valFrac=1;

      uint8_t baseHue = soloMode ? coldHueCurrent[i] : currentHueDisplay[i];
      if(!soloMode){
        baseHue = applyHueWobble(baseHue, fireflyPhase, ledStates[i].seed, factor);
      }

      uint16_t hue16 = (uint16_t)baseHue * 257;
      uint8_t S=255;
      uint8_t V=(uint8_t)lroundf(valFrac*255.0f);
      out=strip.ColorHSV(hue16,S,V);
    }
    strip.setPixelColor(i,out);
  }
  strip.show();

  if(bloomActive && nowMs > bloomEndMs) bloomActive=false;

  if(nowMs - lastStatusLogMs > STATUS_LOG_INTERVAL_MS){
    lastStatusLogMs=nowMs;
    Serial.print("[STATUS] peers="); Serial.print(peerCount);
    Serial.print(" factor="); Serial.print((peerCount>=MAX_PEER_ACCEL_PEERS)?1.0f:(float)peerCount/MAX_PEER_ACCEL_PEERS,2);
    Serial.print(" cycle="); Serial.print((unsigned long)warmColorCycle);
    Serial.print(" pulsePeriod="); Serial.print(pulsePeriodCurrentSec,2);
    Serial.print("s colorPeriod="); Serial.print(colorCyclePeriodCurrentSec,2);
    Serial.print("s peak="); Serial.print(smoothedPeak,2);
    Serial.print(" blackout="); Serial.print(blackoutActive);
    Serial.print(" flash="); Serial.print(postFlashActive);
    Serial.println();
#if PEER_RSSI_DEBUG
    printPeerRSSI();
#endif
#ifdef COLOR_DEBUG
    Serial.print("HUES: ");
    for(int j=0;j<NUM_LEDS;j++){ Serial.print(currentHueDisplay[j]); Serial.print(' '); }
    Serial.println();
#endif
  }
}

// --- Scan callback ---
class PeerScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    int8_t rssi=dev.getRSSI();
    if(rssi < PEER_RSSI_CUTOFF_DBM) return;
    if(!dev.haveManufacturerData()) return;
    String md=dev.getManufacturerData();
    if(md.length() < 9) return;
    const uint8_t* m=(const uint8_t*)md.c_str();
    uint16_t cid=m[0]|(m[1]<<8);
    if(cid!=COMPANY_ID) return;
    if(m[2]!='B'||m[3]!='D'||m[4]!='G') return;
    uint8_t peerProto=m[5];
    uint8_t peerID=m[6];

    int slot=findPeerSlot(dev.getAddress());
    if(slot<0) return;
    bool wasValid=peers[slot].valid;

    peers[slot].addr     = dev.getAddress();
    peers[slot].lastRSSI = rssi;
    peers[slot].lastSeen = millis();
    peers[slot].valid    = true;
    peers[slot].peerID   = peerID;

    uint8_t phaseByte=m[7];
    peers[slot].phase=(float)phaseByte/255.0f;
    peers[slot].phaseValid=true;

    if(peerProto>=6 && md.length()>=12){
      uint16_t peerCycle=(uint16_t)m[9] | ((uint16_t)m[10]<<8);
      peers[slot].colorCycle=peerCycle;
      peers[slot].cycleValid=true;
      if(peerCycle > warmColorCycle){
        adoptColorCycle(peerCycle, millis());
      }
    } else {
      peers[slot].cycleValid=false;
    }

    if(peerID < currentLeaderID){
      currentLeaderID=peerID;
      isLeader=(myID == currentLeaderID);
    }
    neighborPhaseAvailable=true;

    if(!wasValid){
      triggerNewPeerBlackout(millis());
    }
  }
};

void setupContinuousScan(){
  BLEScan* scan=BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new PeerScanCallbacks(), true);
  scan->setActiveScan(true);
  scan->setInterval(120);
  scan->setWindow(120);
  scan->start(0,nullptr);
}

// --- BLE util ---
uint8_t deriveMyIDFromBLEAddr(){
  String addr=BLEDevice::getAddress().toString();
  int pos=addr.lastIndexOf(':');
  String last=(pos>=0)?addr.substring(pos+1):addr;
  last.trim();
  char buf[3]={0,0,0};
  last.toCharArray(buf,3);
  uint8_t v=(uint8_t)strtoul(buf,nullptr,16);
  if(v==0) v=(uint8_t)(esp_random() & 0xFF);
  return v;
}

// --- Setup / Loop ---
void setup(){
  Serial.begin(115200);
  delay(120);
  BLEDevice::init("Badge");
  myID=deriveMyIDFromBLEAddr();
  currentLeaderID=myID;
  isLeader=true;
  Serial.print("PROTO_VERSION="); Serial.println(PROTO_VERSION);
  Serial.print("myID="); Serial.println(myID);

  initLeds();
  setupAdvertising();
  setupContinuousScan();
}

void loop(){
  uint32_t now=millis();
  if(now - lastFrameMs >= FRAME_INTERVAL_MS){
    float dt = (lastFrameMs==0)? (FRAME_INTERVAL_MS/1000.0f):(now - lastFrameMs)/1000.0f;
    lastFrameMs=now;
    updateLeds(now, dt);
    updateAdvertisingData();
  }
  delay(1);
}
