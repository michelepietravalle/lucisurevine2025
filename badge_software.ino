/*
 badge_software.ino (v24c-peer-bursts-5percent-smoothfade)

 - Peer mode: pulsazione sincronizzata a 1.00 s, ordine SEQ uguale per tutti.
 - Cambio colori lento ~3.2 s in peer, con cross-fade più dolce (easing smoothstep).
 - Flash bianco post-blackout all'8% (sempre).
 - Luminosità colori ridotta del 25% SOLO in peer mode.
 - Nuovo peer: lampeggia (blackout+flash) per un numero di volte pari ai peer presenti al momento dell'arrivo.
 - Blackout casuale al 5% (per ciclo), disabilitato durante burst nuovo peer.
 - Rielezione leader, LED_PHASE_OFFSET_ENABLED = true.
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

// Presenza / RSSI
const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -100;
const int8_t   RSSI_MAX_DBM         = -45;
const int8_t   PEER_RSSI_CUTOFF_DBM = -70;
const float    PRESENCE_ALPHA       = 0.30f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

// Protocol
const uint8_t  PROTO_VERSION = 6;

// Solo-mode cold hues
const uint8_t  COLD_MIN_HUE = 140;
const uint8_t  COLD_MAX_HUE = 200;

// Residual hue coupling
const float GROUP_HUE_COUPLING = 0.055f;

// Periodi pulsazione (solo/peer)
const float PULSE_PERIOD_MAX_SEC_SOLO = 3.0f;
const float PULSE_PERIOD_MIN_SEC_SOLO = 1.20f;
const float PULSE_PERIOD_PEER_SEC     = 1.00f; // forzato con >=2 peer

// Periodi cambio colori (solo/peer)
const float COLOR_CYCLE_PERIOD_MAX_SEC_SOLO = 3.0f;
const float COLOR_CYCLE_PERIOD_MIN_SEC_SOLO = 1.50f;
const float COLOR_CYCLE_PERIOD_PEER_SEC     = 3.20f; // ~3–4 s

// Smoothing dinamico
const float PULSE_PERIOD_SMOOTH_ALPHA       = 0.35f; // aggancio rapido
const float COLOR_CYCLE_PERIOD_SMOOTH_ALPHA = 0.25f;

// Frame / adv
const uint32_t FRAME_INTERVAL_MS      = 30;
const uint32_t ADV_UPDATE_INTERVAL_MS = 100;

// Peak smoothing
const float PEAK_SMOOTH_ALPHA           = 0.06f;
const float PEAK_SMOOTH_ALPHA_FALL_MULT = 2.6f;

// Peak targets
const float ZERO_PEER_PEAK = 0.45f;
const float PEAK_SCALE_MAX = 1.10f; // clamp a valle

// Disattiva elementi che rompono la sincronia in peer mode
const bool  ENABLE_PEER_GLOBAL_SHIMMER = false;
const bool  ENABLE_PEER_PIXEL_SHIMMER  = false;
const bool  ENABLE_PEER_MICRO_SPARKLE  = false;
const bool  ENABLE_PEER_PHASE_DRIFT    = false;
const bool  ENABLE_HUE_WOBBLE          = false;

// Sweep progressivo (peer mode)
const float SWEEP_SPEED_MULT = 1.00f;
const float SWEEP_WIDTH      = 0.42f;
const int   SWEEP_COUNT      = 1;
const float SWEEP_MIX        = 1.00f;

// Bloom
const uint32_t BLOOM_DURATION_MS = 600;

// Cross-fade palette (solo / peer) — aumentati per transizioni più dolci
const uint16_t CROSS_FADE_MS_SOLO = 220; // prima 150
const uint16_t CROSS_FADE_MS_PEER = 320; // prima 70

// Blackout casuale (deterministico per ciclo)
const uint8_t  BLACKOUT_PROB        = 13;   // ~5% su 256
const uint16_t BLACKOUT_DURATION_MS = 120;

// Blackout nuovo peer (burst)
#define BLACKOUT_NEW_PEER_ENABLED 1
const uint16_t BLACKOUT_NEW_PEER_DURATION_MS = 150;
const bool     NEW_PEER_BLACKOUT_EXTEND      = true;
const uint16_t NEW_PEER_BURST_GAP_MS         = 60;  // pausa tra colpi della burst

// Flash bianco post-blackout
#define POST_FLASH_ENABLED 1
const uint16_t POST_FLASH_DURATION_MS     = 130;
const float    POST_FLASH_FIXED_INTENSITY = 0.08f; // 8%

// Scala luminosità colori in peer mode (ridotta del 25%)
const float COLOR_BRIGHTNESS_SCALE = 0.75f; // solo in peer mode

// Phase offset per LED
const bool LED_PHASE_OFFSET_ENABLED = true;

// Solo mode cold color jitter
const uint32_t ZERO_PEER_BASE_INTERVAL_MS = 3000;
const uint32_t ZERO_PEER_COLOR_JITTER_MS  = 900;

// Sequenza fissa (ordine sincronizzato)
static const int SEQ[NUM_LEDS] = {0,2,4,6,1,3,5};

// Strutture
struct LedState {
  float baseOffset;
  float driftOffset;
  uint32_t nextDriftUpdate;
  uint32_t seed;
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

// Stato globale
static const uint8_t MAX_PEERS=16;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

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

uint8_t  groupHueLocal=170, groupHueTarget=170;
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
bool     fadeActive=false;

// Rank per sweep progressivo
uint8_t rankOfLed[NUM_LEDS];

// PRNG (solo per palette deterministica)
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

// Burst nuovo peer
bool     newPeerBurstActive=false;
uint8_t  newPeerBurstRemaining=0;
uint32_t newPeerNextStartMs=0;

// ---------- Forward declarations ----------
void setupAdvertising();
void updateAdvertisingData(bool force=false);
void setupContinuousScan();
void initLeds();
void updateLeds(uint32_t nowMs, float dt);
void updateSoloColdColors(uint32_t now);
void leaderColorCycleTimer(uint32_t nowMs, bool peerMode);
void updateGroupHue(uint8_t peerCount);
bool computeNeighborPhaseMean(float& outPhase);
void updateDynamicPeriods(uint8_t peerCount);
void reevaluateLeaderIfNeeded(uint32_t nowMs);
void scheduleBlackoutForCycle();
void startPostFlash(uint32_t now);
void updatePostFlash(uint32_t now);
void advanceColorCycle(uint32_t nowMs);
void adoptColorCycle(uint32_t newCycle,uint32_t nowMs);
void startNewCycleFade(uint32_t nowMs);
uint16_t currentCrossFadeMs(uint8_t peerCount);
void computeTargetPalette();

// ---------- Utility ----------
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

// Easing morbida per cross-fade (smoothstep)
static inline float easeSmoothstep(float t){
  if(t<=0) return 0.0f;
  if(t>=1) return 1.0f;
  return t*t*(3.0f - 2.0f*t);
}

// ---------- Palette ----------
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
  return CROSS_FADE_MS_PEER;
}

// ---------- Blackout scheduling ----------
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

// ---------- Blackout nuovo peer (burst) ----------
void triggerNewPeerBlackout(uint32_t now){
#if BLACKOUT_NEW_PEER_ENABLED
  uint8_t pc = countValidPeers(); // include il nuovo peer già marcato valido
  if(pc==0) return;
  // Attiva/accoda una burst con un numero di colpi pari ai peer presenti
  newPeerBurstActive = true;
  newPeerBurstRemaining += pc;

  // Se siamo liberi, lancia subito il prossimo blackout
  if(!blackoutActive && !postFlashActive && now >= newPeerNextStartMs){
    blackoutActive = true;
    blackoutEndMs  = now + BLACKOUT_NEW_PEER_DURATION_MS;
    if(newPeerBurstRemaining>0) newPeerBurstRemaining--;
  }
#endif
}

// ---------- Post Flash ----------
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

// ---------- Cycle transitions ----------
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
  // Easing smoothstep per transizione più dolce
  float te = easeSmoothstep(t);
  for(int i=0;i<NUM_LEDS;i++){
    currentHueDisplay[i]=hueLerpCircular(prevCycleHue[i], targetCycleHue[i], te);
  }
}

// ---------- Neighbor phase (media vettoriale) ----------
bool computeNeighborPhaseMean(float& outPhase){
  float sx=0.0f, sy=0.0f, wsum=0.0f;
  for(int i=0;i<MAX_PEERS;i++){
    if(!peers[i].valid || !peers[i].phaseValid) continue;
    float w = rssiToWeight(peers[i].lastRSSI);
    float ang = peers[i].phase * TWO_PI;
    sx += cosf(ang)*w;
    sy += sinf(ang)*w;
    wsum += w;
  }
  if(wsum <= 0.0f) return false;
  float ang = atan2f(sy, sx);
  if(ang < 0) ang += TWO_PI;
  outPhase = ang / TWO_PI;
  return true;
}

// ---------- Periodi dinamici ----------
void updateDynamicPeriods(uint8_t peerCount){
  if(peerCount==0){
    // Solo mode
    float pulseTarget = PULSE_PERIOD_MAX_SEC_SOLO + (PULSE_PERIOD_MIN_SEC_SOLO - PULSE_PERIOD_MAX_SEC_SOLO)*presenceFactor;
    pulsePeriodCurrentSec += PULSE_PERIOD_SMOOTH_ALPHA*(pulseTarget - pulsePeriodCurrentSec);
    pulsePeriodCurrentSec = constrain(pulsePeriodCurrentSec, PULSE_PERIOD_MIN_SEC_SOLO, PULSE_PERIOD_MAX_SEC_SOLO);
    fireflyFreq = 1.0f / pulsePeriodCurrentSec;

    float cycleTarget = COLOR_CYCLE_PERIOD_MAX_SEC_SOLO + (COLOR_CYCLE_PERIOD_MIN_SEC_SOLO - COLOR_CYCLE_PERIOD_MAX_SEC_SOLO)*presenceFactor;
    colorCyclePeriodCurrentSec += COLOR_CYCLE_PERIOD_SMOOTH_ALPHA*(cycleTarget - colorCyclePeriodCurrentSec);
    colorCyclePeriodCurrentSec = constrain(colorCyclePeriodCurrentSec, COLOR_CYCLE_PERIOD_MIN_SEC_SOLO, COLOR_CYCLE_PERIOD_MAX_SEC_SOLO);
  } else {
    // Peer mode sincronizzato
    float pulseTarget = (peerCount>=2) ? PULSE_PERIOD_PEER_SEC : 1.20f;
    pulsePeriodCurrentSec += PULSE_PERIOD_SMOOTH_ALPHA*(pulseTarget - pulsePeriodCurrentSec);
    pulsePeriodCurrentSec = constrain(pulsePeriodCurrentSec, PULSE_PERIOD_PEER_SEC, PULSE_PERIOD_MAX_SEC_SOLO);
    fireflyFreq = 1.0f / pulsePeriodCurrentSec;

    float cycleTarget = COLOR_CYCLE_PERIOD_PEER_SEC;
    colorCyclePeriodCurrentSec += COLOR_CYCLE_PERIOD_SMOOTH_ALPHA*(cycleTarget - colorCyclePeriodCurrentSec);
    colorCyclePeriodCurrentSec = constrain(colorCyclePeriodCurrentSec, COLOR_CYCLE_PERIOD_PEER_SEC, COLOR_CYCLE_PERIOD_MAX_SEC_SOLO);
  }
}

// ---------- Leader re-election ----------
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

// ---------- Group hue ----------
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

// ---------- Timer ciclo colori ----------
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

// ---------- Advertising ----------
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
void updateAdvertisingData(bool force){
  if(!gAdv) return;
  uint32_t now=millis();
  if(!force && (now - lastAdvUpdateMs) < ADV_UPDATE_INTERVAL_MS) return;
  lastAdvUpdateMs=now;
  uint8_t payload[12]; buildAdvPayload(payload);
  String s; s.reserve(12); for(int i=0;i<12;i++) s+=(char)payload[i];
  BLEAdvertisementData adv; adv.setManufacturerData(s);
  gAdv->setAdvertisementData(adv);
}

// ---------- Scan callback ----------
class PeerScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    int8_t rssi=dev.getRSSI();
    if(rssi < PEER_RSSI_CUTOFF_DBM) return;
    if(!dev.haveManufacturerData()) return;
    String md=dev.getManufacturerData();
    if(md.length() < 12) return;
    const uint8_t* m=(const uint8_t*)md.c_str();
    uint16_t cid=m[0]|(m[1]<<8);
    if(cid!=COMPANY_ID) return;
    if(m[2]!='B'||m[3]!='D'||m[4]!='G') return;

    uint8_t peerProto=m[5];
    uint8_t peerID   =m[6];

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

    if(peerProto>=6){
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

// ---------- Continuous scan ----------
void setupContinuousScan(){
  BLEScan* scan=BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new PeerScanCallbacks(), true);
  scan->setActiveScan(true);
  scan->setInterval(120);
  scan->setWindow(120);
  scan->start(0,nullptr);
}

// ---------- Solo cold update ----------
void updateSoloColdColors(uint32_t now){
  for(int i=0;i<NUM_LEDS;i++){
    if(now >= coldHueNextChangeMs[i]){
      coldHueCurrent[i]=rand8range(COLD_MIN_HUE,COLD_MAX_HUE);
      coldHueNextChangeMs[i]= now + ZERO_PEER_BASE_INTERVAL_MS +
                              (esp_random() % (ZERO_PEER_COLOR_JITTER_MS+1));
    }
  }
}

// ---------- Init LED ----------
void initLeds(){
  strip.begin(); strip.clear(); strip.setBrightness(255); strip.show();
  uint32_t now=millis();
  // rank map per sweep progressivo
  for(int k=0;k<NUM_LEDS;k++){
    rankOfLed[ SEQ[k] ] = k;
  }
  for(int k=0;k<NUM_LEDS;k++){
    int i=SEQ[k];
    ledStates[i].baseOffset = LED_PHASE_OFFSET_ENABLED ? (float)k/(float)NUM_LEDS : 0.0f;
    ledStates[i].driftOffset = 0.0f;
    ledStates[i].nextDriftUpdate = now + 2000;
    ledStates[i].seed = 0;

    coldHueCurrent[i]=rand8range(COLD_MIN_HUE,COLD_MAX_HUE);
    coldHueNextChangeMs[i]= now + ZERO_PEER_BASE_INTERVAL_MS +
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

// ---------- Rendering ----------
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

  // Aggiorna media fase neighbor
  float meanPhaseTmp=0.0f;
  neighborPhaseAvailable = computeNeighborPhaseMean(meanPhaseTmp);
  if(neighborPhaseAvailable) neighborMeanPhase = meanPhaseTmp;

  float prevPhase=fireflyPhase;
  fireflyPhase += fireflyFreq * dt;
  fireflyPhase = wrap01(fireflyPhase);
  if(!soloMode && neighborPhaseAvailable){
    float eff=0.30f*(0.4f + 0.6f*presenceFactor);
    float diff=phaseDiff(neighborMeanPhase, fireflyPhase);
    fireflyPhase=wrap01(fireflyPhase + diff*eff);
  }

  leaderColorCycleTimer(nowMs, !soloMode);
  updateGroupHue(peerCount);
  if(soloMode) updateSoloColdColors(nowMs);

  updateFade(nowMs, peerCount);

  // Blackout ciclico (disattivato durante burst nuovo peer)
  if(!soloMode){
    if(!newPeerBurstActive && blackoutScheduled && !blackoutActive){
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
      // Se siamo in burst nuovo peer, programma il prossimo colpo
      if(newPeerBurstActive){
        newPeerNextStartMs = postFlashEndMs + NEW_PEER_BURST_GAP_MS;
      }
    }
  } else {
    if(blackoutActive){ blackoutActive=false; startPostFlash(nowMs); }
  }
  updatePostFlash(nowMs);

  // Pianifica successivi colpi della burst nuovo peer
  if(newPeerBurstActive){
    if(!blackoutActive && !postFlashActive){
      if(newPeerBurstRemaining>0 && nowMs >= newPeerNextStartMs){
        blackoutActive=true;
        blackoutEndMs=nowMs + BLACKOUT_NEW_PEER_DURATION_MS;
        newPeerBurstRemaining--;
      } else if(newPeerBurstRemaining==0){
        newPeerBurstActive=false;
      }
    }
  }

  // Peak target
  float targetPeak = soloMode ? ZERO_PEER_PEAK : 1.0f;
  float alpha=PEAK_SMOOTH_ALPHA;
  if(targetPeak < smoothedPeak) alpha *= PEAK_SMOOTH_ALPHA_FALL_MULT;
  smoothedPeak += alpha*(targetPeak - smoothedPeak);
  smoothedPeak = constrain(smoothedPeak, 0.0f, PEAK_SCALE_MAX);

  // Direzione sweep alternata per ciclo
  float c = fireflyPhase * SWEEP_SPEED_MULT;
  if(((warmColorCycle & 1) == 1)) c = 1.0f - c;
  c = wrap01(c);

  // Render
  for(int i=0;i<NUM_LEDS;i++){
    uint32_t out=0;
    if(blackoutActive){
      out=0;
    } else if(postFlashActive){
      uint8_t v=(uint8_t)lroundf(POST_FLASH_FIXED_INTENSITY * 255.0f);
      out = strip.Color(v,v,v);
    } else {
      float valFrac=0.0f;

      if(soloMode){
        float phase_i = wrap01(fireflyPhase + ledStates[i].baseOffset);
        float wave = (sinf(phase_i * TWO_PI) + 1.0f)*0.5f;
        valFrac = wave * smoothedPeak;
      } else {
        float idxNorm = (float)rankOfLed[i] / (float)NUM_LEDS;
        float env = 0.0f;
        for(int s=0;s<SWEEP_COUNT;s++){
          float center = wrap01(c + (s * (1.0f / SWEEP_COUNT)));
          float d = fabs(idxNorm - center);
          if(d > 0.5f) d = 1.0f - d;
          float e = 1.0f - (d / SWEEP_WIDTH);
          if(e < 0.0f) e = 0.0f;
          e = e*e;
          if(e > env) env = e;
        }
        float gate = (1.0f - SWEEP_MIX) + SWEEP_MIX * env;
        valFrac = smoothedPeak * gate;
      }

      if(valFrac<0) valFrac=0;
      if(valFrac>1) valFrac=1;

      uint8_t baseHue = soloMode ? coldHueCurrent[i] : currentHueDisplay[i];
      uint16_t hue16 = (uint16_t)baseHue * 257;

      float brightnessScale = soloMode ? 1.0f : COLOR_BRIGHTNESS_SCALE;
      uint8_t  V=(uint8_t)lroundf(valFrac * brightnessScale * 255.0f);

      out=strip.ColorHSV(hue16, 255, V);
    }
    strip.setPixelColor(i,out);
  }
  strip.show();

  if(bloomActive && nowMs > BLOOM_DURATION_MS) bloomActive=false;

  if(nowMs - lastStatusLogMs > STATUS_LOG_INTERVAL_MS){
    lastStatusLogMs=nowMs;
    Serial.print("[STATUS] peers="); Serial.print(peerCount);
    Serial.print(" cycle="); Serial.print((unsigned long)warmColorCycle);
    Serial.print(" pulse="); Serial.print(pulsePeriodCurrentSec,2);
    Serial.print("s color="); Serial.print(colorCyclePeriodCurrentSec,2);
    Serial.print("s peak="); Serial.print(smoothedPeak,2);
    Serial.print(" blackout="); Serial.print(blackoutActive);
    Serial.print(" flash="); Serial.print(postFlashActive);
    Serial.print(" npBurst="); Serial.print(newPeerBurstActive);
    Serial.print("/"); Serial.print(newPeerBurstRemaining);
    Serial.println();
#ifdef COLOR_DEBUG
    Serial.print("HUES: ");
    for(int j=0;j<NUM_LEDS;j++){ Serial.print(currentHueDisplay[j]); Serial.print(' '); }
    Serial.println();
#endif
  }
}

// ---------- BLE util ----------
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

// ---------- Setup / Loop ----------
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
    updateAdvertisingData(false);
  }
  delay(1);
}
