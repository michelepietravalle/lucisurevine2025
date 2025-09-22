/*
 badge_warm_pulse_sync_ordered_v13.ino

 Estende v12:
 - Logging RSSI dei peer (dBm) + peso normalizzato per il calcolo della presenza.
 - Attivabile/disattivabile via macro PEER_RSSI_DEBUG (attiva di default).
 - Nessuna modifica alla logica di colori, sincronizzazione o cross-fade.

 Modalità PEER:
   Palette di 7 colori (viola, arancione, verde, giallo, rosso, rosa, magenta)
   rigenerata ogni 3s come permutazione pseudo-casuale deterministica (PRNG XORSHIFT32 seed = warmColorCycle).
   Cross-fade 200 ms tra palette.

 Modalità SOLO:
   Colori freddi (140..200 hue) casuali per LED, cambio ~3s, pulsazione minima.

 Leader:
   - Badge con myID più basso (ultimo byte MAC)
   - Incrementa warmColorCycle ogni 3000 ms
   - Failsafe se ciclo non avanza entro 2 intervalli

 Log:
   - [STATUS] ogni 1500 ms
   - Con PEER_RSSI_DEBUG: elenco peer con RSSI, peso e MAC

*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_NeoPixel.h>

// ------------------------------------------------------------------
// Company ID placeholder
#ifndef COMPANY_ID
#define COMPANY_ID 0xFFFF
#endif
// ------------------------------------------------------------------

// Abilita/Disabilita logging esteso RSSI peer
#define PEER_RSSI_DEBUG 1

// #define COLOR_DEBUG 1  // (facoltativo) log degli hue

#define LED_PIN   16
#define NUM_LEDS  7

const uint32_t STATUS_LOG_INTERVAL_MS = 1500;

// Presenza / RSSI
const uint32_t BEACON_TIMEOUT_MS    = 4000;
const int8_t   RSSI_MIN_DBM         = -100;
const int8_t   RSSI_MAX_DBM         = -45;
const float    PRESENCE_ALPHA       = 0.30f;
const uint8_t  PEERS_FOR_FULL_SCALE = 3;

// Protocol
const uint8_t  PROTO_VERSION        = 6;

// Hue freddi solo-mode
const uint8_t  COLD_MIN_HUE         = 140;
const uint8_t  COLD_MAX_HUE         = 200;

// Coupling residuo (non influenza la palette random)
const float    GROUP_HUE_COUPLING   = 0.055f;

// Timings
const uint32_t FRAME_INTERVAL_MS      = 30;
const uint32_t ADV_UPDATE_INTERVAL_MS = 150;
const uint32_t SCAN_PERIOD_MS         = 900;
const uint32_t SCAN_DURATION_MS       = 700;

const float    COLOR_CHANGE_INTERVAL_SEC = 3.0f;
const uint32_t COLOR_CHANGE_INTERVAL_MS  = (uint32_t)(COLOR_CHANGE_INTERVAL_SEC * 1000.0f);

// Pulsazione
const float SLOW_PULSE_FREQ_HZ          = 1.0f / COLOR_CHANGE_INTERVAL_SEC;
const float PEAK_SMOOTH_ALPHA           = 0.03f;
const float PEAK_SMOOTH_ALPHA_FALL_MULT = 2.2f;
const float MIN_FRACTION                = 0.02f;

// Solo mode
const float    ZERO_PEER_PEAK             = 0.03f;
const float    ZERO_PEER_MIN_FRACTION     = 0.0f;
const uint32_t ZERO_PEER_BASE_INTERVAL_MS = 3000;
const uint32_t ZERO_PEER_COLOR_JITTER_MS  = 900;

// Shimmer
const float   SHIMMER_FREQ_MULT    = 2.0f;
const float   SHIMMER_DEPTH        = 0.15f;
const float   SHIMMER_PHASE_SPREAD = 1.7f;

// Bloom
const uint32_t BLOOM_DURATION_MS   = 600;

// Cross-fade
const uint16_t CROSS_FADE_MS       = 200;

// Sequenza picchi (ordine percepito 1-3-5-7-2-4-6)
static const int SEQ[NUM_LEDS] = {0,2,4,6,1,3,5};

struct LedState {
  float phaseOffset;
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

static const uint8_t MAX_PEERS = 16;

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Stato runtime
LedState  ledStates[NUM_LEDS];
BadgePeer peers[MAX_PEERS];

uint8_t  coldHueCurrent[NUM_LEDS];
uint32_t coldHueNextChangeMs[NUM_LEDS];

float presenceFactor        = 0.0f;
float previousPresence      = 0.0f;

uint32_t lastFrameMs        = 0;
uint32_t lastScanStartMs    = 0;
uint32_t lastAdvUpdateMs    = 0;
uint32_t lastStatusLogMs    = 0;

uint8_t  myID               = 0;

// Pulsazione
float fireflyPhase          = 0.0f;
float fireflyFreq           = SLOW_PULSE_FREQ_HZ;
float prevFireflyPhase      = 0.0f;

// Ciclo palette
uint32_t warmColorCycle     = 0;
uint32_t lastColorCycleTickMs = 0;

// Coupling fase
bool  neighborPhaseAvailable=false;
float neighborMeanPhase=0.0f;

// Bloom
uint32_t bloomEndMs         = 0;
bool     bloomActive        = false;

uint8_t  lastPeerCount      = 0;

// Group hue residuo
uint8_t  groupHueLocal      = 170;
uint8_t  groupHueTarget     = 170;
uint32_t lastSoloHueDriftMs = 0;

BLEAdvertising* gAdv        = nullptr;

float smoothedPeak          = MIN_FRACTION;

// Leader
uint8_t currentLeaderID     = 0;
bool    isLeader            = true;

// Palette base (7 colori)
static const uint8_t basePalette[NUM_LEDS] = {
  210, // viola
  28,  // arancione
  85,  // verde
  52,  // giallo
  0,   // rosso
  245, // rosa
  230  // magenta
};

// Cross-fade buffers
uint8_t prevCycleHue[NUM_LEDS];
uint8_t targetCycleHue[NUM_LEDS];
uint8_t currentHueDisplay[NUM_LEDS];
uint32_t lastCycleChangeMs = 0;
bool fadeActive = false;

// PRNG deterministico (XorShift32)
static uint32_t prngState=0;
inline void prngSeed(uint32_t seed){
  prngState = seed ^ 0x9E3779B9u;
  for(int i=0;i<3;i++){
    prngState ^= prngState << 13;
    prngState ^= prngState >> 17;
    prngState ^= prngState << 5;
  }
}
inline uint32_t prngNext(){
  uint32_t x=prngState;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  prngState = x;
  return x;
}

// -------------------------------- Utility --------------------------------
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
    if(peers[i].valid && (now - peers[i].lastSeen) > BEACON_TIMEOUT_MS){
      peers[i].valid=false;
      peers[i].phaseValid=false;
      peers[i].cycleValid=false;
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
  for(int i=0;i<MAX_PEERS;i++)
    if(peers[i].valid) sum+=rssiToWeight(peers[i].lastRSSI);
  float n = sum / (float)PEERS_FOR_FULL_SCALE;
  return (n>1.0f)?1.0f:n;
}

uint8_t countValidPeers(){
  uint8_t c=0;
  for(int i=0;i<MAX_PEERS;i++) if(peers[i].valid) c++;
  return c;
}

static inline float wrap01(float x){
  if(x>=1.0f) return x-(int)x;
  if(x<0.0f)  return x-floorf(x);
  return x;
}

static inline float phaseDiff(float target,float local){
  float d=target-local;
  while(d>0.5f)d-=1.0f;
  while(d<-0.5f)d+=1.0f;
  return d;
}

static inline int8_t hueDiffSigned(uint8_t t,uint8_t c){
  int16_t d=(int16_t)t - (int16_t)c;
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

uint8_t hueLerpCircular(uint8_t a,uint8_t b,float t){
  if(t<=0) return a;
  if(t>=1) return b;
  int16_t da=(int16_t)b - (int16_t)a;
  if(da>127) da-=256;
  if(da<-128) da+=256;
  int16_t v=(int16_t)a + (int16_t)lroundf(da*t);
  if(v<0)v+=256;
  if(v>255)v-=256;
  return (uint8_t)v;
}

// -------------------------------- Palette deterministica --------------------------------
void computeTargetPalette(){
  uint8_t idx[NUM_LEDS];
  for(int i=0;i<NUM_LEDS;i++) idx[i]=i;
  prngSeed(warmColorCycle * 2654435761u);
  for(int i=NUM_LEDS-1;i>0;i--){
    uint32_t r = prngNext() % (i+1);
    uint8_t tmp=idx[i];
    idx[i]=idx[r];
    idx[r]=tmp;
  }
  for(int i=0;i<NUM_LEDS;i++)
    targetCycleHue[i] = basePalette[idx[i]];
}

void startNewCycleFade(uint32_t nowMs){
  for(int i=0;i<NUM_LEDS;i++)
    prevCycleHue[i]=currentHueDisplay[i];
  computeTargetPalette();
  lastCycleChangeMs = nowMs;
  fadeActive = true;
#ifdef COLOR_DEBUG
  Serial.print("FADE start cycle="); Serial.println(warmColorCycle);
#endif
}

void advanceColorCycle(uint32_t nowMs){
  warmColorCycle++;
  startNewCycleFade(nowMs);
  Serial.print("COLOR_CYCLE++ cycle="); Serial.println(warmColorCycle);
}

void adoptColorCycle(uint32_t newCycle, uint32_t nowMs){
  if(newCycle == warmColorCycle) return;
  warmColorCycle = newCycle;
  for(int i=0;i<NUM_LEDS;i++)
    prevCycleHue[i]=currentHueDisplay[i];
  computeTargetPalette();
  lastCycleChangeMs = nowMs;
  fadeActive = true;
#ifdef COLOR_DEBUG
  Serial.print("ADOPT cycle="); Serial.println(warmColorCycle);
#endif
}

void updateFade(uint32_t nowMs){
  if(!fadeActive){
    for(int i=0;i<NUM_LEDS;i++)
      currentHueDisplay[i]=targetCycleHue[i];
    return;
  }
  uint32_t elapsed = nowMs - lastCycleChangeMs;
  float t = (CROSS_FADE_MS==0)?1.0f:(float)elapsed / (float)CROSS_FADE_MS;
  if(t>=1.0f){
    fadeActive=false;
    for(int i=0;i<NUM_LEDS;i++)
      currentHueDisplay[i]=targetCycleHue[i];
    return;
  }
  for(int i=0;i<NUM_LEDS;i++)
    currentHueDisplay[i] = hueLerpCircular(prevCycleHue[i], targetCycleHue[i], t);
}

// -------------------------------- LED Init --------------------------------
void initLeds(){
  strip.begin();
  strip.clear();
  strip.setBrightness(255);
  strip.show();
  uint32_t now=millis();
  for(int k=0;k<NUM_LEDS;k++){
    int i=SEQ[k];
    ledStates[i].phaseOffset = (float)k / (float)NUM_LEDS;
    coldHueCurrent[i] = rand8range(COLD_MIN_HUE, COLD_MAX_HUE);
    coldHueNextChangeMs[i] = now + ZERO_PEER_BASE_INTERVAL_MS +
                             (esp_random() % (ZERO_PEER_COLOR_JITTER_MS+1));
  }
  computeTargetPalette();
  for(int i=0;i<NUM_LEDS;i++){
    currentHueDisplay[i]=targetCycleHue[i];
    prevCycleHue[i]=targetCycleHue[i];
  }
  fadeActive=false;
  lastCycleChangeMs=now;
  lastColorCycleTickMs=now;
}

// -------------------------------- Advertising --------------------------------
void setupAdvertising(){
  gAdv = BLEDevice::getAdvertising();
  uint8_t payload[12];
  payload[0]=(uint8_t)(COMPANY_ID & 0xFF);
  payload[1]=(uint8_t)(COMPANY_ID >> 8);
  payload[2]='B'; payload[3]='D'; payload[4]='G';
  payload[5]=PROTO_VERSION;
  payload[6]=myID;
  payload[7]=(uint8_t)(wrap01(fireflyPhase)*255.0f);
  payload[8]=groupHueLocal;
  payload[9]=(uint8_t)(warmColorCycle & 0xFF);
  payload[10]=(uint8_t)((warmColorCycle>>8)&0xFF);
  payload[11]=(isLeader?0x01:0x00);
  String manuf; manuf.reserve(12);
  for(int i=0;i<12;i++) manuf+=(char)payload[i];
  BLEAdvertisementData adv; adv.setManufacturerData(manuf);
  gAdv->setAdvertisementData(adv);
  gAdv->start();
}

void updateAdvertisingData(){
  if(!gAdv) return;
  uint32_t now=millis();
  if(now - lastAdvUpdateMs < ADV_UPDATE_INTERVAL_MS) return;
  lastAdvUpdateMs=now;
  uint8_t payload[12];
  payload[0]=(uint8_t)(COMPANY_ID & 0xFF);
  payload[1]=(uint8_t)(COMPANY_ID >> 8);
  payload[2]='B'; payload[3]='D'; payload[4]='G';
  payload[5]=PROTO_VERSION;
  payload[6]=myID;
  payload[7]=(uint8_t)(wrap01(fireflyPhase)*255.0f);
  payload[8]=groupHueLocal;
  payload[9]=(uint8_t)(warmColorCycle & 0xFF);
  payload[10]=(uint8_t)((warmColorCycle>>8)&0xFF);
  payload[11]=(isLeader?0x01:0x00);
  String manuf; manuf.reserve(12);
  for(int i=0;i<12;i++) manuf+=(char)payload[i];
  BLEAdvertisementData adv; adv.setManufacturerData(manuf);
  gAdv->setAdvertisementData(adv);
}

// -------------------------------- Scan --------------------------------
void adoptPeerCycleIfNeeded(uint16_t peerCycle, uint32_t nowMs){
  if(peerCycle > warmColorCycle){
    adoptColorCycle(peerCycle, nowMs);
  }
}

void doScanCycle(){
  BLEScan* scan = BLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(160);
  uint32_t durS = SCAN_DURATION_MS / 1000;
  if(durS==0) durS=1;
  BLEScanResults* res=scan->start(durS,false);
  if(!res) return;
  int found=res->getCount();

  float sumSinPhase=0,sumCosPhase=0; int phaseCount=0;
  uint16_t maxPeerCycle=warmColorCycle;
  uint8_t minPeerID=myID;

  for(int i=0;i<found;i++){
    BLEAdvertisedDevice dev=res->getDevice(i);
    if(!dev.haveManufacturerData()) continue;
    String m=dev.getManufacturerData();
    if(m.length()<9) continue;
    uint16_t cid=((uint8_t)m.charAt(0)) | (((uint8_t)m.charAt(1))<<8);
    if(cid!=COMPANY_ID) continue;
    if(m.charAt(2)!='B'||m.charAt(3)!='D'||m.charAt(4)!='G') continue;

    uint8_t peerProto=(uint8_t)m.charAt(5);
    uint8_t peerID=(uint8_t)m.charAt(6);
    if(peerID < minPeerID) minPeerID=peerID;

    int slot=findPeerSlot(dev.getAddress());
    if(slot<0) continue;
    peers[slot].addr=dev.getAddress();
    peers[slot].lastRSSI=dev.getRSSI();
    peers[slot].lastSeen=millis();
    peers[slot].valid=true;
    peers[slot].peerID=peerID;

    uint8_t phaseByte=(uint8_t)m.charAt(7);
    peers[slot].phase=(float)phaseByte/255.0f;
    peers[slot].phaseValid=true;

    if(peerProto>=6 && m.length()>=12){
      uint8_t cL=(uint8_t)m.charAt(9);
      uint8_t cH=(uint8_t)m.charAt(10);
      uint16_t peerCycle=(uint16_t)cL | ((uint16_t)cH<<8);
      peers[slot].colorCycle=peerCycle;
      peers[slot].cycleValid=true;
      if(peerCycle>maxPeerCycle) maxPeerCycle=peerCycle;
    } else {
      peers[slot].cycleValid=false;
    }

    float angleP=peers[slot].phase*TWO_PI;
    sumSinPhase+=sinf(angleP);
    sumCosPhase+=cosf(angleP);
    phaseCount++;
  }

  if(phaseCount>0){
    float meanAngleP=atan2f(sumSinPhase/phaseCount, sumCosPhase/phaseCount);
    if(meanAngleP<0) meanAngleP+=TWO_PI;
    neighborMeanPhase=meanAngleP / TWO_PI;
    neighborPhaseAvailable=true;
  } else neighborPhaseAvailable=false;

  adoptPeerCycleIfNeeded(maxPeerCycle, millis());

  currentLeaderID=minPeerID;
  isLeader=(myID == currentLeaderID);

  uint8_t currentPeerCount=countValidPeers();
  if(currentPeerCount > lastPeerCount){
    if(presenceFactor < 0.02f) presenceFactor=0.15f;
    bloomEndMs = millis() + BLOOM_DURATION_MS;
    bloomActive=true;
  }
  lastPeerCount=currentPeerCount;

  scan->clearResults();
}

// -------------------------------- Group hue (residuo) --------------------------------
void updateGroupHue(uint8_t peerCount){
  if(peerCount==0){
    if(millis() - lastSoloHueDriftMs > 1800){
      lastSoloHueDriftMs=millis();
      int8_t step=(int8_t)rand8range(0,2)-1;
      groupHueLocal = clampCold(hueAdd(groupHueLocal, step));
    }
    groupHueTarget=groupHueLocal;
  } else {
    int8_t d=hueDiffSigned(groupHueTarget, groupHueLocal);
    float step=d * GROUP_HUE_COUPLING;
    if(step>6) step=6;
    if(step<-6) step=-6;
    groupHueLocal = hueAdd(groupHueLocal,(int16_t)lroundf(step));
  }
}

// -------------------------------- Solo cold --------------------------------
void updateSoloColdColors(uint32_t now){
  for(int i=0;i<NUM_LEDS;i++){
    if(now >= coldHueNextChangeMs[i]){
      coldHueCurrent[i]=rand8range(COLD_MIN_HUE, COLD_MAX_HUE);
      coldHueNextChangeMs[i]= now + ZERO_PEER_BASE_INTERVAL_MS +
                              (esp_random() % (ZERO_PEER_COLOR_JITTER_MS+1));
    }
  }
}

// -------------------------------- Timer ciclo colori --------------------------------
void leaderColorCycleTimer(uint32_t nowMs, bool peerMode){
  if(!peerMode) return;
  if(!isLeader) return;
  if(nowMs - lastColorCycleTickMs >= COLOR_CHANGE_INTERVAL_MS){
    lastColorCycleTickMs += COLOR_CHANGE_INTERVAL_MS;
    if(nowMs - lastColorCycleTickMs > COLOR_CHANGE_INTERVAL_MS)
      lastColorCycleTickMs = nowMs;
    advanceColorCycle(nowMs);
  }
  if(nowMs - lastCycleChangeMs > (2 * COLOR_CHANGE_INTERVAL_MS)){
    lastColorCycleTickMs = nowMs;
    advanceColorCycle(nowMs);
    Serial.println("COLOR_CYCLE FAILSAFE triggered");
  }
}

// -------------------------------- Logging RSSI Peer --------------------------------
#if PEER_RSSI_DEBUG
void printPeerRSSI(){
  uint8_t count = countValidPeers();
  if(count == 0){
    Serial.println("[PEERS] nessun peer");
    return;
  }
  float sumRSSI = 0;
  int8_t maxRSSI = -127;
  Serial.print("[PEERS] count=");
  Serial.print(count);
  Serial.print(" (idx: rssi_dBm / weight / MAC)\n");
  for(int i=0;i<MAX_PEERS;i++){
    if(!peers[i].valid) continue;
    int8_t rssi = peers[i].lastRSSI;
    float w = rssiToWeight(rssi);
    if(rssi > maxRSSI) maxRSSI = rssi;
    sumRSSI += rssi;
    Serial.print("  #");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(rssi);
    Serial.print(" dBm / ");
    Serial.print(w,2);
    Serial.print(" / ");
    Serial.println(peers[i].addr.toString().c_str());
  }
  Serial.print("  avgRSSI=");
  Serial.print(sumRSSI / (float)count,1);
  Serial.print(" dBm  maxRSSI=");
  Serial.print(maxRSSI);
  Serial.println(" dBm");
}
#endif

// -------------------------------- LED update --------------------------------
void updateLeds(uint32_t nowMs, float dt){
  float raw=computePresenceRaw();
  presenceFactor += PRESENCE_ALPHA * (raw - presenceFactor);
  if(presenceFactor<0) presenceFactor=0;
  if(presenceFactor>1) presenceFactor=1;

  uint8_t peerCount = countValidPeers();
  bool soloMode = (peerCount==0);

  float presenceDelta = presenceFactor - previousPresence;
  if(presenceDelta > 0.12f && !soloMode){
    bloomEndMs = millis() + BLOOM_DURATION_MS;
    bloomActive = true;
  }
  previousPresence = presenceFactor;

  fireflyPhase += fireflyFreq * dt;
  fireflyPhase = wrap01(fireflyPhase);
  if(!soloMode && neighborPhaseAvailable){
    float eff=0.14f * (0.4f + 0.6f*presenceFactor);
    float diff=phaseDiff(neighborMeanPhase, fireflyPhase);
    fireflyPhase = wrap01(fireflyPhase + diff*eff);
  }

  leaderColorCycleTimer(nowMs, !soloMode);

  updateGroupHue(peerCount);
  if(soloMode) updateSoloColdColors(nowMs);

  if(!soloMode) updateFade(nowMs);
  else {
    for(int i=0;i<NUM_LEDS;i++) currentHueDisplay[i]=coldHueCurrent[i];
    fadeActive=false;
  }

  float targetPeak = soloMode ? ZERO_PEER_PEAK : (float)peerCount / 10.0f;
  if(!soloMode){
    if(targetPeak < 0.12f) targetPeak=0.12f;
    if(targetPeak > 1.0f)  targetPeak=1.0f;
  }
  float alpha=PEAK_SMOOTH_ALPHA;
  if(targetPeak < smoothedPeak) alpha *= PEAK_SMOOTH_ALPHA_FALL_MULT;
  smoothedPeak += alpha*(targetPeak - smoothedPeak);

  if(soloMode){
    if(smoothedPeak < ZERO_PEER_MIN_FRACTION) smoothedPeak=ZERO_PEER_MIN_FRACTION;
  } else {
    if(smoothedPeak < MIN_FRACTION) smoothedPeak=MIN_FRACTION;
  }

  for(int i=0;i<NUM_LEDS;i++){
    float phase_i=wrap01(fireflyPhase + ledStates[i].phaseOffset);
    float wave_i =(sinf(phase_i * TWO_PI) + 1.0f)*0.5f;

    float shimmer=1.0f;
    if(!soloMode){
      float sh = sinf( (fireflyPhase * TWO_PI * SHIMMER_FREQ_MULT) + i*SHIMMER_PHASE_SPREAD );
      shimmer = 1.0f + SHIMMER_DEPTH * sh;
      if(shimmer<0) shimmer=0;
    }

    uint8_t finalHue = currentHueDisplay[i];
    uint8_t S = 255;

    float valFrac;
    if(soloMode){
      valFrac = wave_i * smoothedPeak;
    } else {
      float peak = smoothedPeak;
      if(peak < MIN_FRACTION) peak=MIN_FRACTION;
      valFrac = MIN_FRACTION + wave_i * (peak - MIN_FRACTION);
      valFrac *= shimmer;
    }
    if(valFrac<0) valFrac=0;
    if(valFrac>1) valFrac=1;

    uint8_t V=(uint8_t)lroundf(valFrac * 255.0f);
    uint16_t hue16=(uint16_t)finalHue * 257;
    uint32_t c=strip.ColorHSV(hue16, S, V);
    strip.setPixelColor(i, c);
  }

  strip.show();

  if(bloomActive && millis()>bloomEndMs) bloomActive=false;

  if(nowMs - lastStatusLogMs > STATUS_LOG_INTERVAL_MS){
    lastStatusLogMs=nowMs;
    Serial.print("[STATUS] peers=");
    Serial.print(peerCount);
    Serial.print(" leaderID=");
    Serial.print(currentLeaderID);
    Serial.print(" isLeader=");
    Serial.print(isLeader);
    Serial.print(" cycle=");
    Serial.print((unsigned long)warmColorCycle);
    Serial.print(" fade=");
    Serial.print(fadeActive);
    Serial.print(" phase=");
    Serial.print(fireflyPhase,3);
    Serial.print(" presence=");
    Serial.print(presenceFactor,3);
    Serial.println();
#if PEER_RSSI_DEBUG
    printPeerRSSI();
#endif
#ifdef COLOR_DEBUG
    Serial.print("HUES: ");
    for(int i=0;i<NUM_LEDS;i++){ Serial.print(currentHueDisplay[i]); Serial.print(' ');}
    Serial.println();
#endif
  }
}

// -------------------------------- BLE Scan Config --------------------------------
void setupBLE_scanConfig(){
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(160);
}

// ID locale (ultimo byte MAC)
uint8_t deriveMyIDFromBLEAddr(){
  String addr=BLEDevice::getAddress().toString();
  int pos=addr.lastIndexOf(':');
  String last=(pos>=0)?addr.substring(pos+1):addr;
  last.trim();
  char buf[3]={0,0,0};
  last.toCharArray(buf,3);
  uint8_t v=(uint8_t)strtoul(buf,NULL,16);
  if(v==0) v=(uint8_t)(esp_random() & 0xFF);
  return v;
}

// -------------------------------- Setup / Loop --------------------------------
void setup(){
  Serial.begin(115200);
  delay(120);
  BLEDevice::init("Badge");
  myID = deriveMyIDFromBLEAddr();
  currentLeaderID=myID;
  isLeader=true;

  Serial.print("PROTO_VERSION="); Serial.println(PROTO_VERSION);
  Serial.print("myID="); Serial.println(myID);

  initLeds();
  setupAdvertising();
  setupBLE_scanConfig();
  lastScanStartMs=0;
  lastColorCycleTickMs = millis();
}

void loop(){
  uint32_t now=millis();
  if(now - lastScanStartMs >= SCAN_PERIOD_MS){
    lastScanStartMs = now;
    doScanCycle();
  }
  if(now - lastFrameMs >= FRAME_INTERVAL_MS){
    float dt=(lastFrameMs==0)? (FRAME_INTERVAL_MS/1000.0f):(now - lastFrameMs)/1000.0f;
    lastFrameMs=now;
    updateLeds(now, dt);
    updateAdvertisingData();
  }
  delay(1);
}
