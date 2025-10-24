#include <Arduino.h>

/*
  RC → Arduino → BLDC controller (stabil CW-lock version)
  ----------------------------------------------------------
  - CH3 → D2 (INT0): Throttle
  - CH4 → D4 (PCINT20): Direction
  - D7  → FR: Retning (til BLDC)
  - D8  → EN: Enable
  - D9  → SV: PWM speed
*/

const uint8_t PIN_RC_THROTTLE = 2;
const uint8_t PIN_RC_DIR      = 4;
const uint8_t PIN_FR          = 7;
const uint8_t PIN_EN          = 8;
const uint8_t PIN_PWM         = 9;

const int RC_MID_US = 1500;
const int DIR_HYST_US = 200;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// --- State ---
bool dirCW = true;  // default CW
uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// --- RC measurement ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

volatile uint32_t pulseStartDir = 0;
volatile uint32_t pulseWidthDir = 1500;
volatile bool newPulseDir = false;

static inline void setEnable(bool on) { digitalWrite(PIN_EN, on ? HIGH : LOW); }
static inline void setDirectionCW(bool cw) { dirCW = cw; digitalWrite(PIN_FR, cw ? HIGH : LOW); }
static inline uint8_t pctToPwm(uint8_t pct) { return (uint8_t)((constrain(pct,0,100)*255UL)/100UL); }
static inline void applyPWM(uint8_t pct){ currentPct=pct; analogWrite(PIN_PWM,pctToPwm(pct)); }

static void softTo(uint8_t toPct){
  uint8_t from=currentPct;
  if(from==toPct)return;
  unsigned long start=millis();
  while(true){
    float t=(float)(millis()-start)/(float)RAMP_MS;
    if(t>=1.0f){applyPWM(toPct);break;}
    uint8_t step=from+(uint8_t)((toPct-from)*t);
    applyPWM(step);
    delay(STEP_MS);
  }
}

static void safeChangeDirection(bool newCW){
  if(newCW==dirCW)return;
  softTo(0);
  delay(80);
  setDirectionCW(newCW);
  delay(80);
  if(targetPct>0){ setEnable(true); softTo(targetPct);}
}

// --- Interrupts ---
ISR(INT0_vect){ // throttle
  bool lvl=digitalRead(PIN_RC_THROTTLE);
  if(lvl)pulseStartTh=micros();
  else{
    uint32_t w=micros()-pulseStartTh;
    if(w>=800&&w<=2200){pulseWidthTh=w;newPulseTh=true;}
  }
}

ISR(PCINT2_vect){ // direction
  bool lvl=digitalRead(PIN_RC_DIR);
  if(lvl)pulseStartDir=micros();
  else{
    uint32_t w=micros()-pulseStartDir;
    if(w>=800&&w<=2200){pulseWidthDir=w;newPulseDir=true;}
  }
}

// --- Setup ---
void setup(){
  pinMode(PIN_EN,OUTPUT); pinMode(PIN_FR,OUTPUT);
  pinMode(PIN_PWM,OUTPUT);
  pinMode(PIN_RC_THROTTLE,INPUT);
  pinMode(PIN_RC_DIR,INPUT_PULLUP);

  Serial.begin(115200); delay(200);

  EIMSK |= (1<<INT0);
  EICRA |= (1<<ISC00);
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT20);

  setEnable(true);
  setDirectionCW(true);
  applyPWM(0);
  Serial.println(F("RC→BLDC (forced CW start, stabilized)."));

  // Ignorér CH4 de første 2 sekunder
  delay(2000);
}

// --- Loop ---
void loop(){
  unsigned long now=millis();

  // --- Throttle smoothing ---
  static uint32_t lastWidthTh=1500;
  static uint32_t smoothTh=1500;
  noInterrupts();
  if(newPulseTh){ lastWidthTh=pulseWidthTh; newPulseTh=false; tLastPulse=now; }
  interrupts();

  smoothTh = (smoothTh*3 + lastWidthTh)/4;  // low-pass filter
  int pct = map(smoothTh,1000,2000,0,100);
  targetPct = constrain(pct,0,100);

  if(now - tLastPulse > FAILSAFE_MS){
    softTo(0); setEnable(false); return;
  }

  // --- Direction filtering ---
  static uint32_t lastWidthDir=1500;
  static uint32_t stableDir=1500;
  static uint8_t stableCount=0;
  static bool lastDirCW=true;

  noInterrupts();
  if(newPulseDir){ lastWidthDir=pulseWidthDir; newPulseDir=false; }
  interrupts();

  stableDir = (stableDir*3 + lastWidthDir)/4;

  bool wantCW;
  if(stableDir > (RC_MID_US + DIR_HYST_US)) wantCW=true;
  else if(stableDir < (RC_MID_US - DIR_HYST_US)) wantCW=false;
  else wantCW=lastDirCW;

  if(wantCW==lastDirCW) stableCount++; else stableCount=0;

  // Skift kun hvis 10 målinger i træk
  if(stableCount>10 && wantCW!=dirCW && currentPct<10){
    safeChangeDirection(wantCW);
    stableCount=0;
  }
  lastDirCW=wantCW;

  // --- PWM update ---
  if(now - tLastStep >= STEP_MS){
    tLastStep=now;
    if(targetPct>=PUNCH_THRESH_PCT){ applyPWM(100); setEnable(true);}
    else if(currentPct<targetPct){ applyPWM(currentPct+2); setEnable(true);}
    else if(currentPct>targetPct){
      uint8_t next=(currentPct>2)?currentPct-2:0;
      applyPWM(next);
      if(next==0)setEnable(false);
    }
  }

  // --- Telemetri ---
  static unsigned long tLastPrint=0;
  if(now-tLastPrint>=200){
    tLastPrint=now;
    Serial.print(F("THR(us)=")); Serial.print(smoothTh);
    Serial.print(F("  DIR(us)=")); Serial.print(stableDir);
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.print(F("% dir=")); Serial.println(dirCW?F("CW"):F("CCW"));
  }
}