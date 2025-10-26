#include <Arduino.h>
#undef min
#undef max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

/*
  ==========================================================
  RC → Arduino → BLDC controller
  ----------------------------------------------------------
  CW-only drift med anti-stall start-kick.
  Retningsstyring via CH4 kan aktiveres senere med USE_DIR=true.
  ==========================================================

  Forbindelser:
  -------------
  CH3 → D2 (INT0)  : Throttle (PWM)
  CH4 → D4 (PCINT20): Direction (kun aktiv hvis USE_DIR=true)
  D7  → FR          : Retning (HIGH = CW)
  D8  → EN          : Enable til controller
  D9  → SV          : PWM hastighedssignal
*/

#define USE_DIR false  // ⚙️  true = CH4-retning, false = CW-only

// ------------ PINOPSÆTNING ------------
const uint8_t PIN_RC_THROTTLE = 2;
const uint8_t PIN_RC_DIR      = 4;
const uint8_t PIN_FR          = 7;
const uint8_t PIN_EN          = 8;
const uint8_t PIN_PWM         = 9;

// ------------ KONSTANTER ------------
const int RC_MID_US = 1500;
const int DIR_HYST_US = 120;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// Anti-stall parametre
const uint8_t  MIN_RUN_PCT    = 18;    // min. duty for stabil rotation
const uint8_t  START_KICK_PCT = 100;   // “spark” ved start
const uint16_t START_KICK_MS  = 150;   // længde af spark
const uint16_t CRAWL_HOLD_MS  = 400;   // hold crawl lidt før 0

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = true;
uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep  = 0;
unsigned long tLastPulse = 0;

// Anti-stall tidsmarkører
static bool wasRunning = false;
static unsigned long tKickEnd = 0;
static unsigned long tZeroSince = 0;

// --- RC-måling ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

#if USE_DIR
volatile uint32_t pulseStartDir = 0;
volatile uint32_t pulseWidthDir = 1500;
volatile bool newPulseDir = false;
#endif

// ==================================================
// HJÆLPEFUNKTIONER
// ==================================================
static inline void setEnable(bool on) { digitalWrite(PIN_EN, on ? HIGH : LOW); }
static inline void setDirectionCW(bool cw) { dirCW = cw; digitalWrite(PIN_FR, cw ? HIGH : LOW); }
static inline void forceCW() { digitalWrite(PIN_FR, HIGH); }

static inline uint8_t pctToPwm(uint8_t pct){ return (pct>100?255:(pct*255UL)/100UL); }
static inline void applyPWM(uint8_t pct){ currentPct = pct; analogWrite(PIN_PWM, pctToPwm(pct)); }

// ==================================================
// INTERRUPTS
// ==================================================
ISR(INT0_vect) {  // Throttle
  bool level = digitalRead(PIN_RC_THROTTLE);
  if (level) pulseStartTh = micros();
  else {
    uint32_t width = micros() - pulseStartTh;
    if (width >= 800 && width <= 2200) { pulseWidthTh = width; newPulseTh = true; }
  }
}

#if USE_DIR
ISR(PCINT2_vect) { // Direction
  bool level = digitalRead(PIN_RC_DIR);
  if (level) pulseStartDir = micros();
  else {
    uint32_t width = micros() - pulseStartDir;
    if (width >= 800 && width <= 2200) { pulseWidthDir = width; newPulseDir = true; }
  }
}
#endif

// ==================================================
// SETUP
// ==================================================
void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
#if USE_DIR
  pinMode(PIN_RC_DIR, INPUT_PULLUP);
#endif

  Serial.begin(115200);
  delay(200);

  // Aktiver interrupts
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00);
#if USE_DIR
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
#endif

  setEnable(true);
  forceCW();
  applyPWM(0);
  runState = STOPPED;

#if USE_DIR
  Serial.println(F("RC→BLDC (direction mode enabled)."));
#else
  Serial.println(F("RC→BLDC (CW-only + anti-stall)."));
#endif
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle ---
  static uint32_t lastWidthTh = 1500;
  noInterrupts();
  if (newPulseTh) { lastWidthTh = pulseWidthTh; newPulseTh = false; tLastPulse = now; }
  interrupts();

  int pct = map(lastWidthTh, 1000, 2000, 0, 100);
  pct = constrain(pct, 0, 100);
  targetPct = pct;

  // --- Failsafe ---
  if (now - tLastPulse > FAILSAFE_MS) {
    applyPWM(0);
    setEnable(false);
    runState = STOPPED;
    return;
  }

#if USE_DIR
  // --- Direction (kun aktiv hvis USE_DIR = true) ---
  static uint32_t lastWidthDir = 1500;
  noInterrupts();
  if (newPulseDir) { lastWidthDir = pulseWidthDir; newPulseDir = false; }
  interrupts();

  bool wantCW;
  if (lastWidthDir > (RC_MID_US + DIR_HYST_US))      wantCW = true;
  else if (lastWidthDir < (RC_MID_US - DIR_HYST_US)) wantCW = false;
  else                                                wantCW = dirCW;

  if (wantCW != dirCW && currentPct < 10) setDirectionCW(wantCW);
#else
  // --- Tving CW ---
  forceCW();
#endif

  // --- Anti-stall logik ---
  setEnable(true);

  // 1. Kick ved 0 → >0
  if (currentPct == 0 && targetPct > 0 && now > tKickEnd) {
    applyPWM(START_KICK_PCT);
    tKickEnd = now + START_KICK_MS;
    wasRunning = true;
    return;
  }

  // 2. Efter kick, hold minimum rotation
  if (now <= tKickEnd) {
    // stadig i kick-fase
  } else if (wasRunning && currentPct >= START_KICK_PCT) {
    uint8_t hold = (targetPct < MIN_RUN_PCT) ? MIN_RUN_PCT : targetPct;
    applyPWM(hold);
  }

  // 3. Normal ramp og crawl-hold
  if (now - tLastStep >= STEP_MS && now > tKickEnd) {
    tLastStep = now;

    if (targetPct >= PUNCH_THRESH_PCT) {
      applyPWM(100);
    } else if (currentPct < targetPct) {
      applyPWM(min<uint8_t>(currentPct + 2, targetPct));
    } else if (currentPct > targetPct) {
      if (targetPct == 0) {
        if (currentPct > MIN_RUN_PCT) {
          applyPWM(max<int>(currentPct - 2, MIN_RUN_PCT));
          tZeroSince = now;
        } else {
          if (now - tZeroSince > CRAWL_HOLD_MS) {
            applyPWM(0);
            wasRunning = false;
          }
        }
      } else {
        applyPWM(max<int>(currentPct - 2, targetPct));
      }
    }
  }

  // --- Telemetri ---
  static unsigned long tLastPrint = 0;
  if (now - tLastPrint >= 200) {
    tLastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(lastWidthTh);
#if USE_DIR
    Serial.print(F("  DIR(us)=")); Serial.print(pulseWidthDir);
#endif
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
#if USE_DIR
    Serial.print(F("% dir=")); Serial.println(dirCW ? F("CW") : F("CCW"));
#else
    Serial.println(F("% dir=FORCED_CW"));
#endif
  }
}