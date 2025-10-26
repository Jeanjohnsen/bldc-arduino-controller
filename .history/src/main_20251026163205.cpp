#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller
  ----------------------------------------------------------
  Denne version kan nemt køre i to modes:

  ▶️ CW-only test mode  (USE_DIR = false)
     - Ignorerer CH4 fuldstændigt
     - FR-pin tvinges HIGH (CW)

  🔁 Direction mode (USE_DIR = true)
     - Læser CH4 for CW/CCW
     - Skifter retning sikkert når motoren er næsten stoppet
  ==========================================================
*/

// ==========================================================
// ⚙️ KONFIGURATION
// ==========================================================
#define USE_DIR false   // ← Sæt til true for at aktivere retningsstyring (CH4)

const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal (INT0)
const uint8_t PIN_RC_DIR      = 4;  // CH4 signal (PCINT20)
const uint8_t PIN_FR          = 7;  // FR-pin (CW/CCW)
const uint8_t PIN_EN          = 8;  // Enable til controller
const uint8_t PIN_PWM         = 9;  // PWM-signal (SV)

const int RC_MID_US = 1500;
const int DIR_HYST_US = 120;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// ==========================================================
// 🧭 STATUSVARIABLER
// ==========================================================
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = true; // Default CW
uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// RC pulse data
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

#if USE_DIR
volatile uint32_t pulseStartDir = 0;
volatile uint32_t pulseWidthDir = 1500;
volatile bool newPulseDir = false;
#endif

// ==========================================================
// 🧰 HJÆLPEFUNKTIONER
// ==========================================================
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  digitalWrite(PIN_FR, cw ? HIGH : LOW);
}

static inline void forceCW() {
  dirCW = true;
  digitalWrite(PIN_FR, HIGH);
}

static inline uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

// ==========================================================
// ⏱️ INTERRUPTS
// ==========================================================
ISR(INT0_vect) {
  bool level = digitalRead(PIN_RC_THROTTLE);
  if (level)
    pulseStartTh = micros();
  else {
    uint32_t width = micros() - pulseStartTh;
    if (width >= 800 && width <= 2200) {
      pulseWidthTh = width;
      newPulseTh = true;
    }
  }
}

#if USE_DIR
ISR(PCINT2_vect) {
  bool level = digitalRead(PIN_RC_DIR);
  if (level)
    pulseStartDir = micros();
  else {
    uint32_t width = micros() - pulseStartDir;
    if (width >= 800 && width <= 2200) {
      pulseWidthDir = width;
      newPulseDir = true;
    }
  }
}
#endif

// ==========================================================
// 🚀 SETUP
// ==========================================================
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

  // Throttle interrupt
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00);

#if USE_DIR
  // Direction interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
#endif

  // Init motor
  setEnable(true);
  forceCW(); // Always start CW
  applyPWM(0);
  runState = STOPPED;

#if USE_DIR
  Serial.println(F("RC→BLDC (direction mode active)."));
#else
  Serial.println(F("RC→BLDC (CW-only test mode)."));
#endif
}

// ==========================================================
// 🔁 LOOP
// ==========================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle read ---
  static uint32_t lastWidthTh = 1500;
  noInterrupts();
  if (newPulseTh) {
    lastWidthTh = pulseWidthTh;
    newPulseTh = false;
    tLastPulse = now;
  }
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
  // --- Direction read ---
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
  // --- Force CW ---
  forceCW();
#endif

  // --- PWM ramping ---
  if (now - tLastStep >= STEP_MS) {
    tLastStep = now;
    if (targetPct >= PUNCH_THRESH_PCT) {
      applyPWM(100);
      setEnable(true);
      runState = RUNNING;
    } else if (currentPct < targetPct) {
      applyPWM(currentPct + 2);
      setEnable(true);
      runState = RUNNING;
    } else if (currentPct > targetPct) {
      uint8_t next = (currentPct > 2) ? currentPct - 2 : 0;
      applyPWM(next);
      if (next == 0) {
        setEnable(false);
        runState = STOPPED;
      }
    }
  }

  // --- Debug ---
  static unsigned long tLastPrint = 0;
  if (now - tLastPrint >= 200) {
    tLastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(lastWidthTh);
#if USE_DIR
    Serial.print(F("  DIR(us)=")); Serial.print(lastWidthDir);
#endif
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.print(F("% dir="));
#if USE_DIR
    Serial.println(dirCW ? F("CW") : F("CCW"));
#else
    Serial.println(F("FORCED_CW"));
#endif
  }
}