#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller (CW-only test mode)
  ----------------------------------------------------------
  Denne version ignorerer CH4 helt og tvinger altid motoren
  til at køre i CW-retning. Perfekt til test af throttle,
  ramping og failsafe uden ustabil retningsstyring.
  ==========================================================
*/

// ------------ KONSTANTER OG PINOPSÆTNING ------------
const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal (INT0)
const uint8_t PIN_FR          = 7;  // FR-pin til retning (tvinges HIGH)
const uint8_t PIN_EN          = 8;  // Enable til controller
const uint8_t PIN_PWM         = 9;  // PWM-output til speed (SV)

const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// ------------ DRIFTSTILSTAND ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// --- RC-måling (opdateres af interrupt) ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

// ==================================================
// HJÆLPEFUNKTIONER
// ==================================================
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void forceCW() {
  digitalWrite(PIN_FR, HIGH); // tving CW
}

static inline uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

// ==================================================
// INTERRUPTS
// ==================================================
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

// ==================================================
// SETUP
// ==================================================
void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);

  Serial.begin(115200);
  delay(200);

  // --- Aktiver throttle interrupt ---
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00); // trigger på CHANGE

  // --- Init motor ---
  setEnable(true);
  forceCW();             // permanent CW
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (CW-only mode). D2=CH3 throttle, D9 PWM."));
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle-behandling ---
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

  // --- PWM-opdatering (soft ramp) ---
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

  // --- Seriel debug ---
  static unsigned long tLastPrint = 0;
  if (now - tLastPrint >= 200) {
    tLastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(lastWidthTh);
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.println(F("% dir=FORCED_CW"));
  }
}