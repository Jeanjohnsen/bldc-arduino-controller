#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller (CW default version)
  ----------------------------------------------------------
  - CH3 → D2 (INT0)  : Throttle (PWM fra joystick)
  - CH4 → D4 (PCINT20): Direction (PWM fra switch)
  - D7  → FR          : Retning (CW/CCW) til controller
  - D8  → EN          : Enable
  - D9  → SV          : PWM speed (output)
  ==========================================================
*/

// ------------ KONSTANTER OG PINOPSÆTNING ------------
const uint8_t PIN_RC_THROTTLE = 2;
const uint8_t PIN_RC_DIR      = 4;
const uint8_t PIN_FR          = 7;
const uint8_t PIN_EN          = 8;
const uint8_t PIN_PWM         = 9;

const int RC_MIN_US_DEFAULT = 1000;
const int RC_MAX_US_DEFAULT = 2000;
const int RC_MID_US = 1500;
const int RC_TIMEOUT_US = 25000;
const int DIR_HYST_US = 160;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  MIN_RUN_PCT = 0;
const uint8_t  PUNCH_THRESH_PCT = 95;

// ------------ DRIFTSTILSTAND ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = true;  // 🧭 Default = CW
uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// --- RC-måling (opdateres af interrupts) ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

volatile uint32_t pulseStartDir = 0;
volatile uint32_t pulseWidthDir = 1500;
volatile bool newPulseDir = false;

// ==================================================
// HJÆLPEFUNKTIONER
// ==================================================
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  digitalWrite(PIN_FR, cw ? HIGH : LOW);  // HIGH = CW, LOW = CCW
}

static inline uint8_t pctToPwm(uint8_t pct) {
  return (uint8_t)((constrain(pct, 0, 100) * 255UL) / 100UL);
}

static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

static void softTo(uint8_t toPct) {
  uint8_t from = currentPct;
  if (from == toPct) return;
  unsigned long start = millis();
  while (true) {
    float t = (float)(millis() - start) / (float)RAMP_MS;
    if (t >= 1.0f) { applyPWM(toPct); break; }
    uint8_t step = from + (uint8_t)((toPct - from) * t);
    applyPWM(step);
    delay(STEP_MS);
  }
}

static void safeChangeDirection(bool newCW) {
  if (newCW == dirCW) return;
  softTo(0);
  delay(80);
  setDirectionCW(newCW);
  delay(80);
  if (targetPct > 0) {
    setEnable(true);
    softTo(targetPct);
    runState = RUNNING;
  }
}

// ==================================================
// INTERRUPTS
// ==================================================
ISR(INT0_vect) {  // Throttle via D2
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

ISR(PCINT2_vect) {  // Direction via D4
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

// ==================================================
// SETUP
// ==================================================
void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_DIR, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);

  // Aktiver interrupts
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  // --- Init motorstatus ---
  setEnable(true);
  setDirectionCW(true);     // 🧭 Force CW på startup
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (CW default). D2=CH3 throttle, D4=CH4 dir, D9 PWM."));
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle ---
  static uint32_t lastWidthTh = 1500;
  noInterrupts();
  if (newPulseTh) {
    lastWidthTh = pulseWidthTh;
    newPulseTh = false;
    tLastPulse = now;
  }
  interrupts();

  int pct = map(lastWidthTh, 1000, 2000, 0, 100);
  targetPct = constrain(pct, 0, 100);

  // --- Failsafe ---
  if (now - tLastPulse > FAILSAFE_MS) {
    softTo(0);
    setEnable(false);
    runState = STOPPED;
    return;
  }

  // --- Retning (stabil CH4) ---
  static uint32_t lastWidthDir = 1500;
  static uint32_t stableWidthDir = 1500;
  static uint8_t stableCount = 0;
  static bool lastDirCW = true; // start CW
  static unsigned long startupIgnoreEnd = millis() + 500; // Ignorer signal 0.5 sek.

  noInterrupts();
  if (newPulseDir) {
    lastWidthDir = pulseWidthDir;
    newPulseDir = false;
  }
  interrupts();

  // Ignorer støj i opstart
  if (millis() < startupIgnoreEnd) return;

  // Glidende gennemsnit
  stableWidthDir = (stableWidthDir * 3 + lastWidthDir) / 4;

  bool wantCW;
  if (stableWidthDir > (RC_MID_US + DIR_HYST_US))      wantCW = true;
  else if (stableWidthDir < (RC_MID_US - DIR_HYST_US)) wantCW = false;
  else                                                 wantCW = lastDirCW;

  if (wantCW == lastDirCW) stableCount++;
  else stableCount = 0;

  if (stableCount > 5 && wantCW != dirCW && currentPct < 10) {
    safeChangeDirection(wantCW);
    stableCount = 0;
  }

  lastDirCW = wantCW;

  // --- PWM styring ---
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
      if (next == 0) { setEnable(false); runState = STOPPED; }
    }
  }

  // --- Telemetri ---
  static unsigned long tLastPrint = 0;
  if (now - tLastPrint >= 200) {
    tLastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(lastWidthTh);
    Serial.print(F("  DIR(us)=")); Serial.print(stableWidthDir);
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.print(F("% dir=")); Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}