#include <Arduino.h>

/*
  RC → Arduino → BLDC controller
  ---------------------------------------
  CH3 → D2 (INT0)  : Throttle (PWM signal)
  CH4 → D4 (PCINT20): Direction (HIGH/LOW)
  D7  → FR         : Direction out (til controller)
  D8  → EN         : Enable
  D9  → SV         : PWM speed
*/

// ------------ CONFIG ------------
const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal fra receiver
const uint8_t PIN_RC_DIR      = 4;  // CH4 signal fra receiver (PCINT20)
const uint8_t PIN_FR          = 7;  // Retning til controller
const uint8_t PIN_EN          = 8;  // Enable
const uint8_t PIN_PWM         = 9;  // PWM output til SV

const int RC_MIN_US_DEFAULT = 1000;
const int RC_MAX_US_DEFAULT = 2000;
const int RC_MID_US = 1500;
const int RC_TIMEOUT_US = 25000;
const int DIR_HYST_US = 120;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  MIN_RUN_PCT = 0;
const uint8_t  PUNCH_THRESH_PCT = 95;

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = false; // 🧭 ændret logik: FR LOW = CCW, FR HIGH = CW
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

// ------------ HELPERS ------------
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  // Nu er FR HIGH = CW, LOW = CCW
  digitalWrite(PIN_FR, cw ? HIGH : LOW);
}

static inline uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

// Soft ramp (smooth acceleration)
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

// Safe direction change
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

// ------------ INTERRUPTS ------------
ISR(INT0_vect) { // Throttle via D2
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

// Direction via pin-change interrupt (D4 = PCINT20)
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

// ------------ SETUP ------------
void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_DIR, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);

  // --- Enable interrupts ---
  // INT0 for throttle (D2)
  EIMSK |= (1 << INT0);    // enable external interrupt 0
  EICRA |= (1 << ISC00);   // trigger on CHANGE
  // PCINT for direction (D4)
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  setEnable(true);
  setDirectionCW(false);
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (Input Capture via INT0 + PCINT2). D2=CH3 throttle, D4=CH4 dir, D9 PWM."));
}

// ------------ LOOP ------------
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
    softTo(0);
    setEnable(false);
    runState = STOPPED;
    return;
  }

  // --- Direction ---
  static uint32_t lastWidthDir = 1500;
  noInterrupts();
  if (newPulseDir) { lastWidthDir = pulseWidthDir; newPulseDir = false; }
  interrupts();

  bool wantCW;
  if (lastWidthDir > (RC_MID_US + DIR_HYST_US)) wantCW = true;   // CW
  else if (lastWidthDir < (RC_MID_US - DIR_HYST_US)) wantCW = false; // CCW
  else wantCW = dirCW;

  if (wantCW != dirCW && currentPct < 10) safeChangeDirection(wantCW);

  // --- PWM update ---
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
    Serial.print(F("  DIR(us)=")); Serial.print(lastWidthDir);
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.print(F("% dir=")); Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}