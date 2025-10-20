#include <Arduino.h>

/*
  RC → Arduino → BLDC controller (CH9 = direction switch)
  -------------------------------------------------------
  CH3 → D2 (INT0)  : Throttle (PWM-signal)
  CH9 → D3 (INT1)  : Direction (HIGH/LOW switch)
  D7  → FR          : Direction output til BLDC-controller
  D8  → EN          : Enable-signal
  D9  → SV          : PWM hastighed
*/

// ------------ CONFIG ------------
const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal (INT0)
const uint8_t PIN_RC_DIR      = 3;  // CH9 signal (INT1)
const uint8_t PIN_FR          = 7;  // FR → retning
const uint8_t PIN_EN          = 8;  // Enable
const uint8_t PIN_PWM         = 9;  // PWM output → SV

// --- Parametre ---
const uint16_t RAMP_MS        = 600;
const uint16_t STEP_MS        = 10;
const uint16_t FAILSAFE_MS    = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = false; // FR LOW = CCW, FR HIGH = CW
uint8_t targetPct  = 0;
uint8_t currentPct = 0;
unsigned long tLastStep  = 0;
unsigned long tLastPulse = 0;

// --- RC målinger ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool newPulseTh = false;

volatile bool dirSwitchState = false;   // CH9 HIGH/LOW
volatile bool newDirSignal   = false;   // registrerer ændring

// ------------ HJÆLP ------------
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  // FR HIGH = CW, FR LOW = CCW
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
  delay(100);
  setDirectionCW(newCW);
  delay(100);
  if (targetPct > 0) {
    setEnable(true);
    softTo(targetPct);
    runState = RUNNING;
  }
}

// ------------ INTERRUPTS ------------
ISR(INT0_vect) { // Throttle (CH3)
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

// Retning (CH9) → HIGH/LOW
ISR(INT1_vect) {
  dirSwitchState = digitalRead(PIN_RC_DIR);
  newDirSignal = true;
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
  // INT0 (CH3 throttle)
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00); // CHANGE trigger
  // INT1 (CH9 direction)
  EIMSK |= (1 << INT1);
  EICRA |= (1 << ISC10); // CHANGE trigger

  setEnable(true);
  setDirectionCW(false); // starter CCW
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (INT0=CH3 throttle, INT1=CH9 direction). D9=PWM."));
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

  // --- Direction (CH9) ---
  noInterrupts();
  bool switchState = dirSwitchState;
  bool signalChanged = newDirSignal;
  newDirSignal = false;
  interrupts();

  if (signalChanged && currentPct < 10) {
    bool wantCW = switchState;   // HIGH = CW, LOW = CCW
    safeChangeDirection(wantCW);
  }

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
    Serial.print(F(" tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.print(F("% dir=")); Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}