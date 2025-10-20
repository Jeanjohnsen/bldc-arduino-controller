#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller (stabil retning, digital læsning)
  ----------------------------------------------------------
  - CH3 → D2 (INT0): Throttle (PWM, måles med interrupt)
  - CH9/CH4 → D4   : Direction (2-position, læses digitalt med filtrering)
  - D7  → FR       : Retning ud til BLDC (HIGH=CW, LOW=CCW)
  - D8  → EN       : Enable
  - D9  → SV       : PWM speed (≈490 Hz)
  ==========================================================
*/

// ------------ KONSTANTER ------------
const uint8_t PIN_RC_THROTTLE = 2;  // INT0
const uint8_t PIN_RC_DIR      = 4;  // digital retning (2-pos)
const uint8_t PIN_FR          = 7;
const uint8_t PIN_EN          = 8;
const uint8_t PIN_PWM         = 9;

const int RC_MID_US = 1500;
const uint16_t RAMP_MS = 600;
const uint16_t STEP_MS = 10;
const uint16_t FAILSAFE_MS = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;
const int DEAD_BAND_US = 25;        // ±25 µs omkring neutral

// Retningsfilter (digital)
const uint8_t DIR_SAMPLES       = 10;  // antal hurtige prøver pr. måling
const uint8_t DIR_HIGH_MAJ      = 8;   // ≥8/10 HIGH => CW
const uint8_t DIR_LOW_MAJ       = 2;   // ≤2/10 HIGH => CCW
const uint8_t DIR_STABLE_CYCLES = 3;   // kræv 3 konsekvente beslutninger før skift
const uint16_t DIR_SAMPLE_MS    = 40;  // retning evalueres ca. hver 40 ms

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

// FR HIGH = CW, FR LOW = CCW
bool dirCW = false; // start CCW
uint8_t targetPct = 0;
uint8_t currentPct = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// --- Throttle målinger (INT0) ---
volatile uint32_t pulseStartTh = 0;
volatile uint32_t pulseWidthTh = 1500;
volatile bool     newPulseTh   = false;

// ==================================================
// HJÆLPEFUNKTIONER
// ==================================================
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  digitalWrite(PIN_FR, cw ? HIGH : LOW); // HIGH = CW, LOW = CCW
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
  // Sikker vend: rull ned → flip FR → rull op igen
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
// INTERRUPT: THROTTLE (INT0 på D2)
// ==================================================
ISR(INT0_vect) {
  bool level = digitalRead(PIN_RC_THROTTLE);
  if (level) {
    pulseStartTh = micros();
  } else {
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
  pinMode(PIN_RC_THROTTLE, INPUT);       // ekstern RC-signal
  pinMode(PIN_RC_DIR, INPUT_PULLUP);     // 2-pos switch: brug pull-up for stabil HIGH når åben

  Serial.begin(115200);
  delay(200);

  // Enable INT0 (CHANGE) for throttle
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00); // CHANGE

  setEnable(true);
  setDirectionCW(false); // start CCW
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (stabil retning: digital filter). D2=CH3 throttle(INT0), D4=CH9/CH4 dir(digital), D9 PWM."));
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle (glat + deadband) ---
  static uint32_t lastWidthTh = 1500;
  noInterrupts();
  if (newPulseTh) {
    // 4-sample smoothing
    lastWidthTh = (lastWidthTh * 3 + pulseWidthTh) / 4;
    newPulseTh = false;
    tLastPulse = now;
  }
  interrupts();

  if (abs((int)lastWidthTh - RC_MID_US) < DEAD_BAND_US) {
    targetPct = 0;
  } else {
    int pct = map(lastWidthTh, 1000, 2000, 0, 100);
    targetPct = constrain(pct, 0, 100);
  }

  // --- Failsafe ---
  if (now - tLastPulse > FAILSAFE_MS) {
    softTo(0);
    setEnable(false);
    runState = STOPPED;
    // Vi læser stadig retning nedenfor, men motoren er disabled indtil throttle kommer igen
  }

  // --- Direction (REN DIGITAL, kraftig filtrering) ---
  static unsigned long tLastDirSample = 0;
  static uint8_t stableCWCount = 0;   // hvor mange konsekvente beslutninger på CW
  static uint8_t stableCCWCount = 0;  // … og CCW

  if (now - tLastDirSample >= DIR_SAMPLE_MS) {
    tLastDirSample = now;

    // Majority-vote: tag 10 hurtige samples
    uint8_t highCount = 0;
    for (uint8_t i = 0; i < DIR_SAMPLES; i++) {
      if (digitalRead(PIN_RC_DIR)) highCount++;
      delayMicroseconds(200);
    }

    // Oversæt til “ønsket retning” baseret på tydelig majoritet
    // HIGH-majoritet => CW, LOW-majoritet => CCW
    int decision = 0; // 0 = uafgjort, +1 = CW, -1 = CCW
    if (highCount >= DIR_HIGH_MAJ)      decision = +1;
    else if (highCount <= DIR_LOW_MAJ)  decision = -1;

    // Kræv N konsekvente beslutninger, før vi skifter (debounce på software-niveau)
    if (decision == +1) {
      stableCWCount++;
      stableCCWCount = 0;
    } else if (decision == -1) {
      stableCCWCount++;
      stableCWCount = 0;
    } else {
      // uafklaret → nulstil langsomt (så kortvarig støj ikke tæller)
      if (stableCWCount > 0)  stableCWCount--;
      if (stableCCWCount > 0) stableCCWCount--;
    }

    bool wantCW = dirCW; // default: behold
    if (stableCWCount >= DIR_STABLE_CYCLES) {
      wantCW = true;
      stableCWCount = DIR_STABLE_CYCLES;   // clamp
    } else if (stableCCWCount >= DIR_STABLE_CYCLES) {
      wantCW = false;
      stableCCWCount = DIR_STABLE_CYCLES;  // clamp
    }

    // Skift først når vi næsten står stille
    if (wantCW != dirCW && currentPct < 10) {
      safeChangeDirection(wantCW);
    }
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
    Serial.print(F("  tgt="));   Serial.print(targetPct);
    Serial.print(F("%  cur="));  Serial.print(currentPct);
    Serial.print(F("%  dir="));  Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}