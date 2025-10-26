#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller  (CW-only + anti-stall)
  ----------------------------------------------------------
  - CH3 → D2 (INT0): Throttle (PWM)
  - D7  → FR (fastsat LOW = CW)
  - D8  → EN
  - D9  → SV (PWM-speed)
  ==========================================================

  NOTE:
  På denne controller betyder:
  FR = LOW  → CW (frem)
  FR = HIGH → CCW (tilbage)
*/

// ------------ KONSTANTER ------------
const uint8_t PIN_RC_THROTTLE = 2;
const uint8_t PIN_FR          = 7;
const uint8_t PIN_EN          = 8;
const uint8_t PIN_PWM         = 9;

const uint16_t RAMP_MS        = 600;
const uint16_t STEP_MS        = 10;
const uint16_t FAILSAFE_MS    = 200;
const uint8_t  PUNCH_THRESH_PCT = 95;

// Anti-stall parametre
const uint8_t  MIN_RUN_PCT    = 18;    // min. duty for stabil rotation
const uint8_t  START_KICK_PCT = 100;   // kort “spark” ved start
const uint16_t START_KICK_MS  = 150;   // længde af spark
const uint16_t CRAWL_HOLD_MS  = 400;   // hold crawl lidt før 0

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

uint8_t targetPct   = 0;
uint8_t currentPct  = 0;
unsigned long tLastStep = 0;
unsigned long tLastPulse = 0;

// Anti-stall tidsmarkører
static bool wasRunning = false;
static unsigned long tKickEnd = 0;
static unsigned long tZeroSince = 0;

// --- RC målinger ---
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
  digitalWrite(PIN_FR, LOW);  // permanent CW (din controller tolker LOW som CW)
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
ISR(INT0_vect) { // Throttle
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

  // Aktiver throttle interrupt
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC00);   // trigger på CHANGE

  // Init
  setEnable(true);
  forceCW();
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (CW-only, FR=LOW for CW). D2=CH3 throttle, D9 PWM."));
}

// ==================================================
// LOOP
// ==================================================
void loop() {
  unsigned long now = millis();

  // --- Throttle input ---
  static uint32_t lastWidthTh = 1500;
  noInterrupts();
  if (newPulseTh) {
    lastWidthTh = pulseWidthTh;
    newPulseTh = false;
    tLastPulse = now;
  }
  interrupts();

  // Konverter RC-puls (1000–2000 µs) → procent (0–100)
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

  // --- Anti-stall styring ---
  setEnable(true);    // Hold controller aktiv
  forceCW();          // Retning fast CW

  // 1. START-KICK når vi går fra 0 → >0
  if (currentPct == 0 && targetPct > 0 && now > tKickEnd) {
    applyPWM(START_KICK_PCT);
    tKickEnd = now + START_KICK_MS;
    wasRunning = true;
    return;
  }

  // 2. Efter kick, hold mindst MIN_RUN_PCT
  if (now <= tKickEnd) {
    // stadig i kick-fase
  } else if (wasRunning && currentPct >= START_KICK_PCT) {
    uint8_t hold = max<uint8_t>(MIN_RUN_PCT, targetPct);
    applyPWM(hold);
  }

  // 3. Normal ramp + crawl-hold
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
            // setEnable(false);  // slå først fra i failsafe
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
    Serial.print(F("  tgt=")); Serial.print(targetPct);
    Serial.print(F("% cur=")); Serial.print(currentPct);
    Serial.println(F("% dir=FORCED_CW(FR=LOW)"));
  }
}