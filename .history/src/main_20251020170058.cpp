#include <Arduino.h>

/*
  RC → Arduino → BLDC controller (optimeret for tophastighed)
  - CH3 (throttle) på D2
  - CH4 (direction) på D4
  - EN på D8
  - FR på D7
  - SV (PWM) på D9 (≈490 Hz, bedre “pull” i mange controllere)

  Hvis du vil teste ≈976 Hz: flyt SV-ledningen til D5 og sæt USE_PWM_D5 = true
*/

// ------------ CONFIG ------------
const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal fra receiver
const uint8_t PIN_RC_DIR      = 4;  // CH4 signal fra receiver
const uint8_t PIN_EN          = 8;  // Enable til BLDC
const uint8_t PIN_FR          = 7;  // Direction til BLDC

// PWM output: vælg D9 (≈490 Hz) eller D5 (≈976 Hz)
#define USE_PWM_D5 false
#if USE_PWM_D5
  const uint8_t PIN_PWM = 5;   // ~976 Hz (Timer0/Timer2 effekt)
#else
  const uint8_t PIN_PWM = 9;   // ~490 Hz (Timer1 default) – anbefalet først
#endif

// RC timing (auto-learner korrigerer løbende)
const int RC_MIN_US_DEFAULT  = 1000;
const int RC_MAX_US_DEFAULT  = 2000;
const int RC_MID_US          = 1500;
const int RC_TIMEOUT_US      = 25000;  // pulseIn timeout

// Hysterese for retningsvalg (mod jitter)
const int DIR_HYST_US = 120;

// Soft-ramp
const uint16_t RAMP_MS   = 600;   // kortere = mere bid
const uint16_t STEP_MS   = 10;    // opdateringsperiode

// Failsafe
const uint16_t FAILSAFE_MS = 200;

// Deadzone (for små udsving omkring minimum)
const uint8_t MIN_RUN_PCT = 0;    // sæt til 5 hvis du vil have lidt dødzone

// Punch-through tærskel (giver 100% duty på høj throttle)
const uint8_t PUNCH_THRESH_PCT = 95;

// ------------ STATE ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

bool dirCW = true;                // FR LOW = CW (byt logik i setDirectionCW, hvis din er modsat)
uint8_t targetPct  = 0;
uint8_t currentPct = 0;

unsigned long tLastStep  = 0;
unsigned long tLastPulse = 0;

// Auto-learn endpoints
int learnedMin = 2000;   // starter højt, falder nedad
int learnedMax = 1000;   // starter lavt, stiger opad

// ------------ HJÆLP ------------
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  // På mange controllere = FR LOW er CW; byt LOW/HIGH hvis omvendt
  digitalWrite(PIN_FR, cw ? LOW : HIGH);
}

static inline uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

static inline int readRcUs(uint8_t pin) {
  int us = pulseIn(pin, HIGH, RC_TIMEOUT_US);
  if (us <= 0) return -1;
  // clamp til et rimeligt område (for støj)
  if (us < 800)  us = 800;
  if (us > 2200) us = 2200;
  return us;
}

// Lidt glatning af throttle
static inline int smoothUs(int prev, int now, float alpha = 0.3f) {
  return (int)(prev * (1.0f - alpha) + now * alpha);
}

// Ramp fra currentPct mod toPct
static void softTo(uint8_t toPct) {
  uint8_t from = currentPct;
  if (from == toPct) return;

  unsigned long start = millis();
  while (true) {
    unsigned long now = millis();
    float t = (float)(now - start) / (float)RAMP_MS;
    if (t >= 1.0f) {
      applyPWM(toPct);
      break;
    }
    uint8_t step = from + (uint8_t)((toPct - from) * t);
    applyPWM(step);
    delay(STEP_MS);
  }
}

static void safeChangeDirection(bool newCW) {
  if (newCW == dirCW) return;
  // sikker vend: ned til 0 → flip FR → op igen
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

// ------------ SETUP ------------
void setup() {
  pinMode(PIN_EN,  OUTPUT);
  pinMode(PIN_FR,  OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_DIR, INPUT_PULLUP);

  // Standard 490 Hz på D9 / 976 Hz på D5 bruges som de er – ingen timers tweak
  Serial.begin(115200);
  delay(150);

  setEnable(true);
  setDirectionCW(true);
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (optimeret). D2=CH3 throttle, D4=CH4 dir, D9 PWM."));
}


// Gør retningssignal robust mod jitter
static bool stableDirection(bool lastCW, int pin) {
  const int samples = 6;
  int highCount = 0;
  for (int i = 0; i < samples; i++) {
    int us = pulseIn(pin, HIGH, 25000);
    if (us > (RC_MID_US + DIR_HYST_US)) highCount++;
    delayMicroseconds(500);
  }
  // Retningen ændres kun hvis >75% af målingerne peger i samme retning
  if (highCount >= 5) return false;  // CCW
  if (highCount <= 1) return true;   // CW
  return lastCW;                     // ellers behold retning
}

// ------------ LOOP ------------
void loop() {
  unsigned long now = millis();

  // --- THROTTLE (CH3) ---
  static int smoothedUs = RC_MIN_US_DEFAULT;
  int usTh = readRcUs(PIN_RC_THROTTLE);
  if (usTh > 0) {
    // auto-learn endpoints
    if (usTh < learnedMin) learnedMin = usTh;
    if (usTh > learnedMax) learnedMax = usTh;

    // sørg for fornuftige grænser (så ikke et glitch ødelægger)
    int spanMin = min(learnedMin, 1200);
    int spanMax = max(learnedMax, 1800);
    spanMax = max(spanMax, spanMin + 200);

    smoothedUs = smoothUs(smoothedUs, usTh);

    int pct = map(smoothedUs, spanMin, spanMax, 0, 100);
    pct = constrain(pct, 0, 100);
    if (pct < MIN_RUN_PCT) pct = 0;

    targetPct = (uint8_t)pct;
    tLastPulse = now;
  }

  // --- FAILSAFE ---
  if (now - tLastPulse > FAILSAFE_MS) {
    if (runState != STOPPED || currentPct != 0) {
      softTo(0);
      setEnable(false);
      runState = STOPPED;
    }
  } else {
    // --- RETNING (CH4) ---
    bool wantCW = !digitalRead(PIN_RC_DIR);
    if (wantCW != dirCW && currentPct < 10) {
      setDirectionCW(wantCW);
      delay(50);
  }

    // --- Kør mod target (hurtig respons) ---
    if (now - tLastStep >= STEP_MS) {
      tLastStep = now;

      // punch-through: sikre 100% når throttle ~fuld
      if (targetPct >= PUNCH_THRESH_PCT) {
        applyPWM(100);
        setEnable(true);
        runState = RUNNING;
      } else {
        // normal ramp step (hurtigere end tidligere)
        if (currentPct < targetPct) {
          applyPWM(currentPct + 2);   // større step op
          setEnable(true);
          runState = RUNNING;
        } else if (currentPct > targetPct) {
          uint8_t next = (currentPct > 2) ? currentPct - 2 : 0;
          applyPWM(next);
          if (next == 0) { setEnable(false); runState = STOPPED; }
        }
      }
    }
  }

  // --- Telemetri (hvert ~120 ms) ---
  static unsigned long tLastPrint = 0;
  if (now - tLastPrint >= 120) {
    tLastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(smoothedUs);
    Serial.print(F(" span=["));  Serial.print(learnedMin);
    Serial.print(',');           Serial.print(learnedMax);
    Serial.print(F("]  tgt="));  Serial.print(targetPct); Serial.print('%');
    Serial.print(F(" cur="));    Serial.print(currentPct); Serial.print('%');
    Serial.print(F(" dir="));    Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}