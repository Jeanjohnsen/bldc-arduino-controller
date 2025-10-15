#include <Arduino.h>

/*
  RC → Arduino → BLDC controller (PlatformIO / Arduino UNO)
  - CH3 (throttle) på D2
  - CH4 (direction) på D4
  - EN (enable) på D8
  - FR (direction out) på D7
  - SV (PWM out) på D9
  - Soft-start/stop + sikker retningsskift
  - Telemetri: Serial 115200, "No line ending"
*/

// ====== PINMAP ======
const int PIN_RC_THROTTLE = 2;  // CH3 signal fra receiver
const int PIN_RC_DIR      = 4;  // CH4 signal fra receiver
const int PIN_EN          = 8;  // Enable til BLDC
const int PIN_FR          = 7;  // Direction til BLDC
const int PIN_PWM         = 9;  // PWM til SV på BLDC

// ====== RC TUNING ======
const int RC_MIN_US  = 1000;    // typisk 1000 µs
const int RC_MAX_US  = 2000;    // typisk 2000 µs
const int RC_MID_US  = 1500;    // midtpunkt til retning
const int RC_TIMEOUT = 25000;   // 25 ms timeout for pulseIn

// Hysterese for retningsvalg (for at undgå jitter)
const int DIR_HYST_US = 50;     // ±50 µs omkring midten

// ====== SOFT START/STOP ======
const unsigned long RAMP_MS       = 800;   // tid for 0→target (og omvendt)
const unsigned long UPDATE_MS     = 15;    // PWM opdateringsperiode
const bool USE_FAST_PWM_31K       = true;  // 31 kHz PWM på D9 (stille drift)

// ====== FAILSAFE ======
const unsigned long FAILSAFE_MS   = 200;   // hvis ingen gyldige pulser i 200ms → stop
const int MIN_RUN_PCT             = 0;     // evt. deadzone (fx 5%) hvis ønsket

// ====== STATE ======
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

uint8_t targetPct  = 0;    // ønsket hastighed i %
uint8_t currentPct = 0;    // aktuel hastighed i %
bool dirCW = true;         // FR = LOW for CW (tilpas hvis omvendt på din controller)

unsigned long lastUpdate   = 0;
unsigned long lastPulseTs  = 0;

void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

void setDirectionCW(bool cw) {
  dirCW = cw;
  // På mange controllere: FR LOW = CW, FR HIGH = CCW (skift hvis modsat)
  digitalWrite(PIN_FR, cw ? LOW : HIGH);
}

uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

void softTo(uint8_t toPct) {
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
    delay(UPDATE_MS);
  }
}

void startMotorTo(uint8_t pct) {
  setEnable(true);
  softTo(pct);
  runState = (pct > 0) ? RUNNING : STOPPED;
}

void stopMotorSoft() {
  softTo(0);
  setEnable(false);
  runState = STOPPED;
}

void safeChangeDirection(bool newCW) {
  if (newCW == dirCW) return; // ingen ændring
  // sikker vend: stop → vend FR → start til target
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

// Simpel glatning af throttle (1-pole lowpass)
int smooth(int prev, int now, float alpha = 0.25f) {
  return (int)(prev * (1.0f - alpha) + now * alpha);
}

// Læs RC puls (µs), returnerer -1 ved timeout
int readRcUs(int pin) {
  int us = pulseIn(pin, HIGH, RC_TIMEOUT);
  if (us <= 0) return -1;
  if (us < RC_MIN_US) us = RC_MIN_US;
  if (us > RC_MAX_US) us = RC_MAX_US;
  return us;
}

void setupPwm31kHzOnD9() {
  // Timer1 fast PWM ~31 kHz på D9/D10 (påvirker ikke millis()/delay())
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12)  | _BV(CS10);
}

void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_DIR, INPUT);

  if (USE_FAST_PWM_31K) setupPwm31kHzOnD9();

  Serial.begin(115200);
  delay(200);

  // Default: enable og CW
  setEnable(true);
  setDirectionCW(true);
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC klar. Throttle=CH3(D2), Direction=CH4(D4)."));
}

void loop() {
  unsigned long now = millis();

  // --- Læs RC throttle ---
  static int smoothedUs = RC_MIN_US;
  int usTh = readRcUs(PIN_RC_THROTTLE);
  if (usTh > 0) {
    smoothedUs = smooth(smoothedUs, usTh);
    lastPulseTs = now;

    int pct = map(smoothedUs, RC_MIN_US, RC_MAX_US, 0, 100);
    if (pct < MIN_RUN_PCT) pct = 0;
    if (pct > 100) pct = 100;

    targetPct = (uint8_t)pct;
  }

  // --- Failsafe: hvis ingen gyldige pulser i FAILSAFE_MS -> soft stop ---
  if (now - lastPulseTs > FAILSAFE_MS) {
    if (runState != STOPPED || currentPct != 0) {
      softTo(0);
      setEnable(false);
      runState = STOPPED;
    }
  } else {
    // --- Læs RC retning ---
    int usDir = readRcUs(PIN_RC_DIR);
    if (usDir > 0) {
      bool wantCW;
      if      (usDir > (RC_MID_US + DIR_HYST_US)) wantCW = false; // højre/CCW
      else if (usDir < (RC_MID_US - DIR_HYST_US)) wantCW = true;  // venstre/CW
      else wantCW = dirCW; // indenfor hysterese: behold retning

      if (wantCW != dirCW) {
        safeChangeDirection(wantCW);
      }
    }

    // --- Kør mod target med blød regulering ---
    if (now - lastUpdate >= UPDATE_MS) {
      lastUpdate = now;
      if (currentPct < targetPct) {
        uint8_t step = currentPct + 1;
        applyPWM(step);
        setEnable(true);
        runState = RUNNING;
      } else if (currentPct > targetPct) {
        uint8_t step = currentPct - 1;
        applyPWM(step);
        if (step == 0) { setEnable(false); runState = STOPPED; }
      }
    }
  }

  // --- Telemetri hvert ~100ms ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.print(F("THR(us)=")); Serial.print(smoothedUs);
    Serial.print(F("  Target=")); Serial.print(targetPct); Serial.print('%');
    Serial.print(F("  Current=")); Serial.print(currentPct); Serial.print('%');
    Serial.print(F("  Dir=")); Serial.println(dirCW ? F("CW") : F("CCW"));
  }
}