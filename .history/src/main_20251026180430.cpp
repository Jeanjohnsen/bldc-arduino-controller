#include <Arduino.h>

/*
  ==========================================================
  RC → Arduino → BLDC controller
  ----------------------------------------------------------
  Dette program læser PWM-signaler fra en RC-receiver og
  omsætter dem til styring af en BLDC-controller via
  retning (FR), hastighed (PWM) og enable-signal (EN).
  ==========================================================

  Kanalforbindelser:
  ------------------
  CH3 → D2 (INT0)      : Throttle (PWM-signal fra joystick)
  CH4 → D4 (PCINT20)   : Direction (PWM-signal fra switch)
  D7  → FR             : Retningsudgang til controller
  D8  → EN             : Aktivering af controller
  D9  → SV             : PWM-hastighedssignal (speed input)
*/

// ------------ KONSTANTER OG PINOPSÆTNING ------------
const uint8_t PIN_RC_THROTTLE = 2;  // CH3 signal (INT0)
const uint8_t PIN_RC_DIR      = 4;  // CH4 signal (PCINT20)
const uint8_t PIN_FR          = 7;  // FR-pin til retning (CW/CCW)
const uint8_t PIN_EN          = 8;  // Enable til controller
const uint8_t PIN_PWM         = 9;  // PWM-output til speed (SV)

const int RC_MIN_US_DEFAULT = 1000;   // Minimum RC-pulslængde (us)
const int RC_MAX_US_DEFAULT = 2000;   // Maksimum RC-pulslængde (us)
const int RC_MID_US = 1500;           // Midtpunkt (bruges til retningsvalg)
const int RC_TIMEOUT_US = 25000;      // Maks. ventetid for pulseIn()
const int DIR_HYST_US = 120;          // Hysterese for stabil retningsmåling
const uint16_t RAMP_MS = 600;         // Hvor hurtigt motoren accelererer/decelererer
const uint16_t STEP_MS = 10;          // Opdateringsinterval for ramping
const uint16_t FAILSAFE_MS = 200;     // Stop motor efter 200 ms uden signal
const uint8_t  MIN_RUN_PCT = 0;       // Deadzone omkring 0 %
const uint8_t  PUNCH_THRESH_PCT = 95; // 100 % PWM ved fuld throttle

// ------------ DRIFTSTILSTAND ------------
enum RunState { STOPPED, RUNNING };
RunState runState = STOPPED;

// Retning og PWM-tilstand
bool dirCW = false; // 🧭 Logik: FR LOW = CCW, FR HIGH = CW
uint8_t targetPct = 0;   // Målrettet hastighed i %
uint8_t currentPct = 0;  // Aktuel hastighed i %
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

/**
 * @brief Aktiver eller deaktiver BLDC-controlleren
 */
static inline void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

/**
 * @brief Sæt motorretningen.
 *        FR HIGH = CW, FR LOW = CCW
 */
static inline void setDirectionCW(bool cw) {
  dirCW = cw;
  digitalWrite(PIN_FR, cw ? HIGH : LOW);
}

/**
 * @brief Konverter procent (0–100 %) til PWM (0–255)
 */
static inline uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint8_t)((pct * 255UL) / 100UL);
}

/**
 * @brief Påfør PWM-signal i procent
 */
static inline void applyPWM(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
}

/**
 * @brief Glidende ændring (soft ramp) mellem to PWM-værdier
 */
static void softTo(uint8_t toPct) {
  uint8_t from = currentPct;
  if (from == toPct) return; // ingen ændring nødvendig
  unsigned long start = millis();
  while (true) {
    float t = (float)(millis() - start) / (float)RAMP_MS;
    if (t >= 1.0f) { applyPWM(toPct); break; }
    uint8_t step = from + (uint8_t)((toPct - from) * t);
    applyPWM(step);
    delay(STEP_MS);
  }
}

/**
 * @brief Sikker retningsændring
 *        1) Bremser ned til 0
 *        2) Skifter FR-pin
 *        3) Starter op igen
 */
static void safeChangeDirection(bool newCW) {
  if (newCW == dirCW) return;  // ingen ændring nødvendig
  softTo(0);                   // stop motor
  delay(80);
  setDirectionCW(newCW);       // skift retning
  delay(80);
  if (targetPct > 0) {
    setEnable(true);
    softTo(targetPct);         // accelerér igen
    runState = RUNNING;
  }
}

// ==================================================
// INTERRUPTS
// ==================================================

/**
 * @brief INT0_vect – aflæs throttle PWM-signal (CH3)
 *        Kaldes ved hver op/ned-flanke på D2.
 */
ISR(INT0_vect) {
  bool level = digitalRead(PIN_RC_THROTTLE);
  if (level)
    pulseStartTh = micros();  // registrér starttid
  else {
    uint32_t width = micros() - pulseStartTh;  // beregn pulsbredden
    if (width >= 800 && width <= 2200) {
      pulseWidthTh = width;
      newPulseTh = true;
    }
  }
}

/**
 * @brief PCINT2_vect – aflæs retningssignal (CH4)
 *        Kaldes ved ændring på D4.
 */
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

// ==================================================
// SETUP – kører én gang ved opstart
// ==================================================
void setup() {
  // Pin modes
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_DIR, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);

  // --- Aktiver hardware-interrupts ---
  // INT0 for throttle (D2)
  EIMSK |= (1 << INT0);    // enable ekstern interrupt 0
  EICRA |= (1 << ISC00);   // trigger på CHANGE (både rising/falling)
  // PCINT for direction (D4)
  PCICR |= (1 << PCIE2);   // enable pin change for port D
  PCMSK2 |= (1 << PCINT20);// aktiver PCINT20 (D4)

  // --- Init motorstatus ---
  setEnable(true);
  setDirectionCW(false);   // start i CCW
  applyPWM(0);
  runState = STOPPED;

  Serial.println(F("RC→BLDC (INT0 + PCINT2). D2=CH3 throttle, D4=CH4 dir, D9 PWM."));
}

// ==================================================
// LOOP – hovedprogram (kører kontinuerligt)
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

  // Konverter RC-puls (1000–2000 µs) til procent (0–100)
  int pct = map(lastWidthTh, 1000, 2000, 0, 100);
  pct = constrain(pct, 0, 100);
  targetPct = pct;

  // --- Failsafe ---
  // Hvis der ikke er modtaget signal i >200 ms → stop motor
  if (now - tLastPulse > FAILSAFE_MS) {
    softTo(0);
    setEnable(false);
    runState = STOPPED;
    return;
  }

  // --- Retningsbehandling ---
  static uint32_t lastWidthDir = 1500;
  noInterrupts();
  if (newPulseDir) { lastWidthDir = pulseWidthDir; newPulseDir = false; }
  interrupts();

  // Fortolk retningssignal baseret på midtpunkt + hysterese
  bool wantCW;
  if (lastWidthDir > (RC_MID_US + DIR_HYST_US))      wantCW = true;   // CW
  else if (lastWidthDir < (RC_MID_US - DIR_HYST_US)) wantCW = false;  // CCW
  else                                                wantCW = dirCW;  // ingen ændring

  // Skift kun retning hvis motoren er næsten stoppet
  if (wantCW != dirCW && currentPct < 10) safeChangeDirection(wantCW);

  // --- PWM-opdatering (soft ramping) ---
  if (now - tLastStep >= STEP_MS) {
    tLastStep = now;
    if (targetPct >= PUNCH_THRESH_PCT) {
      applyPWM(100);               // punch-through: fuld power
      setEnable(true);
      runState = RUNNING;
    } else if (currentPct < targetPct) {
      applyPWM(currentPct + 2);    // gradvis acceleration
      setEnable(true);
      runState = RUNNING;
    } else if (currentPct > targetPct) {
      uint8_t next = (currentPct > 2) ? currentPct - 2 : 0;
      applyPWM(next);              // gradvis deceleration
      if (next == 0) {
        setEnable(false);
        runState = STOPPED;
      }
    }
  }

  // --- Seriel telemetri ---
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