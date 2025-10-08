/*
  BLDC "Pro" Controller – UNO
  Hardware:
    D9  -> SV (PWM)
    D8  -> EN (enable)
    D7  -> FR (direction)
    GND -> GND

  Features:
    - Soft-start/stop (ramper)
    - Default: start i 50% duty og kør
    - Live styring via Serial (valgfrit): 
        s=start, x=stop, d=direction toggle, 
        +/- = +/-5% speed, 0..9 = 0..90%, 
        r=EN reset, ?=status
    - LED13 lyser proportionalt med duty

  NOTE: PWM på D9 ~490 Hz (default). 
        Aktiver FAST_PWM_31K for ~31 kHz (stille drift).
*/

// ====== KONFIG ======
const int PIN_PWM = 9;
const int PIN_EN  = 8;
const int PIN_FR  = 7;

const uint8_t SPEED_DEFAULT_PCT = 50;   // starter her
const uint16_t RAMP_MS = 1200;          // rampetid for start/stop (ms)
const bool START_DIRECTION_CW = true;   // true=FR LOW, false=FR HIGH

// Sæt til true for ~31 kHz PWM på D9/D10 (Timer1)
#define FAST_PWM_31K true

// ====== INTERN STATE ======
enum State { STOPPED, RUNNING };
State state = STOPPED;

uint8_t targetPct = SPEED_DEFAULT_PCT;   // ønsket fart i %
uint8_t currentPct = 0;                  // nuværende fart i %
unsigned long lastStep = 0;
const unsigned long STEP_PERIOD_MS = 15; // hvor ofte vi ændrer duty i ramp

// ====== HJÆLP ======
uint8_t pctToPwm(uint8_t pct) {
  if (pct > 100) pct = 100;
  // map 0..100% -> 0..255
  return (uint8_t)((pct * 255UL) / 100UL);
}

void applyPwm(uint8_t pct) {
  currentPct = pct;
  analogWrite(PIN_PWM, pctToPwm(pct));
  // LED13 som “bargraph” (bare en visuel indikator)
  uint8_t led = (uint8_t)((pct * 255UL) / 100UL);
  analogWrite(LED_BUILTIN, led); // UNO’s LED13 er PWM på Timer0 – ok
}

void setEnable(bool on) {
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

void setDirection(bool cw) {
  // CW = FR LOW (efter din controller-label). Skift hvis modsat.
  digitalWrite(PIN_FR, cw ? LOW : HIGH);
}

void softStart(uint8_t toPct) {
  unsigned long start = millis();
  uint8_t from = currentPct;
  if (from > toPct) { // hvis vi står højere end mål, kør ned først
    softStop(); 
  }
  while (currentPct < toPct) {
    unsigned long now = millis();
    if (now - lastStep >= STEP_PERIOD_MS) {
      lastStep = now;
      // lineær ramp:
      float t = (float)(now - start) / (float)RAMP_MS;
      if (t > 1.0f) t = 1.0f;
      uint8_t pct = from + (uint8_t)((toPct - from) * t);
      applyPwm(pct);
    }
  }
  applyPwm(toPct);
}

void softStop() {
  unsigned long start = millis();
  uint8_t from = currentPct;
  while (currentPct > 0) {
    unsigned long now = millis();
    if (now - lastStep >= STEP_PERIOD_MS) {
      lastStep = now;
      float t = (float)(now - start) / (float)RAMP_MS;
      if (t > 1.0f) t = 1.0f;
      uint8_t pct = from - (uint8_t)(from * t);
      applyPwm(pct);
    }
  }
  applyPwm(0);
}

void startMotor() {
  setEnable(true);
  softStart(targetPct);
  state = RUNNING;
}

void stopMotor() {
  softStop();
  setEnable(false);
  state = STOPPED;
}

void toggleDirection() {
  // sikker vend: stop -> flip -> start
  bool wasRunning = (state == RUNNING);
  stopMotor();
  delay(150); // kort settling
  bool nowCW = (digitalRead(PIN_FR) == LOW);
  setDirection(!nowCW);
  delay(150);
  if (wasRunning) startMotor();
}

void enResetPulse() {
  // Bruges hvis controller kan cleare fault via EN-puls
  stopMotor();
  setEnable(false);
  delay(3000);
  setEnable(true);
  // start ikke automatisk; lad bruger starte igen hvis ønsket
}

void printStatus() {
  Serial.print(F("State=")); Serial.print(state == RUNNING ? F("RUNNING") : F("STOPPED"));
  Serial.print(F("  Dir=")); Serial.print(digitalRead(PIN_FR) == LOW ? F("CW") : F("CCW"));
  Serial.print(F("  Target=")); Serial.print(targetPct); Serial.print('%');
  Serial.print(F("  Current=")); Serial.print(currentPct); Serial.println('%');
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 's': // start
      if (state == STOPPED) startMotor();
      break;
    case 'x': // stop
      if (state == RUNNING) stopMotor();
      break;
    case 'd': // toggle direction
      toggleDirection();
      break;
    case '+':
      if (targetPct < 100) targetPct = (uint8_t)min(100, targetPct + 5);
      if (state == RUNNING) softStart(targetPct);
      break;
    case '-':
      if (targetPct > 0) targetPct = (uint8_t)max(0, targetPct - 5);
      if (state == RUNNING) softStart(targetPct);
      break;
    case 'r': // EN reset pulse
      enResetPulse();
      break;
    case '?':
      printStatus();
      break;
    default:
      if (c >= '0' && c <= '9') { // 0..90%
        targetPct = (uint8_t)((c - '0') * 10);
        if (state == RUNNING) softStart(targetPct);
      }
      break;
  }
}

// ====== SETUP/LOOP ======
void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

#if FAST_PWM_31K
  // Sæt Timer1 til ~31.37 kHz fast PWM på D9/D10
  // (påvirker IKKE millis/delay som bruger Timer0)
  // WGM1: Fast PWM 10-bit + prescaler 1
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
#endif

  // Default retning
  setDirection(START_DIRECTION_CW);

  // Serial er valgfri (styring via PC). Hvis ikke tilsluttet, kører den bare selv.
  Serial.begin(115200);
  delay(300);

  // Start pænt op til default 50%
  setEnable(true);
  applyPwm(0);
  startMotor();  // soft-start til targetPct
  printStatus();
}

void loop() {
  handleSerial(); // valgfrit – giver “fed” live-styring, hvis du åbner Serial Monitor
  // Kører ellers bare i valgt hastighed/retning
}
