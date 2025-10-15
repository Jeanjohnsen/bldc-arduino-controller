#include <Servo.h>

const int RC_PIN = 2;    // RC input fra receiver (CH3)
const int PWM_PIN = 9;   // Udgang til BLDC-controller (SV)
const int EN_PIN  = 8;   // Enable pin
const int FR_PIN  = 7;   // Direction pin

void setup() {
  pinMode(RC_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(FR_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);  // Aktivér controller
  digitalWrite(FR_PIN, LOW);   // CW-retning
}

void loop() {
  // Læs RC puls (typisk 1000–2000 µs)
  int pulse = pulseIn(RC_PIN, HIGH, 25000); // timeout 25 ms

  if (pulse > 1000 && pulse < 2000) {
    // Map puls 1000–2000 µs til PWM 0–255
    int pwmValue = map(pulse, 1000, 2000, 0, 255);
    analogWrite(PWM_PIN, pwmValue);
  }
}