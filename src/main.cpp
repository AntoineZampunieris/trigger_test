#include <Arduino.h>

const uint8_t SIG_PIN = 2;   // your 3.3V signal
const uint8_t OUT_PIN = 8;   // pin to mirror the state

// Interrupt Service Routine (no IRAM_ATTR on UNO R4)
void onSigChange() {
  digitalWrite(OUT_PIN, digitalRead(SIG_PIN));
}

void setup() {
  pinMode(SIG_PIN, INPUT);   // or INPUT_PULLUP if needed
  pinMode(OUT_PIN, OUTPUT);

  // initialize output to current input state
  digitalWrite(OUT_PIN, digitalRead(SIG_PIN));

  // attach on both edges
  attachInterrupt(digitalPinToInterrupt(SIG_PIN), onSigChange, CHANGE);
}

void loop() {
  // nothing to do; ISR handles mirroring
}
