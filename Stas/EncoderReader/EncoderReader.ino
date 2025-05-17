#include <Arduino.h>
#include <vector>
#include <avr/interrupt.h>

static const uint8_t ENC0_A_PIN = 2;
static const uint8_t ENC0_B_PIN = 4;
static const uint8_t ENC1_A_PIN = 3;
static const uint8_t ENC1_B_PIN = 5;
static const uint8_t ENC2_A_PIN = 6;
static const uint8_t ENC2_B_PIN = 7;

int encoderCounts[3] = {0, 0, 0};
uint8_t enc2LastA;

void initEncoders() {
  pinMode(ENC0_A_PIN, INPUT_PULLUP);
  pinMode(ENC0_B_PIN, INPUT_PULLUP);
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);

  enc2LastA = digitalRead(ENC2_A_PIN);

  attachInterrupt(digitalPinToInterrupt(ENC0_A_PIN), [](){
    bool a = digitalRead(ENC0_A_PIN);
    bool b = digitalRead(ENC0_B_PIN);

    if (a == b) encoderCounts[0]++;  
    else          encoderCounts[0]--;
  }, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), [](){
    bool a = digitalRead(ENC1_A_PIN);
    bool b = digitalRead(ENC1_B_PIN);
    if (a == b) encoderCounts[1]++;
    else          encoderCounts[1]--;
  }, CHANGE);

  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << (PCINT22 - 16))
         |  (1 << (PCINT23 - 16));
}

ISR(PCINT2_vect) {
  bool a = digitalRead(ENC2_A_PIN);
  if (a != enc2LastA) {
    bool b = digitalRead(ENC2_B_PIN);
    if (a == b) encoderCounts[2]++;
    else          encoderCounts[2]--;
    enc2LastA = a;
  }
}
std::vector<int> readEncoders() {
  std::vector<int> counts(3);
  noInterrupts();
    counts[0] = encoderCounts[0];
    counts[1] = encoderCounts[1];
    counts[2] = encoderCounts[2];
  interrupts();
  return counts;
}
