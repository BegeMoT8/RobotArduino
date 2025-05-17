#include <Arduino.h>
#include <vector>

static const uint8_t TCRT_PINS[2] = {2, 3};

void initTCRT() {
  pinMode(TCRT_PINS[0], INPUT);
  pinMode(TCRT_PINS[1], INPUT);
}

std::vector<bool> readTCRT() {
  std::vector<bool> states(2);
  for (uint8_t i = 0; i < 2; ++i) {
    states[i] = (digitalRead(TCRT_PINS[i]) == HIGH);
  }
  return states;
}

